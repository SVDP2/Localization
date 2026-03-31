#!/usr/bin/env python3

from __future__ import annotations

import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, Imu
from tf2_ros import TransformBroadcaster

from aruco_imu_eskf_localization.pose_filter import PoseFilter


PACKAGE_NAME = 'aruco_imu_eskf_localization'


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _invert_camera_observation(rvec, tvec) -> tuple[np.ndarray, np.ndarray]:
    rotation_cam_from_board = Rotation.from_rotvec(np.asarray(rvec, dtype=float).reshape(3))
    rotation_board_from_cam = rotation_cam_from_board.inv()
    translation_cam_from_board = np.asarray(tvec, dtype=float).reshape(3)
    translation_board_from_cam = -rotation_board_from_cam.as_matrix() @ translation_cam_from_board
    return rotation_board_from_cam.as_rotvec(), translation_board_from_cam


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_detector_node')

        self.declare_parameter('marker_config_file', 'config/markers_board.yaml')
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        self.declare_parameter('camera_calibration_file', 'config/cam_intrinsic.yaml')
        self.declare_parameter('camera_frame_id', 'usb_cam')
        self.declare_parameter('board_frame', 'board')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('board_pose_topic', '/localization/aruco/board_pose')
        self.declare_parameter('debug_image_topic', '/localization/aruco/debug_image')
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_prefix', 'aruco_marker_')
        self.declare_parameter('use_board_pose', True)
        self.declare_parameter('min_markers_for_board', 1)
        self.declare_parameter('reset_timeout_sec', 1.0)
        self.declare_parameter('position_process_noise_std_mps2', 0.3)
        self.declare_parameter('measurement_position_std_xy_m', 0.02)
        self.declare_parameter('measurement_position_std_z_m', 0.05)
        self.declare_parameter('orientation_outlier_threshold_deg', 45.0)
        self.declare_parameter('orientation_measurement_alpha', 0.15)
        self.declare_parameter('output_orientation_std_rad', 0.1)

        marker_config_file = self.get_parameter('marker_config_file').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        camera_calibration_file = self.get_parameter('camera_calibration_file').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.board_frame = self.get_parameter('board_frame').value
        image_topic = self.get_parameter('image_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        board_pose_topic = self.get_parameter('board_pose_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value

        self.detection_threshold = float(self.get_parameter('detection_threshold').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.use_board_pose = bool(self.get_parameter('use_board_pose').value)
        self.min_markers_for_board = int(self.get_parameter('min_markers_for_board').value)
        self.reset_timeout_sec = float(self.get_parameter('reset_timeout_sec').value)
        self.output_orientation_std_rad = float(
            self.get_parameter('output_orientation_std_rad').value
        )

        self._position_measurement_covariance = np.diag(
            [
                float(self.get_parameter('measurement_position_std_xy_m').value) ** 2,
                float(self.get_parameter('measurement_position_std_xy_m').value) ** 2,
                float(self.get_parameter('measurement_position_std_z_m').value) ** 2,
            ]
        )
        self._pose_filter_kwargs = {
            'process_noise_std': float(
                self.get_parameter('position_process_noise_std_mps2').value
            ),
            'measurement_noise_std_xy': float(
                self.get_parameter('measurement_position_std_xy_m').value
            ),
            'measurement_noise_std_z': float(
                self.get_parameter('measurement_position_std_z_m').value
            ),
            'orientation_outlier_threshold_deg': float(
                self.get_parameter('orientation_outlier_threshold_deg').value
            ),
            'orientation_meas_alpha': float(
                self.get_parameter('orientation_measurement_alpha').value
            ),
            # The detector filter runs on solvePnP's camera<-board observation.
            'imu_angular_velocity_sign': -1.0,
            'imu_left_multiply': True,
        }

        self.bridge = CvBridge()

        self.marker_sizes, self.board_geometry = self._load_marker_config(marker_config_file)
        self.camera_matrix, self.dist_coeffs = self._load_camera_calibration(
            camera_calibration_file
        )

        aruco_dict_id = getattr(cv2.aruco, aruco_dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 100)

        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.board_pose_pub = self.create_publisher(PoseWithCovarianceStamped, board_pose_topic, 10)

        self.filters: dict[object, PoseFilter] = {}
        self.last_update_times: dict[object, float] = {}
        self.last_imu_time: float | None = None

        self.get_logger().info(f'aruco dict: {aruco_dict_name}')
        self.get_logger().info(f'image topic: {image_topic}')
        self.get_logger().info(f'imu topic: {imu_topic}')
        self.get_logger().info(f'board pose topic: {board_pose_topic}')
        self.get_logger().info(f'loaded {len(self.marker_sizes)} marker definitions')

    def _resolve_config_path(self, path: str) -> str:
        if os.path.isabs(path):
            return path

        package_share_dir = get_package_share_directory(PACKAGE_NAME)
        share_path = os.path.join(package_share_dir, path)
        if os.path.exists(share_path):
            return share_path

        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            path,
        )

    def _load_marker_config(self, config_file: str):
        try:
            config_path = self._resolve_config_path(config_file)
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)

            marker_sizes = {}
            for marker in config['markers']:
                marker_sizes[int(marker['id'])] = float(marker['size'])

            board_geometry = config.get('board_geometry')
            return marker_sizes, board_geometry
        except Exception as exc:
            self.get_logger().error(f'failed to load marker config: {exc}')
            return {}, None

    def _load_camera_calibration(self, calib_file: str):
        try:
            calib_path = self._resolve_config_path(calib_file)
            with open(calib_path, 'r', encoding='utf-8') as f:
                calib = yaml.safe_load(f)

            camera_matrix = np.array(calib['camera_matrix']['data'], dtype=np.float64)
            dist_coeffs = np.array(
                calib['distortion_coefficients']['data'],
                dtype=np.float64,
            )
            return camera_matrix, dist_coeffs
        except Exception as exc:
            self.get_logger().error(f'failed to load camera calibration: {exc}')
            return np.eye(3, dtype=np.float64), np.zeros(5, dtype=np.float64)

    def _make_pose_filter(self) -> PoseFilter:
        return PoseFilter(**self._pose_filter_kwargs)

    def _get_object_points(self, marker_size_mm: float) -> np.ndarray:
        half_size = (marker_size_mm / 1000.0) * 0.5
        return np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )

    def _estimate_pose(self, corners, marker_id: int):
        if marker_id not in self.marker_sizes:
            self.get_logger().warn(f'marker id {marker_id} not found in config')
            return None, None

        object_points = self._get_object_points(self.marker_sizes[marker_id])
        image_points = corners.reshape((4, 1, 2))

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except Exception as exc:
            self.get_logger().error(f'pose estimation error for marker {marker_id}: {exc}')
            return None, None

        if not success:
            self.get_logger().warn(f'solvePnP failed for marker {marker_id}')
            return None, None

        return rvec, tvec

    def _estimate_board_pose(self, corners, ids):
        if not self.board_geometry or 'marker_positions' not in self.board_geometry:
            return None, None, 0

        marker_positions = {
            int(marker_id): np.asarray(position, dtype=np.float32)
            for marker_id, position in self.board_geometry['marker_positions'].items()
        }

        object_points_list = []
        image_points_list = []
        valid_marker_count = 0

        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in marker_positions or marker_id not in self.marker_sizes:
                continue

            marker_size_m = self.marker_sizes[marker_id] / 1000.0
            half_size = marker_size_m * 0.5
            board_pos_m = marker_positions[marker_id] / 1000.0
            marker_corners = corners[i][0]

            marker_obj_points = np.array(
                [
                    [board_pos_m[0] - half_size, board_pos_m[1] + half_size, board_pos_m[2]],
                    [board_pos_m[0] + half_size, board_pos_m[1] + half_size, board_pos_m[2]],
                    [board_pos_m[0] + half_size, board_pos_m[1] - half_size, board_pos_m[2]],
                    [board_pos_m[0] - half_size, board_pos_m[1] - half_size, board_pos_m[2]],
                ],
                dtype=np.float32,
            )

            object_points_list.append(marker_obj_points)
            image_points_list.append(marker_corners.astype(np.float32))
            valid_marker_count += 1

        if valid_marker_count < self.min_markers_for_board:
            self.get_logger().warn(
                f'insufficient markers for board pose: {valid_marker_count}/{self.min_markers_for_board}',
                throttle_duration_sec=2.0,
            )
            return None, None, 0

        object_points = np.vstack(object_points_list)
        image_points = np.vstack(image_points_list)

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        except Exception as exc:
            self.get_logger().error(f'board pose estimation error: {exc}')
            return None, None, 0

        if not success:
            self.get_logger().warn('board pose solvePnP failed')
            return None, None, 0

        return rvec, tvec, valid_marker_count

    def imu_callback(self, msg: Imu) -> None:
        current_time_sec = _stamp_to_sec(msg.header.stamp)
        if self.last_imu_time is None:
            self.last_imu_time = current_time_sec
            return

        dt = current_time_sec - self.last_imu_time
        self.last_imu_time = current_time_sec
        if dt <= 0.0:
            return

        wx_imu = float(msg.angular_velocity.x)
        wy_imu = float(msg.angular_velocity.y)
        wz_imu = float(msg.angular_velocity.z)

        angular_velocity_cam = np.array(
            [
                -wy_imu,
                -wz_imu,
                wx_imu,
            ],
            dtype=float,
        )

        for pose_filter in self.filters.values():
            pose_filter.predict(dt, angular_velocity_cam)

    def _apply_filter(self, marker_id, rvec, tvec, stamp_sec: float):
        if marker_id not in self.filters:
            self.filters[marker_id] = self._make_pose_filter()
            self.last_update_times[marker_id] = stamp_sec
            self.filters[marker_id].update(tvec, rvec)
            return self.filters[marker_id].get_pose()

        dt = stamp_sec - self.last_update_times[marker_id]
        self.last_update_times[marker_id] = stamp_sec

        if dt > self.reset_timeout_sec:
            self.filters[marker_id] = self._make_pose_filter()
            self.filters[marker_id].update(tvec, rvec)
            return self.filters[marker_id].get_pose()

        self.filters[marker_id].update(tvec, rvec)
        return self.filters[marker_id].get_pose()

    def _publish_camera_tf(self, rvec, tvec, timestamp, child_frame_id: str) -> None:
        if self.tf_broadcaster is None:
            return

        try:
            rotation_cam_from_board, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3))
            translation_cam_from_board = np.asarray(tvec, dtype=float).reshape(3)
            rotation_board_from_cam = rotation_cam_from_board.T
            translation_board_from_cam = -rotation_board_from_cam @ translation_cam_from_board

            tf_msg = TransformStamped()
            tf_msg.header.stamp = timestamp
            tf_msg.header.frame_id = self.board_frame
            tf_msg.child_frame_id = child_frame_id
            tf_msg.transform.translation.x = float(translation_board_from_cam[0])
            tf_msg.transform.translation.y = float(translation_board_from_cam[1])
            tf_msg.transform.translation.z = float(translation_board_from_cam[2])

            quat = Rotation.from_matrix(rotation_board_from_cam).as_quat()
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf_msg)
        except Exception as exc:
            self.get_logger().error(f'error publishing camera tf ({child_frame_id}): {exc}')

    def _publish_marker_tf(self, marker_id: int, rvec, tvec, timestamp) -> None:
        if self.tf_broadcaster is None:
            return

        try:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = timestamp
            tf_msg.header.frame_id = self.camera_frame_id
            tf_msg.child_frame_id = f'{self.tf_prefix}{marker_id}'

            translation = np.asarray(tvec, dtype=float).reshape(3)
            tf_msg.transform.translation.x = float(translation[0])
            tf_msg.transform.translation.y = float(translation[1])
            tf_msg.transform.translation.z = float(translation[2])

            quat = Rotation.from_rotvec(np.asarray(rvec, dtype=float).reshape(3)).as_quat()
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf_msg)
        except Exception as exc:
            self.get_logger().error(f'error publishing marker tf ({marker_id}): {exc}')

    def _publish_board_pose(self, rvec, tvec, timestamp) -> None:
        board_rvec, board_tvec = _invert_camera_observation(rvec, tvec)
        quat = Rotation.from_rotvec(board_rvec).as_quat()

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.board_frame
        pose_msg.pose.pose.position.x = float(board_tvec[0])
        pose_msg.pose.pose.position.y = float(board_tvec[1])
        pose_msg.pose.pose.position.z = float(board_tvec[2])
        pose_msg.pose.pose.orientation.x = float(quat[0])
        pose_msg.pose.pose.orientation.y = float(quat[1])
        pose_msg.pose.pose.orientation.z = float(quat[2])
        pose_msg.pose.pose.orientation.w = float(quat[3])

        covariance = np.zeros((6, 6), dtype=float)
        if 'board' in self.filters and self.filters['board'].initialized:
            covariance[:3, :3] = self.filters['board'].kf.P[:3, :3]
        else:
            covariance[:3, :3] = self._position_measurement_covariance
        covariance[3:, 3:] = np.diag([self.output_orientation_std_rad**2] * 3)
        pose_msg.pose.covariance = covariance.flatten().tolist()

        self.board_pose_pub.publish(pose_msg)

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            debug_image = cv_image.copy()
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is None or len(ids) == 0:
                self._publish_debug_image(debug_image, msg)
                return

            stamp_sec = _stamp_to_sec(msg.header.stamp)

            if self.use_board_pose and self.board_geometry:
                board_rvec, board_tvec, num_markers_used = self._estimate_board_pose(corners, ids)
                if board_rvec is not None and board_tvec is not None:
                    self._publish_camera_tf(board_rvec, board_tvec, msg.header.stamp, self.camera_frame_id)

                    filtered_rvec, filtered_tvec = self._apply_filter(
                        'board',
                        board_rvec,
                        board_tvec,
                        stamp_sec,
                    )
                    if filtered_rvec is not None and filtered_tvec is not None:
                        self._publish_camera_tf(
                            filtered_rvec,
                            filtered_tvec,
                            msg.header.stamp,
                            f'{self.camera_frame_id}_filtered',
                        )
                        self._publish_board_pose(filtered_rvec, filtered_tvec, msg.header.stamp)
                        self.get_logger().info(
                            f'board pose estimated using {num_markers_used} markers',
                            throttle_duration_sec=1.0,
                        )

                    for i, marker_id in enumerate(ids.flatten()):
                        self._draw_marker_debug(
                            debug_image,
                            corners[i][0],
                            int(marker_id),
                            board_tvec,
                        )
            else:
                for i, marker_id in enumerate(ids.flatten()):
                    marker_id = int(marker_id)
                    rvec, tvec = self._estimate_pose(corners[i][0], marker_id)
                    if rvec is None or tvec is None:
                        continue

                    filtered_rvec, filtered_tvec = self._apply_filter(
                        marker_id,
                        rvec,
                        tvec,
                        stamp_sec,
                    )
                    if filtered_rvec is None or filtered_tvec is None:
                        continue

                    self._publish_marker_tf(marker_id, filtered_rvec, filtered_tvec, msg.header.stamp)
                    self._draw_marker_debug(debug_image, corners[i][0], marker_id, filtered_tvec)

            self._publish_debug_image(debug_image, msg)
        except Exception as exc:
            self.get_logger().error(f'error in image callback: {exc}')

    def _publish_debug_image(self, debug_image, msg: Image) -> None:
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f'error publishing debug image: {exc}')

    def _draw_marker_debug(self, image, corners, marker_id: int, tvec) -> None:
        corners_int = corners.astype(int)
        cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)

        center = corners.mean(axis=0).astype(int)
        distance_mm = float(np.linalg.norm(np.asarray(tvec, dtype=float).reshape(3)) * 1000.0)

        cv2.putText(
            image,
            f'ID:{marker_id}',
            tuple(center + np.array([10, -10])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            image,
            f'{distance_mm:.0f}mm',
            tuple(center + np.array([10, 10])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 255),
            1,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
