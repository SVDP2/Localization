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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

from aruco_imu_eskf_localization.board_pose_estimator import (
    BoardDefinition,
    BoardPoseEstimate,
    pose_to_matrix,
)
from aruco_imu_eskf_localization.frame_conventions import transform_leader_rear_from_board


PACKAGE_NAME = 'aruco_imu_eskf_localization'


def _stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_prior_is_fresh(
    previous_stamp_ns: int | None,
    current_stamp_ns: int,
    timeout_sec: float,
) -> bool:
    if previous_stamp_ns is None:
        return False
    return (current_stamp_ns - previous_stamp_ns) <= int(float(timeout_sec) * 1_000_000_000)


def _transform_matrix(translation, quat_xyzw):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def _corner_refinement_method(name: str) -> int:
    normalized = str(name).strip().upper()
    mapping = {
        'NONE': getattr(cv2.aruco, 'CORNER_REFINE_NONE', 0),
        'SUBPIX': getattr(cv2.aruco, 'CORNER_REFINE_SUBPIX', 1),
        'CONTOUR': getattr(cv2.aruco, 'CORNER_REFINE_CONTOUR', 2),
        'APRILTAG': getattr(cv2.aruco, 'CORNER_REFINE_APRILTAG', 3),
    }
    return int(mapping.get(normalized, mapping['SUBPIX']))


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_detector_node')

        self.declare_parameter('marker_config_file', 'config/markers_board.yaml')
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        self.declare_parameter('camera_calibration_file', 'config/cam_intrinsic.yaml')
        self.declare_parameter('camera_frame_id', 'usb_cam')
        self.declare_parameter('board_frame', 'board')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('board_pose_topic', '/localization/aruco/board_pose')
        self.declare_parameter('debug_image_topic', '/localization/aruco/debug_image')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('corner_refinement_method', 'SUBPIX')
        self.declare_parameter('enable_detected_marker_refinement', True)
        self.declare_parameter('min_markers_for_board', 1)
        self.declare_parameter('min_markers_to_initialize', 2)
        self.declare_parameter('max_position_jump_m', 0.35)
        self.declare_parameter('max_rotation_jump_deg', 55.0)
        self.declare_parameter('max_yaw_jump_deg', 40.0)
        self.declare_parameter('max_heading_jump_deg', 40.0)
        self.declare_parameter('max_reprojection_rmse_px', 6.0)
        self.declare_parameter('front_halfspace_min_z_m', 0.05)
        self.declare_parameter('max_view_angle_deg', 75.0)
        self.declare_parameter('feasible_x_min_m', -3.50)
        self.declare_parameter('feasible_x_max_m', -0.10)
        self.declare_parameter('feasible_abs_y_max_m', 1.00)
        self.declare_parameter('feasible_z_min_m', -0.50)
        self.declare_parameter('feasible_z_max_m', 0.80)
        self.declare_parameter('single_marker_prior_timeout_sec', 0.25)
        self.declare_parameter('single_marker_min_score_margin', 0.25)
        self.declare_parameter('pose_refinement_method', 'lm')
        self.declare_parameter('base_to_camera_translation', [0.27, 0.0, 0.135])
        self.declare_parameter(
            'base_to_camera_quaternion_xyzw',
            [-0.5, 0.5, -0.5, 0.5],
        )

        marker_config_file = self.get_parameter('marker_config_file').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        camera_calibration_file = self.get_parameter('camera_calibration_file').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.board_frame = self.get_parameter('board_frame').value
        image_topic = self.get_parameter('image_topic').value
        board_pose_topic = self.get_parameter('board_pose_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        corner_refinement = self.get_parameter('corner_refinement_method').value
        self.enable_detected_marker_refinement = bool(
            self.get_parameter('enable_detected_marker_refinement').value
        )
        self.min_markers_for_board = int(self.get_parameter('min_markers_for_board').value)
        self.min_markers_to_initialize = int(
            self.get_parameter('min_markers_to_initialize').value
        )
        self.max_position_jump_m = float(self.get_parameter('max_position_jump_m').value)
        self.max_rotation_jump_deg = float(self.get_parameter('max_rotation_jump_deg').value)
        self.max_yaw_jump_deg = float(self.get_parameter('max_yaw_jump_deg').value)
        self.max_heading_jump_deg = float(
            self.get_parameter('max_heading_jump_deg').value
        )
        self.max_reprojection_rmse_px = float(
            self.get_parameter('max_reprojection_rmse_px').value
        )
        self.front_halfspace_min_z_m = float(
            self.get_parameter('front_halfspace_min_z_m').value
        )
        self.max_view_angle_deg = float(self.get_parameter('max_view_angle_deg').value)
        self.feasible_x_min_m = float(self.get_parameter('feasible_x_min_m').value)
        self.feasible_x_max_m = float(self.get_parameter('feasible_x_max_m').value)
        self.feasible_abs_y_max_m = float(
            self.get_parameter('feasible_abs_y_max_m').value
        )
        self.feasible_z_min_m = float(self.get_parameter('feasible_z_min_m').value)
        self.feasible_z_max_m = float(self.get_parameter('feasible_z_max_m').value)
        self.single_marker_prior_timeout_sec = float(
            self.get_parameter('single_marker_prior_timeout_sec').value
        )
        self.single_marker_min_score_margin = float(
            self.get_parameter('single_marker_min_score_margin').value
        )
        self.pose_refinement_method = str(
            self.get_parameter('pose_refinement_method').value
        )
        self._base_to_camera = _transform_matrix(
            self.get_parameter('base_to_camera_translation').value,
            self.get_parameter('base_to_camera_quaternion_xyzw').value,
        )
        self._camera_to_base = np.linalg.inv(self._base_to_camera)
        self._leader_rear_from_board = transform_leader_rear_from_board()

        self.bridge = CvBridge()
        self.board_definition = self._load_board_config(marker_config_file)
        self.camera_matrix, self.dist_coeffs = self._load_camera_calibration(
            camera_calibration_file
        )

        aruco_dict_id = getattr(cv2.aruco, aruco_dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = _corner_refinement_method(corner_refinement)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self._aruco_board = self.board_definition.as_aruco_board(self.aruco_dict)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.board_pose_pub = self.create_publisher(PoseWithCovarianceStamped, board_pose_topic, 10)

        self._last_board_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._last_board_pose_stamp_ns: int | None = None

        self.get_logger().info(f'aruco dict: {aruco_dict_name}')
        self.get_logger().info(f'image topic: {image_topic}')
        self.get_logger().info(f'board pose topic: {board_pose_topic}')
        self.get_logger().info(
            f'loaded {len(self.board_definition.marker_sizes_m)} marker definitions'
        )
        if self.board_definition.description:
            self.get_logger().info(f'board: {self.board_definition.description}')

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

    def _load_board_config(self, config_file: str) -> BoardDefinition:
        config_path = self._resolve_config_path(config_file)
        with open(config_path, 'r', encoding='utf-8') as file_obj:
            config = yaml.safe_load(file_obj)
        return BoardDefinition.from_config(config)

    def _load_camera_calibration(self, calib_file: str):
        calib_path = self._resolve_config_path(calib_file)
        with open(calib_path, 'r', encoding='utf-8') as file_obj:
            calib = yaml.safe_load(file_obj)

        camera_matrix = np.asarray(calib['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(
            calib['distortion_coefficients']['data'],
            dtype=np.float64,
        )
        return camera_matrix, dist_coeffs

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            debug_image = cv_image.copy()
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)

            if self.enable_detected_marker_refinement and ids is not None and len(ids) > 0:
                corners, ids, rejected, _ = self.detector.refineDetectedMarkers(
                    gray,
                    self._aruco_board,
                    corners,
                    ids,
                    rejected,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

            if ids is None or len(ids) == 0:
                self._draw_status_overlay(debug_image, 'no markers')
                self._publish_debug_image(debug_image, msg)
                return

            current_stamp_ns = _stamp_to_nanoseconds(msg.header.stamp)
            previous_board_pose = (
                self._last_board_pose
                if pose_prior_is_fresh(
                    self._last_board_pose_stamp_ns,
                    current_stamp_ns,
                    self.single_marker_prior_timeout_sec,
                )
                else None
            )
            estimate = self.board_definition.estimate_pose(
                corners,
                ids,
                self.camera_matrix,
                self.dist_coeffs,
                previous_board_pose=previous_board_pose,
                camera_to_base=self._camera_to_base,
                min_markers=self.min_markers_for_board,
                min_markers_to_initialize=self.min_markers_to_initialize,
                max_position_jump_m=self.max_position_jump_m,
                max_rotation_jump_deg=self.max_rotation_jump_deg,
                max_heading_jump_deg=self.max_heading_jump_deg,
                max_yaw_jump_deg=self.max_yaw_jump_deg,
                front_halfspace_min_z_m=self.front_halfspace_min_z_m,
                max_view_angle_deg=self.max_view_angle_deg,
                feasible_x_min_m=self.feasible_x_min_m,
                feasible_x_max_m=self.feasible_x_max_m,
                feasible_abs_y_max_m=self.feasible_abs_y_max_m,
                feasible_z_min_m=self.feasible_z_min_m,
                feasible_z_max_m=self.feasible_z_max_m,
                single_marker_min_score_margin=self.single_marker_min_score_margin,
                pose_refinement_method=self.pose_refinement_method,
            )

            if estimate is not None and estimate.reprojection_rmse_px <= self.max_reprojection_rmse_px:
                self._last_board_pose = estimate.board_pose
                self._last_board_pose_stamp_ns = current_stamp_ns
                if self.tf_broadcaster is not None:
                    self._publish_camera_tf(estimate, msg.header.stamp)
                self._publish_board_pose(estimate, msg.header.stamp)
                self._draw_detected_markers(debug_image, corners, ids)
                self._draw_estimate_overlay(debug_image, estimate)
            else:
                rejection_reason = 'rejected'
                if estimate is not None:
                    rejection_reason = (
                        f'rmse {estimate.reprojection_rmse_px:.2f}px > '
                        f'{self.max_reprojection_rmse_px:.2f}px'
                    )
                self._draw_detected_markers(debug_image, corners, ids)
                self._draw_status_overlay(debug_image, rejection_reason)

            self._publish_debug_image(debug_image, msg)
        except Exception as exc:
            self.get_logger().error(f'error in image callback: {exc}')

    def _publish_board_pose(self, estimate: BoardPoseEstimate, timestamp) -> None:
        board_rvec, board_tvec = estimate.board_pose
        quat = Rotation.from_rotvec(board_rvec).as_quat()
        covariance = self._measurement_covariance(estimate)

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
        pose_msg.pose.covariance = covariance.flatten().tolist()
        self.board_pose_pub.publish(pose_msg)

    def _publish_camera_tf(self, estimate: BoardPoseEstimate, timestamp) -> None:
        if self.tf_broadcaster is None:
            return

        board_rvec, board_tvec = estimate.board_pose
        quat = Rotation.from_rotvec(board_rvec).as_quat()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = self.board_frame
        tf_msg.child_frame_id = self.camera_frame_id
        tf_msg.transform.translation.x = float(board_tvec[0])
        tf_msg.transform.translation.y = float(board_tvec[1])
        tf_msg.transform.translation.z = float(board_tvec[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

    def _measurement_covariance(self, estimate: BoardPoseEstimate) -> np.ndarray:
        if estimate.visible_markers >= 3:
            base_xy_std = 0.015
            base_z_std = 0.03
            base_rot_std = np.deg2rad(1.0)
        elif estimate.visible_markers == 2:
            base_xy_std = 0.03
            base_z_std = 0.06
            base_rot_std = np.deg2rad(2.5)
        else:
            base_xy_std = 0.07
            base_z_std = 0.12
            base_rot_std = np.deg2rad(5.0)

        reprojection_scale = float(np.clip(1.0 + 0.4 * max(estimate.reprojection_rmse_px - 0.5, 0.0), 1.0, 4.0))
        area_scale = float(np.clip(np.sqrt(16000.0 / max(estimate.image_area_px, 1.0)), 1.0, 4.0))
        total_scale = reprojection_scale * area_scale

        covariance = np.zeros((6, 6), dtype=float)
        covariance[0, 0] = (base_xy_std * total_scale) ** 2
        covariance[1, 1] = (base_xy_std * total_scale) ** 2
        covariance[2, 2] = (base_z_std * total_scale) ** 2
        covariance[3, 3] = (base_rot_std * total_scale) ** 2
        covariance[4, 4] = (base_rot_std * total_scale) ** 2
        covariance[5, 5] = (base_rot_std * total_scale) ** 2
        return covariance

    def _publish_debug_image(self, debug_image, msg: Image) -> None:
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

    def _draw_detected_markers(self, image, corners, ids) -> None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

    def _draw_estimate_overlay(self, image, estimate: BoardPoseEstimate) -> None:
        board_rvec, board_tvec = estimate.board_pose
        board_to_camera = pose_to_matrix(board_rvec, board_tvec)
        board_to_base = board_to_camera @ self._camera_to_base
        leader_rear_to_base = self._leader_rear_from_board @ board_to_base
        leader_position = leader_rear_to_base[:3, 3]
        rotation = Rotation.from_matrix(leader_rear_to_base[:3, :3])
        yaw_deg, pitch_deg, roll_deg = rotation.as_euler('ZYX', degrees=True)
        mode = 'single-marker' if estimate.used_single_marker_fallback else 'multi-marker'
        status = (
            f'{mode} | markers={estimate.visible_markers} '
            f'| lx={leader_position[0]:+.2f} ly={leader_position[1]:+.2f} lz={leader_position[2]:+.2f} '
            f'| heading={yaw_deg:+.1f} pitch={pitch_deg:+.1f} roll={roll_deg:+.1f} '
            f'| rmse={estimate.reprojection_rmse_px:.2f}px'
        )
        self._draw_status_overlay(image, status, color=(40, 220, 40))

    def _draw_status_overlay(self, image, text: str, color=(0, 180, 255)) -> None:
        cv2.rectangle(image, (8, 8), (image.shape[1] - 8, 48), (0, 0, 0), -1)
        cv2.putText(
            image,
            text,
            (18, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
