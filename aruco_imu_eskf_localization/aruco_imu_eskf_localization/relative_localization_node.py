import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from aruco_imu_eskf_localization.pose_filter import PoseFilter


def _transform_matrix(translation, quat_xyzw):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def _pose_to_matrix(position, quat_xyzw):
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    pose[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return pose


class RelativeLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_imu_eskf_localization_node')

        self.declare_parameter('imu_topic', '/localization/imu/data')
        self.declare_parameter('board_pose_topic', '/localization/aruco/board_pose')
        self.declare_parameter('odom_topic', '/localization/relative/odom')
        self.declare_parameter('pose_topic', '/localization/relative/pose')
        self.declare_parameter('board_frame', 'board')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'usb_cam')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_on_imu', True)
        self.declare_parameter('reset_timeout_sec', 1.0)
        self.declare_parameter('position_process_noise_std_mps2', 0.3)
        self.declare_parameter('measurement_position_std_xy_m', 0.02)
        self.declare_parameter('measurement_position_std_z_m', 0.05)
        self.declare_parameter('orientation_outlier_threshold_deg', 45.0)
        self.declare_parameter('orientation_measurement_alpha', 0.15)
        self.declare_parameter('output_orientation_std_rad', 0.1)
        self.declare_parameter('base_to_imu_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('base_to_imu_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('base_to_camera_translation', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'base_to_camera_quaternion_xyzw',
            [-0.5, 0.5, -0.5, 0.5],
        )

        self._board_frame = self.get_parameter('board_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._publish_on_imu = bool(self.get_parameter('publish_on_imu').value)
        self._reset_timeout_sec = float(self.get_parameter('reset_timeout_sec').value)
        self._orientation_output_std = float(self.get_parameter('output_orientation_std_rad').value)

        self._base_to_imu = _transform_matrix(
            self.get_parameter('base_to_imu_translation').value,
            self.get_parameter('base_to_imu_quaternion_xyzw').value,
        )
        self._base_to_camera = _transform_matrix(
            self.get_parameter('base_to_camera_translation').value,
            self.get_parameter('base_to_camera_quaternion_xyzw').value,
        )
        self._camera_to_base = np.linalg.inv(self._base_to_camera)
        self._rotation_base_from_imu = self._base_to_imu[:3, :3]

        self._filter = PoseFilter(
            process_noise_std=float(
                self.get_parameter('position_process_noise_std_mps2').value
            ),
            measurement_noise_std_xy=float(
                self.get_parameter('measurement_position_std_xy_m').value
            ),
            measurement_noise_std_z=float(
                self.get_parameter('measurement_position_std_z_m').value
            ),
            orientation_outlier_threshold_deg=float(
                self.get_parameter('orientation_outlier_threshold_deg').value
            ),
            orientation_meas_alpha=float(
                self.get_parameter('orientation_measurement_alpha').value
            ),
        )

        self._last_imu_time = None
        self._last_measurement_time = None

        imu_topic = self.get_parameter('imu_topic').value
        board_pose_topic = self.get_parameter('board_pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Imu, imu_topic, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped,
            board_pose_topic,
            self._board_pose_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'imu topic: {imu_topic}')
        self.get_logger().info(f'board pose topic: {board_pose_topic}')
        self.get_logger().info(
            'base_to_camera extrinsic loaded; update config if your camera/body '
            'offset differs from defaults.'
        )

    def _board_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        if self._last_measurement_time is not None:
            if (stamp_sec - self._last_measurement_time) > self._reset_timeout_sec:
                self._filter.reset()
        self._last_measurement_time = stamp_sec

        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        quat = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        board_to_camera = _pose_to_matrix(position, quat)
        board_to_base = board_to_camera @ self._camera_to_base

        rvec = Rotation.from_matrix(board_to_base[:3, :3]).as_rotvec()
        tvec = board_to_base[:3, 3]
        self._filter.update(tvec, rvec)
        self._publish_outputs(msg.header.stamp)

    def _imu_callback(self, msg: Imu) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        if self._last_imu_time is None:
            self._last_imu_time = stamp_sec
            return

        dt = stamp_sec - self._last_imu_time
        self._last_imu_time = stamp_sec
        if dt <= 0.0:
            return

        omega_imu = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=float,
        )
        omega_base = self._rotation_base_from_imu @ omega_imu

        if self._filter.initialized:
            self._filter.predict(dt, omega_base)
            if self._publish_on_imu:
                self._publish_outputs(msg.header.stamp)

    def _publish_outputs(self, stamp) -> None:
        rvec, tvec = self._filter.get_pose()
        if rvec is None or tvec is None:
            return
        quat = Rotation.from_rotvec(rvec).as_quat()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._board_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = float(tvec[0])
        odom_msg.pose.pose.position.y = float(tvec[1])
        odom_msg.pose.pose.position.z = float(tvec[2])
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])
        odom_msg.twist.twist.linear.x = float(self._filter.kf.x[3, 0])
        odom_msg.twist.twist.linear.y = float(self._filter.kf.x[4, 0])
        odom_msg.twist.twist.linear.z = float(self._filter.kf.x[5, 0])

        pose_cov = np.zeros((6, 6), dtype=float)
        pose_cov[:3, :3] = self._filter.kf.P[:3, :3]
        pose_cov[3:, 3:] = np.diag([self._orientation_output_std**2] * 3)
        odom_msg.pose.covariance = pose_cov.flatten().tolist()
        self._odom_pub.publish(odom_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose
        self._pose_pub.publish(pose_msg)

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._board_frame
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = float(tvec[0])
            tf_msg.transform.translation.y = float(tvec[1])
            tf_msg.transform.translation.z = float(tvec[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
