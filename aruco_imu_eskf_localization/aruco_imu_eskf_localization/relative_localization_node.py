from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


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

        self.declare_parameter('board_pose_topic', '/localization/aruco/board_pose')
        self.declare_parameter('odom_topic', '/localization/relative/odom')
        self.declare_parameter('pose_topic', '/localization/relative/pose')
        self.declare_parameter('board_frame', 'board')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'usb_cam')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('base_to_camera_translation', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'base_to_camera_quaternion_xyzw',
            [-0.5, 0.5, -0.5, 0.5],
        )

        # Keep compatibility with the existing parameter file.
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('publish_on_imu', False)
        self.declare_parameter('reset_timeout_sec', 1.0)
        self.declare_parameter('position_process_noise_std_mps2', 0.3)
        self.declare_parameter('measurement_position_std_xy_m', 0.02)
        self.declare_parameter('measurement_position_std_z_m', 0.05)
        self.declare_parameter('orientation_outlier_threshold_deg', 45.0)
        self.declare_parameter('orientation_measurement_alpha', 0.15)
        self.declare_parameter('output_orientation_std_rad', 0.1)
        self.declare_parameter('base_to_imu_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('base_to_imu_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])

        self._board_frame = self.get_parameter('board_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        self._base_to_camera = _transform_matrix(
            self.get_parameter('base_to_camera_translation').value,
            self.get_parameter('base_to_camera_quaternion_xyzw').value,
        )
        self._camera_to_base = np.linalg.inv(self._base_to_camera)

        board_pose_topic = self.get_parameter('board_pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(
            PoseWithCovarianceStamped,
            board_pose_topic,
            self._board_pose_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'board pose topic: {board_pose_topic}')
        self.get_logger().info(f'odom topic: {odom_topic}')
        self.get_logger().info(
            'publishing direct board->base pose from detector measurements; '
            'IMU fusion is deferred to a later stage.'
        )

    def _board_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
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
        covariance = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        self._publish_outputs(msg.header.stamp, board_to_base, covariance)

    def _publish_outputs(self, stamp, board_to_base: np.ndarray, covariance: np.ndarray) -> None:
        quat = Rotation.from_matrix(board_to_base[:3, :3]).as_quat()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._board_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = float(board_to_base[0, 3])
        odom_msg.pose.pose.position.y = float(board_to_base[1, 3])
        odom_msg.pose.pose.position.z = float(board_to_base[2, 3])
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])
        odom_msg.pose.covariance = covariance.flatten().tolist()
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
            tf_msg.transform.translation.x = float(board_to_base[0, 3])
            tf_msg.transform.translation.y = float(board_to_base[1, 3])
            tf_msg.transform.translation.z = float(board_to_base[2, 3])
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
