from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def _stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _rotation_from_quat_msg(quat_msg) -> np.ndarray:
    return Rotation.from_quat(
        [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
    ).as_matrix()


def _position_from_odom(msg: Odometry) -> np.ndarray:
    return np.array(
        [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ],
        dtype=float,
    )


def _linear_velocity_from_odom(msg: Odometry) -> np.ndarray:
    return np.array(
        [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ],
        dtype=float,
    )


def _angular_velocity_from_odom(msg: Odometry) -> np.ndarray:
    return np.array(
        [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ],
        dtype=float,
    )


def _translation_from_tf(tf_msg: TransformStamped) -> np.ndarray:
    return np.array(
        [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ],
        dtype=float,
    )


def _rotation_from_tf(tf_msg: TransformStamped) -> np.ndarray:
    return Rotation.from_quat(
        [
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ]
    ).as_matrix()


def _top_left_covariance(covariance_36) -> np.ndarray:
    covariance = np.asarray(covariance_36, dtype=float).reshape(6, 6)
    return covariance[:3, :3]


class RelativeGpsNode(Node):
    def __init__(self) -> None:
        super().__init__('relative_gps_node')

        self.declare_parameter('follower_gps_odom_topic', 'localization/gps/odom')
        self.declare_parameter('leader_gps_odom_topic', '/leader/localization/gps/odom')
        self.declare_parameter('fused_odom_topic', 'localization/leader_rear/odom')
        self.declare_parameter('relative_gps_odom_topic', 'localization/gps_relative/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('leader_base_frame', 'leader/base_link')
        self.declare_parameter('leader_rear_frame', 'leader/leader_rear')
        self.declare_parameter('follower_base_frame', 'follower/base_link')
        self.declare_parameter('follower_gps_frame', 'follower/follower_gps')
        self.declare_parameter('max_input_age_sec', 0.5)
        self.declare_parameter('min_variance', 1.0e-6)

        self._map_frame = self.get_parameter('map_frame').value
        self._leader_base_frame = self.get_parameter('leader_base_frame').value
        self._leader_rear_frame = self.get_parameter('leader_rear_frame').value
        self._follower_base_frame = self.get_parameter('follower_base_frame').value
        self._follower_gps_frame = self.get_parameter('follower_gps_frame').value
        self._max_input_age_ns = int(
            max(0.0, float(self.get_parameter('max_input_age_sec').value)) * 1_000_000_000
        )
        self._min_variance = float(max(1.0e-9, self.get_parameter('min_variance').value))

        self._latest_follower_gps: Odometry | None = None
        self._latest_leader_odom: Odometry | None = None
        self._latest_fused_odom: Odometry | None = None
        self._last_warn_ns: int | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        follower_gps_odom_topic = self.get_parameter('follower_gps_odom_topic').value
        leader_gps_odom_topic = self.get_parameter('leader_gps_odom_topic').value
        fused_odom_topic = self.get_parameter('fused_odom_topic').value
        relative_gps_odom_topic = self.get_parameter('relative_gps_odom_topic').value

        self._relative_pub = self.create_publisher(Odometry, relative_gps_odom_topic, 10)
        self.create_subscription(
            Odometry,
            follower_gps_odom_topic,
            self._follower_gps_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            leader_gps_odom_topic,
            self._leader_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            fused_odom_topic,
            self._fused_odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'follower gps odom topic: {follower_gps_odom_topic}')
        self.get_logger().info(f'leader gps odom topic: {leader_gps_odom_topic}')
        self.get_logger().info(f'fused odom topic: {fused_odom_topic}')
        self.get_logger().info(f'relative gps odom topic: {relative_gps_odom_topic}')

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_warn_ns is not None and (now_ns - self._last_warn_ns) < 1_000_000_000:
            return
        self.get_logger().warn(message)
        self._last_warn_ns = now_ns

    def _follower_gps_callback(self, msg: Odometry) -> None:
        self._latest_follower_gps = msg
        self._try_publish(msg.header.stamp)

    def _leader_odom_callback(self, msg: Odometry) -> None:
        self._latest_leader_odom = msg
        if self._latest_follower_gps is not None:
            self._try_publish(self._latest_follower_gps.header.stamp)

    def _fused_odom_callback(self, msg: Odometry) -> None:
        self._latest_fused_odom = msg

    def _try_publish(self, stamp) -> None:
        follower_gps = self._latest_follower_gps
        leader_odom = self._latest_leader_odom
        fused_odom = self._latest_fused_odom
        if follower_gps is None or leader_odom is None or fused_odom is None:
            return

        stamp_ns = _stamp_to_nanoseconds(stamp)
        if self._is_stale(leader_odom, stamp_ns) or self._is_stale(fused_odom, stamp_ns):
            self._warn_throttled('waiting for fresh leader/fused odom to build relative GPS')
            return

        if follower_gps.header.frame_id != self._map_frame:
            self._warn_throttled(
                f'ignoring follower GPS odom in frame {follower_gps.header.frame_id!r}; '
                f'expected {self._map_frame!r}'
            )
            return
        if leader_odom.header.frame_id != self._map_frame:
            self._warn_throttled(
                f'ignoring leader odom in frame {leader_odom.header.frame_id!r}; '
                f'expected {self._map_frame!r}'
            )
            return

        try:
            leader_base_to_rear = self._tf_buffer.lookup_transform(
                self._leader_base_frame,
                self._leader_rear_frame,
                Time(),
            )
            follower_base_to_gps = self._tf_buffer.lookup_transform(
                self._follower_base_frame,
                self._follower_gps_frame,
                Time(),
            )
        except Exception as exc:
            self._warn_throttled(f'waiting for GPS lever-arm TFs: {exc}')
            return

        output = self._make_relative_odom(
            stamp,
            follower_gps,
            leader_odom,
            fused_odom,
            leader_base_to_rear,
            follower_base_to_gps,
        )
        if output is not None:
            self._relative_pub.publish(output)

    def _is_stale(self, msg: Odometry, reference_stamp_ns: int) -> bool:
        return abs(_stamp_to_nanoseconds(msg.header.stamp) - reference_stamp_ns) > self._max_input_age_ns

    def _make_relative_odom(
        self,
        stamp,
        follower_gps: Odometry,
        leader_odom: Odometry,
        fused_odom: Odometry,
        leader_base_to_rear: TransformStamped,
        follower_base_to_gps: TransformStamped,
    ) -> Odometry | None:
        p_map_follower_gps = _position_from_odom(follower_gps)
        v_map_follower_gps = _linear_velocity_from_odom(follower_gps)
        p_map_leader_base = _position_from_odom(leader_odom)
        v_map_leader_base = _linear_velocity_from_odom(leader_odom)

        rotation_map_from_leader_base = _rotation_from_quat_msg(leader_odom.pose.pose.orientation)
        rotation_leader_base_from_rear = _rotation_from_tf(leader_base_to_rear)
        t_leader_base_to_rear = _translation_from_tf(leader_base_to_rear)
        rotation_map_from_leader_rear = (
            rotation_map_from_leader_base @ rotation_leader_base_from_rear
        )
        t_map_leader_base_to_rear = rotation_map_from_leader_base @ t_leader_base_to_rear
        p_map_leader_rear = p_map_leader_base + t_map_leader_base_to_rear

        rotation_leader_rear_from_follower_base = _rotation_from_quat_msg(
            fused_odom.pose.pose.orientation
        )
        rotation_map_from_follower_base = (
            rotation_map_from_leader_rear @ rotation_leader_rear_from_follower_base
        )
        t_follower_base_to_gps = _translation_from_tf(follower_base_to_gps)
        t_map_follower_base_to_gps = rotation_map_from_follower_base @ t_follower_base_to_gps
        p_map_follower_base = p_map_follower_gps - t_map_follower_base_to_gps

        omega_map_follower = (
            rotation_map_from_follower_base @ _angular_velocity_from_odom(fused_odom)
        )
        omega_map_leader = rotation_map_from_leader_base @ _angular_velocity_from_odom(leader_odom)
        v_map_follower_base = v_map_follower_gps - np.cross(
            omega_map_follower,
            t_map_follower_base_to_gps,
        )
        v_map_leader_rear = v_map_leader_base + np.cross(
            omega_map_leader,
            t_map_leader_base_to_rear,
        )

        if not (
            np.isfinite(p_map_follower_base).all()
            and np.isfinite(p_map_leader_rear).all()
            and np.isfinite(v_map_follower_base).all()
            and np.isfinite(v_map_leader_rear).all()
        ):
            self._warn_throttled('dropping non-finite relative GPS measurement')
            return None

        rotation_leader_rear_from_map = rotation_map_from_leader_rear.T
        relative_position = rotation_leader_rear_from_map @ (
            p_map_follower_base - p_map_leader_rear
        )
        omega_leader_rear = rotation_leader_rear_from_map @ omega_map_leader
        relative_velocity = (
            rotation_leader_rear_from_map @ (v_map_follower_base - v_map_leader_rear)
            - np.cross(omega_leader_rear, relative_position)
        )

        pose_cov_map = _top_left_covariance(follower_gps.pose.covariance) + _top_left_covariance(
            leader_odom.pose.covariance
        )
        twist_cov_map = _top_left_covariance(
            follower_gps.twist.covariance
        ) + _top_left_covariance(leader_odom.twist.covariance)
        pose_cov_relative = self._sanitize_covariance(
            rotation_leader_rear_from_map @ pose_cov_map @ rotation_leader_rear_from_map.T
        )
        twist_cov_relative = self._sanitize_covariance(
            rotation_leader_rear_from_map @ twist_cov_map @ rotation_leader_rear_from_map.T
        )

        output = Odometry()
        output.header.stamp = stamp
        output.header.frame_id = self._leader_rear_frame
        output.child_frame_id = self._follower_base_frame
        output.pose.pose.position.x = float(relative_position[0])
        output.pose.pose.position.y = float(relative_position[1])
        output.pose.pose.position.z = float(relative_position[2])
        output.pose.pose.orientation = fused_odom.pose.pose.orientation
        output.twist.twist.linear.x = float(relative_velocity[0])
        output.twist.twist.linear.y = float(relative_velocity[1])
        output.twist.twist.linear.z = float(relative_velocity[2])
        output.pose.covariance = self._covariance_36_from_3x3(pose_cov_relative)
        output.twist.covariance = self._covariance_36_from_3x3(twist_cov_relative)
        return output

    def _sanitize_covariance(self, covariance: np.ndarray) -> np.ndarray:
        covariance = 0.5 * (covariance + covariance.T)
        covariance = np.where(np.isfinite(covariance), covariance, 0.0)
        for index in range(3):
            covariance[index, index] = max(float(covariance[index, index]), self._min_variance)
        return covariance

    @staticmethod
    def _covariance_36_from_3x3(covariance: np.ndarray) -> list[float]:
        covariance_6 = np.zeros((6, 6), dtype=float)
        covariance_6[:3, :3] = covariance
        return covariance_6.flatten().tolist()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeGpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
