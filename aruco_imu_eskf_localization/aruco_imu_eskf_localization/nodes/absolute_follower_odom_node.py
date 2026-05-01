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


def _rotation_from_quat_msg(quat_msg) -> np.ndarray:
    return Rotation.from_quat(
        [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
    ).as_matrix()


def _transform_from_odom(msg: Odometry) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rotation_from_quat_msg(msg.pose.pose.orientation)
    transform[:3, 3] = _position_from_odom(msg)
    return transform


def _transform_from_tf(tf_msg: TransformStamped) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(
        [
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ]
    ).as_matrix()
    transform[:3, 3] = np.array(
        [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ],
        dtype=float,
    )
    return transform


def _top_left_covariance(covariance_36) -> np.ndarray:
    covariance = np.asarray(covariance_36, dtype=float).reshape(6, 6)
    return covariance[:3, :3]


def _bottom_right_covariance(covariance_36) -> np.ndarray:
    covariance = np.asarray(covariance_36, dtype=float).reshape(6, 6)
    return covariance[3:6, 3:6]


def _sanitize_covariance(covariance: np.ndarray, min_variance: float) -> np.ndarray:
    covariance = 0.5 * (covariance + covariance.T)
    covariance = np.where(np.isfinite(covariance), covariance, 0.0)
    for index in range(covariance.shape[0]):
        covariance[index, index] = max(float(covariance[index, index]), min_variance)
    return covariance


def _make_pose_covariance(
    leader_odom: Odometry,
    relative_odom: Odometry,
    rotation_map_from_leader_rear: np.ndarray,
    min_variance: float,
) -> list[float]:
    covariance = np.zeros((6, 6), dtype=float)
    covariance[0:3, 0:3] = _top_left_covariance(
        leader_odom.pose.covariance
    ) + rotation_map_from_leader_rear @ _top_left_covariance(
        relative_odom.pose.covariance
    ) @ rotation_map_from_leader_rear.T
    covariance[3:6, 3:6] = _bottom_right_covariance(
        leader_odom.pose.covariance
    ) + rotation_map_from_leader_rear @ _bottom_right_covariance(
        relative_odom.pose.covariance
    ) @ rotation_map_from_leader_rear.T
    return _sanitize_covariance(covariance, min_variance).flatten().tolist()


def _make_twist_covariance(
    leader_odom: Odometry,
    relative_odom: Odometry,
    rotation_map_from_leader_rear: np.ndarray,
    min_variance: float,
) -> list[float]:
    covariance = np.zeros((6, 6), dtype=float)
    covariance[0:3, 0:3] = _top_left_covariance(
        leader_odom.twist.covariance
    ) + rotation_map_from_leader_rear @ _top_left_covariance(
        relative_odom.twist.covariance
    ) @ rotation_map_from_leader_rear.T
    covariance[3:6, 3:6] = _bottom_right_covariance(relative_odom.twist.covariance)
    return _sanitize_covariance(covariance, min_variance).flatten().tolist()


def compose_absolute_follower_odom(
    stamp,
    leader_odom: Odometry,
    relative_odom: Odometry,
    leader_base_to_rear: TransformStamped,
    map_frame: str,
    follower_base_frame: str,
    min_variance: float,
) -> Odometry:
    transform_map_leader_base = _transform_from_odom(leader_odom)
    transform_leader_base_rear = _transform_from_tf(leader_base_to_rear)
    transform_leader_rear_follower_base = _transform_from_odom(relative_odom)
    transform_map_leader_rear = transform_map_leader_base @ transform_leader_base_rear
    transform_map_follower_base = (
        transform_map_leader_rear @ transform_leader_rear_follower_base
    )

    rotation_map_from_leader_base = transform_map_leader_base[:3, :3]
    rotation_map_from_leader_rear = transform_map_leader_rear[:3, :3]
    rotation_map_from_follower_base = transform_map_follower_base[:3, :3]
    rotation_leader_rear_from_follower_base = transform_leader_rear_follower_base[
        :3,
        :3,
    ]

    p_leader_rear_follower_base = transform_leader_rear_follower_base[:3, 3]
    t_map_leader_base_to_rear = (
        rotation_map_from_leader_base @ transform_leader_base_rear[:3, 3]
    )

    v_map_leader_base = _linear_velocity_from_odom(leader_odom)
    omega_map_leader = rotation_map_from_leader_base @ _angular_velocity_from_odom(
        leader_odom
    )
    omega_leader_rear = rotation_map_from_leader_rear.T @ omega_map_leader
    v_map_leader_rear = v_map_leader_base + np.cross(
        omega_map_leader,
        t_map_leader_base_to_rear,
    )

    v_follower_base_relative = _linear_velocity_from_odom(relative_odom)
    v_leader_rear_relative = (
        rotation_leader_rear_from_follower_base @ v_follower_base_relative
    )
    v_map_follower_base = v_map_leader_rear + rotation_map_from_leader_rear @ (
        v_leader_rear_relative + np.cross(omega_leader_rear, p_leader_rear_follower_base)
    )
    omega_map_follower = rotation_map_from_follower_base @ _angular_velocity_from_odom(
        relative_odom
    )

    if not (
        np.isfinite(transform_map_follower_base).all()
        and np.isfinite(v_map_follower_base).all()
        and np.isfinite(omega_map_follower).all()
    ):
        raise ValueError('non-finite absolute follower odom composition')

    quat = Rotation.from_matrix(rotation_map_from_follower_base).as_quat()
    output = Odometry()
    output.header.stamp = stamp
    output.header.frame_id = map_frame
    output.child_frame_id = follower_base_frame
    output.pose.pose.position.x = float(transform_map_follower_base[0, 3])
    output.pose.pose.position.y = float(transform_map_follower_base[1, 3])
    output.pose.pose.position.z = float(transform_map_follower_base[2, 3])
    output.pose.pose.orientation.x = float(quat[0])
    output.pose.pose.orientation.y = float(quat[1])
    output.pose.pose.orientation.z = float(quat[2])
    output.pose.pose.orientation.w = float(quat[3])
    output.twist.twist.linear.x = float(v_map_follower_base[0])
    output.twist.twist.linear.y = float(v_map_follower_base[1])
    output.twist.twist.linear.z = float(v_map_follower_base[2])
    output.twist.twist.angular.x = float(omega_map_follower[0])
    output.twist.twist.angular.y = float(omega_map_follower[1])
    output.twist.twist.angular.z = float(omega_map_follower[2])
    output.pose.covariance = _make_pose_covariance(
        leader_odom,
        relative_odom,
        rotation_map_from_leader_rear,
        min_variance,
    )
    output.twist.covariance = _make_twist_covariance(
        leader_odom,
        relative_odom,
        rotation_map_from_leader_rear,
        min_variance,
    )
    return output


class AbsoluteFollowerOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('absolute_follower_odom_node')

        self.declare_parameter('leader_odom_topic', '/leader/localization/gps/odom')
        self.declare_parameter('relative_odom_topic', 'localization/leader_rear/odom')
        self.declare_parameter('absolute_odom_topic', 'localization/global/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('leader_base_frame', 'leader/base_link')
        self.declare_parameter('leader_rear_frame', 'leader/leader_rear')
        self.declare_parameter('follower_base_frame', 'follower/base_link')
        self.declare_parameter('max_input_age_sec', 0.5)
        self.declare_parameter('min_variance', 1.0e-6)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._leader_base_frame = str(self.get_parameter('leader_base_frame').value)
        self._leader_rear_frame = str(self.get_parameter('leader_rear_frame').value)
        self._follower_base_frame = str(self.get_parameter('follower_base_frame').value)
        self._max_input_age_ns = int(
            max(0.0, float(self.get_parameter('max_input_age_sec').value))
            * 1_000_000_000
        )
        self._min_variance = float(max(1.0e-9, self.get_parameter('min_variance').value))

        self._latest_leader_odom: Odometry | None = None
        self._latest_relative_odom: Odometry | None = None
        self._last_warn_ns: int | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        leader_odom_topic = self.get_parameter('leader_odom_topic').value
        relative_odom_topic = self.get_parameter('relative_odom_topic').value
        absolute_odom_topic = self.get_parameter('absolute_odom_topic').value

        self._absolute_pub = self.create_publisher(Odometry, absolute_odom_topic, 10)
        self.create_subscription(
            Odometry,
            leader_odom_topic,
            self._leader_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            relative_odom_topic,
            self._relative_odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'leader odom topic: {leader_odom_topic}')
        self.get_logger().info(f'relative odom topic: {relative_odom_topic}')
        self.get_logger().info(f'absolute follower odom topic: {absolute_odom_topic}')

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_warn_ns is not None and (now_ns - self._last_warn_ns) < 1_000_000_000:
            return
        self.get_logger().warn(message)
        self._last_warn_ns = now_ns

    def _leader_odom_callback(self, msg: Odometry) -> None:
        self._latest_leader_odom = msg
        if self._latest_relative_odom is not None:
            self._try_publish(self._latest_relative_odom.header.stamp)

    def _relative_odom_callback(self, msg: Odometry) -> None:
        self._latest_relative_odom = msg
        self._try_publish(msg.header.stamp)

    def _try_publish(self, stamp) -> None:
        leader_odom = self._latest_leader_odom
        relative_odom = self._latest_relative_odom
        if leader_odom is None or relative_odom is None:
            return

        stamp_ns = _stamp_to_nanoseconds(stamp)
        if self._is_stale(leader_odom, stamp_ns):
            self._warn_throttled('waiting for fresh leader odom to build absolute follower odom')
            return
        if leader_odom.header.frame_id != self._map_frame:
            self._warn_throttled(
                f'ignoring leader odom in frame {leader_odom.header.frame_id!r}; '
                f'expected {self._map_frame!r}'
            )
            return
        if relative_odom.header.frame_id != self._leader_rear_frame:
            self._warn_throttled(
                f'ignoring relative odom in frame {relative_odom.header.frame_id!r}; '
                f'expected {self._leader_rear_frame!r}'
            )
            return

        try:
            leader_base_to_rear = self._tf_buffer.lookup_transform(
                self._leader_base_frame,
                self._leader_rear_frame,
                Time(),
            )
            output = compose_absolute_follower_odom(
                stamp=stamp,
                leader_odom=leader_odom,
                relative_odom=relative_odom,
                leader_base_to_rear=leader_base_to_rear,
                map_frame=self._map_frame,
                follower_base_frame=self._follower_base_frame,
                min_variance=self._min_variance,
            )
        except Exception as exc:
            self._warn_throttled(f'waiting for absolute follower odom inputs: {exc}')
            return

        self._absolute_pub.publish(output)

    def _is_stale(self, msg: Odometry, reference_stamp_ns: int) -> bool:
        return abs(_stamp_to_nanoseconds(msg.header.stamp) - reference_stamp_ns) > self._max_input_age_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AbsoluteFollowerOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
