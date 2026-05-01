from __future__ import annotations

import math

import numpy as np

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from aruco_imu_eskf_localization.common.geodesy import Wgs84Origin, lla_to_enu


def _stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class GpsOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('gps_odom_node')

        self.declare_parameter('fix_topic', 'ublox_gps_node/fix')
        self.declare_parameter('fix_velocity_topic', 'ublox_gps_node/fix_velocity')
        self.declare_parameter('odom_topic', 'localization/gps/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('child_frame', 'follower/follower_gps')
        self.declare_parameter('map_origin_lat_deg', 0.0)
        self.declare_parameter('map_origin_lon_deg', 0.0)
        self.declare_parameter('map_origin_alt_m', 0.0)
        self.declare_parameter('max_velocity_age_sec', 0.5)
        self.declare_parameter('fallback_velocity_variance', 1.0e6)

        self._origin = Wgs84Origin(
            latitude_deg=float(self.get_parameter('map_origin_lat_deg').value),
            longitude_deg=float(self.get_parameter('map_origin_lon_deg').value),
            altitude_m=float(self.get_parameter('map_origin_alt_m').value),
        )
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._child_frame = str(self.get_parameter('child_frame').value)
        self._max_velocity_age_ns = int(
            max(0.0, float(self.get_parameter('max_velocity_age_sec').value)) * 1_000_000_000
        )
        self._fallback_velocity_variance = float(
            max(1.0, self.get_parameter('fallback_velocity_variance').value)
        )
        self._latest_velocity: TwistWithCovarianceStamped | None = None
        self._last_warn_ns: int | None = None

        fix_topic = self.get_parameter('fix_topic').value
        fix_velocity_topic = self.get_parameter('fix_velocity_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(
            NavSatFix,
            fix_topic,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            fix_velocity_topic,
            self._velocity_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'fix topic: {fix_topic}')
        self.get_logger().info(f'fix velocity topic: {fix_velocity_topic}')
        self.get_logger().info(f'gps odom topic: {odom_topic}')
        self.get_logger().info(
            f'map origin lat={self._origin.latitude_deg:.8f}, '
            f'lon={self._origin.longitude_deg:.8f}, alt={self._origin.altitude_m:.3f}'
        )

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_warn_ns is not None and (now_ns - self._last_warn_ns) < 1_000_000_000:
            return
        self.get_logger().warn(message)
        self._last_warn_ns = now_ns

    def _velocity_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self._latest_velocity = msg

    def _fix_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self._warn_throttled('dropping GPS fix without valid status')
            return
        if (
            not math.isfinite(msg.latitude)
            or not math.isfinite(msg.longitude)
            or abs(msg.latitude) > 90.0
            or abs(msg.longitude) > 180.0
        ):
            self._warn_throttled('dropping GPS fix with invalid latitude/longitude')
            return

        altitude = float(msg.altitude)
        if not math.isfinite(altitude) or abs(altitude) > 1.0e6:
            altitude = self._origin.altitude_m

        position_enu = lla_to_enu(msg.latitude, msg.longitude, altitude, self._origin)
        if not np.isfinite(position_enu).all():
            self._warn_throttled('dropping GPS fix after non-finite ENU projection')
            return

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = float(position_enu[0])
        odom.pose.pose.position.y = float(position_enu[1])
        odom.pose.pose.position.z = float(position_enu[2])
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance = self._pose_covariance_from_fix(msg)
        odom.twist.covariance = [0.0] * 36
        self._copy_velocity_if_fresh(msg, odom)
        self._odom_pub.publish(odom)

    def _pose_covariance_from_fix(self, msg: NavSatFix) -> list[float]:
        covariance = [0.0] * 36
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            covariance[0] = 1.0
            covariance[7] = 1.0
            covariance[14] = 4.0
            return covariance

        covariance[0] = float(max(msg.position_covariance[0], 1.0e-6))
        covariance[7] = float(max(msg.position_covariance[4], 1.0e-6))
        covariance[14] = float(max(msg.position_covariance[8], 1.0e-6))
        return covariance

    def _copy_velocity_if_fresh(self, fix_msg: NavSatFix, odom_msg: Odometry) -> None:
        latest = self._latest_velocity
        if latest is None:
            odom_msg.twist.covariance[0] = self._fallback_velocity_variance
            odom_msg.twist.covariance[7] = self._fallback_velocity_variance
            odom_msg.twist.covariance[14] = self._fallback_velocity_variance
            return

        fix_stamp_ns = _stamp_to_nanoseconds(fix_msg.header.stamp)
        vel_stamp_ns = _stamp_to_nanoseconds(latest.header.stamp)
        if abs(fix_stamp_ns - vel_stamp_ns) > self._max_velocity_age_ns:
            odom_msg.twist.covariance[0] = self._fallback_velocity_variance
            odom_msg.twist.covariance[7] = self._fallback_velocity_variance
            odom_msg.twist.covariance[14] = self._fallback_velocity_variance
            return

        odom_msg.twist.twist.linear = latest.twist.twist.linear
        odom_msg.twist.covariance = list(latest.twist.covariance)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
