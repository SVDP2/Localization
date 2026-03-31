#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker, MarkerArray


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class PoseRvizMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_rviz_marker_node')

        self.declare_parameter('odom_topic', '/localization/relative/odom')
        self.declare_parameter('marker_topic', '/localization/relative/pose_markers')
        self.declare_parameter('marker_namespace', 'relative_pose_viz')
        self.declare_parameter('axis_line_width', 0.01)
        self.declare_parameter('text_size', 0.08)
        self.declare_parameter('vehicle_arrow_length', 0.22)
        self.declare_parameter('vehicle_arrow_width', 0.05)
        self.declare_parameter('good_cov_trace', 0.02)
        self.declare_parameter('warn_cov_trace', 0.12)

        self._marker_topic = self.get_parameter('marker_topic').value
        self._marker_namespace = self.get_parameter('marker_namespace').value
        self._axis_line_width = float(self.get_parameter('axis_line_width').value)
        self._text_size = float(self.get_parameter('text_size').value)
        self._vehicle_arrow_length = float(self.get_parameter('vehicle_arrow_length').value)
        self._vehicle_arrow_width = float(self.get_parameter('vehicle_arrow_width').value)
        self._good_cov_trace = float(self.get_parameter('good_cov_trace').value)
        self._warn_cov_trace = float(self.get_parameter('warn_cov_trace').value)

        odom_topic = self.get_parameter('odom_topic').value
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self.get_logger().info(f'odom topic: {odom_topic}')
        self.get_logger().info(f'marker topic: {self._marker_topic}')

    def _odom_callback(self, msg: Odometry) -> None:
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
        yaw_deg, pitch_deg, roll_deg = Rotation.from_quat(quat).as_euler('ZYX', degrees=True)
        covariance = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        color = self._color_from_covariance(float(np.trace(covariance[:3, :3])))

        marker_array = MarkerArray()
        marker_array.markers = [
            self._axis_lines_marker(msg, position, color),
            self._vehicle_arrow_marker(msg, position, quat, color),
            self._text_marker(msg, 10, f'x {position[0]:+.3f} m', _point(position[0] * 0.5, 0.0, 0.03), color),
            self._text_marker(msg, 11, f'y {position[1]:+.3f} m', _point(0.0, position[1] * 0.5, 0.08), color),
            self._text_marker(msg, 12, f'z {position[2]:+.3f} m', _point(0.0, 0.0, max(position[2] * 0.5, 0.12)), color),
            self._text_marker(msg, 13, f'roll {roll_deg:+.1f} deg', _point(position[0], position[1], position[2] + 0.12), color),
            self._text_marker(msg, 14, f'pitch {pitch_deg:+.1f} deg', _point(position[0], position[1], position[2] + 0.22), color),
            self._text_marker(msg, 15, f'yaw {yaw_deg:+.1f} deg', _point(position[0], position[1], position[2] + 0.32), color),
        ]
        self._marker_pub.publish(marker_array)

    def _axis_lines_marker(self, msg: Odometry, position: np.ndarray, color: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header = msg.header
        marker.ns = self._marker_namespace
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self._axis_line_width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [
            _point(0.0, 0.0, 0.0),
            _point(position[0], 0.0, 0.0),
            _point(0.0, 0.0, 0.0),
            _point(0.0, position[1], 0.0),
            _point(0.0, 0.0, 0.0),
            _point(0.0, 0.0, position[2]),
        ]
        marker.lifetime = Duration(seconds=0.5).to_msg()
        return marker

    def _vehicle_arrow_marker(
        self,
        msg: Odometry,
        position: np.ndarray,
        quat: np.ndarray,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header = msg.header
        marker.ns = self._marker_namespace
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.x = float(quat[0])
        marker.pose.orientation.y = float(quat[1])
        marker.pose.orientation.z = float(quat[2])
        marker.pose.orientation.w = float(quat[3])
        marker.scale.x = self._vehicle_arrow_length
        marker.scale.y = self._vehicle_arrow_width
        marker.scale.z = self._vehicle_arrow_width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = Duration(seconds=0.5).to_msg()
        return marker

    def _text_marker(
        self,
        msg: Odometry,
        marker_id: int,
        text: str,
        position: Point,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header = msg.header
        marker.ns = self._marker_namespace
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._text_size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        marker.lifetime = Duration(seconds=0.5).to_msg()
        return marker

    def _color_from_covariance(self, position_cov_trace: float) -> tuple[float, float, float, float]:
        if position_cov_trace <= self._good_cov_trace:
            return 0.20, 0.85, 0.36, 1.0
        if position_cov_trace <= self._warn_cov_trace:
            return 0.95, 0.75, 0.18, 1.0
        return 0.95, 0.25, 0.25, 1.0


def main() -> None:
    rclpy.init()
    node = PoseRvizMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
