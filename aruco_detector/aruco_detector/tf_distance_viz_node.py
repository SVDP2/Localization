#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


class TfDistanceVizNode(Node):
    def __init__(self) -> None:
        super().__init__('tf_distance_viz_node')

        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('target_frame_id', 'aruco_marker_0')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('line_width', 0.003)
        self.declare_parameter('text_size', 0.04)
        self.declare_parameter('marker_topic', '/aruco_detector/distance_markers')
        self.declare_parameter('marker_namespace', 'aruco_distance_viz')

        self._camera_frame_id = self.get_parameter('camera_frame_id').value
        self._target_frame_id = self.get_parameter('target_frame_id').value
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._line_width = float(self.get_parameter('line_width').value)
        self._text_size = float(self.get_parameter('text_size').value)
        self._marker_topic = self.get_parameter('marker_topic').value
        self._marker_namespace = self.get_parameter('marker_namespace').value

        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        rate_hz = max(1e-3, self._publish_rate)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _make_text_marker(
        self,
        marker_id: int,
        stamp,
        text: str,
        position: Point,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._camera_frame_id
        marker.header.stamp = stamp
        marker.ns = self._marker_namespace
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = position
        marker.pose.orientation.w = 1.0

        marker.scale.z = float(self._text_size)

        # Mint-ish color.
        marker.color.r = 0.24
        marker.color.g = 0.71
        marker.color.b = 0.52
        marker.color.a = 1.0

        marker.text = text
        marker.lifetime = Duration(seconds=1.0).to_msg()
        return marker

    def _on_timer(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._camera_frame_id,
                self._target_frame_id,
                Time(),
            )
        except Exception as exc:
            self.get_logger().debug(
                f'No TF {self._camera_frame_id} -> {self._target_frame_id}: {exc}'
            )
            return

        dx = float(tf.transform.translation.x)
        dy = float(tf.transform.translation.y)
        dz = float(tf.transform.translation.z)

        stamp = self.get_clock().now().to_msg()

        origin = _point(0.0, 0.0, 0.0)
        x_end = _point(dx, 0.0, 0.0)
        y_end = _point(0.0, dy, 0.0)
        z_end = _point(0.0, 0.0, dz)

        line_marker = Marker()
        line_marker.header.frame_id = self._camera_frame_id
        line_marker.header.stamp = stamp
        line_marker.ns = self._marker_namespace
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD

        line_marker.scale.x = float(self._line_width)

        line_marker.color.r = 0.24
        line_marker.color.g = 0.71
        line_marker.color.b = 0.52
        line_marker.color.a = 1.0

        # Three orthogonal components, all starting at the camera origin.
        line_marker.points = [
            origin,
            x_end,
            origin,
            y_end,
            origin,
            z_end,
        ]
        line_marker.lifetime = Duration(seconds=1.0).to_msg()

        # Place text near the center of each segment.
        label_offset = 0.001
        x_mid = _point(dx * 0.5, label_offset, 0.0)
        y_mid = _point(label_offset, dy * 0.5, 0.0)
        z_mid = _point(label_offset, 0.0, dz * 0.5)

        x_text = f'{dx * 100.0:+.1f} cm'
        y_text = f'{dy * 100.0:+.1f} cm'
        z_text = f'{dz * 100.0:+.1f} cm'

        marker_array = MarkerArray()
        marker_array.markers = [
            line_marker,
            self._make_text_marker(1, stamp, x_text, x_mid),
            self._make_text_marker(2, stamp, y_text, y_mid),
            self._make_text_marker(3, stamp, z_text, z_mid),
        ]

        self._marker_pub.publish(marker_array)


def main() -> None:
    rclpy.init()
    node = TfDistanceVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
