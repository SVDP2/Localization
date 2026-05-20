#!/usr/bin/env python3

import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import yaml


def _flatten_yaml_matrix(value):
    data = value.get('data', value) if isinstance(value, dict) else value
    array = np.array(data, dtype=np.float64)
    return array


def load_camera_calibration(path):
    with open(path, 'r') as handle:
        config = yaml.safe_load(handle)

    camera_matrix = _flatten_yaml_matrix(config['camera_matrix'])
    if camera_matrix.shape != (3, 3):
        camera_matrix = camera_matrix.reshape((3, 3))

    distortion = _flatten_yaml_matrix(config['distortion_coefficients'])
    distortion = distortion.reshape((-1, 1))

    image_size = config.get('image_size', {})
    width = int(image_size.get('width', 0))
    height = int(image_size.get('height', 0))
    camera_model = str(config.get('camera_model', 'pinhole')).lower()

    return camera_matrix, distortion, (width, height), camera_model


def load_lidar_to_camera_extrinsic(path):
    with open(path, 'r') as handle:
        config = yaml.safe_load(handle)

    if 'extrinsic_matrix' in config:
        transform = np.array(config['extrinsic_matrix'], dtype=np.float64)
        return transform[:3, :3], transform[:3, 3].reshape((3, 1))

    rotation = config.get('rotation_matrix')
    if rotation is None:
        rotation = config.get('R')
    translation = config.get('translation')
    if translation is None:
        translation = config.get('t')

    if isinstance(translation, dict):
        translation = [translation['x'], translation['y'], translation['z']]

    rotation_matrix = _flatten_yaml_matrix(rotation)
    if rotation_matrix.shape != (3, 3):
        rotation_matrix = rotation_matrix.reshape((3, 3))
    translation_vector = np.array(translation, dtype=np.float64).reshape((3, 1))
    return rotation_matrix, translation_vector


class CameraLidarProjectionNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_projection_node')

        self.declare_parameter('image_topic', '/follower/image_raw')
        self.declare_parameter('scan_topic', '/follower/scan')
        self.declare_parameter(
            'projected_image_topic', '/follower/camera_lidar/projected_image')
        self.declare_parameter('camera_frame', 'follower/usb_cam')
        self.declare_parameter('lidar_frame', 'follower/lidar')
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('extrinsic_file', '')
        self.declare_parameter('max_projection_range_m', 4.0)
        self.declare_parameter('min_camera_depth_m', 0.03)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('draw_radius_px', 2)
        self.declare_parameter('colorize_by_range', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        projected_image_topic = self.get_parameter('projected_image_topic').value
        self.max_projection_range_m = float(
            self.get_parameter('max_projection_range_m').value)
        self.min_camera_depth_m = float(
            self.get_parameter('min_camera_depth_m').value)
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.draw_radius_px = max(1, int(self.get_parameter('draw_radius_px').value))
        self.colorize_by_range = bool(self.get_parameter('colorize_by_range').value)

        camera_calibration_file = str(
            self.get_parameter('camera_calibration_file').value)
        extrinsic_file = str(self.get_parameter('extrinsic_file').value)
        if not camera_calibration_file:
            raise ValueError('camera_calibration_file parameter is required')
        if not extrinsic_file:
            raise ValueError('extrinsic_file parameter is required')

        self.camera_matrix, self.distortion, self.image_size, self.camera_model = (
            load_camera_calibration(camera_calibration_file)
        )
        self.lidar_to_camera_R, self.lidar_to_camera_t = (
            load_lidar_to_camera_extrinsic(extrinsic_file)
        )

        self.bridge = CvBridge()
        self.latest_scan = None

        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.projected_image_pub = self.create_publisher(Image, projected_image_topic, 10)

        self.get_logger().info(
            'camera_lidar_projection_node started: '
            f'image={self.image_topic} scan={self.scan_topic} '
            f'output={projected_image_topic} model={self.camera_model}')
        self.get_logger().warn(
            'using configured camera-LiDAR extrinsic; validate projection after '
            'changing calibration files')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        if self.latest_scan is None:
            self.get_logger().warn('waiting for LaserScan', throttle_duration_sec=2.0)
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'image conversion failed: {exc}')
            return

        lidar_points = self.scan_to_lidar_points(self.latest_scan)
        projected = self.draw_projected_points(image, lidar_points)
        output = self.bridge.cv2_to_imgmsg(projected, encoding='bgr8')
        output.header = msg.header
        self.projected_image_pub.publish(output)

    def scan_to_lidar_points(self, scan):
        points = []
        angle = scan.angle_min
        for index, distance in enumerate(scan.ranges):
            if index % self.point_stride != 0:
                angle += scan.angle_increment
                continue
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                if distance <= self.max_projection_range_m:
                    points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
            angle += scan.angle_increment

        if not points:
            return np.empty((0, 3), dtype=np.float64)
        return np.array(points, dtype=np.float64)

    def draw_projected_points(self, image, lidar_points):
        if lidar_points.size == 0:
            return image

        camera_points = (self.lidar_to_camera_R @ lidar_points.T + self.lidar_to_camera_t).T
        depth_mask = camera_points[:, 2] > self.min_camera_depth_m
        camera_points = camera_points[depth_mask]
        if camera_points.size == 0:
            return image

        if self.camera_model == 'fisheye':
            object_points = camera_points.reshape((-1, 1, 3)).astype(np.float64)
            image_points, _ = cv2.fisheye.projectPoints(
                object_points,
                np.zeros((3, 1), dtype=np.float64),
                np.zeros((3, 1), dtype=np.float64),
                self.camera_matrix,
                self.distortion[:4],
            )
        else:
            image_points, _ = cv2.projectPoints(
                camera_points.astype(np.float64),
                np.zeros((3, 1), dtype=np.float64),
                np.zeros((3, 1), dtype=np.float64),
                self.camera_matrix,
                self.distortion,
            )

        pixels = image_points.reshape((-1, 2))
        h, w = image.shape[:2]
        ranges = np.linalg.norm(camera_points, axis=1)
        max_range = max(self.max_projection_range_m, 1.0)

        for (u, v), range_m in zip(pixels, ranges):
            px = int(round(u))
            py = int(round(v))
            if 0 <= px < w and 0 <= py < h:
                color = (0, 0, 255)
                if self.colorize_by_range:
                    alpha = min(max(range_m / max_range, 0.0), 1.0)
                    color = (int(255 * alpha), int(255 * (1.0 - alpha)), 255)
                cv2.circle(image, (px, py), self.draw_radius_px, color, -1)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraLidarProjectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
