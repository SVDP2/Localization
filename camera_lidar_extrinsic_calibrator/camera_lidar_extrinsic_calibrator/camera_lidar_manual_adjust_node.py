#!/usr/bin/env python3

from datetime import datetime
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

from camera_lidar_extrinsic_calibrator.calibration_utils import (
    load_camera_calibration,
    load_lidar_to_camera_extrinsic,
    project_lidar_points,
    scan_to_lidar_points,
    write_extrinsic_yaml,
)


def _rotation_x(angle_rad):
    cos_v = math.cos(angle_rad)
    sin_v = math.sin(angle_rad)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cos_v, -sin_v], [0.0, sin_v, cos_v]],
        dtype=np.float64,
    )


def _rotation_y(angle_rad):
    cos_v = math.cos(angle_rad)
    sin_v = math.sin(angle_rad)
    return np.array(
        [[cos_v, 0.0, sin_v], [0.0, 1.0, 0.0], [-sin_v, 0.0, cos_v]],
        dtype=np.float64,
    )


def _rotation_z(angle_rad):
    cos_v = math.cos(angle_rad)
    sin_v = math.sin(angle_rad)
    return np.array(
        [[cos_v, -sin_v, 0.0], [sin_v, cos_v, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


class CameraLidarManualAdjustNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_manual_adjust_node')

        self.declare_parameter('image_topic', '/follower/image_raw')
        self.declare_parameter('scan_topic', '/follower/scan')
        self.declare_parameter('camera_frame', 'follower/usb_cam')
        self.declare_parameter('lidar_frame', 'follower/lidar')
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('initial_extrinsic_file', '')
        self.declare_parameter('output_root', '/home/xytron/follower_ws/calibration/camera_lidar')
        self.declare_parameter('max_lidar_range_m', 5.0)
        self.declare_parameter('min_camera_depth_m', 0.03)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('translation_range_m', 0.5)
        self.declare_parameter('rotation_range_deg', 45.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.output_root = Path(str(self.get_parameter('output_root').value))
        self.max_lidar_range_m = float(self.get_parameter('max_lidar_range_m').value)
        self.min_camera_depth_m = float(self.get_parameter('min_camera_depth_m').value)
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.translation_range_m = float(
            self.get_parameter('translation_range_m').value
        )
        self.rotation_range_deg = float(self.get_parameter('rotation_range_deg').value)

        camera_calibration_file = str(
            self.get_parameter('camera_calibration_file').value
        )
        initial_extrinsic_file = str(
            self.get_parameter('initial_extrinsic_file').value
        )
        if not camera_calibration_file:
            raise ValueError('camera_calibration_file parameter is required')
        if not initial_extrinsic_file:
            raise ValueError('initial_extrinsic_file parameter is required')

        self.camera_matrix, self.distortion, self.image_size, self.camera_model = (
            load_camera_calibration(camera_calibration_file)
        )
        self.base_rotation, self.base_translation = load_lidar_to_camera_extrinsic(
            initial_extrinsic_file
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.frozen_image = None
        self.frozen_scan = None
        self.freeze = False
        self.running = True
        self.status = 'waiting for image and scan'
        self.window_name = 'camera_lidar_manual_adjust'

        self.translation_scale = 1000.0
        self.rotation_scale = 10.0
        self.translation_slider_max = int(round(2.0 * self.translation_range_m * 1000.0))
        self.rotation_slider_max = int(round(2.0 * self.rotation_range_deg * self.rotation_scale))
        self.translation_slider_center = self.translation_slider_max // 2
        self.rotation_slider_center = self.rotation_slider_max // 2

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.create_trackbars()

        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            'camera_lidar_manual_adjust_node started: '
            f'image={self.image_topic} scan={self.scan_topic} model={self.camera_model}'
        )

    def create_trackbars(self):
        for name in ('dx_mm', 'dy_mm', 'dz_mm'):
            cv2.createTrackbar(
                name,
                self.window_name,
                self.translation_slider_center,
                self.translation_slider_max,
                lambda value: None,
            )
        for name in ('roll_0.1deg', 'pitch_0.1deg', 'yaw_0.1deg'):
            cv2.createTrackbar(
                name,
                self.window_name,
                self.rotation_slider_center,
                self.rotation_slider_max,
                lambda value: None,
            )

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'image conversion failed: {exc}')
            return
        self.latest_image = image

    def scan_callback(self, msg):
        self.latest_scan = msg

    def slider_translation(self, name):
        value = cv2.getTrackbarPos(name, self.window_name)
        return (value - self.translation_slider_center) / self.translation_scale

    def slider_rotation_rad(self, name):
        value = cv2.getTrackbarPos(name, self.window_name)
        degrees = (value - self.rotation_slider_center) / self.rotation_scale
        return math.radians(degrees)

    def current_offsets(self):
        dx = self.slider_translation('dx_mm')
        dy = self.slider_translation('dy_mm')
        dz = self.slider_translation('dz_mm')
        roll = self.slider_rotation_rad('roll_0.1deg')
        pitch = self.slider_rotation_rad('pitch_0.1deg')
        yaw = self.slider_rotation_rad('yaw_0.1deg')
        return dx, dy, dz, roll, pitch, yaw

    def current_extrinsic(self):
        dx, dy, dz, roll, pitch, yaw = self.current_offsets()
        delta_rotation = _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)
        rotation = delta_rotation @ self.base_rotation
        translation = self.base_translation + np.array(
            [[dx], [dy], [dz]],
            dtype=np.float64,
        )
        return rotation, translation

    def reset_sliders(self):
        for name in ('dx_mm', 'dy_mm', 'dz_mm'):
            cv2.setTrackbarPos(name, self.window_name, self.translation_slider_center)
        for name in ('roll_0.1deg', 'pitch_0.1deg', 'yaw_0.1deg'):
            cv2.setTrackbarPos(name, self.window_name, self.rotation_slider_center)
        self.status = 'reset offsets to initial extrinsic'

    def active_image_and_scan(self):
        if self.freeze:
            return self.frozen_image, self.frozen_scan
        return self.latest_image, self.latest_scan

    def set_freeze(self):
        if self.freeze:
            self.freeze = False
            self.frozen_image = None
            self.frozen_scan = None
            self.status = 'live projection'
            return
        if self.latest_image is None or self.latest_scan is None:
            self.status = 'cannot freeze yet: waiting for image and scan'
            return
        self.frozen_image = self.latest_image.copy()
        self.frozen_scan = self.latest_scan
        self.freeze = True
        self.status = 'frozen frame; press f to resume live'

    def draw_projection(self, image, scan, rotation, translation):
        points = scan_to_lidar_points(
            scan,
            max_range_m=self.max_lidar_range_m,
            point_stride=self.point_stride,
        )
        if points.size == 0:
            return image, 0

        camera_points = (rotation @ points.T + translation).T
        depth_mask = camera_points[:, 2] > self.min_camera_depth_m
        points = points[depth_mask]
        camera_points = camera_points[depth_mask]
        if points.size == 0:
            return image, 0

        pixels = project_lidar_points(
            points,
            rotation,
            translation,
            self.camera_matrix,
            self.distortion,
            self.camera_model,
        )

        h, w = image.shape[:2]
        ranges = np.linalg.norm(camera_points[:, :2], axis=1)
        max_range = max(self.max_lidar_range_m, 0.1)
        drawn = 0
        for (u, v), range_m in zip(pixels, ranges):
            px = int(round(u))
            py = int(round(v))
            if 0 <= px < w and 0 <= py < h:
                alpha = min(max(range_m / max_range, 0.0), 1.0)
                color = (int(255 * alpha), int(255 * (1.0 - alpha)), 255)
                cv2.circle(image, (px, py), 2, color, -1)
                drawn += 1
        return image, drawn

    def draw_overlay_text(self, image, drawn_count):
        dx, dy, dz, roll, pitch, yaw = self.current_offsets()
        mode = 'frozen' if self.freeze else 'live'
        lines = [
            f'{mode} | projected={drawn_count} | f freeze, r reset, s save, q quit',
            (
                f'dxyz=({dx:+.3f}, {dy:+.3f}, {dz:+.3f}) m | '
                f'rpy=({math.degrees(roll):+.1f}, '
                f'{math.degrees(pitch):+.1f}, {math.degrees(yaw):+.1f}) deg'
            ),
            self.status,
        ]
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (image.shape[1], 78), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, image, 0.45, 0.0, image)
        for index, line in enumerate(lines):
            cv2.putText(
                image,
                line[:150],
                (10, 22 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (80, 230, 255) if index == 2 else (245, 245, 245),
                1,
                cv2.LINE_AA,
            )

    def render(self):
        image, scan = self.active_image_and_scan()
        if image is None:
            display = np.zeros((480, 640, 3), dtype=np.uint8)
            self.draw_overlay_text(display, 0)
            cv2.imshow(self.window_name, display)
            return
        display = image.copy()
        drawn = 0
        if scan is not None:
            rotation, translation = self.current_extrinsic()
            display, drawn = self.draw_projection(display, scan, rotation, translation)
            self.status = 'adjust sliders until projected points align'
        else:
            self.status = 'waiting for scan'
        self.draw_overlay_text(display, drawn)
        cv2.imshow(self.window_name, display)

    def save_current(self):
        image, scan = self.active_image_and_scan()
        if image is None or scan is None:
            self.status = 'cannot save yet: waiting for image and scan'
            return
        rotation, translation = self.current_extrinsic()
        output_dir = self.output_root / datetime.now().strftime('%Y%m%d_%H%M%S_manual')
        output_dir.mkdir(parents=True, exist_ok=True)
        preview, _ = self.draw_projection(image.copy(), scan, rotation, translation)
        cv2.imwrite(str(output_dir / 'manual_projection_preview.png'), preview)
        write_extrinsic_yaml(
            output_dir / 'lidar_to_camera_extrinsic.yaml',
            rotation,
            translation,
            self.camera_frame,
            self.lidar_frame,
        )
        self.status = f'saved manual extrinsic to {output_dir}'
        self.get_logger().info(self.status)

    def handle_key(self, key):
        if key in (ord('q'), 27):
            self.running = False
        elif key == ord('f'):
            self.set_freeze()
        elif key == ord('r'):
            self.reset_sliders()
        elif key == ord('s'):
            self.save_current()

    def run(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.render()
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                self.handle_key(key)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraLidarManualAdjustNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
