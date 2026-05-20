#!/usr/bin/env python3

from collections import deque
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
    solve_lidar_to_camera_pnp,
    write_correspondences_csv,
    write_extrinsic_yaml,
)


def _as_bool(value):
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class CameraLidarExtrinsicCalibratorNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_extrinsic_calibrator_node')

        self.declare_parameter('image_topic', '/follower/image_raw')
        self.declare_parameter('scan_topic', '/follower/scan')
        self.declare_parameter('camera_frame', 'follower/usb_cam')
        self.declare_parameter('lidar_frame', 'follower/lidar')
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('initial_extrinsic_file', '')
        self.declare_parameter('output_root', '/home/xytron/follower_ws/calibration/camera_lidar')
        self.declare_parameter('max_pair_dt_sec', 0.08)
        self.declare_parameter('allow_stale_pair', False)
        self.declare_parameter('image_buffer_size', 60)
        self.declare_parameter('scan_buffer_size', 60)
        self.declare_parameter('max_lidar_range_m', 5.0)
        self.declare_parameter('lidar_view_range_m', 5.0)
        self.declare_parameter('lidar_display_rotation_deg', 230.0)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('min_correspondences', 6)
        self.declare_parameter('nearest_point_max_px', 14)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.output_root = Path(str(self.get_parameter('output_root').value))
        self.max_pair_dt_sec = float(self.get_parameter('max_pair_dt_sec').value)
        self.allow_stale_pair = _as_bool(
            self.get_parameter('allow_stale_pair').value
        )
        self.max_lidar_range_m = float(self.get_parameter('max_lidar_range_m').value)
        self.lidar_view_range_m = float(self.get_parameter('lidar_view_range_m').value)
        self.lidar_display_rotation_rad = math.radians(
            float(self.get_parameter('lidar_display_rotation_deg').value)
        )
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.min_correspondences = max(4, int(self.get_parameter('min_correspondences').value))
        self.nearest_point_max_px = float(self.get_parameter('nearest_point_max_px').value)

        camera_calibration_file = str(
            self.get_parameter('camera_calibration_file').value
        )
        if not camera_calibration_file:
            raise ValueError('camera_calibration_file parameter is required')
        self.camera_matrix, self.distortion, self.image_size, self.camera_model = (
            load_camera_calibration(camera_calibration_file)
        )

        self.initial_rotation = None
        self.initial_translation = None
        initial_extrinsic_file = str(self.get_parameter('initial_extrinsic_file').value)
        if initial_extrinsic_file:
            self.initial_rotation, self.initial_translation = (
                load_lidar_to_camera_extrinsic(initial_extrinsic_file)
            )

        self.current_rotation = self.initial_rotation
        self.current_translation = self.initial_translation

        image_buffer_size = max(2, int(self.get_parameter('image_buffer_size').value))
        scan_buffer_size = max(2, int(self.get_parameter('scan_buffer_size').value))
        self.image_buffer = deque(maxlen=image_buffer_size)
        self.scan_buffer = deque(maxlen=scan_buffer_size)

        self.bridge = CvBridge()
        self.window_name = 'camera_lidar_extrinsic_calibrator'
        self.status = 'waiting for image and scan; press c to capture'
        self.running = True
        self.frozen = False
        self.frozen_image = None
        self.frozen_scan = None
        self.frozen_scan_points = np.empty((0, 3), dtype=np.float64)
        self.frozen_pair_dt = None
        self.pending_image_point = None
        self.pending_lidar_point = None
        self.correspondences = []
        self.session_dir = None
        self.preview_image = None
        self.latest_lidar_pixels = []
        self.latest_lidar_points_for_pixels = np.empty((0, 3), dtype=np.float64)
        self.latest_left_width = 640

        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.get_logger().info(
            'camera_lidar_extrinsic_calibrator_node started: '
            f'image={self.image_topic} scan={self.scan_topic} model={self.camera_model}'
        )

    def stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'image conversion failed: {exc}')
            return
        self.image_buffer.append({
            'stamp': self.stamp_to_sec(msg.header.stamp),
            'msg': msg,
            'image': image,
        })

    def scan_callback(self, msg):
        self.scan_buffer.append({
            'stamp': self.stamp_to_sec(msg.header.stamp),
            'msg': msg,
        })

    def capture_pair(self):
        if not self.image_buffer or not self.scan_buffer:
            self.status = 'cannot capture yet: waiting for image and scan'
            return

        latest_scan = self.scan_buffer[-1]
        best_image = min(
            self.image_buffer,
            key=lambda item: abs(item['stamp'] - latest_scan['stamp']),
        )
        pair_dt = abs(best_image['stamp'] - latest_scan['stamp'])
        if pair_dt > self.max_pair_dt_sec and not self.allow_stale_pair:
            self.status = (
                f'pair rejected: dt={pair_dt:.3f}s > {self.max_pair_dt_sec:.3f}s '
                '(set allow_stale_pair:=true to override)'
            )
            return

        self.frozen = True
        self.frozen_image = best_image['image'].copy()
        self.frozen_scan = latest_scan['msg']
        self.frozen_scan_points = scan_to_lidar_points(
            self.frozen_scan,
            max_range_m=self.max_lidar_range_m,
            point_stride=self.point_stride,
        )
        self.frozen_pair_dt = pair_dt
        self.pending_image_point = None
        self.pending_lidar_point = None
        self.correspondences = []
        self.preview_image = None
        self.current_rotation = self.initial_rotation
        self.current_translation = self.initial_translation
        stale_note = ' stale allowed' if pair_dt > self.max_pair_dt_sec else ''
        self.status = (
            f'captured dt={pair_dt:.3f}s points={len(self.frozen_scan_points)}; '
            f'click image and LiDAR points{stale_note}'
        )

    def resume_live(self):
        self.frozen = False
        self.preview_image = None
        self.pending_image_point = None
        self.pending_lidar_point = None
        self.status = 'live mode; press c to freeze a synchronized pair'

    def undo_last(self):
        if self.correspondences:
            self.correspondences.pop()
            self.status = f'undid last correspondence; count={len(self.correspondences)}'
        else:
            self.pending_image_point = None
            self.pending_lidar_point = None
            self.status = 'nothing to undo'

    def append_correspondence_if_ready(self):
        if self.pending_image_point is None or self.pending_lidar_point is None:
            return
        u, v = self.pending_image_point
        x, y, z = self.pending_lidar_point
        self.correspondences.append({
            'u': float(u),
            'v': float(v),
            'x': float(x),
            'y': float(y),
            'z': float(z),
        })
        self.pending_image_point = None
        self.pending_lidar_point = None
        self.status = f'added correspondence #{len(self.correspondences)}'

    def on_mouse(self, event, x, y, flags, userdata):
        del flags, userdata
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if not self.frozen:
            self.status = 'press c to freeze a pair before selecting points'
            return

        if x < self.latest_left_width:
            image = self.get_display_image()
            h, w = image.shape[:2]
            if 0 <= x < w and 0 <= y < h:
                self.pending_image_point = (float(x), float(y))
                self.status = f'image point selected: ({x}, {y})'
                self.append_correspondence_if_ready()
            return

        lidar_x = x - self.latest_left_width
        if len(self.latest_lidar_pixels) == 0:
            self.status = 'no LiDAR points available in current capture'
            return

        pixels = np.asarray(self.latest_lidar_pixels, dtype=np.float64)
        distances = np.linalg.norm(pixels - np.array([lidar_x, y], dtype=np.float64), axis=1)
        nearest_index = int(np.argmin(distances))
        if distances[nearest_index] > self.nearest_point_max_px:
            self.status = f'no LiDAR point within {self.nearest_point_max_px:.0f}px'
            return

        point = self.latest_lidar_points_for_pixels[nearest_index]
        self.pending_lidar_point = tuple(float(value) for value in point)
        self.status = (
            'LiDAR point selected: '
            f'x={point[0]:.3f} y={point[1]:.3f} z={point[2]:.3f}'
        )
        self.append_correspondence_if_ready()

    def get_display_image(self):
        if self.preview_image is not None:
            return self.preview_image.copy()
        if self.frozen and self.frozen_image is not None:
            return self.frozen_image.copy()
        if self.image_buffer:
            return self.image_buffer[-1]['image'].copy()
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def get_display_scan_points(self):
        if self.frozen:
            return self.frozen_scan_points
        if self.scan_buffer:
            return scan_to_lidar_points(
                self.scan_buffer[-1]['msg'],
                max_range_m=self.max_lidar_range_m,
                point_stride=self.point_stride,
            )
        return np.empty((0, 3), dtype=np.float64)

    def rotate_lidar_display_xy(self, x, y):
        cos_theta = math.cos(self.lidar_display_rotation_rad)
        sin_theta = math.sin(self.lidar_display_rotation_rad)
        return (
            cos_theta * x - sin_theta * y,
            sin_theta * x + cos_theta * y,
        )

    def lidar_point_to_display_pixel(self, point, center, scale):
        display_x, display_y = self.rotate_lidar_display_xy(point[0], point[1])
        return (
            int(round(center[0] - display_y * scale)),
            int(round(center[1] - display_x * scale)),
        )

    def render_scan_view(self, points, height, width):
        image = np.full((height, width, 3), 28, dtype=np.uint8)
        center = np.array([width * 0.5, height * 0.88], dtype=np.float64)
        scale = min(width / (2.0 * self.lidar_view_range_m), height / self.lidar_view_range_m)

        for meter in range(1, int(self.lidar_view_range_m) + 1):
            radius = int(round(meter * scale))
            cv2.circle(image, tuple(center.astype(int)), radius, (55, 55, 55), 1)
        cv2.line(
            image,
            (int(center[0]), int(center[1])),
            (int(center[0]), max(0, int(center[1] - self.lidar_view_range_m * scale))),
            (80, 120, 80),
            1,
        )
        cv2.line(
            image,
            (0, int(center[1])),
            (width - 1, int(center[1])),
            (80, 80, 120),
            1,
        )

        pixels = []
        visible_points = []
        for point in points:
            px, py = self.lidar_point_to_display_pixel(point, center, scale)
            if 0 <= px < width and 0 <= py < height:
                pixels.append((px, py))
                visible_points.append(point)
                cv2.circle(image, (px, py), 2, (170, 220, 255), -1)

        for item in self.correspondences:
            px, py = self.lidar_point_to_display_pixel(
                (item['x'], item['y'], item.get('z', 0.0)),
                center,
                scale,
            )
            if 0 <= px < width and 0 <= py < height:
                cv2.circle(image, (px, py), 6, (0, 220, 255), 2)

        if self.pending_lidar_point is not None:
            px, py = self.lidar_point_to_display_pixel(
                self.pending_lidar_point,
                center,
                scale,
            )
            if 0 <= px < width and 0 <= py < height:
                cv2.circle(image, (px, py), 8, (0, 255, 0), 2)

        self.latest_lidar_pixels = pixels
        if visible_points:
            self.latest_lidar_points_for_pixels = np.array(visible_points, dtype=np.float64)
        else:
            self.latest_lidar_points_for_pixels = np.empty((0, 3), dtype=np.float64)
        return image

    def draw_image_overlays(self, image):
        for index, item in enumerate(self.correspondences, start=1):
            center = (int(round(item['u'])), int(round(item['v'])))
            cv2.circle(image, center, 5, (0, 220, 255), 2)
            cv2.putText(
                image,
                str(index),
                (center[0] + 6, center[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 220, 255),
                1,
                cv2.LINE_AA,
            )
        if self.pending_image_point is not None:
            center = (
                int(round(self.pending_image_point[0])),
                int(round(self.pending_image_point[1])),
            )
            cv2.circle(image, center, 7, (0, 255, 0), 2)
        return image

    def draw_text_bar(self, canvas):
        bar_height = 54
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (canvas.shape[1], bar_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0.0, canvas)
        mode = 'frozen' if self.frozen else 'live'
        pair_text = ''
        if self.frozen_pair_dt is not None and self.frozen:
            pair_text = f' dt={self.frozen_pair_dt:.3f}s'
        header = (
            f'{mode}{pair_text} | pairs={len(self.correspondences)} | '
            'c capture, u undo, s solve/save, p preview, r live, q quit'
        )
        cv2.putText(
            canvas,
            header,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            self.status[:150],
            (10, 43),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (80, 230, 255),
            1,
            cv2.LINE_AA,
        )

    def render(self):
        image = self.draw_image_overlays(self.get_display_image())
        h, w = image.shape[:2]
        self.latest_left_width = w
        scan_points = self.get_display_scan_points()
        scan_view = self.render_scan_view(scan_points, h, w)
        canvas = np.hstack([image, scan_view])
        cv2.line(canvas, (w, 0), (w, h - 1), (180, 180, 180), 1)
        self.draw_text_bar(canvas)
        cv2.imshow(self.window_name, canvas)

    def make_preview_image(self, save_path=None):
        if self.current_rotation is None or self.current_translation is None:
            self.status = 'no extrinsic available for preview'
            return None
        if self.frozen_image is None or self.frozen_scan_points.size == 0:
            self.status = 'capture a pair before previewing projection'
            return None

        preview = self.frozen_image.copy()
        pixels = project_lidar_points(
            self.frozen_scan_points,
            self.current_rotation,
            self.current_translation,
            self.camera_matrix,
            self.distortion,
            self.camera_model,
        )
        h, w = preview.shape[:2]
        for u, v in pixels:
            px = int(round(u))
            py = int(round(v))
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(preview, (px, py), 2, (0, 0, 255), -1)

        if save_path is not None:
            cv2.imwrite(str(save_path), preview)
        self.preview_image = preview
        return preview

    def ensure_session_dir(self):
        if self.session_dir is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.session_dir = self.output_root / timestamp
        self.session_dir.mkdir(parents=True, exist_ok=True)
        return self.session_dir

    def solve_and_save(self):
        if not self.frozen or self.frozen_image is None:
            self.status = 'capture a synchronized pair before solving'
            return
        if len(self.correspondences) < self.min_correspondences:
            self.status = (
                f'need at least {self.min_correspondences} correspondences; '
                f'currently {len(self.correspondences)}'
            )
            return

        try:
            rotation, translation, _ = solve_lidar_to_camera_pnp(
                self.correspondences,
                self.camera_matrix,
                self.distortion,
                self.camera_model,
                self.current_rotation,
                self.current_translation,
            )
        except Exception as exc:
            self.status = f'solvePnP failed: {exc}'
            self.get_logger().error(self.status)
            return

        self.current_rotation = rotation
        self.current_translation = translation

        try:
            output_dir = self.ensure_session_dir()
            write_correspondences_csv(output_dir / 'correspondences.csv', self.correspondences)
            cv2.imwrite(str(output_dir / 'capture_image.png'), self.frozen_image)
            scan_debug = self.render_scan_view(
                self.frozen_scan_points,
                self.frozen_image.shape[0],
                self.frozen_image.shape[1],
            )
            cv2.imwrite(str(output_dir / 'capture_scan_debug.png'), scan_debug)
            write_extrinsic_yaml(
                output_dir / 'lidar_to_camera_extrinsic.yaml',
                rotation,
                translation,
                self.camera_frame,
                self.lidar_frame,
            )
            self.make_preview_image(output_dir / 'projection_preview.png')
        except Exception as exc:
            self.status = f'solve succeeded but saving failed: {exc}'
            self.get_logger().error(self.status)
            return

        self.status = f'solved and saved calibration to {output_dir}'
        self.get_logger().info(self.status)

    def handle_key(self, key):
        if key in (ord('q'), 27):
            self.running = False
        elif key == ord('c'):
            self.capture_pair()
        elif key == ord('r'):
            self.resume_live()
        elif key == ord('u'):
            self.undo_last()
        elif key == ord('s'):
            self.solve_and_save()
        elif key == ord('p'):
            preview = self.make_preview_image()
            if preview is not None:
                self.status = 'projection preview updated'

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
        node = CameraLidarExtrinsicCalibratorNode()
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
