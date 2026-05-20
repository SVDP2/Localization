from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from camera_lidar_extrinsic_calibrator.calibration_utils import (
    load_camera_calibration,
    rotation_matrix_to_quaternion_xyzw,
    scan_to_lidar_points,
    solve_lidar_to_camera_pnp,
    write_extrinsic_yaml,
)


class FakeScan:
    angle_min = 0.0
    angle_increment = np.pi / 2.0
    range_min = 0.1
    range_max = 5.0
    ranges = [1.0, float('inf'), 2.0, 6.0]


def test_scan_to_lidar_points_filters_invalid_ranges():
    points = scan_to_lidar_points(FakeScan(), max_range_m=4.0)

    assert points.shape == (2, 3)
    np.testing.assert_allclose(points[0], [1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(points[1], [-2.0, 0.0, 0.0], atol=1e-9)


def test_load_fisheye_intrinsic_snapshot():
    root = Path(__file__).resolve().parents[2]
    path = (
        root
        / 'follower_camera_lidar_projection'
        / 'config'
        / 'follower_cam_intrinsic_20260407.yaml'
    )

    camera_matrix, distortion, image_size, camera_model = load_camera_calibration(path)

    assert camera_model == 'fisheye'
    assert image_size == (640, 480)
    assert camera_matrix.shape == (3, 3)
    assert distortion.shape == (4, 1)


def test_synthetic_fisheye_solve_pnp_with_initial_guess():
    camera_matrix = np.array(
        [
            [380.0, 0.0, 320.0],
            [0.0, 380.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.array([-0.01, 0.001, 0.0, 0.0], dtype=np.float64).reshape((4, 1))
    object_points = np.array(
        [
            [-0.8, -0.6, 0.0],
            [-0.2, -0.5, 0.0],
            [0.5, -0.4, 0.0],
            [-0.7, 0.1, 0.0],
            [0.1, 0.2, 0.0],
            [0.8, 0.3, 0.0],
            [-0.4, 0.7, 0.0],
            [0.4, 0.8, 0.0],
        ],
        dtype=np.float64,
    )
    true_rvec = np.array([0.18, -0.12, 0.07], dtype=np.float64).reshape((3, 1))
    true_translation = np.array([0.05, -0.04, 4.2], dtype=np.float64).reshape((3, 1))
    true_rotation, _ = cv2.Rodrigues(true_rvec)
    image_points, _ = cv2.fisheye.projectPoints(
        object_points.reshape((-1, 1, 3)),
        true_rvec,
        true_translation,
        camera_matrix,
        distortion,
    )
    correspondences = []
    for point, pixel in zip(object_points, image_points.reshape((-1, 2))):
        correspondences.append({
            'u': float(pixel[0]),
            'v': float(pixel[1]),
            'x': float(point[0]),
            'y': float(point[1]),
            'z': float(point[2]),
        })

    initial_rvec = true_rvec + np.array([0.01, -0.005, 0.004]).reshape((3, 1))
    initial_rotation, _ = cv2.Rodrigues(initial_rvec)
    initial_translation = true_translation + np.array([0.02, -0.01, 0.03]).reshape((3, 1))

    rotation, translation, _ = solve_lidar_to_camera_pnp(
        correspondences,
        camera_matrix,
        distortion,
        camera_model='fisheye',
        initial_rotation=initial_rotation,
        initial_translation=initial_translation,
    )

    np.testing.assert_allclose(rotation, true_rotation, atol=1e-5)
    np.testing.assert_allclose(translation, true_translation, atol=1e-5)


def test_output_yaml_quaternion_is_normalized(tmp_path):
    rotation = np.eye(3, dtype=np.float64)
    translation = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    path = tmp_path / 'lidar_to_camera_extrinsic.yaml'

    write_extrinsic_yaml(path, rotation, translation, 'follower/usb_cam', 'follower/lidar')

    with open(path, 'r') as handle:
        data = yaml.safe_load(handle)
    quat = data['quaternion_xyzw']
    norm = np.linalg.norm([quat['x'], quat['y'], quat['z'], quat['w']])

    assert data['parent_frame'] == 'follower/usb_cam'
    assert data['child_frame'] == 'follower/lidar'
    assert norm == pytest.approx(1.0)
    np.testing.assert_allclose(rotation_matrix_to_quaternion_xyzw(rotation), [0, 0, 0, 1])
