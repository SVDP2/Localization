import csv
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def _flatten_yaml_matrix(value):
    data = value.get('data', value) if isinstance(value, dict) else value
    return np.array(data, dtype=np.float64)


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


def scan_to_lidar_points(scan, max_range_m=None, point_stride=1):
    points = []
    angle = float(scan.angle_min)
    stride = max(1, int(point_stride))
    range_max = float(scan.range_max)
    if max_range_m is not None:
        range_max = min(range_max, float(max_range_m))

    for index, distance in enumerate(scan.ranges):
        if index % stride == 0 and math.isfinite(distance):
            if float(scan.range_min) <= distance <= range_max:
                points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
        angle += float(scan.angle_increment)

    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.array(points, dtype=np.float64)


def undistort_fisheye_pixels(image_points, camera_matrix, distortion):
    pixels = np.asarray(image_points, dtype=np.float64).reshape((-1, 1, 2))
    normalized = cv2.fisheye.undistortPoints(
        pixels,
        np.asarray(camera_matrix, dtype=np.float64),
        np.asarray(distortion, dtype=np.float64).reshape((-1, 1))[:4],
    )
    return normalized.reshape((-1, 2))


def solve_lidar_to_camera_pnp(
    correspondences,
    camera_matrix,
    distortion,
    camera_model='fisheye',
    initial_rotation=None,
    initial_translation=None,
):
    if len(correspondences) < 6:
        raise ValueError('at least 6 correspondences are required for solvePnP')

    object_points = np.array(
        [[item['x'], item['y'], item.get('z', 0.0)] for item in correspondences],
        dtype=np.float64,
    ).reshape((-1, 1, 3))
    image_points = np.array(
        [[item['u'], item['v']] for item in correspondences],
        dtype=np.float64,
    )

    if str(camera_model).lower() == 'fisheye':
        normalized_points = undistort_fisheye_pixels(
            image_points, camera_matrix, distortion
        ).reshape((-1, 1, 2))
        solve_camera_matrix = np.eye(3, dtype=np.float64)
        solve_distortion = None
    else:
        normalized_points = image_points.reshape((-1, 1, 2))
        solve_camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        solve_distortion = np.asarray(distortion, dtype=np.float64)

    use_guess = initial_rotation is not None and initial_translation is not None
    if use_guess:
        rvec, _ = cv2.Rodrigues(np.asarray(initial_rotation, dtype=np.float64))
        tvec = np.asarray(initial_translation, dtype=np.float64).reshape((3, 1)).copy()
    else:
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        normalized_points,
        solve_camera_matrix,
        solve_distortion,
        rvec,
        tvec,
        useExtrinsicGuess=use_guess,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError('cv2.solvePnP failed')

    rotation, _ = cv2.Rodrigues(rvec)
    return rotation, tvec.reshape((3, 1)), rvec.reshape((3, 1))


def project_lidar_points(points, rotation, translation, camera_matrix, distortion, camera_model):
    points = np.asarray(points, dtype=np.float64).reshape((-1, 1, 3))
    rvec, _ = cv2.Rodrigues(np.asarray(rotation, dtype=np.float64))
    tvec = np.asarray(translation, dtype=np.float64).reshape((3, 1))

    if str(camera_model).lower() == 'fisheye':
        image_points, _ = cv2.fisheye.projectPoints(
            points,
            rvec,
            tvec,
            np.asarray(camera_matrix, dtype=np.float64),
            np.asarray(distortion, dtype=np.float64).reshape((-1, 1))[:4],
        )
    else:
        image_points, _ = cv2.projectPoints(
            points,
            rvec,
            tvec,
            np.asarray(camera_matrix, dtype=np.float64),
            np.asarray(distortion, dtype=np.float64),
        )
    return image_points.reshape((-1, 2))


def rotation_matrix_to_quaternion_xyzw(rotation):
    matrix = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError('rotation matrix produced a zero quaternion')
    return quaternion / norm


def write_extrinsic_yaml(path, rotation, translation, parent_frame, child_frame):
    path = Path(path)
    quaternion = rotation_matrix_to_quaternion_xyzw(rotation)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    rotation = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    data = {
        'parent_frame': parent_frame,
        'child_frame': child_frame,
        'translation': {
            'x': float(translation[0]),
            'y': float(translation[1]),
            'z': float(translation[2]),
        },
        'rotation_matrix': {
            'rows': 3,
            'columns': 3,
            'data': rotation.tolist(),
        },
        'quaternion_xyzw': {
            'x': float(quaternion[0]),
            'y': float(quaternion[1]),
            'z': float(quaternion[2]),
            'w': float(quaternion[3]),
        },
    }
    with open(path, 'w') as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    return data


def write_correspondences_csv(path, correspondences):
    fieldnames = ['index', 'u', 'v', 'x', 'y', 'z']
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, item in enumerate(correspondences):
            writer.writerow({
                'index': index,
                'u': item['u'],
                'v': item['v'],
                'x': item['x'],
                'y': item['y'],
                'z': item.get('z', 0.0),
            })
