import tempfile
import unittest

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from aruco_imu_eskf_localization.common.camera_calibration import (
    build_fisheye_rectification,
    load_camera_calibration,
)


def _write_calibration_yaml(data: dict) -> str:
    temp_file = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    with temp_file:
        yaml.safe_dump(data, temp_file)
    return temp_file.name


class TestCameraCalibration(unittest.TestCase):
    def test_load_camera_calibration_defaults_missing_model_to_fisheye(self):
        calibration_path = _write_calibration_yaml(
            {
                'camera_matrix': {
                    'rows': 3,
                    'columns': 3,
                    'data': [
                        [320.0, 0.0, 320.0],
                        [0.0, 320.0, 240.0],
                        [0.0, 0.0, 1.0],
                    ],
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'columns': 4,
                    'data': [0.01, -0.02, 0.003, -0.001],
                },
                'image_size': {'width': 640, 'height': 480},
            }
        )

        calibration = load_camera_calibration(calibration_path)

        self.assertEqual('fisheye', calibration.camera_model)
        self.assertTrue(calibration.used_legacy_default)

    def test_load_camera_calibration_rejects_unsupported_model(self):
        calibration_path = _write_calibration_yaml(
            {
                'camera_model': 'pinhole',
                'camera_matrix': {
                    'rows': 3,
                    'columns': 3,
                    'data': [
                        [320.0, 0.0, 320.0],
                        [0.0, 320.0, 240.0],
                        [0.0, 0.0, 1.0],
                    ],
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'columns': 4,
                    'data': [0.01, -0.02, 0.003, -0.001],
                },
                'image_size': {'width': 640, 'height': 480},
            }
        )

        with self.assertRaisesRegex(ValueError, 'unsupported camera_model'):
            load_camera_calibration(calibration_path)

    def test_rectified_fisheye_points_support_zero_distortion_pnp(self):
        calibration_path = _write_calibration_yaml(
            {
                'camera_model': 'fisheye',
                'camera_matrix': {
                    'rows': 3,
                    'columns': 3,
                    'data': [
                        [290.0, 0.0, 320.0],
                        [0.0, 292.0, 240.0],
                        [0.0, 0.0, 1.0],
                    ],
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'columns': 4,
                    'data': [0.015, -0.005, 0.0015, -0.0007],
                },
                'image_size': {'width': 640, 'height': 480},
            }
        )
        calibration = load_camera_calibration(calibration_path)
        rectified_camera_matrix, zero_distortion, _, _ = build_fisheye_rectification(
            calibration,
            balance=0.0,
        )

        object_points = np.array(
            [
                [-0.1, 0.1, 0.0],
                [0.1, 0.1, 0.0],
                [0.1, -0.1, 0.0],
                [-0.1, -0.1, 0.0],
            ],
            dtype=np.float64,
        )
        true_rotation = Rotation.from_euler('ZYX', [8.0, -5.0, 2.0], degrees=True)
        true_rvec = true_rotation.as_rotvec().reshape(3, 1)
        true_tvec = np.array([[0.03], [0.02], [1.4]], dtype=np.float64)

        distorted_points, _ = cv2.fisheye.projectPoints(
            object_points.reshape(1, -1, 3),
            true_rvec,
            true_tvec,
            calibration.camera_matrix,
            calibration.distortion_coefficients,
        )
        rectified_points = cv2.fisheye.undistortPoints(
            distorted_points,
            calibration.camera_matrix,
            calibration.distortion_coefficients,
            P=rectified_camera_matrix,
        )

        success, estimated_rvec, estimated_tvec = cv2.solvePnP(
            object_points,
            rectified_points,
            rectified_camera_matrix,
            zero_distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        self.assertTrue(success)
        translation_error = np.linalg.norm(estimated_tvec.reshape(3) - true_tvec.reshape(3))
        self.assertLess(translation_error, 1.0e-2)

        estimated_rotation = Rotation.from_rotvec(estimated_rvec.reshape(3))
        rotation_error_deg = np.degrees(
            (estimated_rotation * true_rotation.inv()).magnitude()
        )
        self.assertLess(rotation_error_deg, 1.0)


if __name__ == '__main__':
    unittest.main()
