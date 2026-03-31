import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from aruco_imu_eskf_localization.estimation.gyro_relative_eskf import (
    GyroRelativeEskf,
    transform_pose_covariance,
)


def _pose_matrix(position, euler_zyx_deg):
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = Rotation.from_euler('ZYX', euler_zyx_deg, degrees=True).as_matrix()
    pose[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return pose


class TestGyroRelativeEskf(unittest.TestCase):
    def test_transform_pose_covariance_rotates_translation_and_rotation_blocks(self):
        covariance = np.diag([1.0, 4.0, 9.0, 0.1, 0.2, 0.3]).astype(float)
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = Rotation.from_euler('Z', 90.0, degrees=True).as_matrix()

        transformed = transform_pose_covariance(covariance, transform)

        self.assertTrue(np.allclose(np.diag(transformed)[:3], [4.0, 1.0, 9.0], atol=1.0e-9))
        self.assertTrue(np.allclose(np.diag(transformed)[3:], [0.2, 0.1, 0.3], atol=1.0e-9))

    def test_predict_integrates_gyro_heading(self):
        filter_core = GyroRelativeEskf()
        filter_core.initialize(
            stamp_ns=0,
            pose_matrix=np.eye(4, dtype=float),
            measurement_covariance=np.diag([1.0e-3] * 6),
        )

        filter_core.predict(500_000_000, np.array([0.0, 0.0, 0.4], dtype=float))
        yaw_deg = Rotation.from_matrix(filter_core.pose_matrix()[:3, :3]).as_euler(
            'ZYX',
            degrees=True,
        )[0]

        self.assertAlmostEqual(yaw_deg, np.degrees(0.2), delta=0.75)

    def test_update_pose_skips_rotation_when_gate_is_exceeded(self):
        filter_core = GyroRelativeEskf()
        filter_core.initialize(
            stamp_ns=0,
            pose_matrix=np.eye(4, dtype=float),
            measurement_covariance=np.diag([1.0e-3] * 6),
        )
        measurement_pose = _pose_matrix([1.0, 0.0, 0.0], [70.0, 0.0, 0.0])
        measurement_covariance = np.diag([1.0e-3, 1.0e-3, 1.0e-3, 1.0e-4, 1.0e-4, 1.0e-4])

        result = filter_core.update_pose(
            measurement_pose,
            measurement_covariance,
            rotation_gate_deg=10.0,
        )
        yaw_deg = Rotation.from_matrix(filter_core.pose_matrix()[:3, :3]).as_euler(
            'ZYX',
            degrees=True,
        )[0]

        self.assertFalse(result.used_rotation_update)
        self.assertGreater(filter_core.pose_matrix()[0, 3], 0.49)
        self.assertAlmostEqual(yaw_deg, 0.0, delta=1.0)

    def test_pose_update_can_correct_gyro_bias(self):
        filter_core = GyroRelativeEskf(
            initial_orientation_std_deg=25.0,
            initial_gyro_bias_std_radps=0.30,
        )
        filter_core.initialize(
            stamp_ns=0,
            pose_matrix=np.eye(4, dtype=float),
            measurement_covariance=np.diag([1.0e-3] * 6),
        )
        filter_core.predict(1_000_000_000, np.array([0.0, 0.0, 0.2], dtype=float))

        result = filter_core.update_pose(
            np.eye(4, dtype=float),
            np.diag([1.0e-3, 1.0e-3, 1.0e-3, 2.5e-4, 2.5e-4, 2.5e-4]),
            rotation_gate_deg=45.0,
        )
        snapshot = filter_core.snapshot()

        self.assertTrue(result.used_rotation_update)
        self.assertGreater(snapshot.gyro_bias_radps[2], 0.01)


if __name__ == '__main__':
    unittest.main()
