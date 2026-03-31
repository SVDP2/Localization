import unittest

import numpy as np

from aruco_imu_eskf_localization.filters.pose_filter import PoseFilter


class TestPoseFilter(unittest.TestCase):
    def test_smoothing_reduces_position_variance(self):
        pose_filter = PoseFilter(dt=0.1)
        true_pos = np.array([1.0, 2.0, 3.0])
        true_rot = np.array([0.1, 0.2, 0.3])

        np.random.seed(42)
        measurements = []
        filtered_positions = []

        for i in range(100):
            measurement = true_pos + np.random.normal(0.0, 0.1, 3)
            measurements.append(measurement)

            if i > 0:
                pose_filter.predict(0.1)

            pose_filter.update(measurement, true_rot)
            _, estimate = pose_filter.get_pose()
            filtered_positions.append(estimate)

        measurements = np.array(measurements)
        filtered_positions = np.array(filtered_positions)

        measurement_var = np.var(measurements[10:], axis=0).mean()
        filtered_var = np.var(filtered_positions[10:], axis=0).mean()

        self.assertLess(filtered_var, measurement_var)

    def test_reset_clears_state(self):
        pose_filter = PoseFilter(dt=0.1)
        pose_filter.update([1.0, 2.0, 3.0], [0.0, 0.0, 0.1])

        self.assertTrue(pose_filter.initialized)

        pose_filter.reset()

        self.assertFalse(pose_filter.initialized)
        self.assertEqual((None, None), pose_filter.get_pose())


if __name__ == '__main__':
    unittest.main()
