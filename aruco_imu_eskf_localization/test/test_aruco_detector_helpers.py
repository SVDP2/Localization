import unittest

import numpy as np

from aruco_imu_eskf_localization.estimation.board_pose_estimator import BoardPoseEstimate
from aruco_imu_eskf_localization.nodes.aruco_detector_node import (
    measurement_covariance_from_estimate,
    pose_prior_is_fresh,
    select_latest_pose_prior,
)


class TestArucoDetectorHelpers(unittest.TestCase):
    def test_prior_freshness_timeout(self):
        now_ns = 2_000_000_000
        self.assertTrue(pose_prior_is_fresh(1_900_000_000, now_ns, 0.2))
        self.assertFalse(pose_prior_is_fresh(1_700_000_000, now_ns, 0.2))
        self.assertFalse(pose_prior_is_fresh(None, now_ns, 0.2))

    def test_measurement_covariance_penalizes_depth_more_than_lateral(self):
        estimate = BoardPoseEstimate(
            rvec=np.zeros(3, dtype=float),
            tvec=np.array([0.0, 0.0, 1.0], dtype=float),
            visible_markers=3,
            reprojection_rmse_px=0.4,
            image_area_px=20000.0,
            used_single_marker_fallback=False,
        )

        covariance = measurement_covariance_from_estimate(estimate)

        self.assertGreater(covariance[2, 2], covariance[0, 0])

    def test_measurement_covariance_penalizes_single_marker_fallback(self):
        base_estimate = BoardPoseEstimate(
            rvec=np.zeros(3, dtype=float),
            tvec=np.array([0.0, 0.0, 1.0], dtype=float),
            visible_markers=1,
            reprojection_rmse_px=0.4,
            image_area_px=20000.0,
            used_single_marker_fallback=False,
        )
        fallback_estimate = BoardPoseEstimate(
            rvec=np.zeros(3, dtype=float),
            tvec=np.array([0.0, 0.0, 1.0], dtype=float),
            visible_markers=1,
            reprojection_rmse_px=0.4,
            image_area_px=20000.0,
            used_single_marker_fallback=True,
        )

        base_covariance = measurement_covariance_from_estimate(base_estimate)
        fallback_covariance = measurement_covariance_from_estimate(fallback_estimate)

        self.assertGreater(fallback_covariance[2, 2], base_covariance[2, 2])
        self.assertGreater(fallback_covariance[5, 5], base_covariance[5, 5])

    def test_select_latest_pose_prior_ignores_future_and_stale_samples(self):
        prior_samples = [
            (1_700_000_000, np.eye(4, dtype=float) * 1.0),
            (1_950_000_000, np.eye(4, dtype=float) * 2.0),
            (2_100_000_000, np.eye(4, dtype=float) * 3.0),
        ]

        selected_prior = select_latest_pose_prior(
            prior_samples,
            current_stamp_ns=2_000_000_000,
            timeout_sec=0.2,
        )

        self.assertIsNotNone(selected_prior)
        self.assertTrue(np.allclose(selected_prior, np.eye(4, dtype=float) * 2.0))


if __name__ == '__main__':
    unittest.main()
