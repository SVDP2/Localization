import unittest

from aruco_imu_eskf_localization.aruco_detector_node import pose_prior_is_fresh


class TestArucoDetectorHelpers(unittest.TestCase):
    def test_prior_freshness_timeout(self):
        now_ns = 2_000_000_000
        self.assertTrue(pose_prior_is_fresh(1_900_000_000, now_ns, 0.2))
        self.assertFalse(pose_prior_is_fresh(1_700_000_000, now_ns, 0.2))
        self.assertFalse(pose_prior_is_fresh(None, now_ns, 0.2))


if __name__ == '__main__':
    unittest.main()
