import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from aruco_imu_eskf_localization.board_pose_estimator import BoardDefinition, invert_observation


def _make_board_definition() -> BoardDefinition:
    return BoardDefinition.from_config(
        {
            'markers': [
                {'id': 0, 'size': 124.0},
                {'id': 1, 'size': 124.0},
                {'id': 2, 'size': 62.0},
            ],
            'board_geometry': {
                'marker_positions': {
                    '2': [0.0, 0.0, 0.0],
                    '0': [0.0, 119.0, 93.0],
                    '1': [0.0, -119.0, 93.0],
                }
            },
        }
    )


def _camera_matrix():
    return np.array(
        [
            [640.0, 0.0, 320.0],
            [0.0, 640.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _true_observation():
    rotation = Rotation.from_matrix(
        np.array(
            [
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],
                [-1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
    )
    rvec = rotation.as_rotvec()
    tvec = np.array([0.02, 0.03, 1.15], dtype=np.float64)
    return rvec, tvec


def _project_marker(board: BoardDefinition, marker_id: int, rvec, tvec):
    corners, _ = cv2.projectPoints(
        board.marker_object_points(marker_id),
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        _camera_matrix(),
        np.zeros(5, dtype=np.float64),
    )
    return corners.reshape(1, 4, 2)


class TestBoardPoseEstimator(unittest.TestCase):
    def test_single_marker_requires_prior(self):
        board = _make_board_definition()
        true_rvec, true_tvec = _true_observation()
        corners = _project_marker(board, 2, true_rvec, true_tvec)
        ids = np.array([[2]], dtype=np.int32)

        estimate = board.estimate_pose(
            corners,
            ids,
            _camera_matrix(),
            np.zeros(5, dtype=np.float64),
            previous_board_pose=None,
            min_markers=1,
            min_markers_to_initialize=2,
        )

        self.assertIsNone(estimate)

    def test_single_marker_with_prior_returns_board_pose(self):
        board = _make_board_definition()
        true_rvec, true_tvec = _true_observation()
        true_board_pose = invert_observation(true_rvec, true_tvec)
        corners = _project_marker(board, 2, true_rvec, true_tvec)
        ids = np.array([[2]], dtype=np.int32)

        estimate = board.estimate_pose(
            corners,
            ids,
            _camera_matrix(),
            np.zeros(5, dtype=np.float64),
            previous_board_pose=true_board_pose,
            min_markers=1,
            min_markers_to_initialize=2,
        )

        self.assertIsNotNone(estimate)
        self.assertTrue(estimate.used_single_marker_fallback)
        self.assertEqual(1, estimate.visible_markers)

        estimated_board_rvec, estimated_board_tvec = estimate.board_pose
        self.assertLess(np.linalg.norm(estimated_board_tvec - true_board_pose[1]), 1.0e-3)

        estimated_rotation = Rotation.from_rotvec(estimated_board_rvec)
        true_rotation = Rotation.from_rotvec(true_board_pose[0])
        rotation_error_deg = np.degrees((estimated_rotation * true_rotation.inv()).magnitude())
        self.assertLess(rotation_error_deg, 0.5)

    def test_two_markers_initialize_without_prior(self):
        board = _make_board_definition()
        true_rvec, true_tvec = _true_observation()
        corners = np.concatenate(
            [
                _project_marker(board, 0, true_rvec, true_tvec),
                _project_marker(board, 2, true_rvec, true_tvec),
            ],
            axis=0,
        )
        ids = np.array([[0], [2]], dtype=np.int32)

        estimate = board.estimate_pose(
            corners,
            ids,
            _camera_matrix(),
            np.zeros(5, dtype=np.float64),
            previous_board_pose=None,
            min_markers=1,
            min_markers_to_initialize=2,
        )

        self.assertIsNotNone(estimate)
        self.assertFalse(estimate.used_single_marker_fallback)
        self.assertEqual(2, estimate.visible_markers)


if __name__ == '__main__':
    unittest.main()
