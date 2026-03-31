import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from aruco_imu_eskf_localization.board_pose_estimator import BoardDefinition, invert_observation
from aruco_imu_eskf_localization.frame_conventions import vector_leader_rear_from_board


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
                    '0': [-119.0, 93.0, 0.0],
                    '1': [119.0, 93.0, 0.0],
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
    rotation = Rotation.from_euler('ZYX', [4.0, -6.0, 2.0], degrees=True) * Rotation.from_euler(
        'X',
        180.0,
        degrees=True,
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
    def test_marker_points_are_planar_in_board_frame(self):
        board = _make_board_definition()
        object_points = board.marker_object_points(0)
        self.assertTrue(np.allclose(object_points[:, 2], 0.0))
        self.assertTrue(np.allclose(object_points[0], [-0.181, 0.155, 0.0]))
        self.assertTrue(np.allclose(object_points[2], [-0.057, 0.031, 0.0]))

    def test_board_vector_maps_to_leader_rear_axes(self):
        vector_in_board = np.array([0.25, 0.40, 1.20], dtype=float)
        vector_in_leader_rear = vector_leader_rear_from_board(vector_in_board)
        self.assertTrue(np.allclose(vector_in_leader_rear, [1.20, -0.25, 0.40]))

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

    def test_front_halfspace_rejects_pose_behind_board(self):
        board = _make_board_definition()
        true_rvec, true_tvec = _true_observation()
        corners = np.concatenate(
            [
                _project_marker(board, 1, true_rvec, true_tvec),
                _project_marker(board, 2, true_rvec, true_tvec),
            ],
            axis=0,
        )
        ids = np.array([[1], [2]], dtype=np.int32)

        estimate = board.estimate_pose(
            corners,
            ids,
            _camera_matrix(),
            np.zeros(5, dtype=np.float64),
            previous_board_pose=None,
            min_markers=1,
            min_markers_to_initialize=2,
            front_halfspace_min_z_m=2.0,
        )

        self.assertIsNone(estimate)


if __name__ == '__main__':
    unittest.main()
