import unittest

import numpy as np

from aruco_imu_eskf_localization.frame_conventions import (
    transform_board_from_leader_rear,
    transform_leader_rear_from_board,
    vector_leader_rear_from_board,
)


class TestFrameConventions(unittest.TestCase):
    def test_board_and_leader_rear_transforms_are_inverses(self):
        leader_rear_from_board = transform_leader_rear_from_board()
        board_from_leader_rear = transform_board_from_leader_rear()
        self.assertTrue(
            np.allclose(
                leader_rear_from_board @ board_from_leader_rear,
                np.eye(4, dtype=float),
            )
        )
        self.assertTrue(
            np.allclose(
                board_from_leader_rear @ leader_rear_from_board,
                np.eye(4, dtype=float),
            )
        )

    def test_board_translation_maps_to_gap_lateral_height(self):
        board_translation = np.array([0.25, 0.40, 1.20], dtype=float)
        leader_rear_translation = vector_leader_rear_from_board(board_translation)
        self.assertTrue(np.allclose(leader_rear_translation, [1.20, -0.25, 0.40]))


if __name__ == '__main__':
    unittest.main()
