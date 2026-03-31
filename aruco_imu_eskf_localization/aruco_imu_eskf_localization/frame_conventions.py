from __future__ import annotations

import numpy as np


def rotation_leader_rear_from_board() -> np.ndarray:
    """Return the fixed rotation that maps board-frame vectors into leader_rear."""
    return np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=float,
    )


def rotation_board_from_leader_rear() -> np.ndarray:
    return rotation_leader_rear_from_board().T


def transform_leader_rear_from_board() -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation_leader_rear_from_board()
    return transform


def transform_board_from_leader_rear() -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation_board_from_leader_rear()
    return transform


def vector_leader_rear_from_board(vector_in_board) -> np.ndarray:
    return rotation_leader_rear_from_board() @ np.asarray(vector_in_board, dtype=float).reshape(3)
