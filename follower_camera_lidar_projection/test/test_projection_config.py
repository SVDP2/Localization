from pathlib import Path

from follower_camera_lidar_projection.camera_lidar_projection_node import (
    load_lidar_to_camera_extrinsic,
)


def test_provisional_extrinsic_shape():
    root = Path(__file__).resolve().parents[1]
    path = root / 'config' / 'provisional_lidar_to_camera_extrinsic.yaml'
    rotation, translation = load_lidar_to_camera_extrinsic(path)

    assert rotation.shape == (3, 3)
    assert translation.shape == (3, 1)
