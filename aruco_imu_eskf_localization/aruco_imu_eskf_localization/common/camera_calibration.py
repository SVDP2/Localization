from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class CameraCalibration:
    camera_model: str
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    image_width: int
    image_height: int
    used_legacy_default: bool = False

    @property
    def image_size(self) -> tuple[int, int]:
        return self.image_width, self.image_height


def load_camera_calibration(path: str) -> CameraCalibration:
    with open(path, 'r', encoding='utf-8') as file_obj:
        calib = yaml.safe_load(file_obj)

    raw_camera_model = str(calib.get('camera_model', '')).strip().lower()
    used_legacy_default = False
    if not raw_camera_model:
        raw_camera_model = 'fisheye'
        used_legacy_default = True

    if raw_camera_model != 'fisheye':
        raise ValueError(f'unsupported camera_model {raw_camera_model!r}; expected fisheye')

    camera_matrix = np.asarray(
        calib['camera_matrix']['data'],
        dtype=np.float64,
    ).reshape(3, 3)
    distortion_coefficients = np.asarray(
        calib['distortion_coefficients']['data'],
        dtype=np.float64,
    ).reshape(-1, 1)
    if distortion_coefficients.shape[0] != 4:
        raise ValueError(
            'fisheye calibration requires exactly 4 distortion coefficients'
        )

    image_size = calib.get('image_size') or {}
    image_width = int(image_size['width'])
    image_height = int(image_size['height'])

    return CameraCalibration(
        camera_model=raw_camera_model,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        image_width=image_width,
        image_height=image_height,
        used_legacy_default=used_legacy_default,
    )


def build_fisheye_rectification(
    calibration: CameraCalibration,
    balance: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if calibration.camera_model != 'fisheye':
        raise ValueError(
            f'unsupported camera_model {calibration.camera_model!r}; expected fisheye'
        )

    image_size = calibration.image_size
    rectified_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        calibration.camera_matrix,
        calibration.distortion_coefficients,
        image_size,
        np.eye(3, dtype=np.float64),
        balance=float(np.clip(balance, 0.0, 1.0)),
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        calibration.camera_matrix,
        calibration.distortion_coefficients,
        np.eye(3, dtype=np.float64),
        rectified_camera_matrix,
        image_size,
        cv2.CV_32FC1,
    )
    zero_distortion = np.zeros((4, 1), dtype=np.float64)
    return rectified_camera_matrix, zero_distortion, map1, map2
