from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def skew_symmetric(vector) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def adjoint_from_transform(transform) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    adjoint = np.zeros((6, 6), dtype=float)
    adjoint[:3, :3] = rotation
    adjoint[:3, 3:] = skew_symmetric(translation) @ rotation
    adjoint[3:, 3:] = rotation
    return adjoint


def transform_pose_covariance(covariance, left_transform) -> np.ndarray:
    covariance = np.asarray(covariance, dtype=float).reshape(6, 6)
    adjoint = adjoint_from_transform(left_transform)
    return adjoint @ covariance @ adjoint.T


@dataclass(frozen=True)
class GyroRelativeEskfSnapshot:
    stamp_ns: int | None
    position_m: np.ndarray
    velocity_mps: np.ndarray
    rotation_quat_xyzw: np.ndarray
    gyro_bias_radps: np.ndarray
    covariance: np.ndarray
    last_angular_velocity_base: np.ndarray
    initialized: bool


@dataclass(frozen=True)
class GyroRelativeEskfUpdateResult:
    accepted_pose_update: bool
    used_rotation_update: bool
    position_innovation_m: float
    rotation_innovation_deg: float


class GyroRelativeEskf:
    def __init__(
        self,
        velocity_damping_per_sec: float = 1.5,
        position_process_noise_std_mps2: float = 0.3,
        gyro_noise_std_radps: float = 0.05,
        gyro_bias_random_walk_std_radps2: float = 0.01,
        initial_position_std_m: float = 0.20,
        initial_velocity_std_mps: float = 0.75,
        initial_orientation_std_deg: float = 20.0,
        initial_gyro_bias_std_radps: float = 0.15,
    ) -> None:
        self._velocity_damping_per_sec = float(max(velocity_damping_per_sec, 0.0))
        self._position_process_noise_std_mps2 = float(
            max(position_process_noise_std_mps2, 1.0e-4)
        )
        self._gyro_noise_std_radps = float(max(gyro_noise_std_radps, 1.0e-5))
        self._gyro_bias_random_walk_std_radps2 = float(
            max(gyro_bias_random_walk_std_radps2, 1.0e-6)
        )
        self._initial_position_std_m = float(max(initial_position_std_m, 1.0e-3))
        self._initial_velocity_std_mps = float(max(initial_velocity_std_mps, 1.0e-3))
        self._initial_orientation_std_rad = float(
            np.deg2rad(max(initial_orientation_std_deg, 1.0e-3))
        )
        self._initial_gyro_bias_std_radps = float(max(initial_gyro_bias_std_radps, 1.0e-4))

        self.reset()

    def reset(self) -> None:
        self._stamp_ns: int | None = None
        self._position_m = np.zeros(3, dtype=float)
        self._velocity_mps = np.zeros(3, dtype=float)
        self._rotation = Rotation.identity()
        self._gyro_bias_radps = np.zeros(3, dtype=float)
        self._covariance = self._initial_covariance()
        self._last_angular_velocity_base = np.zeros(3, dtype=float)
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def stamp_ns(self) -> int | None:
        return self._stamp_ns

    def _initial_covariance(self) -> np.ndarray:
        covariance = np.zeros((12, 12), dtype=float)
        covariance[0:3, 0:3] = (self._initial_position_std_m**2) * np.eye(3, dtype=float)
        covariance[3:6, 3:6] = (self._initial_velocity_std_mps**2) * np.eye(3, dtype=float)
        covariance[6:9, 6:9] = (self._initial_orientation_std_rad**2) * np.eye(3, dtype=float)
        covariance[9:12, 9:12] = (
            self._initial_gyro_bias_std_radps**2
        ) * np.eye(3, dtype=float)
        return covariance

    def snapshot(self) -> GyroRelativeEskfSnapshot:
        return GyroRelativeEskfSnapshot(
            stamp_ns=self._stamp_ns,
            position_m=self._position_m.copy(),
            velocity_mps=self._velocity_mps.copy(),
            rotation_quat_xyzw=self._rotation.as_quat().copy(),
            gyro_bias_radps=self._gyro_bias_radps.copy(),
            covariance=self._covariance.copy(),
            last_angular_velocity_base=self._last_angular_velocity_base.copy(),
            initialized=self._initialized,
        )

    def restore(self, snapshot: GyroRelativeEskfSnapshot) -> None:
        self._stamp_ns = snapshot.stamp_ns
        self._position_m = snapshot.position_m.copy()
        self._velocity_mps = snapshot.velocity_mps.copy()
        self._rotation = Rotation.from_quat(snapshot.rotation_quat_xyzw)
        self._gyro_bias_radps = snapshot.gyro_bias_radps.copy()
        self._covariance = snapshot.covariance.copy()
        self._last_angular_velocity_base = snapshot.last_angular_velocity_base.copy()
        self._initialized = bool(snapshot.initialized)

    def initialize(
        self,
        stamp_ns: int,
        pose_matrix,
        measurement_covariance=None,
        gyro_bias_radps=None,
    ) -> None:
        pose_matrix = np.asarray(pose_matrix, dtype=float).reshape(4, 4)
        self._stamp_ns = int(stamp_ns)
        self._position_m = pose_matrix[:3, 3].copy()
        self._velocity_mps = np.zeros(3, dtype=float)
        self._rotation = Rotation.from_matrix(pose_matrix[:3, :3])
        self._gyro_bias_radps = (
            np.asarray(gyro_bias_radps, dtype=float).reshape(3)
            if gyro_bias_radps is not None
            else np.zeros(3, dtype=float)
        )
        self._covariance = self._initial_covariance()
        if measurement_covariance is not None:
            measurement_covariance = np.asarray(measurement_covariance, dtype=float).reshape(6, 6)
            self._covariance[0:3, 0:3] = measurement_covariance[0:3, 0:3]
            self._covariance[6:9, 6:9] = measurement_covariance[3:6, 3:6]
        self._last_angular_velocity_base = np.zeros(3, dtype=float)
        self._initialized = True

    def predict(self, target_stamp_ns: int, angular_velocity_base) -> None:
        if not self._initialized:
            return

        target_stamp_ns = int(target_stamp_ns)
        angular_velocity_base = np.asarray(angular_velocity_base, dtype=float).reshape(3)
        if self._stamp_ns is None:
            self._stamp_ns = target_stamp_ns
            self._last_angular_velocity_base = angular_velocity_base.copy()
            return

        dt = max(0.0, (target_stamp_ns - self._stamp_ns) * 1.0e-9)
        if dt <= 0.0:
            self._last_angular_velocity_base = angular_velocity_base.copy()
            return

        unbiased_omega = angular_velocity_base - self._gyro_bias_radps
        damping = float(np.exp(-self._velocity_damping_per_sec * dt))

        self._position_m = self._position_m + self._velocity_mps * dt
        self._velocity_mps = damping * self._velocity_mps
        self._rotation = self._rotation * Rotation.from_rotvec(unbiased_omega * dt)

        transition = np.eye(12, dtype=float)
        transition[0:3, 3:6] = dt * np.eye(3, dtype=float)
        transition[3:6, 3:6] = damping * np.eye(3, dtype=float)
        transition[6:9, 9:12] = -dt * np.eye(3, dtype=float)

        accel_noise_var = self._position_process_noise_std_mps2**2
        process_covariance = np.zeros((12, 12), dtype=float)
        process_noise_gain = np.block(
            [
                [0.5 * dt * dt * np.eye(3, dtype=float)],
                [dt * np.eye(3, dtype=float)],
            ]
        )
        process_covariance[0:6, 0:6] = (
            process_noise_gain @ (accel_noise_var * np.eye(3, dtype=float)) @ process_noise_gain.T
        )
        process_covariance[6:9, 6:9] = (
            self._gyro_noise_std_radps**2 * dt
        ) * np.eye(3, dtype=float)
        process_covariance[9:12, 9:12] = (
            self._gyro_bias_random_walk_std_radps2**2 * dt
        ) * np.eye(3, dtype=float)

        self._covariance = transition @ self._covariance @ transition.T + process_covariance
        self._stamp_ns = target_stamp_ns
        self._last_angular_velocity_base = angular_velocity_base.copy()

    def update_pose(
        self,
        measurement_pose_matrix,
        measurement_covariance,
        rotation_gate_deg: float = 35.0,
    ) -> GyroRelativeEskfUpdateResult:
        measurement_pose_matrix = np.asarray(measurement_pose_matrix, dtype=float).reshape(4, 4)
        measurement_covariance = np.asarray(measurement_covariance, dtype=float).reshape(6, 6)

        if not self._initialized:
            self.initialize(
                stamp_ns=0,
                pose_matrix=measurement_pose_matrix,
                measurement_covariance=measurement_covariance,
            )

        measured_position = measurement_pose_matrix[:3, 3]
        measured_rotation = Rotation.from_matrix(measurement_pose_matrix[:3, :3])
        position_residual = measured_position - self._position_m
        rotation_residual = (measured_rotation * self._rotation.inv()).as_rotvec()
        rotation_residual_deg = float(np.degrees(np.linalg.norm(rotation_residual)))
        used_rotation_update = rotation_residual_deg <= float(max(rotation_gate_deg, 0.0))

        if not used_rotation_update:
            return GyroRelativeEskfUpdateResult(
                accepted_pose_update=False,
                used_rotation_update=False,
                position_innovation_m=float(np.linalg.norm(position_residual)),
                rotation_innovation_deg=rotation_residual_deg,
            )

        innovation = np.concatenate([position_residual, rotation_residual], axis=0)
        observation = np.zeros((6, 12), dtype=float)
        observation[0:3, 0:3] = np.eye(3, dtype=float)
        observation[3:6, 6:9] = np.eye(3, dtype=float)
        measurement_covariance_used = measurement_covariance.copy()

        measurement_covariance_used = 0.5 * (
            measurement_covariance_used + measurement_covariance_used.T
        )
        measurement_covariance_used += 1.0e-9 * np.eye(
            measurement_covariance_used.shape[0], dtype=float
        )

        innovation_covariance = (
            observation @ self._covariance @ observation.T + measurement_covariance_used
        )
        kalman_gain = self._covariance @ observation.T @ np.linalg.inv(innovation_covariance)
        delta_state = kalman_gain @ innovation

        self._position_m = self._position_m + delta_state[0:3]
        self._velocity_mps = self._velocity_mps + delta_state[3:6]
        self._rotation = Rotation.from_rotvec(delta_state[6:9]) * self._rotation
        self._gyro_bias_radps = self._gyro_bias_radps + delta_state[9:12]

        identity = np.eye(12, dtype=float)
        covariance_update = identity - kalman_gain @ observation
        self._covariance = (
            covariance_update @ self._covariance @ covariance_update.T
            + kalman_gain @ measurement_covariance_used @ kalman_gain.T
        )

        return GyroRelativeEskfUpdateResult(
            accepted_pose_update=True,
            used_rotation_update=used_rotation_update,
            position_innovation_m=float(np.linalg.norm(position_residual)),
            rotation_innovation_deg=rotation_residual_deg,
        )

    def pose_matrix(self) -> np.ndarray:
        pose_matrix = np.eye(4, dtype=float)
        pose_matrix[:3, :3] = self._rotation.as_matrix()
        pose_matrix[:3, 3] = self._position_m.copy()
        return pose_matrix

    def pose_covariance(self) -> np.ndarray:
        covariance = np.zeros((6, 6), dtype=float)
        covariance[0:3, 0:3] = self._covariance[0:3, 0:3]
        covariance[0:3, 3:6] = self._covariance[0:3, 6:9]
        covariance[3:6, 0:3] = self._covariance[6:9, 0:3]
        covariance[3:6, 3:6] = self._covariance[6:9, 6:9]
        return covariance

    def linear_velocity_base_mps(self) -> np.ndarray:
        return self._rotation.inv().apply(self._velocity_mps)

    def angular_velocity_base_radps(self) -> np.ndarray:
        return self._last_angular_velocity_base - self._gyro_bias_radps
