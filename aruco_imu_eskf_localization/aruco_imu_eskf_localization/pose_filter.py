import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation, Slerp


class PoseFilter:
    def __init__(
        self,
        dt: float = 0.1,
        process_noise_std: float = 0.3,
        measurement_noise_std_xy: float = 0.02,
        measurement_noise_std_z: float = 0.05,
        orientation_outlier_threshold_deg: float = 45.0,
        orientation_meas_alpha: float = 0.15,
        imu_angular_velocity_sign: float = 1.0,
        imu_left_multiply: bool = False,
        init_position_std_m: float = 0.3,
        init_velocity_std_mps: float = 1.0,
    ) -> None:
        dt = float(np.clip(dt, 1.0e-4, 0.2))

        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.array(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=float,
        )
        self.kf.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=float,
        )
        self.kf.R = np.diag(
            [
                measurement_noise_std_xy**2,
                measurement_noise_std_xy**2,
                measurement_noise_std_z**2,
            ]
        )
        self._process_noise_std = float(process_noise_std)
        self.kf.Q = self._make_process_noise(dt)
        self._init_position_std_m = float(init_position_std_m)
        self._init_velocity_std_mps = float(init_velocity_std_mps)
        self.kf.P = self._make_initial_covariance()

        self.orientation = None
        self.orientation_outlier_threshold_deg = float(orientation_outlier_threshold_deg)
        self.orientation_meas_alpha = float(orientation_meas_alpha)
        self.imu_angular_velocity_sign = float(imu_angular_velocity_sign)
        self.imu_left_multiply = bool(imu_left_multiply)
        self.initialized = False

    def _make_initial_covariance(self) -> np.ndarray:
        return np.diag(
            [
                self._init_position_std_m**2,
                self._init_position_std_m**2,
                self._init_position_std_m**2,
                self._init_velocity_std_mps**2,
                self._init_velocity_std_mps**2,
                self._init_velocity_std_mps**2,
            ]
        )

    def _make_process_noise(self, dt: float) -> np.ndarray:
        dt = float(np.clip(dt, 1.0e-4, 0.2))
        return Q_discrete_white_noise(
            dim=2,
            dt=dt,
            var=self._process_noise_std**2,
            block_size=3,
        )

    def reset(self) -> None:
        self.initialized = False
        self.orientation = None
        self.kf.x[:] = 0.0
        self.kf.P = self._make_initial_covariance()

    def predict(self, dt: float, angular_velocity=None) -> None:
        dt = float(np.clip(dt, 1.0e-4, 0.2))
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        self.kf.Q = self._make_process_noise(dt)
        self.kf.predict()

        if self.initialized and angular_velocity is not None:
            omega = np.asarray(angular_velocity, dtype=float).reshape(3)
            signed_omega = self.imu_angular_velocity_sign * omega
            angle = float(np.linalg.norm(signed_omega) * dt)
            if angle > 1.0e-6:
                delta = Rotation.from_rotvec(signed_omega * dt)
                if self.imu_left_multiply:
                    self.orientation = delta * self.orientation
                else:
                    self.orientation = self.orientation * delta

    def update(self, tvec, rvec) -> None:
        position = np.asarray(tvec, dtype=float).reshape(3)
        measured_rot = Rotation.from_rotvec(np.asarray(rvec, dtype=float).reshape(3))

        if not self.initialized:
            self.kf.x[:3] = position.reshape(3, 1)
            self.kf.x[3:] = 0.0
            self.orientation = measured_rot
            self.initialized = True
            return

        self.kf.update(position)

        diff = measured_rot * self.orientation.inv()
        angle_diff_deg = np.degrees(diff.magnitude())
        if angle_diff_deg <= self.orientation_outlier_threshold_deg:
            slerp = Slerp(
                [0.0, 1.0],
                Rotation.concatenate([self.orientation, measured_rot]),
            )
            self.orientation = slerp([self.orientation_meas_alpha])[0]

    def get_pose(self):
        if not self.initialized or self.orientation is None:
            return None, None
        return self.orientation.as_rotvec(), self.kf.x[:3].flatten()
