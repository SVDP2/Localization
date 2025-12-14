import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation, Slerp


# =============================================================================
# User-tunable parameters (edit numbers only)
# =============================================================================
# 목표: 이 파일의 "칼만/휴리스틱 파라미터"를 한 곳에 모아서, 숫자만 바꿔서 쉽게 튜닝.
#
# 단위 주의:
# - tvec: meters (aruco_detector_node.py에서 object_points를 m로 만들기 때문에 solvePnP도 m)
# - rvec: radians (Rodrigues rotation vector)
# - IMU angular velocity: rad/s
#
# 베이스라인(일반적인 실내 USB 카메라 + 5~15cm ArUco, 0.5~2m 거리, 보정 OK 기준):
# - 평면(x,y) 위치 측정 표준편차: 0.005~0.03 m
# - 깊이(z) 위치 측정 표준편차: x,y의 1.5~3배 (PnP 특성상 z가 더 흔들림)
# - 등속 모델 프로세스(가속도) 표준편차: 0.1~1.0 m/s^2 (로봇 움직임/진동에 따라)
# - 자세 이상치(플립) 컷: 30~60 deg
# - 자세 융합 alpha: 0.05~0.4 (IMU 깨끗하면 작게, IMU 드리프트 크면 크게)

# --- Position Kalman Filter (Constant Velocity) ---
DEFAULT_DT_SEC = 0.1

# 측정 잡음 (Aruco solvePnP tvec)
# - 키우면: 출력이 더 부드럽고(지터↓) 반응이 느림(지연↑)
# - 줄이면: 출력이 더 빠르지만(지연↓) 지터↑ / 튐↑
# 권장 시작 범위(보정 OK 기준): x,y = 0.005~0.03 m, z = 0.01~0.08 m
POS_MEAS_STD_XY_M = 0.02
POS_MEAS_STD_Z_M = 0.05

# 프로세스 잡음(등속 모델에서 '가속도' 표준편차로 해석)
# - 키우면: 모델을 덜 믿고 측정을 더 따라감(반응↑, 지터↑)
# - 줄이면: 모델을 더 믿음(반응↓, 부드러움↑)
# 권장 시작 범위: 0.1~1.0 m/s^2
POS_PROCESS_ACCEL_STD_MPS2 = 0.30

# 초기 불확실성 (처음 수렴 성향)
# - 키우면: 초기에는 측정을 더 강하게 따라가며 빠르게 자리를 잡음(초기 지터↑ 가능)
# - 줄이면: 초기 진동은 줄지만 수렴이 느려질 수 있음
INIT_POS_STD_M = 0.30
INIT_VEL_STD_MPS = 1.00

# dt 안정화 (IMU 콜백 dt가 튀는 경우 보호)
# - IMU dt가 간헐적으로 크게 튀면 exp(rotvec) / Q가 과대해져 필터가 망가질 수 있어 클램프 권장
MIN_PREDICT_DT_SEC = 1e-4
MAX_PREDICT_DT_SEC = 0.2

# --- Orientation fusion (IMU gyro prediction + ArUco rvec correction) ---
# IMU(카메라 본체 각속도)를 넣어서 "보드의 자세(카메라 좌표계에서)"를 예측할 때는 보통 부호가 반대가 됩니다.
# - +1.0: "관측되는 보드가 카메라에서 그 방향으로 돈다"라고 해석할 때
# - -1.0: "카메라가 돈 만큼 보드는 반대로 보인다"라고 해석할 때(대부분 IMU가 카메라에 붙은 경우)
# 튜닝 팁:
# - IMU 예측을 켰더니 보드가 '반대로' 도는 느낌이면: 부호(+/-)를 뒤집어 보세요.
# - IMU 예측을 켰더니 보드가 '이상하게 꼬이거나' 축이 섞여 보이면: IMU→Camera 축 변환(노드 쪽)도 재점검하세요.
IMU_ANGVEL_SIGN_FOR_VISUAL_POSE = -1.0

# 각속도 적분 방식:
# - True : rot_delta를 왼쪽에서 곱함 (R_new = dR * R_old)  [ω가 카메라(body) 프레임 기준일 때 보통 더 자연스러움]
# - False: rot_delta를 오른쪽에서 곱함 (R_new = R_old * dR)
# 튜닝 팁: 위 부호를 바꿔도 해결 안 되면, 곱셈 방향(True/False)도 같이 바꿔 보세요.
IMU_LEFT_MULTIPLY = True

# 자세 측정(PnP) 이상치 제거 임계값 [deg]
ORIENTATION_OUTLIER_THRESHOLD_DEG = 45.0

# SLERP 융합 계수 alpha (0=예측만, 1=측정만)
# - 키우면: ArUco 자세를 더 잘 따라감(지터↑, 플립 리스크↑)
# - 줄이면: IMU 예측을 더 믿음(지터↓, IMU 드리프트 영향↑)
ORIENTATION_MEAS_ALPHA = 0.15


class PoseFilter:
    def __init__(
        self,
        dt: float = DEFAULT_DT_SEC,
        process_noise_std: float = POS_PROCESS_ACCEL_STD_MPS2,
        measurement_noise_std: float | None = None,
    ):
        """
        Initialize the PoseFilter with a Constant Velocity model for position
        and IMU-based prediction for orientation.
        
        Args:
            dt (float): Initial time step assumption.
            process_noise_std (float): Standard deviation of (acceleration) process noise [m/s^2].
            measurement_noise_std (float|None): If set, overrides POS_MEAS_STD_* with a single std [m].
        """
        dt = float(np.clip(dt, MIN_PREDICT_DT_SEC, MAX_PREDICT_DT_SEC))

        # --- Position Filter (Constant Velocity Model) ---
        # State: [x, y, z, vx, vy, vz]
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self._process_accel_std = float(process_noise_std)
        
        # State Transition Matrix (F)
        self.kf.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement Function (H)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Measurement Noise Covariance (R): tvec measurement noise
        # - R를 키우면(=측정이 더 시끄럽다) 출력이 더 "안정적/관성적"이고, 반응이 느려집니다.
        # - R를 줄이면 측정을 더 잘 따라가지만, 출력이 더 "지터"납니다.
        if measurement_noise_std is not None:
            pos_meas_std_xy = float(measurement_noise_std)
            pos_meas_std_z = float(measurement_noise_std)
        else:
            pos_meas_std_xy = float(POS_MEAS_STD_XY_M)
            pos_meas_std_z = float(POS_MEAS_STD_Z_M)
        self.kf.R = np.diag([pos_meas_std_xy**2, pos_meas_std_xy**2, pos_meas_std_z**2])
        
        # Process Noise Covariance (Q)
        # - Q를 키우면 모델(등속)을 덜 믿고 측정을 더 따라갑니다(반응 ↑, 지터 ↑).
        # - Q를 줄이면 모델을 더 믿습니다(반응 ↓, 부드러움 ↑).
        self.kf.Q = self._make_process_noise(dt)
        
        # Initial State Covariance (P)
        # - INIT_POS_STD_M / INIT_VEL_STD_MPS를 키우면 초기에는 측정을 더 강하게 따라가며,
        #   작게 주면 초기 오버슈트는 줄지만 수렴이 느려질 수 있습니다.
        self.kf.P = np.diag([
            float(INIT_POS_STD_M) ** 2,
            float(INIT_POS_STD_M) ** 2,
            float(INIT_POS_STD_M) ** 2,
            float(INIT_VEL_STD_MPS) ** 2,
            float(INIT_VEL_STD_MPS) ** 2,
            float(INIT_VEL_STD_MPS) ** 2,
        ])
        
        # --- Orientation Filter ---
        self.orientation = None  # Scipy Rotation object
        self.angular_velocity = np.zeros(3)  # [wx, wy, wz] in camera frame
        
        # Outlier rejection threshold (degrees)
        self.orientation_outlier_threshold_deg = float(ORIENTATION_OUTLIER_THRESHOLD_DEG)
        
        self.initialized = False

    def _make_process_noise(self, dt: float) -> np.ndarray:
        dt = float(np.clip(dt, MIN_PREDICT_DT_SEC, MAX_PREDICT_DT_SEC))
        return Q_discrete_white_noise(
            dim=2,
            dt=dt,
            var=float(self._process_accel_std) ** 2,
            block_size=3,
        )

    def predict(self, dt, angular_velocity=None):
        """
        Predict the next state.
        Args:
            dt (float): Time delta since last update in seconds.
            angular_velocity (np.array): [wx, wy, wz] in rad/s (optional)
        """
        dt = float(np.clip(dt, MIN_PREDICT_DT_SEC, MAX_PREDICT_DT_SEC))

        # --- Position Prediction ---
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        self.kf.Q = self._make_process_noise(dt)
        self.kf.predict()
        
        # --- Orientation Prediction ---
        if self.initialized and angular_velocity is not None:
            # Update stored angular velocity
            self.angular_velocity = np.asarray(angular_velocity, dtype=float).reshape(3)
            
            # Create rotation delta from angular velocity * dt
            # Magnitude is angle, direction is axis
            signed_w = IMU_ANGVEL_SIGN_FOR_VISUAL_POSE * self.angular_velocity
            angle = float(np.linalg.norm(signed_w) * dt)
            if angle > 1e-6:
                rot_delta = Rotation.from_rotvec(signed_w * dt)
                if IMU_LEFT_MULTIPLY:
                    self.orientation = rot_delta * self.orientation
                else:
                    self.orientation = self.orientation * rot_delta

    def update(self, tvec, rvec):
        """
        Update the filter with a new measurement.
        
        Args:
            tvec (array-like): Translation vector [x, y, z]
            rvec (array-like): Rotation vector [rx, ry, rz]
        """
        z = np.array(tvec).flatten()
        measured_rot = Rotation.from_rotvec(np.array(rvec).flatten())
        
        if not self.initialized:
            # Initialize state
            self.kf.x[:3] = z.reshape(3, 1)
            self.kf.x[3:] = 0
            self.orientation = measured_rot
            self.initialized = True
            return

        # --- Position Update ---
        self.kf.update(z)
        
        # --- Orientation Update ---
        # 1. Outlier Rejection
        # Calculate angle difference between predicted and measured
        # diff = measured * predicted.inv()
        diff = measured_rot * self.orientation.inv()
        angle_diff_rad = diff.magnitude()
        angle_diff_deg = np.degrees(angle_diff_rad)
        
        # Normalize angle to [0, 180]
        if angle_diff_deg > 180:
            angle_diff_deg = 360 - angle_diff_deg
            
        if angle_diff_deg > self.orientation_outlier_threshold_deg:
            # Reject measurement, keep prediction
            # But maybe trust measurement a tiny bit if prediction is drifting?
            # For now, hard reject to avoid flips.
            pass
        else:
            # SLERP (Spherical Linear Interpolation) for fusion
            # Simple fusion: interpolate between predicted and measured
            alpha = float(ORIENTATION_MEAS_ALPHA)
            key_times = [0, 1]
            key_rots = Rotation.concatenate([self.orientation, measured_rot])
            slerp = Slerp(key_times, key_rots)
            self.orientation = slerp([alpha])[0]

    def get_pose(self):
        """
        Get the current filtered pose.
        """
        if not self.initialized:
            return None, None
            
        # Position
        tvec = self.kf.x[:3].flatten()
        
        # Rotation
        rvec = self.orientation.as_rotvec()
        
        return rvec, tvec
