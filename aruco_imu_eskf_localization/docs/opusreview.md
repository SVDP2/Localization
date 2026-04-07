# gptreview2.md 코드 대조 검증 및 보충

> 이 문서는 `gptreview2.md`의 진단을 실제 코드와 대조 검증하고, 문서에서 다루지 않은 추가 이슈를 정리한 것이다.

---

## 1. 문서 진단의 코드 대조 검증

### 3.1 "ESKF라기보다 gyro-led pose filter" -- 정확하다

`gyro_relative_eskf.py` predict 단계를 보면:

- rotation: `self._rotation = self._rotation * Rotation.from_rotvec(unbiased_omega * dt)` (gyro 적분)
- position: `self._position_m = self._position_m + self._velocity_mps * dt` (CV 모델)
- velocity: `damping * self._velocity_mps` (지수 감쇠)
- 가속도계 입력 없음, accelerometer bias 상태 없음

error-state 12차: `[dp(3), dv(3), dtheta(3), db_g(3)]`. 가속도계 bias `db_a` 미포함. 문서 진단 정확.

### 3.2 "회전 gate 초과 시 position-only update" -- 정확하다, 코드에서 직접 확인됨

`gyro_relative_eskf.py` `update_pose()` 241~244행:

```python
# used_rotation_update == False 일 때
innovation = position_residual
observation = np.zeros((3, 12), dtype=float)
observation[:, 0:3] = np.eye(3, dtype=float)
measurement_covariance_used = measurement_covariance[0:3, 0:3].copy()
```

회전 gate를 넘으면 rotation innovation은 버리되 position innovation은 그대로 Kalman update에 투입한다. 문서 진단 정확.

추가 확인: 기존 테스트 `test_gyro_relative_eskf.py`의 `test_update_pose_skips_rotation_when_gate_is_exceeded`가 이 동작을 **통과 조건으로 검증**하고 있다. 즉, 현재 테스트가 "rotation만 skip하고 position은 update한다"를 정상 동작으로 간주한다. 수정 시 이 테스트도 함께 바꿔야 한다.

### 3.3 "ArUco front-end가 IMU prior를 사용하지 않는다" -- 정확하다

`aruco_detector_node.py`는 IMU 관련 subscription이 전혀 없다. `previous_board_pose`는 detector 자체의 이전 detection 결과에서만 온다. ESKF 예측 자세가 PnP solve에 반영되는 경로가 없다.

### 3.4 "fisheye calibration/model mismatch" -- 부분적으로 과대 평가

`cam_intrinsic.yaml`을 보면:

- 왜곡 계수 4개: `[-0.0149, -0.0198, 0.0189, -0.00703]`
- 절대값이 모두 0.02 미만으로 매우 작다
- 교정 RMS: `0.4346 px` (양호)
- 초점거리 `fx=378.6, fy=378.8` at 640x480 -- FOV 약 80도
- chessboard 9x9, 0.07m square로 표준 교정

이 수준의 왜곡과 교정 잔차는 standard pinhole + `(k1, k2, p1, p2)` 모델로 충분히 설명되는 범위다. 진짜 fisheye (equidistant projection) 카메라라면 왜곡 계수가 이렇게 작게 나오지 않는다. 현재 코드가 `cv2.solvePnP` + `cv2.projectPoints`를 standard 모델로 호출하는 것은 이 교정 파일 기준으로는 크게 잘못된 것이 아닐 수 있다.

문서가 fisheye mismatch를 **P0-2 (매우 가능성이 높은 원인)** 으로 올린 것은 현재 교정 데이터 기준으로는 과대 평가다. rectification을 적용하면 나쁠 것 없으므로 "해야 할 일" 목록에서 빠질 필요는 없지만, 현 상황의 **주범으로 fisheye mismatch를 지목하는 것은 근거가 약하다**. 주범은 여전히 P0-1 (position-only fallback + planar IPPE ambiguity) 쪽이 더 유력하다.

### 3.5 "camera extrinsic 이중 정의" -- 정확하다, 기본값 불일치도 존재

- `aruco_detector_node.py` 95행: `base_to_camera_translation` 기본값 `[0.27, 0.0, 0.135]`
- `relative_localization_node.py` 91행: `base_to_camera_translation` 기본값 `[0.0, 0.0, 0.0]`

params.yaml가 정상 로드되면 둘 다 `[0.27, 0.0, 0.135]`로 통일되지만, **params.yaml 없이 노드를 단독 실행하면 기본값이 서로 다르다**. 이것은 잠재적 버그다.

### 3.6 "single-marker fallback 리스크" -- 정확하다

현재 `params.yaml`에서 `min_markers_for_board: 1`이므로 single-marker fallback이 활성화 상태다.

---

## 2. 문서에서 누락된 이슈

### A. `PoseFilter`는 dead code이고 `filterpy` 의존성이 불필요하다

`filters/pose_filter.py`는 `filterpy.kalman.KalmanFilter` 기반의 구식 위치 필터다. 테스트 `test_pose_filter.py`에서 검증되지만 **어떤 노드에서도 import하거나 사용하지 않는다**. 현재 실제 필터링은 전부 `GyroRelativeEskf`가 담당한다.

`setup.py`의 `install_requires`에 `filterpy`가 있는데, 런타임에는 필요 없다. 불필요한 의존성이다.

### B. `single_marker_prior_timeout_sec`가 multi-marker 검출의 temporal gate도 같이 끈다

`aruco_detector_node.py` 241~249행에서:

```python
previous_board_pose = (
    self._last_board_pose
    if pose_prior_is_fresh(
        self._last_board_pose_stamp_ns,
        current_stamp_ns,
        self.single_marker_prior_timeout_sec,
    )
    else None
)
```

이 timeout이 만료되면 `previous_board_pose = None`이 되고, `estimate_pose()`에 `previous_board_pose=None`으로 전달된다. 이 경우:

- single-marker는 prior 없이 불가이므로 자동 거부됨 (정상)
- **multi-marker도 temporal motion gate가 적용되지 않는다** (`_passes_motion_gate`는 `previous_board_pose is not None` 일 때만 실행)

즉, 0.25초 이상 검출이 끊겼다가 다시 보이면 multi-marker 검출에 temporal consistency check가 없다. 이름은 `single_marker_prior_timeout_sec`이지만 실제로는 **모든 마커 수의 temporal gate 유효 기간**을 제어한다.

이것은 짧은 occlusion 후 갑작스러운 pose jump를 허용할 수 있는 경로다.

### C. gyro orientation process noise 이산화가 필터 자세 확신도를 과도하게 높일 수 있다

`gyro_relative_eskf.py` 200~203행:

```python
process_covariance[6:9, 6:9] = (
    self._gyro_noise_std_radps**2 * dt * dt
) * np.eye(3, dtype=float)
```

이것은 `sigma^2 * dt^2` 형태인데, 이 해석이 맞으려면 `gyro_noise_std_radps`가 **샘플당 표준편차 (discrete-time noise)** 여야 한다. 만약 continuous-time spectral density (rad/s/sqrt(Hz)) 로 의도한 거라면 `sigma^2 * dt`가 맞다.

어느 해석이든 현재 결과를 보면: sigma=0.05, dt=0.01s (100 Hz 기준)일 때 step당 orientation variance = `0.0025 * 0.0001 = 2.5e-7 rad^2`. 0.5초(50 step) 후 누적 std = 0.0035 rad = **0.2 deg**.

이는 매우 타이트한 값이다. 필터가 자기 자세 예측을 과도하게 확신하면, ArUco vision update에서 position correction의 Kalman gain이 상대적으로 커진다. 이것이 gptreview2.md 3.2의 position-only update 문제를 더 악화시키는 기여 요인이 된다.

비교: `sigma^2 * dt` 형태라면 같은 조건에서 0.5초 후 std = 2.0 deg. 현재 대비 10배 넓은 불확실성을 가져, vision update 시 rotation 쪽 Kalman gain이 더 커지고 position 쪽은 상대적으로 줄어든다.

### D. `_measurement_covariance()`의 position 불확실성이 isotropic이다

`aruco_detector_node.py` 336~337행:

```python
base_xy_std = 0.015  # 3마커 기준
```

x와 y에 같은 std를 쓴다. 그러나 planar board에서 **종방향(depth, leader_rear x)** 불확실성은 **횡방향(lateral, leader_rear y)** 불확실성보다 일반적으로 2~4배 크다. gptreview2.md 6.6절에서 anisotropic을 권장하고 있지만, 현재 covariance가 isotropic이라는 사실 자체를 현 문제의 기여 요인으로 명시하지는 않았다.

isotropic covariance는 필터가 종방향 position update를 실제보다 더 강하게 신뢰하게 만든다. 순수 yaw 회전 시 ArUco가 만드는 가짜 종방향 translation이 과소 평가된 uncertainty를 뚫고 state를 끌어당기는 현상을 악화시킨다.

### E. stale `__pycache__` 파일이 남아 있다

`aruco_imu_eskf_localization/__pycache__/`에 `camera_driver_node.cpython-310.pyc`, `imu_driver_node.cpython-310.pyc` 등 현재 코드에 존재하지 않는 모듈의 바이트코드가 남아 있다. 또한 `launch/__pycache__/`에도 이미 삭제/개명된 launch 파일의 잔해가 있다. 기능에 영향은 없으나 혼란 유발. `__pycache__`를 `.gitignore`에 추가하고 정리 권장.

### F. 통합 테스트 부재

현재 테스트는 모두 개별 모듈 단위 테스트다:

- `test_board_pose_estimator.py`: PnP/gate 로직 단위 테스트
- `test_gyro_relative_eskf.py`: 필터 수학 단위 테스트
- `test_pose_filter.py`: 구식 필터 테스트 (dead code)
- `test_frame_conventions.py`: 좌표 변환 단위 테스트
- `test_aruco_detector_helpers.py`: prior 유효시간 판정 단위 테스트

없는 것:

- detector -> filter -> output 전체 파이프라인 통합 테스트
- delayed measurement replay 로직 테스트
- timer 기반 extrapolation 정합성 테스트
- `relative_localization_node` 자체의 단위 테스트

특히 delayed vision replay (`_restore_snapshot_before` -> predict -> update -> predict) 경로는 복잡한 상태 관리를 포함하는데 테스트가 없다.

---

## 3. 수정 우선순위 재평가

문서의 원래 우선순위에 위 보충 내용을 반영하면:

| 순위 | 항목 | 문서 원래 등급 | 재평가 |
|------|------|--------------|--------|
| 1 | rotation gate 초과 시 position-only update 금지 | P0-1 | **P0 유지, 가장 먼저** |
| 2 | isotropic -> anisotropic measurement covariance | 6.6 (P1) | **P0으로 격상** -- isotropic이 position-only update 문제를 직접 악화 |
| 3 | single-marker fallback 비활성화 | P0 보수 | P0 유지 |
| 4 | `single_marker_prior_timeout_sec`가 multi-marker temporal gate도 끄는 문제 분리 | 미언급 | **P1 신규** |
| 5 | IMU/ESKF yaw prior를 PnP에 연속 반영 | P0 보수 | P0 유지 (다만 1~3번보다 구현 비용이 큼) |
| 6 | camera extrinsic 단일 진실 원천 + 기본값 불일치 수정 | P0 수리 | P0 유지 |
| 7 | fisheye rectification | P0-2 | **P1로 하향** -- 현재 왜곡 계수가 매우 작아 영향이 제한적 |
| 8 | orientation process noise 이산화 재검토 | 미언급 | **P1 신규** |
| 9 | dead code 정리 (PoseFilter, filterpy, stale pycache) | 미언급 | P2 |
| 10 | 통합 테스트 추가 | 미언급 | P2 |

---

## 4. 결론

문서의 핵심 진단은 **대부분 정확하다**. 특히:

- position-only update fallback이 현재 증상의 주된 경로라는 분석
- ArUco front-end에 IMU prior가 없다는 분석
- camera extrinsic 이중 소스 문제

이 세 가지는 코드에서 직접 확인된다.

**fisheye mismatch 진단은 현 카메라 교정 데이터 기준으로 과대 평가**되어 있다. 왜곡이 매우 작고 교정 잔차가 양호하므로 fisheye rectification이 현 문제의 주범일 가능성은 낮다. rectification 자체는 해서 나쁠 것 없으나 우선순위는 P0에서 P1으로 내리는 것이 합리적이다.

문서에서 **누락된 가장 실질적인 이슈**는:

1. measurement covariance isotropic 문제가 position-only fallback과 결합해 종방향 오차를 키운다는 점
2. `single_marker_prior_timeout_sec`가 multi-marker temporal gate까지 비활성화한다는 점
3. orientation process noise 이산화 방식이 필터 자세 확신도를 과도하게 높여 position-only update 문제를 간접 악화시킨다는 점

이 세 가지다.
