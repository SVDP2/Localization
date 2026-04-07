# ArUco-IMU 상대측위 시스템 수리·개조·보수·리팩토링 결과보고서

## 0. 요약 결론

현재 관찰 결과를 기준으로 보면 **문제의 주범은 IMU가 아니라 ArUco pose 업데이트 경로**다. IMU만 사용할 때 제자리 yaw 회전이 정확하게 따라간다는 것은,

- IMU 자이로 자체,
- IMU 축 뒤집힘 보정,
- yaw 적분 경로

이 적어도 **현재 실험 범위에서는 치명적 문제를 일으키지 않는다**는 뜻이다.

반대로 카메라/ArUco를 켰을 때 **제자리 회전인데 base_link가 translation하는 현상**이 나타난다면, 지금 시스템은 사실상 아래 셋 중 하나 또는 복합 문제다.

1. **ArUco front-end가 회전과 병진을 안정적으로 분리하지 못한다.**
2. **ArUco 측정이 수상한데도 back-end 필터가 위치 업데이트를 그대로 받아들인다.**
3. **camera extrinsic(카메라-차량 바디 변환) 또는 fisheye 보정 모델이 맞지 않아 회전 시 base_link 위치가 흔들린다.**

따라서 지금 필요한 방향은 “IMU를 의심해서 다시 손보는 것”이 아니라,

- **ArUco 측정 생성부를 rectification + IMU prior 기반으로 고치고**
- **ESKF 업데이트 로직을 ‘ArUco 전체 pose 맹신’에서 ‘조건부 선택적 업데이트’로 바꾸는 것**

이다.

사용자가 생각한 **“초기화 후 IMU orientation을 prior로 ArUco position을 정한다”**는 아이디어는 **방향이 맞다.** 다만 **초기화 1회 prior**로는 부족하고, **매 프레임 연속 prior**로 써야 한다.

---

## 1. 프로젝트 문맥과 현재 설계의 정합성

기존 문서상 팔로워 측위는 리더 기준 상대 상태를 추정하는 구조이며, IMU 고주파 예측과 ArUco/상대 pose 측정 업데이트를 결합하는 방향으로 설계되어 있다. 12주차 자료에서도 상대 상태를 `[Δx, Δy, Δθ, Δvx, Δvy, Δω]`로 두고 IMU 기반 predict + ArUco/GPS 기반 update 구조를 명시했다. fileciteturn0file1 중간보고서 역시 팔로워가 카메라, IMU, RTK-GPS/UWB 등 최소 센서로 리더 대비 상대 위치를 추정하는 비대칭 구조를 목표로 하고 있다. fileciteturn0file4

또한 14주차 및 최종 발표자료를 보면, 측위 쪽 핵심 리스크로 **ArUco 인식 실패/강건성**이 이미 상위 리스크로 잡혀 있고, Fail-safe와 측위 정확도 검증이 남은 과제로 정리되어 있다. fileciteturn0file3turn0file2

즉, 지금 드러난 현상은 원래 설계 의도와 어긋난 “엉뚱한 문제”가 아니라, **문서에서 우려하던 ArUco 강건성 문제가 실제 구현 단계에서 정확히 터진 것**으로 보는 게 맞다.

---

## 2. 중간 업데이트 반영: 지금까지의 진단 판단

### 2.1 지금 확실히 말할 수 있는 것

현재 증상만 놓고 보면 다음은 꽤 강하게 결론내릴 수 있다.

- **IMU yaw 적분은 현재 범위에서 충분히 쓸 만하다.**
- **문제는 ArUco pose가 pure yaw rotation 상황에서 translation을 같이 만들어낸다는 점이다.**
- 따라서 **필터가 “ArUco pose 전체를 믿는 구조”를 유지하면 계속 망가진다.**

### 2.2 사용자의 가설 평가

사용자 가설:

> “IMU만으로 base_link orientation을 결정하고, 그걸 prior로 ArUco 기반 position을 정한다.”

평가:

- **정확한 방향이다.**
- 다만 **초기화 1회**가 아니라 **매 프레임** 써야 한다.
- 더 정확히는 다음과 같이 해야 한다.

1. ESKF가 만든 **예측 자세(q_pred)** 를 만든다.  
2. 그 자세를 ArUco PnP의 **extrinsic guess / candidate selection prior** 로 넣는다.  
3. ArUco가 만든 pose가 q_pred와 크게 다르면 **전체 pose 업데이트를 버리거나, translation 일부만 매우 약하게 쓴다.**

즉, **IMU는 back-end에서 yaw만 보강하는 보조 센서가 아니라, 이제부터는 ArUco front-end를 제약하는 prior 센서**가 되어야 한다.

---

## 3. 코드 리뷰 기준 진단 결과

아래 내용은 실제 저장소 구조를 훑어본 뒤 정리한 것이다.

### 3.1 현재 Python 패키지는 “진짜 ESKF”보다는 gyro-led pose filter에 가깝다

현재 `aruco_imu_eskf_localization` 패키지는 이름은 ESKF지만, 실제 구현은 다음 특성을 가진다.

- 고주파 IMU는 사실상 **자이로 적분 중심**이다.
- 위치는 CV(constant velocity) + damping으로 유지한다.
- vision update가 들어오면 pose를 보정한다.
- accelerometer 기반 translational dynamics는 거의 쓰지 않는다.

이건 지금 단계에서는 나쁜 선택은 아니다. 오히려 **현재 리더 정지, 팔로워 상대 pose 검증 단계**에서는 맞는 단순화다. 다만 이 구조에서는 **translation의 진실 공급원이 거의 ArUco밖에 없기 때문에**, ArUco translation이 흔들리면 필터도 바로 끌려간다.

### 3.2 현재 back-end의 핵심 문제: 회전 gate가 걸려도 위치는 계속 업데이트한다

지금 필터 로직의 가장 위험한 점은 이것이다.

- vision rotation residual이 threshold를 넘으면
- **회전 업데이트만 버리고**
- **position update는 그대로 수행하는 경로**가 있다.

이 구조는 지금 관찰된 현상과 정확히 맞물린다.

즉,

- IMU는 yaw를 잘 예측하고 있고,
- ArUco는 yaw/translation이 같이 꼬여서 이상한 pose를 내고 있는데,
- 필터는 “회전은 이상하네? 그럼 회전만 버리고 위치는 받자”라고 동작한다.

그러면 **ArUco가 만든 가짜 translation이 state를 잡아당긴다.**

지금 증상은 이 설계가 실제로 실패하고 있다는 직접적인 증거다.

### 3.3 현재 ArUco front-end는 IMU prior를 pose solve에 본격적으로 사용하지 않는다

현재 detector/estimator는

- multi-marker IPPE,
- single-marker fallback,
- 이전 pose 기반 temporal gate,
- reprojection RMSE gate

를 갖고 있다.

이 자체는 나쁘지 않다. 하지만 중요한 한계가 있다.

- **이전 pose는 candidate selection/gating에만 쓰이고**,
- **IMU/ESKF 예측 자세를 PnP 해 선택의 강한 prior로 쓰지 않는다.**

즉, “후단 필터가 알고 있는 좋은 yaw 정보”가 “전단 PnP가 translation을 추정하는 과정”에 충분히 반영되지 않는다.

### 3.4 fisheye 카메라 모델 mismatch 가능성이 매우 크다

현재 calibration 파일은 **4개 왜곡 계수** 형태이고, 사용자 설명에도 카메라가 어안 계열로 보인다. 그런데 현재 검출 파이프라인은

- raw image에서 바로 detectMarkers,
- raw corners로 solvePnP / solvePnPGeneric,
- reprojection RMSE도 standard `projectPoints`

경로를 쓴다.

이 조합은 **fisheye 보정 모델을 제대로 반영하지 못할 가능성**이 크다. 특히 마커가 화면 좌우로 이동할수록,

- yaw와 lateral translation이 서로 섞여 보이고,
- reprojection RMSE가 생각보다 낮게 나오는데도,
- 실제 pose는 systematic bias를 가질 수 있다.

사용자가 관찰한 “마커가 좌우로 움직일수록 base_link 전체가 translation한다”는 현상과 아주 잘 맞는다.

### 3.5 camera extrinsic이 TF와 파라미터에 이중 정의되어 있다

현재 구조상 `base_link -> camera`, `base_link -> imu`가

- static TF launch,
- detector/filter parameter

에 중복으로 들어간다.

IMU는 TF lookup으로 어느 정도 해결했지만, camera는 여전히 **static TF와 내부 파라미터가 이중 소스**다.

지금 값은 우연히 맞더라도, 나중에 한쪽만 고치면 다른 쪽이 stale해질 수 있다. camera extrinsic이 조금만 틀려도 pure rotation 시 base_link 환산 위치가 원호를 그리며 흔들릴 수 있다.

### 3.6 single-marker fallback은 지금 단계에서는 이득보다 리스크가 크다

3-marker planar board에서 single-marker fallback은 availability를 높여주지만, 현재 같은 상황에서는

- ambiguity,
- fisheye off-axis error,
- yaw-translation coupling

을 훨씬 키운다.

지금 목표가 “일단 정확한 상대 위치”라면 **single-marker fallback은 끄는 게 맞다.**

---

## 4. 원인 우선순위 진단

### P0-1. 가장 가능성이 높은 원인

**ArUco 회전-병진 결합 + back-end position-only fallback**

현재 구조에서는 ArUco yaw가 흔들릴 때 translation도 같이 흔들리고, 회전만 버리고 위치를 받는 필터 로직 때문에 그 오차가 그대로 state로 들어간다.

### P0-2. 매우 가능성이 높은 원인

**fisheye calibration/model mismatch**

raw fisheye image + standard solvePnP 조합은 중심에서 멀어질수록 pose bias를 키운다. 지금 현상 설명력이 높다.

### P1. 높은 가능성이 있는 원인

**camera extrinsic 또는 base_link pivot 정의 불일치**

회전 중심이 base_link가 아닌데 base_link라고 가정했거나, camera offset/rotation이 조금 틀리면 회전 시 base pose가 흔들린다.

### P2. 중간 가능성

**planar board 자체의 구조적 한계**

완전한 “불가능”은 아니지만, planar board는 yaw와 lateral translation이 강하게 결합되기 쉽다. 특히 3-marker, wide FOV, small board, off-axis 조합에서는 더 그렇다.

---

## 5. 핵심 판단: 지금 필요한 건 “IMU로 orientation만 보강”이 아니다

지금은 더 강하게 바꿔야 한다.

### 현재 구조

- ArUco가 full pose 제안
- ESKF가 yaw residual이 크면 회전만 버림
- position은 계속 받음

### 바꿔야 할 구조

- ESKF가 **predicted orientation prior** 제공
- ArUco front-end가 그 prior를 이용해 pose solve
- back-end는 **pose 전체를 선택적으로 수용**
- yaw residual이 크면 **전체 update reject 또는 translation 일부만 약하게 수용**

즉, **IMU는 후단 보정용이 아니라 전단 pose disambiguation용 prior**가 되어야 한다.

---

## 6. 즉시 실행 권고안 (이번 주 바로 적용)

### 6.1 P0 수리: vision rotation gate 초과 시 position-only update 금지

가장 먼저 해야 한다.

현재는 회전 residual이 gate를 넘으면 rotation만 버리고 translation을 계속 넣는 구조인데, 이걸 다음처럼 바꿔야 한다.

#### 권장 정책

- `|Δyaw_vis-pred| > 10 deg` 이면 **full pose update reject**
- 예외적으로 정말 써야 하면 **translation 전체가 아니라 longitudinal gap만 약하게 반영**

지금 단계에서는 리더가 정지해 있고 IMU yaw가 꽤 좋으므로, **보수적으로 full reject**가 맞다.

#### 추천 초기값

- `vision_rotation_gate_deg = 10.0`
- 현재 단계에서는 gate 초과 시 **position update도 같이 reject**

### 6.2 P0 수리: fisheye rectification 파이프라인 도입

ArUco 검출과 PnP를 raw image에서 직접 하지 말고,

1. fisheye calibration으로 rectification map 생성  
2. raw image를 rectified image로 remap  
3. rectified image에서 marker detect  
4. rectified corners 기준으로 PnP  

로 바꿔야 한다.

#### 구현 지시

- Python front-end 그대로 유지 가능
- OpenCV `cv2.fisheye.initUndistortRectifyMap`, `cv2.remap` 사용
- runtime용 `K_rect`, `D_rect=0` 체계로 통일
- reprojection RMSE도 rectified image 기준으로 재정의

#### 이유

지금 증상은 “화면 좌우 위치에 따라 pose bias가 달라지는” 전형적인 형태라 rectification이 가장 우선이다.

### 6.3 P0 보수: single-marker fallback 비활성화

현재 목표는 availability보다 stability다.

#### 추천 초기값

- `min_markers_for_board = 2`
- `min_markers_to_initialize = 2`
- `single_marker_prior_timeout_sec = 0.0` 또는 기능 자체 disable

### 6.4 P0 보수: ArUco PnP에 IMU/ESKF yaw prior 연속 반영

사용자 아이디어를 정식 구조로 올려야 한다.

#### 구현 방식 1: 가장 현실적

- detector가 최신 filtered pose를 subscribe
- predicted `board -> camera` 또는 `leader_rear -> base_link`를 가져옴
- 이를 `solvePnP(... useExtrinsicGuess=True, SOLVEPNP_ITERATIVE)` 초기값으로 사용
- IPPE 후보는 **seed 생성용**으로만 사용
- 최종 선택은 “reprojection + yaw prior + motion prior” 최소값

#### 구현 방식 2: 더 강한 방법

- yaw는 IMU prior 근처에서만 허용
- PnP 최적화는 translation + small roll/pitch correction 위주로 제한
- yaw는 `q_pred ± 5 deg` 범위로 clip 또는 strong penalty

### 6.5 P0 수리: camera extrinsic 단일 진실 원천(single source of truth)화

`base_link -> camera`는

- static TF,
- detector 내부 파라미터,
- filter 내부 파라미터

에 따로 있으면 안 된다.

#### 권장안

- camera extrinsic은 **TF만** 진실 원천으로 사용
- detector와 filter는 둘 다 TF lookup으로 읽음
- YAML에는 fallback 값만 남김

### 6.6 P1 보수: vision covariance를 훨씬 보수적으로 수정

현재 초기 covariance는 ArUco translation을 너무 잘 믿는 편이다. 지금 단계에서는 아래처럼 시작하는 걸 권장한다.

#### 권장 초기값 (leader_rear 기준 해석)

**3 marker 가시, rectified image 기준**
- longitudinal x std: **0.04 m**
- lateral y std: **0.10 m**
- vertical z std: **0.12 m**
- yaw std: **8 deg**
- roll/pitch std: **15 deg**

**2 marker 가시**
- longitudinal x std: **0.07 m**
- lateral y std: **0.16 m**
- vertical z std: **0.18 m**
- yaw std: **12 deg**
- roll/pitch std: **20 deg**

**1 marker**
- 현재 단계에서는 **비활성화**

#### 추가 inflation 규칙

- `|ω_z| > 0.25 rad/s` 이면 translation covariance **4배**
- `|ω_z| > 0.5 rad/s` 이면 translation covariance **10배**
- `view_angle > 50 deg` 이면 lateral/yaw covariance 추가 inflation
- reprojection RMSE가 2.5px 초과하면 가중치 급감

### 6.7 P1 보수: roll/pitch는 vision에서 거의 쓰지 말 것

현재 차량에서 중요한 건 yaw와 상대 위치다. roll/pitch는 지금 단계에서 ArUco가 잘못 건드릴 이유만 많다.

#### 권장 정책

- IMU quaternion 또는 중력축으로 roll/pitch는 유지
- ArUco는 **yaw + position** 위주로만 사용
- current ESKF measurement model을 6D pose에서 **4D `[x, y, z, yaw]`** 또는 **3D `[x, y, yaw]`**로 줄이는 것을 권장

---

## 7. 권장 아키텍처: “Python front-end + C++ ESKF back-end”

## 7.1 왜 이렇게 나누는가

지금 저장소를 보면 Python 패키지는 검출/실험 속도는 좋지만, 고주파 필터/버퍼/선택적 업데이트를 계속 쌓기에는 유지보수성이 떨어진다. 반면 별도의 C++ ESKF 패키지는 이미 더 큰 GNSS/LiDAR 기반 ESKF 구조를 갖고 있지만, 현재는 의존성이 무겁고 `COLCON_IGNORE` 상태라 그대로 가져다 쓰기엔 과하다.

따라서 현실적인 정답은 아래다.

### 권장 구조

1. **`aruco_frontend_py` (ament_python)**
   - rectification
   - marker detect
   - board corner assemble
   - IMU prior 기반 pose solve
   - quality metric 산출

2. **`relative_pose_eskf` (ament_cmake)**
   - Eigen 기반 ESKF core
   - IMU buffer / delayed vision replay
   - selective measurement update
   - odom / tf publish

3. **`relative_pose_msgs` (ament_cmake or interface package)**
   - custom measurement msg

### 추천 메시지

`ArucoBoardMeasurement.msg`

- `std_msgs/Header header`
- `geometry_msgs/Pose board_to_camera`
- `float64[36] covariance`
- `uint8 visible_markers`
- `float32 reprojection_rmse_px`
- `float32 image_area_px`
- `float32 yaw_prior_residual_deg`
- `bool used_single_marker_fallback`
- `bool rectified`
- `bool accepted_frontend`

이렇게 나누면 “검출 문제”와 “필터 문제”를 확실히 분리할 수 있다.

---

## 8. ESKF 설계 권고

## 8.1 현재 단계(리더 정지)용 MVP 상태벡터

지금은 리더가 움직이지 않으므로, 굳이 복잡한 상대동역학을 바로 넣을 필요가 없다.

### 권장 nominal state

- `p_LB` : leader_rear 기준 follower base 위치 (3)
- `v_LB` : leader_rear 기준 follower base 속도 (3)
- `q_LB` : leader_rear 기준 follower 자세 quaternion (4 nominal, 3 error)
- `b_g` : gyro bias (3)

### error-state

- `δx = [δp, δv, δθ, δb_g]`  → **12차 오차상태**

이건 현재 Python 필터와 철학이 비슷하지만, **측정 모델과 업데이트 정책을 제대로 다시 짜는 것**이 핵심이다.

## 8.2 추후 확장용 full state

리더가 움직이기 시작하면 아래로 확장한다.

- `δx = [δp, δv, δθ, δb_g, δb_a]`  → **15차 오차상태**

가속도계를 본격적으로 쓰거나 leader motion compensation을 넣을 때 이 상태가 더 적합하다.

## 8.3 현재 단계 측정 모델 권장

현재는 다음 측정만으로 충분하다.

1. **IMU gyro predict**
2. **ArUco yaw/position update**
3. 가능하면 **wheel speed / ZUPT / NHC**

### 권장 업데이트 정책

- yaw residual 작음 + quality 좋음 → **full `[x,y,z,yaw]` update**
- yaw residual 큼 → **reject**
- marker 2개만 보임 → 위치 covariance inflation 후 update
- marker 1개 → 현재 단계 disable

## 8.4 current phase 추천 파라미터

### 필터

- `output_rate_hz = 100.0`
- `reset_timeout_sec = 0.5`
- `vision_delay_buffer_sec = 0.30`
- `position_process_noise_std_mps2 = 0.35`
- `gyro_noise_std_radps = 0.03`
- `gyro_bias_random_walk_std_radps2 = 0.003`
- `gyro_bias_bootstrap_sec = 1.0`
- `gyro_bias_bootstrap_max_std_radps = 0.03`
- `initial_position_std_m = 0.15`
- `initial_velocity_std_mps = 0.20`
- `initial_orientation_std_deg = 5.0`
- `initial_gyro_bias_std_radps = 0.05`

### detector / gate

- `min_markers_for_board = 2`
- `min_markers_to_initialize = 2`
- `single_marker_prior_timeout_sec = 0.0`
- `max_reprojection_rmse_px = 3.0`
- `max_position_jump_m = 0.15`
- `max_rotation_jump_deg = 20.0`
- `max_heading_jump_deg = 10.0`
- `front_halfspace_min_z_m = 0.10`
- `max_view_angle_deg = 60.0`

이 값들은 “지금 바로 안정화를 우선”하는 초기값이다. 정밀 최적화보다 **가짜 pose를 안 넣는 것**에 더 초점을 맞춘 값이다.

---

## 9. 파일 단위 수리/개조/보수/리팩토링 지시서

## 9.1 수리

### `gyro_relative_eskf.py`

- `rotation gate 초과 시 position-only update` 제거
- `update_pose()`를 다음처럼 분리
  - `update_position_yaw()`
  - `update_position_only()`
  - `reject_update()`
- 기본 정책은 `yaw mismatch -> reject_update()`

### `aruco_detector_node.py`

- fisheye rectification 추가
- raw image 대신 rectified image 기준 detect/PnP
- filtered pose prior subscribe 추가
- single-marker fallback 비활성화 옵션 기본값 변경

### `board_pose_estimator.py`

- IPPE candidate selection만으로 끝내지 말고, **prior-seeded iterative refine**를 최종 단계로 사용
- candidate score에 `yaw prior residual` 비중 크게 반영
- translation covariance anisotropic 적용

## 9.2 보수

### `relative_localization_node.py`

- camera extrinsic도 TF lookup으로 통일
- detector 품질 metric 받아서 backend gating 강화
- 현재 단계에서는 roll/pitch update 삭제 또는 대폭 약화

## 9.3 개조

### 하드웨어/표식 개조 권장

- 가능하면 **non-coplanar board**로 개조
  - 예: center marker를 20~30 mm 앞으로 띄움
- 또는 runtime board는 유지하되, calibration용으로는 **ChArUco / AprilGrid** 사용
- camera mount 강성 재점검
- camera optical axis와 board center 정렬 재점검

이건 즉시 필수는 아니지만, planar ambiguity를 근본적으로 줄이는 데 매우 효과적이다.

## 9.4 리팩토링

### 패키지 분리

- detector는 Python에 남김
- ESKF backend는 신규 `ament_cmake` C++ 패키지로 분리
- custom msg 패키지 분리
- TF/extrinsic config는 한 곳으로 통일

### 기존 C++ ESKF 패키지 활용 방안

현재 별도 C++ ESKF 패키지는 GNSS/LiDAR/Autoware 의존성이 크고 비활성화 상태다. 그대로 붙이는 건 현재 목표에 비해 무겁다.

#### 권장 방식

- 기존 `EskfCore`의 수학 구조는 참고/재활용
- 하지만 지금 deliverable용 backend는 **상대측위 전용 slim package**로 새로 만드는 것이 맞다
- 의존성은 최소한으로 유지
  - `rclcpp`
  - `sensor_msgs`
  - `geometry_msgs`
  - `nav_msgs`
  - `tf2_ros`
  - `Eigen3`
  - 선택적으로 `Sophus`

---

## 10. 검증 계획

## 10.1 가장 먼저 해야 할 실험: pure rotation 분리 실험

### 실험 목적

제자리 회전 시 translation이 어디서 생기는지 분해한다.

### 로그 대상

- `board -> camera` raw pose
- `board -> base_link` transformed pose
- filtered `leader_rear -> base_link`
- IMU yaw / gyro z
- reprojection RMSE
- visible marker count
- image center 대비 marker centroid 위치

### 해석

- `board -> camera`는 흔들리지만 `board -> base_link`가 안정적이면 → **정상 lever arm 효과**
- `board -> base_link`도 함께 흔들리면 → **extrinsic 또는 PnP 문제**
- yaw residual 커질 때만 translation 튀면 → **filter update logic 문제**

## 10.2 정량 통과 기준 (현재 리더 정지 단계)

### Pure yaw 회전 테스트

- follower를 제자리에서 ±45° 회전
- leader board는 고정

#### 통과 기준

- yaw 오차: **2° 이내**
- 상대 위치 변동량 `||Δp||`: **3 cm 이내**
- longitudinal gap 변동량: **2 cm 이내**
- lateral 변동량: **3 cm 이내**
- rotation gate 초과 vision update 비율은 있어도 됨. 대신 state가 안 끌려가면 됨.

### 좌우 off-axis 테스트

- 같은 거리에서 board가 영상 중심/좌측/우측에 오도록 배치

#### 통과 기준

- 위치 bias 변화량: **5 cm 이내**
- reprojection RMSE 증가 시 covariance가 함께 증가해야 함

---

## 11. 지금 바로 추천하는 운영 모드

리더 정지, follower 상대 pose 정확도만 보는 현재 단계에서는 아래 모드를 추천한다.

### 권장 운영 모드 A (가장 안전)

- rectified ArUco
- 2 marker 이상만 사용
- IMU yaw predict
- yaw mismatch면 ArUco update reject
- roll/pitch는 IMU 또는 고정값 유지

이 모드는 availability는 조금 떨어져도, **가짜 translation으로 state가 깨지는 문제를 가장 빨리 막는다.**

### 권장 운영 모드 B (다음 단계)

- rectified ArUco
- IMU prior 기반 iterative solvePnP
- `[x,y,z,yaw]` selective update
- angular-rate dependent covariance inflation

이 모드가 본선용에 가깝다.

---

## 12. 사용자의 아이디어에 대한 최종 답변

사용자가 말한

> “처음 초기화한 다음 IMU만으로 orientation을 결정하고, 그걸 prior로 ArUco 기반 position을 정하자”

는 생각은 **정답에 가깝다.**

다만 수정이 필요하다.

### 잘못하면 안 되는 버전

- 초기화 때만 IMU 자세를 씀
- 이후 ArUco는 다시 full pose를 독자적으로 냄
- filter가 translation만 계속 받음

이러면 지금과 같은 문제가 다시 난다.

### 맞는 버전

- **매 프레임** ESKF predicted orientation을 prior로 사용
- ArUco pose solve 자체가 그 prior의 제약을 받음
- yaw residual이 크면 **translation까지 함부로 쓰지 않음**

즉, **orientation prior는 초기화용이 아니라 지속적 관측 제약**이어야 한다.

---

## 13. 추후 개선 사항 (리더가 움직이기 시작한 이후)

아래는 요청대로 **중요도별**로 나눈다.

### 중요도 상

1. **리더 motion compensation**
   - leader yaw rate, speed, steering을 V2V로 받아 relative dynamics에 반영
   - leader 정지 가정을 제거

2. **wheel odom / vehicle speed integration**
   - follower가 pure rotation인지 실제 translation 중인지 구분
   - 지금 문제를 근본적으로 더 잘 분리

3. **timestamp alignment / delayed measurement replay 강화**
   - moving leader 환경에서는 vision 지연이 바로 상대 위치 오차로 바뀜

4. **ESKF를 C++ backend로 이전**
   - moving leader 단계부터는 Python보다 deterministic backend가 훨씬 낫다

### 중요도 중

1. **non-coplanar board 또는 더 큰 marker array**
   - planar ambiguity 감소

2. **UWB를 truly fallback 거리 센서로 결합**
   - vision dropout 시 gap 안전성 확보

3. **yaw 외에 longitudinal gap 전용 measurement channel 분리**
   - full pose 대신 목적지향 측정 모델로 단순화 가능

4. **Mahalanobis gate + quality learning**
   - reprojection, image area, off-axis angle, marker count를 합친 quality score 도입

### 중요도 하

1. **roll/pitch까지 fully estimated 6D relative pose 고도화**
   - 현재 목적에서는 우선순위 낮음

2. **visual-inertial preintegration 수준의 고급 모델**
   - 현재 프로젝트 범위에서는 과투자 위험

3. **AprilTag/ChArUco 런타임 전환 비교실험**
   - 좋은 옵션이지만 지금 당장 필수는 아님

---

## 14. 최종 권고

지금 가장 중요한 한 줄 결론은 이것이다.

> **IMU는 이미 꽤 괜찮다. 지금 고쳐야 하는 건 ArUco 측정 생성부와, 그 측정을 받아들이는 ESKF 업데이트 정책이다.**

실행 순서를 아주 짧게 정리하면 아래다.

1. **fisheye rectification 넣기**  
2. **single-marker 끄기**  
3. **rotation gate 초과 시 position-only update 금지**  
4. **IMU/ESKF yaw prior를 PnP에 매 프레임 넣기**  
5. **camera extrinsic을 TF 단일 원천으로 통일**  
6. **translation covariance를 훨씬 보수적으로 키우기**

이 6개만 해도 지금 보이는 “제자리 회전인데 translation하는 문제”는 상당 부분 정리될 가능성이 높다.
