# ArUco-IMU 상대측위 시스템 현황 및 개선 로드맵

## 1. 시스템 개요

이 패키지(`aruco_imu_eskf_localization`)는 follower 차량이 leader 차량 후방의 ArUco marker board를 보고, IMU 자이로와 결합하여 leader 대비 상대 pose를 실시간으로 추정하는 시스템이다. ROS 2 ament_python 패키지로 구성되어 있다.

### 현재 단계

- leader 차량 정지 상태에서 follower의 상대 위치/자세를 안정적으로 추정
- 100 Hz gyro-led 예측 + 카메라 프레임율 vision update

---

## 2. 아키텍처

### 2.1 노드 구성

```
image_raw ──> [aruco_detector_node] ──board_pose──> [relative_localization_node] ──> odom/pose/TF
                   ^    |                                  |    ^
                   |    v                                  |    |
              debug_image                              /imu    |
                   ^                                           |
                   |       <---- pose prior (feedback) --------+
```

| 노드 | 역할 |
|------|------|
| `aruco_detector_node` | fisheye rectification, marker 검출, board pose 추정, ESKF prior 기반 candidate 선택 |
| `relative_localization_node` | gyro 적분 예측 + vision pose update, delayed measurement replay, odom/TF 출력 |
| `pose_rviz_marker_node` | leader_rear 기준 pose를 RViz MarkerArray로 시각화 |
| `tf_distance_viz_node` | TF 기반 거리 시각화 (디버그용) |

### 2.2 데이터 흐름

1. 카메라 이미지 수신 -> fisheye rectification -> ArUco marker 검출
2. Board pose 추정 (IPPE + iterative refinement, ESKF prior 기반 candidate selection)
3. `board -> camera` pose를 PoseWithCovarianceStamped로 publish
4. Localization node가 `board -> camera -> base_link -> leader_rear` 변환 수행
5. ESKF에서 gyro predict + vision update -> `leader_rear -> base_link` odom/TF 출력
6. 출력된 pose가 다시 detector에 prior로 피드백 (closed-loop)

### 2.3 좌표계

| 프레임 | 정의 |
|--------|------|
| `board` | 마커 보드 기하 정의 프레임. +x 보드 우측, +y 위, +z 보드에서 follower 방향 |
| `leader_rear` | 제어용 프레임. board와 원점 동일, +x leader 전방, +y leader 좌측, +z 위 |
| `base_link` | follower 차량 바디 |
| `usb_cam` | 카메라 광학 프레임 |

board-leader_rear 변환은 순수 회전이며, static TF로 broadcast된다.

### 2.4 TF 체인

```
board ──(static)──> leader_rear ──(dynamic, ESKF 출력)──> base_link
                                                            |
                                                      (static TF)
                                                            |
                                                    usb_cam, imu_link
```

---

## 3. 모듈 구조

```
aruco_imu_eskf_localization/
  common/
    camera_calibration.py    # fisheye 교정 로드 및 rectification map 생성
    frame_conventions.py     # board <-> leader_rear 좌표 변환
  estimation/
    board_pose_estimator.py  # IPPE/iterative PnP, candidate gating/scoring/refinement
    gyro_relative_eskf.py    # gyro-led ESKF (12-state error-state)
  filters/
    __init__.py              # (PoseFilter 제거됨, 디렉토리 유지)
  nodes/
    aruco_detector_node.py   # detector + rectification + prior feedback
    relative_localization_node.py  # ESKF 필터 + delayed replay + odom/TF
    pose_rviz_marker_node.py       # RViz 시각화
    tf_distance_viz_node.py        # TF 거리 시각화
config/
  cam_intrinsic.yaml         # fisheye 카메라 교정 파라미터
  markers_board.yaml         # 3-marker 보드 기하 정의
  params.yaml                # 전체 노드 파라미터
```

---

## 4. 핵심 설계 결정

### 4.1 ESKF 구조

12차 error-state: `[dp(3), dv(3), dtheta(3), db_g(3)]`

- **predict**: gyro 적분으로 orientation, CV + damping으로 position/velocity
- **update**: vision pose 측정 (position + orientation 6D)
- 가속도계 미사용, accel bias 미추정 (현 단계에서는 불필요)

### 4.2 vision update 정책

- rotation residual이 `vision_rotation_gate_deg` (기본 10도) 초과 시 **전체 pose update reject**
- position-only fallback 없음 (회전이 이상하면 translation도 신뢰 불가)

### 4.3 fisheye rectification

- `cv2.fisheye.calibrate()`로 교정된 equidistant 모델 사용
- rectified image에서 marker 검출 및 PnP 수행
- PnP는 `rectified_camera_matrix` + `zero_distortion`으로 standard pinhole처럼 동작

### 4.4 ESKF prior -> detector 피드백

- detector가 ESKF의 filtered pose를 subscribe
- `board -> camera` prior로 변환 후:
  - iterative solvePnP의 initial guess로 사용
  - candidate selection에서 prior와 회전이 가까운 후보 우선 선택 (reference rotation gate)
- prior가 없으면 이전 detection 결과로 fallback

### 4.5 camera extrinsic 단일 원천

- detector와 localization 모두 `base_link <- usb_cam` TF lookup으로 camera extrinsic 해결
- params.yaml에서 중복 파라미터 제거됨

### 4.6 measurement covariance

- anisotropic: depth(종방향) > vertical > lateral 순으로 불확실성 크게 설정
- marker 수, reprojection RMSE, view angle, image area, single-marker fallback 여부에 따라 scaling
- single-marker fallback은 기본 비활성 (`min_markers_for_board: 2`)

### 4.7 temporal gate 분리

- `single_marker_prior_timeout_sec` (0.25s): single-marker fallback용 prior 유효 시간
- `motion_gate_prior_timeout_sec` (1.0s): multi-marker temporal consistency gate 유효 시간
- 서로 독립적으로 동작

---

## 5. 빌드 및 실행

### 빌드

```bash
cd ~/xycar_ws
source /opt/ros/$ROS_DISTRO/setup.zsh
colcon build --packages-select aruco_imu_eskf_localization --symlink-install
```

### 실행

```bash
source ~/xycar_ws/install/setup.zsh
ros2 launch aruco_imu_eskf_localization relative_localization.launch.py
```

### 테스트

```bash
cd ~/xycar_ws/src/localization/aruco_imu_eskf_localization
python3 -m pytest test/ -v
```

### 주요 launch 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `enable_camera` | true | 카메라 드라이버 시작 여부 |
| `enable_imu` | true | IMU 드라이버 시작 여부 |
| `enable_vehicle_static_tf` | true | 차량 static TF 시작 여부 |
| `enable_pose_viz` | true | RViz marker 시각화 |
| `publish_aruco_tf` | false | detector 단계 board->camera TF 출력 |

---

## 6. 핵심 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/image_raw` | Image | 입력 | 카메라 원본 이미지 |
| `/imu` | Imu | 입력 | IMU 각속도/가속도 |
| `/localization/aruco/board_pose` | PoseWithCovarianceStamped | detector -> filter | board 기준 camera pose 측정 |
| `/localization/relative/pose` | PoseWithCovarianceStamped | filter -> detector | ESKF filtered pose (prior 피드백) |
| `/localization/relative/odom` | Odometry | 출력 | board 기준 filtered odom |
| `/localization/leader_rear/odom` | Odometry | 출력 | leader_rear 기준 filtered odom (제어용) |
| `/localization/leader_rear/pose` | PoseWithCovarianceStamped | 출력 | leader_rear 기준 filtered pose |
| `/localization/aruco/debug_image` | Image | 출력 | rectified 디버그 오버레이 이미지 |

---

## 7. 현재 한계 및 추후 개선 로드맵

### 7.1 중요도 상 -- leader 움직임 대응

| 항목 | 현재 상태 | 개선 방향 |
|------|-----------|-----------|
| **leader motion compensation** | leader 정지 가정 | V2V 통신으로 leader yaw rate/speed를 받아 상대 dynamics 보정 |
| **wheel odom / vehicle speed** | 미사용 | follower wheel odom 통합으로 pure rotation vs translation 구분, ZUPT/NHC 적용 |
| **ESKF C++ 이전** | Python 구현 | moving leader 단계부터 deterministic latency가 중요해지므로 C++ backend 권장 |
| **timestamp alignment 강화** | 2초 replay buffer | moving leader에서는 vision 지연이 상대 위치 오차로 직결, 더 정밀한 시간 보상 필요 |

### 7.2 중요도 중 -- 정확도/강건성 개선

| 항목 | 현재 상태 | 개선 방향 |
|------|-----------|-----------|
| **파라미터 튜닝** | 초기 안정화 위주 보수적 값 | 실제 주행 데이터 기반 process noise, measurement covariance, gate threshold 최적화 |
| **non-coplanar board** | 평면 3-marker | 중심 마커를 20~30mm 앞으로 띄워 planar ambiguity 근본 감소 |
| **UWB fallback** | 미구현 | vision dropout 시 gap 유지용 거리 센서 |
| **angular velocity dependent covariance** | 미구현 | 고 yaw rate 시 vision translation covariance 자동 inflation |
| **Mahalanobis gate** | reprojection RMSE + heuristic gate | 통계적 Mahalanobis distance 기반 outlier rejection |
| **accelerometer 통합** | gyro only | 가속도계 기반 translational dynamics, `db_a` 상태 확장 (15차) |

### 7.3 중요도 하 -- 장기 고도화

| 항목 | 설명 |
|------|------|
| roll/pitch 6D full estimation | 현재는 yaw + position 위주, 필요시 확장 |
| visual-inertial preintegration | 고급 모델, 현 프로젝트 범위 대비 과투자 |
| AprilTag/ChArUco 런타임 전환 비교 | 대안 마커 시스템 실험 |
| 통합 테스트 | detector -> filter -> output 전체 파이프라인 end-to-end 테스트 |
| `relative_localization_node` 단위 테스트 | delayed replay, extrapolation 등 복잡한 상태 관리 경로 검증 |

---

## 8. 카메라 교정

별도 도구 `cam_intrinsic_calibrator/calibrate.py`가 포함되어 있다.

- `cv2.fisheye.calibrate()` 기반 equidistant fisheye 모델
- 실시간 체스보드 검출 + 캡처 + 교정 + YAML 저장
- 교정 결과에 `camera_model: fisheye` 필드 포함

```bash
cd ~/xycar_ws/src/localization/cam_intrinsic_calibrator
pip install -r requirements.txt
python3 calibrate.py --device /dev/video0 --output cam_intrinsic.yaml
```

교정 후 결과 파일을 `aruco_imu_eskf_localization/config/cam_intrinsic.yaml`에 복사.

---

## 9. 마커 보드 사양

| 항목 | 값 |
|------|-----|
| 사전 | DICT_6X6_250 |
| 마커 수 | 3 (ID 0, 1, 2) |
| ID 0/1 | 124mm, 좌/우 상단 |
| ID 2 | 62mm, 하단 중앙 (원점) |
| 배치 | coplanar (z=0), ID 2 중심 기준 ID 0 (-119, 93, 0)mm, ID 1 (119, 93, 0)mm |

---

## 10. 개발 이력 요약

| 단계 | 내용 |
|------|------|
| 초기 구현 | detector + pose filter (filterpy 기반), board frame 직접 사용 |
| frame 정리 | board/leader_rear 좌표계 분리, base 기준 gate, stale prior timeout |
| IMU 결합 | gyro-led ESKF 도입, 100 Hz 예측 출력, IMU TF lookup |
| fusion 강화 | rotation gate 초과 시 full reject, anisotropic covariance, single-marker 비활성, prior timeout 분리, dead code 제거 |
| rectification + prior | fisheye rectification 도입, ESKF pose -> detector prior 피드백, camera extrinsic TF 통합, orientation process noise 수정, iterative refinement + reference rotation gate |
