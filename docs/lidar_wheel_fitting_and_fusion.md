# LiDAR Wheel Fitting 및 Fusion 아키텍처

이 문서는 follower 2D LiDAR가 관측하는 leader 타이어 뒷면 선분을 이용해
leader 상대 pose를 추정하고, 이후 ArUco/IMU/GPS와 어떻게 합칠지 정리하는
계약이다.

## 1. 패키지 역할

- `follower_lidar_localization`: follower LiDAR 기반 leader wheel fitting.
  센서 측위 알고리즘 패키지이며, V2V 통신이나 controller bridge를 맡지 않는다.
- `relative_localization_eskf`: ArUco+IMU 상대 pose filter. v1에서는
  LiDAR wheel fitting을 직접 fusion하지 않는다.
- `comm/platoon_localization`: 리더/팔로워 공용 TF, V2V 상대상태 bridge,
  실행 편의 launch만 둔다. LiDAR fitting 알고리즘은 넣지 않는다.

기존 `relative_localization_eskf` 패키지 이름 정리는 별도 작업으로
분리한다. 이번 단계에서는 새 LiDAR 패키지와 문서만 추가한다.

## 2. 관측 모델

현재 RViz scan 관측에서는 각 바퀴가 원/박스 전체가 아니라 타이어 뒷면
폭 `110 mm` 정도의 짧은 선분으로 보인다. 따라서 v1 model은 wheel center가
아니라 타이어 rear edge segment를 fitting 대상으로 둔다.

Leader 기준 기본 기하:

| 항목 | 값 |
|---|---:|
| wheelbase | `0.720 m` |
| track width | `0.700 m` |
| wheel radius | `0.1325 m` |
| visible segment length | `0.110 m` |

`leader/base_link`는 후륜축 중심, 바퀴 축 높이이다. 각 visible segment는
leader frame에서 다음 위치에 둔다.

```text
rear_left:   x=-0.1325, y=+0.350
rear_right:  x=-0.1325, y=-0.350
front_left:  x=+0.5875, y=+0.350
front_right: x=+0.5875, y=-0.350
```

각 segment 방향은 leader y축 방향이고 길이는 `0.110 m`이다.

## 3. v1 출력 계약

입력:

| 입력 | 타입 | 의미 |
|---|---|---|
| `/follower/scan` | `sensor_msgs/LaserScan` | follower 2D LiDAR scan |
| `/follower/localization/leader_rear/odom` | `nav_msgs/Odometry` optional | ArUco/IMU prior |
| TF `follower/base_link <- follower/lidar` | TF | scan point를 follower 기준으로 변환 |
| TF/param `leader/base_link -> leader/leader_rear` | static geometry | leader base/rear 변환 |

출력:

| 출력 | 타입 | frame 의미 |
|---|---|---|
| `/follower/localization/lidar_wheels/leader_base_detection` | `nav_msgs/Odometry` | `follower/base_link -> leader/base_link_lidar` |
| `/follower/localization/lidar_wheels/odom` | `nav_msgs/Odometry` | `leader/leader_rear -> follower/base_link` |
| `/follower/localization/lidar_wheels/markers` | `visualization_msgs/MarkerArray` | candidate/model/status RViz overlay |
| `/follower/localization/lidar_wheels/diagnostics` | `diagnostic_msgs/DiagnosticArray` | fit/tracker 상태 |

Detection 결과는 TF로 publish하지 않는다. TF는 차량 내부 고정 기하와 최종
fused pose만 담당하고, LiDAR wheel fitting은 debug measurement odom으로 둔다.

## 4. Fitting 단계

1. LaserScan point를 `follower/base_link` 2D point로 변환한다.
2. ROI를 적용한다. 기본값은 `x=0.15..2.50 m`, `abs(y)<=1.20 m`.
3. scan adjacency gap으로 cluster를 나눈다.
4. 각 cluster에 PCA line fitting을 하고 다음 조건으로 후보를 고른다.
   - point 수 충분
   - line length `0.05..0.18 m`
   - line RMS residual 작음
5. 후보 line segment를 leader wheel prior segment에 robust assignment한다.
6. SE(2) pose를 계산한다.

가시 segment 수에 따른 정책:

- 3~4개: full pose measurement로 사용.
- 2개: geometry와 prior가 충분할 때 pose update로 사용.
- 1개: 새 pose를 바로 만들지 않고 tracker prediction/coasting에 맡긴다.

## 5. Tracking 정책

Tracker state는 다음 네 가지다.

| 상태 | 의미 |
|---|---|
| `TRACKING` | fitting이 정상적으로 pose를 갱신 중 |
| `COASTING` | 가림/선회/dropout으로 measurement가 약해져 예측으로 버티는 중 |
| `REACQUIRING` | 다시 보인 후보를 검증 중이며 아직 snap 금지 |
| `LOST` | 오래 끊겨 pose를 더 이상 publish하면 안 되는 상태 |

Tracker는 `[x, y, yaw, vx, vy, yaw_rate]` constant-velocity 형태로 운용한다.
v1 구현은 debug 안정성을 우선해 measurement smoothing과 gate 기반
state machine을 사용한다.

재검출 정책:

- predicted pose 근처 후보만 accept한다.
- 큰 jump는 바로 odom에 반영하지 않고 `REACQUIRING`으로 표시한다.
- 연속 2 frame 이상 통과하면 `TRACKING`으로 복귀한다.
- `COASTING` 중 covariance scale은 커진다.

## 6. Fusion 단계 계획

v1:

- LiDAR wheel fitting은 ESKF에 넣지 않는다.
- RViz에서 ArUco odom과 LiDAR wheel odom을 나란히 비교한다.
- 선회/가림/dropout/reacquire 상태가 납득되기 전까지 controller 입력으로
  쓰지 않는다.

v2:

- `relative_localization_eskf`에 optional LiDAR wheel odom input을 추가한다.
- 기본값은 `enable_lidar_wheel_fusion:=false`.
- LiDAR는 position+yaw 보조 measurement로만 사용한다.
- visible segment count, residual, covariance, tracker state로 gate한다.

Fusion 역할:

- ArUco: marker board identity와 가까운 거리 pose anchor.
- IMU: short-term yaw propagation.
- LiDAR wheel: GPS 없는 실내에서 leader geometry 기반 yaw/relative pose 보조.
- GPS/RTK: 실외에서 상대 position/velocity drift 보조.

최종 controller 계약은 계속 `/platoon/relative_leader/state`이다. Controller는
어떤 센서가 fusion에 쓰였는지 몰라야 한다.

## 7. 실행

LiDAR driver가 이미 떠 있으면:

```bash
ros2 launch follower_lidar_localization leader_wheel_fitting.launch.py
```

LiDAR driver까지 같이 띄우려면:

```bash
ros2 launch follower_lidar_localization leader_wheel_fitting.launch.py \
  include_lidar_driver:=true
```

실내 상대측위 launch 없이 LiDAR fitting만 단독으로 볼 때는 LiDAR static TF도
같이 띄운다.

```bash
ros2 launch follower_lidar_localization leader_wheel_fitting.launch.py \
  include_lidar_driver:=true \
  publish_lidar_static_tf:=true \
  use_aruco_prior:=false
```

`publish_lidar_static_tf:=true`는 기존 `relative_localization.launch.py`
또는 다른 static TF launch와 동시에 켜지 않는다. 같은 child frame을 두 번
publish하면 TF tree가 충돌한다.

RViz:

- Fixed Frame: `follower/base_link`
- LaserScan: `/follower/scan`
- MarkerArray: `/follower/localization/lidar_wheels/markers`
- Odometry: `/follower/localization/lidar_wheels/leader_base_detection`
- Odometry: `/follower/localization/lidar_wheels/odom`

## 8. 검증 기준

- scan 위에 파란 candidate segment가 타이어 뒷면 4개에 붙어야 한다.
- 초록 model segment가 파란 candidate와 겹쳐야 한다.
- `/follower/localization/lidar_wheels/leader_base_detection`이 갑자기 튀지 않아야 한다.
- 선회/가림에서는 `COASTING` 또는 `REACQUIRING`으로 표시되어야 한다.
- 다시 직진으로 돌아왔을 때 `TRACKING`으로 부드럽게 복귀해야 한다.
