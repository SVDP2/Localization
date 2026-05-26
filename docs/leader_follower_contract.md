# Leader/Follower Localization Contract

이 문서는 localization repository가 보장해야 하는 frame, topic, fusion 입력의 의미를 고정한다. Bringup 실행 명령과 wrapper ownership은 `comm/docs/bringup_launch_contract.md`를 우선 참조한다.

## Core frame policy

모든 frame id는 ROS namespace와 별개로 문자열 자체에 vehicle prefix를 포함한다.

| 차량 | 프레임 | 의미 |
|---|---|---|
| leader | `leader/base_link` | leader 후륜축 중심, leader vehicle body frame |
| leader | `leader/leader_rear` | ArUco board/rear reference frame |
| leader | `leader/board` | ArUco board geometry frame |
| leader | `leader/leader_gps` | leader GPS antenna frame |
| follower | `follower/base_link` | follower 후륜축 중심 |
| follower | `follower/usb_cam` | follower camera optical frame |
| follower | `follower/base_imu_link` | follower IMU frame |
| follower | `follower/lidar` | follower 2D LiDAR frame |
| follower | `follower/follower_gps` | follower GPS antenna frame |
| global | `map` | outdoor local UTM-offset meter frame |

Static TF는 차량 내부 기하만 표현한다.

- `leader/base_link -> leader/leader_gps`
- `leader/base_link -> leader/leader_rear`
- `leader/leader_rear -> leader/board`
- `follower/base_link -> follower/usb_cam`
- `follower/base_link -> follower/base_imu_link`
- `follower/base_link -> follower/follower_gps`
- `follower/usb_cam -> follower/lidar`

Detection 결과를 static TF로 붙이면 안 된다. ArUco/LiDAR/GPS 결과는 dynamic TF 또는 Odometry 측정값이어야 한다.

## Indoor relative localization

Current C++ ESKF output is leader-base centered.

- Primary fused odom: `/follower/localization/leader_base/odom`
  - frame: `leader/base_link`
  - child: `follower/base_link`
- Derived rear/reference odom: `/follower/localization/leader_rear/odom`
  - frame: `leader/leader_rear`
  - child: `follower/base_link`
- Dynamic TF: `leader/base_link -> follower/base_link`
- Detector input: `/follower/localization/aruco/board_pose`
- Detector prior feedback: `/follower/localization/relative/pose`

`leader_rear` remains important for ArUco board geometry and LiDAR prior/debug, but the final fused relative pose and controller bridge should use `leader/base_link` unless a node explicitly documents otherwise.

## Fusion source policy

The relative ESKF accepts multiple pose sources with source-specific gates.

- RTK/GPS odometry is the outdoor primary correction source when available.
- ArUco is the indoor initialization and main relative correction source.
- LiDAR wheel fitting is strict auxiliary support only.

LiDAR constraints:

- LiDAR cannot initialize the ESKF by default.
- LiDAR yaw update is disabled by default.
- LiDAR ICP yaw is disabled by default.
- LiDAR position/yaw gates and covariance gates are intentionally strict because suspension/body motion and partial occlusion can change the scan geometry quickly.

ArUco covariance already reflects marker quality:

- 3+ visible markers: strongest covariance.
- 2 visible markers: medium covariance.
- 1 visible marker: weaker covariance plus single-marker fallback inflation.
- Reprojection RMSE and image area further inflate covariance.

## Outdoor GPS/RTK localization

Outdoor uses `map` as a local UTM-offset meter frame. Leader and follower must load the same origin artifact from `platoon_localization/config/outdoor_utm_map.yaml`.

Derived GPS odometry:

| Topic | Meaning |
|---|---|
| `/leader/localization/gps/odom` | `map -> leader/base_link` |
| `/v2v/leader/odom` | V2V-forwarded leader odom |
| `/follower/localization/gps/odom` | `map -> follower/follower_gps` |
| `/follower/localization/global/odom` | `map -> follower/base_link` |

Outdoor autonomous mode must avoid conflicting TF ownership:

- Allowed: `map -> leader/base_link`, `map -> follower/base_link`, static vehicle/sensor TF.
- Forbidden: also publishing an indoor-style relative dynamic TF as the final outdoor authority.
- If ArUco/ESKF runs outdoors as observation/debug, set final-state and TF ownership so it does not conflict with the GPS/selector/fusion owner.

## Canonical control state

Controllers consume `/platoon/relative_leader/state`.

Rules:

- Only one publisher may own `/platoon/relative_leader/state` at a time.
- Indoor default owner: ArUco/IMU ESKF bridge from `/follower/localization/leader_base/odom`.
- LiDAR default owner: none; LiDAR publishes wheel-fitting odom/markers and can optionally publish a candidate state.
- Outdoor P0 owner: GPS relative node may own final state directly.
- Outdoor selector/fusion owner: selector or full ESKF owns final state; GPS/ArUco/LiDAR publish candidate topics only.

Recommended candidate topics when comparing sources:

- `/platoon/relative_leader/gps_state`
- `/platoon/relative_leader/aruco_state`
- `/platoon/relative_leader/lidar_state`

## Key topics

| Topic | Type | Producer | Meaning |
|---|---|---|---|
| `/follower/image_raw` | `sensor_msgs/Image` | camera driver | follower camera image |
| `/follower/imu` | `sensor_msgs/Imu` | IMU driver | follower IMU |
| `/follower/scan` | `sensor_msgs/LaserScan` | LiDAR driver | follower 2D LiDAR scan |
| `/follower/localization/aruco/board_pose` | `PoseWithCovarianceStamped` | ArUco detector | board/camera measurement |
| `/follower/localization/leader_base/odom` | `nav_msgs/Odometry` | relative ESKF | fused `leader/base_link -> follower/base_link` |
| `/follower/localization/leader_rear/odom` | `nav_msgs/Odometry` | relative ESKF | derived rear/reference odom |
| `/follower/localization/lidar_wheels/leader_base_detection` | `nav_msgs/Odometry` | LiDAR wheel fitting | strict auxiliary leader-base detection |
| `/follower/localization/lidar_wheels/markers` | `MarkerArray` | LiDAR wheel fitting | wheel/segment debug markers |
| `/leader/localization/gps/odom` | `nav_msgs/Odometry` | leader GPS odom | `map -> leader/base_link` |
| `/follower/localization/global/odom` | `nav_msgs/Odometry` | follower GPS odom | `map -> follower/base_link` |
| `/platoon/relative_leader/state` | `RelativeLeaderState` | selected bridge/fusion owner | canonical control input |

## Verification

Indoor checks:

```bash
ros2 topic info /platoon/relative_leader/state -v
ros2 topic echo /follower/localization/leader_base/odom --once
ros2 run tf2_ros tf2_echo leader/base_link follower/base_link
ros2 run tf2_ros tf2_echo leader/base_link leader/leader_rear
ros2 run tf2_ros tf2_echo follower/base_link follower/usb_cam
```

Outdoor checks:

```bash
ros2 topic echo /leader/localization/gps/odom --once
ros2 topic echo /v2v/leader/odom --once
ros2 topic echo /follower/localization/global/odom --once
ros2 topic info /platoon/relative_leader/state -v
```

If `/platoon/relative_leader/state` has multiple publishers, stop and fix launch ownership before driving.
