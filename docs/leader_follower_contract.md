# Leader/Follower GPS 통합 계약

이 문서는 leader/follower 차량을 동시에 띄웠을 때 토픽, TF, GPS 좌표계, fusion 입력의 의미가 서로 어긋나지 않도록 고정하는 계약이다. 코드 구현은 이 계약을 기준으로 한다.

## 0. 핵심 원칙

- Leader와 follower의 GPS 절대위치는 공통 `map` ENU frame에서 계산한다.
- Follower는 절대 위치와 상대 위치를 둘 다 가진다. 절대 위치는 공통 waypoint/route 진행도 판단에 쓰고, 상대 위치는 leader와의 간격/횡방향 정렬/착 붙는 추종에 쓴다.
- ESKF fused relative state는 `leader/leader_rear -> follower/base_link` 상대좌표이다. 이것은 platooning gap control의 기준이다.
- `/follower/localization/global/odom`은 `map -> follower/base_link` 절대좌표이다. 이것은 waypoint follower, RViz global overlay, leader/follower route progress 비교에 쓴다.
- `map`은 더 이상 단순 debug frame이 아니다. Leader와 follower가 같은 웨이포인트를 로컬에 저장하고 같은 ENU origin을 쓰면 `map`은 route 운용 frame이다.
- GPS fusion은 ArUco+IMU 상대 localization을 대체하지 않는다. GPS는 상대 position/velocity drift를 줄이는 보조 측정이다.
- Orientation은 GPS로 보정하지 않는다. Follower orientation은 ArUco+IMU ESKF가 담당한다.
- Leader는 절대위치 기반 `map -> leader/base_link`를 안정적으로 제공해도 된다. 이 경우 follower도 절대 GPS를 `map -> follower/follower_gps`로 만들고, 둘의 차이만 ESKF에 넣는다.
- Leader/follower 동시 운용에서는 route tracking과 platooning을 분리한다. Route tracking은 각자 로컬 waypoint와 자기 absolute odom을 보고, platooning은 V2V leader state와 relative odom을 본다.

## 1. 좌표계와 TF

모든 frame id는 ROS namespace와 별개로 frame 문자열 자체에 prefix를 포함한다.

| 차량 | 프레임 | 의미 |
|---|---|---|
| leader | `leader/base_link` | leader 후륜축 중심 또는 leader odom 기준 바디 프레임 |
| leader | `leader/leader_rear` | ArUco marker board 기준 제어 프레임 |
| leader | `leader/board` | ArUco board 기하 프레임 |
| follower | `follower/base_link` | follower 후륜축 중심 |
| follower | `follower/usb_cam` | follower camera optical frame |
| follower | `follower/base_imu_link` | follower IMU frame |
| follower | `follower/follower_gps` | follower GPS antenna frame |
| global | `map` | 공통 WGS84 origin에서 투영한 ENU frame |

Follower static TF 기본값:

| parent | child | translation xyz [m] | quaternion xyzw |
|---|---|---:|---|
| `follower/base_link` | `follower/base_imu_link` | `(0.060, 0.000, 0.095)` | `(1, 0, 0, 0)` |
| `follower/base_link` | `follower/usb_cam` | `(0.270, 0.000, 0.135)` | `(-0.5, 0.5, -0.5, 0.5)` |
| `follower/base_link` | `follower/follower_gps` | `(0.1625, 0.000, 0.135)` | `(0, 0, 0, 1)` |

상대 localization의 동적 TF는 `leader/leader_rear -> follower/base_link`이다. Leader workspace는 `map -> leader/base_link` Odometry와 `leader/base_link -> leader/leader_rear` TF를 제공해야 한다.

주의사항:

- ROS namespace(`/follower`)와 TF frame prefix(`follower/`)는 별개이다. 둘 중 하나만 맞추면 안 된다.
- Leader frame 문자열은 follower launch에서 기본 `leader/...`로 참조한다. Leader workspace도 같은 frame id를 써야 한다.
- `follower/follower_gps`는 GPS antenna 위치이다. 차량 제어 기준점은 `follower/base_link`이다.
- GPS antenna z는 현재 카메라 높이와 같은 값으로 둔다. 실제 마운트가 바뀌면 static TF를 먼저 고쳐야 한다.
- `leader/leader_rear -> follower/base_link` TF와 `/follower/localization/leader_rear/odom`은 같은 의미여야 한다.
- `/follower/localization/global/odom`은 TF를 broadcast하지 않는다. TF tree의 상대 localization과 global debug overlay가 충돌하지 않게 Odometry topic으로만 제공한다.

## 2. Topic 계약

Follower namespace는 기본 `follower`이다. 절대 토픽을 피하고, follower 내부 출력은 `/follower/...` 아래에 둔다.

| 토픽 | 타입 | 생산자 | 소비자 | 의미 |
|---|---|---|---|---|
| `/follower/image_raw` | `sensor_msgs/Image` | camera driver | ArUco detector | follower camera image |
| `/follower/imu` | `sensor_msgs/Imu` | IMU driver | ESKF | follower IMU |
| `/follower/ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | ublox | GPS odom node | follower GPS antenna LLA |
| `/follower/ublox_gps_node/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | ublox | GPS odom node | ENU velocity |
| `/follower/localization/gps/odom` | `nav_msgs/Odometry` | GPS odom node | relative GPS node | `map -> follower/follower_gps` |
| `/leader/localization/gps/odom` | `nav_msgs/Odometry` | leader workspace | relative GPS node | `map -> leader/base_link` |
| `/follower/localization/gps_relative/odom` | `nav_msgs/Odometry` | relative GPS node | ESKF/debug | `leader/leader_rear -> follower/base_link` GPS measurement |
| `/follower/localization/leader_rear/odom` | `nav_msgs/Odometry` | ESKF | controller/debug | fused relative odom |
| `/follower/localization/leader_rear/pose` | `geometry_msgs/PoseWithCovarianceStamped` | ESKF | debug | fused relative pose |
| `/follower/localization/global/odom` | `nav_msgs/Odometry` | absolute follower odom node | waypoint/RViz/debug | `map -> follower/base_link` |

`/follower/localization/gps/odom`과 `/leader/localization/gps/odom`의 `twist.linear`는 `map` ENU frame 성분으로 해석한다. `twist.angular`가 제공되는 경우에는 각 차량의 `child_frame_id` frame 성분으로 해석한다.

`/follower/localization/gps_relative/odom`의 `pose.position`과 `twist.linear`는 모두 `leader/leader_rear` frame 성분으로 해석한다. Orientation은 GPS fusion에서 사용하지 않는다.

`/follower/localization/global/odom`의 `pose`와 `twist.linear`는 `map` frame 성분이다. `child_frame_id`는 `follower/base_link`이다.

주의사항:

- `/leader/localization/gps/odom`은 절대 토픽이다. Follower namespace 안으로 remap하지 않는다.
- Follower 내부 토픽은 relative name으로 launch에 넣고, node namespace로 `/follower/...`를 만든다.
- Ublox node 이름이 다르면 `fix_topic`, `fix_velocity_topic` launch/param을 실제 토픽에 맞춘다.
- `fix_velocity`가 없거나 stamp가 오래되면 GPS odom은 큰 velocity covariance를 넣고, ESKF는 velocity update를 사실상 약하게 받는다.
- `NavSatFix.status.status == 0`은 일반 GPS fix이다. NTRIP client가 valid RTCM packet을 받는 것과 RTK float/fix 상태는 다르다.
- RTK fixed가 아니어도 covariance가 합리적이면 GPS debug/보조 fusion은 가능하지만, gate와 covariance를 보수적으로 둔다.
- Follower waypoint follower가 절대 위치를 소비할 때는 `/follower/localization/gps/odom`이 아니라 `/follower/localization/global/odom`을 우선 사용한다. 전자는 antenna 위치이고, 후자는 차량 기준점이다.

## 3. GPS ENU 변환 계약

Leader와 follower는 같은 map origin을 사용해야 한다.

```yaml
map_frame: map
map_origin_lat_deg: <same value on both vehicles>
map_origin_lon_deg: <same value on both vehicles>
map_origin_alt_m: <same value on both vehicles>
```

`NavSatFix(latitude, longitude, altitude)`는 WGS84 ECEF로 변환한 뒤 origin ECEF를 빼고 ENU 회전 행렬로 투영한다. 외부 `pyproj`/`geographiclib` 의존성은 쓰지 않는다.

Ublox `fix_velocity`는 driver 코드 기준 ENU 속도이다. GPS odom node는 이 값을 `map` frame velocity로 그대로 전달한다.

주의사항:

- `map_origin_*` 기본값 `0, 0, 0`은 실주행용 값이 아니다. 실제 테스트 장소 근처의 고정 origin을 leader/follower 양쪽에 동일하게 넣는다.
- Origin은 주행 중 바꾸지 않는다. 한쪽만 바뀌면 상대 GPS가 즉시 깨진다.
- Leader와 follower가 서로 다른 launch 파일을 쓰더라도 `map_frame`, `map_origin_lat_deg`, `map_origin_lon_deg`, `map_origin_alt_m` 값은 반드시 일치해야 한다.
- ENU `x`는 East, `y`는 North, `z`는 Up이다.
- Altitude 품질이 낮으면 z covariance를 크게 둔다. 현재 follower 제어에서 z는 카메라/차량 기하 기준으로 보는 것이 더 안전하다.
- GPS 절대좌표가 drift해도 leader/follower가 같은 기준에서 drift하면 상대값은 더 안정적일 수 있다. 반대로 두 수신기의 bias가 다르게 움직이면 상대값도 흔들린다.

## 4. Relative GPS 측정 생성

Relative GPS node는 다음 값을 사용한다.

- follower: `/follower/localization/gps/odom`
- leader: `/leader/localization/gps/odom`
- leader rear offset: TF `leader/base_link -> leader/leader_rear`
- follower base offset: TF `follower/base_link -> follower/follower_gps`
- follower yaw/orientation: 최신 fused odom `/follower/localization/leader_rear/odom`

Follower GPS antenna 위치는 최신 fused orientation을 이용해 `follower/base_link` 위치로 환산한다. Leader는 leader odom orientation과 `leader/base_link -> leader/leader_rear` TF로 `leader/leader_rear` 위치를 계산한다.

GPS relative position은 다음과 같이 계산한다.

```text
p_rel = R_map_leader_rear^T * (p_map_follower_base - p_map_leader_rear)
```

GPS relative velocity도 같은 frame의 시간미분으로 publish한다. Leader 기준 frame이 회전하므로 단순 속도 차이 회전값에서 `omega_leader_rear x p_rel` 항을 뺀다. Lever-arm velocity 보정은 가능한 경우 angular velocity를 사용하고, 불가능하면 선속도 차이를 사용한다.

주의사항:

- Follower GPS antenna 위치를 `follower/base_link`로 바꿀 때 최신 fused orientation을 쓴다. 그래서 ESKF가 아직 초기화되지 않았으면 relative GPS가 나오지 않는다.
- Leader rear 위치는 leader odom orientation과 `leader/base_link -> leader/leader_rear` TF로 계산한다.
- Leader odom orientation이 틀리면 상대 GPS position이 leader frame에서 잘못 회전된다.
- Leader angular velocity가 없으면 lever-arm velocity와 회전 frame 속도 보정이 약해진다. 저속에서는 영향이 작지만 회전 중에는 debug에서 차이가 보일 수 있다.
- GPS relative covariance는 follower GPS covariance와 leader odom covariance를 합산해 회전한다. covariance가 0이거나 NaN이면 fusion 품질이 망가진다.
- `/follower/localization/gps_relative/odom`은 RViz/debug overlay의 1차 확인 대상이다. 이 값이 납득되지 않으면 `enable_gps_fusion`을 켜지 않는다.

## 5. Fusion 조건

GPS fusion은 기본 비활성이다.

```yaml
enable_gps_fusion: false
```

다음 조건이 모두 만족될 때만 ESKF fused 출력에 반영한다.

- `enable_gps_fusion == true`
- ArUco+IMU ESKF가 이미 초기화됨
- leader/follower GPS odom stamp가 timeout 안에 있음
- 필요한 TF lookup 성공
- GPS covariance가 finite이고 양수
- position/velocity innovation gate 통과

GPS update는 position+velocity만 보정한다. Orientation update는 ArUco+IMU 경로가 담당한다. 속도 상태는 유지하며, 카메라 지연 replay와 GPS velocity 보정에 계속 사용한다.

주의사항:

- 초기 pose는 ArUco+IMU 경로가 잡는다. GPS만으로 ESKF를 초기화하지 않는다.
- GPS fusion을 켜기 전에는 `/follower/localization/gps_relative/odom`과 `/follower/localization/leader_rear/odom`을 RViz에서 나란히 비교한다.
- `gps_position_gate_m`, `gps_velocity_gate_mps`가 너무 작으면 정상 GPS도 reject된다. 너무 크면 잘못된 GPS jump가 들어온다.
- `gps_min_variance`는 covariance 0으로 인한 과신을 막는 하한이다.
- `gps_max_velocity_variance`보다 큰 velocity covariance는 velocity update에서 제외한다.
- GPS update는 orientation과 gyro bias Kalman gain을 0으로 막는다. GPS가 yaw를 끌고 가면 계약 위반이다.
- Camera delay replay와 GPS stamp replay가 같이 동작하므로 stamp가 미래/과거로 튀는 센서는 먼저 고쳐야 한다.

## 6. Absolute follower odom 계약

Absolute follower odom node는 다음 입력을 합성한다.

- leader absolute odom: `/leader/localization/gps/odom` (`map -> leader/base_link`)
- leader rear offset: TF `leader/base_link -> leader/leader_rear`
- follower relative fused odom: `/follower/localization/leader_rear/odom` (`leader/leader_rear -> follower/base_link`)

출력은 다음이다.

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/follower/localization/global/odom` | `nav_msgs/Odometry` | `map -> follower/base_link` 절대 pose/velocity |

계산식:

```text
T_map_follower_base =
  T_map_leader_base * T_leader_base_leader_rear * T_leader_rear_follower_base
```

주의사항:

- 이 odom은 follower가 같은 waypoint를 로컬에서 따라갈 때 쓰는 절대 pose이다.
- Leader absolute odom이 끊기면 `/follower/localization/global/odom`도 멈춘다.
- Relative ESKF가 초기화되지 않으면 absolute follower odom도 나오지 않는다.
- `/follower/localization/global/odom`은 GPS antenna 위치가 아니라 `follower/base_link` 위치이다.
- Leader yaw가 틀리면 follower global pose도 같이 틀어진다.
- Follower absolute orientation은 leader absolute orientation과 follower relative orientation을 합성한 값이다. GPS heading 단독 추정값이 아니다.
- 이 토픽을 controller가 쓰는 경우에도 leader와의 간격 유지에는 `/follower/localization/leader_rear/odom` 또는 V2V relative 상태를 같이 봐야 한다.

## 7. Shared waypoint + V2V 운용 계약

미리 따놓은 같은 waypoint를 leader/follower 양쪽 로컬에 저장하는 운용은 가능하다. 이 경우 구조는 다음처럼 나눈다.

- Leader: 자기 `/leader/localization/gps/odom` 기준으로 waypoint를 추종한다.
- Follower: 자기 `/follower/localization/global/odom` 기준으로 같은 waypoint의 진행도를 계산한다.
- V2V: leader의 현재 진행도, 속도, 정지/출발 상태, emergency stop 상태를 공유한다.
- Platooning: follower는 route 진행도만 자기 위치로 맞추지 않고, leader와의 상대 간격을 `/follower/localization/leader_rear/odom`으로 닫는다.

권장 V2V 토픽 계약:

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/v2v/leader/odom` | `nav_msgs/Odometry` | leader가 공유하는 `map -> leader/base_link` odom |
| `/v2v/leader/route_state` | 추후 custom msg 또는 `std_msgs/String` 임시 | waypoint index, s progress, target speed, driving/stopped state |
| `/v2v/leader/command_state` | 추후 custom msg 또는 `std_msgs/String` 임시 | start/stop/emergency/mission state |

주의사항:

- 같은 DDS network에서 모든 토픽이 보인다고 해서 모든 토픽을 소비하면 안 된다. 각 차량 내부 토픽은 namespace로 격리하고, 통신용 토픽은 `/v2v/...`로 의도적으로 공개한다.
- Waypoint 파일, map origin, waypoint 좌표계는 leader/follower가 동일해야 한다.
- Follower는 leader의 waypoint index를 그대로 맹신하지 말고 자기 global odom으로 nearest/progress sanity check를 해야 한다.
- Stop/start 동기화는 position controller보다 상위 mission state에서 먼저 맞춘다. Leader가 stopped인데 follower가 route speed만 보고 움직이면 안 된다.
- 붙어서 따라가는 핵심 제어량은 global waypoint error가 아니라 leader 상대거리, 상대속도, leader command state이다.
- V2V 지연이 커지면 follower는 마지막 leader speed를 유지하지 말고 감속 또는 hold 정책으로 들어간다.
- RViz에서는 global route tracking 오차와 relative gap 오차를 동시에 봐야 한다.

## 8. 운영 체크

기본 실행 후 다음 항목을 확인한다.

```bash
ros2 topic list | grep follower/localization
ros2 topic echo /follower/localization/gps/odom --once
ros2 topic echo /leader/localization/gps/odom --once
ros2 topic echo /follower/localization/gps_relative/odom --once
ros2 topic echo /follower/localization/global/odom --once
ros2 run tf2_ros tf2_echo leader/leader_rear follower/base_link
ros2 run tf2_ros tf2_echo follower/base_link follower/follower_gps
```

`/follower/localization/gps_relative/odom`이 나오지만 fused odom이 바뀌지 않는다면 `enable_gps_fusion` 값과 ESKF 초기화 여부를 먼저 확인한다.

Launch 예시:

```bash
ros2 launch aruco_imu_eskf_localization relative_localization.launch.py \
  robot_namespace:=follower \
  frame_prefix:=follower/ \
  leader_frame_prefix:=leader/ \
  map_origin_lat_deg:=<fixed_origin_lat> \
  map_origin_lon_deg:=<fixed_origin_lon> \
  map_origin_alt_m:=<fixed_origin_alt> \
  leader_gps_odom_topic:=/leader/localization/gps/odom \
  enable_absolute_follower_odom:=true \
  enable_gps_fusion:=false
```

Fusion을 켜기 전 최소 체크:

```bash
ros2 topic hz /follower/ublox_gps_node/fix
ros2 topic hz /follower/ublox_gps_node/fix_velocity
ros2 topic hz /follower/localization/gps/odom
ros2 topic hz /follower/localization/gps_relative/odom
ros2 topic hz /follower/localization/global/odom
ros2 topic echo /follower/ublox_gps_node/fix --once
ros2 topic echo /follower/localization/gps_relative/odom --once
```

문제별 확인 순서:

- NTRIP client에서 valid packet이 계속 보이지만 `NavSatFix.status.status`가 0이면 RTCM 수신은 되고 있으나 receiver가 아직 RTK 상태로 올라가지 않은 것이다.
- `/follower/localization/gps/odom`이 안 나오면 follower ublox `fix` topic, `NavSatFix.status`, `map_origin_*`를 확인한다.
- `/follower/localization/gps_relative/odom`이 안 나오면 leader odom, fused odom 초기화, `leader/base_link -> leader/leader_rear`, `follower/base_link -> follower/follower_gps` TF를 확인한다.
- `/follower/localization/global/odom`이 안 나오면 leader odom, fused relative odom, `leader/base_link -> leader/leader_rear` TF를 확인한다.
- GPS relative position이 반대로 움직이면 `map` ENU axis, leader orientation, frame prefix를 확인한다.
- GPS relative z가 이상하면 altitude covariance와 `follower/follower_gps` static TF z를 확인한다.
- Fusion을 켰을 때 fused odom이 튀면 즉시 `enable_gps_fusion:=false`로 돌리고 GPS relative odom/covariance/gate를 먼저 본다.

## 9. RViz debug overlay 기준

다음 항목은 RViz overlay에서 동시에 볼 수 있어야 한다.

- fused relative odom: `/follower/localization/leader_rear/odom`
- GPS relative odom: `/follower/localization/gps_relative/odom`
- ArUco measurement/debug: `/follower/localization/aruco/board_pose`
- follower GPS antenna absolute: `/follower/localization/gps/odom`
- follower base absolute: `/follower/localization/global/odom`
- leader GPS/odom absolute: `/leader/localization/gps/odom`
- TF: `leader/leader_rear`, `follower/base_link`, `follower/follower_gps`, `leader/base_link`

Overlay에서 fused와 GPS relative가 장시간 일정한 offset을 보이면 lever-arm, leader rear offset, map origin, camera extrinsic 순서로 확인한다. Offset이 천천히 커지면 GPS covariance/gate 또는 leader/follower time sync를 확인한다. 순간적으로 튀면 `NavSatFix` covariance, RTK 상태, NTRIP 연결 상태를 먼저 확인한다.

Global overlay에서 leader와 follower가 같은 waypoint line 위에 있어야 하고, relative overlay에서 follower가 leader rear 기준 목표 간격에 있어야 한다. 둘 중 하나만 맞는 상태는 정상 platooning으로 보지 않는다.
