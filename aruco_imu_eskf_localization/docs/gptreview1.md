# ArUco Board Review Notes

이 문서는 현재 패키지의 좌표계 계약과 pose selection 규칙을 한 번에 볼 수 있도록 정리한 구현 기준 문서다.

## 1. Frame Contract

### `board`
- 용도: ArUco board 기하 정의와 PnP 계산용 기준 프레임
- 원점: small marker `ID 2` 중심
- 축:
  - `+x`: follower가 board를 정면에서 볼 때 board의 오른쪽
  - `+y`: 위쪽
  - `+z`: board에서 follower 쪽으로 나오는 법선

즉, 모든 marker corner는 `z=0` 평면에 놓이고, front/back 판정은 `z_B`로 한다.

### `leader_rear`
- 용도: 제어와 디버깅용 기준 프레임
- 원점: `board`와 동일
- 축:
  - `+x`: follower 방향, 즉 차간거리 축
  - `+y`: leader 좌측, 즉 횡오프셋 축
  - `+z`: 위쪽

고정 회전 관계는 아래와 같다.

```text
x_leader_rear =  z_board
y_leader_rear = -x_board
z_leader_rear =  y_board
```

## 2. Pose Estimation Rules

모든 측정은 항상 같은 `board` frame pose로 다룬다. 개별 marker pose는 내부 fallback 용도일 뿐 외부 인터페이스가 아니다.

### 3개 또는 2개 marker
- 보이는 marker들의 board corner를 모두 모아 `solvePnPGeneric(..., SOLVEPNP_IPPE)`로 후보를 만든다.
- IPPE 후보가 안 나오면 `solvePnP(..., SOLVEPNP_ITERATIVE)`를 fallback으로 사용한다.
- 후보 해는 모두 `board -> camera` pose로 평가한다.

### 1개 marker
- 반드시 `solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)`를 사용한다.
- 나온 후보 해를 그대로 쓰지 않고 모두 `board -> camera` pose로 환산한다.
- prior가 없으면 1-marker initialization은 하지 않는다.

### Candidate Gates
후보 해는 아래 순서대로 통과해야 한다.

1. front-half-space gate
   - `z_B > 0.05 m`
2. view-angle gate
   - `acos(z_B / ||p_B||) <= 75 deg`
3. feasible box gate in `leader_rear`
   - `0.10 <= x <= 3.50`
   - `|y| <= 1.00`
   - `-0.50 <= z <= 0.80`
4. temporal gate
   - `Δposition <= 0.35 m`
   - `Δrotation <= 55 deg`
   - `Δyaw <= 40 deg`

남은 후보 중 최종 선택은 reprojection RMSE와 prior 차이를 합친 점수의 최소값으로 한다.

## 3. ROS Output Contract

### Detector
- 입력: `/image_raw`, camera calibration
- 출력:
  - `/localization/aruco/board_pose`
  - optional TF `board -> camera`

detector는 여전히 `board` 기준 측정치를 publish한다.

### Localization
- detector의 `board -> camera`를 받아 `board -> base_link`와 `leader_rear -> base_link`를 모두 계산한다.
- 메시지 출력:
  - 호환 출력: `/localization/relative/odom`, `/localization/relative/pose`
  - 제어 출력: `/localization/leader_rear/odom`, `/localization/leader_rear/pose`
- TF 출력:
  - static TF `board -> leader_rear`
  - dynamic TF `leader_rear -> base_link`

`base_link`에는 하나의 동적 부모만 두기 위해 `board -> base_link` TF는 publish하지 않는다.

## 4. RViz Interpretation

기본 RViz marker 입력은 `/localization/leader_rear/odom`이다.

따라서 marker 텍스트는 아래 의미로 읽는다.

- `gap x`: follower와 leader rear 사이 거리
- `lateral y`: follower의 좌우 오프셋
- `height z`: 높이 차
- `roll/pitch/yaw`: `leader_rear` 기준 follower 자세

## 5. Current Scope

- 현재 단계는 detector-first 안정화가 목표다.
- IMU 융합과 100 Hz extrapolated output은 다음 단계 과제다.
- hardware board는 기존 평면 3-marker를 유지한다.
