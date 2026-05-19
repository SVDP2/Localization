# EBIMU ROS2 Driver

E2BOX EBIMU-9DOFV6 IMU/AHRS를 ROS2 Humble에서 쓰기 위한 드라이버 작업 공간이다.

현재 구조는 역할을 분리한다.

```text
ebimu_tui.py       -> /dev/imu를 직접 열어서 EBIMU 설정/probe
ebimu_node         -> /dev/imu stream을 읽어서 ROS topic 발행
ebimu_driver.launch.py -> node 파라미터만 넘겨서 실행
```

TUI와 ROS driver는 같은 serial port를 동시에 열 수 없다. TUI로 설정을 맞춘 뒤 TUI를 종료하고 driver를 launch한다.

## 현재 상태

완료된 것:

- C++ ROS2 driver
- ASCII parser
- HEX/binary parser
- `sensor_msgs/msg/Imu` 발행
- `ebimu/status`, diagnostics 발행
- standalone Rich serial menu
- serial simulator
- `/dev/imu`, `/dev/gps` udev 문서

남은 실제 하드웨어 확인:

- `/dev/imu` udev rule 적용 확인
- TUI baud scan 확인
- 115200 ASCII 100Hz 수신 확인
- TUI에서 460800 전환 확인
- TUI에서 binary 500Hz 설정 확인
- launch 후 `/imu/data`, `/ebimu/status` 확인
- 축 방향, 부호, REP-103 정합성 확인

## udev

정책:

```text
IMU -> /dev/imu
GPS -> /dev/gps
```

설정:

```bash
sudo tee /etc/udev/rules.d/99-ins-serial.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="imu", GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gps", GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

확인:

```bash
ls -l /dev/imu /dev/gps
```

## 권장 사용 순서

1. TUI 실행

```bash
ros2 run ebimu_driver ebimu_tui.py
```

2. 메뉴에서 `1` 입력: scan
3. 필요하면 `3` 입력 후 `460800` 선택
4. `5` 입력: bin500
5. `6` 입력: 현재 설정 수신 검증
6. `7` 입력: launch 명령 확인
7. TUI 종료
8. ROS driver 실행

```bash
ros2 launch ebimu_driver ebimu_driver.launch.py \
  baudrate:=460800 output_mode:=binary output_interval_ms:=2
```

기본 bring-up은 아래처럼 실행한다.

```bash
ros2 launch ebimu_driver ebimu_driver.launch.py
```

기본값은 `/dev/imu`, `115200`, `ascii`, `10ms`다.

## Launch Arguments

launch는 YAML config를 쓰지 않는다. 필요한 값은 launch argument로 직접 넘긴다.

주요 argument:

```text
port                 default /dev/imu
baudrate             default 115200
output_mode          ascii | binary
output_interval_ms   10 for 100Hz, 2 for 500Hz
orientation_source   quaternion | euler
enable_gyro          true
enable_accel         true
enable_magnetometer  false
enable_temperature   false
enable_timestamp     true
frame_id             imu_link
publish_data_raw     true
```

중요한 점:

- launch는 EBIMU 레지스터를 쓰지 않는다.
- launch argument는 “현재 EBIMU가 내보내는 stream을 어떻게 해석할지”를 driver에 알려주는 값이다.
- EBIMU 자체 설정은 TUI에서 한다.

## TUI 기능

TUI는 ROS와 독립적으로 `/dev/imu`를 직접 연다.

기능:

- baudrate 자동 scan
- `<ver>` probe
- 유효 ASCII/binary frame 감지
- 대략적인 수신률 표시
- 숫자 메뉴 입력
- baudrate 선택 메뉴
- ASCII 100Hz 설정
- binary 500Hz 설정
- gyro calibration
- magnetometer calibration start/finish
- raw command 전송

ROS driver가 이미 `/dev/imu`를 열고 있으면 TUI는 port open에 실패한다. 이 경우 driver를 종료한 뒤 TUI로 설정한다.

주요 TUI 메뉴:

```text
1  포트/보드레이트 자동 찾기
2  port 변경
3  baudrate 변경
4  ascii100
5  bin500
6  현재 설정 수신 검증
7  launch 명령 보기
8  gyro calibration
9  magnetometer calibration start
10 magnetometer calibration finish
11 reset
12 raw command
0  quit
```

## EBIMU 명령 메모

Baudrate:

```text
<sb5>  115200
<sb7>  460800
<sb8>  921600
```

Output rate:

```text
<sor10> 100Hz
<sor2>  500Hz
<sor1>  1000Hz
```

Output mode:

```text
<soc1> ascii
<soc2> binary
```

권장 runtime:

```text
460800 baud + binary + 500Hz
```

1000Hz와 921600 baud는 기본 workflow에서 제외하고 나중 실험으로 둔다.

## 확인 명령

```bash
ros2 topic hz /imu/data
ros2 topic echo /ebimu/status
ros2 topic echo /diagnostics
```

## Simulator

```bash
ros2 run ebimu_driver ebimu_serial_sim.py --mode ascii --rate 100
ros2 run ebimu_driver ebimu_serial_sim.py --mode binary --rate 500
```

simulator가 출력하는 pseudo terminal을 launch에 넘긴다.

```bash
ros2 launch ebimu_driver ebimu_driver.launch.py port:=/dev/pts/X
```
