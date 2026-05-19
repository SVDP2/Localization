# EBIMU Driver Design

## Goals

- Publish EBIMU samples as ROS2 topics.
- Keep `ebimu_node` read-only with respect to EBIMU device settings.
- Use the standalone TUI as the only normal path for serial probing and EBIMU register changes.

## Ownership Model

- `ebimu_tui.py` owns `/dev/imu` while configuring the device.
- `ebimu_node` owns `/dev/imu` while publishing ROS topics.
- They must not run against the same port at the same time.

## Architecture

- `SerialPort`: Boost.Asio 8N1 byte transport for the ROS driver.
- `EbimuParser`: stream parser for ASCII and binary frames.
- `EbimuNode`: opens serial, parses stream, publishes IMU/status/diagnostics.
- `ebimu_tui.py`: standalone pyserial/Rich menu tool for probe, baudrate, output mode/rate, calibration, and raw commands.
  Probe success requires multiple valid ASCII lines or checksum-valid binary frames, not just arbitrary bytes.

## Configuration Policy

The launch path has no YAML config file. Launch arguments describe how to open and parse the current stream:

- `port`
- `baudrate`
- `output_mode`
- `output_interval_ms`
- output field flags
- frame/covariance parameters

The launch path does not send EBIMU commands. Device settings are applied before launch through the TUI.

Recommended runtime:

- `460800` baud
- binary output
- 500 Hz

1000 Hz remains experimental.

## Safety Rules

- Do not run calibration automatically on startup.
- Do not write persistent EBIMU settings from ROS launch.
- Stop the ROS driver before using the TUI on `/dev/imu`.
- Parser failures and checksum failures increment counters and do not terminate the node.
