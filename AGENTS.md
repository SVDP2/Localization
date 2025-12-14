# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 `colcon` workspace `src/` containing two packages:

- `aruco_detector/` (Python, `ament_python`)
  - Node code: `aruco_detector/aruco_detector/` (entry point: `aruco_detector_node`)
  - Launch/config: `aruco_detector/launch/`, `aruco_detector/config/`
  - Tests/lint: `aruco_detector/test/` (pytest + ament linters)
- `myahrs_ros2_driver/` (C++, `ament_cmake`)
  - Headers: `myahrs_ros2_driver/include/myahrs_ros2_driver/`
  - Sources: `myahrs_ros2_driver/src/`
  - Launch/config/RViz: `myahrs_ros2_driver/launch/`, `myahrs_ros2_driver/config/`, `myahrs_ros2_driver/rviz/`

## Build, Test, and Development Commands

Run `colcon` from the workspace root (the parent directory of this repo):

- Build all packages: `cd .. && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install`
- Build one package: `cd .. && colcon build --packages-select aruco_detector`
- Run tests: `cd .. && colcon test --event-handlers console_direct+ && colcon test-result --verbose`
- Launch:
  - ArUco: `ros2 launch aruco_detector aruco_detector.launch.py`
  - IMU driver: `ros2 launch myahrs_ros2_driver myahrs_ros2_driver.launch.py use_rviz:=False`

## Coding Style & Naming Conventions

- Python: 4-space indentation, `rclpy` nodes as `*Node` classes, keep parameters declared in code and wired in `aruco_detector/config/params.yaml`.
- C++: C++14 (see `myahrs_ros2_driver/CMakeLists.txt`), keep headers in `include/myahrs_ros2_driver/` and sources in `src/`.
- ROS naming: packages are lowercase with underscores; launch files use `*.launch.py`; runtime configs live under `config/*.yaml`.

## Testing Guidelines

- `aruco_detector` uses `pytest` and ament linters. Add new tests as `aruco_detector/test/test_*.py`.
- `myahrs_ros2_driver` currently relies on `ament_lint_auto` when `BUILD_TESTING` is enabled; add gtests under `myahrs_ros2_driver/test/` if introducing non-trivial logic.

## Commit & Pull Request Guidelines

- Git history does not yet establish a convention (initial commit only). Use a clear, consistent format such as Conventional Commits: `type(scope): summary`.
- PRs should include: what changed, how to build/test (`colcon build/test` output), and any hardware/runtime assumptions (camera topic, TF frames, IMU serial port/baud).

## Configuration & Runtime Tips

- ArUco runtime parameters live in `aruco_detector/config/params.yaml`.
- IMU parameters live in `myahrs_ros2_driver/config/config.yaml`; serial device/baud are currently set in `myahrs_ros2_driver/launch/myahrs_ros2_driver.launch.py`.
