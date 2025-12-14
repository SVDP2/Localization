# IMU Integration for Aruco Detector EKF

## Goal Description
Integrate IMU data (specifically angular velocity from gyroscope) into the Aruco Detector's Extended Kalman Filter (EKF). The goal is to:
1.  Improve orientation smoothness.
2.  Mitigate "pose ambiguity" (180-degree flips) by using IMU prediction to reject outliers.
3.  Reduce "teleportation" artifacts by bridging gaps in visual detection with IMU propagation.

## User Review Required
> [!IMPORTANT]
> **Frame Transformations**:
> Based on user input:
> - IMU Frame: FLU (X-Forward, Y-Left, Z-Up)
> - Camera Frame: Optical (Z-Forward, X-Right, Y-Down)
> - Transform $R_{imu}^{cam}$:
>   - IMU X (Forward) -> Cam Z
>   - IMU Y (Left) -> Cam -X
>   - IMU Z (Up) -> Cam -Y
>   - We will hardcode this rotation or make it a default parameter `imu_to_camera_rotation`.

## Proposed Changes

### `aruco_detector` Package

#### [MODIFY] [aruco_detector_node.py](file:///home/user1/ROS2_Workspace/svdp_ws/src/aruco_detector/aruco_detector/aruco_detector_node.py)
-   Add subscription to `/imu/data` (topic name configurable).
-   Add `imu_callback` to process high-rate IMU messages.
-   In `imu_callback`:
    -   Predict state for all active marker filters using `dt` and `angular_velocity`.
    -   Publish TF updates immediately for smoother high-rate output (optional, but good for control).
-   Update `_apply_filter` to use the prediction from IMU if available.

#### [MODIFY] [pose_filter.py](file:///home/user1/ROS2_Workspace/svdp_ws/src/aruco_detector/aruco_detector/pose_filter.py)
-   Update `PoseFilter` class to handle IMU inputs.
-   **State**: Keep `[x, y, z, vx, vy, vz]` for position.
-   **Orientation**:
    -   Replace SLERP smoothing with a Quaternion-based prediction step using Angular Velocity.
    -   State: Quaternion `q`.
    -   Prediction: $q_{k+1} = q_k \otimes \Delta q(\omega, dt)$.
    -   Update: Correct with Aruco orientation.
-   **Outlier Rejection**:
    -   Before update, check the difference between Predicted Orientation and Measured Aruco Orientation.
    -   If difference > Threshold (e.g., 90 degrees), reject the measurement (handling ambiguity).

## Verification Plan

### Automated Tests
-   Unit test for `PoseFilter` with IMU prediction:
    -   Feed constant angular velocity -> Check if orientation rotates correctly.
    -   Feed outlier measurement -> Check if rejected (or smoothed).

### Manual Verification
-   **Setup**: Run `aruco_detector` with a camera and IMU (bag file or live).
-   **Test 1: Smoothness**:
    -   Cover the camera briefly. The TF should continue to move/stay stable based on IMU, not freeze immediately.
    -   Shake the camera. The TF should be smoother than before.
-   **Test 2: Ambiguity**:
    -   Hold a marker at an angle where it flickers.
    -   Observe if the TF flips. With IMU, it should resist flipping.
