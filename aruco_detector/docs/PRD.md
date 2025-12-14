# ArUco Detector - Product Requirements Document

## 1. Overview
ROS2-based ArUco marker detection and pose estimation system. Detects multiple ArUco markers from a single camera, calculates 3D poses, and publishes as TF frames.

## 2. System Architecture

### 2.1 Input
- **Topic**: `/usb_cam_3/image_raw` (sensor_msgs/Image)
- **Camera Calibration**: `config/cam_intrinsic.yaml` (pre-calibrated)

### 2.2 Output
- **TF Frames**: Each detected marker's pose
  - Parent: `camera_link`
  - Child: `aruco_marker_<id>`
- **RViz2 Visualization**: 3D marker poses with optional ID labels

### 2.3 Processing Flow
```
Image Topic -> ArUco Detection -> solvePnP -> TF Broadcast -> RViz2
               (OpenCV)           (Pose Est.)  (tf2_ros)
```

## 3. Marker Configuration

### 3.1 Marker Layout (13 markers total)
- **Large squares (4)**: Corner positions (ID: 0-3)
- **Medium squares (5)**: Mid-edges + center (ID: 4-8)
- **Small squares (4)**: Between medium squares (ID: 9-12)

### 3.2 Marker Config File (config/markers_board.yaml)
```yaml
markers:
  - id: 0
    size: 150.0  # mm
    type: "corner"
  - id: 1
    size: 150.0
    type: "corner"
  - id: 2
    size: 150.0
    type: "corner"
  - id: 3
    size: 150.0
    type: "corner"
  - id: 4
    size: 100.0  # mm
    type: "middle"
  - id: 5
    size: 100.0
    type: "middle"
  - id: 6
    size: 100.0
    type: "middle"
  - id: 7
    size: 100.0
    type: "middle"
  - id: 8
    size: 100.0
    type: "middle"
  - id: 9
    size: 50.0  # mm
    type: "small"
  - id: 10
    size: 50.0
    type: "small"
  - id: 11
    size: 50.0
    type: "small"
  - id: 12
    size: 50.0
    type: "small"
```

### 3.3 Independent Marker Processing
- Each marker detected and processed separately
- Pros:
  - Works with partial occlusion
  - Per-marker confidence assessment
  - Simple implementation
- Cons:
  - Individual marker errors (lower accuracy than board-based)

## 4. Coordinate Frames and TF

### 4.1 TF Tree
```
map (optional)
 └─ camera_link (base)
     ├─ aruco_marker_0
     ├─ aruco_marker_1
     ├─ aruco_marker_2
     ├─ ...
     └─ aruco_marker_12
```

### 4.2 Pose Estimation
- **Method**: OpenCV `cv2.solvePnP()`
- **Inputs**:
  - 2D image points (marker corners)
  - 3D object points (from marker size)
  - Camera matrix K (from cam_intrinsic.yaml)
  - Distortion coefficients D (from cam_intrinsic.yaml)
- **Outputs**:
  - Rotation vector (rvec) -> rotation matrix
  - Translation vector (tvec)

## 5. Implementation Requirements

### 5.1 ROS2 Node
- **Node name**: `aruco_detector_node`
- **Package type**: ament_python
- **Dependencies**:
  - rclpy
  - sensor_msgs
  - tf2_ros
  - cv_bridge
  - opencv-python
  - opencv-contrib-python (ArUco)

### 5.2 Parameters (config/params.yaml)
```yaml
aruco_detector:
  ros__parameters:
    # Marker settings
    marker_config_file: "config/markers_board.yaml"
    aruco_dict: "DICT_4X4_50"

    # Camera settings
    camera_calibration_file: "config/cam_intrinsic.yaml"
    camera_frame_id: "camera_link"

    # Detection parameters
    detection_threshold: 0.5
    publish_rate: 30.0  # Hz

    # TF settings
    publish_tf: true
    tf_prefix: "aruco_marker_"
```

### 5.3 Camera Calibration File
- **Path**: `config/cam_intrinsic.yaml` (already exists)
- **Format**: Pre-calibrated camera intrinsics
- **Contents**:
  - camera_matrix: K (3x3)
  - distortion_coefficients: D (1x5)
  - image_size: width, height

## 6. Core Functions

### 6.1 detect_markers()
- Use OpenCV ArUco library
- Return detected marker IDs and corner points

### 6.2 estimate_pose()
- For each detected marker:
  1. Lookup size from config
  2. Generate 3D object points
  3. Call solvePnP
  4. Get rvec, tvec

### 6.3 publish_tf()
- Create TransformStamped messages
- Broadcast each marker pose relative to camera_link
- Use tf2_ros.TransformBroadcaster

### 6.4 visualization()
- RViz2 TF display
- Config: Fixed Frame = camera_link, Add TF, Show Names

## 7. Error Handling

### 7.1 Detection Failure
- No marker visible: Skip TF publish
- Partial occlusion: Skip undetected markers

### 7.2 Pose Estimation Failure
- solvePnP fails: Skip frame, log warning
- Log: "pose estimation failed for marker ID X"

### 7.3 Calibration Errors
- Missing YAML: Error and exit
- Invalid parameters: Use defaults or exit

## 8. Performance Targets
- **Frame rate**: 30 FPS
- **Detection latency**: < 50ms per frame
- **Pose accuracy**: ±5mm at 1m distance

## 9. Test Scenarios
1. Single marker detection
2. All 13 markers visible
3. Partial occlusion
4. Various distances (0.5m ~ 3m)
5. Various angles (frontal, oblique)

## 10. Future Extensions
- ArUco Board-based processing
- Inter-marker geometry validation
- Multi-camera support
- Online calibration refinement
