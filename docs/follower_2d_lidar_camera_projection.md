# Follower 2D LiDAR Camera Projection

This records the first ROS 2 Humble-native follower 2D LiDAR bringup and
camera projection path.

## Driver

The active follower driver package is `follower_lidar_driver`.

- Device: `/dev/ttyLIDAR` (`/dev/ttyUSB1`, CP2102 USB-UART on the tested car)
- LiDAR model observed at runtime: YDLidar G2B
- Topic: `/follower/scan`
- Frame: `follower/lidar`
- Expected rate: about 10 Hz
- SDK: target robot's installed YDLidar-SDK 1.2.10

Run:

```bash
cd /home/xytron/follower_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch follower_lidar_driver follower_lidar.launch.py
```

Check:

```bash
ros2 topic echo /follower/scan --once
ros2 topic hz /follower/scan
```

## Projection

The projection package is `follower_camera_lidar_projection`.

- Image input: `/follower/image_raw`
- Scan input: `/follower/scan`
- Image output: `/follower/camera_lidar/projected_image`
- Camera intrinsic snapshot: `follower_cam_intrinsic_20260407.yaml`
- Provisional extrinsic: `provisional_lidar_to_camera_extrinsic.yaml`

The provisional extrinsic is copied from the older calibration workspace and
must be visually revalidated before it is used for localization correction or
for a permanent static TF in the vehicle launch.

Combined bringup:

```bash
cd /home/xytron/follower_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch follower_lidar_driver follower_lidar_projection_bringup.launch.py
```

View:

```bash
rqt_image_view /follower/camera_lidar/projected_image
```

## Verified On 2026-05-20

- `follower_lidar_driver` and `follower_camera_lidar_projection` built in
  `/home/xytron/follower_ws`.
- `/follower/scan` published with `frame_id: follower/lidar` and about 9.6 Hz.
- `/follower/camera_lidar/projected_image` published at about 39-40 Hz while
  the existing follower camera stack was running.

