import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _prefixed_frame(frame_prefix, frame):
    return PythonExpression(["'", frame_prefix, "' + '", frame, "'"])


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')
    leader_frame_prefix = LaunchConfiguration('leader_frame_prefix')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_vehicle_static_tf = LaunchConfiguration('enable_vehicle_static_tf')
    enable_pose_viz = LaunchConfiguration('enable_pose_viz')
    enable_gps_odom = LaunchConfiguration('enable_gps_odom')
    enable_relative_gps = LaunchConfiguration('enable_relative_gps')
    enable_gps_fusion = LaunchConfiguration('enable_gps_fusion')
    publish_aruco_tf = LaunchConfiguration('publish_aruco_tf')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    map_frame = LaunchConfiguration('map_frame')
    map_origin_lat_deg = LaunchConfiguration('map_origin_lat_deg')
    map_origin_lon_deg = LaunchConfiguration('map_origin_lon_deg')
    map_origin_alt_m = LaunchConfiguration('map_origin_alt_m')
    leader_gps_odom_topic = LaunchConfiguration('leader_gps_odom_topic')

    localization_params = os.path.join(
        get_package_share_directory('aruco_imu_eskf_localization'),
        'config',
        'params.yaml',
    )
    aruco_params = localization_params

    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_namespace', default_value='follower'),
            DeclareLaunchArgument('frame_prefix', default_value='follower/'),
            DeclareLaunchArgument('leader_frame_prefix', default_value='leader/'),
            DeclareLaunchArgument('enable_camera', default_value='true'),
            DeclareLaunchArgument('enable_imu', default_value='true'),
            DeclareLaunchArgument('enable_vehicle_static_tf', default_value='true'),
            DeclareLaunchArgument('enable_pose_viz', default_value='true'),
            DeclareLaunchArgument('enable_gps_odom', default_value='true'),
            DeclareLaunchArgument('enable_relative_gps', default_value='true'),
            DeclareLaunchArgument('enable_gps_fusion', default_value='false'),
            DeclareLaunchArgument('publish_aruco_tf', default_value='false'),
            DeclareLaunchArgument('video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
            DeclareLaunchArgument('image_width', default_value='640'),
            DeclareLaunchArgument('image_height', default_value='480'),
            DeclareLaunchArgument('framerate', default_value='60.0'),
            DeclareLaunchArgument('image_topic', default_value='image_raw'),
            DeclareLaunchArgument('imu_topic', default_value='imu'),
            DeclareLaunchArgument('map_frame', default_value='map'),
            DeclareLaunchArgument('map_origin_lat_deg', default_value='0.0'),
            DeclareLaunchArgument('map_origin_lon_deg', default_value='0.0'),
            DeclareLaunchArgument('map_origin_alt_m', default_value='0.0'),
            DeclareLaunchArgument(
                'leader_gps_odom_topic',
                default_value='/leader/localization/gps/odom',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('aruco_imu_eskf_localization'),
                        'launch',
                        'vehicle_static_tf.launch.py',
                    )
                ),
                launch_arguments={'frame_prefix': frame_prefix}.items(),
                condition=IfCondition(enable_vehicle_static_tf),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('xycar_cam'),
                        'launch',
                        'xycar_cam.launch.py',
                    )
                ),
                launch_arguments={
                    'namespace': robot_namespace,
                    'video_device': video_device,
                    'pixel_format': pixel_format,
                    'image_width': image_width,
                    'image_height': image_height,
                    'framerate': framerate,
                    'camera_frame_id': _prefixed_frame(frame_prefix, 'usb_cam'),
                }.items(),
                condition=IfCondition(enable_camera),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('xycar_imu'),
                        'launch',
                        'xycar_imu.launch.py',
                    )
                ),
                launch_arguments={
                    'namespace': robot_namespace,
                    'frame_header': _prefixed_frame(frame_prefix, 'base_imu_link'),
                }.items(),
                condition=IfCondition(enable_imu),
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='gps_odom_node',
                name='gps_odom_node',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    {
                        'fix_topic': 'ublox_gps_node/fix',
                        'fix_velocity_topic': 'ublox_gps_node/fix_velocity',
                        'odom_topic': 'localization/gps/odom',
                        'map_frame': map_frame,
                        'child_frame': _prefixed_frame(frame_prefix, 'follower_gps'),
                        'map_origin_lat_deg': map_origin_lat_deg,
                        'map_origin_lon_deg': map_origin_lon_deg,
                        'map_origin_alt_m': map_origin_alt_m,
                    },
                ],
                condition=IfCondition(enable_gps_odom),
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='relative_gps_node',
                name='relative_gps_node',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    {
                        'follower_gps_odom_topic': 'localization/gps/odom',
                        'leader_gps_odom_topic': leader_gps_odom_topic,
                        'fused_odom_topic': 'localization/leader_rear/odom',
                        'relative_gps_odom_topic': 'localization/gps_relative/odom',
                        'map_frame': map_frame,
                        'leader_base_frame': _prefixed_frame(
                            leader_frame_prefix,
                            'base_link',
                        ),
                        'leader_rear_frame': _prefixed_frame(
                            leader_frame_prefix,
                            'leader_rear',
                        ),
                        'follower_base_frame': _prefixed_frame(frame_prefix, 'base_link'),
                        'follower_gps_frame': _prefixed_frame(frame_prefix, 'follower_gps'),
                    },
                ],
                condition=IfCondition(enable_relative_gps),
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='aruco_detector_node',
                name='aruco_detector_node',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    aruco_params,
                    {
                        'image_topic': image_topic,
                        'board_pose_topic': 'localization/aruco/board_pose',
                        'pose_prior_topic': 'localization/relative/pose',
                        'debug_image_topic': 'localization/aruco/debug_image',
                        'base_frame': _prefixed_frame(frame_prefix, 'base_link'),
                        'camera_frame_id': _prefixed_frame(frame_prefix, 'usb_cam'),
                        'board_frame': _prefixed_frame(leader_frame_prefix, 'board'),
                        'publish_tf': publish_aruco_tf,
                    },
                ],
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='relative_localization_node',
                name='aruco_imu_eskf_localization_node',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    localization_params,
                    {
                        'imu_topic': imu_topic,
                        'board_pose_topic': 'localization/aruco/board_pose',
                        'odom_topic': 'localization/relative/odom',
                        'pose_topic': 'localization/relative/pose',
                        'leader_rear_odom_topic': 'localization/leader_rear/odom',
                        'leader_rear_pose_topic': 'localization/leader_rear/pose',
                        'gps_relative_odom_topic': 'localization/gps_relative/odom',
                        'enable_gps_fusion': enable_gps_fusion,
                        'board_frame': _prefixed_frame(leader_frame_prefix, 'board'),
                        'leader_rear_frame': _prefixed_frame(
                            leader_frame_prefix,
                            'leader_rear',
                        ),
                        'base_frame': _prefixed_frame(frame_prefix, 'base_link'),
                        'camera_frame': _prefixed_frame(frame_prefix, 'usb_cam'),
                    },
                ],
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='pose_rviz_marker_node',
                name='pose_rviz_marker_node',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    localization_params,
                    {
                        'odom_topic': 'localization/leader_rear/odom',
                        'marker_topic': 'localization/relative/pose_markers',
                    },
                ],
                condition=IfCondition(enable_pose_viz),
            ),
        ]
    )
