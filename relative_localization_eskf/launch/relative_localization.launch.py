import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _prefixed_frame(frame_prefix, frame):
    return PythonExpression(["'", frame_prefix, "' + '", frame, "'"])


def _node_params(params_file, node_name):
    with open(params_file, 'r') as f:
        data = yaml.safe_load(f) or {}
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def _xycar_imu_actions(context):
    try:
        package_share = get_package_share_directory('xycar_imu')
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    'xycar_imu package not found; not starting IMU driver. '
                    'Source the xycar IMU underlay or launch the existing IMU '
                    'driver separately so /follower/imu is available.'
                )
            )
        ]

    config_path = os.path.join(package_share, 'config', 'xycar_imu.yaml')
    if not os.path.exists(config_path):
        config_path = os.path.join(package_share, 'config', 'imu.yaml')

    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    return [
        Node(
            package='xycar_imu',
            executable='imu_node',
            name='imu_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                config_path,
                {
                    'frame_header': f'{frame_prefix}base_imu_link',
                },
            ],
        )
    ]


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')
    leader_frame_prefix = LaunchConfiguration('leader_frame_prefix')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_lidar = LaunchConfiguration('enable_lidar')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    lidar_scan_topic = LaunchConfiguration('lidar_scan_topic')
    publish_aruco_tf = LaunchConfiguration('publish_aruco_tf')
    publish_relative_tf = LaunchConfiguration('publish_relative_tf')
    enable_pose_viz = LaunchConfiguration('enable_pose_viz')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')

    params = os.path.join(
        get_package_share_directory('relative_localization_eskf'),
        'config',
        'params.yaml',
    )
    aruco_detector_params = _node_params(params, 'aruco_detector_node')
    relative_localization_params = _node_params(params, 'relative_localization_node')
    pose_rviz_marker_params = _node_params(params, 'pose_rviz_marker_node')

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='follower'),
        DeclareLaunchArgument('frame_prefix', default_value='follower/'),
        DeclareLaunchArgument('leader_frame_prefix', default_value='leader/'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),
        DeclareLaunchArgument('lidar_scan_topic', default_value='/follower/scan'),
        DeclareLaunchArgument('publish_aruco_tf', default_value='false'),
        DeclareLaunchArgument('publish_relative_tf', default_value='true'),
        DeclareLaunchArgument('enable_pose_viz', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='60.0'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'video_device': video_device,
                'framerate': ParameterValue(framerate, value_type=float),
                'io_method': 'mmap',
                'frame_id': _prefixed_frame(frame_prefix, 'usb_cam'),
                'pixel_format': pixel_format,
                'av_device_format': 'YUV422P',
                'image_width': ParameterValue(image_width, value_type=int),
                'image_height': ParameterValue(image_height, value_type=int),
                'camera_name': 'follower_usb_cam',
                'camera_info_url': 'package://usb_cam/config/camera_info.yaml',
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'white_balance': 4000,
                'autoexposure': True,
                'exposure': 100,
                'autofocus': False,
                'focus': -1,
            }],
            condition=IfCondition(enable_camera),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('follower_lidar_driver'),
                    'launch',
                    'follower_lidar.launch.py',
                )
            ),
            condition=IfCondition(enable_lidar),
        ),
        OpaqueFunction(
            function=_xycar_imu_actions,
            condition=IfCondition(enable_imu),
        ),
        Node(
            package='relative_localization_eskf',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                aruco_detector_params,
                {
                    'image_topic': image_topic,
                    'board_pose_topic': 'localization/aruco/board_pose',
                    'pose_prior_topic': 'localization/relative/pose',
                    'debug_image_topic': 'localization/aruco/debug_image',
                    'diagnostics_topic': 'diagnostics',
                    'base_frame': _prefixed_frame(frame_prefix, 'base_link'),
                    'camera_frame': _prefixed_frame(frame_prefix, 'usb_cam'),
                    'board_frame': _prefixed_frame(leader_frame_prefix, 'board'),
                    'leader_rear_frame': _prefixed_frame(leader_frame_prefix, 'leader_rear'),
                    'raw_aruco_child_frame': _prefixed_frame(frame_prefix, 'base_link_aruco_raw'),
                    'publish_tf': publish_aruco_tf,
                },
            ],
        ),
        Node(
            package='relative_localization_eskf',
            executable='relative_localization_node',
            name='relative_localization_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                relative_localization_params,
                {
                    'imu_topic': imu_topic,
                    'board_pose_topic': 'localization/aruco/board_pose',
                    'odom_topic': 'localization/relative/odom',
                    'pose_topic': 'localization/relative/pose',
                    'leader_base_odom_topic': 'localization/leader_base/odom',
                    'leader_base_pose_topic': 'localization/leader_base/pose',
                    'leader_rear_odom_topic': 'localization/leader_rear/odom',
                    'leader_rear_pose_topic': 'localization/leader_rear/pose',
                    'diagnostics_topic': 'diagnostics',
                    'lidar_scan_topic': lidar_scan_topic,
                    'board_frame': _prefixed_frame(leader_frame_prefix, 'board'),
                    'leader_base_frame': _prefixed_frame(leader_frame_prefix, 'base_link'),
                    'leader_rear_frame': _prefixed_frame(leader_frame_prefix, 'leader_rear'),
                    'base_frame': _prefixed_frame(frame_prefix, 'base_link'),
                    'camera_frame': _prefixed_frame(frame_prefix, 'usb_cam'),
                    'publish_tf': publish_relative_tf,
                },
            ],
        ),
        Node(
            package='relative_localization_eskf',
            executable='pose_rviz_marker_node',
            name='pose_rviz_marker_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                pose_rviz_marker_params,
                {
                    'odom_topic': 'localization/leader_rear/odom',
                    'marker_topic': 'localization/relative/pose_markers',
                },
            ],
            condition=IfCondition(enable_pose_viz),
        ),
    ])
