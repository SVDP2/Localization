import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import yaml


def _prefixed_frame(frame_prefix, frame):
    return PythonExpression(["'", frame_prefix, "' + '", frame, "'"])


def _node_params(params_file, node_name):
    with open(params_file, 'r') as f:
        data = yaml.safe_load(f) or {}
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')
    leader_frame_prefix = LaunchConfiguration('leader_frame_prefix')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_vehicle_static_tf = LaunchConfiguration('enable_vehicle_static_tf')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    publish_aruco_tf = LaunchConfiguration('publish_aruco_tf')
    enable_pose_viz = LaunchConfiguration('enable_pose_viz')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')

    params = os.path.join(
        get_package_share_directory('aruco_imu_eskf_localization_cpp'),
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
        DeclareLaunchArgument('enable_vehicle_static_tf', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),
        DeclareLaunchArgument('publish_aruco_tf', default_value='false'),
        DeclareLaunchArgument('enable_pose_viz', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='60.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('aruco_imu_eskf_localization_cpp'),
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
            package='aruco_imu_eskf_localization_cpp',
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
            package='aruco_imu_eskf_localization_cpp',
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
                    'leader_rear_odom_topic': 'localization/leader_rear/odom',
                    'leader_rear_pose_topic': 'localization/leader_rear/pose',
                    'diagnostics_topic': 'diagnostics',
                    'board_frame': _prefixed_frame(leader_frame_prefix, 'board'),
                    'leader_rear_frame': _prefixed_frame(leader_frame_prefix, 'leader_rear'),
                    'base_frame': _prefixed_frame(frame_prefix, 'base_link'),
                    'camera_frame': _prefixed_frame(frame_prefix, 'usb_cam'),
                },
            ],
        ),
        Node(
            package='aruco_imu_eskf_localization_cpp',
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
