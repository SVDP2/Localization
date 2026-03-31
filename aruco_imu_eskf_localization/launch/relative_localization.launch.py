import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_vehicle_static_tf = LaunchConfiguration('enable_vehicle_static_tf')
    enable_pose_viz = LaunchConfiguration('enable_pose_viz')
    publish_aruco_tf = LaunchConfiguration('publish_aruco_tf')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    localization_params = os.path.join(
        get_package_share_directory('aruco_imu_eskf_localization'),
        'config',
        'params.yaml',
    )
    aruco_params = localization_params

    return LaunchDescription(
        [
            DeclareLaunchArgument('enable_camera', default_value='true'),
            DeclareLaunchArgument('enable_imu', default_value='true'),
            DeclareLaunchArgument('enable_vehicle_static_tf', default_value='true'),
            DeclareLaunchArgument('enable_pose_viz', default_value='true'),
            DeclareLaunchArgument('publish_aruco_tf', default_value='false'),
            DeclareLaunchArgument('video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
            DeclareLaunchArgument('image_width', default_value='640'),
            DeclareLaunchArgument('image_height', default_value='480'),
            DeclareLaunchArgument('framerate', default_value='60.0'),
            DeclareLaunchArgument('image_topic', default_value='/image_raw'),
            DeclareLaunchArgument('imu_topic', default_value='/imu'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('aruco_imu_eskf_localization'),
                        'launch',
                        'vehicle_static_tf.launch.py',
                    )
                ),
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
                    'video_device': video_device,
                    'pixel_format': pixel_format,
                    'image_width': image_width,
                    'image_height': image_height,
                    'framerate': framerate,
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
                condition=IfCondition(enable_imu),
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='aruco_detector_node',
                name='aruco_detector_node',
                output='screen',
                parameters=[
                    aruco_params,
                    {
                        'image_topic': image_topic,
                        'publish_tf': publish_aruco_tf,
                    },
                ],
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='relative_localization_node',
                name='aruco_imu_eskf_localization_node',
                output='screen',
                parameters=[
                    localization_params,
                    {
                        'imu_topic': imu_topic,
                    },
                ],
            ),
            Node(
                package='aruco_imu_eskf_localization',
                executable='pose_rviz_marker_node',
                name='pose_rviz_marker_node',
                output='screen',
                parameters=[localization_params],
                condition=IfCondition(enable_pose_viz),
            ),
        ]
    )
