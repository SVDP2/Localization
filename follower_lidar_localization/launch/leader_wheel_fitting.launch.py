import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    include_lidar_driver = LaunchConfiguration('include_lidar_driver')
    publish_lidar_static_tf = LaunchConfiguration('publish_lidar_static_tf')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    use_aruco_prior = LaunchConfiguration('use_aruco_prior')
    aruco_prior_topic = LaunchConfiguration('aruco_prior_topic')

    lidar_launch = os.path.join(
        get_package_share_directory('follower_lidar_driver'),
        'launch',
        'follower_lidar.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('include_lidar_driver', default_value='false'),
        DeclareLaunchArgument('publish_lidar_static_tf', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/follower/scan'),
        DeclareLaunchArgument('base_frame', default_value='follower/base_link'),
        DeclareLaunchArgument('use_aruco_prior', default_value='true'),
        DeclareLaunchArgument(
            'aruco_prior_topic',
            default_value='/follower/localization/leader_rear/odom',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(include_lidar_driver),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='follower_base_link_to_usb_cam_static_tf_for_lidar_fitting',
            output='screen',
            arguments=[
                '--x', '0.270',
                '--y', '0.0',
                '--z', '0.135',
                '--qx', '-0.5',
                '--qy', '0.5',
                '--qz', '-0.5',
                '--qw', '0.5',
                '--frame-id', 'follower/base_link',
                '--child-frame-id', 'follower/usb_cam',
            ],
            condition=IfCondition(publish_lidar_static_tf),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='follower_usb_cam_to_lidar_static_tf_for_lidar_fitting',
            output='screen',
            arguments=[
                '--x', '0.0025602837540221954',
                '--y', '0.12819786529006177',
                '--z', '0.133867715594006',
                '--qx', '-0.17415269402591296',
                '--qy', '-0.6768985720658026',
                '--qz', '0.6965430734104255',
                '--qw', '-0.16219404792640454',
                '--frame-id', 'follower/usb_cam',
                '--child-frame-id', 'follower/lidar',
            ],
            condition=IfCondition(publish_lidar_static_tf),
        ),
        Node(
            package='follower_lidar_localization',
            executable='leader_wheel_fitting_node',
            name='leader_wheel_fitting_node',
            output='screen',
            parameters=[
                {
                    'scan_topic': scan_topic,
                    'base_frame': base_frame,
                    'use_aruco_prior': ParameterValue(
                        use_aruco_prior,
                        value_type=bool,
                    ),
                    'aruco_prior_topic': aruco_prior_topic,
                },
            ],
        ),
    ])
