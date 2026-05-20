import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('follower_lidar_driver')
    default_params = os.path.join(package_share, 'config', 'follower_lidar.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YDLidar driver parameter file.',
        ),
        Node(
            package='follower_lidar_driver',
            executable='follower_lidar_node',
            name='follower_lidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
