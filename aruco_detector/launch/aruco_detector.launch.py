from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_detector')

    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        aruco_detector_node,
    ])
