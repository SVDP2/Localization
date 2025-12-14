from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_detector')

    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Static TF: camera_link (optical) -> imu_link
    # - translation: 2 cm along -Y of the camera optical frame
    # - rotation: x_imu=+z_cam, y_imu=-x_cam, z_imu=-y_cam (right-handed)
    camera_to_imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_imu_link_static_tf',
        output='screen',
        arguments=[
            '0.0', '-0.02', '0.0',
            '0.5', '-0.5', '0.5', '0.5',
            'camera_link', 'imu_link',
        ],
    )

    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        camera_to_imu_static_tf,
        aruco_detector_node,
    ])
