import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('follower_camera_lidar_projection')

    default_params = os.path.join(package_share, 'config', 'projection.yaml')
    default_extrinsic = os.path.join(
        package_share, 'config', 'provisional_lidar_to_camera_extrinsic.yaml')
    default_intrinsic = os.path.join(
        package_share, 'config', 'follower_cam_intrinsic_20260407.yaml')

    projection_params_file = LaunchConfiguration('projection_params_file')
    camera_calibration_file = LaunchConfiguration('camera_calibration_file')
    extrinsic_file = LaunchConfiguration('extrinsic_file')
    publish_static_tf = LaunchConfiguration('publish_static_tf')

    return LaunchDescription([
        DeclareLaunchArgument('projection_params_file', default_value=default_params),
        DeclareLaunchArgument('camera_calibration_file', default_value=default_intrinsic),
        DeclareLaunchArgument('extrinsic_file', default_value=default_extrinsic),
        DeclareLaunchArgument('publish_static_tf', default_value='false'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='follower_usb_cam_to_lidar_static_tf',
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
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(publish_static_tf),
        ),
        Node(
            package='follower_camera_lidar_projection',
            executable='camera_lidar_projection_node',
            name='camera_lidar_projection_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                projection_params_file,
                {
                    'camera_calibration_file': camera_calibration_file,
                    'extrinsic_file': extrinsic_file,
                },
            ],
        ),
    ])
