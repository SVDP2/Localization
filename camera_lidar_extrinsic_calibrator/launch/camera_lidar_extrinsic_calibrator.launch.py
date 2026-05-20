import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('camera_lidar_extrinsic_calibrator')
    projection_share = get_package_share_directory('follower_camera_lidar_projection')

    default_params = os.path.join(package_share, 'config', 'calibrator.yaml')
    default_intrinsic = os.path.join(
        projection_share, 'config', 'follower_cam_intrinsic_20260407.yaml')
    default_initial_extrinsic = os.path.join(
        projection_share, 'config', 'provisional_lidar_to_camera_extrinsic.yaml')

    params_file = LaunchConfiguration('params_file')
    camera_calibration_file = LaunchConfiguration('camera_calibration_file')
    initial_extrinsic_file = LaunchConfiguration('initial_extrinsic_file')
    output_root = LaunchConfiguration('output_root')
    allow_stale_pair = LaunchConfiguration('allow_stale_pair')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('camera_calibration_file', default_value=default_intrinsic),
        DeclareLaunchArgument('initial_extrinsic_file', default_value=default_initial_extrinsic),
        DeclareLaunchArgument(
            'output_root',
            default_value='/home/xytron/follower_ws/calibration/camera_lidar',
        ),
        DeclareLaunchArgument('allow_stale_pair', default_value='true'),
        Node(
            package='camera_lidar_extrinsic_calibrator',
            executable='camera_lidar_extrinsic_calibrator_node',
            name='camera_lidar_extrinsic_calibrator_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'camera_calibration_file': camera_calibration_file,
                    'initial_extrinsic_file': initial_extrinsic_file,
                    'output_root': output_root,
                    'allow_stale_pair': ParameterValue(
                        allow_stale_pair,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
