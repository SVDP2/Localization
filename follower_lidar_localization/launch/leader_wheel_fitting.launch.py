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
    leader_base_odom_topic = LaunchConfiguration('leader_base_odom_topic')
    leader_rear_odom_topic = LaunchConfiguration('leader_rear_odom_topic')
    marker_topic = LaunchConfiguration('marker_topic')
    diagnostics_topic = LaunchConfiguration('diagnostics_topic')
    wheelbase_m = LaunchConfiguration('wheelbase_m')
    track_width_m = LaunchConfiguration('track_width_m')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    wheel_width_m = LaunchConfiguration('wheel_width_m')
    roi_x_min_m = LaunchConfiguration('roi_x_min_m')
    roi_x_max_m = LaunchConfiguration('roi_x_max_m')
    roi_abs_y_max_m = LaunchConfiguration('roi_abs_y_max_m')
    pose_x_min_m = LaunchConfiguration('pose_x_min_m')
    pose_x_max_m = LaunchConfiguration('pose_x_max_m')
    pose_abs_y_max_m = LaunchConfiguration('pose_abs_y_max_m')
    max_abs_yaw_deg = LaunchConfiguration('max_abs_yaw_deg')
    cluster_gap_m = LaunchConfiguration('cluster_gap_m')
    min_cluster_points = LaunchConfiguration('min_cluster_points')
    candidate_min_length_m = LaunchConfiguration('candidate_min_length_m')
    candidate_max_length_m = LaunchConfiguration('candidate_max_length_m')
    candidate_max_rms_m = LaunchConfiguration('candidate_max_rms_m')
    min_visible_segments = LaunchConfiguration('min_visible_segments')
    assignment_max_center_distance_m = LaunchConfiguration(
        'assignment_max_center_distance_m'
    )
    assignment_max_angle_error_deg = LaunchConfiguration(
        'assignment_max_angle_error_deg'
    )
    leader_rear_x_m = LaunchConfiguration('leader_rear_x_m')
    leader_rear_y_m = LaunchConfiguration('leader_rear_y_m')
    leader_rear_z_m = LaunchConfiguration('leader_rear_z_m')
    leader_rear_yaw_deg = LaunchConfiguration('leader_rear_yaw_deg')

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
        DeclareLaunchArgument(
            'leader_base_odom_topic',
            default_value='/follower/localization/lidar_wheels/leader_base_detection',
        ),
        DeclareLaunchArgument(
            'leader_rear_odom_topic',
            default_value='/follower/localization/lidar_wheels/odom',
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/follower/localization/lidar_wheels/markers',
        ),
        DeclareLaunchArgument(
            'diagnostics_topic',
            default_value='/follower/localization/lidar_wheels/diagnostics',
        ),
        DeclareLaunchArgument('wheelbase_m', default_value='0.720'),
        DeclareLaunchArgument('track_width_m', default_value='0.700'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.1325'),
        DeclareLaunchArgument('wheel_width_m', default_value='0.110'),
        DeclareLaunchArgument('roi_x_min_m', default_value='0.15'),
        DeclareLaunchArgument('roi_x_max_m', default_value='2.50'),
        DeclareLaunchArgument('roi_abs_y_max_m', default_value='1.20'),
        DeclareLaunchArgument('pose_x_min_m', default_value='0.0'),
        DeclareLaunchArgument('pose_x_max_m', default_value='3.00'),
        DeclareLaunchArgument('pose_abs_y_max_m', default_value='1.50'),
        DeclareLaunchArgument('max_abs_yaw_deg', default_value='70.0'),
        DeclareLaunchArgument('cluster_gap_m', default_value='0.060'),
        DeclareLaunchArgument('min_cluster_points', default_value='4'),
        DeclareLaunchArgument('candidate_min_length_m', default_value='0.050'),
        DeclareLaunchArgument('candidate_max_length_m', default_value='0.180'),
        DeclareLaunchArgument('candidate_max_rms_m', default_value='0.025'),
        DeclareLaunchArgument('min_visible_segments', default_value='2'),
        DeclareLaunchArgument(
            'assignment_max_center_distance_m',
            default_value='0.180',
        ),
        DeclareLaunchArgument('assignment_max_angle_error_deg', default_value='37.0'),
        DeclareLaunchArgument('leader_rear_x_m', default_value='-0.275'),
        DeclareLaunchArgument('leader_rear_y_m', default_value='0.0'),
        DeclareLaunchArgument('leader_rear_z_m', default_value='0.0525'),
        DeclareLaunchArgument('leader_rear_yaw_deg', default_value='0.0'),
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
                    'leader_base_odom_topic': leader_base_odom_topic,
                    'leader_rear_odom_topic': leader_rear_odom_topic,
                    'marker_topic': marker_topic,
                    'diagnostics_topic': diagnostics_topic,
                    'wheelbase_m': ParameterValue(wheelbase_m, value_type=float),
                    'track_width_m': ParameterValue(track_width_m, value_type=float),
                    'wheel_radius_m': ParameterValue(wheel_radius_m, value_type=float),
                    'wheel_width_m': ParameterValue(wheel_width_m, value_type=float),
                    'roi_x_min_m': ParameterValue(roi_x_min_m, value_type=float),
                    'roi_x_max_m': ParameterValue(roi_x_max_m, value_type=float),
                    'roi_abs_y_max_m': ParameterValue(
                        roi_abs_y_max_m,
                        value_type=float,
                    ),
                    'pose_x_min_m': ParameterValue(pose_x_min_m, value_type=float),
                    'pose_x_max_m': ParameterValue(pose_x_max_m, value_type=float),
                    'pose_abs_y_max_m': ParameterValue(
                        pose_abs_y_max_m,
                        value_type=float,
                    ),
                    'max_abs_yaw_deg': ParameterValue(
                        max_abs_yaw_deg,
                        value_type=float,
                    ),
                    'cluster_gap_m': ParameterValue(cluster_gap_m, value_type=float),
                    'min_cluster_points': ParameterValue(
                        min_cluster_points,
                        value_type=int,
                    ),
                    'candidate_min_length_m': ParameterValue(
                        candidate_min_length_m,
                        value_type=float,
                    ),
                    'candidate_max_length_m': ParameterValue(
                        candidate_max_length_m,
                        value_type=float,
                    ),
                    'candidate_max_rms_m': ParameterValue(
                        candidate_max_rms_m,
                        value_type=float,
                    ),
                    'min_visible_segments': ParameterValue(
                        min_visible_segments,
                        value_type=int,
                    ),
                    'assignment_max_center_distance_m': ParameterValue(
                        assignment_max_center_distance_m,
                        value_type=float,
                    ),
                    'assignment_max_angle_error_deg': ParameterValue(
                        assignment_max_angle_error_deg,
                        value_type=float,
                    ),
                    'leader_rear_x_m': ParameterValue(
                        leader_rear_x_m,
                        value_type=float,
                    ),
                    'leader_rear_y_m': ParameterValue(
                        leader_rear_y_m,
                        value_type=float,
                    ),
                    'leader_rear_z_m': ParameterValue(
                        leader_rear_z_m,
                        value_type=float,
                    ),
                    'leader_rear_yaw_deg': ParameterValue(
                        leader_rear_yaw_deg,
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
