# KISS-ICP Multi-LiDAR Launch File
# 
# Triple LiDAR (left, right, rear) 모드로 KISS-ICP 실행
# config.yaml의 설정을 로드하여 바로 실행
#
# Usage:
#   ros2 launch kiss_icp multi_lidar.launch.py
#   ros2 launch kiss_icp multi_lidar.launch.py visualize:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "kiss_icp"


def generate_launch_description():
    # Get default config file path
    default_config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "config.yaml"
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    visualize = LaunchConfiguration("visualize", default="true")
    config_file = LaunchConfiguration("config_file", default=default_config_file)

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time (for rosbag playback)"
    )
    
    declare_visualize = DeclareLaunchArgument(
        "visualize",
        default_value="true",
        description="Launch RViz for visualization"
    )
    
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="Path to KISS-ICP config file"
    )

    # KISS-ICP node - Multi-LiDAR mode
    # config.yaml에서 multi_lidar.enabled=true 설정이 되어있으므로
    # 별도 remapping 없이 config.yaml의 topics를 사용
    kiss_icp_node = Node(
        package=PACKAGE_NAME,
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_debug_clouds": visualize,
            },
            config_file,  # config.yaml의 모든 설정 로드
        ],
    )
    
    # RViz for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), "rviz", "kiss_icp.rviz"]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(visualize),
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        # declare_visualize,
        declare_config_file,
        # Nodes
        kiss_icp_node,
        # rviz_node,
    ])
