from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    output_mode = LaunchConfiguration("output_mode")
    output_interval_ms = LaunchConfiguration("output_interval_ms")
    orientation_source = LaunchConfiguration("orientation_source")
    enable_gyro = LaunchConfiguration("enable_gyro")
    enable_accel = LaunchConfiguration("enable_accel")
    enable_magnetometer = LaunchConfiguration("enable_magnetometer")
    enable_temperature = LaunchConfiguration("enable_temperature")
    enable_timestamp = LaunchConfiguration("enable_timestamp")
    frame_id = LaunchConfiguration("frame_id")
    publish_data_raw = LaunchConfiguration("publish_data_raw")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/imu"),  # 포트 예: /dev/imu, /dev/ttyUSB0
        DeclareLaunchArgument("baudrate", default_value="460800"),  # 보드레이트 예: 115200, 460800
        DeclareLaunchArgument("output_mode", default_value="binary"),  # 출력 모드: ascii, binary
        DeclareLaunchArgument("output_interval_ms", default_value="2"),  # 주기 ms: 10=100Hz, 2=500Hz
        DeclareLaunchArgument("orientation_source", default_value="quaternion"),  # 자세 형식: quaternion, euler
        DeclareLaunchArgument("enable_gyro", default_value="true"),  # gyro 필드 사용: true, false
        DeclareLaunchArgument("enable_accel", default_value="true"),  # accel 필드 사용: true, false
        DeclareLaunchArgument("enable_magnetometer", default_value="false"),  # mag 필드 사용: true, false
        DeclareLaunchArgument("enable_temperature", default_value="false"),  # 온도 필드 사용: true, false
        DeclareLaunchArgument("enable_timestamp", default_value="true"),  # timestamp 필드 사용: true, false
        DeclareLaunchArgument("frame_id", default_value="imu_link"),  # ROS frame_id 예: imu_link
        DeclareLaunchArgument("publish_data_raw", default_value="true"),  # imu/data_raw 발행: true, false
        Node(
            package="ebimu_driver",
            executable="ebimu_node",
            name="ebimu_driver",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "output_mode": output_mode,
                "output_interval_ms": ParameterValue(output_interval_ms, value_type=int),
                "orientation_source": orientation_source,
                "enable_gyro": ParameterValue(enable_gyro, value_type=bool),
                "enable_accel": ParameterValue(enable_accel, value_type=bool),
                "enable_magnetometer": ParameterValue(enable_magnetometer, value_type=bool),
                "enable_temperature": ParameterValue(enable_temperature, value_type=bool),
                "enable_timestamp": ParameterValue(enable_timestamp, value_type=bool),
                "frame_id": frame_id,
                "publish_data_raw": ParameterValue(publish_data_raw, value_type=bool),
            }],
        ),
    ])
