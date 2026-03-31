from launch import LaunchDescription
from launch_ros.actions import Node


# Vehicle frame convention: FLU (x forward, y left, z up)
# base_link: rear axle center
# Units below are meters.
TRACK_WIDTH_M = 0.235
WHEEL_BASE_M = 0.325

# Wheel diameter is approximately 108-110 mm, and the wheel axle center is
# approximately 55 mm above the ground.
WHEEL_RADIUS_M = 0.055

HALF_TRACK_M = TRACK_WIDTH_M * 0.5
USB_CAM_OPTICAL_QUAT_XYZW = (-0.5, 0.5, -0.5, 0.5)

TRANSFORMS = (
    {
        'name': 'base_link_to_base_imu_link_static_tf',
        'parent': 'base_link',
        'child': 'base_imu_link',
        'translation': (0.060, 0.0, 0.150 - WHEEL_RADIUS_M),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
    {
        'name': 'base_link_to_usb_cam_static_tf',
        'parent': 'base_link',
        'child': 'usb_cam',
        'translation': (0.270, 0.0, 0.190 - WHEEL_RADIUS_M),
        'quaternion': USB_CAM_OPTICAL_QUAT_XYZW,
    },
    {
        'name': 'base_link_to_front_left_wheel_static_tf',
        'parent': 'base_link',
        'child': 'front_left_wheel',
        'translation': (WHEEL_BASE_M, HALF_TRACK_M, 0.0),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
    {
        'name': 'base_link_to_front_right_wheel_static_tf',
        'parent': 'base_link',
        'child': 'front_right_wheel',
        'translation': (WHEEL_BASE_M, -HALF_TRACK_M, 0.0),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
    {
        'name': 'base_link_to_rear_left_wheel_static_tf',
        'parent': 'base_link',
        'child': 'rear_left_wheel',
        'translation': (0.0, HALF_TRACK_M, 0.0),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
    {
        'name': 'base_link_to_rear_right_wheel_static_tf',
        'parent': 'base_link',
        'child': 'rear_right_wheel',
        'translation': (0.0, -HALF_TRACK_M, 0.0),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
)


def _static_tf_node(spec):
    tx, ty, tz = spec['translation']
    qx, qy, qz, qw = spec['quaternion']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=spec['name'],
        output='screen',
        arguments=[
            '--x', str(tx),
            '--y', str(ty),
            '--z', str(tz),
            '--qx', str(qx),
            '--qy', str(qy),
            '--qz', str(qz),
            '--qw', str(qw),
            '--frame-id', spec['parent'],
            '--child-frame-id', spec['child'],
        ],
    )


def generate_launch_description():
    return LaunchDescription([_static_tf_node(spec) for spec in TRANSFORMS])
