import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from aruco_imu_eskf_localization.nodes.absolute_follower_odom_node import (
    compose_absolute_follower_odom,
)


def _odom(frame_id, child_frame_id, position, yaw_rad):
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(position[0])
    msg.pose.pose.position.y = float(position[1])
    msg.pose.pose.position.z = float(position[2])
    quat = Rotation.from_euler('z', yaw_rad).as_quat()
    msg.pose.pose.orientation.x = float(quat[0])
    msg.pose.pose.orientation.y = float(quat[1])
    msg.pose.pose.orientation.z = float(quat[2])
    msg.pose.pose.orientation.w = float(quat[3])
    msg.pose.covariance = [0.0] * 36
    msg.twist.covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        msg.pose.covariance[index] = 0.01
        msg.twist.covariance[index] = 0.01
    return msg


def _tf(parent_frame, child_frame, translation, yaw_rad):
    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(translation[0])
    msg.transform.translation.y = float(translation[1])
    msg.transform.translation.z = float(translation[2])
    quat = Rotation.from_euler('z', yaw_rad).as_quat()
    msg.transform.rotation.x = float(quat[0])
    msg.transform.rotation.y = float(quat[1])
    msg.transform.rotation.z = float(quat[2])
    msg.transform.rotation.w = float(quat[3])
    return msg


class TestAbsoluteFollowerOdom(unittest.TestCase):
    def test_composes_leader_and_relative_pose(self):
        leader = _odom('map', 'leader/base_link', [10.0, 2.0, 0.0], np.pi / 2.0)
        relative = _odom(
            'leader/leader_rear',
            'follower/base_link',
            [-2.0, 1.0, 0.0],
            0.0,
        )
        leader_base_to_rear = _tf(
            'leader/base_link',
            'leader/leader_rear',
            [0.5, 0.0, 0.0],
            0.0,
        )

        output = compose_absolute_follower_odom(
            stamp=relative.header.stamp,
            leader_odom=leader,
            relative_odom=relative,
            leader_base_to_rear=leader_base_to_rear,
            map_frame='map',
            follower_base_frame='follower/base_link',
            min_variance=1.0e-6,
        )

        self.assertEqual(output.header.frame_id, 'map')
        self.assertEqual(output.child_frame_id, 'follower/base_link')
        self.assertTrue(
            np.allclose(
                [
                    output.pose.pose.position.x,
                    output.pose.pose.position.y,
                    output.pose.pose.position.z,
                ],
                [9.0, 0.5, 0.0],
                atol=1.0e-9,
            )
        )

    def test_composes_map_velocity_with_rotating_leader_frame(self):
        leader = _odom('map', 'leader/base_link', [0.0, 0.0, 0.0], 0.0)
        leader.twist.twist.linear.x = 1.0
        leader.twist.twist.angular.z = 0.5
        relative = _odom(
            'leader/leader_rear',
            'follower/base_link',
            [-2.0, 0.0, 0.0],
            0.0,
        )
        relative.twist.twist.linear.x = 0.2
        leader_base_to_rear = _tf(
            'leader/base_link',
            'leader/leader_rear',
            [0.0, 0.0, 0.0],
            0.0,
        )

        output = compose_absolute_follower_odom(
            stamp=relative.header.stamp,
            leader_odom=leader,
            relative_odom=relative,
            leader_base_to_rear=leader_base_to_rear,
            map_frame='map',
            follower_base_frame='follower/base_link',
            min_variance=1.0e-6,
        )

        self.assertTrue(
            np.allclose(
                [
                    output.twist.twist.linear.x,
                    output.twist.twist.linear.y,
                    output.twist.twist.linear.z,
                ],
                [1.2, -1.0, 0.0],
                atol=1.0e-9,
            )
        )


if __name__ == '__main__':
    unittest.main()
