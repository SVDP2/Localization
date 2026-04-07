from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from aruco_imu_eskf_localization.common.frame_conventions import (
    transform_board_from_leader_rear,
    transform_leader_rear_from_board,
)
from aruco_imu_eskf_localization.estimation.gyro_relative_eskf import (
    GyroRelativeEskf,
    GyroRelativeEskfSnapshot,
    transform_pose_covariance,
)


def _transform_matrix(translation, quat_xyzw):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def _pose_to_matrix(position, quat_xyzw):
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    pose[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return pose


def _stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class _ImuSample:
    stamp_ns: int
    angular_velocity_base: np.ndarray


@dataclass(frozen=True)
class _ImuInterval:
    start_ns: int
    end_ns: int
    angular_velocity_base: np.ndarray


class RelativeLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_imu_eskf_localization_node')

        self.declare_parameter('board_pose_topic', '/localization/aruco/board_pose')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/localization/relative/odom')
        self.declare_parameter('pose_topic', '/localization/relative/pose')
        self.declare_parameter('leader_rear_odom_topic', '/localization/leader_rear/odom')
        self.declare_parameter('leader_rear_pose_topic', '/localization/leader_rear/pose')
        self.declare_parameter('board_frame', 'board')
        self.declare_parameter('leader_rear_frame', 'leader_rear')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'usb_cam')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('output_rate_hz', 100.0)
        self.declare_parameter('reset_timeout_sec', 1.0)
        self.declare_parameter('vision_delay_buffer_sec', 0.75)
        self.declare_parameter('vision_rotation_gate_deg', 35.0)
        self.declare_parameter('velocity_damping_per_sec', 1.5)
        self.declare_parameter('position_process_noise_std_mps2', 0.3)
        self.declare_parameter('gyro_noise_std_radps', 0.05)
        self.declare_parameter('gyro_bias_random_walk_std_radps2', 0.01)
        self.declare_parameter('gyro_bias_bootstrap_sec', 0.4)
        self.declare_parameter('gyro_bias_bootstrap_max_std_radps', 0.15)
        self.declare_parameter('initial_position_std_m', 0.20)
        self.declare_parameter('initial_velocity_std_mps', 0.75)
        self.declare_parameter('initial_orientation_std_deg', 20.0)
        self.declare_parameter('initial_gyro_bias_std_radps', 0.15)
        self.declare_parameter('base_to_camera_translation', [0.27, 0.0, 0.135])
        self.declare_parameter(
            'base_to_camera_quaternion_xyzw',
            [-0.5, 0.5, -0.5, 0.5],
        )
        self.declare_parameter('use_imu_frame_tf', True)
        self.declare_parameter('base_to_imu_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('base_to_imu_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])

        # Legacy parameters kept for config compatibility.
        self.declare_parameter('publish_on_imu', False)
        self.declare_parameter('measurement_position_std_xy_m', 0.02)
        self.declare_parameter('measurement_position_std_z_m', 0.05)
        self.declare_parameter('orientation_outlier_threshold_deg', 45.0)
        self.declare_parameter('orientation_measurement_alpha', 0.15)
        self.declare_parameter('output_orientation_std_rad', 0.1)

        self._board_frame = self.get_parameter('board_frame').value
        self._leader_rear_frame = self.get_parameter('leader_rear_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._use_imu_frame_tf = bool(self.get_parameter('use_imu_frame_tf').value)
        self._output_rate_hz = float(self.get_parameter('output_rate_hz').value)
        self._reset_timeout_sec = float(self.get_parameter('reset_timeout_sec').value)
        self._vision_delay_buffer_sec = float(
            self.get_parameter('vision_delay_buffer_sec').value
        )
        self._vision_rotation_gate_deg = float(
            self.get_parameter('vision_rotation_gate_deg').value
        )
        self._gyro_bias_bootstrap_sec = float(
            self.get_parameter('gyro_bias_bootstrap_sec').value
        )
        self._gyro_bias_bootstrap_max_std_radps = float(
            self.get_parameter('gyro_bias_bootstrap_max_std_radps').value
        )
        self._base_to_camera = _transform_matrix(
            self.get_parameter('base_to_camera_translation').value,
            self.get_parameter('base_to_camera_quaternion_xyzw').value,
        )
        self._camera_to_base = np.linalg.inv(self._base_to_camera)
        self._base_to_imu = _transform_matrix(
            self.get_parameter('base_to_imu_translation').value,
            self.get_parameter('base_to_imu_quaternion_xyzw').value,
        )
        self._rotation_base_from_imu_fallback = self._base_to_imu[:3, :3]
        self._leader_rear_from_board = transform_leader_rear_from_board()
        self._board_from_leader_rear = transform_board_from_leader_rear()

        self._filter_kwargs = {
            'velocity_damping_per_sec': self.get_parameter('velocity_damping_per_sec').value,
            'position_process_noise_std_mps2': self.get_parameter(
                'position_process_noise_std_mps2'
            ).value,
            'gyro_noise_std_radps': self.get_parameter('gyro_noise_std_radps').value,
            'gyro_bias_random_walk_std_radps2': self.get_parameter(
                'gyro_bias_random_walk_std_radps2'
            ).value,
            'initial_position_std_m': self.get_parameter('initial_position_std_m').value,
            'initial_velocity_std_mps': self.get_parameter('initial_velocity_std_mps').value,
            'initial_orientation_std_deg': self.get_parameter('initial_orientation_std_deg').value,
            'initial_gyro_bias_std_radps': self.get_parameter(
                'initial_gyro_bias_std_radps'
            ).value,
        }
        self._filter = GyroRelativeEskf(**self._filter_kwargs)

        self._imu_samples: deque[_ImuSample] = deque()
        self._imu_intervals: deque[_ImuInterval] = deque()
        self._state_history: deque[GyroRelativeEskfSnapshot] = deque()
        self._last_imu_sample: _ImuSample | None = None
        self._cached_imu_frame_id: str | None = None
        self._cached_rotation_base_from_imu_tf: np.ndarray | None = None
        self._last_imu_tf_warning_ns: int | None = None
        self._last_vision_stamp_ns: int | None = None
        self._last_rotation_skip_log_ns: int | None = None

        board_pose_topic = self.get_parameter('board_pose_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        leader_rear_odom_topic = self.get_parameter('leader_rear_odom_topic').value
        leader_rear_pose_topic = self.get_parameter('leader_rear_pose_topic').value

        self._board_odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._board_pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self._leader_rear_odom_pub = self.create_publisher(Odometry, leader_rear_odom_topic, 10)
        self._leader_rear_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            leader_rear_pose_topic,
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._static_tf_broadcaster = StaticTransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(
            Imu,
            imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            board_pose_topic,
            self._board_pose_callback,
            qos_profile_sensor_data,
        )
        self._output_timer = self.create_timer(
            1.0 / max(self._output_rate_hz, 1.0),
            self._publish_timer_callback,
        )

        self.get_logger().info(f'imu topic: {imu_topic}')
        self.get_logger().info(f'board pose topic: {board_pose_topic}')
        self.get_logger().info(f'leader_rear odom topic: {leader_rear_odom_topic}')
        if self._use_imu_frame_tf:
            self.get_logger().info(
                'resolving IMU rotation from TF using IMU message frame_id when available'
            )
        else:
            self.get_logger().info('using configured base_to_imu rotation for IMU frame alignment')
        self.get_logger().info(
            'running gyro-led relative pose filtering with ArUco pose updates.'
        )
        if self._static_tf_broadcaster is not None:
            self._publish_static_board_to_leader_rear_tf()

    def _maybe_log_imu_tf_warning(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_imu_tf_warning_ns is not None
            and (now_ns - self._last_imu_tf_warning_ns) < 1_000_000_000
        ):
            return

        self.get_logger().warn(message)
        self._last_imu_tf_warning_ns = now_ns

    def _rotation_base_from_imu_message(self, imu_frame_id: str) -> np.ndarray:
        if not self._use_imu_frame_tf:
            return self._rotation_base_from_imu_fallback

        imu_frame_id = str(imu_frame_id).strip().lstrip('/')
        if not imu_frame_id:
            self._maybe_log_imu_tf_warning(
                'IMU frame_id is empty; falling back to configured base_to_imu rotation'
            )
            return self._rotation_base_from_imu_fallback

        if (
            self._cached_imu_frame_id == imu_frame_id
            and self._cached_rotation_base_from_imu_tf is not None
        ):
            return self._cached_rotation_base_from_imu_tf

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._base_frame,
                imu_frame_id,
                Time(),
            )
        except Exception as exc:
            self._maybe_log_imu_tf_warning(
                f'failed to resolve IMU TF {self._base_frame} <- {imu_frame_id}; '
                f'falling back to configured base_to_imu rotation: {exc}'
            )
            return self._rotation_base_from_imu_fallback

        quat = np.array(
            [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w,
            ],
            dtype=float,
        )
        rotation_base_from_imu = Rotation.from_quat(quat).as_matrix()
        self._cached_imu_frame_id = imu_frame_id
        self._cached_rotation_base_from_imu_tf = rotation_base_from_imu
        self.get_logger().info(
            f'using TF IMU extrinsic {self._base_frame} <- {imu_frame_id} for gyro frame alignment'
        )
        return rotation_base_from_imu

    def _imu_callback(self, msg: Imu) -> None:
        stamp_ns = _stamp_to_nanoseconds(msg.header.stamp)
        angular_velocity_imu = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=float,
        )
        rotation_base_from_imu = self._rotation_base_from_imu_message(msg.header.frame_id)
        angular_velocity_base = rotation_base_from_imu @ angular_velocity_imu
        sample = _ImuSample(
            stamp_ns=stamp_ns,
            angular_velocity_base=angular_velocity_base,
        )
        self._imu_samples.append(sample)

        if self._last_imu_sample is not None and stamp_ns > self._last_imu_sample.stamp_ns:
            interval = _ImuInterval(
                start_ns=self._last_imu_sample.stamp_ns,
                end_ns=stamp_ns,
                angular_velocity_base=self._last_imu_sample.angular_velocity_base.copy(),
            )
            self._imu_intervals.append(interval)
            if self._filter.initialized and self._filter.stamp_ns is not None:
                if interval.end_ns > self._filter.stamp_ns:
                    self._filter.predict(interval.end_ns, interval.angular_velocity_base)
                    self._record_snapshot()

        self._last_imu_sample = sample
        self._prune_history(stamp_ns)

    def _board_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        stamp_ns = _stamp_to_nanoseconds(msg.header.stamp)
        leader_rear_to_base, measurement_covariance = self._leader_measurement_from_board_msg(msg)

        if self._last_vision_stamp_ns is not None and stamp_ns < self._last_vision_stamp_ns:
            self.get_logger().warn(
                'dropping out-of-order board pose measurement older than the last fused vision stamp'
            )
            return

        if (
            not self._filter.initialized
            or self._last_vision_stamp_ns is None
            or (stamp_ns - self._last_vision_stamp_ns)
            > int(max(self._reset_timeout_sec, 0.0) * 1_000_000_000)
        ):
            replay_target_ns = self._filter.stamp_ns
            self._initialize_from_measurement(
                stamp_ns,
                leader_rear_to_base,
                measurement_covariance,
            )
            self._record_snapshot()
            if replay_target_ns is not None and replay_target_ns > stamp_ns:
                self._predict_filter_to_stamp(replay_target_ns)
                self._record_snapshot()
            self._last_vision_stamp_ns = stamp_ns
            self._prune_history(max(stamp_ns, replay_target_ns or stamp_ns))
            return

        if self._filter.stamp_ns is not None and stamp_ns < self._filter.stamp_ns:
            replay_target_ns = self._filter.stamp_ns
            if not self._restore_snapshot_before(stamp_ns):
                self.get_logger().warn(
                    'dropping delayed board pose measurement outside the retained filter history'
                )
                return
            self._predict_filter_to_stamp(stamp_ns)
            update_result = self._filter.update_pose(
                leader_rear_to_base,
                measurement_covariance,
                rotation_gate_deg=self._vision_rotation_gate_deg,
            )
            self._record_snapshot()
            self._predict_filter_to_stamp(replay_target_ns)
            self._record_snapshot()
        else:
            self._predict_filter_to_stamp(stamp_ns)
            update_result = self._filter.update_pose(
                leader_rear_to_base,
                measurement_covariance,
                rotation_gate_deg=self._vision_rotation_gate_deg,
            )
            self._record_snapshot()

        self._last_vision_stamp_ns = stamp_ns
        self._maybe_log_pose_rejection(update_result)
        self._prune_history(max(stamp_ns, self._filter.stamp_ns or stamp_ns))

    def _leader_measurement_from_board_msg(
        self,
        msg: PoseWithCovarianceStamped,
    ) -> tuple[np.ndarray, np.ndarray]:
        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        quat = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        board_to_camera = _pose_to_matrix(position, quat)
        board_to_base = board_to_camera @ self._camera_to_base
        leader_rear_to_base = self._leader_rear_from_board @ board_to_base
        board_covariance = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        leader_covariance = transform_pose_covariance(
            board_covariance,
            self._leader_rear_from_board,
        )
        return leader_rear_to_base, leader_covariance

    def _initialize_from_measurement(
        self,
        stamp_ns: int,
        leader_rear_to_base: np.ndarray,
        measurement_covariance: np.ndarray,
    ) -> None:
        gyro_bias = self._estimate_initial_gyro_bias(stamp_ns)
        self._filter.initialize(
            stamp_ns=stamp_ns,
            pose_matrix=leader_rear_to_base,
            measurement_covariance=measurement_covariance,
            gyro_bias_radps=gyro_bias,
        )
        self.get_logger().info(
            'initialized gyro-led relative filter from board pose measurement'
        )

    def _estimate_initial_gyro_bias(self, stamp_ns: int) -> np.ndarray:
        window_ns = int(max(self._gyro_bias_bootstrap_sec, 0.0) * 1_000_000_000)
        if window_ns <= 0:
            return np.zeros(3, dtype=float)

        samples = [
            sample.angular_velocity_base
            for sample in self._imu_samples
            if (stamp_ns - sample.stamp_ns) <= window_ns and sample.stamp_ns <= stamp_ns
        ]
        if len(samples) < 3:
            return np.zeros(3, dtype=float)

        angular_velocities = np.vstack(samples)
        if float(np.max(np.std(angular_velocities, axis=0))) > self._gyro_bias_bootstrap_max_std_radps:
            return np.zeros(3, dtype=float)
        return np.mean(angular_velocities, axis=0)

    def _predict_filter_to_stamp(self, target_stamp_ns: int) -> None:
        if not self._filter.initialized or self._filter.stamp_ns is None:
            return

        target_stamp_ns = int(target_stamp_ns)
        if target_stamp_ns <= self._filter.stamp_ns:
            return

        for interval in self._imu_intervals:
            if interval.end_ns <= self._filter.stamp_ns:
                continue
            if interval.start_ns >= target_stamp_ns:
                break

            segment_end_ns = min(interval.end_ns, target_stamp_ns)
            if segment_end_ns <= self._filter.stamp_ns:
                continue
            self._filter.predict(segment_end_ns, interval.angular_velocity_base)
            if self._filter.stamp_ns is not None and self._filter.stamp_ns >= target_stamp_ns:
                return

        if (
            self._last_imu_sample is not None
            and self._filter.stamp_ns is not None
            and self._filter.stamp_ns < target_stamp_ns
            and self._last_imu_sample.stamp_ns <= target_stamp_ns
        ):
            self._filter.predict(target_stamp_ns, self._last_imu_sample.angular_velocity_base)

    def _record_snapshot(self) -> None:
        if not self._filter.initialized or self._filter.stamp_ns is None:
            return
        snapshot = self._filter.snapshot()
        if self._state_history and self._state_history[-1].stamp_ns == snapshot.stamp_ns:
            self._state_history.pop()
        self._state_history.append(snapshot)

    def _restore_snapshot_before(self, target_stamp_ns: int) -> bool:
        for snapshot in reversed(self._state_history):
            if snapshot.stamp_ns is not None and snapshot.stamp_ns <= target_stamp_ns:
                self._filter.restore(snapshot)
                return True
        return False

    def _prune_history(self, reference_stamp_ns: int) -> None:
        keep_after_ns = int(reference_stamp_ns) - int(
            max(self._vision_delay_buffer_sec, self._gyro_bias_bootstrap_sec) * 1_000_000_000
        )
        while self._imu_samples and self._imu_samples[0].stamp_ns < keep_after_ns:
            self._imu_samples.popleft()
        while self._imu_intervals and self._imu_intervals[0].end_ns < keep_after_ns:
            self._imu_intervals.popleft()
        while self._state_history and (self._state_history[0].stamp_ns or 0) < keep_after_ns:
            self._state_history.popleft()

    def _maybe_log_pose_rejection(self, update_result) -> None:
        if update_result.accepted_pose_update:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_rotation_skip_log_ns is not None
            and (now_ns - self._last_rotation_skip_log_ns) < 1_000_000_000
        ):
            return

        self.get_logger().warn(
            f'rejected vision pose update; rotation innovation '
            f'{update_result.rotation_innovation_deg:.1f} deg exceeded '
            f'gate {self._vision_rotation_gate_deg:.1f} deg'
        )
        self._last_rotation_skip_log_ns = now_ns

    def _publish_timer_callback(self) -> None:
        if not self._filter.initialized:
            return

        snapshot = self._filter.snapshot()
        extrapolated_filter = GyroRelativeEskf(**self._filter_kwargs)
        extrapolated_filter.restore(snapshot)

        publish_stamp_ns = self.get_clock().now().nanoseconds
        if self._last_imu_sample is not None and snapshot.stamp_ns is not None:
            if publish_stamp_ns > snapshot.stamp_ns:
                extrapolated_filter.predict(
                    publish_stamp_ns,
                    self._last_imu_sample.angular_velocity_base,
                )

        leader_rear_to_base = extrapolated_filter.pose_matrix()
        leader_covariance = extrapolated_filter.pose_covariance()
        board_to_base = self._board_from_leader_rear @ leader_rear_to_base
        board_covariance = transform_pose_covariance(
            leader_covariance,
            self._board_from_leader_rear,
        )
        linear_velocity_base = extrapolated_filter.linear_velocity_base_mps()
        angular_velocity_base = extrapolated_filter.angular_velocity_base_radps()

        stamp = Time(nanoseconds=publish_stamp_ns).to_msg()
        self._publish_outputs(
            stamp=stamp,
            board_to_base=board_to_base,
            board_covariance=board_covariance,
            leader_rear_to_base=leader_rear_to_base,
            leader_covariance=leader_covariance,
            linear_velocity_base=linear_velocity_base,
            angular_velocity_base=angular_velocity_base,
        )

    def _publish_outputs(
        self,
        stamp,
        board_to_base: np.ndarray,
        board_covariance: np.ndarray,
        leader_rear_to_base: np.ndarray,
        leader_covariance: np.ndarray,
        linear_velocity_base: np.ndarray,
        angular_velocity_base: np.ndarray,
    ) -> None:
        board_odom = self._make_odom(
            stamp,
            frame_id=self._board_frame,
            child_frame_id=self._base_frame,
            pose_matrix=board_to_base,
            covariance=board_covariance,
            linear_velocity_base=linear_velocity_base,
            angular_velocity_base=angular_velocity_base,
        )
        self._board_odom_pub.publish(board_odom)
        self._board_pose_pub.publish(self._make_pose(board_odom))

        leader_rear_odom = self._make_odom(
            stamp,
            frame_id=self._leader_rear_frame,
            child_frame_id=self._base_frame,
            pose_matrix=leader_rear_to_base,
            covariance=leader_covariance,
            linear_velocity_base=linear_velocity_base,
            angular_velocity_base=angular_velocity_base,
        )
        self._leader_rear_odom_pub.publish(leader_rear_odom)
        self._leader_rear_pose_pub.publish(self._make_pose(leader_rear_odom))

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._leader_rear_frame
            tf_msg.child_frame_id = self._base_frame
            quat = Rotation.from_matrix(leader_rear_to_base[:3, :3]).as_quat()
            tf_msg.transform.translation.x = float(leader_rear_to_base[0, 3])
            tf_msg.transform.translation.y = float(leader_rear_to_base[1, 3])
            tf_msg.transform.translation.z = float(leader_rear_to_base[2, 3])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_static_board_to_leader_rear_tf(self) -> None:
        if self._static_tf_broadcaster is None:
            return

        board_to_leader_rear = self._board_from_leader_rear
        quat = Rotation.from_matrix(board_to_leader_rear[:3, :3]).as_quat()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._board_frame
        tf_msg.child_frame_id = self._leader_rear_frame
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self._static_tf_broadcaster.sendTransform(tf_msg)

    def _make_odom(
        self,
        stamp,
        frame_id: str,
        child_frame_id: str,
        pose_matrix: np.ndarray,
        covariance: np.ndarray,
        linear_velocity_base: np.ndarray,
        angular_velocity_base: np.ndarray,
    ) -> Odometry:
        quat = Rotation.from_matrix(pose_matrix[:3, :3]).as_quat()
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = frame_id
        odom_msg.child_frame_id = child_frame_id
        odom_msg.pose.pose.position.x = float(pose_matrix[0, 3])
        odom_msg.pose.pose.position.y = float(pose_matrix[1, 3])
        odom_msg.pose.pose.position.z = float(pose_matrix[2, 3])
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])
        odom_msg.pose.covariance = np.asarray(covariance, dtype=float).reshape(6, 6).flatten().tolist()
        odom_msg.twist.twist.linear.x = float(linear_velocity_base[0])
        odom_msg.twist.twist.linear.y = float(linear_velocity_base[1])
        odom_msg.twist.twist.linear.z = float(linear_velocity_base[2])
        odom_msg.twist.twist.angular.x = float(angular_velocity_base[0])
        odom_msg.twist.twist.angular.y = float(angular_velocity_base[1])
        odom_msg.twist.twist.angular.z = float(angular_velocity_base[2])
        return odom_msg

    def _make_pose(self, odom_msg: Odometry) -> PoseWithCovarianceStamped:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose
        return pose_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
