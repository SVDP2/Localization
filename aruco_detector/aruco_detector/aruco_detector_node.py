#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from aruco_detector.pose_filter import PoseFilter


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Declare parameters
        self.declare_parameter('marker_config_file', 'config/markers_board.yaml')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('camera_calibration_file', 'config/cam_intrinsic.yaml')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('image_topic', '/usb_cam_3/image_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        # Present in config/params.yaml; declare to avoid ParameterNotDeclaredException
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_prefix', 'aruco_marker_')
        self.declare_parameter('use_board_pose', True)  # Use board-based pose estimation
        self.declare_parameter('min_markers_for_board', 3)  # Minimum markers needed for board pose

        # Get parameters
        marker_config_file = self.get_parameter('marker_config_file').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        camera_calibration_file = self.get_parameter('camera_calibration_file').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        image_topic = self.get_parameter('image_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        # Currently not used in the processing pipeline, but kept for compatibility with params.yaml
        self.detection_threshold = float(self.get_parameter('detection_threshold').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.use_board_pose = self.get_parameter('use_board_pose').value
        self.min_markers_for_board = self.get_parameter('min_markers_for_board').value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Load marker configurations and board geometry
        self.marker_sizes, self.board_geometry = self._load_marker_config(marker_config_file)
        self.get_logger().info(f'Loaded {len(self.marker_sizes)} marker configurations')
        if self.board_geometry:
            self.get_logger().info(f'Loaded board geometry with origin at marker {self.board_geometry["origin_marker_id"]}')

        # Load camera calibration
        self.camera_matrix, self.dist_coeffs = self._load_camera_calibration(camera_calibration_file)
        self.get_logger().info('Loaded camera calibration')

        # Initialize ArUco detector
        aruco_dict_id = getattr(cv2.aruco, aruco_dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.get_logger().info(f'Initialized ArUco detector with {aruco_dict_name}')

        # Initialize TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info('TF broadcaster initialized')

        # Subscribe to image topic
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {image_topic}')
        
        # Subscribe to IMU topic
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            100 # Higher queue size for IMU
        )
        self.get_logger().info(f'Subscribed to {imu_topic}')

        # Publisher for debug image
        self.debug_image_pub = self.create_publisher(
            Image,
            '/aruco_detector/debug_image',
            10
        )
        self.get_logger().info('Debug image publisher created')

        # Initialize filters
        self.filters = {}  # Dictionary to store PoseFilter for each marker ID (and 'board')
        self.last_update_times = {}  # Dictionary to store last update time for each ID
        self.last_imu_time = None

    def _load_marker_config(self, config_file):
        """Load marker configurations and board geometry from YAML file"""
        try:
            package_share_dir = get_package_share_directory('aruco_detector')
            config_path = os.path.join(package_share_dir, config_file)

            # If not found in share directory, try relative to package
            if not os.path.exists(config_path):
                # Try from src directory during development
                config_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    config_file
                )

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Create dictionary mapping marker ID to size (in mm)
            marker_sizes = {}
            for marker in config['markers']:
                marker_sizes[marker['id']] = marker['size']

            # Load board geometry if available
            board_geometry = config.get('board_geometry', None)

            return marker_sizes, board_geometry

        except Exception as e:
            self.get_logger().error(f'Failed to load marker config: {e}')
            return {}, None

    def _load_camera_calibration(self, calib_file):
        """Load camera calibration from YAML file"""
        try:
            package_share_dir = get_package_share_directory('aruco_detector')
            calib_path = os.path.join(package_share_dir, calib_file)

            # If not found in share directory, try relative to package
            if not os.path.exists(calib_path):
                calib_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    calib_file
                )

            with open(calib_path, 'r') as f:
                calib = yaml.safe_load(f)

            # Extract camera matrix (3x3)
            camera_matrix = np.array(calib['camera_matrix']['data'], dtype=np.float64)

            # Extract distortion coefficients (1x5)
            dist_coeffs = np.array(calib['distortion_coefficients']['data'], dtype=np.float64)

            return camera_matrix, dist_coeffs

        except Exception as e:
            self.get_logger().error(f'Failed to load camera calibration: {e}')
            # Return default values
            return np.eye(3, dtype=np.float64), np.zeros(5, dtype=np.float64)

    def _get_object_points(self, marker_size_mm):
        """
        Generate 3D object points for ArUco marker corners
        marker_size_mm: size in millimeters
        Returns: 4x3 array of 3D points in meters
        """
        # Convert mm to meters
        half_size = (marker_size_mm / 1000.0) / 2.0

        # Define 4 corners of the marker in 3D (marker coordinate system)
        # Order: top-left, top-right, bottom-right, bottom-left
        object_points = np.array([
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0]
        ], dtype=np.float64)

        return object_points

    def _estimate_pose(self, corners, marker_id):
        """
        Estimate pose of detected marker using solvePnP
        corners: detected corner points (4x2 array)
        marker_id: ID of the marker
        Returns: (rvec, tvec) or (None, None) if estimation fails
        """
        if marker_id not in self.marker_sizes:
            self.get_logger().warn(f'Marker ID {marker_id} not found in config')
            return None, None

        # Get marker size and generate 3D object points
        marker_size = self.marker_sizes[marker_id]
        object_points = self._get_object_points(marker_size)

        # Reshape corners to (4, 1, 2) format expected by solvePnP
        image_points = corners.reshape((4, 1, 2))

        try:
            # Solve PnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if success:
                return rvec, tvec
            else:
                self.get_logger().warn(f'solvePnP failed for marker {marker_id}')
                return None, None

        except Exception as e:
            self.get_logger().error(f'Error in pose estimation for marker {marker_id}: {e}')
            return None, None

    def _estimate_board_pose(self, corners, ids):
        """
        Estimate board pose using multiple detected markers.
        Uses all visible markers to compute a single board pose.
        Returns: (rvec, tvec, num_markers_used) or (None, None, 0) if insufficient markers
        """
        if not self.board_geometry or 'marker_positions' not in self.board_geometry:
            return None, None, 0

        marker_positions = self.board_geometry['marker_positions']
        detected_ids = ids.flatten()

        # Collect 3D-2D point correspondences from all detected markers
        object_points_list = []
        image_points_list = []
        valid_marker_count = 0

        for i, marker_id in enumerate(detected_ids):
            marker_id = int(marker_id)

            # Check if this marker is defined in board geometry
            if marker_id not in marker_positions:
                continue

            # Get marker position on board (in mm)
            board_pos = marker_positions[marker_id]
            if board_pos is None:
                continue

            # Get marker size
            if marker_id not in self.marker_sizes:
                continue

            marker_size_mm = self.marker_sizes[marker_id]
            marker_size_m = marker_size_mm / 1000.0

            # Get marker corners in image
            marker_corners = corners[i][0]

            # Define 3D object points for this marker's corners relative to board origin
            # Marker corners are at: center ± size/2 in x and y
            half_size = marker_size_m / 2.0
            board_pos_m = np.array(board_pos) / 1000.0  # Convert mm to m

            # Marker corners in board coordinate system
            # Assuming marker Z is on board plane (z=0)
            marker_obj_points = np.array([
                [board_pos_m[0] - half_size, board_pos_m[1] + half_size, board_pos_m[2]],  # Top-left
                [board_pos_m[0] + half_size, board_pos_m[1] + half_size, board_pos_m[2]],  # Top-right
                [board_pos_m[0] + half_size, board_pos_m[1] - half_size, board_pos_m[2]],  # Bottom-right
                [board_pos_m[0] - half_size, board_pos_m[1] - half_size, board_pos_m[2]],  # Bottom-left
            ], dtype=np.float32)

            object_points_list.append(marker_obj_points)
            image_points_list.append(marker_corners.astype(np.float32))
            valid_marker_count += 1

        # Check if we have enough markers
        if valid_marker_count < self.min_markers_for_board:
            self.get_logger().warn(
                f'Insufficient markers for board pose: {valid_marker_count}/{self.min_markers_for_board}',
                throttle_duration_sec=2.0
            )
            return None, None, 0

        # Concatenate all points
        object_points = np.vstack(object_points_list)
        image_points = np.vstack(image_points_list)

        try:
            # Solve PnP with all points
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                return rvec, tvec, valid_marker_count
            else:
                self.get_logger().warn('Board pose solvePnP failed')
                return None, None, 0

        except Exception as e:
            self.get_logger().error(f'Error in board pose estimation: {e}')
            return None, None, 0

    def imu_callback(self, msg):
        """
        Process IMU data for prediction.
        """
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_imu_time is None:
            self.last_imu_time = current_time_sec
            return
            
        dt = current_time_sec - self.last_imu_time
        self.last_imu_time = current_time_sec
        
        # Extract angular velocity
        # IMU Frame: FLU (Forward, Left, Up)
        # Camera Frame: Optical (Z-Forward, X-Right, Y-Down)
        # Transform IMU angular velocity to Camera Frame
        # IMU X -> Camera Z
        # IMU Y -> Camera -X
        # IMU Z -> Camera -Y
        
        wx_imu = msg.angular_velocity.x
        wy_imu = msg.angular_velocity.y
        wz_imu = msg.angular_velocity.z
        
        # Transform to Camera Optical Frame
        # w_cam = R_imu_to_cam * w_imu
        wx_cam = -wy_imu
        wy_cam = -wz_imu
        wz_cam = wx_imu
        
        angular_velocity_cam = np.array([wx_cam, wy_cam, wz_cam])
        
        # Predict all active filters
        for marker_id in self.filters:
            self.filters[marker_id].predict(dt, angular_velocity_cam)

    def _apply_filter(self, marker_id, rvec, tvec, current_time):
        """
        Apply EKF to smooth pose estimation.
        Args:
            marker_id: Marker ID or 'board'
            rvec, tvec: Raw pose
            current_time: ROS Time object
        Returns:
            (filtered_rvec, filtered_tvec)
        """
        # Convert ROS time to seconds (float)
        current_time_sec = current_time.nanoseconds / 1e9
        
        # Create filter if new
        if marker_id not in self.filters:
            self.filters[marker_id] = PoseFilter()
            self.last_update_times[marker_id] = current_time_sec
            # Initialize with first measurement
            self.filters[marker_id].update(tvec, rvec)
            return rvec, tvec

        # Calculate dt
        last_time_sec = self.last_update_times[marker_id]
        dt = current_time_sec - last_time_sec
        
        # Update last time
        self.last_update_times[marker_id] = current_time_sec
        
        # Reset filter if dt is too large (e.g., marker lost for > 1 second)
        if dt > 1.0:
            self.filters[marker_id] = PoseFilter()
            self.filters[marker_id].update(tvec, rvec)
            return rvec, tvec
            
        # Predict (using 0 angular velocity if no IMU update happened recently, 
        # or rely on IMU callback having already predicted)
        # Note: In a proper EKF, predict is called before update.
        # Here, imu_callback calls predict() at high rate.
        # When we get a visual measurement, we might want to predict up to this exact timestamp.
        # But since IMU rate >> Camera rate, the error is small if we just update.
        # However, we should ensure we don't double-predict if we just predicted in imu_callback.
        # For simplicity in this step, we will assume imu_callback handles the prediction integration.
        # But we DO need to handle the case where IMU is NOT available or slow.
        # So we can check if we have recent IMU data.
        
        # For now, let's just call update(). The IMU callback handles the prediction steps.
        # If we want to be precise, we should predict from last_imu_time to current_image_time.
        
        self.filters[marker_id].update(tvec, rvec)
        
        return self.filters[marker_id].get_pose()

    def _publish_camera_tf(self, rvec, tvec, timestamp, child_frame_id):
        """
        Publish TF transform: board -> camera
        solvePnP gives Camera <- Marker transform, so we invert it to get Marker <- Camera.
        This way, 'board' is the fixed world frame and camera moves relative to it.
        
        Args:
            rvec, tvec: Pose from solvePnP (Camera <- Board)
            timestamp: ROS timestamp
            child_frame_id: child frame id for the camera (e.g. '<camera_frame_id>' or '<camera_frame_id>_filtered')
        """
        if not self.publish_tf:
            return

        try:
            # solvePnP result: T_camera_board (board in camera frame)
            # We want: T_board_camera (camera in board frame)
            # T_board_camera = inv(T_camera_board)
            
            # Convert rvec to rotation matrix
            R_camera_board, _ = cv2.Rodrigues(rvec)
            tvec_flat = np.array(tvec).flatten()
            
            # Invert the transformation
            # R_board_camera = R_camera_board.T
            # t_board_camera = -R_camera_board.T @ t_camera_board
            R_board_camera = R_camera_board.T
            t_board_camera = -R_board_camera @ tvec_flat
            
            # Create TransformStamped message
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = 'board'  # Parent: fixed board frame
            t.child_frame_id = child_frame_id  # Child: camera

            # Set translation
            t.transform.translation.x = float(t_board_camera[0])
            t.transform.translation.y = float(t_board_camera[1])
            t.transform.translation.z = float(t_board_camera[2])

            # Convert rotation matrix to quaternion
            r = Rotation.from_matrix(R_board_camera)
            quat = r.as_quat()  # Returns [x, y, z, w]

            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error publishing camera TF ({child_frame_id}): {e}')

    def _publish_marker_tf(self, marker_id, rvec, tvec, timestamp):
        """
        Publish TF transform: camera_frame_id -> aruco_marker_<id>
        solvePnP gives X_cam = R * X_marker + t (camera <- marker), which matches TF parent=camera, child=marker.
        """
        if not self.publish_tf:
            return

        try:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = self.camera_frame_id
            t.child_frame_id = f'{self.tf_prefix}{marker_id}'

            tvec_flat = np.array(tvec).flatten()
            t.transform.translation.x = float(tvec_flat[0])
            t.transform.translation.y = float(tvec_flat[1])
            t.transform.translation.z = float(tvec_flat[2])

            r = Rotation.from_rotvec(np.array(rvec).flatten())
            quat = r.as_quat()  # [x, y, z, w]
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error publishing marker TF (id={marker_id}): {e}')

    def image_callback(self, msg):
        """Process incoming image and detect ArUco markers"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            debug_image = cv_image.copy()

            # Convert to grayscale for detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                current_time = self.get_clock().now()
                
                # Prefer board pose if enabled and geometry is available; otherwise fall back to per-marker pose.
                if self.use_board_pose and self.board_geometry:
                    board_rvec, board_tvec, num_markers_used = self._estimate_board_pose(corners, ids)

                    if board_rvec is not None and board_tvec is not None:
                        # Publish raw camera TF: board -> <camera_frame_id>
                        self._publish_camera_tf(board_rvec, board_tvec, msg.header.stamp, self.camera_frame_id)

                        # Apply filter to board pose
                        filtered_rvec, filtered_tvec = self._apply_filter('board', board_rvec, board_tvec, current_time)
                        
                        if filtered_rvec is not None:
                            # Publish filtered camera TF: board -> <camera_frame_id>_filtered
                            self._publish_camera_tf(
                                filtered_rvec,
                                filtered_tvec,
                                msg.header.stamp,
                                f'{self.camera_frame_id}_filtered'
                            )
                            self.get_logger().info(
                                f'Board pose estimated using {num_markers_used} markers',
                                throttle_duration_sec=1.0
                            )
                        
                        # Draw debug visualization for all detected markers
                        for i, marker_id in enumerate(ids.flatten()):
                            marker_corners = corners[i][0]
                            self._draw_marker_debug(debug_image, marker_corners, int(marker_id), board_tvec)
                else:
                    # Per-marker pose estimation + TF publish (uses marker sizes from marker_config_file)
                    for i, marker_id in enumerate(ids.flatten()):
                        marker_id_int = int(marker_id)
                        marker_corners = corners[i][0]

                        rvec, tvec = self._estimate_pose(marker_corners, marker_id_int)
                        if rvec is None or tvec is None:
                            continue

                        filtered_rvec, filtered_tvec = self._apply_filter(marker_id_int, rvec, tvec, current_time)
                        if filtered_rvec is None or filtered_tvec is None:
                            continue

                        self._publish_marker_tf(marker_id_int, filtered_rvec, filtered_tvec, msg.header.stamp)
                        self._draw_marker_debug(debug_image, marker_corners, marker_id_int, filtered_tvec)

            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing debug image: {e}')

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def _draw_marker_debug(self, image, corners, marker_id, tvec):
        """Draw marker outline, ID, and distance on debug image"""
        # Draw marker outline
        corners_int = corners.astype(int)
        cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)

        # Calculate center of marker
        center = corners.mean(axis=0).astype(int)

        # Calculate distance (Euclidean distance from camera)
        distance = np.linalg.norm(tvec) * 1000  # Convert to mm

        # Draw marker ID
        cv2.putText(
            image,
            f'ID:{marker_id}',
            tuple(center + np.array([10, -10])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        # Draw distance
        cv2.putText(
            image,
            f'{distance:.0f}mm',
            tuple(center + np.array([10, 10])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 255),
            1
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
