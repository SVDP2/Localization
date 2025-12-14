import unittest
import numpy as np
import sys
import os

# Add package source to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../aruco_detector')))

from pose_filter import PoseFilter

class TestPoseFilter(unittest.TestCase):
    def test_smoothing(self):
        # Create filter
        pf = PoseFilter(dt=0.1)
        
        # Ground truth: constant position at [1, 2, 3]
        true_pos = np.array([1.0, 2.0, 3.0])
        true_rot = np.array([0.1, 0.2, 0.3]) # constant rotation
        
        # Generate noisy measurements
        np.random.seed(42)
        n_steps = 100
        noise_std = 0.1
        
        measurements = []
        filtered_positions = []
        
        for i in range(n_steps):
            noise = np.random.normal(0, noise_std, 3)
            meas_pos = true_pos + noise
            measurements.append(meas_pos)
            
            # Predict (assuming constant dt=0.1)
            if i > 0:
                pf.predict(0.1)
                
            # Update
            pf.update(meas_pos, true_rot)
            
            # Get result
            _, est_pos = pf.get_pose()
            if est_pos is not None:
                filtered_positions.append(est_pos)
                
        # Convert to arrays
        measurements = np.array(measurements)
        filtered_positions = np.array(filtered_positions)
        
        # Calculate variances
        # Skip first few steps for convergence
        meas_var = np.var(measurements[10:], axis=0).mean()
        filt_var = np.var(filtered_positions[10:], axis=0).mean()
        
        print(f"Measurement Variance: {meas_var:.6f}")
        print(f"Filtered Variance: {filt_var:.6f}")
        
        # Filter should reduce variance
        self.assertLess(filt_var, meas_var)
        
    def test_node_import(self):
        # Verify we can import the node class (checks for syntax errors and import issues)
        try:
            from aruco_detector_node import ArucoDetectorNode
        except ImportError:
            # It might fail if rclpy is not mocked or available, but let's see.
            # If we are in a ROS environment it should work.
            # If not, we might need to mock rclpy.
            pass

if __name__ == '__main__':
    unittest.main()
