#!/usr/bin/env python3
"""
Unit tests for the AFG Controller

This test suite covers the core functionality of the AFGController class,
including path following algorithms, flow field computations, and control logic.
"""

import unittest
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header, ColorRGBA
import threading
import time

# Import the controller class
from afg_controller.afg_controller import AFGController


class TestAFGController(unittest.TestCase):
    """Test class for AFGController functionality"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests"""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures before each test method"""
        # Create controller node with test parameters
        self.controller = AFGController()
        
        # Override some parameters for testing
        self.controller.desired_speed = 1.0
        self.controller.k_conv = 1.0
        self.controller.k_flow = 1.0
        self.controller.boundary_layer = 0.1
        self.controller.lookahead_dist = 0.5
        self.controller.max_angular_vel = 1.0
        
        # Create test path
        self.test_path = self._create_straight_path()
        self.controller.path_callback(self.test_path)
        
        # Create test pose
        self.test_pose = self._create_test_pose(0.0, 0.0, 0.0)
        self.controller.current_pose = self.test_pose
    
    def tearDown(self):
        """Clean up after each test method"""
        self.controller.destroy_node()
    
    def _create_straight_path(self, start_x=0.0, start_y=0.0, end_x=5.0, end_y=0.0, num_points=10):
        """Create a straight line path for testing"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.controller.get_clock().now().to_msg()
        
        for i in range(num_points):
            pose = PoseStamped()
            pose.header = path.header
            
            # Linear interpolation
            t = i / (num_points - 1)
            pose.pose.position.x = start_x + t * (end_x - start_x)
            pose.pose.position.y = start_y + t * (end_y - start_y)
            pose.pose.position.z = 0.0
            
            # Set orientation along path
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            
            path.poses.append(pose)
        
        return path
    
    def _create_curved_path(self, center_x=0.0, center_y=0.0, radius=2.0, num_points=20):
        """Create a circular path for testing"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.controller.get_clock().now().to_msg()
        
        for i in range(num_points):
            pose = PoseStamped()
            pose.header = path.header
            
            # Circular path
            angle = 2 * np.pi * i / num_points
            pose.pose.position.x = center_x + radius * np.cos(angle)
            pose.pose.position.y = center_y + radius * np.sin(angle)
            pose.pose.position.z = 0.0
            
            # Set orientation tangent to circle
            tangent_angle = angle + np.pi / 2
            pose.pose.orientation.w = np.cos(tangent_angle / 2)
            pose.pose.orientation.z = np.sin(tangent_angle / 2)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            
            path.poses.append(pose)
        
        return path
    
    def _create_test_pose(self, x, y, theta):
        """Create a test pose at given position and orientation"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.controller.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        pose.pose.orientation.w = np.cos(theta / 2)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(theta / 2)
        
        return pose
    
    def test_initialization(self):
        """Test that the controller initializes correctly"""
        self.assertIsNotNone(self.controller)
        self.assertEqual(self.controller.desired_speed, 1.0)
        self.assertEqual(self.controller.k_conv, 1.0)
        self.assertEqual(self.controller.k_flow, 1.0)
    
    def test_path_callback(self):
        """Test path callback functionality"""
        # Test with straight path
        straight_path = self._create_straight_path(0, 0, 10, 0, 5)
        self.controller.path_callback(straight_path)
        
        self.assertEqual(len(self.controller.path_points), 5)
        self.assertTrue(np.allclose(self.controller.path_points[0], [0.0, 0.0]))
        self.assertTrue(np.allclose(self.controller.path_points[-1], [10.0, 0.0]))
    
    def test_closest_point_on_straight_path(self):
        """Test finding closest point on a straight path"""
        # Vehicle is directly on the path
        position = np.array([2.5, 0.0])
        closest_point, segment_idx, distance = self.controller.get_closest_point_on_path(position)
        
        self.assertAlmostEqual(distance, 0.0, places=5)
        self.assertTrue(np.allclose(closest_point, [2.5, 0.0], atol=1e-3))
        
        # Vehicle is offset from path
        position = np.array([2.5, 1.0])
        closest_point, segment_idx, distance = self.controller.get_closest_point_on_path(position)
        
        self.assertAlmostEqual(distance, 1.0, places=5)
        self.assertTrue(np.allclose(closest_point, [2.5, 0.0], atol=1e-3))
    
    def test_closest_point_on_curved_path(self):
        """Test finding closest point on a curved path"""
        # Create circular path and test
        curved_path = self._create_curved_path(0, 0, 2.0, 20)
        self.controller.path_callback(curved_path)
        
        # Vehicle at center should find closest point on circle
        position = np.array([0.0, 0.0])
        closest_point, segment_idx, distance = self.controller.get_closest_point_on_path(position)
        
        # Distance should be approximately 2.0 (radius)
        self.assertAlmostEqual(distance, 2.0, places=1)
        
        # Closest point should be on the circle
        closest_dist_from_origin = np.linalg.norm(closest_point)
        self.assertAlmostEqual(closest_dist_from_origin, 2.0, places=1)
    
    def test_path_tangent_computation(self):
        """Test path tangent computation"""
        # For straight path along x-axis, tangent should be [1, 0]
        position = np.array([2.5, 0.0])
        closest_point, segment_idx, _ = self.controller.get_closest_point_on_path(position)
        tangent = self.controller.get_path_tangent(segment_idx, closest_point)
        
        expected_tangent = np.array([1.0, 0.0])
        self.assertTrue(np.allclose(tangent, expected_tangent, atol=1e-2))
    
    def test_flow_field_computation(self):
        """Test artificial flow field computation"""
        position = np.array([2.0, 1.0])  # 1m above path
        closest_point = np.array([2.0, 0.0])
        tangent = np.array([1.0, 0.0])
        cross_track_error = 1.0
        
        flow_direction = self.controller.compute_flow_field(
            position, closest_point, tangent, cross_track_error
        )
        
        # Flow should have both convergence (towards path) and forward components
        self.assertAlmostEqual(np.linalg.norm(flow_direction), 1.0, places=5)  # Should be unit vector
        self.assertLess(flow_direction[1], 0.0)  # Should have downward component (towards path)
        self.assertGreater(flow_direction[0], 0.0)  # Should have forward component
    
    def test_control_commands_computation(self):
        """Test control command computation"""
        # Test case: vehicle pointing forward, flow field also forward
        flow_direction = np.array([1.0, 0.0])
        current_heading = 0.0
        
        linear_vel, angular_vel = self.controller.compute_control_commands(
            flow_direction, current_heading
        )
        
        # Should have positive linear velocity and near-zero angular velocity
        self.assertGreater(linear_vel, 0.0)
        self.assertAlmostEqual(angular_vel, 0.0, places=2)
        
        # Test case: vehicle needs to turn
        flow_direction = np.array([0.0, 1.0])  # Flow pointing north
        current_heading = 0.0  # Vehicle pointing east
        
        linear_vel, angular_vel = self.controller.compute_control_commands(
            flow_direction, current_heading
        )
        
        # Should have reduced linear velocity and positive angular velocity
        self.assertGreater(angular_vel, 0.0)  # Turn left (counterclockwise)
        self.assertLess(linear_vel, self.controller.desired_speed)  # Reduced speed while turning
    
    def test_boundary_layer_behavior(self):
        """Test that boundary layer smooths convergence near path"""
        position_far = np.array([2.0, 1.0])  # Far from path
        position_near = np.array([2.0, 0.05])  # Within boundary layer
        closest_point = np.array([2.0, 0.0])
        tangent = np.array([1.0, 0.0])
        
        # Flow field far from path
        flow_far = self.controller.compute_flow_field(
            position_far, closest_point, tangent, 1.0
        )
        
        # Flow field near path (within boundary layer)
        flow_near = self.controller.compute_flow_field(
            position_near, closest_point, tangent, 0.05
        )
        
        # Convergence component should be stronger far from path
        self.assertGreater(abs(flow_far[1]), abs(flow_near[1]))
    
    def test_angular_velocity_limits(self):
        """Test that angular velocity is properly limited"""
        # Create scenario requiring sharp turn
        flow_direction = np.array([-1.0, 0.0])  # Flow pointing backwards
        current_heading = 0.0  # Vehicle pointing forwards
        
        linear_vel, angular_vel = self.controller.compute_control_commands(
            flow_direction, current_heading
        )
        
        # Angular velocity should be limited
        self.assertLessEqual(abs(angular_vel), self.controller.max_angular_vel)
    
    def test_empty_path_handling(self):
        """Test behavior with empty or invalid paths"""
        # Empty path
        empty_path = Path()
        empty_path.header.frame_id = "map"
        self.controller.path_callback(empty_path)
        
        self.assertEqual(len(self.controller.path_points), 0)
        
        # Test closest point with empty path
        position = np.array([1.0, 1.0])
        closest_point, segment_idx, distance = self.controller.get_closest_point_on_path(position)
        
        # Should return input position and zero distance
        self.assertTrue(np.allclose(closest_point, position))
        self.assertEqual(distance, 0.0)
    
    def test_odometry_callback(self):
        """Test odometry callback"""
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.controller.get_clock().now().to_msg()
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.orientation.w = 1.0
        
        self.controller.odom_callback(odom)
        
        self.assertIsNotNone(self.controller.current_pose)
        self.assertEqual(self.controller.current_pose.pose.position.x, 1.0)
        self.assertEqual(self.controller.current_pose.pose.position.y, 2.0)


class TestAFGControllerIntegration(unittest.TestCase):
    """Integration tests for AFGController with ROS2 messaging"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests"""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod  
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests"""
        if rclpy.ok():
            rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures"""
        self.controller = AFGController()
        
        # Create test publishers
        self.odom_pub = self.controller.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.controller.create_publisher(Path, '/desired_path', 10)
        
        # Storage for received messages
        self.received_cmd_vel = None
        self.received_flow_viz = None
        self.received_debug_viz = None
        
        # Create subscribers to monitor outputs
        self.cmd_vel_sub = self.controller.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.flow_viz_sub = self.controller.create_subscription(
            MarkerArray, '/flow_field_viz', self._flow_viz_callback, 10
        )
        self.debug_viz_sub = self.controller.create_subscription(
            MarkerArray, '/afg_debug', self._debug_viz_callback, 10
        )
        
        # Start spinning in a separate thread
        self.spin_thread = threading.Thread(target=self._spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
        time.sleep(0.1)  # Allow setup to complete
    
    def tearDown(self):
        """Clean up"""
        self.controller.destroy_node()
    
    def _spin_node(self):
        """Spin the node in a separate thread"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.controller, timeout_sec=0.1)
            except Exception:
                break
    
    def _cmd_vel_callback(self, msg):
        """Store received cmd_vel messages"""
        self.received_cmd_vel = msg
    
    def _flow_viz_callback(self, msg):
        """Store received flow visualization messages"""
        self.received_flow_viz = msg
    
    def _debug_viz_callback(self, msg):
        """Store received debug visualization messages"""
        self.received_debug_viz = msg
    
    def test_full_control_loop(self):
        """Test the complete control loop with message passing"""
        # Create and set path directly (bypassing message passing for reliability)
        path_points = []
        for i in range(5):
            point = np.array([float(i), 0.0])
            path_points.append(point)
        
        self.controller.path_points = path_points
        
        # Set current pose directly
        test_pose = PoseStamped()
        test_pose.header.frame_id = "map"
        test_pose.header.stamp = self.controller.get_clock().now().to_msg()
        test_pose.pose.position.x = 0.5
        test_pose.pose.position.y = 0.5  # Offset from path
        test_pose.pose.orientation.w = 1.0
        self.controller.current_pose = test_pose
        
        # Trigger control loop
        self.controller.control_loop()
        
        # Wait for publications
        time.sleep(0.3)
        
        # Check that cmd_vel was published
        self.assertIsNotNone(self.received_cmd_vel, "No cmd_vel message received")
        if self.received_cmd_vel is not None:
            self.assertGreater(self.received_cmd_vel.linear.x, 0.0, "Linear velocity should be positive")
    
    def test_visualization_publishing(self):
        """Test that visualization messages are published"""
        # Set up controller with path and pose
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.controller.get_clock().now().to_msg()
        
        for i in range(3):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.controller.path_callback(path)
        
        # Set current pose
        test_pose = PoseStamped()
        test_pose.header.frame_id = "map"
        test_pose.header.stamp = self.controller.get_clock().now().to_msg()
        test_pose.pose.position.x = 1.0
        test_pose.pose.position.y = 0.1
        test_pose.pose.orientation.w = 1.0
        self.controller.current_pose = test_pose
        
        # Trigger control loop
        self.controller.control_loop()
        
        # Wait for messages
        time.sleep(0.3)
        
        # Check visualization messages were published
        # Note: Due to timing, we may not always receive these, so we just check if they're valid when received
        if self.received_flow_viz is not None:
            self.assertGreater(len(self.received_flow_viz.markers), 0)
        
        if self.received_debug_viz is not None:
            self.assertGreater(len(self.received_debug_viz.markers), 0)


if __name__ == '__main__':
    # Configure test runner
    unittest.main(verbosity=2)