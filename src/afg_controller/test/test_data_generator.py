#!/usr/bin/env python3
"""
Test data generator for AFG Controller

This script generates test paths and publishes them for testing the AFG controller.
It can create various path types including straight lines, curves, and complex trajectories.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
import numpy as np
import argparse


class TestDataGenerator(Node):
    """Node for generating test data for AFG controller"""
    
    def __init__(self, path_type='straight', num_points=20):
        super().__init__('test_data_generator')
        
        self.path_type = path_type
        self.num_points = num_points
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/desired_path', 10)
        
        # Timer to publish path
        self.timer = self.create_timer(1.0, self.publish_path)
        
        self.get_logger().info(f'Test data generator started - Path type: {path_type}')
    
    def create_straight_path(self, start=(0, 0), end=(10, 0)):
        """Create a straight line path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header = path.header
            
            t = i / (self.num_points - 1)
            pose.pose.position.x = start[0] + t * (end[0] - start[0])
            pose.pose.position.y = start[1] + t * (end[1] - start[1])
            pose.pose.position.z = 0.0
            
            # Set orientation along path
            angle = np.arctan2(end[1] - start[1], end[0] - start[0])
            pose.pose.orientation.w = np.cos(angle / 2)
            pose.pose.orientation.z = np.sin(angle / 2)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            
            path.poses.append(pose)
        
        return path
    
    def create_circular_path(self, center=(0, 0), radius=5.0):
        """Create a circular path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header = path.header
            
            angle = 2 * np.pi * i / self.num_points
            pose.pose.position.x = center[0] + radius * np.cos(angle)
            pose.pose.position.y = center[1] + radius * np.sin(angle)
            pose.pose.position.z = 0.0
            
            # Set orientation tangent to circle
            tangent_angle = angle + np.pi / 2
            pose.pose.orientation.w = np.cos(tangent_angle / 2)
            pose.pose.orientation.z = np.sin(tangent_angle / 2)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            
            path.poses.append(pose)
        
        return path
    
    def create_sine_path(self, length=10.0, amplitude=2.0, frequency=1.0):
        """Create a sinusoidal path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header = path.header
            
            x = (i / (self.num_points - 1)) * length
            y = amplitude * np.sin(2 * np.pi * frequency * x / length)
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Calculate tangent for orientation
            if i < self.num_points - 1:
                next_x = ((i + 1) / (self.num_points - 1)) * length
                next_y = amplitude * np.sin(2 * np.pi * frequency * next_x / length)
                angle = np.arctan2(next_y - y, next_x - x)
            else:
                angle = 0.0
            
            pose.pose.orientation.w = np.cos(angle / 2)
            pose.pose.orientation.z = np.sin(angle / 2)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            
            path.poses.append(pose)
        
        return path
    
    def create_figure_eight_path(self, size=3.0):
        """Create a figure-eight path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header = path.header
            
            t = 2 * np.pi * i / self.num_points
            
            # Figure-eight parametric equations
            pose.pose.position.x = size * np.sin(t)
            pose.pose.position.y = size * np.sin(t) * np.cos(t)
            pose.pose.position.z = 0.0
            
            # Calculate tangent for orientation
            dx = size * np.cos(t)
            dy = size * (np.cos(t)**2 - np.sin(t)**2)
            angle = np.arctan2(dy, dx)
            
            pose.pose.orientation.w = np.cos(angle / 2)
            pose.pose.orientation.z = np.sin(angle / 2)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            
            path.poses.append(pose)
        
        return path
    
    def publish_path(self):
        """Publish the selected path type"""
        if self.path_type == 'straight':
            path = self.create_straight_path()
        elif self.path_type == 'circle':
            path = self.create_circular_path()
        elif self.path_type == 'sine':
            path = self.create_sine_path()
        elif self.path_type == 'figure8':
            path = self.create_figure_eight_path()
        else:
            self.get_logger().error(f'Unknown path type: {self.path_type}')
            return
        
        self.path_pub.publish(path)
        self.get_logger().info(f'Published {self.path_type} path with {len(path.poses)} points')


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Generate test paths for AFG controller')
    parser.add_argument('--path-type', choices=['straight', 'circle', 'sine', 'figure8'],
                       default='straight', help='Type of path to generate')
    parser.add_argument('--num-points', type=int, default=20,
                       help='Number of points in the path')
    
    # Parse known args to handle ROS args
    args, unknown = parser.parse_known_args()
    
    rclpy.init(args=unknown)
    
    generator = None
    try:
        generator = TestDataGenerator(args.path_type, args.num_points)
        rclpy.spin(generator)
    except KeyboardInterrupt:
        pass
    finally:
        if generator is not None:
            generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()