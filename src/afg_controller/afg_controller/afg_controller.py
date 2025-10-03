#!/usr/bin/env python3
"""
Artificial Flow Guidance (AFG) Controller for ROS2
Based on vector field guidance for path following

This node implements an AFG controller that generates velocity commands
to guide a vehicle along a desired path using artificial flow fields.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from typing import Optional, List, Tuple


class AFGController(Node):
    """
    Artificial Flow Guidance Controller Node
    
    Subscribes to:
        - /odom (nav_msgs/Odometry): Vehicle odometry
        - /desired_path (nav_msgs/Path): Desired path to follow
    
    Publishes:
        - /cmd_vel (geometry_msgs/Twist): Velocity commands
        - /flow_field_viz (visualization_msgs/MarkerArray): Flow field visualisation
        - /afg_debug (visualization_msgs/MarkerArray): Debug markers (closest point, etc.)
    """
    
    def __init__(self):
        super().__init__('afg_controller')
        
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_speed', 0.5),          # m/s
                ('convergence_gain', 1.5),       # Controls convergence to path
                ('flow_gain', 2.0),              # Controls flow along path
                ('boundary_layer', 0.1),         # m, boundary layer width
                ('lookahead_distance', 0.5),     # m, lookahead for path point
                ('max_angular_vel', 1.0),        # rad/s
                ('update_rate', 20.0),           # Hz
                ('viz_grid_resolution', 0.5),    # m, spacing for flow field grid
                ('viz_grid_size', 5.0),          # m, half-size of visualisation grid
                ('viz_arrow_scale', 0.3),        # Scale for visualisation arrows
            ]
        )
        
        self.desired_speed = self.get_parameter('desired_speed').value
        self.k_conv = self.get_parameter('convergence_gain').value
        self.k_flow = self.get_parameter('flow_gain').value
        self.boundary_layer = self.get_parameter('boundary_layer').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        update_rate = self.get_parameter('update_rate').value
        self.viz_grid_res = self.get_parameter('viz_grid_resolution').value
        self.viz_grid_size = self.get_parameter('viz_grid_size').value
        self.viz_arrow_scale = self.get_parameter('viz_arrow_scale').value
        
        # State variables
        self.current_pose: Optional[PoseStamped] = None
        self.desired_path: Optional[Path] = None
        self.path_points: List[np.ndarray] = []
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flow_field_viz_pub = self.create_publisher(
            MarkerArray, '/flow_field_viz', 10
        )
        self.debug_viz_pub = self.create_publisher(
            MarkerArray, '/afg_debug', 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/desired_path', self.path_callback, 10
        )
        
        # Control timer
        self.timer = self.create_timer(1.0 / update_rate, self.control_loop)
        
        self.get_logger().info('AFG Controller initialized')
        self.get_logger().info(f'Parameters: speed={self.desired_speed}, '
                              f'k_conv={self.k_conv}, k_flow={self.k_flow}')
    
    def odom_callback(self, msg: Odometry):
        """Store current vehicle pose from odometry"""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
    
    def path_callback(self, msg: Path):
        """Store desired path and extract waypoints"""
        self.desired_path = msg
        self.path_points = []
        
        for pose in msg.poses:
            point = np.array([
                pose.pose.position.x,
                pose.pose.position.y
            ])
            self.path_points.append(point)
        
        self.get_logger().info(f'Received path with {len(self.path_points)} points')
    
    def get_closest_point_on_path(self, position: np.ndarray) -> Tuple[np.ndarray, int, float]:
        """
        Find closest point on path and associated path tangent
        
        Returns:
            closest_point: Closest point on path
            segment_idx: Index of path segment
            distance: Cross-track error (distance to path)
        """
        if len(self.path_points) < 2:
            return position, 0, 0.0
        
        min_dist = float('inf')
        closest_point = self.path_points[0]
        segment_idx = 0
        
        # Find closest segment and point on that segment
        for i in range(len(self.path_points) - 1):
            p1 = self.path_points[i]
            p2 = self.path_points[i + 1]
            
            # Project position onto line segment
            segment_vec = p2 - p1
            segment_len = np.linalg.norm(segment_vec)
            
            if segment_len < 1e-6:
                continue
            
            segment_unit = segment_vec / segment_len
            pos_vec = position - p1
            projection = np.dot(pos_vec, segment_unit)
            
            # Clamp projection to segment
            projection = np.clip(projection, 0, segment_len)
            point_on_segment = p1 + projection * segment_unit
            
            dist = np.linalg.norm(position - point_on_segment)
            
            if dist < min_dist:
                min_dist = dist
                closest_point = point_on_segment
                segment_idx = i
        
        return closest_point, segment_idx, min_dist
    
    def get_path_tangent(self, segment_idx: int, closest_point: np.ndarray) -> np.ndarray:
        """
        Get path tangent at closest point using lookahead
        
        Returns:
            tangent: Unit tangent vector along path
        """
        if segment_idx >= len(self.path_points) - 1:
            segment_idx = len(self.path_points) - 2
        
        # Use lookahead point for tangent calculation
        lookahead_point = None
        cumulative_dist = 0.0
        
        for i in range(segment_idx, len(self.path_points) - 1):
            p1 = self.path_points[i]
            p2 = self.path_points[i + 1]
            segment_length = np.linalg.norm(p2 - p1)
            
            if cumulative_dist + segment_length >= self.lookahead_dist:
                # Interpolate along this segment
                remaining = self.lookahead_dist - cumulative_dist
                t = remaining / segment_length if segment_length > 0 else 0
                lookahead_point = p1 + t * (p2 - p1)
                break
            
            cumulative_dist += segment_length
        
        if lookahead_point is None:
            lookahead_point = self.path_points[-1]
        
        # Compute tangent from closest point to lookahead point
        tangent_vec = lookahead_point - closest_point
        tangent_norm = np.linalg.norm(tangent_vec)
        
        if tangent_norm < 1e-6:
            # Fallback to segment tangent
            p1 = self.path_points[segment_idx]
            p2 = self.path_points[segment_idx + 1]
            tangent_vec = p2 - p1
            tangent_norm = np.linalg.norm(tangent_vec)
        
        if tangent_norm < 1e-6:
            return np.array([1.0, 0.0])
        
        return tangent_vec / tangent_norm
    
    def compute_flow_field(self, position: np.ndarray, closest_point: np.ndarray,
                          tangent: np.ndarray, cross_track_error: float) -> np.ndarray:
        """
        Compute artificial flow field vector at current position
        
        The flow field consists of:
        1. Convergence component: Guides vehicle towards path
        2. Flow component: Guides vehicle along path
        
        Returns:
            flow_vector: Desired velocity direction (unit vector)
        """
        # Error vector (perpendicular to path, pointing towards path)
        error_vec = closest_point - position
        
        # Apply boundary layer smoothing near path
        if cross_track_error < self.boundary_layer:
            # Linear interpolation within boundary layer
            alpha = cross_track_error / self.boundary_layer
            convergence_strength = alpha
        else:
            convergence_strength = 1.0
        
        # Convergence component (towards path)
        if cross_track_error > 1e-6:
            convergence_comp = self.k_conv * convergence_strength * (error_vec / cross_track_error)
        else:
            convergence_comp = np.array([0.0, 0.0])
        
        # Flow component (along path)
        flow_comp = self.k_flow * tangent
        
        # Combined flow field
        flow_vector = convergence_comp + flow_comp
        
        # Normalise to get direction
        flow_norm = np.linalg.norm(flow_vector)
        if flow_norm < 1e-6:
            return tangent
        
        return flow_vector / flow_norm
    
    def compute_control_commands(self, flow_direction: np.ndarray,
                                current_heading: float) -> Tuple[float, float]:
        """
        Compute linear and angular velocity commands
        
        Args:
            flow_direction: Desired heading direction (unit vector)
            current_heading: Current vehicle heading (radians)
        
        Returns:
            linear_vel: Linear velocity command
            angular_vel: Angular velocity command
        """
        # Desired heading from flow field
        desired_heading = np.arctan2(flow_direction[1], flow_direction[0])
        
        # Heading error (wrapped to [-pi, pi])
        heading_error = desired_heading - current_heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Angular velocity proportional to heading error
        angular_vel = 2.0 * heading_error
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Reduce linear velocity when turning sharply
        heading_alignment = np.cos(heading_error)
        linear_vel = self.desired_speed * max(0.3, heading_alignment)
        
        return linear_vel, angular_vel
    
    def create_arrow_marker(self, marker_id: int, position: np.ndarray,
                           direction: np.ndarray, scale: float,
                           color: ColorRGBA, frame_id: str = 'map') -> Marker:
        """
        Create an arrow marker for visualisation
        
        Args:
            marker_id: Unique marker ID
            position: Start position [x, y]
            direction: Direction vector [dx, dy]
            scale: Arrow scale
            color: Arrow colour
            frame_id: Reference frame
        
        Returns:
            Marker object
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "flow_field"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Start point
        start = Point()
        start.x = float(position[0])
        start.y = float(position[1])
        start.z = 0.1
        
        # End point
        end = Point()
        end.x = float(position[0] + direction[0] * scale)
        end.y = float(position[1] + direction[1] * scale)
        end.z = 0.1
        
        marker.points = [start, end]
        
        # Arrow appearance
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1   # Head diameter
        marker.scale.z = 0.1   # Head length
        
        marker.color = color
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100000000  # 0.1 seconds
        
        return marker
    
    def publish_flow_field_visualisation(self):
        """
        Publish flow field visualisation as a grid of arrows around vehicle
        """
        if self.current_pose is None or len(self.path_points) < 2:
            return
        
        # Get vehicle position
        pos = self.current_pose.pose.position
        vehicle_pos = np.array([pos.x, pos.y])
        
        marker_array = MarkerArray()
        marker_id = 0
        
        # Create grid of sample points around vehicle
        x_range = np.arange(-self.viz_grid_size, self.viz_grid_size + self.viz_grid_res,
                           self.viz_grid_res)
        y_range = np.arange(-self.viz_grid_size, self.viz_grid_size + self.viz_grid_res,
                           self.viz_grid_res)
        
        for x_offset in x_range:
            for y_offset in y_range:
                sample_pos = vehicle_pos + np.array([x_offset, y_offset])
                
                # Get flow field at this position
                closest_point, segment_idx, cross_track_error = \
                    self.get_closest_point_on_path(sample_pos)
                
                if cross_track_error > self.viz_grid_size * 2:
                    continue  # Skip points too far from path
                
                tangent = self.get_path_tangent(segment_idx, closest_point)
                flow_direction = self.compute_flow_field(
                    sample_pos, closest_point, tangent, cross_track_error
                )
                
                # Colour based on distance to path (green=on path, red=far)
                color = ColorRGBA()
                color.a = 0.7
                
                # Normalised distance for colouring
                norm_dist = min(cross_track_error / 2.0, 1.0)
                color.r = float(norm_dist)
                color.g = float(1.0 - norm_dist)
                color.b = 0.2
                
                # Create arrow marker
                marker = self.create_arrow_marker(
                    marker_id, sample_pos, flow_direction,
                    self.viz_arrow_scale, color,
                    self.current_pose.header.frame_id
                )
                marker_array.markers.append(marker)
                marker_id += 1
        
        self.flow_field_viz_pub.publish(marker_array)
    
    def publish_debug_visualisation(self, closest_point: np.ndarray,
                                   position: np.ndarray,
                                   flow_direction: np.ndarray):
        """
        Publish debug markers showing closest point, vehicle position, and current flow
        """
        marker_array = MarkerArray()
        frame_id = self.current_pose.header.frame_id
        
        # Marker 1: Closest point on path
        closest_marker = Marker()
        closest_marker.header.frame_id = frame_id
        closest_marker.header.stamp = self.get_clock().now().to_msg()
        closest_marker.ns = "debug"
        closest_marker.id = 0
        closest_marker.type = Marker.SPHERE
        closest_marker.action = Marker.ADD
        closest_marker.pose.position.x = float(closest_point[0])
        closest_marker.pose.position.y = float(closest_point[1])
        closest_marker.pose.position.z = 0.1
        closest_marker.scale.x = 0.2
        closest_marker.scale.y = 0.2
        closest_marker.scale.z = 0.2
        closest_marker.color.r = 1.0
        closest_marker.color.g = 0.5
        closest_marker.color.b = 0.0
        closest_marker.color.a = 1.0
        marker_array.markers.append(closest_marker)
        
        # Marker 2: Line from vehicle to closest point
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "debug"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        start_point = Point()
        start_point.x = float(position[0])
        start_point.y = float(position[1])
        start_point.z = 0.1
        
        end_point = Point()
        end_point.x = float(closest_point[0])
        end_point.y = float(closest_point[1])
        end_point.z = 0.1
        
        line_marker.points = [start_point, end_point]
        line_marker.scale.x = 0.05
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8
        marker_array.markers.append(line_marker)
        
        # Marker 3: Current flow direction at vehicle
        flow_color = ColorRGBA()
        flow_color.r = 0.0
        flow_color.g = 1.0
        flow_color.b = 1.0
        flow_color.a = 1.0
        
        flow_marker = self.create_arrow_marker(
            2, position, flow_direction, 0.8, flow_color, frame_id
        )
        flow_marker.ns = "debug"
        flow_marker.scale.x = 0.1
        flow_marker.scale.y = 0.15
        flow_marker.scale.z = 0.15
        marker_array.markers.append(flow_marker)
        
        self.debug_viz_pub.publish(marker_array)
    
    def control_loop(self):
        """
        Main control loop - computes and publishes velocity commands
        """
        # Check if we have required information
        if self.current_pose is None:
            return
        
        if len(self.path_points) < 2:
            # No valid path - stop the vehicle
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Get current position
        pos = self.current_pose.pose.position
        position = np.array([pos.x, pos.y])
        
        # Get current heading from quaternion
        quat = self.current_pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        current_heading = np.arctan2(siny_cosp, cosy_cosp)
        
        # Find closest point on path
        closest_point, segment_idx, cross_track_error = \
            self.get_closest_point_on_path(position)
        
        # Get path tangent
        tangent = self.get_path_tangent(segment_idx, closest_point)
        
        # Compute flow field direction
        flow_direction = self.compute_flow_field(
            position, closest_point, tangent, cross_track_error
        )
        
        # Compute control commands
        linear_vel, angular_vel = self.compute_control_commands(
            flow_direction, current_heading
        )
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        # Publish visualisations
        self.publish_flow_field_visualisation()
        self.publish_debug_visualisation(closest_point, position, flow_direction)
        
        # Log status occasionally
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 100 == 0:
            self.get_logger().info(
                f'Cross-track error: {cross_track_error:.3f}m, '
                f'Linear vel: {linear_vel:.2f}m/s, '
                f'Angular vel: {angular_vel:.2f}rad/s'
            )


def main(args=None):
    """Main entry point for the AFG controller node"""
    rclpy.init(args=args)
    
    try:
        afg_controller = AFGController()
        rclpy.spin(afg_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'afg_controller' in locals():
            afg_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
