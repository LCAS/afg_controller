#!/usr/bin/env python3
"""
Launch file for testing the AFG Controller

This launch file starts the AFG controller with test-friendly parameters
and can be used for manual testing and debugging.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for AFG controller testing"""
    
    # Declare launch arguments
    desired_speed_arg = DeclareLaunchArgument(
        'desired_speed', default_value='0.5',
        description='Desired forward speed in m/s'
    )
    
    convergence_gain_arg = DeclareLaunchArgument(
        'convergence_gain', default_value='1.5',
        description='Convergence gain for path following'
    )
    
    flow_gain_arg = DeclareLaunchArgument(
        'flow_gain', default_value='2.0',
        description='Flow gain along path'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    
    # AFG Controller node
    afg_controller_node = Node(
        package='afg_controller',
        executable='afg_controller',
        name='afg_controller',
        output='screen',
        parameters=[{
            'desired_speed': LaunchConfiguration('desired_speed'),
            'convergence_gain': LaunchConfiguration('convergence_gain'),
            'flow_gain': LaunchConfiguration('flow_gain'),
            'boundary_layer': 0.1,
            'lookahead_distance': 0.5,
            'max_angular_vel': 1.0,
            'update_rate': 20.0,
            'viz_grid_resolution': 0.5,
            'viz_grid_size': 5.0,
            'viz_arrow_scale': 0.3,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/odom', '/odom'),
            ('/desired_path', '/desired_path'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        desired_speed_arg,
        convergence_gain_arg,
        flow_gain_arg,
        use_sim_time_arg,
        afg_controller_node,
    ])