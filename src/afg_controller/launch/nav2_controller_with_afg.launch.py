#!/usr/bin/env python3
"""
Example launch file demonstrating how to launch Nav2 Controller Server 
with the AFG Controller plugin for testing purposes.

This is a minimal example - in production you would use the full Nav2 stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('afg_controller')
    
    # Paths to config files
    default_params_file = os.path.join(pkg_dir, 'config', 'afg_controller_params.yaml')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for controller server'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Controller server node
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Remap topics if needed
            # ('/cmd_vel', '/robot/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        controller_server_node,
    ])
