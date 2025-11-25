#!/usr/bin/env python3

"""
Multi-Robot Fleet Launch File
==============================
Launches a complete fleet with 3 different robot types.

This demonstrates:
- Multiple nodes in one launch file
- Using namespaces for multiple instances
- Loading multi-robot configuration from YAML

Usage:
    ros2 launch fleet_manager multi_robot_fleet.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch a complete fleet: 3 robots + 1 monitor.
    """
    
    # ============================================
    # GET CONFIG FILE
    # ============================================
    config_dir = get_package_share_directory('fleet_manager')
    multi_config = os.path.join(config_dir, 'config', 'multi_robot_fleet.yaml')
    
    # ============================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================
    return LaunchDescription([
        
        # ============================================
        # ROBOT ALPHA - Scout
        # ============================================
        Node(
            package='fleet_manager',
            executable='robot_simulator',
            name='robot_simulator',
            namespace='robot_alpha',  # Namespace isolates this robot
            output='screen',
            parameters=[
                multi_config,  # Load config (will use robot_alpha section)
                {'robot_id': 'alpha'}  # Override robot_id parameter
            ]
        ),
        
        # ============================================
        # ROBOT BETA - Transporter
        # ============================================
        Node(
            package='fleet_manager',
            executable='robot_simulator',
            name='robot_simulator',
            namespace='robot_beta',
            output='screen',
            parameters=[
                multi_config,
                {'robot_id': 'beta'}
            ]
        ),
        
        # ============================================
        # ROBOT GAMMA - Cleaner
        # ============================================
        Node(
            package='fleet_manager',
            executable='robot_simulator',
            name='robot_simulator',
            namespace='robot_gamma',
            output='screen',
            parameters=[
                multi_config,
                {'robot_id': 'gamma'}
            ]
        ),
        
        # ============================================
        # FLEET MONITOR
        # ============================================
        Node(
            package='fleet_manager',
            executable='fleet_monitor',
            name='fleet_monitor',
            output='screen',
            parameters=[multi_config]  # Uses fleet_monitor section
        ),
    ])