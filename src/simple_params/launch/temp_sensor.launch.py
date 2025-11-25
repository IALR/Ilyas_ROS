"""
Simple Temperature Sensor Launch File
======================================
Launches a temperature publisher with configurable settings.

Usage:
    ros2 launch simple_params temp_sensor.launch.py
    ros2 launch simple_params temp_sensor.launch.py sensor_type:=indoor
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument
    sensor_type_arg = DeclareLaunchArgument(
        'sensor_type',
        default_value='outdoor',
        description='Type of sensor: outdoor or indoor',
        choices=['outdoor', 'indoor']
    )
    
    # Get package directory
    pkg_dir = get_package_share_directory('simple_params')
    
    # Build config file path based on sensor_type
    config_file = [
        pkg_dir, '/config/',
        LaunchConfiguration('sensor_type'),
        '_sensor.yaml'
    ]
    
    # Define the node
    temp_node = Node(
        package='simple_params',
        executable='temp_publisher',
        name='temp_publisher',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        sensor_type_arg,
        temp_node
    ])