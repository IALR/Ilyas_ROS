"""
Single Robot Launch File
=========================
Launches one robot simulator and the fleet monitor.

Usage:
    ros2 launch fleet_manager single_robot.launch.py
    ros2 launch fleet_manager single_robot.launch.py robot_id:=my_robot
    ros2 launch fleet_manager single_robot.launch.py robot_type:=transporter
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    This function MUST be named 'generate_launch_description'.
    ROS2 calls this to get the launch configuration.
    """
    
    # ============================================
    # DECLARE LAUNCH ARGUMENTS
    # ============================================
    # These can be overridden from command line:
    # ros2 launch ... robot_id:=custom_name
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_1',
        description='Unique identifier for this robot'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='scout',
        description='Type of robot to simulate',
        choices=['scout', 'transporter', 'cleaner']  # Validation!
    )
    
    # ============================================
    # GET PACKAGE PATHS
    # ============================================
    # Find where our config files are installed
    config_dir = get_package_share_directory('fleet_manager')
    
    # Build config file path based on robot_type argument
    # Results in: /path/to/config/scout_robot.yaml (or transporter, cleaner)
    config_file = [
        config_dir, '/config/', 
        LaunchConfiguration('robot_type'),  # Substituted at runtime!
        '_robot.yaml'
    ]
    
    # ============================================
    # DEFINE ROBOT NODE
    # ============================================
    robot_node = Node(
        package='fleet_manager',
        executable='robot_simulator',
        name='robot_simulator',
        output='screen',  # Show logs in terminal
        parameters=[
            config_file,  # Load YAML config
            {'robot_id': LaunchConfiguration('robot_id')}  # Override robot_id
        ]
    )
    
    # ============================================
    # DEFINE MONITOR NODE
    # ============================================
    monitor_config = os.path.join(config_dir, 'config', 'fleet_monitor.yaml')
    
    monitor_node = Node(
        package='fleet_manager',
        executable='fleet_monitor',
        name='fleet_monitor',
        output='screen',
        parameters=[monitor_config]
    )
    
    # ============================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================
    # Must return LaunchDescription with all args and nodes
    return LaunchDescription([
        robot_id_arg,      # Declare argument
        robot_type_arg,    # Declare argument
        robot_node,        # Start robot
        monitor_node       # Start monitor
    ])