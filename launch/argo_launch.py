#!/usr/bin/env python3
"""
ROS2 Launch file for Argo sailboat
Launches all sensor nodes and control system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # Declare launch arguments
    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Whether to launch rosbridge_server'
    )
    
    # Get script directory (direct path since we're not using colcon build)
    script_dir = '/home/orangepi/argo/scripts'
    
    # Define all nodes
    nodes = [
        # Anemometer node
        Node(
            package='argo',
            executable=os.path.join(script_dir, 'anem.py'),
            name='anem',
            output='screen',
            parameters=[os.path.join(script_dir, 'argo.yaml')]
        ),
        
        # PWM node
        Node(
            package='argo',
            executable=os.path.join(script_dir, 'pwm.py'),
            name='pwm',
            output='screen',
            parameters=[os.path.join(script_dir, 'argo.yaml')]
        ),
        
        # GPS node
        Node(
            package='argo',
            executable=os.path.join(script_dir, 'gps.py'),
            name='gps',
            output='screen',
            parameters=[os.path.join(script_dir, 'argo.yaml')]
        ),
        
        # IMU node
        Node(
            package='argo',
            executable=os.path.join(script_dir, 'imu.py'),
            name='imu',
            output='screen',
            parameters=[os.path.join(script_dir, 'argo.yaml')]
        ),
        
        # Control node
        Node(
            package='argo',
            executable=os.path.join(script_dir, 'control.py'),
            name='control',
            output='screen',
            parameters=[os.path.join(script_dir, 'argo.yaml')]
        ),
        
        # Battery/Water node
        Node(
            package='argo',
            executable=os.path.join(script_dir, 'battery_water.py'),
            name='battery_water',
            output='screen',
            parameters=[os.path.join(script_dir, 'argo.yaml')]
        ),
    ]
    
    # Conditionally add rosbridge_server if available
    # Note: rosbridge_server might not be installed, so we make this optional
    # Check if rosbridge_server package exists before launching
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('rosbridge_server')
        rosbridge_available = True
    except:
        rosbridge_available = False
        print("Warning: rosbridge_server package not found, skipping rosbridge launch")
    
    # Only create rosbridge node if package is available
    rosbridge_nodes = []
    if rosbridge_available:
        rosbridge_node = ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('use_rosbridge')),
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen',
            shell=True
        )
        rosbridge_nodes.append(rosbridge_node)
    
    return LaunchDescription([
        use_rosbridge_arg,
        *nodes,
        *rosbridge_nodes,
    ])
