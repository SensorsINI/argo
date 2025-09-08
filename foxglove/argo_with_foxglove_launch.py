#!/usr/bin/env python3
"""
Combined launch file for Argo sailboat with Foxglove support
Launches all sensor nodes, control system, and rosbridge_server
"""

from launch import LaunchDescription
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
        description='Whether to launch rosbridge_server for Foxglove'
    )
    
    # Get script directory
    script_dir = '/home/orangepi/argo/scripts'
    
    # Define all argo nodes
    argo_nodes = [
        # Anemometer node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'anem.py')],
            name='anem',
            output='screen'
        ),
        
        # PWM node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'pwm.py')],
            name='pwm',
            output='screen'
        ),
        
        # GPS node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'gps.py')],
            name='gps',
            output='screen'
        ),
        
        # IMU node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'imu.py')],
            name='imu',
            output='screen'
        ),
        
        # Control node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'control.py')],
            name='control',
            output='screen'
        ),
        
        # Battery/Water node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'battery_water.py')],
            name='battery_water',
            output='screen'
        ),
    ]
    
    # Rosbridge node for Foxglove
    rosbridge_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_rosbridge')),
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        use_rosbridge_arg,
        *argo_nodes,
        rosbridge_node,
    ])
