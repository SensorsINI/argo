#!/usr/bin/env python3
"""
Launch file for rosbridge_server to enable Foxglove connection
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen',
            shell=True
        )
    ])
