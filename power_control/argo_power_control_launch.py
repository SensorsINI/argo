#!/usr/bin/env python3

"""
Launch file for Argo Power Control ROS2 Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Run in test mode (disable actual shutdown and power control)'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='5.0',
            description='Button press threshold for shutdown in seconds'
        ),
        Node(
            package='argo_power_control',
            executable='argo_power_control.py',
            name='argo_power_control',
            output='screen',
            parameters=[{
                'test_mode': LaunchConfiguration('test_mode'),
                'threshold': LaunchConfiguration('threshold')
            }],
            remappings=[
                ('argo/recording/bagfile_status', 'argo/recording/bagfile_status'),
                ('argo/power_control/led_status', 'argo/power_control/led_status'),
                ('argo/power_control/node_health', 'argo/power_control/node_health'),
                ('argo/power_control/set_led', 'argo/power_control/set_led')
            ]
        )
    ])

