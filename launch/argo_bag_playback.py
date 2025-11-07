#!/usr/bin/env python3
"""
Argo Bag Playback with Visualization
=====================================

Launches ROS2 nodes needed for playing back bag files with full visualization
support in Foxglove. This includes:

- Bag file playback (ros2 bag play)
- Visualization node (recreates markers from source topics)
- Transform publisher (provides /tf for 3D visualization)
- Sailing area publisher (optional, for boundaries/waypoints)
- Foxglove bridge (optional, for Foxglove connection)

Usage:
    ros2 launch launch/argo_bag_playback.py bag_file:=/path/to/bag
    ros2 launch launch/argo_bag_playback.py bag_file:=/path/to/bag use_foxglove:=true
    ros2 launch launch/argo_bag_playback.py bag_file:=/path/to/bag use_sailing_area:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for bag playback with visualization"""
    
    # Get Argo directory
    argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Launch arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        description='Path to bag file to play back',
        default_value=''
    )
    
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Launch foxglove_bridge for Foxglove connection'
    )
    
    use_sailing_area_arg = DeclareLaunchArgument(
        'use_sailing_area',
        default_value='true',
        description='Launch sailing_area_publisher for boundaries/waypoints'
    )
    
    use_visualization_arg = DeclareLaunchArgument(
        'use_visualization',
        default_value='true',
        description='Launch visualization node to recreate markers'
    )
    
    use_transform_arg = DeclareLaunchArgument(
        'use_transform',
        default_value='true',
        description='Launch transform publisher for /tf transforms'
    )
    
    # Bag playback process
    # Use bash -c with proper quoting to handle spaces in bag file paths
    bag_play = ExecuteProcess(
        cmd=['bash', '-c', 
             'source /opt/ros/humble/setup.bash && ros2 bag play "$1"',
             '--', LaunchConfiguration('bag_file')],
        output='screen',
        name='bag_playback'
    )
    
    # Visualization node - recreates markers from source topics in the bag
    visualization_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_boat_visualization.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_visualization')),
        name='argo_boat_visualization'
    )
    
    # Transform publisher - provides /tf transforms for 3D visualization
    transform_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_transform_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_transform')),
        name='argo_transform_publisher'
    )
    
    # Sailing area publisher - provides boundaries/waypoints/hazards
    sailing_area_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'sailing_area_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sailing_area')),
        name='sailing_area_publisher'
    )
    
    # Foxglove bridge - for Foxglove Studio connection
    # Uses Node action (proper ROS2 way) instead of ExecuteProcess
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove'))
    )
    
    return LaunchDescription([
        bag_file_arg,
        use_foxglove_arg,
        use_sailing_area_arg,
        use_visualization_arg,
        use_transform_arg,
        LogInfo(msg='=== Argo Bag Playback with Visualization ==='),
        LogInfo(msg=['Playing bag file: ', LaunchConfiguration('bag_file')]),
        LogInfo(msg='Launching visualization support nodes...'),
        bag_play,
        visualization_node,
        transform_node,
        sailing_area_node,
        foxglove_bridge,
        LogInfo(msg='Bag playback started. Connect Foxglove to ws://localhost:8765'),
    ])
