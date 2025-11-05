#!/usr/bin/env python3
"""
Argo Bag Re-recording with Visualization Markers
==================================================

Re-records an existing bag file with visualization markers added. This allows you to:
1. Process expensive visualization nodes once on a workstation
2. Import the resulting bag directly into Foxglove Studio
3. Get full visualization without running expensive nodes

The script:
- Plays back the original bag file
- Runs visualization nodes (argo_boat_visualization, argo_transform_publisher, sailing_area_publisher)
- Records everything (original topics + visualization markers) to a new bag file
- Automatically stops recording when playback completes

Usage:
    ros2 launch launch/argo_bag_rerecord.py input_bag:=/path/to/original/bag output_bag:=output_name
    ros2 launch launch/argo_bag_rerecord.py input_bag:=bags/argo_20251105_141014/ output_bag:=argo_20251105_141014_with_viz
    ros2 launch launch/argo_bag_rerecord.py input_bag:=bags/argo_20251105_141014/ output_bag:=argo_20251105_141014_with_viz use_sailing_area:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():
    """Generate launch description for bag re-recording with visualization"""
    
    # Get Argo directory
    argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Get default bags directory
    bags_dir = os.path.join(os.path.expanduser('~'), 'argo', 'bags')
    
    # Launch arguments
    input_bag_arg = DeclareLaunchArgument(
        'input_bag',
        description='Path to input bag file to play back',
        default_value=''
    )
    
    output_bag_arg = DeclareLaunchArgument(
        'output_bag',
        description='Name for output bag (without path, will be saved to bags/)',
        default_value=PythonExpression([
            "'argo_rerecord_', str('", datetime.now().strftime('%Y%m%d_%H%M%S'), "')"
        ])
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
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='100.0',
        description='Playback rate multiplier (1.0 = realtime, higher = faster). Maximum rate for fastest processing.'
    )
    
    # Bag playback process with rate control for maximum speed
    # Set ERROR log level to reduce verbosity
    bag_play = ExecuteProcess(
        cmd=['bash', '-c', 
             'source /opt/ros/humble/setup.bash && '
             'export RCUTILS_LOGGING_SEVERITY=ERROR && '
             'ros2 bag play "$1" --rate "$2"',
             '--', LaunchConfiguration('input_bag'), LaunchConfiguration('playback_rate')],
        output='screen',
        name='bag_playback'
    )
    
    # Bag recording process - records ALL topics (original + visualization)
    # Start recording after a short delay to let nodes initialize
    # Set ERROR log level to reduce verbosity
    bag_record = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'export RCUTILS_LOGGING_SEVERITY=ERROR && '
             'ros2 bag record -a --include-hidden-topics -o "$1"',
             '--', LaunchConfiguration('output_bag')],
        cwd=bags_dir,
        output='screen',
        name='bag_recording'
    )
    
    # Create environment dict with ERROR log level for all processes
    # This filters out WARN/INFO/DEBUG messages for minimal output during fast playback
    error_log_env = os.environ.copy()
    error_log_env['RCUTILS_LOGGING_SEVERITY'] = 'ERROR'
    
    # Visualization node - recreates markers from source topics in the bag
    visualization_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_boat_visualization.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_visualization')),
        name='argo_boat_visualization',
        env=error_log_env
    )
    
    # Transform publisher - provides /tf transforms for 3D visualization
    transform_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_transform_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_transform')),
        name='argo_transform_publisher',
        env=error_log_env
    )
    
    # Sailing area publisher - provides boundaries/waypoints/hazards
    sailing_area_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'sailing_area_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sailing_area')),
        name='sailing_area_publisher',
        env=error_log_env
    )
    
    # Event handler: Start recording 2 seconds after bag playback starts
    # This ensures nodes have time to initialize before recording begins
    start_recording_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=bag_play,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[bag_record]
                )
            ]
        )
    )
    
    # Process to stop all nodes when bag playback completes
    stop_all_processes = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'echo "🛑 Stopping recording..." && '
             'pkill -f "ros2 bag record" 2>/dev/null || true && '
             'sleep 0.5 && '
             'echo "🛑 Stopping visualization nodes..." && '
             # Match python3 processes with the specific node scripts
             'pkill -f "python3.*argo_boat_visualization" 2>/dev/null || true && '
             'pkill -f "python3.*argo_transform_publisher" 2>/dev/null || true && '
             'pkill -f "python3.*sailing_area_publisher" 2>/dev/null || true && '
             # Also try matching by node name in case processes are still running
             'pkill -f "argo_boat_visualization" 2>/dev/null || true && '
             'pkill -f "argo_transform_publisher" 2>/dev/null || true && '
             'pkill -f "sailing_area_publisher" 2>/dev/null || true && '
             'sleep 1 && '
             'echo "✅ All processes stopped - re-recording complete!"'],
        output='screen',
        name='stop_all_processes'
    )
    
    # Event handler: Stop all nodes and shutdown when bag playback completes
    stop_all_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                LogInfo(msg='Bag playback completed, stopping all processes...'),
                stop_all_processes,
                TimerAction(
                    period=2.0,
                    actions=[Shutdown(reason='Bag playback completed')]
                )
            ]
        )
    )
    
    return LaunchDescription([
        input_bag_arg,
        output_bag_arg,
        use_sailing_area_arg,
        use_visualization_arg,
        use_transform_arg,
        playback_rate_arg,
        LogInfo(msg='=== Argo Bag Re-recording with Visualization ==='),
        LogInfo(msg=['Input bag: ', LaunchConfiguration('input_bag')]),
        LogInfo(msg=['Output bag: ', LaunchConfiguration('output_bag')]),
        LogInfo(msg=['Playback rate: ', LaunchConfiguration('playback_rate'), 'x (maximum speed)']),
        LogInfo(msg='Recording will start automatically 2 seconds after playback begins'),
        LogInfo(msg='All processes will stop automatically when playback completes'),
        visualization_node,
        transform_node,
        sailing_area_node,
        bag_play,
        start_recording_handler,
        stop_all_handler,
    ])

