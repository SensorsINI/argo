#!/usr/bin/env python3
"""
Argo Bag Re-recording with Visualization Markers
==================================================

Re-records an existing bag file with visualization markers added. This allows you to:
1. Process expensive visualization nodes once on a workstation
2. Import the resulting bag directly into Foxglove Studio
3. Get full visualization without running expensive nodes

The script:
- Plays back the original bag file with --clock option to publish simulated time
- Runs visualization nodes (argo_boat_visualization, argo_transform_publisher, sailing_area_publisher)
  - All nodes use use_sim_time:=true to preserve original timestamps
- Records everything (original topics + visualization markers) to a new bag file
- Uses MCAP format by default (configurable via nodes/record.yaml)
- Preserves original timestamps even when playing back at high speed (e.g., 100x)
- Automatically stops recording when playback completes

Note: The --clock option and use_sim_time ensure that re-recorded bags maintain
the original time scale, even when playback is accelerated for faster processing.

Usage:
    ros2 launch launch/argo_bag_rerecord.py input_bag:=/path/to/original/bag output_bag:=output_name
    ros2 launch launch/argo_bag_rerecord.py input_bag:=bags/argo_20251105_141014/ output_bag:=argo_20251105_141014_with_viz
    ros2 launch launch/argo_bag_rerecord.py input_bag:=bags/argo_20251105_141014/ output_bag:=argo_20251105_141014_with_viz use_sailing_area:=false
"""

import os
import yaml
import tempfile
import atexit
import subprocess
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime


def check_mcap_plugin_available():
    """Check if MCAP storage plugin is available for ros2 bag record"""
    try:
        # Try to get help output which lists available storage plugins
        result = subprocess.run(
            ['ros2', 'bag', 'record', '--help'],
            capture_output=True,
            text=True,
            timeout=5
        )
        # Check if 'mcap' appears in the help output as a storage option
        if 'mcap' in result.stdout or 'mcap' in result.stderr:
            return True
        
        # Alternative: Try to check if the plugin package is installed
        # This works by checking if ros2 bag record accepts -s mcap without error
        test_result = subprocess.run(
            ['ros2', 'bag', 'record', '-s', 'mcap', '--help'],
            capture_output=True,
            text=True,
            timeout=5
        )
        # If it doesn't error with "invalid choice", the plugin is available
        if 'invalid choice' not in test_result.stderr.lower():
            return True
        
        return False
    except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
        # If ros2 command not found or times out, assume not available
        return False


def generate_launch_description():
    """Generate launch description for bag re-recording with visualization"""
    
    # Get Argo directory
    argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Get default bags directory
    bags_dir = os.path.join(os.path.expanduser('~'), 'argo', 'bags')
    
    # Load map name from argo_nodes.yaml (same as sailing_area_publisher does)
    map_name = None
    argo_nodes_yaml_path = os.path.join(argo_dir, 'launch', 'argo_nodes.yaml')
    try:
        with open(argo_nodes_yaml_path, 'r') as f:
            config = yaml.safe_load(f)
            simulation_config = config.get('simulation_config', {})
            map_name = simulation_config.get('map_name')
    except (FileNotFoundError, Exception):
        # If config file not found or invalid, map_name will be None
        pass
    
    # Load recording configuration from record.yaml (same as record.py)
    record_config_path = os.path.join(argo_dir, 'nodes', 'record.yaml')
    storage_format = 'mcap'  # Default
    mcap_config = {}
    preset_profile = None
    
    try:
        with open(record_config_path, 'r') as f:
            config = yaml.safe_load(f)
            storage_format = config.get('storage_format', 'mcap')
            if storage_format not in ['mcap', 'sqlite3']:
                storage_format = 'mcap'
            mcap_config = config.get('mcap', {})
            preset_profile = config.get('preset_profile', None)
    except (FileNotFoundError, Exception):
        # Use defaults if config file not found or invalid
        pass
    
    # Check if MCAP plugin is available when MCAP format is requested
    if storage_format == 'mcap':
        if not check_mcap_plugin_available():
            error_msg = """
❌ ERROR: MCAP storage plugin is not installed!

The bag re-recording requires the MCAP storage plugin for ros2 bag record.
Install it with:

    source /opt/ros/humble/setup.bash
    make install-rosbag2-mcap

Or install all ROS2 dependencies:

    source /opt/ros/humble/setup.bash
    make install-deps

After installation, try the re-recording again.
"""
            print(error_msg, file=sys.stderr)
            sys.exit(1)
    
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
    
    storage_format_arg = DeclareLaunchArgument(
        'storage_format',
        default_value=storage_format,
        description='Storage format for output bag: "mcap" or "sqlite3" (defaults to record.yaml config)'
    )
    
    # Build ros2 bag record command with storage format support
    # Start with base command
    # Use --use-sim-time to make ros2 bag record use /clock topic for log time
    # This ensures Foxglove "log time" (when message was recorded) matches original robot time
    # Without --use-sim-time, ros2 bag record uses wall-clock time for log time
    # Note: ros2 bag record will wait for /clock messages before recording starts
    record_base_cmd = 'ros2 bag record -a --include-hidden-topics --use-sim-time -o "$1"'
    
    # Add storage format option based on config
    # For MCAP, we'll create a temporary config file with unique name
    mcap_config_file = None
    if storage_format == 'mcap':
        record_base_cmd += ' -s mcap'
        if preset_profile:
            # Use preset profile (no config file needed)
            record_base_cmd += f' --storage-preset-profile {preset_profile}'
        elif mcap_config:
            # Create temporary config file with unique name (will be cleaned up)
            try:
                # Use tempfile to get a unique filename, but create in bags_dir
                fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix='mcap_config_', dir=bags_dir, text=True)
                mcap_config_file = temp_path
                with os.fdopen(fd, 'w') as f:
                    yaml.dump(mcap_config, f, default_flow_style=False)
                record_base_cmd += f' --storage-config-file {mcap_config_file}'
                # Register cleanup function for this file
                def cleanup_config_file():
                    if os.path.exists(mcap_config_file):
                        try:
                            os.remove(mcap_config_file)
                        except Exception:
                            pass
                atexit.register(cleanup_config_file)
            except Exception:
                # If config file creation fails, continue without it (uses MCAP defaults)
                if mcap_config_file and os.path.exists(mcap_config_file):
                    try:
                        os.remove(mcap_config_file)
                    except Exception:
                        pass
                mcap_config_file = None
    
    # Bag playback process with rate control for maximum speed
    # Use --clock to publish /clock topic for simulated time
    # This preserves original timestamps even when playing at 100x speed
    # Set ERROR log level to reduce verbosity
    bag_play = ExecuteProcess(
        cmd=['bash', '-c', 
             'source /opt/ros/humble/setup.bash && '
             'export RCUTILS_LOGGING_SEVERITY=ERROR && '
             'ros2 bag play "$1" --rate "$2" --clock',
             '--', LaunchConfiguration('input_bag'), LaunchConfiguration('playback_rate')],
        output='screen',
        name='bag_playback'
    )
    
    # Bag recording process - records ALL topics (original + visualization)
    # Start recording after a short delay to let nodes initialize
    # Set WARNING log level to reduce verbosity
    # Uses MCAP format by default (configurable via record.yaml)
    # --use-sim-time makes ros2 bag record use /clock topic for log time
    # This ensures Foxglove "log time" (when message was recorded) matches original robot time
    # Original messages from bag play preserve their header stamps automatically
    # New visualization messages use /clock time for header stamps (via our node subscriptions)
    # This preserves original timestamps even when playing at 100x speed
    bag_record_cmd = f'source /opt/ros/humble/setup.bash && export RCUTILS_LOGGING_SEVERITY=WARNING && {record_base_cmd}'
    
    bag_record = ExecuteProcess(
        cmd=['bash', '-c', bag_record_cmd, '--', LaunchConfiguration('output_bag')],
        cwd=bags_dir,
        output='screen',
        name='bag_recording'
    )
    
    # Create environment dict with ERROR log level for all processes
    # This filters out WARN/INFO/DEBUG messages for minimal output during fast playback
    error_log_env = os.environ.copy()
    error_log_env['RCUTILS_LOGGING_SEVERITY'] = 'ERROR'
    
    # Visualization node - recreates markers from source topics in the bag
    # Use simulated time to preserve original timestamps from playback
    visualization_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_boat_visualization.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_visualization')),
        name='argo_boat_visualization',
        parameters=[{'use_sim_time': True}],
        env=error_log_env
    )
    
    # Transform publisher - provides /tf transforms for 3D visualization
    # Use simulated time to preserve original timestamps from playback
    transform_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_transform_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_transform')),
        name='argo_transform_publisher',
        parameters=[{'use_sim_time': True}],
        env=error_log_env
    )
    
    # Sailing area publisher - provides boundaries/waypoints/hazards
    # Use simulated time to preserve original timestamps from playback
    # Pass map name from config so it uses the correct map origin
    sailing_area_args = [os.path.join(argo_dir, 'nodes', 'sailing_area_publisher.py')]
    if map_name:
        sailing_area_args.extend(['--map', map_name])
    sailing_area_node = Node(
        executable='python3',
        arguments=sailing_area_args,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sailing_area')),
        name='sailing_area_publisher',
        parameters=[{'use_sim_time': True}],
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
    # Also clean up temporary MCAP config file if it exists
    cleanup_config = ''
    if mcap_config_file:
        cleanup_config = f'rm -f "{mcap_config_file}" 2>/dev/null || true && '
    
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
             + cleanup_config +
             'echo "✅ All processes stopped - re-recording complete!"'],
        output='screen',
        name='stop_all_processes'
    )
    
    # Cleanup function for MCAP config file (runs on any shutdown)
    def cleanup_mcap_config(context):
        if mcap_config_file and os.path.exists(mcap_config_file):
            try:
                os.remove(mcap_config_file)
            except Exception:
                pass
    
    # Event handler: Stop all nodes and shutdown when bag playback completes
    stop_all_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                LogInfo(msg='Bag playback completed, stopping all processes...'),
                stop_all_processes,
                TimerAction(
                    period=2.0,
                    actions=[
                        OpaqueFunction(function=cleanup_mcap_config),
                        Shutdown(reason='Bag playback completed')
                    ]
                )
            ]
        )
    )
    
    # Cleanup handler for any shutdown (Ctrl+C, errors, etc.)
    cleanup_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=cleanup_mcap_config)
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
        storage_format_arg,
        LogInfo(msg='=== Argo Bag Re-recording with Visualization ==='),
        LogInfo(msg=['Input bag: ', LaunchConfiguration('input_bag')]),
        LogInfo(msg=['Output bag: ', LaunchConfiguration('output_bag')]),
        LogInfo(msg=['Storage format: ', storage_format, ' (from record.yaml config)']),
        LogInfo(msg=['Playback rate: ', LaunchConfiguration('playback_rate'), 'x (maximum speed)']),
        LogInfo(msg=['Map name: ', map_name if map_name else 'None (will use first GPS fix for origin)']),
        LogInfo(msg='Recording will start automatically 2 seconds after playback begins'),
        LogInfo(msg='All processes will stop automatically when playback completes'),
        visualization_node,
        transform_node,
        sailing_area_node,
        bag_play,
        start_recording_handler,
        stop_all_handler,
        cleanup_on_shutdown,
    ])

