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
import sys
import yaml
import tempfile
import atexit
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime


def validate_and_print_paths(context):
    """Validate paths and print output bag location"""
    try:
        # Get launch configuration values
        input_bag = context.launch_configurations.get('input_bag', '')
        output_bag = context.launch_configurations.get('output_bag', '')
        
        # Get Argo directory
        argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        bags_dir = os.path.join(argo_dir, 'bags')
        
        # Validate input bag
        if not input_bag:
            error_msg = "❌ Error: input_bag argument is required"
            print(error_msg, file=sys.stderr)
            raise ValueError(error_msg)
        
        # Resolve input bag path
        if not os.path.isabs(input_bag):
            # Try relative to bags directory first
            if os.path.exists(os.path.join(bags_dir, input_bag)):
                input_bag = os.path.join(bags_dir, input_bag)
            elif os.path.exists(os.path.join(argo_dir, input_bag)):
                input_bag = os.path.join(argo_dir, input_bag)
            else:
                input_bag = os.path.abspath(input_bag)
        
        if not os.path.exists(input_bag):
            error_msg = f"❌ Error: Input bag not found: {input_bag}"
            print(error_msg, file=sys.stderr)
            raise FileNotFoundError(error_msg)
        
        if not os.path.isdir(input_bag):
            error_msg = f"❌ Error: Input bag path is not a directory: {input_bag}"
            print(error_msg, file=sys.stderr)
            raise ValueError(error_msg)
        
        # Validate bags directory
        if not os.path.exists(bags_dir):
            error_msg = f"❌ Error: Bags directory does not exist: {bags_dir}"
            print(error_msg, file=sys.stderr)
            raise FileNotFoundError(error_msg)
        
        # Calculate full output bag path
        output_bag_full_path = os.path.join(bags_dir, output_bag)
        
        # Print output bag location
        print(f"📦 Creating output bag at: {output_bag_full_path}", file=sys.stdout)
        print(f"   Input bag:  {input_bag}", file=sys.stdout)
        print(f"   Output bag: {output_bag_full_path}", file=sys.stdout)
        
        # Check if output bag already exists
        if os.path.exists(output_bag_full_path):
            print(f"⚠️  Warning: Output bag already exists and will be overwritten: {output_bag_full_path}", file=sys.stderr)
        
        return []
    except Exception as e:
        error_msg = f"❌ Error in path validation: {str(e)}"
        print(error_msg, file=sys.stderr)
        import traceback
        print(traceback.format_exc(), file=sys.stderr)
        raise


def generate_launch_description():
    """Generate launch description for bag re-recording with visualization"""
    
    try:
        # Get Argo directory
        argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # Get bags directory (use project's bags/ directory, same as bash script)
        bags_dir = os.path.join(argo_dir, 'bags')
        
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
        error_log_env['RCUTILS_LOGGING_SEVERITY'] = 'WARNING'
        
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
        sailing_area_node = Node(
            executable='python3',
            arguments=[os.path.join(argo_dir, 'nodes', 'sailing_area_publisher.py')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sailing_area')),
            name='sailing_area_publisher',
            parameters=[{'use_sim_time': True}],
            env=error_log_env
        )
        
        # Event handler: Start recording 0.5 seconds after bag playback starts
        # Reduced delay for short bags - nodes initialize quickly and we need to capture data ASAP
        # For very short bags playing at 100x speed, even 2 seconds can be too long
        start_recording_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=bag_play,
                on_start=[
                    TimerAction(
                        period=0.5,
                        actions=[bag_record]
                    )
                ]
            )
        )
        
        # Process to stop all nodes when bag playback completes
        # Also clean up temporary MCAP config file if it exists
        # IMPORTANT: Wait for output bag directory to be created before stopping
        # This ensures recording has actually started and captured data
        cleanup_config = ''
        if mcap_config_file:
            cleanup_config = f'rm -f "{mcap_config_file}" 2>/dev/null || true && '
        
        # Function to create stop process with output_bag from context
        def create_stop_process(context):
            output_bag = context.launch_configurations.get('output_bag', '')
            output_path = os.path.join(bags_dir, output_bag)
            
            stop_cmd = (
                'source /opt/ros/humble/setup.bash && '
                f'OUTPUT_PATH="{output_path}" && '
                'echo "🛑 Bag playback completed, waiting for recording to finish..." && '
                # Wait for output bag directory to be created (max 10 seconds)
                # This ensures recording has actually started
                'WAIT_COUNT=0 && '
                'while [ ! -d "$OUTPUT_PATH" ] && [ $WAIT_COUNT -lt 20 ]; do '
                '  sleep 0.5 && '
                '  WAIT_COUNT=$((WAIT_COUNT + 1)) && '
                '  if [ $((WAIT_COUNT % 4)) -eq 0 ]; then '
                '    echo "   Waiting for recording to start... ($((WAIT_COUNT * 50))ms)" '
                '  fi '
                'done && '
                # If output bag still doesn't exist, wait a bit more and check again
                'if [ ! -d "$OUTPUT_PATH" ]; then '
                '  echo "⚠️  Warning: Output bag directory not found, waiting additional 2 seconds..." && '
                '  sleep 2 && '
                'fi && '
                # Now stop recording gracefully
                'echo "🛑 Stopping recording (allowing time for final writes)..." && '
                # Send SIGTERM to allow graceful shutdown
                'pkill -TERM -f "ros2 bag record" 2>/dev/null || true && '
                # Wait longer for bag to finish writing (MCAP format may need time to flush)
                'sleep 3 && '
                # Force kill if still running
                'pkill -KILL -f "ros2 bag record" 2>/dev/null || true && '
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
                'echo "✅ All processes stopped - re-recording complete!"'
            )
            
            return [ExecuteProcess(
                cmd=['bash', '-c', stop_cmd],
                output='screen',
                name='stop_all_processes'
            )]
        
        # Cleanup function for MCAP config file (runs on any shutdown)
        def cleanup_mcap_config(context):
            if mcap_config_file and os.path.exists(mcap_config_file):
                try:
                    os.remove(mcap_config_file)
                except Exception:
                    pass
        
        # Event handler: Stop all nodes and shutdown when bag playback completes
        # Note: stop_all_processes will wait for recording to finish, so we give it more time
        stop_all_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play,
                on_exit=[
                    LogInfo(msg='Bag playback completed, waiting for recording to finish...'),
                    OpaqueFunction(function=create_stop_process),
                    TimerAction(
                        period=15.0,  # Increased timeout to allow for recording wait + shutdown
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
            # Validate paths and print output bag location (runs early)
            OpaqueFunction(function=validate_and_print_paths),
            LogInfo(msg='=== Argo Bag Re-recording with Visualization ==='),
            LogInfo(msg=['Input bag: ', LaunchConfiguration('input_bag')]),
            LogInfo(msg=['Output bag: ', LaunchConfiguration('output_bag')]),
            LogInfo(msg=['Storage format: ', storage_format, ' (from record.yaml config)']),
            LogInfo(msg=['Playback rate: ', LaunchConfiguration('playback_rate'), 'x (maximum speed)']),
            LogInfo(msg='Recording will start automatically 0.5 seconds after playback begins'),
            LogInfo(msg='All processes will stop automatically when playback completes'),
            visualization_node,
            transform_node,
            sailing_area_node,
            bag_play,
            start_recording_handler,
            stop_all_handler,
            cleanup_on_shutdown,
        ])
    except Exception as e:
        error_msg = f"❌ Error generating launch description: {str(e)}"
        print(error_msg, file=sys.stderr)
        import traceback
        print(traceback.format_exc(), file=sys.stderr)
        # Re-raise to let ros2 launch handle it (will result in nonzero exit)
        raise

