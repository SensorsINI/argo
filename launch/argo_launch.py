#!/usr/bin/env python3
"""
ROS2 Launch file for Argo sailboat
Launches all sensor nodes, control system, and Foxglove Bridge for visualization

Launch arguments:
  use_foxglove_bridge:=true/false   Enable/disable Foxglove Bridge (default: true)
  foxglove_port:=<port>            Set Foxglove Bridge port (default: 8765)
  foxglove_address:=<address>      Set bind address (default: 0.0.0.0)

Example usage:
  ros2 launch launch/argo_launch.py                                    # Default (Foxglove enabled)
  ros2 launch launch/argo_launch.py use_foxglove_bridge:=false        # Disable Foxglove
  ros2 launch launch/argo_launch.py foxglove_port:=9090               # Custom port
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
import sys
import shutil

# Recording parameters (from our analysis)
RECORDING_RATE_MB_PER_HOUR = 13.5  # MB per hour
RECORDING_RATE_GB_PER_HOUR = RECORDING_RATE_MB_PER_HOUR / 1024.0

def check_storage_and_warn():
    """Check storage space and display warning if needed"""
    try:
        # Get disk usage for root filesystem
        total, used, free = shutil.disk_usage("/")
        free_gb = free / (1024**3)
        used_percent = (used / total) * 100
        
        # Calculate recording hours
        recording_hours = free_gb / RECORDING_RATE_GB_PER_HOUR
        
        # Warning thresholds
        critical_hours = 2.0  # Less than 2 hours
        warning_hours = 8.0   # Less than 8 hours
        
        print("\n" + "="*60)
        print("🚢 ARGO AUTONOMOUS SAILBOAT - STORAGE STATUS")
        print("="*60)
        
        if recording_hours < critical_hours:
            print("🔴 CRITICAL: SD card will be FULL in {:.1f} hours!".format(recording_hours))
            print("   ⚠️  STOP RECORDING IMMEDIATELY to prevent data loss!")
        elif recording_hours < warning_hours:
            print("🟡 WARNING: SD card will be full in {:.1f} hours".format(recording_hours))
            print("   📝 Consider stopping recording soon")
        else:
            print("🟢 OK: Can record for {:.1f} hours".format(recording_hours))
        
        print("   📊 Free space: {:.1f} GB ({:.1f}% free)".format(free_gb, 100-used_percent))
        print("   📈 Recording rate: {:.1f} MB/hour".format(RECORDING_RATE_MB_PER_HOUR))
        print("   💾 Used: {:.1f} GB ({:.1f}%)".format(used/(1024**3), used_percent))
        print("="*60)
        
        if recording_hours < critical_hours:
            print("⚠️  CRITICAL WARNING: Insufficient storage space!")
            print("   Consider freeing up space or stopping recording.")
            print("="*60)
        
    except Exception as e:
        print("⚠️  Warning: Could not check storage space: {}".format(e))

def generate_launch_description():
    # Check storage space and display warning
    check_storage_and_warn()
    
    # Declare launch arguments
    use_foxglove_bridge_arg = DeclareLaunchArgument(
        'use_foxglove_bridge',
        default_value='true',
        description='Whether to launch foxglove_bridge for visualization'
    )
    
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Port for foxglove_bridge websocket server'
    )
    
    foxglove_address_arg = DeclareLaunchArgument(
        'foxglove_address',
        default_value='0.0.0.0',
        description='Address for foxglove_bridge to bind to (0.0.0.0 = all interfaces)'
    )
    
    # Get script directory (relative to this launch file)
    script_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')
    
    # Define all nodes
    nodes = [
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
        
        # Temperature Monitor node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'temp_monitor.py')],
            name='temp_monitor',
            output='screen'
        ),
    ]
    
    # Conditionally add foxglove_bridge if available
    # Check if foxglove_bridge package exists before launching
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('foxglove_bridge')
        foxglove_bridge_available = True
    except:
        foxglove_bridge_available = False
        print("Warning: foxglove_bridge package not found, skipping foxglove_bridge launch")
        print("Install with: sudo apt install ros-$ROS_DISTRO-foxglove-bridge")
    
    # Only create foxglove_bridge node if package is available
    foxglove_bridge_nodes = []
    if foxglove_bridge_available:
        foxglove_bridge_node = ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('use_foxglove_bridge')),
            cmd=[
                'ros2', 'run', 'foxglove_bridge', 'foxglove_bridge',
                '--ros-args',
                '-p', ['port:=', LaunchConfiguration('foxglove_port')],
                '-p', ['address:=', LaunchConfiguration('foxglove_address')]
            ],
            output='screen',
            shell=True
        )
        foxglove_bridge_nodes.append(foxglove_bridge_node)
        
        # Print connection info when foxglove bridge is enabled
        # Note: This will always print since we can't evaluate LaunchConfiguration at this point
        # The actual launching is controlled by the IfCondition
        print("\n" + "="*60)
        print("🔗 FOXGLOVE BRIDGE AVAILABLE")
        print("="*60)
        print("   To enable: launch with use_foxglove_bridge:=true (default)")
        print("   To disable: launch with use_foxglove_bridge:=false")
        print("   WebSocket URL: ws://<your-ip>:8765 (when enabled)")
        print("   Configure port: foxglove_port:=<port>")
        print("   Configure address: foxglove_address:=<address>")
        print("="*60)
    
    return LaunchDescription([
        use_foxglove_bridge_arg,
        foxglove_port_arg,
        foxglove_address_arg,
        *nodes,
        *foxglove_bridge_nodes,
    ])
