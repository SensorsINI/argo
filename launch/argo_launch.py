#!/usr/bin/env python3
"""
ROS2 Launch file for Argo sailboat
Launches all sensor nodes and control system
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
