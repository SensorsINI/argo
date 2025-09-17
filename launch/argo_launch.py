#!/usr/bin/env python3
"""
DEPRECATED: This file is no longer used. 
Use argo_lifecycle_manager.py instead for direct node launching.

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
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition

# Import centralized node utilities
import sys
import os
# Add the launch directory to Python path so we can import argo_node_utils
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from argo_node_utils import ArgoNodeManager
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
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
    print("🔧 DEBUG: Starting generate_launch_description()")
    
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
    script_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes')
    print(f"🔧 DEBUG: Script directory: {script_dir}")
    print(f"🔧 DEBUG: Script directory exists: {os.path.exists(script_dir)}")
    
    # Define all nodes
    print("🔧 DEBUG: Creating node definitions...")
    
    # Discover node scripts dynamically
    argo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    node_manager = ArgoNodeManager(argo_root)
    discovered_nodes = node_manager.discover_nodes()
    node_scripts = [f"{node}.py" for node in discovered_nodes if node != 'foxglove_bridge']
    
    for script in node_scripts:
        script_path = os.path.join(script_dir, script)
        exists = os.path.exists(script_path)
        print(f"🔧 DEBUG: {script}: {'✓' if exists else '✗'} {script_path}")
    
    # Generate node definitions dynamically
    nodes = []
    for script in node_scripts:
        node_name = script[:-3]  # Remove .py extension
        nodes.append(ExecuteProcess(
            cmd=[os.path.join(script_dir, script)],
            name=node_name,
            output='screen'
        ))
    
    # Conditionally add foxglove_bridge if available
    print("🔧 DEBUG: Checking foxglove_bridge availability...")
    # Check if foxglove_bridge package exists before launching
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('foxglove_bridge')
        foxglove_bridge_available = True
        print("🔧 DEBUG: foxglove_bridge package found ✓")
    except Exception as e:
        foxglove_bridge_available = False
        print(f"🔧 DEBUG: foxglove_bridge package not found ✗: {e}")
        print("Warning: foxglove_bridge package not found, skipping foxglove_bridge launch")
        print("Install with: sudo apt install ros-$ROS_DISTRO-foxglove-bridge")
    
    # Only create foxglove_bridge node if package is available
    foxglove_bridge_nodes = []
    foxglove_bridge_event_handlers = []
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
            shell=True,
            name='foxglove_bridge'
        )
        foxglove_bridge_nodes.append(foxglove_bridge_node)
        
        # Add explicit shutdown handling for foxglove_bridge
        def on_foxglove_shutdown(event, context):
            print("\n🔗 FOXGLOVE BRIDGE: Shutting down gracefully...")
            return []
        
        foxglove_shutdown_handler = RegisterEventHandler(
            OnShutdown(
                on_shutdown=on_foxglove_shutdown
            )
        )
        foxglove_bridge_event_handlers.append(foxglove_shutdown_handler)
        
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
    
    # Debug: Show what we're launching
    print(f"🔧 DEBUG: Total nodes to launch: {len(nodes)}")
    print(f"🔧 DEBUG: Foxglove bridge nodes: {len(foxglove_bridge_nodes)}")
    print(f"🔧 DEBUG: Event handlers: {len(foxglove_bridge_event_handlers)}")
    print("🔧 DEBUG: Creating LaunchDescription...")
    
    launch_desc = LaunchDescription([
        use_foxglove_bridge_arg,
        foxglove_port_arg,
        foxglove_address_arg,
        *nodes,
        *foxglove_bridge_nodes,
        *foxglove_bridge_event_handlers,
    ])
    
    print("🔧 DEBUG: LaunchDescription created successfully ✓")
    return launch_desc

def main():
    """Main function for direct script execution"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Argo Sailboat ROS2 Launch File',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This is a ROS2 launch file for the Argo autonomous sailboat system.

Usage:
  ros2 launch launch/argo_launch.py                    # Default (Foxglove enabled)
  ros2 launch launch/argo_launch.py use_foxglove_bridge:=false  # Disable Foxglove
  ros2 launch launch/argo_launch.py foxglove_port:=9090         # Custom port

Direct execution (for testing):
  python3 launch/argo_launch.py --help                # Show this help
  python3 launch/argo_launch.py --test                # Test launch file generation

Arguments:
  use_foxglove_bridge:=true/false   Enable/disable Foxglove Bridge (default: true)
  foxglove_port:=<port>            Set Foxglove Bridge port (default: 8765)
  foxglove_address:=<address>      Set bind address (default: 0.0.0.0)
        """
    )
    
    parser.add_argument('--test', action='store_true', 
                       help='Test launch file generation without launching')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output')
    
    args = parser.parse_args()
    
    if args.test:
        print("🧪 Testing launch file generation...")
        try:
            launch_desc = generate_launch_description()
            print("✅ Launch file generation successful!")
            print(f"📊 LaunchDescription contains {len(launch_desc.entities)} entities")
            return 0
        except Exception as e:
            print(f"❌ Launch file generation failed: {e}")
            return 1
    else:
        print("ℹ️  This is a ROS2 launch file. Use 'ros2 launch' to run it:")
        print("   ros2 launch launch/argo_launch.py")
        print("   ros2 launch launch/argo_launch.py --help")
        return 0

if __name__ == '__main__':
    sys.exit(main())
