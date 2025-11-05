#!/usr/bin/env python3
"""
Argo ROS2 Launch File
=====================

Standard ROS2 launch file for Argo sailboat control system.
Uses Python launch API for node management and monitoring.

This replaces the custom lifecycle manager with standard ROS2 launch system
while maintaining equivalent functionality.

Usage:
    ros2 launch launch/argo_launch.py
    ros2 launch launch/argo_launch.py mode:=simulation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_node_config():
    """Load node configuration from YAML file"""
    argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(argo_dir, 'launch', 'argo_nodes.yaml')
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def generate_launch_description():
    """Generate launch description for Argo ROS2 nodes"""
    
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='normal',
        description='Launch mode: normal or simulation',
        choices=['normal', 'simulation']
    )
    
    # Load configuration
    config = load_node_config()
    argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    argo_yaml_path = os.path.join(argo_dir, 'nodes', 'argo.yaml')
    
    # Determine which nodes to launch based on mode
    def launch_nodes(context):
        mode = context.launch_configurations.get('mode', 'normal')
        launch_actions = []
        
        if mode == 'simulation':
            node_configs = config.get('simulation_nodes', [])
            launch_actions.append(LogInfo(msg="Launching Argo in SIMULATION mode"))
        else:
            node_configs = config.get('nodes', [])
            launch_actions.append(LogInfo(msg="Launching Argo in NORMAL mode"))
        
        # Health monitor runs as independent systemd service (argo_health_monitor.service)
        # It monitors all nodes automatically, so we don't launch it here
        
        for node_cfg in node_configs:
            # Skip excluded nodes (monitored but not launched)
            if node_cfg.get('excluded', False):
                continue
            
            node_name = node_cfg['name']
            executable = node_cfg['executable']
            required = node_cfg.get('required', True)
            critical = node_cfg.get('critical', False)
            description = node_cfg.get('description', '')
            
            # Handle special nodes (like foxglove_bridge)
            if node_cfg.get('special', False):
                if 'ros2 run' in executable:
                    # Extract package and executable from "ros2 run package executable"
                    parts = executable.replace('ros2 run', '').strip().split()
                    if len(parts) >= 2:
                        package = parts[0]
                        exec_name = parts[1]
                        node_action = ExecuteProcess(
                            cmd=['ros2', 'run', package, exec_name],
                            output='screen',
                            name=node_name
                        )
                        launch_actions.append(node_action)
                        launch_actions.append(LogInfo(msg=f"  - {node_name}: {description}"))
                continue
            
            # Build executable path
            if executable.startswith('nodes/'):
                executable_path = os.path.join(argo_dir, executable)
            else:
                executable_path = executable
            
            # Get args and add --no-curses for simulator bridge (required for non-interactive launch)
            node_args = node_cfg.get('args', []).copy()
            
            # Add map name from simulation_config if available (for simulator bridge and sailing_area_publisher)
            if mode == 'simulation':
                simulation_config = config.get('simulation_config', {})
                map_name = simulation_config.get('map_name')
                if map_name:
                    # Add to simulator bridge
                    if 'argo_unified_simulator_bridge' in node_name or 'argo_unified_simulator_bridge.py' in executable:
                        if '--no-curses' not in node_args:
                            node_args.append('--no-curses')
                        if '--map' not in node_args:
                            node_args.extend(['--map', map_name])
                    # Add to sailing_area_publisher so it uses the same coordinate origin
                    elif 'sailing_area_publisher' in node_name or 'sailing_area_publisher.py' in executable:
                        if '--map' not in node_args:
                            node_args.extend(['--map', map_name])
            elif 'argo_unified_simulator_bridge' in node_name or 'argo_unified_simulator_bridge.py' in executable:
                # Add --no-curses even in normal mode if simulator bridge is present
                if '--no-curses' not in node_args:
                    node_args.append('--no-curses')
            
            # Create ROS2 node action using ExecuteProcess for Python scripts
            # Note: For non-package nodes, we use ExecuteProcess instead of Node
            # Add parameter file to command so nodes can load argo.yaml parameters
            cmd = ['python3', executable_path] + node_args + ['--ros-args', '--params-file', argo_yaml_path]
            node_action = ExecuteProcess(
                cmd=cmd,
                output='screen',
                name=node_name,
                respawn=not critical,  # Only respawn non-critical nodes
            )
            
            launch_actions.append(node_action)
            
            # Log node info
            critical_str = " (CRITICAL)" if critical else ""
            launch_actions.append(LogInfo(msg=f"  - {node_name}: {description}{critical_str}"))
        
        return launch_actions
    
    # Create opaque function to generate nodes dynamically
    nodes_action = OpaqueFunction(function=launch_nodes)
    
    return LaunchDescription([
        mode_arg,
        LogInfo(msg="=" * 60),
        LogInfo(msg="Argo ROS2 Launch System"),
        LogInfo(msg="=" * 60),
        nodes_action,
        LogInfo(msg="=" * 60),
        LogInfo(msg="All nodes launched. Monitor with: ros2 node list"),
        LogInfo(msg="Monitor health with: ros2 service call /argo/health/status std_srvs/srv/Trigger"),
        LogInfo(msg="=" * 60),
    ])


if __name__ == '__main__':
    # Allow direct execution for testing
    from launch import LaunchService
    
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()

