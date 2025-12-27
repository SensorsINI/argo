#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
Argo Status and Simulation Manager
==================================

A tool for checking system status and managing simulations for the Argo sailboat.
This script provides detailed diagnostics and is the backend for the 'as' and 'asq' aliases.

It uses argo_nodes.yaml as the single source of truth for node configuration.

Node lifecycle (start/stop/restart) is now managed by the systemd service
'argo_launch_standard.service' and the aliases 'al', 'aq', 'ars'.

Commands:
    status           (Default) Show comprehensive system status with diagnostics.
    quick_status     Show condensed one-line status (fast check).
    simulate_local   Start local simulation mode.
    simulate_remote  Start remote simulation mode.
    help             Show this help message.

Options:
    --toggle_pause   Toggle controller pause state (pauses autonomous navigation).
    --debug          Enable debug output for troubleshooting.
    --quiet          Suppress initialization messages.

Examples:
    # Show detailed status (default command)
    python3 argo_lifecycle_manager.py
    argo_lifecycle_manager.py status

    # Show quick, one-line status
    argo_lifecycle_manager.py quick_status

    # Start local simulation
    argo_lifecycle_manager.py simulate_local
"""

import os
import sys
import time
import signal
import subprocess
import threading
import argparse
import argcomplete
import json
import shlex
import importlib.util
import atexit
from datetime import datetime
from typing import Dict, List, Optional, Any
import select
import yaml  # Add YAML import

# Import centralized node utilities
from argo_node_utils import ArgoNodeManager
import psutil

# Color codes for terminal output
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DARK_GREEN = '\033[32m'  # Dark green for healthy nodes
    RED = '\033[31m'         # Red for unhealthy nodes
    DIM = '\033[2m'          # Dim/Gray for stopped nodes

# ROS2 imports for service client
try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    from std_msgs.msg import Bool, String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Add config loading for remote simulation
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(SCRIPT_DIR, '..', 'scripts'))
    from load_config import load_config
    REMOTE_CONFIG = load_config()
except ImportError:
    REMOTE_CONFIG = None

# MP2672 CHG timer warning threshold (hours)
# Warn users when remaining charging time drops below this threshold
CHARGING_TIME_WARNING_THRESHOLD_HOURS = 5.0


class ArgoLifecycleManager:
    def __init__(self,
                 quiet: bool = True,
                 debug_nodes: bool = False,
                 debug_node_port_base: int = 5678,
                 debug_node_wait: bool = False):
        # Become session leader to manage child processes effectively
        try:
            os.setsid()
        except OSError as e:
            # This will fail if we are already a session leader, which is fine
            pass
        
        self.argo_dir = os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))
        self.process = None
        self.node_processes = []
        self.quiet = quiet  # Suppress initialization messages
        self.debug_nodes = debug_nodes
        self.debug_node_port_base = debug_node_port_base
        self.debug_node_wait = debug_node_wait
        self.debug_node_listen_host = os.environ.get('ARGO_DEBUG_NODE_HOST', '127.0.0.1')
        self._next_debug_port = debug_node_port_base
        # Removed restart logic - nodes should not be restarted automatically
        # Failures should be preserved for debugging
        self.stabilization_wait = 15.0  # Additional wait time for nodes to stabilize
        self.journal_since = 'today'
        self.remote_simulator_proc = None
        self.remote_tunnel_proc = None
        self.shutdown_requested = False  # Flag to coordinate shutdown
        self.pgid = os.getpgrp() # Get our own process group ID

        if self.debug_nodes:
            if importlib.util.find_spec("debugpy") is None:
                print("⚠️  debugpy is not installed; launching nodes without debugger.")
                self.debug_nodes = False
            else:
                print(f"🐞 Node debugging enabled (host={self.debug_node_listen_host}, start_port={self.debug_node_port_base})")

        # Initialize ROS2 for service client if available
        self.ros2_node = None
        self.battery_service_client = None
        self.controller_pause_client = None
        self.controller_pause_state = False
        self.status_publisher = None
        self.lifecycle_services_created = False
        
        # Health status monitoring
        self.node_health_status = {}  # Track health status for each node
        self.health_subscribers = {}  # ROS2 subscribers for health topics
        
        # --- NEW: Load all node configuration from argo_nodes.yaml ---
        self._load_nodes_from_yaml()
        
        # ROS2 node will be created lazily when needed
        # This prevents creating a node for simple commands that don't need ROS2
        
        # Initialize node manager, but only for process management, not discovery
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # ROS2 node initialization deferred until actually needed
    
    def _ensure_ros2_node(self):
        """Lazily create ROS2 node and clients if not already created"""
        if self.ros2_node is not None:
            return  # Already initialized
        
        if not ROS2_AVAILABLE:
            return
        
        try:
            if not rclpy.ok():
                rclpy.init()
            self.ros2_node = Node('argo_lifecycle_manager')
            
            # Declare a dummy parameter to prevent foxglove_bridge errors when querying parameters
            # (foxglove_bridge tries to discover parameters from all nodes)
            self.ros2_node.declare_parameter('node_type', 'lifecycle_manager')
            
            self.battery_service_client = self.ros2_node.create_client(
                Trigger, '/battery_status')

            # Create controller pause service client
            self.controller_pause_client = self.ros2_node.create_client(
                SetBool, '/controller_node/pause')
            
            # Subscribe to controller pause state
            self.controller_pause_sub = self.ros2_node.create_subscription(
                Bool, '/controller_pause_state', self._controller_pause_state_callback, 10)
            
            # Initialize health status monitoring (uses self.all_expected_nodes from YAML)
            self._setup_health_monitoring()
        except Exception as e:
            if not self.quiet:
                print(f"Warning: Could not initialize ROS2 service client: {e}")
            self.ros2_node = None
            self.battery_service_client = None
            self.controller_pause_client = None
        
        # Query initial controller pause state after a brief delay to allow ROS2 to initialize
        if self.controller_pause_client:
            import threading
            def query_initial_pause_state():
                time.sleep(2.0)  # Wait for ROS2 to initialize
                self._query_controller_pause_state()
            threading.Thread(target=query_initial_pause_state, daemon=True).start()
        
        
        # Setup signal handlers only from main thread (signal handlers can only be set from main thread)
        # Use threading.main_thread() to check if we're in the main thread
        if threading.current_thread() is threading.main_thread():
            try:
                signal.signal(signal.SIGINT, self._signal_handler)
                signal.signal(signal.SIGTERM, self._signal_handler)
            except ValueError:
                # Signal handlers can only be set from main thread - ignore if called from background thread
                pass
    
    def _load_nodes_from_yaml(self):
        """Load and parse node definitions from argo_nodes.yaml."""
        # print("ℹ️  Loading node configuration from argo_nodes.yaml...")
        config_path = os.path.join(self.argo_dir, 'launch', 'argo_nodes.yaml')
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except (IOError, yaml.YAMLError) as e:
            print(f"❌ CRITICAL: Could not load or parse argo_nodes.yaml: {e}")
            # Exit if the config is essential and cannot be loaded
            sys.exit(1)

        self.expected_nodes = []
        self.special_nodes = []
        self.excluded_nodes = []
        self.critical_nodes = []
        self.all_expected_nodes = []
        
        # Simulation node configuration (loaded but not used until simulation mode)
        self.simulation_expected_nodes = []
        self.simulation_special_nodes = []
        self.simulation_critical_nodes = []
        self.simulation_node_args = {}  # Map of script_name -> args list
        self.simulation_map_name = None  # Map name for simulation start location

        # --- NEW: Physical Robot node configuration ---
        self.physical_robot_nodes = []
        self.physical_robot_special_nodes = []
        self.physical_robot_node_args = {}

        all_node_configs = config.get('nodes', [])
        
        # Create a lookup map for node configs by name for easy reference
        node_config_map = {node['name']: node for node in all_node_configs if 'name' in node}

        # --- Process node groups ---
        groups = config.get('groups', {})
        physical_robot_group = groups.get('physical_robot', [])

        for node_name in physical_robot_group:
            if node_name not in node_config_map:
                print(f"⚠️  Warning: Node '{node_name}' from 'physical_robot' group not found in main node list.")
                continue

            node_cfg = node_config_map[node_name]
            script_name = os.path.basename(node_cfg.get('executable', ''))

            if node_cfg.get('special', False):
                # For special nodes, the "script_name" is just the node name
                self.physical_robot_special_nodes.append(node_name)
            else:
                self.physical_robot_nodes.append(script_name)

            if node_cfg.get('args'):
                self.physical_robot_node_args[script_name] = node_cfg.get('args', [])

        for node_cfg in all_node_configs:
            name = node_cfg.get('name')
            if not name:
                continue

            # Convert to the script name format used by the manager (e.g., 'anem.py' or 'foxglove_bridge')
            script_name = name
            if not node_cfg.get('special', False):
                 # Assuming non-special nodes follow the pattern 'node_name.py'
                 # We need to find the executable to get the script name
                 executable = node_cfg.get('executable', '')
                 script_name = os.path.basename(executable) if executable else f"{name}.py"

            if node_cfg.get('excluded', False):
                self.excluded_nodes.append(script_name)
            elif node_cfg.get('special', False):
                self.special_nodes.append(script_name)
            else:
                self.expected_nodes.append(script_name)
            
            if node_cfg.get('critical', False):
                self.critical_nodes.append(script_name)

        # all_expected_nodes for monitoring should include all non-excluded nodes
        self.all_expected_nodes = self.expected_nodes + self.special_nodes
        
        # Load simulation configuration (map name, etc.)
        simulation_config = config.get('simulation_config', {})
        self.simulation_map_name = simulation_config.get('map_name')
        
        # Load simulation nodes from 'simulation' group (same pattern as physical_robot)
        simulation_group = groups.get('simulation', [])
        
        for node_name in simulation_group:
            if node_name not in node_config_map:
                # Also check simulation_nodes list for nodes that might not be in main nodes list
                # (like argo_unified_simulator_bridge which is simulation-only)
                simulation_node_configs = config.get('simulation_nodes', [])
                node_cfg = None
                for sim_cfg in simulation_node_configs:
                    if sim_cfg.get('name') == node_name:
                        node_cfg = sim_cfg
                        break
                
                if not node_cfg:
                    print(f"⚠️  Warning: Node '{node_name}' from 'simulation' group not found in node lists.")
                    continue
            else:
                node_cfg = node_config_map[node_name]
            
            script_name = os.path.basename(node_cfg.get('executable', ''))
            
            if node_cfg.get('special', False):
                # For special nodes, the "script_name" is just the node name
                self.simulation_special_nodes.append(node_name)
            else:
                self.simulation_expected_nodes.append(script_name)
            
            if node_cfg.get('args'):
                self.simulation_node_args[script_name] = node_cfg.get('args', [])
            
            if node_cfg.get('critical', False):
                self.simulation_critical_nodes.append(script_name)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        if self.shutdown_requested:
            return  # Already shutting down
        
        self.shutdown_requested = True
        print(
            f"\n🛑 argo_lifecycle_manager: Received signal {signum}, shutting down...")
        
        # Don't call stop() here - let the main loop handle it
        # Just set the flag and let continuous() exit cleanly
    
    def _cleanup_ros2(self):
        """Clean up ROS2 resources - only call this once at final shutdown"""
        if self.ros2_node:
            try:
                # Destroy node first
                self.ros2_node.destroy_node()
            except Exception as e:
                # Silently fail - context may already be shutdown
                pass
            self.ros2_node = None
        
        # Only shutdown context if it's still valid
        if ROS2_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                # Silently fail - may already be shutdown
                pass

    def _controller_pause_state_callback(self, msg):
        """Receive controller pause state updates."""
        self.controller_pause_state = msg.data

    def _setup_health_monitoring(self):
        """Setup health status monitoring for nodes that publish health topics"""
        self._ensure_ros2_node()
        if not self.ros2_node:
            return
            
        # Define health services for each node (new ArgoBaseNode approach)
        health_services = {
            'gps.py': '/gps_node/health',
            'lora.py': '/lora_node/health', 
            'anem.py': '/anem_node/health',
            'argo_battery_water.py': '/battery_water_node/health',
            'rudder_sail_radio.py': '/rudder_sail_radio_node/health',
            'temp_monitor.py': '/temp_monitor_node/health',
            'argo_transform_publisher.py': '/argo_transform_publisher/health',
            'argo_boat_visualization.py': '/argo_boat_visualization/health',
            'bno085.py': '/bno085_bridge/health'  # BNO085 bridge health service
        }
        
        # Also keep health topics for backward compatibility
        health_topics = {
            'gps.py': '/gps_node_health',
            'lora.py': '/lora_node_health',
            'anem.py': '/anem_node_health',
            'argo_battery_water.py': '/battery_water_node_health',
            'rudder_sail_radio.py': '/rudder_sail_radio_node_health',
            'temp_monitor.py': '/temp_monitor_node_health',
            'bno085.py': '/imu_health'  # BNO085 IMU health status
        }
        
        # Create subscribers for health topics
        for node_name, health_topic in health_topics.items():
            try:
                # Create callback function for this specific node
                def create_health_callback(node):
                    def health_callback(msg):
                        self.node_health_status[node] = msg.data
                    return health_callback
                
                callback = create_health_callback(node_name)
                subscriber = self.ros2_node.create_subscription(
                    Bool, health_topic, callback, 10)
                self.health_subscribers[node_name] = subscriber
                
                # Initialize health status as unknown
                self.node_health_status[node_name] = None
                
            except Exception as e:
                if not self.quiet:
                    print(f"Warning: Could not subscribe to health topic {health_topic}: {e}")

    def _query_node_health_services(self) -> Dict[str, Dict]:
        """Query health status from all nodes via services
        
        Fast path: Try aggregated health monitor service first (1 service call)
        Fallback: Query individual node health services (10+ service calls)
        """
        self._ensure_ros2_node()
        health_data = {}
        
        # Check if ROS2 node is available
        if not self.ros2_node:
            return health_data
        
        # FAST PATH: Try to use health monitor aggregated service
        try:
            health_monitor_client = self.ros2_node.create_client(Trigger, '/argo/health/status')
            if health_monitor_client.wait_for_service(timeout_sec=2.0):
                request = Trigger.Request()
                future = health_monitor_client.call_async(request)
                rclpy.spin_until_future_complete(
                    self.ros2_node, future, timeout_sec=2.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        try:
                            monitor_data = json.loads(response.message)
                            # Convert health monitor format to lifecycle manager format
                            # Health monitor uses YAML node names (which are .py filenames for regular nodes)
                            # or node names for special nodes (e.g., 'foxglove_bridge')
                            if 'nodes' in monitor_data:
                                for node_name, node_info in monitor_data['nodes'].items():
                                    # Handle both old format (executable string for special nodes) and new format (node name)
                                    # Old format: 'ros2 run foxglove_bridge foxglove_bridge'
                                    # New format: 'foxglove_bridge'
                                    lookup_key = node_name
                                    
                                    # If this looks like an old format special node key (contains 'ros2 run'), try to extract node name
                                    if 'ros2 run' in node_name:
                                        # Load YAML config to find the matching node
                                        try:
                                            config_path = os.path.join(self.argo_dir, 'launch', 'argo_nodes.yaml')
                                            with open(config_path, 'r') as f:
                                                config = yaml.safe_load(f)
                                            
                                            # Check both regular and simulation nodes
                                            for cfg in (config.get('nodes', []) + config.get('simulation_nodes', [])):
                                                if cfg.get('executable', '') == node_name and cfg.get('special', False):
                                                    lookup_key = cfg.get('name', node_name)
                                                    break
                                        except Exception:
                                            # If we can't load config, keep original key
                                            pass
                                    
                                    health_data[lookup_key] = {
                                        'healthy': node_info.get('healthy'),
                                        'last_seen': node_info.get('last_seen', 0.0)
                                    }
                                    
                                    # Also keep the original key for backward compatibility during transition
                                    if lookup_key != node_name:
                                        health_data[node_name] = health_data[lookup_key]
                            # If we got data from health monitor, return it (fast path succeeded)
                            if health_data:
                                return health_data
                        except json.JSONDecodeError:
                            # JSON parse error, fall through to individual calls
                            pass
        except Exception as e:
            print(f"🔍 Error in health monitor service call: {e}")
            # Health monitor unavailable or error, fall through to individual calls
            print(f"Falling through to individual calls: {e}")
            pass
        
        # SLOW PATH: Query individual node health services (fallback)        
        # Define health services for each node
        health_services = {
            'gps.py': '/gps_node/health',
            'lora.py': '/lora_node/health', 
            'anem.py': '/anem_node/health',
            'argo_battery_water.py': '/battery_water_node/health',
            'rudder_sail_radio.py': '/rudder_sail_radio_node/health',
            'temp_monitor.py': '/temp_monitor_node/health',
            'controller.py': '/controller_node/health',
            'argo_transform_publisher.py': '/argo_transform_publisher/health',
            'argo_boat_visualization.py': '/argo_boat_visualization/health',
            'record.py': '/record/health',
            'bno085.py': '/bno085_bridge/health'  # Monitor BNO085 health even when excluded from launch
        }
        
        for node_name, service_name in health_services.items():
            try:
                # Create temporary service client
                client = self.ros2_node.create_client(Trigger, service_name)
                
                # Wait for service with short timeout
                if client.wait_for_service(timeout_sec=1.0):
                    request = Trigger.Request()
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(
                        self.ros2_node, future, timeout_sec=2.0)
                    
                    if future.done():
                        response = future.result()
                        if response.success:
                            try:
                                health_info = json.loads(response.message)
                                health_data[node_name] = health_info
                            except json.JSONDecodeError as e:
                                # Handle non-JSON responses (like BNO085 before fix)
                                health_data[node_name] = {'healthy': None, 'error': f'JSON parse error: {e}'}
                        else:
                            health_data[node_name] = {'healthy': False, 'error': response.message}
                    else:
                        health_data[node_name] = {'healthy': None, 'error': 'timeout'}
                else:
                    health_data[node_name] = {'healthy': None, 'error': 'service unavailable'}
                    
            except Exception as e:
                health_data[node_name] = {'healthy': None, 'error': str(e)}
        
        return health_data

    def _get_node_health_status(self, node_name: str, health_data: Dict = None) -> str:
        """Get health status string for a node"""
        # Try service data first if provided
        if health_data and node_name in health_data:
            node_health = health_data[node_name]
            if 'healthy' in node_health:
                if node_health['healthy'] is None:
                    return "❓"
                elif node_health['healthy']:
                    return "🟢"
                else:
                    return "🔴"
        
        # Try with .py extension if not found
        if health_data and f"{node_name}.py" in health_data:
            node_health = health_data[f"{node_name}.py"]
            if 'healthy' in node_health:
                if node_health['healthy'] is None:
                    return "❓"
                elif node_health['healthy']:
                    return "🟢"
                else:
                    return "🔴"
        
        # Fallback to topic data
        if node_name not in self.node_health_status:
            return "❓"
        
        health_status = self.node_health_status[node_name]
        if health_status is None:
            return "❓"
        elif health_status:
            return "🟢"
        else:
            return "🔴"

    def _query_controller_pause_state(self):
        """Query the current controller pause state via service call."""
        self._ensure_ros2_node()
        try:
            if not self.controller_pause_client:
                return
            
            # Wait for service to be available
            if not self.controller_pause_client.wait_for_service(timeout_sec=3.0):
                return
            
            # Create request with None to get current state
            request = SetBool.Request()
            request.data = None  # None means return current state
            
            # Call service
            future = self.controller_pause_client.call_async(request)
            rclpy.spin_until_future_complete(self.ros2_node, future, timeout_sec=3.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    # Parse the message to determine current state
                    message = response.message.lower()
                    self.controller_pause_state = 'paused' in message
                else:
                    # Default to unpaused if we can't determine state
                    self.controller_pause_state = False
                    
        except Exception as e:
            # Default to unpaused if query fails
            self.controller_pause_state = False

    def _handle_toggle_pause(self, request, response):
        """Handle toggle_pause service requests - now delegates to controller pause service."""
        try:
            # Check if controller is running
            node_status = self._get_node_status()
            if 'controller.py' not in node_status or "STOPPED" in node_status['controller.py']:
                response.success = False
                response.message = "Controller node is not running"
                return response

            # Toggle controller pause state
            new_pause_state = not self.controller_pause_state
            success, message = self._call_controller_pause_service(new_pause_state)
            
            if success:
                response.success = True
                response.message = f"Controller {'paused' if new_pause_state else 'unpaused'} successfully"
            else:
                response.success = False
                response.message = f"Failed to toggle controller pause: {message}"

        except Exception as e:
            print(f"❌ Error in toggle_pause handler: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def _call_controller_pause_service(self, pause_state: bool) -> tuple[bool, str]:
        """Call the controller pause service with the specified state."""
        try:
            if not self.controller_pause_client:
                return False, "Controller pause service client not available"

            # Wait for service to be available
            if not self.controller_pause_client.wait_for_service(timeout_sec=2.0):
                return False, "Controller pause service not available"

            # Create request
            request = SetBool.Request()
            request.data = pause_state

            # Call service
            future = self.controller_pause_client.call_async(request)
            rclpy.spin_until_future_complete(self.ros2_node, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                return response.success, response.message
            else:
                return False, "Service call timed out"

        except Exception as e:
            return False, f"Error calling controller pause service: {e}"

    # Removed old pause methods - now using centralized controller pause service
    
    def _get_ros2_processes(self) -> List[psutil.Process]:
        """Get all ROS2 processes related to Argo using node manager"""
        try:
            all_processes = self.node_manager.get_all_ros_processes()
            # Convert to psutil.Process objects
            processes = []
            for node_name, node_processes in all_processes.items():
                for proc_info in node_processes:
                    try:
                        proc = psutil.Process(proc_info['pid'])
                        processes.append(proc)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
            return processes
        except Exception as e:
            print(f"⚠️  Error getting processes: {e}")
            return []
    
    def _is_launch_running(self) -> bool:
        """Check if any of the expected nodes are still running"""
        if self.process and self.process.poll() is None:
            return True
        
        # Check if any expected nodes are running
        # Exclude special nodes (bno085) that run as independent services
        node_status = self._get_node_status()
        running_nodes = [node for node, status in node_status.items() 
                        if "RUNNING" in status and node in self.expected_nodes]
        return len(running_nodes) > 0
    
    def _reset_debug_ports(self):
        """Reset debug port allocator to the base value."""
        self._next_debug_port = self.debug_node_port_base

    def _allocate_debug_port(self) -> int:
        """Return the next debug port and increment allocator."""
        port = self._next_debug_port
        self._next_debug_port += 1
        return port
    
    def _launch_nodes_directly(self):
        """Launch all expected nodes directly without using ros2 launch"""
        print("🚀 Launching nodes directly...")
        
        # Clear the process cache so we get fresh results after launching
        self.node_manager.clear_process_cache()
        if self.debug_nodes:
            self._reset_debug_ports()
        
        # Get the nodes directory
        nodes_dir = os.path.join(self.argo_dir, 'nodes')
        
        # Use expected_nodes if defined (e.g., in simulation mode), otherwise discover all nodes
        if hasattr(self, 'expected_nodes') and self.expected_nodes:
            node_scripts = self.expected_nodes
            print(f"Launching specific nodes: {', '.join(node_scripts)}")
        else:
            # --- REFACTORED: Use physical_robot group from YAML ---
            node_scripts = self.physical_robot_nodes
            print(f"✅ Launching physical robot nodes from YAML: {', '.join(node_scripts)}")
        
        # Launch each node in a separate process
        # Store as list of dicts: {'proc': process, 'name': script_name}
        self.node_processes = []
        self.node_output_threads = []
        
        # Launch regular Python nodes
        for script in node_scripts:
            script_path = os.path.join(nodes_dir, script)
            if os.path.exists(script_path):
                print(f"✅ Launching {script}...")
                # Launch each node with proper ROS2 environment
                # Use None for stdout/stderr so output goes directly to systemd journal
                argo_yaml_path = os.path.join(self.argo_dir, 'nodes', 'argo.yaml')
                python_cmd_parts: List[str] = []
                debug_port: Optional[int] = None

                if self.debug_nodes:
                    debug_port = self._allocate_debug_port()
                    python_cmd_parts.extend([
                        'python3',
                        '-m',
                        'debugpy',
                        '--listen',
                        shlex.quote(f'{self.debug_node_listen_host}:{debug_port}')
                    ])
                    if self.debug_node_wait:
                        python_cmd_parts.append('--wait-for-client')
                else:
                    python_cmd_parts.append('python3')

                python_cmd_parts.append(shlex.quote(script_path))

                # Add args from YAML configuration if available (for simulation nodes)
                if hasattr(self, 'simulation_node_args') and script in self.simulation_node_args:
                    args = self.simulation_node_args[script]
                    if args:
                        # Properly quote arguments to handle spaces in values (e.g., map names)
                        quoted_args = [shlex.quote(arg) for arg in args]
                        python_cmd_parts.extend(quoted_args)

                # Add parameter file so nodes can load argo.yaml parameters
                python_cmd_parts.extend(['--ros-args', '--params-file', shlex.quote(argo_yaml_path)])

                cmd_str = 'source /opt/ros/humble/setup.bash && ' + ' '.join(python_cmd_parts)

                cmd = ['bash', '-c', cmd_str]
                proc = subprocess.Popen(
                    cmd,
                    cwd=self.argo_dir,
                    stdout=None,  # Let output go to systemd journal
                    stderr=None,  # Let errors go to systemd journal
                    universal_newlines=True
                )
                # Store process with script name for easier debugging
                process_info = {'proc': proc, 'name': script}
                if debug_port is not None:
                    process_info['debug_port'] = debug_port
                    wait_text = "waiting for debugger attach" if self.debug_node_wait else "listening for debugger"
                    print(f"🐞 {script} {wait_text} on {self.debug_node_listen_host}:{debug_port}")
                self.node_processes.append(process_info)
                print(f"✅ Launched {script} (PID: {proc.pid})")
            else:
                print(f"⚠️  Warning: {script} not found at {script_path}")
        
        # Launch special nodes (like foxglove_bridge)
        # In simulation mode, launch special nodes defined in self.special_nodes
        # In normal mode, discover and launch all special nodes
        special_nodes_to_launch = []
        if hasattr(self, 'special_nodes') and self.special_nodes:
            # Simulation mode: use explicitly defined special nodes
            special_nodes_to_launch = self.special_nodes
        elif not (hasattr(self, 'expected_nodes') and self.expected_nodes):
            # --- REFACTORED: Use physical_robot_special_nodes from YAML ---
            special_nodes_to_launch = self.physical_robot_special_nodes
            print(f"✅ Launching physical robot special nodes from YAML: {', '.join(special_nodes_to_launch)}")

        for special_node in special_nodes_to_launch:
            if special_node == 'foxglove_bridge':
                print(f"✅ Launching {special_node}...")
                # Launch foxglove_bridge as a ROS2 package
                cmd = [
                    'bash',
                    '-c',
                    (
                        'source /opt/ros/humble/setup.bash && '
                        'ros2 run foxglove_bridge foxglove_bridge '
                        '--ros-args --log-level warn'
                    ),
                ]
                try:
                    # First, verify the package exists by trying to get package info
                    import shutil
                    check_cmd = ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 pkg executables foxglove_bridge']
                    check_proc = subprocess.run(
                        check_cmd,
                        cwd=self.argo_dir,
                        capture_output=True,
                        text=True,
                        timeout=5
                    )
                    
                    if check_proc.returncode != 0:
                        print(f"❌ FAILED to start {special_node}!")
                        print("")
                        print("   ERROR: foxglove-bridge package not found!")
                        print("")
                        if "package" in check_proc.stderr.lower() or "not found" in check_proc.stderr.lower():
                            print("   The foxglove-bridge package is not installed.")
                            print("")
                            print("   To fix this, run:")
                            print("     make install-foxglove-bridge")
                            print("")
                            print("   Or install all dependencies:")
                            print("     make install-deps")
                            print("")
                        else:
                            print(f"   Error: {check_proc.stderr[:200]}")
                        raise RuntimeError(f"foxglove-bridge package not found: {check_proc.stderr}")
                    
                    # Package exists, now launch it
                    proc = subprocess.Popen(
                        cmd,
                        cwd=self.argo_dir,
                        stdout=None,  # Let output go to systemd journal
                        stderr=None,  # Let errors go to systemd journal
                        universal_newlines=True
                    )
                    # Wait a moment to see if it fails immediately
                    import time
                    time.sleep(2)
                    
                    # Check if process is still running
                    if proc.poll() is not None:
                        # Process exited immediately - likely an error
                        print(f"❌ FAILED to start {special_node}!")
                        print(f"   Exit code: {proc.returncode}")
                        print("")
                        print("   ERROR: foxglove-bridge started but exited immediately!")
                        print("")
                        print("   This usually means:")
                        print("   - The package is installed but there's a runtime error")
                        print("   - Check system logs for details: journalctl -u argo_launch_standard.service -n 50")
                        print("")
                        print("   Or try running manually to see the error:")
                        print("     ros2 run foxglove_bridge foxglove_bridge")
                        print("")
                        raise RuntimeError(f"foxglove_bridge exited with code {proc.returncode}")
                    
                    # Store process with node name for easier debugging
                    self.node_processes.append({'proc': proc, 'name': special_node})
                    print(f"✅ Launched {special_node} (PID: {proc.pid})")
                    
                    # Verify it's actually running by checking for the process
                    import psutil
                    foxglove_running = False
                    for p in psutil.process_iter(['pid', 'name', 'cmdline']):
                        try:
                            cmdline = ' '.join(p.info['cmdline'] or [])
                            if 'foxglove_bridge' in cmdline and 'ros2 run' in cmdline:
                                foxglove_running = True
                                break
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            continue
                    
                    if not foxglove_running:
                        print(f"❌ CRITICAL ERROR: {special_node} process not found after launch!")
                        print("   The launch may have failed silently.")
                        print("   Simulation requires foxglove_bridge for visualization.")
                        print("")
                        print("   Check for errors above or run manually to diagnose:")
                        print("     ros2 run foxglove_bridge foxglove_bridge")
                        raise RuntimeError(f"foxglove_bridge failed to start - process not found after launch")
                    
                except subprocess.SubprocessError as e:
                    print(f"❌ ERROR launching {special_node}: {e}")
                    print("")
                    print("   foxglove-bridge may not be installed.")
                    print("   Run: make install-foxglove-bridge")
                    raise
            elif special_node == 'bno085':
                # BNO085 is excluded from launch but should be monitored for health
                print(f"ℹ️  Monitoring {special_node} (running as independent service)")
        
        # Set the main process to the first node process for compatibility
        if self.node_processes:
            self.process = self.node_processes[0]['proc']
        else:
            print("❌ No nodes were launched")
            self.process = None
    
    def _get_node_status(self) -> Dict[str, str]:
        """Get status of individual nodes"""
        status = {}
        processes = self._get_ros2_processes()
        
        # Check all expected nodes (regular Python nodes + special nodes)
        for node in self.all_expected_nodes:
            node_running = False
            for proc in processes:
                try:
                    cmdline = ' '.join(proc.cmdline() or [])
                    # Handle different node types
                    if node == 'foxglove_bridge':
                        # Special case: foxglove_bridge runs as "ros2 run foxglove_bridge foxglove_bridge"
                        if 'foxglove_bridge' in cmdline and 'ros2 run' in cmdline:
                            node_running = True
                            break
                    else:
                        # Regular Python nodes: look for the .py filename
                        if node in cmdline:
                            node_running = True
                            break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            status[node] = "🟢 RUNNING" if node_running else "🔴 STOPPED"
        
        return status
    
    def _is_critical_node(self, node: str) -> bool:
        """Check if a node is critical for boat operation"""
        return node in self.critical_nodes
    
    # Removed _should_restart_node method - nodes are not restarted automatically
    # Node failures are preserved for debugging purposes
    
    def _create_lifecycle_services(self):
        """Create ROS2 services for lifecycle management (only in continuous mode)"""
        if not self.ros2_node or self.lifecycle_services_created:
            return
        
        try:
            # Create lifecycle management services
            self.start_service = self.ros2_node.create_service(
                Trigger, '/argo/lifecycle/start', self._handle_start_service)
            self.stop_service = self.ros2_node.create_service(
                Trigger, '/argo/lifecycle/stop', self._handle_stop_service)
            self.restart_service = self.ros2_node.create_service(
                Trigger, '/argo/lifecycle/restart', self._handle_restart_service)
            self.status_service = self.ros2_node.create_service(
                Trigger, '/argo/lifecycle/status', self._handle_status_service)
            
            # Create status publisher
            self.status_publisher = self.ros2_node.create_publisher(
                String, '/argo/lifecycle/status', 10)
            
            self.lifecycle_services_created = True
            print("✅ Lifecycle management ROS2 services created:")
            print("   - /argo/lifecycle/start")
            print("   - /argo/lifecycle/stop")
            print("   - /argo/lifecycle/restart")
            print("   - /argo/lifecycle/status")
            print("   - /argo/lifecycle/status (topic)")
        except Exception as e:
            print(f"⚠️  Failed to create lifecycle services: {e}")
    
    def _handle_start_service(self, request, response):
        """Handle /argo/lifecycle/start service request"""
        try:
            if self._is_launch_running():
                response.success = False
                response.message = "Argo nodes are already running"
            else:
                # Start nodes in background thread to avoid blocking service call
                import threading
                threading.Thread(target=self._launch_nodes_directly, daemon=True).start()
                response.success = True
                response.message = "Argo nodes starting..."
        except Exception as e:
            response.success = False
            response.message = f"Error starting nodes: {e}"
        return response
    
    def _handle_stop_service(self, request, response):
        """Handle /argo/lifecycle/stop service request"""
        try:
            if not self._is_launch_running():
                response.success = False
                response.message = "Argo nodes are not running"
            else:
                success = self.stop()
                response.success = success
                response.message = "Argo nodes stopped" if success else "Failed to stop some nodes"
        except Exception as e:
            response.success = False
            response.message = f"Error stopping nodes: {e}"
        return response
    
    def _handle_restart_service(self, request, response):
        """Handle /argo/lifecycle/restart service request"""
        try:
            # Restart in background thread to avoid blocking service call
            import threading
            def do_restart():
                self.stop()
                time.sleep(2)
                self._launch_nodes_directly()
            threading.Thread(target=do_restart, daemon=True).start()
            response.success = True
            response.message = "Argo nodes restarting..."
        except Exception as e:
            response.success = False
            response.message = f"Error restarting nodes: {e}"
        return response
    
    def _handle_status_service(self, request, response):
        """Handle /argo/lifecycle/status service request"""
        try:
            node_status = self._get_node_status()
            running_count = sum(1 for status in node_status.values() if "RUNNING" in status)
            total_count = len(node_status)
            
            status_dict = {
                'running_count': running_count,
                'total_count': total_count,
                'nodes': node_status
            }
            
            response.success = True
            response.message = json.dumps(status_dict)
        except Exception as e:
            response.success = False
            response.message = f"Error getting status: {e}"
        return response
    
    def _publish_status_update(self, message: str):
        """Publish a status update to /argo/lifecycle/status topic"""
        if self.status_publisher:
            try:
                msg = String()
                msg.data = message
                self.status_publisher.publish(msg)
            except Exception as e:
                # Silently fail - status updates are not critical
                pass
    
    def continuous(self) -> bool:
        """Start Argo and keep it running with fault tolerance"""
        print("🚀 Starting Argo ROS2 nodes...")
        
        if self._is_launch_running():
            print("⚠️  Argo launch process is already running")
            return True
        
        # Check if shutdown has been requested before launching
        if self.shutdown_requested:
            print("⚠️  Shutdown requested before node launch - aborting")
            return True
        
        try:
            # Start the nodes directly
            self._launch_nodes_directly()
            
            # Check again after launch
            if self.shutdown_requested:
                print("⚠️  Shutdown requested during launch - stopping nodes")
                self.stop()
                return True
            
            # Wait and check for actual node startup with continuous feedback
            print("⏳ Waiting for nodes to start...")
            
            timeout = 30  # 30 second timeout
            check_interval = .3  # Check every .3 second
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # Check for shutdown request during launch
                if self.shutdown_requested:
                    print("⚠️  Shutdown requested during node startup - stopping nodes")
                    self.stop()
                    return True
                
                if not self._is_launch_running():
                    print("❌ Launch process died during startup")
                    
                    # Log that process died (output is now in systemd journal)
                    if self.process and self.process.poll() is not None:
                        print(
                            "💀 Process died - check systemd journal for detailed output")
                    
                    return False
                
                # Check which nodes are running
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]
                
                # Show progress periodically (journal-friendly)
                elapsed = time.time() - start_time
                if elapsed > 0 and int(elapsed * 10) % 10 == 0:  # Print every second
                    if running_nodes and stopped_nodes:
                        print(
                            f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                        print(f"   Not started: {', '.join(stopped_nodes)}")
                    elif running_nodes:
                        print(
                            f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                    else:
                        print(
                            f"⏳ Waiting for nodes to start... (no nodes detected yet)")
                
                # If all expected nodes are running, wait for stabilization with monitoring
                if len(running_nodes) == len(self.all_expected_nodes):
                    print(
                        f"\n✅ All {len(running_nodes)} nodes detected: {', '.join(running_nodes)}")
                    print(
                        f"⏳ Monitoring nodes during {self.stabilization_wait}s stabilization period...")
                    
                    # Monitor nodes during stabilization period
                    stabilization_start = time.time()
                    stabilization_check_interval = 1.0  # Check every second during stabilization
                    
                    while time.time() - stabilization_start < self.stabilization_wait:
                        # Check for shutdown request during stabilization
                        if self.shutdown_requested:
                            print("⚠️  Shutdown requested during stabilization - stopping nodes")
                            self.stop()
                            return True
                        
                        # Check for node failures during stabilization
                        current_status = self._get_node_status()
                        current_running = [
                            node for node, status in current_status.items() if "RUNNING" in status]
                        current_stopped = [
                            node for node, status in current_status.items() if "STOPPED" in status]
                        
                        if current_stopped:
                            # Node(s) failed during stabilization
                            print(
                                f"\n⚠️  Node failure detected during stabilization: {', '.join(current_stopped)}")
                            
                            # Get error messages for failed nodes
                            fatal_messages = self._get_fatal_messages_for_nodes()
                            for failed_node in current_stopped:
                                if failed_node in fatal_messages:
                                    print(
                                        f"   {failed_node}: {fatal_messages[failed_node]}")
                            
                            # Break out of stabilization to handle the failure
                            break
                        
                        # Show progress during stabilization (journal-friendly)
                        remaining_time = self.stabilization_wait - \
                            (time.time() - stabilization_start)
                        # Only print every 2 seconds
                        if remaining_time > 0 and int(remaining_time) % 2 == 0:
                            print(
                                f"⏳ Stabilizing... {remaining_time:.1f}s remaining ({len(current_running)}/{len(self.all_expected_nodes)} nodes)")
                        
                        time.sleep(stabilization_check_interval)
                    
                    # Final check after stabilization period
                    final_node_status = self._get_node_status()
                    final_running_nodes = [
                        node for node, status in final_node_status.items() if "RUNNING" in status]
                    
                    if len(final_running_nodes) == len(self.all_expected_nodes):
                        print(f"✅ Argo launch process started successfully")
                        print(
                            f"✅ All {len(final_running_nodes)} nodes running and stable: {', '.join(final_running_nodes)}")
                        break  # Continue to monitoring phase
                    else:
                        failed_nodes = [
                            node for node in self.all_expected_nodes if node not in final_running_nodes]
                        print(f"⚠️  Some nodes failed during stabilization period")
                        print(
                            f"   Still running: {', '.join(final_running_nodes)}")
                        print(f"   Failed nodes: {', '.join(failed_nodes)}")
                        
                        # Check if we have critical nodes running
                        critical_running = [
                            node for node in self.critical_nodes if node in final_running_nodes]
                        
                        if len(critical_running) == len(self.critical_nodes):
                            print(
                                f"✅ Critical nodes operational: {', '.join(critical_running)}")
                            print(
                                f"✅ Argo will continue operating with {len(final_running_nodes)}/{len(self.all_expected_nodes)} nodes")
                            break  # Continue to monitoring phase
                        elif len(final_running_nodes) >= 3:  # At least 3 nodes running
                            print(
                                f"✅ Sufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(
                                f"✅ Argo will continue operating with available sensors")
                            break  # Continue to monitoring phase
                        else:
                            print(
                                f"❌ Insufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(
                                f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in final_running_nodes])}")
                            return False
                
                time.sleep(check_interval)
            
            # Check if we timed out
            if time.time() - start_time >= timeout:
                print(f"\n⚠️  Timeout reached after {timeout}s")
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]
                
                if running_nodes:
                    failed_nodes = [
                        node for node in self.all_expected_nodes if node not in running_nodes]
                    print(
                        f"✅ {len(running_nodes)} nodes running: {', '.join(running_nodes)}")
                    if failed_nodes:
                        print(
                            f"⚠️  {len(failed_nodes)} nodes not started: {', '.join(failed_nodes)}")
                    
                    # Check if we have critical nodes running
                    critical_running = [
                        node for node in self.critical_nodes if node in running_nodes]
                    
                    if len(critical_running) == len(self.critical_nodes):
                        print(
                            f"✅ Critical nodes operational: {', '.join(critical_running)}")
                        print(
                            f"✅ Argo will continue operating with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                    elif len(running_nodes) >= 3:  # At least 3 nodes running
                        print(
                            f"✅ Sufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                        print(
                            f"✅ Argo will continue operating with available sensors")
                    else:
                        print(
                            f"❌ Insufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                        print(
                            f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in running_nodes])}")
                        return False
                else:
                    print(f"❌ No nodes running after timeout")
                    return False
                
        except Exception as e:
            print(f"❌ Error starting Argo: {e}")
            return False
        
        # Create ROS2 services for lifecycle management
        self._create_lifecycle_services()
        
        print("🔄 Starting continuous monitoring with ROS2 integration...")
        print("   Press Ctrl+C to stop")
        print("   NOTE: Node failures will be logged but NOT restarted for debugging")
        print("   ROS2 services available for remote control")
        
        # Publish initial status
        self._publish_status_update("Argo lifecycle manager running")
        
        try:
            last_check_time = time.time()
            check_interval = 300  # Check every 5 minutes (much less frequent)
            
            while not self.shutdown_requested and (rclpy.ok() if self.ros2_node else True):
                # Check shutdown flag FIRST before any operations
                if self.shutdown_requested:
                    break
                
                # Spin ROS2 node to process service requests (if available)
                if self.ros2_node:
                    try:
                        rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
                    except Exception:
                        # Context may be shutting down
                        if self.shutdown_requested:
                            break
                else:
                    time.sleep(0.1)
                
                # Periodic status check
                current_time = time.time()
                if current_time - last_check_time >= check_interval:
                    last_check_time = current_time
                    
                    # Check node status
                    node_status = self._get_node_status()
                    running_nodes = [
                        node for node, status in node_status.items() if "RUNNING" in status]
                    stopped_nodes = [
                        node for node, status in node_status.items() if "STOPPED" in status]
                    
                    # Publish status update
                    status_msg = f"Running: {len(running_nodes)}/{len(self.all_expected_nodes)}"
                    self._publish_status_update(status_msg)
                    
                    if stopped_nodes:
                        # Log stopped nodes but do NOT restart them
                        print(
                            f"⚠️  {len(stopped_nodes)} nodes stopped: {', '.join(stopped_nodes)}")
                        
                        # Check if critical nodes are still running
                        critical_running = [
                            n for n in self.critical_nodes if n in running_nodes]
                        critical_stopped = [
                            n for n in self.critical_nodes if n in stopped_nodes]
                        
                        if critical_stopped:
                            print(
                                f"❌ CRITICAL NODES STOPPED: {', '.join(critical_stopped)}")
                            print(
                                f"   System will continue with remaining nodes for debugging")
                            print(f"   Check systemd journal for error details")
                            self._publish_status_update(f"CRITICAL: {', '.join(critical_stopped)} stopped")
                        else:
                            print(
                                f"✅ Critical nodes operational: {', '.join(critical_running)}")
                        
                        # Show system status
                        if len(running_nodes) >= 3:
                            print(
                                f"✅ System operational with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                        else:
                            print(
                                f"⚠️  Low node count: {len(running_nodes)}/{len(self.all_expected_nodes)} nodes running")
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping continuous monitoring...")
            self.shutdown_requested = True
        except Exception as e:
            print(f"❌ Error in continuous mode: {e}")
            self._publish_status_update(f"Error: {e}")
            self.shutdown_requested = True
        finally:
            # Publish final status before cleanup
            self._publish_status_update("Argo lifecycle manager stopping")
            
            # Stop all nodes first only if they are actually running
            if self._is_launch_running():
                self.stop()
            
            # Clean up ROS2 last
            self._cleanup_ros2()
            
            # Ensure we don't relaunch after shutdown
            self.shutdown_requested = True
            
            # Explicitly exit if shutdown was requested
            if self.shutdown_requested:
                print("✅ Lifecycle manager shutdown complete - exiting")
                # Flush stdout/stderr to ensure all messages are written before prompt appears
                sys.stdout.flush()
                sys.stderr.flush()
                # CRITICAL: Wait for any remaining shutdown messages from child processes to flush
                # Even though we waited in stop(), there might be stragglers
                # ROS2 nodes write shutdown messages to stderr which is unbuffered
                time.sleep(3.0)
                sys.stdout.flush()
                sys.stderr.flush()
                # One final wait to ensure everything is flushed
                time.sleep(1.0)
                sys.stdout.flush()
                sys.stderr.flush()
                sys.exit(0)
        
        return True  # Always return True for clean exit
    
    def stop(self) -> bool:
        """Stop the Argo launch process and all related nodes"""
        # Set shutdown flag immediately to prevent loops
        self.shutdown_requested = True
        
        print("🛑 Stopping Argo ROS2 nodes...")
        
        # First, try to terminate tracked processes individually for better control
        terminated_processes = []
        if hasattr(self, 'node_processes') and self.node_processes:
            for proc_info in self.node_processes:
                if 'proc' in proc_info:
                    proc = proc_info['proc']
                    name = proc_info.get('name', 'unknown')
                    try:
                        if proc.poll() is None:  # Process is still running
                            proc.terminate()
                            terminated_processes.append((proc, name))
                            print(f"  📤 Sent SIGTERM to {name} (PID: {proc.pid})")
                    except Exception as e:
                        print(f"  ⚠️  Error terminating {name}: {e}")
        
        # Wait for tracked processes to finish
        max_wait_per_process = 3.0
        for proc, name in terminated_processes:
            try:
                proc.wait(timeout=max_wait_per_process)
                print(f"  ✅ {name} stopped gracefully")
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                    proc.wait(timeout=1.0)
                    print(f"  ⚡ Force killed {name}")
                except:
                    print(f"  ⚠️  Could not force kill {name}")
            except:
                pass
        
        # Wait for main process if it exists
        if self.process:
            try:
                if self.process.poll() is None:
                    self.process.terminate()
                    print(f"  📤 Sent SIGTERM to main launch process")
                    self.process.wait(timeout=max_wait_per_process)
                    print(f"  ✅ Main launch process stopped")
            except subprocess.TimeoutExpired:
                try:
                    self.process.kill()
                    self.process.wait(timeout=1.0)
                    print(f"  ⚡ Force killed main launch process")
                except:
                    pass
            except:
                pass
        
        # Also use process group kill as backup (may catch bash wrapper processes)
        try:
            os.killpg(self.pgid, signal.SIGTERM)
            print(f"  📤 Sent SIGTERM to process group {self.pgid}")
        except ProcessLookupError:
            # Process group already gone
            print(f"  ℹ️  Process group {self.pgid} already terminated")
        except Exception as e:
            print(f"  ⚠️  Could not kill process group: {e}")
        
        # Use pkill as final backup to catch any remaining simulation processes
        try:
            subprocess.run(['pkill', '-TERM', '-f', 'argo_unified_simulator_bridge'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-TERM', '-f', 'controller.py'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-TERM', '-f', 'foxglove_bridge'], 
                         capture_output=True, timeout=2)
            print(f"  📤 Sent SIGTERM via pkill to remaining nodes")
        except Exception as e:
            print(f"  ⚠️  pkill cleanup failed: {e}")
        
        # Wait a bit for processes to terminate
        time.sleep(2.0)
        
        # Force kill any remaining processes
        try:
            subprocess.run(['pkill', '-9', '-f', 'argo_unified_simulator_bridge'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-9', '-f', 'controller.py'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-9', '-f', 'foxglove_bridge'], 
                         capture_output=True, timeout=2)
        except:
            pass
        
        sys.stdout.flush()
        sys.stderr.flush()
        print("✅ All node processes terminated.")
        
        self.node_processes.clear()
        self._cleanup_ros2()
        return True

    def _stop_fallback(self) -> bool:
        """Fallback method to stop nodes individually if process group kill fails."""
        print("⚠️  Falling back to individual process termination...")
        # Stop recording first if it's active (only if we have a ROS2 node available)
        # Don't try to stop recording during shutdown as ROS2 context may be unavailable
        if self.ros2_node and not self.shutdown_requested:
            recording_active = self._check_recording_status()
            if recording_active:
                print("⚠️  Recording is active - stopping recording first...")
                try:
                    success, message = self._call_trigger_service('/argo/recording/stop', timeout_sec=5.0, debug=False)
                    if success:
                        print("✅ Recording stopped successfully")
                        time.sleep(1)  # Allow recording to fully stop
                    else:
                        print(f"⚠️  Failed to stop recording: {message}")
                        print("⚠️  Proceeding with node shutdown anyway...")
                except Exception as e:
                    print(f"⚠️  Could not stop recording: {e}")
        
        # Clear the process cache so next query gets fresh results
        if hasattr(self, 'node_manager'):
            self.node_manager.clear_process_cache()
        
        success = True
        
        # Stop main launch process
        if self.process and self._is_launch_running():
            try:
                print(f"DEBUG (Thread: {threading.get_ident()}): Terminating main launch process {self.process.pid}")
                self.process.terminate()
                self.process.wait(timeout=5)
                print("✅ Main launch process stopped")
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing launch process...")
                self.process.kill()
                self.process.wait()
            except Exception as e:
                print(f"⚠️  Error stopping launch process: {e}")
                success = False
        
        # Stop individual node processes
        if hasattr(self, 'node_processes') and self.node_processes:
            print(
                f"DEBUG (Thread: {threading.get_ident()}): Stopping {len(self.node_processes)} node processes (fallback)...")
            for node_info in self.node_processes:
                proc = node_info['proc']
                name = node_info['name']
                
                if proc and proc.poll() is None:
                    try:
                        print(f"  DEBUG (Thread: {threading.get_ident()}): Terminating {name} (PID: {proc.pid})...")
                        proc.terminate()
                        proc.wait(timeout=2)
                        print(f"  ✅ {name} stopped gracefully")
                    except subprocess.TimeoutExpired:
                        print(f"  ⚡ Force killing {name} (PID: {proc.pid})")
                        proc.kill()
                        proc.wait(timeout=1)
                    except Exception as e:
                        print(f"  ⚠️  Error stopping {name} (PID: {proc.pid}): {e}")
            self.node_processes = []
        
        # Also use pkill as backup to catch any remaining processes
        print(f"DEBUG (Thread: {threading.get_ident()}): Using pkill as a fallback...")
        for node in self.expected_nodes:
            print(f"  DEBUG: pkill -f /{node}")
            subprocess.run(['pkill', '-f', f'/{node}'], 
                           capture_output=True, timeout=2)

        # Also kill special nodes (like foxglove_bridge)
        if hasattr(self, 'special_nodes') and self.special_nodes:
            for special_node in self.special_nodes:
                if special_node == 'foxglove_bridge':
                    print(f"  DEBUG: pkill -9 -f foxglove_bridge")
                    # Use -9 (SIGKILL) to force kill both parent and child processes
                    subprocess.run(['pkill', '-9', '-f', 'foxglove_bridge'],
                         capture_output=True, timeout=2)
                    # Also try killing by process name directly
                    subprocess.run(['pkill', '-9', 'foxglove_bridge'],
                         capture_output=True, timeout=2)
        
        print("✅ Argo processes terminated")
        
        self.process = None
        # Stop remote processes if they were started
        self._stop_remote_processes()
        print("✅ All Argo processes stopped")
        self._cleanup_ros2()
        return success
    
    def restart(self) -> bool:
        """Restart the Argo launch process with recording awareness"""
        print("🔄 Restarting Argo ROS2 nodes...")
        
        # Check if recording is active
        recording_active = self._check_recording_status()
        if recording_active:
            print("⚠️  Recording is active - stopping recording before restart...")
            success, message = self._call_trigger_service('/argo/recording/stop', timeout_sec=10.0)
            if success:
                print("✅ Recording stopped successfully")
                time.sleep(2)  # Allow recording to fully stop
            else:
                print(f"⚠️  Failed to stop recording: {message}")
                print("⚠️  Proceeding with restart anyway...")
        
        self.stop()
        time.sleep(1)
        return self.start()
    
    def _check_recording_status(self) -> bool:
        """Check if recording is currently active"""
        try:
            success, message = self._call_trigger_service(
                '/argo/recording/get_status', timeout_sec=5.0, debug=False)
            
            # The 'success' field of Trigger response indicates recording state
            # True means recording is active
            return success and message.lower() != 'not active'
        except Exception:
            # If we can't check, assume not recording to be safe
            return False
    
    def _check_simulation_prerequisites(self) -> bool:
        """Check for simulation prerequisites and guide user if missing."""
        print("🔍 Checking simulation prerequisites...")
        
        # 1. Check for sailboat-playground submodule
        submodule_path = os.path.join(self.argo_dir, "simulator", "sailboat-playground")
        if not os.path.exists(os.path.join(submodule_path, ".git")):
            print("❌ sailboat-playground submodule not found!")
            print("   The simulator depends on this submodule.")
            
            try:
                response = input("   Would you like to initialize it now? (y/N): ").lower()
                if response == 'y':
                    print("🔄 Initializing submodule...")
                    # Use absolute path for make to avoid ambiguity
                    # Use -C to run make in the correct directory.
                    subprocess.run(
                        ['make', '-C', self.argo_dir, 'submodule-init'], check=True)
                    print("✅ Submodule initialized successfully!")
                else:
                    print("   Aborting. Please run 'make submodule-init' to proceed.")
                    return False
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                print(f"   Error initializing submodule: {e}")
                print("   Please run 'make submodule-init' manually.")
                return False

        # 2. Check for foxglove-bridge
        try:
            ros_distro = os.environ.get("ROS_DISTRO")
            if not ros_distro:
                print("⚠️  ROS_DISTRO not set, cannot check for foxglove-bridge.")
            else:
                subprocess.run(
                    ['dpkg', '-l', f'ros-{ros_distro}-foxglove-bridge'], 
                    check=True, capture_output=True, text=True)
                print("✅ foxglove-bridge is installed.")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("❌ foxglove-bridge is not installed!")
            print("   The simulation uses foxglove-bridge for visualization.")
            try:
                response = input("   Would you like to install it now? (y/N): ").lower()
                if response == 'y':
                    print("🔧 Installing foxglove-bridge...")
                    subprocess.run(
                        ['make', '-C', self.argo_dir, 'install-foxglove-bridge'], 
                        check=True)
                    print("✅ foxglove-bridge installed successfully!")
                else:
                    print("   Aborting. Please run 'make install-foxglove-bridge' to proceed.")
                    return False
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                print(f"   Error installing foxglove-bridge: {e}")
                print("   Please run 'make install-foxglove-bridge' manually.")
                return False
                
        # 3. Check for Python dependencies
        try:
            import rclpy  # Check a core ROS2 dependency
        except ImportError:
            print("❌ Python dependencies (rclpy) not found!")
            print("   The simulation requires Python packages from requirements.txt.")
            try:
                response = input("   Would you like to install them now? (y/N): ").lower()
                if response == 'y':
                    print("🐍 Installing Python dependencies...")
                    subprocess.run(
                        ['make', '-C', self.argo_dir, 'install-python-deps'],
                        check=True)
                    print("✅ Python dependencies installed successfully!")
                else:
                    print("   Aborting. Please run 'make install-python-deps' to proceed.")
                    return False
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                print(f"   Error installing Python dependencies: {e}")
                print("   Please run 'make install-python-deps' manually.")
                return False
                
        return True

    def simulate_local(self, force_mock: bool = False, debug: bool = False) -> bool:
        """Launch Argo in local simulation mode.
        
        Args:
            force_mock: If True, force use of mock simulator even if real simulator is available
            debug: If True, enable debug tracing in the simulator bridge
        """
        return self._simulate(mode='local', force_mock=force_mock, debug=debug)

    def simulate_remote(self) -> bool:
        """Launch Argo in remote simulation mode."""
        print("INFO: Remote simulation requires manual setup on the remote machine")
        print("      and running 'scripts/remote_simulator_tunnel.sh' on this machine.")
        # Start remote processes
        if not self._start_remote_tunnel():
            return False
        if not self._start_remote_simulator():
            self._stop_remote_processes()
            return False
        return self._simulate(mode='remote')

    def _simulate(self, mode: str, force_mock: bool = False, debug: bool = False) -> bool:
        """
        Launch Argo in simulation mode.

        In simulation mode, only essential nodes are launched:
        - argo_unified_simulator_bridge.py (provides simulated sensor data)
        - controller.py (autonomous navigation)
        - sailing_area_publisher.py (publishes sailing area markers)
        - foxglove_bridge (provides visualization via Foxglove Studio)

        Hardware nodes that conflict with simulator are excluded:
        - gps.py (conflicts with simulator GPS topics)
        - bno085.py (conflicts with simulator compass topics)
        - anem.py (conflicts with simulator wind topics)
        - rudder_sail_radio.py (conflicts with simulator control)
        
        Note: argo_battery_water.py and temp_monitor.py run as independent systemd services
        """
        self.simulation_mode = mode
        print(f"🚢 Starting Argo in SIMULATION mode ({mode.upper()})...")

        if not self._check_simulation_prerequisites():
            return False

        if mode == 'local':
            print("Local simulation runs the simulator on this machine.")
        else:
            print("Remote simulation connects to a simulator on another machine.")

        print("Simulation mode excludes conflicting hardware nodes:")
        print("  - gps.py (GPS data provided by simulator)")
        print("  - bno085.py (compass data provided by simulator)")
        print("  - anem.py (wind data provided by simulator)")
        print("  - rudder_sail_radio.py (control handled by simulator)")
        print("Simulation mode includes visualization:")
        print("  - foxglove_bridge (Foxglove Studio at ws://localhost:8765)")

        # Use simulation nodes from YAML configuration
        if not self.simulation_expected_nodes and not self.simulation_special_nodes:
            print("❌ No simulation nodes configured in argo_nodes.yaml")
            print("   Please add a 'simulation_nodes' section to the YAML file")
            return False

        # Check if simulator bridge exists (it should be in simulation nodes)
        simulator_bridge_path = os.path.join(
            self.argo_dir, "nodes", "argo_unified_simulator_bridge.py")
        if not os.path.exists(simulator_bridge_path):
            print(f"❌ Simulator bridge not found: {simulator_bridge_path}")
            return False

        # Use simulation nodes from YAML configuration
        self.expected_nodes = self.simulation_expected_nodes.copy()
        self.special_nodes = self.simulation_special_nodes.copy()
        self.critical_nodes = self.simulation_critical_nodes.copy()
        # Update all_expected_nodes for simulation mode monitoring
        self.all_expected_nodes = self.expected_nodes + self.special_nodes

        # Update simulator bridge args to use the correct mode and add --no-curses
        if 'argo_unified_simulator_bridge.py' in self.simulation_node_args:
            # Replace --mode local/remote with the actual mode
            args = self.simulation_node_args['argo_unified_simulator_bridge.py'].copy()
            # Replace any --mode argument with the current mode
            mode_found = False
            for i, arg in enumerate(args):
                if arg == '--mode' and i + 1 < len(args):
                    args[i + 1] = mode
                    mode_found = True
                    break
            # If --mode wasn't found in args, add it
            if not mode_found:
                args.extend(['--mode', mode])
            # Ensure --no-curses is present (required for non-interactive launch)
            if '--no-curses' not in args:
                args.append('--no-curses')
            # Add --force-mock if requested
            if force_mock and '--force-mock' not in args:
                args.append('--force-mock')
            # Add --debug if requested
            if debug and '--debug' not in args:
                args.append('--debug')
            # Note: Map name is now automatically loaded from argo_nodes.yaml by the simulator bridge
            # --map can still be passed to override if needed
            self.simulation_node_args['argo_unified_simulator_bridge.py'] = args
        else:
            # If simulator bridge doesn't have args configured, add --mode and --no-curses
            args = ['--mode', mode, '--no-curses']
            # Add --force-mock if requested
            if force_mock:
                args.append('--force-mock')
            # Add --debug if requested
            if debug:
                args.append('--debug')
            # Note: Map name is now automatically loaded from argo_nodes.yaml by the simulator bridge
            self.simulation_node_args['argo_unified_simulator_bridge.py'] = args
        
        # Also pass map name to sailing_area_publisher so it uses the same coordinate origin
        # Note: sailing_area_publisher now auto-loads from YAML, but we pass it explicitly to ensure consistency
        if self.simulation_map_name:
            if 'sailing_area_publisher.py' not in self.simulation_node_args:
                self.simulation_node_args['sailing_area_publisher.py'] = []
            args = self.simulation_node_args['sailing_area_publisher.py'].copy()
            # Add --map if not already present (explicit passing overrides auto-load if needed)
            if '--map' not in args:
                args.extend(['--map', self.simulation_map_name])
                self.simulation_node_args['sailing_area_publisher.py'] = args

        print(f"Expected simulation nodes: {', '.join(self.expected_nodes)}")
        print(f"Special simulation nodes: {', '.join(self.special_nodes)}")
        print(f"Critical simulation nodes: {', '.join(self.critical_nodes)}")

        # Launch simulation nodes
        self._launch_nodes_directly()

        # Wait for nodes to start
        print("⏳ Waiting for simulation nodes to start...")
        start_time = time.time()
        timeout = 30
        check_interval = 0.3

        # Calculate total expected nodes (regular + special)
        total_expected_nodes = len(
            self.expected_nodes) + len(self.special_nodes)

        while time.time() - start_time < timeout:
            node_status = self._get_node_status()
            running_nodes = [
                node for node, status in node_status.items() if "RUNNING" in status]
            stopped_nodes = [
                node for node, status in node_status.items() if "STOPPED" in status]

            # Check if any expected nodes have crashed
            all_expected = self.expected_nodes + self.special_nodes
            crashed_expected = [node for node in all_expected if node in stopped_nodes]
            if crashed_expected:
                # Node crashed during startup - exit immediately
                print(f"\n⚠️  Node crash detected during startup: {', '.join(crashed_expected)}")
                break

            # Progress reporting (journal-friendly)
            elapsed = time.time() - start_time
            if int(elapsed * 10) % 10 == 0:
                missing_nodes = [node for node in all_expected if node not in running_nodes]
                missing_str = f" (missing: {', '.join(missing_nodes)})" if missing_nodes else ""
                print(
                    f"⏳ Waiting for simulation nodes... {len(running_nodes)}/{total_expected_nodes} running{missing_str}")

            if len(running_nodes) == total_expected_nodes:
                break  # All nodes detected, proceed to stabilization

            time.sleep(check_interval)

        # Check if all expected nodes are running
        if len(running_nodes) != total_expected_nodes:
            all_expected = self.expected_nodes + self.special_nodes
            missing_nodes = [node for node in all_expected if node not in running_nodes]
            print(
                f"❌ Only {len(running_nodes)}/{total_expected_nodes} simulation nodes started")
            print(f"✅ Running: {', '.join(running_nodes) if running_nodes else '(none)'}")
            print(f"❌ Missing: {', '.join(missing_nodes) if missing_nodes else '(none)'}")
            print(f"📋 Expected: {', '.join(all_expected)}")
            
            # Get and display error messages for missing nodes
            fatal_messages = self._get_fatal_messages_for_nodes()
            for missing_node in missing_nodes:
                # Check if the process exists but isn't running
                node_status = self._get_node_status()
                if missing_node in node_status:
                    status = node_status[missing_node]
                    if "STOPPED" in status or "ERROR" in status:
                        if missing_node in fatal_messages:
                            print(f"   ⚠️  {missing_node}: {fatal_messages[missing_node]}")
                        else:
                            print(f"   ⚠️  {missing_node}: {status}")
                else:
                    if missing_node in fatal_messages:
                        print(f"   ⚠️  {missing_node}: {fatal_messages[missing_node]}")
                    else:
                        print(f"   ⚠️  {missing_node}: Not found in node status")
            
            # Clean up any nodes that did start before returning
            print("🛑 Stopping nodes that were started...")
            self.stop()
            return False

        # Monitor during stabilization period
        print("⏳ Monitoring simulation nodes during stabilization...")
        stabilization_start = time.time()
        self.stabilization_wait = 15.0  # 15 seconds stabilization for simulation

        while time.time() - stabilization_start < self.stabilization_wait:
            current_status = self._get_node_status()
            current_stopped = [
                node for node, status in current_status.items() if "STOPPED" in status]

            if current_stopped:
                print(
                    f"\n{'='*70}")
                print(f"❌ FATAL ERROR: Simulation node failure detected during stabilization: {', '.join(current_stopped)}")
                print(f"{'='*70}")
                # Get and display error messages
                fatal_messages = self._get_fatal_messages_for_nodes()
                for failed_node in current_stopped:
                    if failed_node in fatal_messages:
                        print(f"   {failed_node}: {fatal_messages[failed_node]}")
                    else:
                        print(f"   {failed_node}: Node crashed (check journal for details)")
                print(f"{'='*70}\n")
                # Terminate simulation on node software failure
                print("🛑 Terminating simulation due to node software failure...")
                self.stop()
                return False

            time.sleep(1.0)  # Check every second

        # Final status check
        final_node_status = self._get_node_status()
        final_running_nodes = [
            node for node, status in final_node_status.items() if "RUNNING" in status]
        final_stopped_nodes = [
            node for node, status in final_node_status.items() if "STOPPED" in status]

        # Check for critical nodes - foxglove_bridge is required for simulation
        critical_running = [
            node for node in self.critical_nodes if node in final_running_nodes]
        critical_stopped = [
            node for node in self.critical_nodes if node in final_stopped_nodes]
        
        # Also check special nodes (like foxglove_bridge) - these are required for simulation
        special_running = [
            node for node in self.special_nodes if node in final_running_nodes]
        
        # Check for critical node software failures - terminate simulation
        if critical_stopped:
            print(f"\n{'='*70}")
            print(f"❌ FATAL ERROR: Critical simulation nodes crashed: {', '.join(critical_stopped)}")
            print(f"{'='*70}")
            # Get and display error messages
            fatal_messages = self._get_fatal_messages_for_nodes()
            for failed_node in critical_stopped:
                if failed_node in fatal_messages:
                    print(f"   {failed_node}: {fatal_messages[failed_node]}")
                else:
                    print(f"   {failed_node}: Node crashed (check journal for details)")
            print(f"{'='*70}\n")
            # Terminate simulation on critical node software failure
            print("🛑 Terminating simulation due to critical node software failure...")
            self.stop()
            return False
        
        # Simulation requires foxglove_bridge to be running
        if 'foxglove_bridge' in self.special_nodes and 'foxglove_bridge' not in final_running_nodes:
            print(f"\n{'='*70}")
            print("❌ FATAL ERROR: foxglove_bridge is not running!")
            print("   Simulation requires foxglove_bridge for visualization.")
            print("   Please check the error messages above.")
            print("   Try running manually: ros2 run foxglove_bridge foxglove_bridge")
            print(f"{'='*70}\n")
            # Terminate simulation
            print("🛑 Terminating simulation due to missing critical component...")
            self.stop()
            return False
        
        if len(critical_running) == len(self.critical_nodes):
            print("✅ All critical simulation nodes running")
            success = True
        elif len(final_running_nodes) >= 2:  # At least simulator + one other
            print(
                f"✅ Sufficient simulation nodes running ({len(final_running_nodes)}/2+)")
            success = True
        else:
            print(f"\n{'='*70}")
            print(f"❌ FATAL ERROR: Insufficient simulation nodes running ({len(final_running_nodes)}/2+)")
            print(f"{'='*70}\n")
            # Terminate simulation on insufficient nodes
            print("🛑 Terminating simulation due to insufficient nodes...")
            self.stop()
            return False

        if success:
            print("🎉 Argo simulation mode started successfully!")
            print("Simulated sensor data available on:")
            print("  - /pose (compass heading)")
            print("  - /gps_cog, /gps_sog, /gps_velocity (GPS navigation)")
            print("  - /anem_speed_angle_temp (wind data)")
            print("  - /rudder_sail_radio (integrated keyboard control)")
            print("Control commands sent to simulator via /rudder_sail_servo")
            print(
                "Keyboard control: Use arrow keys in curses display to control rudder and sail")
            print("Visualization available via Foxglove Studio at ws://localhost:8765")
            print("\n🔄 Simulation running... Press Ctrl+C to stop and clean up all nodes")

            # Start continuous monitoring with proper cleanup
            try:
                last_status_check = time.time()
                status_check_interval = 2.0  # Check status every 2 seconds for faster failure detection
                
                while not self.shutdown_requested:
                    # Check for shutdown request frequently
                    current_time = time.time()
                    time_since_check = current_time - last_status_check
                    
                    # Sleep in small increments to check shutdown frequently
                    sleep_interval = min(0.5, status_check_interval - time_since_check)
                    if sleep_interval > 0:
                        time.sleep(sleep_interval)
                    
                    # Check if it's time for status check
                    if current_time - last_status_check >= status_check_interval:
                        last_status_check = current_time
                        
                        # Check node status
                        node_status = self._get_node_status()
                        running_nodes = [
                            node for node, status in node_status.items() if "RUNNING" in status]
                        stopped_nodes = [
                            node for node, status in node_status.items() if "STOPPED" in status]

                        if stopped_nodes:
                            # Check if critical nodes are still running
                            critical_running = [
                                n for n in self.critical_nodes if n in running_nodes]
                            critical_stopped = [
                                n for n in self.critical_nodes if n in stopped_nodes]

                            if critical_stopped:
                                # Critical node software failure - terminate simulation immediately
                                print(f"\n{'='*70}")
                                print(f"❌ FATAL ERROR: Critical simulation nodes stopped: {', '.join(critical_stopped)}")
                                print(f"{'='*70}")
                                # Get error messages for stopped nodes
                                fatal_messages = self._get_fatal_messages_for_nodes()
                                for failed_node in critical_stopped:
                                    if failed_node in fatal_messages:
                                        print(f"   {failed_node}: {fatal_messages[failed_node]}")
                                    else:
                                        print(f"   {failed_node}: Node exited (check journal for details)")
                                print(f"{'='*70}\n")
                                print("🛑 Terminating simulation due to critical node software failure...")
                                self.stop()
                                return False
                            else:
                                # Non-critical nodes stopped - log but continue
                                print(
                                    f"⚠️  {len(stopped_nodes)} simulation nodes stopped: {', '.join(stopped_nodes)}")
                                print(
                                    f"✅ Critical simulation nodes operational: {', '.join(critical_running)}")

                            # Show system status
                            if len(running_nodes) >= 2:
                                print(
                                    f"✅ Simulation operational with {len(running_nodes)}/{len(self.expected_nodes + self.special_nodes)} nodes")
                            else:
                                print(
                                    f"⚠️  Low node count: {len(running_nodes)}/{len(self.expected_nodes + self.special_nodes)} nodes running")

            except KeyboardInterrupt:
                print("\n🛑 Stopping simulation and cleaning up all nodes...")
                self.shutdown_requested = True
                self.stop()
                # stop() already waits for all processes and stderr flush
                print("✅ All simulation nodes terminated")
                return True
            except Exception as e:
                import traceback
                error_msg = str(e) if e else "Unknown error"
                print(f"❌ Error in simulation mode: {error_msg}")
                # Print full traceback to help with debugging
                traceback.print_exc()
                self.shutdown_requested = True
                self.stop()
                return False
        else:
            print("❌ Argo simulation mode failed to start")

        return success
    
    def status(self) -> None:
        """Show current status of Argo nodes"""
        # Show checking message and clear it
        print("🔍 Checking Argo status...", end='', flush=True)
        time.sleep(0.1)  # Brief pause to show the message
        print("\r" + " " * 50 + "\r", end='', flush=True)  # Clear the line

        print(
            f"🚢 ARGO STATUS - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 60)
        
        # Check all systemd services status (compact one-line format)
        def check_service_status(service_name):
            """Helper to check if a systemd service is active"""
            try:
                result = subprocess.run(['systemctl', 'is-active', service_name],
                                        capture_output=True, text=True, timeout=2)
                return result.returncode == 0 and result.stdout.strip() == 'active'
            except Exception:
                return False
        
        power_control_running = check_service_status('argo_power_control.service')
        battery_service_running = check_service_status('argo_battery_water.service')
        bno085_service_running = check_service_status('argo_bno085.service')
        health_monitor_running = check_service_status('argo_health_monitor.service')
        service_running = check_service_status('argo_launch_standard.service')
        
        # Display all services on one compact line
        services_status = []
        services_status.append(f"⚡ Power: {'🟢' if power_control_running else '🔴'}")
        services_status.append(f"🔋 Battery: {'🟢' if battery_service_running else '🔴'}")
        services_status.append(f"🧭 BNO085: {'🟢' if bno085_service_running else '🔴'}")
        services_status.append(f"🏥 Health: {'🟢' if health_monitor_running else '🔴'}")
        services_status.append(f"📋 Launch: {'🟢' if service_running else '🔴'}")
        
        print(" | ".join(services_status))
        
        # Get FATAL messages for stopped nodes if service is running
        node_fatal_messages = {}
        if service_running:
            node_fatal_messages = self._get_fatal_messages_for_nodes()
        
        # Individual node status - tabular format
        node_status = self._get_node_status()
        running_count = sum(
            1 for status in node_status.values() if "RUNNING" in status)
        total_count = len(node_status)
        print(f"🤖 ROS NODES: [{running_count}/{total_count}]")
        
        # # Show controller pause state
        # if 'controller.py' in node_status and "RUNNING" in node_status['controller.py']:
        #     # Query current pause state to ensure we have the latest
        #     self._query_controller_pause_state()
        #     pause_status = "⏸️ PAUSED" if self.controller_pause_state else "▶️ RUNNING"
        #     print(f"🎮 CONTROLLER: {pause_status}")
        # else:
        #     print(f"🎮 CONTROLLER: 🔴 STOPPED")
        
        # Only query health status if nodes are running
        health_data = {}
        if running_count > 0:
            print('Querying health status from all nodes via services...',end='',flush=True)
            # Query health status from all nodes via services
            health_data = self._query_node_health_services()
            # Erase 'Querying health status...' and replace with dynamic query output
            sys.stdout.write('\r' + ' ' * 60 + '\r')  # Clear the current line
            print(f"Health Services Queried: {len(health_data)} node{'s' if len(health_data)!=1 else ''} responded")
        else:
            # No nodes running - skip health query
            print("No nodes running - skipping health status query")
        # Display nodes in tabular format
        # print("📋 NODE STATUS TABLE:")
        print("┌" + "─" * 30 + "┬" + "─" * 12 + "┐")
        print("│ " + "NODE NAME".ljust(28) + " │ " + "HEALTH".ljust(10) + " │")
        print("├" + "─" * 30 + "┼" + "─" * 12 + "┤")
        
        stopped_nodes = []
        for node, status in node_status.items():
            # Get health status for this node (prefer service data)
            health_status = self._get_node_health_status(node, health_data)
            
            # Track stopped nodes for error reporting
            if "STOPPED" in status:
                stopped_nodes.append(node)
            
            # Format node name (remove .py extension for display)
            display_name = node.replace('.py', '')
            
            # Pad the display name first, then wrap with color codes for proper table alignment
            padded_display_name = display_name.ljust(28)
            
            # Color node name based on health and running status
            if "STOPPED" in status:
                # Node is not running - gray/dim
                colored_name = f"{Colors.DIM}{padded_display_name}{Colors.RESET}"
            elif health_status == "🟢":
                # Node is running and healthy - dark green
                colored_name = f"{Colors.DARK_GREEN}{padded_display_name}{Colors.RESET}"
            elif health_status == "🔴":
                # Node is running but unhealthy - red
                colored_name = f"{Colors.RED}{padded_display_name}{Colors.RESET}"
            else:
                # Health status unknown - no color
                colored_name = padded_display_name
            
            print(f"│ {colored_name} │ {health_status.ljust(10)} │")
        
        print("└" + "─" * 30 + "┴" + "─" * 12 + "┘")
        
        # Show key error messages for stopped nodes
        if stopped_nodes:
            # print with no newline so that if there are no errors, the next line is not indented
            # use green good symbol if no errors
            try:
                # Get recent error messages from systemd journal
                result = subprocess.run([
                    'journalctl', '-u', 'argo_launch_standard.service', '--since', '5 minutes ago',
                    '--grep', '(FATAL|ERROR|CRITICAL)', '--no-pager'
                ], capture_output=True, text=True, timeout=5)
                
                if result.returncode == 0 and result.stdout:
                    # Parse and show relevant error messages
                    lines = result.stdout.strip().split('\n')
                    recent_errors = []
                    
                    for line in lines:
                        # Check if this line contains errors from any stopped node
                        node_match = False
                        matched_node = None
                        for node in stopped_nodes:
                            node_name = node.replace('.py', '_node')
                            if node_name in line:
                                node_match = True
                                matched_node = node
                                break
                        
                        if node_match:
                            # Extract timestamp and message more robustly
                            if ']: ' in line:
                                parts = line.split(']: ', 1)
                                if len(parts) == 2:
                                    # Get timestamp (first 3 parts: month, day, time)
                                    timestamp_parts = parts[0].split(' ')[0:3]
                                    timestamp = ' '.join(timestamp_parts)
                                    message = parts[1]
                                    recent_errors.append(
                                        f"  {matched_node}: {timestamp} - {message}")
                            else:
                                # Fallback if parsing fails
                                recent_errors.append(
                                    f"  {matched_node}: {line}")
                    
                    if recent_errors:
                        # new line before actual errors
                        print("⚠️  Errors found in systemd journal:")
                        # Group errors by node to ensure we show at least one error per stopped node
                        errors_by_node = {}
                        for error in recent_errors:
                            # Extract node name from error line
                            node_name = None
                            for node in stopped_nodes:
                                if node in error:
                                    node_name = node
                                    break
                            if node_name:
                                if node_name not in errors_by_node:
                                    errors_by_node[node_name] = []
                                errors_by_node[node_name].append(error)
                        
                        # Show errors, prioritizing FATAL messages and ensuring each stopped node is represented
                        display_errors = []
                        
                        # First, collect all FATAL errors
                        for node_errors in errors_by_node.values():
                            fatal_node_errors = [
                                e for e in node_errors if 'FATAL' in e]
                            if fatal_node_errors:
                                # Most recent FATAL per node
                                display_errors.append(fatal_node_errors[-1])
                        
                        # Then add other errors if we have space (max 8 total)
                        for node_errors in errors_by_node.values():
                            other_errors = [
                                e for e in node_errors if 'FATAL' not in e]
                            if other_errors and len(display_errors) < 8:
                                # Most recent other error per node
                                display_errors.append(other_errors[-1])
                        
                        # Sort by timestamp (newest first) and limit to 8 errors
                        display_errors = display_errors[-8:]
                        
                        for error in display_errors:
                            print(error)
                    else:
                        print("  No specific node errors found in recent logs")
                else:
                    print(
                        f"🟢  No systemd journal errors found since {self.journal_since}")
            except Exception as e:
                print(f"  Error retrieving messages: {e}")
        
        # If nothing is running, provide I2C bus health info to help diagnostics
        try:
            launch_stopped = not self._is_launch_running()
            all_nodes_stopped = all(
                "STOPPED" in s for s in node_status.values()) if node_status else True
            if launch_stopped or all_nodes_stopped:
                print("\n🔌 I2C BUS HEALTH (bus 0):")
                self._print_i2c_health(bus=0)
        except Exception as e:
            print(f"⚠️  I2C health check failed: {e}")
        
        # System info
        try:
            # Allow user to skip detailed system info (abort during slow CPU check)
            print("  [Press Enter within 1s to skip system info...]", end='', flush=True)
            if select.select([sys.stdin], [], [], 0.5)[0]:  # 0.5s timeout
                sys.stdin.readline()  # Consume the Enter key
                print("\r" + " " * 60 + "\r", end='', flush=True)  # Clear the line
                print("⏩ Skipped detailed system info")
                print("=" * 60)
                return
            print("\r" + " " * 60 + "\r", end='', flush=True)  # Clear the line
            
            # CPU percentage (this waits 1 second by design)
            cpu_percent = psutil.cpu_percent(interval=0.3)

            # Memory and disk
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage("/")
            free_disk = disk.free / (1024**3)

            # CPU temperature from thermal logs
            cpu_temp = None
            try:
                # Try to find the most recent thermal log file
                import glob
                thermal_logs = sorted(
                    glob.glob('/var/log.hdd/persistent/thermal-*.log'), reverse=True)
                for thermal_log in thermal_logs:
                    if os.path.exists(thermal_log) and os.path.getsize(thermal_log) > 0:
                        with open(thermal_log, 'r') as f:
                            # Read last line
                            lines = f.readlines()
                            if lines:
                                last_line = lines[-1].strip()
                                # Parse: "2025-10-01 06:30:23: GPU:60°C VE:57°C CPU:58°C DDR:58°C"
                                if 'CPU:' in last_line:
                                    cpu_part = last_line.split(
                                        'CPU:')[1].split()[0]
                                    cpu_temp = cpu_part.replace('°C', '')
                        break
            except Exception:
                pass

            # Fallback to reading thermal zone directly if log method failed
            if cpu_temp is None:
                try:
                    with open('/sys/class/thermal/thermal_zone2/temp', 'r') as f:
                        temp_millicelsius = int(f.read().strip())
                        cpu_temp = str(temp_millicelsius // 1000)
                except Exception:
                    cpu_temp = None
            
            # Get node status (needed for battery check)
            node_status = self._get_node_status()

            # Battery and alerts
            # Note: argo_battery_water.py runs as independent service
            battery_summary, critical_alerts, charging_status, usb_power_status, time_to_full_hours, time_to_empty_hours, charging_time_remaining_hours = None, None, None, None, None, None, None
            # Check if argo_battery_water service is running independently
            battery_service_running = False
            try:
                result = subprocess.run(['systemctl', 'is-active', 'argo_battery_water.service'],
                                        capture_output=True, text=True, timeout=2)
                battery_service_running = result.returncode == 0 and result.stdout.strip() == 'active'
            except Exception:
                pass

            if battery_service_running:
                battery_summary, critical_alerts, charging_status, usb_power_status, time_to_full_hours, time_to_empty_hours, charging_time_remaining_hours = self._get_battery_water_status_alerts()
            
            # Use unified formatting function for system info line
            system_info = self._format_status_summary(
                running_count=running_count,
                total_count=total_count,
                cpu_percent=cpu_percent,
                memory_percent=memory.percent,
                free_disk_gb=free_disk,
                disk_percent=disk.percent,
                cpu_temp=cpu_temp,
                battery_summary=battery_summary,
                charging_status=charging_status,
                usb_power_status=usb_power_status,
                time_to_full_hours=time_to_full_hours,
                time_to_empty_hours=time_to_empty_hours
            )
            
            print(system_info)
            
            # Display battery lifetime estimates if available
            if time_to_full_hours is not None:
                if time_to_full_hours < 0.1:
                    print(f"⏱️  Battery: Fully charged (< 6 min to 8.2V)")
                elif time_to_full_hours < 1.0:
                    minutes = int(time_to_full_hours * 60)
                    print(f"⏱️  Time to full charge: {minutes} min")
                else:
                    print(f"⏱️  Time to full charge: {time_to_full_hours:.1f} hours")
            elif time_to_empty_hours is not None:
                if time_to_empty_hours < 0.1:
                    print(f"⚠️  Battery: Nearly depleted (< 6 min remaining)")
                elif time_to_empty_hours < 1.0:
                    minutes = int(time_to_empty_hours * 60)
                    print(f"⏱️  Est. battery lifetime: {minutes} min")
                else:
                    print(f"⏱️  Est. battery lifetime: {time_to_empty_hours:.1f} hours")
            
            # Display MP2672 CHG timer remaining time if AC power is present
            if charging_time_remaining_hours is not None and usb_power_status:
                if charging_time_remaining_hours < 0.1:
                    print(f"⚠️  MP2672 CHG timer: EXPIRED (charging disabled by safety timer)")
                elif charging_time_remaining_hours < 1.0:
                    minutes = int(charging_time_remaining_hours * 60)
                    print(f"⏱️  MP2672 CHG timer: {minutes} min remaining")
                else:
                    print(f"⏱️  MP2672 CHG timer: {charging_time_remaining_hours:.1f} hours remaining")
            
            # Display critical alerts if any
            if critical_alerts:
                print(f"⚠️  CRITICAL ALERTS: {critical_alerts}")

        except Exception as e:
            print(f"📊 SYSTEM: Unable to get system info - {e}")
            import traceback
            traceback.print_exc()
        
        # Update timestamp file to prevent quick timer from running on next terminal startup
        # This ensures that manual status checks are treated the same as quick timer checks
        try:
            home_dir = os.path.expanduser('~')
            last_check_file = os.path.join(home_dir, ".argo_last_check")
            current_time = int(time.time())
            with open(last_check_file, 'w') as f:
                f.write(str(current_time))
        except Exception:
            # Silently fail - this is just for preventing redundant quick timer checks
            pass
        
        print("=" * 60)
    
    def _format_status_summary(self, running_count: int, total_count: int, cpu_percent: float, 
                              memory_percent: float, free_disk_gb: float, disk_percent: float, 
                              cpu_temp: str, battery_summary: str, charging_status: bool, 
                              usb_power_status: bool, time_to_full_hours: float = None, 
                              time_to_empty_hours: float = None, healthy_count: int = 0, 
                              unhealthy_count: int = 0, controller_paused: bool = None) -> str:
        """
        Format a unified single-line status summary.
        
        Args:
            running_count: Number of running nodes
            total_count: Total number of nodes
            cpu_percent: CPU usage percentage
            memory_percent: Memory usage percentage
            free_disk_gb: Free disk space in GB
            disk_percent: Disk usage percentage
            cpu_temp: CPU temperature string
            battery_summary: Battery voltage/percentage string
            charging_status: True/False/None for charging status
            usb_power_status: True/False/None for USB power status
            time_to_full_hours: Hours to full charge (optional)
            time_to_empty_hours: Hours to empty (optional)
            healthy_count: Number of healthy nodes (optional)
            unhealthy_count: Number of unhealthy nodes (optional)
            controller_paused: Controller pause state (optional)
        
        Returns:
            Formatted status line string with 🚢 ARGO: prefix
        """
        # Use sailing boat emoji for both contexts
        prefix = f"🚢 ARGO: [{running_count}/{total_count}]"
        
        # Build status line
        status_line = f"{prefix} | 🖥️ {cpu_percent:.1f}%"
        
        # Add CPU temperature with thermometer emoji
        if cpu_temp:
            status_line += f" | 🌡️ {cpu_temp}°C"
        
        # Add memory
        status_line += f" | Mem {memory_percent:.1f}%"
        
        # Add disk space with disk emoji
        status_line += f" | 💾 {free_disk_gb:.1f}GB ({disk_percent:.1f}% used)"
        
        # Add health summary if we have health data
        if healthy_count > 0 or unhealthy_count > 0:
            status_line += f" | 🏥 {healthy_count}H/{unhealthy_count}U"
        
        # Add controller pause state
        if controller_paused is not None:
            pause_indicator = "⏸️" if controller_paused else "▶️"
            status_line += f" | 🎮 {pause_indicator}"
        
        # Always show battery status (use ??? if unavailable)
        battery_display = battery_summary if battery_summary else "???"
        status_line += f" | 🔋 {battery_display}"
        
        # Always show charging status (use ??? if unavailable)
        if charging_status is True:
            status_line += f" | Charging: 🔌"
        elif charging_status is False:
            status_line += f" | Charging: ❌"
        else:
            status_line += f" | Charging: ???"
        
        # Always show USB power status (use ??? if unavailable)
        if usb_power_status is True:
            status_line += f" | USB: ⚡"
        elif usb_power_status is False:
            status_line += f" | USB: ❌"
        else:
            status_line += f" | USB: ???"
        
        # Add battery lifetime estimate if available
        if time_to_full_hours is not None:
            if time_to_full_hours < 0.1:
                status_line += f" | ⏱️ Full"
            elif time_to_full_hours < 1.0:
                minutes = int(time_to_full_hours * 60)
                status_line += f" | ⏱️ +{minutes}m"
            else:
                status_line += f" | ⏱️ +{time_to_full_hours:.1f}h"
        elif time_to_empty_hours is not None:
            if time_to_empty_hours < 0.1:
                status_line += f" | ⚠️ Empty"
            elif time_to_empty_hours < 1.0:
                minutes = int(time_to_empty_hours * 60)
                status_line += f" | ⏱️ -{minutes}m"
            else:
                status_line += f" | ⏱️ -{time_to_empty_hours:.1f}h"
        
        return status_line
    
    def quick_status(self) -> None:
        """Show condensed one-line status of Argo system (optimized for quick checks)"""
        try:
            # Get node status
            node_status = self._get_node_status()
            running_count = sum(1 for status in node_status.values() if "RUNNING" in status)
            total_count = len(node_status)
            
            # Get CPU usage (quick check with minimal interval)
            cpu_percent = psutil.cpu_percent(interval=0.2)
            
            # Get memory and disk info (quick check)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage("/")
            free_disk_gb = disk.free / (1024**3)
            
            # Get CPU temperature (quick check)
            cpu_temp = None
            try:
                with open('/sys/class/thermal/thermal_zone2/temp', 'r') as f:
                    temp_millicelsius = int(f.read().strip())
                    cpu_temp = str(temp_millicelsius // 1000)
            except KeyboardInterrupt:
                print("🚢 ARGO: Keyboard interrupt detected, exiting...")
                return
            except Exception:
                pass
            
            # Get battery info and power status if available (with retry for robustness)
            battery_summary = None
            charging_status = None
            usb_power_status = None
            time_to_full_hours = None
            time_to_empty_hours = None
            charging_time_remaining_hours = None
            try:
                result = subprocess.run(['systemctl', 'is-active', 'argo_battery_water.service'],
                                        capture_output=True, text=True, timeout=2)
                battery_service_running = result.returncode == 0 and result.stdout.strip() == 'active'
                if battery_service_running:
                    # Try to get battery info, with retry on failure
                    for attempt in range(2):
                        try:
                            battery_summary, _, charging_status, usb_power_status, time_to_full_hours, time_to_empty_hours, charging_time_remaining_hours = self._get_battery_water_status_alerts()
                            if battery_summary:
                                break  # Success
                            time.sleep(0.1)  # Brief delay before retry
                        except Exception:
                            if attempt == 0:
                                time.sleep(0.1)  # Brief delay before retry
                            pass
            except KeyboardInterrupt:
                print("🚢 ARGO: Keyboard interrupt detected, exiting...")
                return
            except Exception:
                pass
            
            # Count healthy nodes
            healthy_count = 0
            unhealthy_count = 0
            for node in node_status.keys():
                health_status = self.node_health_status.get(node)
                if health_status is True:
                    healthy_count += 1
                elif health_status is False:
                    unhealthy_count += 1
            
            # Get controller pause state
            controller_paused = None
            if 'controller.py' in node_status and "RUNNING" in node_status['controller.py']:
                # Query current pause state to ensure we have the latest
                self._query_controller_pause_state()
                controller_paused = self.controller_pause_state
            
            # Use unified formatting function
            status_line = self._format_status_summary(
                running_count=running_count,
                total_count=total_count,
                cpu_percent=cpu_percent,
                memory_percent=memory.percent,
                free_disk_gb=free_disk_gb,
                disk_percent=disk.percent,
                cpu_temp=cpu_temp,
                battery_summary=battery_summary,
                charging_status=charging_status,
                usb_power_status=usb_power_status,
                time_to_full_hours=time_to_full_hours,
                time_to_empty_hours=time_to_empty_hours,
                healthy_count=healthy_count,
                unhealthy_count=unhealthy_count,
                controller_paused=controller_paused
            )
            
            print(status_line, flush=True)
            
            # Warn if MP2672 CHG timer is getting low (when AC power is present)
            if charging_time_remaining_hours is not None and usb_power_status:
                if charging_time_remaining_hours < CHARGING_TIME_WARNING_THRESHOLD_HOURS:
                    if charging_time_remaining_hours < 0.1:
                        print(f"⚠️  WARNING: MP2672 CHG timer has EXPIRED - charging disabled by safety timer!", flush=True)
                    elif charging_time_remaining_hours < 1.0:
                        minutes = int(charging_time_remaining_hours * 60)
                        print(f"⚠️  WARNING: MP2672 CHG timer low: {minutes} min remaining (charging will disable after timer expires)", flush=True)
                    else:
                        print(f"⚠️  WARNING: MP2672 CHG timer low: {charging_time_remaining_hours:.1f} hours remaining (charging will disable after {CHARGING_TIME_WARNING_THRESHOLD_HOURS:.0f}h)", flush=True)
            
            # Update timestamp file to prevent redundant quick timer checks
            try:
                home_dir = os.path.expanduser('~')
                last_check_file = os.path.join(home_dir, ".argo_last_check")
                current_time = int(time.time())
                with open(last_check_file, 'w') as f:
                    f.write(str(current_time))
            except Exception:
                pass
        except KeyboardInterrupt:
            print("🚢 ARGO: Keyboard interrupt detected, exiting...")
            return
        except Exception as e:
            print(f"🚢 ARGO: Status check failed - {e}", flush=True)
    
    def _start_remote_tunnel(self):
        """Start SSH tunnel for remote simulation"""
        if not REMOTE_CONFIG:
            print("❌ Remote config not found")
            return False

        host = REMOTE_CONFIG['remote']['host']
        user = REMOTE_CONFIG['remote']['user']
        local_port = REMOTE_CONFIG['network']['local_port']
        remote_port = REMOTE_CONFIG['network']['remote_port']

        print(f"🔗 Creating SSH tunnel to {user}@{host}...")
        cmd = [
            'ssh', '-N', '-L', f'{local_port}:localhost:{remote_port}',
            f'{user}@{host}'
        ]
        self.remote_tunnel_proc = subprocess.Popen(cmd)
        time.sleep(2)  # Wait for tunnel to establish

        if self.remote_tunnel_proc.poll() is None:
            print(
                f"✅ SSH tunnel established (PID: {self.remote_tunnel_proc.pid})")
            return True
        else:
            print("❌ SSH tunnel failed to start")
            return False

    def _start_remote_simulator(self):
        """Start the simulator on the remote machine"""
        if not REMOTE_CONFIG:
            print("❌ Remote config not found")
            return False

        host = REMOTE_CONFIG['remote']['host']
        user = REMOTE_CONFIG['remote']['user']
        argo_dir = REMOTE_CONFIG['remote']['argo_dir']
        ros_domain_id = REMOTE_CONFIG['ros2']['domain_id']

        print(f"🚀 Launching remote simulator on {user}@{host}...")
        remote_cmd = (
            f"cd {argo_dir} && "
            f"source /opt/ros/humble/setup.bash && "
            f"export ROS_DOMAIN_ID={ros_domain_id} && "
            f"python3 nodes/argo_unified_simulator_bridge.py --mode local"
        )
        cmd = ['ssh', f'{user}@{host}', remote_cmd]

        self.remote_simulator_proc = subprocess.Popen(cmd)
        if self.remote_simulator_proc.poll() is None:
            print(
                f"✅ Remote simulator launched (PID: {self.remote_simulator_proc.pid})")
            return True
        else:
            print("❌ Failed to launch remote simulator")
            return False

    def _stop_remote_processes(self):
        """Stop remote simulator and SSH tunnel"""
        if self.remote_simulator_proc:
            print("🛑 Stopping remote simulator...")
            # Need to kill the process on the remote machine
            if REMOTE_CONFIG:
                host = REMOTE_CONFIG['remote']['host']
                user = REMOTE_CONFIG['remote']['user']
                kill_cmd = "pkill -f 'argo_unified_simulator_bridge.py'"
                subprocess.run(['ssh', f'{user}@{host}', kill_cmd])
            self.remote_simulator_proc.terminate()
            self.remote_simulator_proc.wait(timeout=5)
            self.remote_simulator_proc = None

        if self.remote_tunnel_proc:
            print("🛑 Stopping SSH tunnel...")
            self.remote_tunnel_proc.terminate()
            self.remote_tunnel_proc.wait(timeout=5)
            self.remote_tunnel_proc = None

    def _get_i2c_addresses(self, bus: int = 0) -> List[int]:
        """Run i2cdetect and parse detected device addresses on the given bus."""
        try:
            proc = subprocess.run(
                ["i2cdetect", "-y", str(bus)],
                capture_output=True,
                text=True,
                timeout=1,
                check=True,
            )
            output = proc.stdout
        except FileNotFoundError:
            raise RuntimeError(
                "i2cdetect not found. Install i2c-tools or ensure it's in PATH.")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                f"i2cdetect failed (exit {e.returncode}). stderr: {e.stderr}")
        except subprocess.TimeoutExpired:
            raise TimeoutError("i2cdetect timed out after 1s")

        detected: List[int] = []
        for line in output.splitlines():
            line = line.strip()
            if not line:
                continue
            # lines with rows look like: "10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --"
            if ":" not in line:
                continue
            try:
                _, cells_str = line.split(":", 1)
            except ValueError:
                continue
            cells = [c.strip() for c in cells_str.strip().split()]
            for cell in cells:
                # Valid device entries are two-hex-digit addresses; '--' is empty; 'UU' is in-use by driver
                if cell == "--" or cell == "UU":
                    continue
                # Accept hex like '34', '69'
                if len(cell) == 2:
                    try:
                        addr = int(cell, 16)
                        detected.append(addr)
                    except ValueError:
                        continue
        return sorted(set(detected))
    
    def _get_fatal_messages_for_nodes(self) -> Dict[str, str]:
        """Get the most recent FATAL message or Python exception for each node from systemd journal"""
        fatal_messages = {}
        
        try:
            # Search for FATAL messages and Python exceptions (ImportError, ModuleNotFoundError, etc.)
            # Use extended-regex to search for multiple patterns
            result = subprocess.run([
                'journalctl', '-u', 'argo_launch_standard.service', '--since', self.journal_since,
                '--grep', 'FATAL|ImportError|ModuleNotFoundError|Exception:', '--no-pager', '-o', 'short-precise'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and result.stdout:
                lines = result.stdout.strip().split('\n')
                
                # Parse lines to extract node-specific error messages
                # Process in reverse order to get the most recent message for each node
                for line in reversed(lines):
                    # Check for Python exceptions first (most common startup failures)
                    if 'ImportError:' in line or 'ModuleNotFoundError:' in line:
                        # Try to identify which node this error belongs to
                        for node in self.expected_nodes:
                            if node in fatal_messages:
                                continue  # Already have a message for this node
                            
                            # Check if this line mentions the node file
                            if f'/{node}' in line or f'nodes/{node}' in line:
                                # Extract the error message
                                if 'ImportError:' in line:
                                    error_part = line.split('ImportError:', 1)[1].strip()
                                elif 'ModuleNotFoundError:' in line:
                                    error_part = line.split('ModuleNotFoundError:', 1)[1].strip()
                                
                                # Clean up and truncate message
                                if len(error_part) > 80:
                                    error_part = error_part[:77] + "..."
                                
                                fatal_messages[node] = error_part
                                break
                    
                    # Check for FATAL messages (ROS2 logger)
                    elif 'FATAL' in line:
                        # Try to identify which node this FATAL message belongs to
                        for node in self.expected_nodes:
                            node_name = node.replace('.py', '')
                            # Look for node name patterns in the log line
                            if (f'{node_name}_node' in line or 
                                f'/{node_name}' in line or 
                                f'{node_name}.py' in line or
                                f'[{node_name}]' in line or
                                f'anemometer' in line.lower() and node_name == 'anem'):
                                
                                # Skip if we already have a message for this node (most recent)
                                if node in fatal_messages:
                                    continue
                                
                                # Extract the FATAL message part
                                if 'FATAL:' in line:
                                    # Find the FATAL message and extract a concise version
                                    fatal_part = line.split('FATAL:', 1)
                                    if len(fatal_part) > 1:
                                        message = fatal_part[1].strip()
                                        # Clean up the message - take first meaningful sentence
                                        if '.' in message:
                                            message = message.split('.')[0].strip()
                                        # Limit message length for display
                                        if len(message) > 80:
                                            message = message[:77] + "..."
                                        
                                        # Store the most recent FATAL for this node
                                        fatal_messages[node] = f"FATAL: {message}"
                                elif 'fatal(' in line.lower():
                                    # Handle ROS2 logger format: .fatal("message")
                                    import re
                                    match = re.search(
                                        r'fatal\("([^"]*)"', line, re.IGNORECASE)
                                    if match:
                                        message = match.group(1)
                                        if len(message) > 80:
                                            message = message[:77] + "..."
                                        fatal_messages[node] = f"FATAL: {message}"
                                break
                
        except Exception as e:
            # Silently fail - this is just for enhanced display
            pass
            
        return fatal_messages

    def _call_trigger_service(self, service_name: str, timeout_sec: float = 1.0, debug: bool = False) -> tuple[bool, str]:
        """
        Centralized function for calling ROS2 Trigger services.

        Args:
            service_name: Name of the service (e.g., '/toggle_pause', '/battery_status')
            timeout_sec: Timeout for service call
            debug: Enable debug output

        Returns:
            tuple: (success: bool, message: str)
        """
        try:
            if debug:
                print(f"🔧 DEBUG: Calling Trigger service: {service_name}")

            # Check if ROS2 node is available
            if not self.ros2_node:
                error_msg = "ROS2 node not available"
                if debug:
                    print(f"🔧 DEBUG: {error_msg}")
                return False, error_msg

            # Create a service client
            service_client = self.ros2_node.create_client(
                Trigger, service_name)

            # Wait for the service to be available
            # Use the same timeout as the service call itself for consistency
            if not service_client.wait_for_service(timeout_sec=timeout_sec):
                error_msg = f"Service {service_name} not available"
                if debug:
                    print(f"🔧 DEBUG: {error_msg}")
                return False, error_msg

            if debug:
                print(
                    f"🔧 DEBUG: Service {service_name} is available, making request...")

            # Create the request
            request = Trigger.Request()

            # Call the service
            future = service_client.call_async(request)

            # Wait for the response with timeout
            rclpy.spin_until_future_complete(
                self.ros2_node, future, timeout_sec=timeout_sec)

            if future.done():
                response = future.result()

                if debug:
                    print(
                        f"🔧 DEBUG: Service {service_name} response: success={response.success}, message='{response.message}'")

                return response.success, response.message
            else:
                error_msg = f"Service {service_name} call timed out after {timeout_sec}s"
                if debug:
                    print(f"🔧 DEBUG: {error_msg}")
                return False, error_msg
            
        except Exception as e:
            error_msg = f"Error calling service {service_name}: {e}"
            if debug:
                print(f"🔧 DEBUG: {error_msg}")
                import traceback
                traceback.print_exc()
            return False, error_msg

    def toggle_pause_nodes(self, debug: bool = False) -> bool:
        """Toggle pause state of controller node via ROS2 service call."""
        self._ensure_ros2_node()
        print("🔄 Toggling controller pause state...")

        # Toggle controller pause state
        new_pause_state = not self.controller_pause_state
        success, message = self._call_controller_pause_service(new_pause_state)

        if success:
            print(f"✅ Controller {'paused' if new_pause_state else 'unpaused'} successfully")
            return True
        else:
            print(f"❌ Toggle pause failed: {message}")
            return False

    def _get_battery_water_status_alerts(self) -> tuple[Optional[str], Optional[str], Optional[bool], Optional[bool], Optional[float], Optional[float], Optional[float]]:
        """Get battery info, alerts, charging status, USB power status, lifetime estimates, and charging time remaining using the battery Trigger service client"""
        try:
            # Ensure ROS2 node is initialized (required for service calls)
            self._ensure_ros2_node()
            
            # Use ROS2 service client if available, otherwise fallback to subprocess
            if self.battery_service_client and ROS2_AVAILABLE:
                battery_data = self._call_battery_service_client()
            else:
                battery_data = self._call_battery_service_subprocess()

            if battery_data:
                # Extract battery summary and alerts
                battery_summary = battery_data.get('battery_summary')
                critical_alerts = battery_data.get('critical_alerts')
                # Extract charging and USB power status from raw_data
                raw_data = battery_data.get('raw_data', {})
                charging_status = raw_data.get('charging_status')
                usb_power_status = raw_data.get('ac_power_present')
                time_to_full_hours = raw_data.get('time_to_full_hours')
                time_to_empty_hours = raw_data.get('time_to_empty_hours')
                charging_time_remaining_hours = raw_data.get('charging_time_remaining_hours')
                return battery_summary, critical_alerts, charging_status, usb_power_status, time_to_full_hours, time_to_empty_hours, charging_time_remaining_hours
            else:
                return None, None, None, None, None, None, None

        except Exception as e:
            print(f"    Error getting battery and alerts: {e}")
            return None, None, None, None, None, None, None

    def _call_battery_service_client(self) -> Optional[Dict[str, Any]]:
        """Call battery service using centralized Trigger service call"""
        try:
            # Use centralized Trigger service call with longer timeout
            # Battery service may be slow to respond during sensor re-initialization
            success, message = self._call_trigger_service(
                '/battery_status', timeout_sec=5.0, debug=False)

            if success:
                # Parse JSON from response message
                return json.loads(message)
            else:
                return None

        except Exception as e:
            return None

    def _call_battery_service_subprocess(self) -> Optional[Dict[str, Any]]:
        """Fallback: Call battery service using subprocess ros2 service call"""
        try:
            # Ensure ROS2 node is initialized (may have been called without it)
            self._ensure_ros2_node()
            
            # Use centralized Trigger service call with longer timeout
            # Battery service may be slow to respond during sensor re-initialization
            success, message = self._call_trigger_service(
                '/battery_status', timeout_sec=5.0, debug=False)

            if success:
                # Parse JSON from response message
                return json.loads(message)
            else:
                return None

        except Exception as e:
            return None

    def _print_i2c_health(self, bus: int = 0) -> None:
        """Print a summary of I2C device presence vs expected sensors."""
        try:
            detected = self._get_i2c_addresses(bus)
        except TimeoutError as e:
            print(f"  I2C bus {bus} is malfunctioning ({e})")
            return
        except Exception as e:
            print(f"  I2C scan failed: {e}")
            return
        hex_list = ", ".join(
            [f"0x{a:02x}" for a in detected]) if detected else "<none>"
        print(f"  Detected addresses: {hex_list}")

        # Critical warning if no I2C devices detected at all
        if not detected:
            print(f"  ⚠️  CRITICAL: I2C bus {bus} is malfunctioning - NO devices detected!")
            print(f"  ⚠️  This prevents battery voltage monitoring (ADC at 0x34)")
            print(f"  ⚠️  Critical battery halt protection is DISABLED without battery monitoring!")
            print(f"  ⚠️  Check I2C bus wiring and dtoverlay configuration (pi-i2c0)")
            return

        expected_map = {
            "anem": [0x21, 0x22, 0x23],
            "argo_battery_water": [0x34, 0x44],  # 0x34: ADC, 0x44: humidity
            "bno085": [0x4a],  # BNO085 IMU at 0x4a (replaces old IMU at 0x69)
        }

        for sensor, addrs in expected_map.items():
            present = [a for a in addrs if a in detected]
            missing = [a for a in addrs if a not in detected]
            if present and not missing:
                print(
                    f"  {sensor}: 🟢 present ({', '.join([f'0x{x:02x}' for x in present])})")
            elif present and missing:
                print(
                    f"  {sensor}: 🟡 partially present (found {', '.join([f'0x{x:02x}' for x in present])}; missing {', '.join([f'0x{x:02x}' for x in missing])})"
                )
            else:
                print(
                    f"  {sensor}: 🔴 not present (expected {', '.join([f'0x{x:02x}' for x in addrs])})")

    def start(self, debug=False) -> bool:
        """Start the Argo launch process"""
        if self._is_launch_running():
            print("🚢 Argo is already running.")
            # If nodes are running but service is not, start monitoring
            if not self._is_service_active():
                 print("   Monitoring existing nodes...")
                 self.continuous(debug=debug, monitor_only=True)
            return True
            
        # --- REFACTORED: Set node lists for physical robot mode ---
        # This ensures that status checks and health monitoring use the correct
        # node list defined in the 'physical_robot' group in argo_nodes.yaml
        self.expected_nodes = self.physical_robot_nodes
        self.special_nodes = self.physical_robot_special_nodes
        self.all_expected_nodes = self.expected_nodes + self.special_nodes

        # Check if the service is running - if so, delegate to service
        if self._is_service_active():
            print("🚀 Launch service is running - it will manage the nodes.")
            return True
        
        print("🚀 Starting Argo ROS2 nodes...")
        
        if self.shutdown_requested:
            print("⚠️  Shutdown requested before node launch - aborting")
            return False
        
        try:
            # Start the nodes directly
            self._launch_nodes_directly()
            
            # Check again after launch
            if self.shutdown_requested:
                print("⚠️  Shutdown requested during launch - stopping nodes")
                self.stop()
                return False
            
            # Wait and check for actual node startup with continuous feedback
            print("⏳ Waiting for nodes to start...")
            
            timeout = 30  # 30 second timeout
            check_interval = .3  # Check every .3 second
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # Check for shutdown request during launch
                if self.shutdown_requested:
                    print("⚠️  Shutdown requested during node startup - stopping nodes")
                    self.stop()
                    return False
                
                if not self._is_launch_running():
                    print("❌ Launch process died during startup")
                    
                    # Log that process died (output is now in systemd journal)
                    if self.process and self.process.poll() is not None:
                        print(
                            "💀 Process died - check systemd journal for detailed output")
                    
                    return False
                
                # Check which nodes are running
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]
                
                # Show progress periodically (journal-friendly)
                elapsed = time.time() - start_time
                if elapsed > 0 and int(elapsed * 10) % 10 == 0:  # Print every second
                    if running_nodes and stopped_nodes:
                        print(
                            f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                        print(f"   Not started: {', '.join(stopped_nodes)}")
                    elif running_nodes:
                        print(
                            f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                    else:
                        print(
                            f"⏳ Waiting for nodes to start... (no nodes detected yet)")
                
                # If all expected nodes are running, wait for stabilization with monitoring
                if len(running_nodes) == len(self.all_expected_nodes):
                    print(
                        f"\n✅ All {len(running_nodes)} nodes detected: {', '.join(running_nodes)}")
                    print(
                        f"⏳ Monitoring nodes during {self.stabilization_wait}s stabilization period...")
                    
                    # Monitor nodes during stabilization period
                    stabilization_start = time.time()
                    stabilization_check_interval = 1.0  # Check every second during stabilization
                    
                    while time.time() - stabilization_start < self.stabilization_wait:
                        # Check for shutdown request during stabilization
                        if self.shutdown_requested:
                            print("⚠️  Shutdown requested during stabilization - stopping nodes")
                            self.stop()
                            return False
                        
                        # Check for node failures during stabilization
                        current_status = self._get_node_status()
                        current_running = [
                            node for node, status in current_status.items() if "RUNNING" in status]
                        current_stopped = [
                            node for node, status in current_status.items() if "STOPPED" in status]
                        
                        if current_stopped:
                            # Node(s) failed during stabilization
                            print(
                                f"\n⚠️  Node failure detected during stabilization: {', '.join(current_stopped)}")
                            
                            # Get error messages for failed nodes
                            fatal_messages = self._get_fatal_messages_for_nodes()
                            for failed_node in current_stopped:
                                if failed_node in fatal_messages:
                                    print(
                                        f"   {failed_node}: {fatal_messages[failed_node]}")
                            
                            # Break out of stabilization to handle the failure
                            break
                        
                        # Show progress during stabilization (journal-friendly)
                        remaining_time = self.stabilization_wait - \
                            (time.time() - stabilization_start)
                        # Only print every 2 seconds
                        if remaining_time > 0 and int(remaining_time) % 2 == 0:
                            print(
                                f"⏳ Stabilizing... {remaining_time:.1f}s remaining ({len(current_running)}/{len(self.all_expected_nodes)} nodes)")
                        
                        time.sleep(stabilization_check_interval)
                    
                    # Final check after stabilization period
                    final_node_status = self._get_node_status()
                    final_running_nodes = [
                        node for node, status in final_node_status.items() if "RUNNING" in status]
                    
                    if len(final_running_nodes) == len(self.all_expected_nodes):
                        print(f"✅ Argo launch process started successfully")
                        print(
                            f"✅ All {len(final_running_nodes)} nodes running and stable: {', '.join(final_running_nodes)}")
                        break  # Continue to monitoring phase
                    else:
                        failed_nodes = [
                            node for node in self.all_expected_nodes if node not in final_running_nodes]
                        print(f"⚠️  Some nodes failed during stabilization period")
                        print(
                            f"   Still running: {', '.join(final_running_nodes)}")
                        print(f"   Failed nodes: {', '.join(failed_nodes)}")
                        
                        # Check if we have critical nodes running
                        critical_running = [
                            node for node in self.critical_nodes if node in final_running_nodes]
                        
                        if len(critical_running) == len(self.critical_nodes):
                            print(
                                f"✅ Critical nodes operational: {', '.join(critical_running)}")
                            print(
                                f"✅ Argo will continue operating with {len(final_running_nodes)}/{len(self.all_expected_nodes)} nodes")
                            break  # Continue to monitoring phase
                        elif len(final_running_nodes) >= 3:  # At least 3 nodes running
                            print(
                                f"✅ Sufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(
                                f"✅ Argo will continue operating with available sensors")
                            break  # Continue to monitoring phase
                        else:
                            print(
                                f"❌ Insufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(
                                f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in final_running_nodes])}")
                            return False
                
                time.sleep(check_interval)
            
            # Check if we timed out
            if time.time() - start_time >= timeout:
                print(f"\n⚠️  Timeout reached after {timeout}s")
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]
                
                if running_nodes:
                    failed_nodes = [
                        node for node in self.all_expected_nodes if node not in running_nodes]
                    print(
                        f"✅ {len(running_nodes)} nodes running: {', '.join(running_nodes)}")
                    if failed_nodes:
                        print(
                            f"⚠️  {len(failed_nodes)} nodes not started: {', '.join(failed_nodes)}")
                    
                    # Check if we have critical nodes running
                    critical_running = [
                        node for node in self.critical_nodes if node in running_nodes]
                    
                    if len(critical_running) == len(self.critical_nodes):
                        print(
                            f"✅ Critical nodes operational: {', '.join(critical_running)}")
                        print(
                            f"✅ Argo will continue operating with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                    elif len(running_nodes) >= 3:  # At least 3 nodes running
                        print(
                            f"✅ Sufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                        print(
                            f"✅ Argo will continue operating with available sensors")
                    else:
                        print(
                            f"❌ Insufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                        print(
                            f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in running_nodes])}")
                        return False
                else:
                    print(f"❌ No nodes running after timeout")
                    return False
                
        except Exception as e:
            print(f"❌ Error starting Argo: {e}")
            return False
        
        # Create ROS2 services for lifecycle management
        self._create_lifecycle_services()
        
        print("🔄 Starting continuous monitoring with ROS2 integration...")
        print("   Press Ctrl+C to stop")
        print("   NOTE: Node failures will be logged but NOT restarted for debugging")
        print("   ROS2 services available for remote control")
        
        # Publish initial status
        self._publish_status_update("Argo lifecycle manager running")
        
        try:
            last_check_time = time.time()
            check_interval = 300  # Check every 5 minutes (much less frequent)
            
            while not self.shutdown_requested and (rclpy.ok() if self.ros2_node else True):
                # Check shutdown flag FIRST before any operations
                if self.shutdown_requested:
                    break
                
                # Spin ROS2 node to process service requests (if available)
                if self.ros2_node:
                    try:
                        rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
                    except Exception:
                        # Context may be shutting down
                        if self.shutdown_requested:
                            break
                else:
                    time.sleep(0.1)
                
                # Periodic status check
                current_time = time.time()
                if current_time - last_check_time >= check_interval:
                    last_check_time = current_time
                    
                    # Check node status
                    node_status = self._get_node_status()
                    running_nodes = [
                        node for node, status in node_status.items() if "RUNNING" in status]
                    stopped_nodes = [
                        node for node, status in node_status.items() if "STOPPED" in status]
                    
                    # Publish status update
                    status_msg = f"Running: {len(running_nodes)}/{len(self.all_expected_nodes)}"
                    self._publish_status_update(status_msg)
                    
                    if stopped_nodes:
                        # Log stopped nodes but do NOT restart them
                        print(
                            f"⚠️  {len(stopped_nodes)} nodes stopped: {', '.join(stopped_nodes)}")
                        
                        # Check if critical nodes are still running
                        critical_running = [
                            n for n in self.critical_nodes if n in running_nodes]
                        critical_stopped = [
                            n for n in self.critical_nodes if n in stopped_nodes]
                        
                        if critical_stopped:
                            print(
                                f"❌ CRITICAL NODES STOPPED: {', '.join(critical_stopped)}")
                            print(
                                f"   System will continue with remaining nodes for debugging")
                            print(f"   Check systemd journal for error details")
                            self._publish_status_update(f"CRITICAL: {', '.join(critical_stopped)} stopped")
                        else:
                            print(
                                f"✅ Critical nodes operational: {', '.join(critical_running)}")
                        
                        # Show system status
                        if len(running_nodes) >= 3:
                            print(
                                f"✅ System operational with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                        else:
                            print(
                                f"⚠️  Low node count: {len(running_nodes)}/{len(self.all_expected_nodes)} nodes running")
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping continuous monitoring...")
            self.shutdown_requested = True
        except Exception as e:
            print(f"❌ Error in continuous mode: {e}")
            self._publish_status_update(f"Error: {e}")
            self.shutdown_requested = True
        finally:
            # Publish final status before cleanup
            self._publish_status_update("Argo lifecycle manager stopping")
            
            # Stop all nodes first only if they are actually running
            if self._is_launch_running():
                self.stop()
                # stop() already waits for all processes and stderr flush, no additional wait needed
            
            # Clean up ROS2 last
            self._cleanup_ros2()
            
            # Ensure we don't relaunch after shutdown
            self.shutdown_requested = True
            
            # Explicitly exit if shutdown was requested
            if self.shutdown_requested:
                print("✅ Lifecycle manager shutdown complete - exiting")
                sys.stdout.flush()
                sys.stderr.flush()
                sys.exit(0)
        
        return True  # Always return True for clean exit
    
    def _is_service_active(self) -> bool:
        """Check if the service is running"""
        try:
            result = subprocess.run(['systemctl', 'is-active', 'argo_launch_standard.service'],
                                    capture_output=True, text=True, timeout=2)
            return result.returncode == 0 and result.stdout.strip() == 'active'
        except Exception:
            return False


def _final_stderr_flush():
    """Final flush of stderr before process exits to catch any straggler ROS2 shutdown messages"""
    # Wait 2 seconds for all ROS2 child processes to finish writing shutdown messages
    time.sleep(2.0)
    sys.stdout.flush()
    sys.stderr.flush()

def main():
    # Register final stderr flush to run when process exits
    atexit.register(_final_stderr_flush)
    
    parser = argparse.ArgumentParser(
        description='Argo Status and Simulation Manager - Check status and manage simulations for the Argo sailboat.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
COMMANDS:
  status           Show comprehensive status of Argo system. This is the default command.
                   - Displays service status, node health, and system diagnostics.
                   - Extracts and displays recent error messages from systemd journal.

  quick_status     Show condensed one-line status, optimized for quick checks.
                   - Ideal for shell prompts and frequent checks.

  simulate_local   Start Argo in local simulation mode.
                   - Runs sailboat simulator directly on the Orange Pi.
                   - Excludes conflicting hardware nodes.

  simulate_remote  Start Argo in remote simulation mode.
                   - Connects to a simulator running on a remote machine.

  help             Show this help message and exit.

LIFECYCLE MANAGEMENT:
  The lifecycle of Argo nodes is now managed by the 'argo_launch_standard.service' 
  and controlled via the following aliases:
    al - Start Argo nodes
    aq - Stop Argo nodes
    ars - Restart Argo nodes

EXAMPLES:
  # Show detailed system status
  python3 argo_lifecycle_manager.py status
  argo_lifecycle_manager.py

  # Show quick one-line status
  python3 argo_lifecycle_manager.py quick_status

  # Start local simulation
  python3 argo_lifecycle_manager.py simulate_local
        """)

    parser.add_argument('command',
                        choices=['status', 'quick_status', 'simulate_local', 'simulate_remote', 'help'],
                        nargs='?',  # Make command optional
                        default='status', # Default to 'status'
                        help='Command to execute (see detailed descriptions below)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug output for troubleshooting')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress initialization messages (useful for quick_status)')
    parser.add_argument('--toggle_pause', action='store_true',
                        help='Toggle controller pause state (pauses autonomous navigation)')
    parser.add_argument('--force-mock', action='store_true',
                        help='Force use of mock simulator even if real simulator (sailboat-playground) is available (only for simulate_local)')
    parser.add_argument('--debug-nodes', action='store_true',
                        help='Launch Python node subprocesses under debugpy so IDE breakpoints trigger')
    parser.add_argument('--debug-node-port-base', type=int, default=5678,
                        help='Starting TCP port for debugpy listeners when --debug-nodes is enabled (increments per node)')
    parser.add_argument('--debug-node-wait', action='store_true',
                        help='If set with --debug-nodes, each node waits for debugger to attach before running')
    
    # Enable bash completion for command-line arguments
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    
    # Handle help command first (no need to initialize manager)
    if args.command == 'help':
        parser.print_help()
        sys.exit(0)
    
    # Validate that either a command or --toggle_pause is provided
    if not args.command and not args.toggle_pause:
        # Default to status if no command is given
        args.command = 'status'
    
    manager = ArgoLifecycleManager(
        quiet=args.quiet,
        debug_nodes=args.debug_nodes,
        debug_node_port_base=args.debug_node_port_base,
        debug_node_wait=args.debug_node_wait
    )
    if args.debug:
        print("🔧 DEBUG: Debug mode enabled")
    
    try:
        # Execute the command first (if provided)
        if args.command == 'status':
            manager.status()
            manager._cleanup_ros2()
        elif args.command == 'quick_status':
            manager.quick_status()
            manager._cleanup_ros2()
        elif args.command == 'simulate_local':
            success = manager.simulate_local(force_mock=args.force_mock, debug=args.debug)
            # simulate handles its own cleanup
            sys.exit(0 if success else 1)
        elif args.command == 'simulate_remote':
            success = manager.simulate_remote()
            # simulate handles its own cleanup
            sys.exit(0 if success else 1)

        # Handle --toggle_pause option (after command execution or standalone)
        if args.toggle_pause:
            success = manager.toggle_pause_nodes(debug=args.debug)
            manager._cleanup_ros2()
            sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        # Ctrl+C pressed - ensure all nodes are stopped
        print("\n🛑 Keyboard interrupt detected, stopping all nodes...")
        try:
            manager.stop()
        except:
            pass
        try:
            manager._cleanup_ros2()
        except:
            pass
        sys.exit(0)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        manager._cleanup_ros2()
        sys.exit(1)


if __name__ == '__main__':
    main()
