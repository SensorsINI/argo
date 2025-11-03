#!/usr/bin/env python3
"""
Argo Health Monitor Node
========================

ROS2 node that continuously monitors the health of all Argo nodes.
Publishes health status on topics and provides services for status queries.

This provides the continuous health monitoring that was missing from
the custom lifecycle manager.
"""

import os
import sys
import time
import yaml
import subprocess
from typing import Dict, Optional
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class ArgoHealthMonitor(Node):
    """Monitor health status of all Argo ROS2 nodes"""
    
    def __init__(self):
        super().__init__('argo_health_monitor')
        
        # Load node configuration
        argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(argo_dir, 'launch', 'argo_nodes.yaml')
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Track node health status
        # Maps: executable filename (e.g., 'gps.py') -> {'healthy': bool/None, 'last_seen': float, 'pid': int, 'ros2_node_name': str}
        self.node_health = {}  # Keyed by .py filename for compatibility with lifecycle manager
        self.node_configs = {}  # Keyed by ROS2 node name from YAML
        
        # Map executable filenames to ROS2 node names and health topics
        # This builds a mapping: 'nodes/gps.py' -> ('gps_node', '/gps_node_health')
        self.executable_to_health = {}
        
        # Health topic subscribers - track actual health from ArgoBaseNode topics
        self.health_subscribers = {}
        
        # Fast polling during startup (first 60 seconds)
        self.startup_time = time.time()
        self.startup_duration = 60.0  # Fast polling for first 60 seconds
        self.fast_poll_period = 5.0   # 5 seconds during startup
        self.normal_poll_period = 30.0  # 30 seconds after startup
        self.using_fast_poll = True
        
        # Load all node configs (normal + simulation) and build mappings
        for node_cfg in self.config.get('nodes', []):
            self.node_configs[node_cfg['name']] = node_cfg
        for node_cfg in self.config.get('simulation_nodes', []):
            self.node_configs[node_cfg['name']] = node_cfg
        
        # Build executable -> health topic mapping
        self._build_executable_mapping()
        
        # Subscribe to health topics for nodes that publish them
        self._subscribe_to_health_topics()
    
    def _build_executable_mapping(self):
        """Build mapping from executable filenames to ROS2 node names and health topics
        
        Reads health_topic from YAML config. If not specified, generates default
        based on node name pattern: /{node_name}_health
        """
        import os
        
        # Build mapping from YAML configs
        for node_cfg in list(self.node_configs.values()):
            ros2_node_name = node_cfg.get('name', '')
            executable = node_cfg.get('executable', '')
            filename = os.path.basename(executable)
            
            # Get health topic from YAML config, or generate default
            health_topic = node_cfg.get('health_topic')
            if not health_topic:
                # Default pattern: /{node_name}_health (ArgoBaseNode convention)
                health_topic = f"/{ros2_node_name}_health"
            
            # Store mapping: both full path and filename as keys
            self.executable_to_health[executable] = (ros2_node_name, health_topic)
            self.executable_to_health[filename] = (ros2_node_name, health_topic)
        
        # Publishers
        self.health_pub = self.create_publisher(
            String, '/argo/health/status', 10)
        
        # Services
        self.status_service = self.create_service(
            Trigger, '/argo/health/status', self.status_callback)
        
        # Timer for periodic health checks (fallback for nodes without health topics)
        # Start with fast polling (5s) during startup
        self.timer = self.create_timer(self.fast_poll_period, self.check_node_health)
        
        # Timer for status publishing
        self.status_timer = self.create_timer(self.fast_poll_period, self.publish_status)
        
        # Timer to check if we should transition from fast to normal polling
        self.poll_transition_timer = self.create_timer(10.0, self._check_poll_transition)
        
        # Run initial health check immediately to populate entries for all nodes
        self.check_node_health()
        
        self.get_logger().info("Argo Health Monitor started")
        self.get_logger().info(f"Monitoring {len(self.node_configs)} nodes")
        self.get_logger().info(f"Subscribed to {len(self.health_subscribers)} health topics")
        self.get_logger().info(f"⚡ Fast polling enabled (every {self.fast_poll_period}s) for first {self.startup_duration}s")
    
    def _subscribe_to_health_topics(self):
        """Subscribe to health topics published by ArgoBaseNode nodes"""
        import os
        for node_cfg in self.node_configs.values():
            executable = node_cfg.get('executable', '')
            filename = os.path.basename(executable)  # e.g., 'gps.py'
            
            # Find health topic mapping for this executable
            if executable in self.executable_to_health or filename in self.executable_to_health:
                mapping_key = executable if executable in self.executable_to_health else filename
                ros2_node_name, health_topic = self.executable_to_health[mapping_key]
                
                try:
                    # Create callback that captures the filename (key for health dict)
                    def create_health_callback(exec_filename):
                        def health_callback(msg):
                            # msg.data is bool: True = healthy, False = unhealthy
                            if exec_filename not in self.node_health:
                                self.node_health[exec_filename] = {
                                    'healthy': None,
                                    'last_seen': 0.0,
                                    'pid': None,
                                    'ros2_node_name': ros2_node_name
                                }
                            self.node_health[exec_filename]['healthy'] = msg.data
                            self.node_health[exec_filename]['last_seen'] = time.time()
                        return health_callback
                    
                    callback = create_health_callback(filename)
                    subscriber = self.create_subscription(
                        Bool, health_topic, callback, 10)
                    self.health_subscribers[filename] = subscriber
                    
                    # Initialize health status as unknown
                    if filename not in self.node_health:
                        self.node_health[filename] = {
                            'healthy': None,
                            'last_seen': 0.0,
                            'pid': None,
                            'ros2_node_name': ros2_node_name
                        }
                    
                except Exception as e:
                    self.get_logger().debug(f"Could not subscribe to {health_topic}: {e}")
    
    def check_node_health(self):
        """Check health of all configured nodes (fallback for nodes without health topics)
        
        This is a fallback method that only updates nodes that don't have health topic subscriptions.
        Nodes with health topic subscriptions are updated in real-time via callbacks.
        """
        import os
        current_time = time.time()
        
        # Get list of running ROS2 nodes
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            if result.returncode == 0:
                # ros2 node list returns node names with '/' prefix (e.g., '/controller_node')
                # Normalize to remove '/' prefix and filter out warnings/empty lines
                node_lines = [line.strip() for line in result.stdout.strip().split('\n') if line.strip() and not line.startswith('WARNING')]
                running_nodes = set([name.lstrip('/') for name in node_lines])  # Remove '/' prefix
            else:
                running_nodes = set()
        except Exception as e:
            self.get_logger().warn(f"Failed to get node list: {e}")
            running_nodes = set()
        
        # Check each configured node
        for node_cfg in self.node_configs.values():
            ros2_node_name = node_cfg['name']
            executable = node_cfg.get('executable', '')
            is_special = node_cfg.get('special', False)
            
            # For special nodes, use node name as key; for regular nodes, use filename from executable
            # This matches how lifecycle manager tracks nodes (special nodes by name, regular by .py filename)
            if is_special:
                health_key = ros2_node_name  # e.g., 'foxglove_bridge'
            else:
                health_key = os.path.basename(executable)  # e.g., 'gps.py'
            
            # Check if node is running (ros2_node_name from YAML should match without '/' prefix)
            is_running = ros2_node_name in running_nodes
            
            # Only update if we don't have a health topic subscription for this node
            # (health topic subscriptions take precedence)
            # Note: health_subscribers uses filename for regular nodes
            # For special nodes, executable is a command string (e.g., "ros2 run package node")
            # so we need to handle them differently
            if is_special:
                # Special nodes don't have health topics, never skip
                pass
            else:
                # Regular nodes: check if we have a health topic subscription
                filename_for_sub = os.path.basename(executable)
                if filename_for_sub in self.health_subscribers:
                    continue
            
            if health_key not in self.node_health:
                self.node_health[health_key] = {
                    'healthy': False,
                    'last_seen': 0.0,
                    'pid': None,
                    'ros2_node_name': ros2_node_name
                }
            
            # Update health status (only for nodes without health topic subscriptions)
            if is_running:
                # Special nodes: if running, always mark as healthy (they don't publish health topics)
                # Regular nodes: if we don't have health topic data, assume running = healthy
                # (nodes without ArgoBaseNode are considered healthy if they're running)
                self.node_health[health_key]['healthy'] = True
                self.node_health[health_key]['last_seen'] = current_time
                
                # Migration: Remove old entries that used executable string as key for special nodes
                # This handles the case where the health monitor was initialized before the fix
                if is_special:
                    old_key = executable  # Full executable string was used as key before
                    if old_key in self.node_health and old_key != health_key:
                        # Remove old entry with executable string as key
                        del self.node_health[old_key]
            else:
                # Node not running
                if node_cfg.get('required', True):
                    self.node_health[health_key]['healthy'] = False
                else:
                    # Optional node - mark as unknown if we haven't seen it recently
                    if current_time - self.node_health[health_key].get('last_seen', 0.0) > 30.0:
                        self.node_health[health_key]['healthy'] = None  # Unknown
    
    def publish_status(self):
        """Publish health status summary"""
        healthy_count = sum(1 for h in self.node_health.values() if h['healthy'] is True)
        unhealthy_count = sum(1 for h in self.node_health.values() if h['healthy'] is False)
        total_count = len(self.node_health)
        
        status_msg = String()
        status_msg.data = f"Nodes: {healthy_count}/{total_count} healthy, {unhealthy_count} unhealthy"
        
        self.health_pub.publish(status_msg)
    
    def _check_poll_transition(self):
        """Check if we should transition from fast to normal polling rate."""
        if not self.using_fast_poll:
            return  # Already transitioned
        
        elapsed = time.time() - self.startup_time
        if elapsed >= self.startup_duration:
            # Transition to normal polling
            self.get_logger().info(f"🐢 Transitioning to normal polling (every {self.normal_poll_period}s)")
            
            # Cancel and recreate timers with normal period
            self.timer.cancel()
            self.timer = self.create_timer(self.normal_poll_period, self.check_node_health)
            
            self.status_timer.cancel()
            self.status_timer = self.create_timer(self.normal_poll_period, self.publish_status)
            
            # Cancel the transition check timer (no longer needed)
            self.poll_transition_timer.cancel()
            
            self.using_fast_poll = False
    
    def status_callback(self, request, response):
        """Service callback for health status query
        
        Returns health data keyed by .py filenames (e.g., 'gps.py', 'record.py') or
        node names for special nodes (e.g., 'foxglove_bridge') for compatibility 
        with argo_lifecycle_manager.py
        """
        try:
            import os
            status_dict = {
                'timestamp': datetime.now().isoformat(),
                'nodes': {}
            }
            
            # Clean up old entries that use executable strings as keys for special nodes
            # This handles migration from old format
            old_keys_to_remove = []
            for health_key in list(self.node_health.keys()):
                # Check if this looks like an old special node key (contains 'ros2 run')
                if 'ros2 run' in health_key:
                    # Try to find the matching node config by executable
                    for cfg in self.node_configs.values():
                        if cfg.get('executable', '') == health_key and cfg.get('special', False):
                            # This is an old key for a special node - mark for removal
                            old_keys_to_remove.append(health_key)
                            break
            
            # Remove old keys
            for old_key in old_keys_to_remove:
                if old_key in self.node_health:
                    del self.node_health[old_key]
            
            # Ensure special nodes have entries (trigger health check if needed)
            # This handles the case where cleanup removed an old entry but check_node_health hasn't run yet
            special_nodes_missing = []
            for cfg in self.node_configs.values():
                if cfg.get('special', False):
                    node_name = cfg.get('name')
                    if node_name and node_name not in self.node_health:
                        special_nodes_missing.append(cfg)
            
            # If we have missing special nodes, trigger a quick health check
            if special_nodes_missing:
                try:
                    result = subprocess.run(
                        ['ros2', 'node', 'list'],
                        capture_output=True,
                        text=True,
                        timeout=2.0
                    )
                    if result.returncode == 0:
                        node_lines = [line.strip() for line in result.stdout.strip().split('\n') 
                                     if line.strip() and not line.startswith('WARNING')]
                        running_nodes = set([name.lstrip('/') for name in node_lines])
                        
                        # Initialize missing special nodes
                        for cfg in special_nodes_missing:
                            node_name = cfg.get('name')
                            is_running = node_name in running_nodes
                            self.node_health[node_name] = {
                                'healthy': True if is_running else None,
                                'last_seen': time.time() if is_running else 0.0,
                                'pid': None,
                                'ros2_node_name': node_name
                            }
                            if is_running:
                                self.get_logger().debug(f"Created missing entry for special node '{node_name}' (running)")
                            else:
                                self.get_logger().debug(f"Created missing entry for special node '{node_name}' (not running)")
                except Exception as e:
                    # If health check fails, log it but continue with existing data
                    self.get_logger().warn(f"Failed to create missing special node entries: {e}")
            
            # node_health is keyed by .py filenames for regular nodes (e.g., 'gps.py')
            # or by node name for special nodes (e.g., 'foxglove_bridge')
            for health_key, health in self.node_health.items():
                # Find node config by matching either:
                # 1. For special nodes: match by node name (cfg['name'] == health_key)
                # 2. For regular nodes: match by executable filename (os.path.basename(executable) == health_key)
                node_cfg = None
                for cfg in self.node_configs.values():
                    is_special = cfg.get('special', False)
                    if is_special:
                        # Special node: match by node name
                        if cfg.get('name') == health_key:
                            node_cfg = cfg
                            break
                    else:
                        # Regular node: match by executable filename
                        if os.path.basename(cfg.get('executable', '')) == health_key:
                            node_cfg = cfg
                            break
                
                # Skip old entries that don't match any config (migration cleanup)
                if node_cfg is None and 'ros2 run' in health_key:
                    continue
                
                if node_cfg is None:
                    # Use defaults if config not found
                    node_cfg = {}
                
                status_dict['nodes'][health_key] = {
                    'healthy': health['healthy'],
                    'required': node_cfg.get('required', True),
                    'critical': node_cfg.get('critical', False),
                    'description': node_cfg.get('description', ''),
                    'last_seen': health.get('last_seen', 0.0)
                }
            
            import json
            response.success = True
            response.message = json.dumps(status_dict, indent=2)
            
        except Exception as e:
            response.success = False
            response.message = f"Error getting status: {e}"
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ArgoHealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
