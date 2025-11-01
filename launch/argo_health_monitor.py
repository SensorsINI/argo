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
        self.node_health = {}  # {node_name: {'healthy': bool, 'last_seen': float, 'pid': int}}
        self.node_configs = {}
        
        # Load all node configs (normal + simulation)
        for node_cfg in self.config.get('nodes', []):
            self.node_configs[node_cfg['name']] = node_cfg
        for node_cfg in self.config.get('simulation_nodes', []):
            self.node_configs[node_cfg['name']] = node_cfg
        
        # Publishers
        self.health_pub = self.create_publisher(
            String, '/argo/health/status', 10)
        
        # Services
        self.status_service = self.create_service(
            Trigger, '/argo/health/status', self.status_callback)
        
        # Timer for periodic health checks (every 2 seconds)
        self.timer = self.create_timer(30.0, self.check_node_health)
        
        # Timer for status publishing (every 5 seconds)
        self.status_timer = self.create_timer(30.0, self.publish_status)
        
        self.get_logger().info("Argo Health Monitor started")
        self.get_logger().info(f"Monitoring {len(self.node_configs)} nodes")
    
    def check_node_health(self):
        """Check health of all configured nodes"""
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
                running_nodes = set(result.stdout.strip().split('\n'))
            else:
                running_nodes = set()
        except Exception as e:
            self.get_logger().warn(f"Failed to get node list: {e}")
            running_nodes = set()
        
        # Check each configured node
        for node_name, node_cfg in self.node_configs.items():
            is_running = node_name in running_nodes
            
            if node_name not in self.node_health:
                self.node_health[node_name] = {
                    'healthy': False,
                    'last_seen': 0.0,
                    'pid': None
                }
            
            # Update health status
            if is_running:
                self.node_health[node_name]['healthy'] = True
                self.node_health[node_name]['last_seen'] = current_time
                
                # Try to get PID
                try:
                    result = subprocess.run(
                        ['ros2', 'node', 'info', node_name],
                        capture_output=True,
                        text=True,
                        timeout=1.0
                    )
                    # Parse PID from node info if available
                    # (This is a simplified version - actual PID parsing would be more complex)
                except Exception:
                    pass
            else:
                # Node not running - check if it's required
                if node_cfg.get('required', True):
                    self.node_health[node_name]['healthy'] = False
                else:
                    # Optional node - mark as unknown if we haven't seen it recently
                    if current_time - self.node_health[node_name]['last_seen'] > 30.0:
                        self.node_health[node_name]['healthy'] = None  # Unknown
    
    def publish_status(self):
        """Publish health status summary"""
        healthy_count = sum(1 for h in self.node_health.values() if h['healthy'] is True)
        unhealthy_count = sum(1 for h in self.node_health.values() if h['healthy'] is False)
        total_count = len(self.node_health)
        
        status_msg = String()
        status_msg.data = f"Nodes: {healthy_count}/{total_count} healthy, {unhealthy_count} unhealthy"
        
        self.health_pub.publish(status_msg)
    
    def status_callback(self, request, response):
        """Service callback for health status query"""
        try:
            status_dict = {
                'timestamp': datetime.now().isoformat(),
                'nodes': {}
            }
            
            for node_name, health in self.node_health.items():
                node_cfg = self.node_configs.get(node_name, {})
                status_dict['nodes'][node_name] = {
                    'healthy': health['healthy'],
                    'required': node_cfg.get('required', True),
                    'critical': node_cfg.get('critical', False),
                    'description': node_cfg.get('description', ''),
                    'last_seen': health['last_seen']
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

