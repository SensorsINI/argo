#!/usr/bin/env python3

"""
Demo script showing how to test the node health monitoring and SOS pattern
This simulates missing argo nodes to trigger the SOS pattern
"""

import rclpy
from rclpy.node import Node
import subprocess
import time

class NodeHealthDemo(Node):
    def __init__(self):
        super().__init__('node_health_demo')
        
        self.get_logger().info("Node health demo started")
        self.get_logger().info("This demo shows how the power control node monitors other argo nodes")
        self.get_logger().info("The blue LED will flash SOS pattern if any argo nodes are missing")
        
        # Timer to check node status
        self.timer = self.create_timer(3.0, self.check_node_status)
    
    def check_node_status(self):
        try:
            # Get list of active nodes
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                active_nodes = [line.strip() for line in result.stdout.split('\n') if line.strip()]
                
                # Check for expected argo nodes
                expected_argo_nodes = [
                    'argo_navigation',
                    'argo_sensors', 
                    'argo_actuators',
                    'argo_communication',
                    'argo_power_control'
                ]
                
                missing_nodes = []
                for expected_node in expected_argo_nodes:
                    if expected_node not in active_nodes:
                        missing_nodes.append(expected_node)
                
                if missing_nodes:
                    self.get_logger().warn(f"⚠️  Missing argo nodes: {missing_nodes}")
                    self.get_logger().warn("🔴 Blue LED should be flashing SOS pattern!")
                else:
                    self.get_logger().info("✅ All expected argo nodes are running")
                    self.get_logger().info("🟢 Blue LED should be off (no SOS pattern)")
                
                self.get_logger().info(f"Active nodes: {len(active_nodes)} total")
                
            else:
                self.get_logger().error("Failed to get node list")
                
        except Exception as e:
            self.get_logger().error(f"Error checking node status: {e}")

def main():
    rclpy.init()
    
    try:
        demo = NodeHealthDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info("Demo stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

