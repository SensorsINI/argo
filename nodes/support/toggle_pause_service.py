#!/usr/bin/env python3
"""
Shared TogglePause Service for Argo ROS2 Nodes
==============================================

This module provides a standardized pause/unpause service that can be used
across all Argo ROS2 nodes. When paused, nodes minimize their processing
and reduce their ROS spin loop activity.

Usage:
    from support.toggle_pause_service import TogglePauseService
    
    class MyNode(Node):
        def __init__(self):
            super().__init__('my_node')
            self.pause_service = TogglePauseService(self)
            
        def timer_callback(self):
            if self.pause_service.is_paused():
                return  # Skip processing when paused
            # Normal processing here
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from typing import Optional


class TogglePauseService:
    """
    Shared service for pausing/unpausing ROS2 nodes.
    
    When paused:
    - Node processing is minimized
    - Timers and callbacks can check is_paused() to skip work
    - Health status reflects paused state
    - Service remains responsive for unpause requests
    """
    
    def __init__(self, node: Node, service_name: str = 'toggle_pause'):
        """
        Initialize the pause service.
        
        Args:
            node: The ROS2 node to attach the service to
            service_name: Name of the service (default: 'toggle_pause')
        """
        self.node = node
        self._is_paused = False
        self._pause_count = 0  # Track number of pause requests
        
        # Create the service
        self.service = self.node.create_service(
            Trigger,
            service_name,
            self._handle_toggle_pause
        )
        
        # Create health publisher to indicate pause status
        self.health_pub = self.node.create_publisher(
            Bool,
            f'{self.node.get_name()}_health',
            10
        )
        
        # Publish initial health status
        self._publish_health()
        
        self.node.get_logger().info(f"TogglePause service created at /{service_name}")
    
    def _handle_toggle_pause(self, request, response):
        """Handle toggle pause service requests."""
        try:
            if self._is_paused:
                # Currently paused - unpause
                self._is_paused = False
                self._pause_count = 0
                self.node.get_logger().info("Node UNPAUSED - resuming normal operation")
                response.success = True
                response.message = "Node unpaused successfully"
            else:
                # Currently running - pause
                self._is_paused = True
                self._pause_count += 1
                self.node.get_logger().info(f"Node PAUSED (pause #{self._pause_count}) - minimizing processing")
                response.success = True
                response.message = f"Node paused successfully (pause #{self._pause_count})"
            
            # Publish updated health status
            self._publish_health()
            
        except Exception as e:
            self.node.get_logger().error(f"Error handling toggle pause: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def is_paused(self) -> bool:
        """
        Check if the node is currently paused.
        
        Returns:
            True if paused, False if running normally
        """
        return self._is_paused
    
    def force_pause(self):
        """Force pause the node (for external control)."""
        if not self._is_paused:
            self._is_paused = True
            self._pause_count += 1
            self.node.get_logger().info(f"Node FORCE PAUSED (pause #{self._pause_count})")
            self._publish_health()
    
    def force_unpause(self):
        """Force unpause the node (for external control)."""
        if self._is_paused:
            self._is_paused = False
            self._pause_count = 0
            self.node.get_logger().info("Node FORCE UNPAUSED - resuming normal operation")
            self._publish_health()
    
    def _publish_health(self):
        """Publish current health status (including pause state)."""
        try:
            # Health is False when paused, True when running normally
            health_msg = Bool()
            health_msg.data = not self._is_paused
            self.health_pub.publish(health_msg)
        except Exception as e:
            self.node.get_logger().error(f"Error publishing health status: {e}")
    
    def get_pause_status(self) -> dict:
        """
        Get detailed pause status information.
        
        Returns:
            Dictionary with pause status details
        """
        return {
            'is_paused': self._is_paused,
            'pause_count': self._pause_count,
            'service_name': self.service.srv_name,
            'health_topic': self.health_pub.topic_name
        }

