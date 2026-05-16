#!/usr/bin/env python3
"""
ArgoBaseNode - Base class for all Argo ROS2 nodes

Provides standardized functionality:
- Health monitoring (service + topic)
- Safe publishing and logging
- Graceful shutdown handling
- Signal handling
- Standardized argument parsing
- Timer management
- Context validation
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import json
import time
import signal
import atexit
import argparse
import argcomplete
import sys
import os
from typing import Dict, Any, Optional, List

# Import safe publishing utilities
# Use absolute path to ensure reliable import regardless of working directory
_support_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'support'))
if _support_dir not in sys.path:
    sys.path.insert(0, _support_dir)
from safe_publish import safe_publish, safe_log, is_context_valid

class ArgoBaseNode(Node):
    """
    Base class for all Argo ROS2 nodes providing standardized functionality.
    
    Features:
    - Standardized health service endpoint and publishing
    - Safe publishing and logging utilities
    - Graceful shutdown handling
    - Signal handling for safe hardware state
    - Standardized argument parsing
    - Timer management
    - Context validation
    """
    
    def __init__(self, node_name: str, enable_health_service: bool = True, 
                 enable_health_publisher: bool = True, enable_signal_handlers: bool = True):
        super().__init__(node_name)
        
        self.node_name = node_name
        self.health_status = False
        self.health_details = "Initializing"
        self.last_health_update = time.time()
        self.shutdown_requested = False
        self._timers: List[rclpy.timer.Timer] = []
        
        # Health service (for on-demand queries)
        if enable_health_service:
            self.health_service = self.create_service(
                Trigger, f'/{node_name}/health', self._handle_health_request)
            self.get_logger().info(f"Health service available at /{node_name}/health")
        
        # Health publisher (for continuous monitoring)
        if enable_health_publisher:
            self.health_publisher = self.create_publisher(
                Bool, f'/{node_name}_health', 10)
        self.health_timer = self.create_timer(5.0, self._publish_health_status)
        self._timers.append(self.health_timer)
        
        # Signal handlers for graceful shutdown
        if enable_signal_handlers:
            self._setup_signal_handlers()
        
        # Initialize health as unhealthy
        self._update_health_status(False, "Node starting up")
    
    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        atexit.register(self._cleanup_on_exit)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True
        self._cleanup_on_exit()
        
        # Allow normal signal handling to proceed
        if signum == signal.SIGINT:
            raise KeyboardInterrupt()
        elif signum == signal.SIGTERM:
            sys.exit(0)
    
    def _cleanup_on_exit(self):
        """Cleanup function called on exit - override in subclasses"""
        pass
    
    def create_timer(self, period: float, callback, **kwargs):
        """Create timer and track it for cleanup"""
        timer = super().create_timer(period, callback, **kwargs)
        self._timers.append(timer)
        return timer
    
    def _handle_health_request(self, request, response):
        """Handle health status service request"""
        try:
            response.success = True
            response.message = json.dumps({
                'healthy': self.health_status,
                'details': self.health_details,
                'timestamp': self.last_health_update,
                'node_name': self.node_name
            })
        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {e}"
        
        return response
    
    def _publish_health_status(self):
        """Publish health status to topic"""
        if not is_context_valid(self):
            return
            
        try:
            msg = Bool()
            msg.data = self.health_status
            safe_publish(self.health_publisher, msg, self)
        except Exception as e:
            safe_log(self, 'error', f"Failed to publish health status: {e}")
    
    def _update_health_status(self, is_healthy: bool, details: str = None):
        """Update health status and publish if changed"""
        if self.health_status != is_healthy or details != self.health_details:
            self.health_status = is_healthy
            self.health_details = details or ("Healthy" if is_healthy else "Unhealthy")
            self.last_health_update = time.time()
            
            # Publish health status
            if hasattr(self, 'health_publisher'):
                self._publish_health_status()
            
            # Log health change
            if is_healthy:
                safe_log(self, 'info', f"Health status: HEALTHY - {self.health_details}")
            else:
                safe_log(self, 'warn', f"Health status: UNHEALTHY - {self.health_details}")
    
    def set_healthy(self, details: str = "Node operating normally"):
        """Set node as healthy"""
        self._update_health_status(True, details)
    
    def set_unhealthy(self, details: str = "Node has issues"):
        """Set node as unhealthy"""
        self._update_health_status(False, details)
    
    def get_health_status(self) -> Dict[str, Any]:
        """Get current health status"""
        return {
            'healthy': self.health_status,
            'details': self.health_details,
            'timestamp': self.last_health_update,
            'node_name': self.node_name
        }
    
    def destroy_node(self):
        """Override destroy_node for graceful shutdown"""
        self.shutdown_requested = True
        
        # Cancel all timers first
        for timer in self._timers:
            try:
                timer.cancel()
            except Exception:
                pass  # Ignore errors during shutdown
        
        # Publish health status as failed
        self._update_health_status(False, "Node shutting down")
        
        # Call node-specific cleanup
        self._cleanup_on_exit()
        
        # Call parent destroy_node
        super().destroy_node()
    
    @staticmethod
    def create_standard_parser(description: str, epilog: str = None) -> argparse.ArgumentParser:
        """Create standardized argument parser for Argo nodes"""
        parser = argparse.ArgumentParser(
            description=description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=epilog
        )
        parser.add_argument('--debug', action='store_true',
                          help='Enable debug logging')
        return parser
    
    @staticmethod
    def run_node(node_class, args=None, parser=None):
        """Standardized main function for Argo nodes"""
        node = None
        
        try:
            # Parse arguments if parser provided
            if parser:
                parsed_args, unknown_args = parser.parse_known_args(args)
                argcomplete.autocomplete(parser)
            else:
                unknown_args = args or []
            
            # Initialize ROS2
            rclpy.init(args=unknown_args)
            
            # Create node
            if parser:
                # Pass standard arguments (debug) and any node-specific arguments
                kwargs = {'debug_mode': getattr(parsed_args, 'debug', False)}
                
                # Add node-specific arguments if they exist
                if hasattr(parsed_args, 'reset'):
                    kwargs['do_hardware_reset'] = parsed_args.reset
                if hasattr(parsed_args, 'factory_reset'):
                    kwargs['do_factory_reset'] = parsed_args.factory_reset
                
                node = node_class(**kwargs)
            else:
                node = node_class()
            
            # Spin node
            rclpy.spin(node)
            
        except KeyboardInterrupt:
            if node:
                safe_log(node, 'info', f"{node.node_name} interrupted, shutting down gracefully...")
        except ExternalShutdownException:
            if node:
                safe_log(node, 'info', "External shutdown received, exiting gracefully...")
        except Exception as e:
            if node:
                safe_log(node, 'error', f"Unexpected error: {e}")
            else:
                print(f"Error before node creation: {e}")
        finally:
            # Clean shutdown
            if node:
                try:
                    node.destroy_node()
                except Exception:
                    pass  # Suppress any errors during node destruction
            
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass  # Suppress ROS shutdown errors
