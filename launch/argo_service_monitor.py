#!/usr/bin/env python3

"""
Argo Service Monitor Node
Monitors the argo-record systemd service and publishes recording status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import subprocess
import time

class ArgoServiceMonitor(Node):
    def __init__(self):
        super().__init__('argo_service_monitor')
        
        # Publishers
        self.recording_status_pub = self.create_publisher(Bool, 'argo/recording/bagfile_status', 10)
        self.service_status_pub = self.create_publisher(String, 'argo/recording/service_status', 10)
        
        # Timer to check service status
        self.check_timer = self.create_timer(2.0, self.check_service_status)  # Check every 2 seconds
        
        self.service_active = False
        self.last_status = None
        
        self.get_logger().info("Argo Service Monitor started - monitoring ROS2 recording service")
    
    def check_service_status(self):
        """Check the status of the ROS2 recording service"""
        try:
            # Check if ROS2 recording service is available and active
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                # Check if recording service exists
                service_available = '/argo/recording/start' in result.stdout
                
                # Check if recording is actually active by checking the status topic
                if service_available:
                    status_result = subprocess.run(['ros2', 'topic', 'echo', '/argo/recording/status', '--once'], 
                                                 capture_output=True, text=True, timeout=3.0)
                    is_active = False
                    if status_result.returncode == 0 and 'data: true' in status_result.stdout:
                        is_active = True
                else:
                    is_active = False
                
                # Check if status changed
                if is_active != self.service_active:
                    self.service_active = is_active
                    
                    # Publish recording status
                    status_msg = Bool()
                    status_msg.data = self.service_active
                    self.recording_status_pub.publish(status_msg)
                    
                    if self.service_active:
                        self.get_logger().info("🎬 ROS2 recording service is ACTIVE - recording started")
                    else:
                        self.get_logger().info("⏹️  ROS2 recording service is INACTIVE - recording stopped")
                
                # Always publish service status info
                status_info = f"ROS2 recording service: {'active' if is_active else 'inactive'}"
                if status_info != self.last_status:
                    self.last_status = status_info
                    info_msg = String()
                    info_msg.data = status_info
                    self.service_status_pub.publish(info_msg)
                    
                    self.get_logger().debug(f"Service status: {status_info}")
            
            else:
                self.get_logger().warning("Failed to check service status")
                
        except subprocess.TimeoutExpired:
            self.get_logger().warning("Timeout checking service status")
        except Exception as e:
            self.get_logger().error(f"Error checking service status: {e}")

def main():
    rclpy.init()
    
    try:
        monitor = ArgoServiceMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Service monitor stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
