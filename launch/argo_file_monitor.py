#!/usr/bin/env python3

"""
Argo File Monitor Node
Monitors recording status files created by argo_record.sh
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os
import time

class ArgoFileMonitor(Node):
    def __init__(self):
        super().__init__('argo_file_monitor')
        
        # Publishers
        self.recording_status_pub = self.create_publisher(Bool, 'argo/recording/bagfile_status', 10)
        self.recording_info_pub = self.create_publisher(String, 'argo/recording/info', 10)
        
        # File paths
        self.status_file = '/tmp/argo_recording_status'
        self.bag_file = '/tmp/argo_current_bag'
        
        # Timer to check file status
        self.check_timer = self.create_timer(1.0, self.check_file_status)  # Check every second
        
        self.recording_active = False
        self.current_bag = None
        
        self.get_logger().info("Argo File Monitor started - monitoring recording status files")
    
    def check_file_status(self):
        """Check recording status by reading status files"""
        try:
            # Check if status file exists and contains "true"
            is_recording = False
            bag_name = None
            
            if os.path.exists(self.status_file):
                try:
                    with open(self.status_file, 'r') as f:
                        status_content = f.read().strip()
                        is_recording = (status_content == 'true')
                except Exception as e:
                    self.get_logger().debug(f"Could not read status file: {e}")
            
            # Get current bag name if available
            if os.path.exists(self.bag_file):
                try:
                    with open(self.bag_file, 'r') as f:
                        bag_name = f.read().strip()
                except Exception as e:
                    self.get_logger().debug(f"Could not read bag file: {e}")
            
            # Publish status if it changed
            if is_recording != self.recording_active:
                self.recording_active = is_recording
                
                status_msg = Bool()
                status_msg.data = self.recording_active
                self.recording_status_pub.publish(status_msg)
                
                if self.recording_active:
                    self.get_logger().info(f"🎬 Recording STARTED - bag: {bag_name or 'unknown'}")
                else:
                    self.get_logger().info("⏹️  Recording STOPPED")
            
            # Publish recording info
            if self.recording_active and bag_name:
                if bag_name != self.current_bag:
                    self.current_bag = bag_name
                    info_msg = String()
                    info_msg.data = f"Recording: {bag_name}"
                    self.recording_info_pub.publish(info_msg)
            elif not self.recording_active:
                self.current_bag = None
                
        except Exception as e:
            self.get_logger().error(f"Error checking file status: {e}")

def main():
    rclpy.init()
    
    try:
        monitor = ArgoFileMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("File monitor stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

