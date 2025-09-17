#!/usr/bin/env python3

"""
Argo Recording Bridge Node
Monitors the argo-record service and publishes recording status to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import subprocess
import time
import psutil

class ArgoRecordingBridge(Node):
    def __init__(self):
        super().__init__('argo_recording_bridge')
        
        # Publishers
        self.recording_status_pub = self.create_publisher(Bool, 'argo/recording/bagfile_status', 10)
        self.recording_info_pub = self.create_publisher(String, 'argo/recording/info', 10)
        
        # Timer to check recording status
        self.check_timer = self.create_timer(1.0, self.check_recording_status)  # Check every second
        
        self.recording_active = False
        self.last_bag_name = None
        
        self.get_logger().info("Argo Recording Bridge started - monitoring argo-record service")
    
    def check_recording_status(self):
        """Check if recording is active by looking for ros2 bag record processes"""
        try:
            # Check for running ros2 bag record processes
            recording_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.info['cmdline'] and any('ros2' in cmd and 'bag' in cmd and 'record' in cmd for cmd in proc.info['cmdline']):
                        recording_processes.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            # Also check for the argo_record.sh script
            argo_record_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.info['cmdline'] and any('argo_record.sh' in cmd for cmd in proc.info['cmdline']):
                        argo_record_processes.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            # Determine if recording is active
            is_recording = len(recording_processes) > 0 or len(argo_record_processes) > 0
            
            # Only publish if status changed
            if is_recording != self.recording_active:
                self.recording_active = is_recording
                
                # Publish recording status
                status_msg = Bool()
                status_msg.data = self.recording_active
                self.recording_status_pub.publish(status_msg)
                
                if self.recording_active:
                    self.get_logger().info("🎬 Recording STARTED - publishing status to ROS2")
                    # Try to get current bag name from log
                    self.get_current_bag_info()
                else:
                    self.get_logger().info("⏹️  Recording STOPPED - publishing status to ROS2")
            
            # Always publish recording info (with current bag name if available)
            if self.recording_active:
                self.publish_recording_info()
                
        except Exception as e:
            self.get_logger().error(f"Error checking recording status: {e}")
    
    def get_current_bag_info(self):
        """Try to get current bag name from the recording log"""
        try:
            # Read the last few lines of the recording log
            result = subprocess.run(['tail', '-20', '/tmp/argo_record.log'], 
                                  capture_output=True, text=True, timeout=2.0)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'Bag name:' in line:
                        # Extract bag name from log line
                        parts = line.split('Bag name:')
                        if len(parts) > 1:
                            self.last_bag_name = parts[1].strip()
                            break
        except Exception as e:
            self.get_logger().debug(f"Could not get bag info from log: {e}")
    
    def publish_recording_info(self):
        """Publish detailed recording information"""
        info_msg = String()
        if self.last_bag_name:
            info_msg.data = f"Recording bag: {self.last_bag_name}"
        else:
            info_msg.data = "Recording active - bag info unavailable"
        
        self.recording_info_pub.publish(info_msg)

def main():
    rclpy.init()
    
    try:
        bridge = ArgoRecordingBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Recording bridge stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

