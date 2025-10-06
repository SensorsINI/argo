#!/usr/bin/env python3
"""
Argo ROS2 Recording Node
========================

A ROS2 node that handles rosbag recording with start/stop control via ROS2 messages.
This replaces the standalone recording scripts and integrates with the Argo lifecycle manager.

Features:
- ROS2 service interface for start/stop recording
- ROS2 topic publishing for status updates
- Automatic bag file naming with timestamps
- Graceful shutdown handling
- Integration with Argo lifecycle management

Usage:
    ros2 run argo record.py
    ros2 service call /argo/recording/start std_srvs/srv/Empty
    ros2 service call /argo/recording/stop std_srvs/srv/Empty
    ros2 topic echo /argo/recording/status
"""

import os
import sys
import time
import signal
import subprocess
import threading
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
# Removed QoS imports - using default QoS only
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


class ArgoRecordingNode(Node):
    """ROS2 node for managing Argo bag recording"""
    
    def __init__(self):
        super().__init__('record')
        
        # Configuration
        self.bagfiles_dir = os.path.join(os.path.expanduser('~'), 'bagfiles')
        self.current_bag_name = None
        self.recording_process = None
        self.is_recording = False
        
        # Ensure bagfiles directory exists
        os.makedirs(self.bagfiles_dir, exist_ok=True)
        
        # Using default QoS for all publishers
        
        # ROS2 Services
        self.start_service = self.create_service(
            Empty,
            '/argo/recording/start',
            self.start_recording_callback
        )
        
        self.stop_service = self.create_service(
            Empty,
            '/argo/recording/stop',
            self.stop_recording_callback
        )
        
        # ROS2 Publishers
        self.status_publisher = self.create_publisher(
            Bool,
            '/argo/recording/status',
            10
        )
        
        self.info_publisher = self.create_publisher(
            String,
            '/argo/recording/info',
            10
        )
        
        # ROS2 Subscribers
        self.recording_control_sub = self.create_subscription(
            Twist,
            '/argo/recording/control',
            self.recording_control_callback,
            10
        )
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info("🚀 Argo Recording Node started")
        self.get_logger().info(f"📁 Bagfiles directory: {self.bagfiles_dir}")
        self.get_logger().info("🎮 Services available:")
        self.get_logger().info("   /argo/recording/start - Start recording")
        self.get_logger().info("   /argo/recording/stop - Stop recording")
        self.get_logger().info("📡 Topics available:")
        self.get_logger().info("   /argo/recording/status - Recording status (Bool)")
        self.get_logger().info("   /argo/recording/info - Recording info (String)")
        self.get_logger().info("   /argo/recording/control - Recording control (Twist)")
        
        # Publish initial status
        self.publish_status()
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info(f"🛑 Received signal {signum}, shutting down...")
        self.stop_recording()
        rclpy.shutdown()
        sys.exit(0)
    
    def generate_bag_name(self) -> str:
        """Generate a unique bag name with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"argo_{timestamp}"
    
    def start_recording_callback(self, request, response):
        """Service callback to start recording"""
        self.get_logger().info("🎬 Start recording service called")
        
        if self.is_recording:
            self.get_logger().warn("⚠️  Recording is already in progress")
            return response
        
        success = self.start_recording()
        if success:
            self.get_logger().info("✅ Recording started successfully")
        else:
            self.get_logger().error("❌ Failed to start recording")
        
        return response
    
    def stop_recording_callback(self, request, response):
        """Service callback to stop recording"""
        self.get_logger().info("🛑 Stop recording service called")
        
        if not self.is_recording:
            self.get_logger().warn("⚠️  No recording in progress")
            return response
        
        success = self.stop_recording()
        if success:
            self.get_logger().info("✅ Recording stopped successfully")
        else:
            self.get_logger().error("❌ Failed to stop recording")
        
        return response
    
    def recording_control_callback(self, msg):
        """Handle recording control via Twist message"""
        # Use linear.x to control recording: > 0 = start, <= 0 = stop
        if msg.linear.x > 0 and not self.is_recording:
            self.get_logger().info("🎬 Recording control: START")
            self.start_recording()
        elif msg.linear.x <= 0 and self.is_recording:
            self.get_logger().info("🛑 Recording control: STOP")
            self.stop_recording()
    
    def start_recording(self) -> bool:
        """Start rosbag recording"""
        try:
            if self.is_recording:
                self.get_logger().warn("⚠️  Recording already in progress")
                return False
            
            # Generate new bag name
            self.current_bag_name = self.generate_bag_name()
            bag_path = os.path.join(self.bagfiles_dir, self.current_bag_name)
            
            self.get_logger().info(f"🚀 Starting recording: {self.current_bag_name}")
            self.get_logger().info(f"📁 Bag path: {bag_path}")
            self.get_logger().info(f"🕐 Start time: {datetime.now()}")
            
            # Start rosbag recording process
            cmd = ['ros2', 'bag', 'record', '-a', '-o', self.current_bag_name]
            self.recording_process = subprocess.Popen(
                cmd,
                cwd=self.bagfiles_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            self.is_recording = True
            
            # Publish status updates
            self.publish_status()
            self.publish_info(f"Recording started - {self.current_bag_name}")
            
            # Start monitoring thread
            self.monitor_thread = threading.Thread(target=self._monitor_recording)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error starting recording: {e}")
            return False
    
    def stop_recording(self) -> bool:
        """Stop rosbag recording"""
        try:
            if not self.is_recording:
                self.get_logger().warn("⚠️  No recording in progress")
                return False
            
            self.get_logger().info("🛑 Stopping recording...")
            
            if self.recording_process:
                # Send SIGTERM to rosbag process
                self.recording_process.terminate()
                
                # Wait for graceful shutdown
                try:
                    self.recording_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("⚠️  Force killing recording process...")
                    self.recording_process.kill()
                    self.recording_process.wait()
                
                self.recording_process = None
            
            # Log final bag information
            if self.current_bag_name:
                bag_path = os.path.join(self.bagfiles_dir, self.current_bag_name)
                if os.path.exists(bag_path):
                    try:
                        # Get bag size
                        size_cmd = ['du', '-sh', bag_path]
                        size_result = subprocess.run(size_cmd, capture_output=True, text=True)
                        bag_size = size_result.stdout.split()[0] if size_result.returncode == 0 else "Unknown"
                        
                        self.get_logger().info(f"📦 Final bag location: {bag_path}")
                        self.get_logger().info(f"💾 Bag size: {bag_size}")
                        
                        # List bag contents
                        ls_cmd = ['ls', '-la', bag_path]
                        ls_result = subprocess.run(ls_cmd, capture_output=True, text=True)
                        if ls_result.returncode == 0:
                            self.get_logger().info("📁 Bag contents:")
                            for line in ls_result.stdout.strip().split('\n'):
                                self.get_logger().info(f"   {line}")
                    except Exception as e:
                        self.get_logger().warn(f"⚠️  Could not get bag info: {e}")
            
            self.is_recording = False
            self.current_bag_name = None
            
            # Publish status updates
            self.publish_status()
            self.publish_info("Recording stopped")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error stopping recording: {e}")
            return False
    
    def _monitor_recording(self):
        """Monitor recording process in background thread"""
        if not self.recording_process:
            return
        
        try:
            # Read output from recording process
            for line in iter(self.recording_process.stdout.readline, ''):
                if line:
                    # Log recording output (can be verbose, so use debug level)
                    self.get_logger().debug(f"rosbag: {line.strip()}")
            
            # Check if process ended unexpectedly
            return_code = self.recording_process.wait()
            if return_code != 0:
                self.get_logger().error(f"❌ Recording process ended with code {return_code}")
                self.is_recording = False
                self.publish_status()
                self.publish_info(f"Recording failed (exit code {return_code})")
            else:
                self.get_logger().info("✅ Recording process completed normally")
                self.is_recording = False
                self.publish_status()
                self.publish_info("Recording completed")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error monitoring recording: {e}")
            self.is_recording = False
            self.publish_status()
    
    def publish_status(self):
        """Publish current recording status"""
        try:
            status_msg = Bool()
            status_msg.data = self.is_recording
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing status: {e}")
    
    def publish_info(self, message: str):
        """Publish recording info message"""
        try:
            info_msg = String()
            info_msg.data = message
            self.info_publisher.publish(info_msg)
            self.get_logger().info(f"📡 Published info: {message}")
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing info: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = ArgoRecordingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nRecording node stopped by user.")
    except rclpy.executors.ExternalShutdownException:
        print("External shutdown signal received, exiting gracefully.")
    except Exception as e:
        print(f"❌ Error in main: {e}")
    finally:
        try:
            if node:
                node.destroy_node()
        except Exception:
            pass  # Ignore errors during shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore errors during shutdown


if __name__ == '__main__':
    main()
