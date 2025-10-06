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
    ros2 service call /argo/recording/start std_srvs/srv/Trigger
    ros2 service call /argo/recording/stop std_srvs/srv/Trigger
    ros2 topic echo /argo/recording/status
"""

import os
import sys
import time
import signal
import subprocess
import threading
import logging
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
# Removed QoS imports - using default QoS only
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


class ArgoRecordingNode(Node):
    """ROS2 node for managing Argo bag recording"""

    def __init__(self):
        super().__init__('record')

        # Setup file logging for detailed debugging
        self._setup_file_logging()

        # Configuration
        self.bagfiles_dir = os.path.join(os.path.expanduser('~'), 'bagfiles')
        self.current_bag_name = None
        self.bag_path = None
        self.recording_process = None
        self.is_recording = False

        # Ensure bagfiles directory exists
        os.makedirs(self.bagfiles_dir, exist_ok=True)
        
        # Using default QoS for all publishers
        # ROS2 Services
        self.start_service = self.create_service(
            Trigger,
            '/argo/recording/start',
            self.start_recording_callback
        )

        self.stop_service = self.create_service(
            Trigger,
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

        self._log_info("🚀 Argo Recording Node started")
        self._log_info(f"📁 Bagfiles directory: {self.bagfiles_dir}")
        self._log_info("🎮 Services available:")
        self._log_info("   /argo/recording/start - Start recording (Trigger service)")
        self._log_info("   /argo/recording/stop - Stop recording (Trigger service)")
        self._log_info("📡 Topics available:")
        self._log_info("   /argo/recording/status - Recording status (Bool)")
        self._log_info("   /argo/recording/info - Recording info (String)")
        self._log_info("   /argo/recording/control - Recording control (Twist)")

        # Publish initial status
        self.publish_status()

    def _setup_file_logging(self):
        """Setup file logging to /tmp/record.log for detailed debugging"""
        try:
            # Create a separate logger for file logging since ROS2 logger doesn't support addHandler
            self.file_logger = logging.getLogger('record_file')
            self.file_logger.setLevel(logging.DEBUG)
            
            # Create a file handler for detailed logging
            file_handler = logging.FileHandler('/tmp/record.log', mode='a')
            file_handler.setLevel(logging.DEBUG)
            
            # Create a formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            file_handler.setFormatter(formatter)
            
            # Add the file handler to the file logger
            self.file_logger.addHandler(file_handler)
            
            # Prevent propagation to avoid duplicate logs
            self.file_logger.propagate = False
            
            # Log that file logging is set up
            self.file_logger.info("📝 File logging enabled: /tmp/record.log")
            self._log_info("📝 File logging enabled: /tmp/record.log")
            
        except Exception as e:
            # If file logging fails, continue without it
            self._log_warn(f"⚠️  Could not setup file logging: {e}")
            self.file_logger = None

    def _log_debug(self, message):
        """Log debug message to both ROS2 logger and file logger"""
        self.get_logger().debug(message)
        if self.file_logger:
            self.file_logger.debug(message)

    def _log_info(self, message):
        """Log info message to both ROS2 logger and file logger"""
        self.get_logger().info(message)
        if self.file_logger:
            self.file_logger.info(message)

    def _log_warn(self, message):
        """Log warning message to both ROS2 logger and file logger"""
        self.get_logger().warn(message)
        if self.file_logger:
            self.file_logger.warning(message)

    def _log_error(self, message):
        """Log error message to both ROS2 logger and file logger"""
        self.get_logger().error(message)
        if self.file_logger:
            self.file_logger.error(message)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self._log_info(f"🛑 Received signal {signum}, shutting down...")
        self.stop_recording()
        rclpy.shutdown()
        sys.exit(0)

    def generate_bag_name(self) -> str:
        """Generate a unique bag name with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"argo_{timestamp}"

    def start_recording_callback(self, request, response):
        """Service callback to start recording"""
        self._log_info("🎬 Start recording service called")
        self._log_debug(f"Service request received: {request}")
        self._log_debug(f"Current recording state: {self.is_recording}")

        if self.is_recording:
            response.success = False
            response.message = "Recording is already in progress"
            self._log_warn("⚠️  Recording is already in progress")
            self._log_debug(f"Service response: success={response.success}, message='{response.message}'")
            return response

        self._log_debug("Calling start_recording() method...")
        success = self.start_recording()
        self._log_debug(f"start_recording() returned: {success}")
        
        if success:
            response.success = True
            response.message = f"Recording started successfully - {self.current_bag_name}"
            self._log_info("✅ Recording started successfully")
        else:
            response.success = False
            response.message = "Failed to start recording"
            self._log_error("❌ Failed to start recording")

        self._log_debug(f"Service response: success={response.success}, message='{response.message}'")
        return response

    def stop_recording_callback(self, request, response):
        """Service callback to stop recording"""
        self._log_info("🛑 Stop recording service called")

        if not self.is_recording:
            response.success = False
            response.message = "No recording in progress"
            self._log_warn("⚠️  No recording in progress")
            return response

        # Store bag name before stopping for response message
        success = self.stop_recording()
        
        if success:
            response.success = True
            response.message = f"Recording stopped successfully - {self.stopped_bag_name}"
            self._log_info("✅ Recording stopped successfully")
        else:
            response.success = False
            response.message = "Failed to stop recording"
            self._log_error("❌ Failed to stop recording")

        return response

    def recording_control_callback(self, msg):
        """Handle recording control via Twist message"""
        # Use linear.x to control recording: > 0 = start, <= 0 = stop
        if msg.linear.x > 0 and not self.is_recording:
            self._log_info("🎬 Recording control: START")
            self.start_recording()
        elif msg.linear.x <= 0 and self.is_recording:
            self._log_info("🛑 Recording control: STOP")
            self.stop_recording()

    def start_recording(self) -> bool:
        """Start rosbag recording"""
        try:
            self._log_debug("start_recording() method called")
            
            if self.is_recording:
                self._log_warn("⚠️  Recording already in progress")
                self._log_debug("start_recording() returning False - already recording")
                return False

            # Generate new bag name
            self.current_bag_name = self.generate_bag_name()
            self.bag_path = os.path.join(self.bagfiles_dir, self.current_bag_name)
            self._log_debug(f"Generated bag name: {self.current_bag_name}")
            self._log_debug(f"Bag path: {self.bag_path}")

            self._log_info(f"🚀 Starting recording: {self.bag_path}")
            self._log_info(f"📁 Bag path: {self.bag_path}")
            self._log_info(f"🕐 Start time: {datetime.now()}")

            # Start rosbag recording process
            cmd = ['ros2', 'bag', 'record', '-a', '-o', self.current_bag_name]
            self._log_debug(f"Rosbag command: {' '.join(cmd)}")
            self._log_debug("Creating subprocess for rosbag recording...")
            self.recording_process = subprocess.Popen(
                cmd,
                cwd=self.bagfiles_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            self._log_debug(f"Subprocess created with PID: {self.recording_process.pid}")

            self.is_recording = True
            self._log_debug("Setting is_recording = True")

            # Publish status updates
            self.publish_status()
            self.publish_info(f"Recording started - {self.current_bag_name}")
            self._log_debug("Published initial status and info")

            # Start monitoring thread
            self._log_debug("Starting monitoring thread...")
            self.monitor_thread = threading.Thread(
                target=self._monitor_recording)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            self._log_debug("Monitoring thread started")

            self._log_debug("start_recording() returning True")
            return True

        except Exception as e:
            self._log_error(f"❌ Error starting recording: {e}")
            return False

    def stop_recording(self) -> bool:
        """Stop rosbag recording"""
        try:
            if not self.is_recording:
                self._log_warn("⚠️  No recording in progress")
                return False

            self._log_info("🛑 Stopping recording...")

            if self.recording_process:
                # Send SIGTERM to rosbag process
                self.recording_process.terminate()

                # Wait for graceful shutdown
                try:
                    self.recording_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._log_warn("⚠️  Force killing recording process...")
                    self.recording_process.kill()
                    self.recording_process.wait()

                self.recording_process = None

            # Log final bag information
            if self.current_bag_name and self.bag_path:
                if os.path.exists(self.bag_path):
                    try:
                        # Get bag size
                        size_cmd = ['du', '-sh', self.bag_path]
                        size_result = subprocess.run(
                            size_cmd, capture_output=True, text=True)
                        bag_size = size_result.stdout.split(
                        )[0] if size_result.returncode == 0 else "Unknown"

                        self._log_info(
                            f"📦 Final bag location: {self.bag_path}")
                        self._log_info(f"💾 Bag size: {bag_size}")

                        # List bag contents
                        ls_cmd = ['ls', '-la', self.bag_path]
                        ls_result = subprocess.run(
                            ls_cmd, capture_output=True, text=True)
                        if ls_result.returncode == 0:
                            self._log_info("📁 Bag contents:")
                            for line in ls_result.stdout.strip().split('\n'):
                                self._log_info(f"   {line}")
                    except Exception as e:
                        self._log_warn(
                            f"⚠️  Could not get bag info: {e}")

            # Store bag name before clearing it
            stopped_bag_name = self.current_bag_name

            self.is_recording = False
            self.current_bag_name = None
            self.bag_path = None

            # Publish status updates
            self.publish_status()

            # Publish detailed info about the stopped recording
            if stopped_bag_name:
                bag_path = os.path.join(self.bagfiles_dir, stopped_bag_name)
                if os.path.exists(bag_path):
                    try:
                        # Get bag size
                        size_cmd = ['du', '-sh', bag_path]
                        size_result = subprocess.run(
                            size_cmd, capture_output=True, text=True)
                        bag_size = size_result.stdout.split(
                        )[0] if size_result.returncode == 0 else "Unknown"

                        # Publish detailed info with bag path and size
                        detailed_info = f"Recording stopped - Bag: {stopped_bag_name} ({bag_size})"
                        self.publish_info(detailed_info)
                    except Exception as e:
                        self._log_warn(
                            f"⚠️  Could not get bag info for publishing: {e}")
                        self.publish_info("Recording stopped")
                else:
                    self.publish_info("Recording stopped")
            else:
                self.publish_info("Recording stopped")

            return True

        except Exception as e:
            self._log_error(f"❌ Error stopping recording: {e}")
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
                    self._log_debug(f"rosbag: {line.strip()}")

            # Check if process ended unexpectedly
            return_code = self.recording_process.wait()
            if return_code != 0:
                self._log_error(
                    f"❌ Recording process ended with code {return_code}")
                self.is_recording = False
                self.publish_status()
                self.publish_info(
                    f"Recording failed (exit code {return_code})")
            else:
                self._log_info("✅ Recording process completed normally")
                self.is_recording = False
                self.publish_status()
                self.publish_info("Recording completed")

        except Exception as e:
            self._log_error(f"❌ Error monitoring recording: {e}")
            self.is_recording = False
            self.publish_status()

    def publish_status(self):
        """Publish current recording status"""
        try:
            status_msg = Bool()
            status_msg.data = self.is_recording
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self._log_error(f"❌ Error publishing status: {e}")

    def publish_info(self, message: str):
        """Publish recording info message"""
        try:
            info_msg = String()
            info_msg.data = message
            self.info_publisher.publish(info_msg)
            self._log_info(f"📡 Published info: {message}")
        except Exception as e:
            self._log_error(f"❌ Error publishing info: {e}")


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
