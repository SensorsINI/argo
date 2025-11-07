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
- MCAP format support (default) with configuration via record.yaml
- Standard ROS2 SQLite3 bags as fallback option

Configuration:
    Configuration is loaded from record.yaml in the same directory as this file.
    - Set storage_format to "mcap" (default) or "sqlite3"
    - Configure MCAP options in the mcap: section
    - Use preset profiles: "fastwrite", "zstd_fast", "zstd_small"

Usage:
    ros2 run argo record.py
    ros2 service call /argo/recording/start std_srvs/srv/Trigger
    ros2 service call /argo/recording/stop std_srvs/srv/Trigger
    ros2 topic echo /argo/recording/status
"""

import os
import sys
import subprocess
import threading
import logging
import yaml
import tempfile
from datetime import datetime

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

# Import the base node (must be after ROS2 imports)
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode


class ArgoRecordingNode(ArgoBaseNode):
    """ROS2 node for managing Argo bag recording"""

    def __init__(self):
        super().__init__('record')

        # Setup file logging for detailed debugging
        self._setup_file_logging()

        # Load recording configuration
        self._load_recording_config()

        # Configuration
        self.bagfiles_dir = os.path.join(os.path.expanduser('~'), 'argo', 'bags')
        self.current_bag_name = None
        self.bag_path = None
        self.recording_process = None
        self.is_recording = False
        self.stopped_bag_name = None
        self.mcap_config_file = None  # Temporary file for MCAP config

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

        self.get_status_service = self.create_service(
            Trigger,
            '/argo/recording/get_status',
            self.get_status_callback
        )

        # ROS2 Publishers
        # Use TRANSIENT_LOCAL durability so subscribers get the last published value immediately
        # This ensures the dashboard knows the current recording state when it subscribes
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.status_publisher = self.create_publisher(
            Bool,
            '/argo/recording/status',
            transient_local_qos
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

        self._log_info("🚀 Argo Recording Node started")
        self._log_info(f"📁 Bagfiles directory: {self.bagfiles_dir}")
        self._log_info(f"💾 Storage format: {self.storage_format}")
        self._log_info("🎮 Services available:")
        self._log_info(
            "   /argo/recording/start - Start recording (Trigger service)")
        self._log_info(
            "   /argo/recording/stop - Stop recording (Trigger service)")
        self._log_info(
            "   /argo/recording/get_status - Get recording status (Trigger service)")
        self._log_info("📡 Topics available:")
        self._log_info("   /argo/recording/status - Recording status (Bool)")
        self._log_info("   /argo/recording/info - Recording info (String)")
        self._log_info(
            "   /argo/recording/control - Recording control (Twist)")

        # Check for required dependencies
        self._check_dependencies()

        # Set initial health status - record node is only healthy when recording
        self.set_unhealthy("Recording inactive")

        # Publish initial status
        self.publish_status()

    def get_status_callback(self, request, response):
        """Service callback to get current recording status."""
        response.success = self.is_recording
        if self.is_recording:
            response.message = f"Recording is active: ~/argo/bags/{self.current_bag_name}"
        else:
            response.message = "Recording is not active"
        self._log_info(
            f"Status query received: {'ACTIVE' if self.is_recording else 'INACTIVE'}")
        return response

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

    def _check_dependencies(self):
        """Check for required dependencies and exit with fatal error if missing."""
        if self.storage_format == 'mcap':
            # Check if MCAP storage plugin is installed
            try:
                # Get ROS_DISTRO from environment
                ros_distro = os.environ.get('ROS_DISTRO', 'humble')
                package_name = f'ros-{ros_distro}-rosbag2-storage-mcap'
                
                # Check if package is installed using dpkg
                # dpkg -l returns 0 if package is found, non-zero if not found
                check_cmd = ['dpkg', '-l', package_name]
                result = subprocess.run(
                    check_cmd,
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                # Check if package is actually installed (dpkg -l returns 0 and output contains package name with 'ii' status)
                is_installed = (result.returncode == 0 and 
                               'ii' in result.stdout and 
                               package_name in result.stdout)
                
                if not is_installed:
                    # Package not installed - fatal error
                    error_msg = (
                        "=" * 60 + "\n"
                        "❌ FATAL: MCAP storage plugin is NOT installed!\n"
                        "\n"
                        "The record node is configured to use MCAP format,\n"
                        "but the required package is missing.\n"
                        "\n"
                        "To install the MCAP storage plugin, run:\n"
                        "  make install-rosbag2-mcap\n"
                        "\n"
                        "Or manually:\n"
                        f"  sudo apt install -y {package_name}\n"
                        "\n"
                        "The node will exit now. Please install the dependency and restart.\n"
                        "=" * 60
                    )
                    self._log_error(error_msg)
                    self.get_logger().fatal(error_msg)
                    # Exit with error code
                    sys.exit(1)
                else:
                    self._log_info(f"✅ MCAP storage plugin is installed ({package_name})")
            except Exception as e:
                # If we can't check, treat it as fatal to be safe
                error_msg = (
                    f"❌ FATAL: Could not check MCAP dependency: {e}\n"
                    "The node cannot verify that MCAP plugin is installed.\n"
                    "Please ensure the dependency is installed and restart the node."
                )
                self._log_error(error_msg)
                self.get_logger().fatal(error_msg)
                sys.exit(1)
    
    def _load_recording_config(self):
        """Load recording configuration from record.yaml"""
        config_path = os.path.join(os.path.dirname(__file__), 'record.yaml')
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Storage format (default: mcap)
            self.storage_format = config.get('storage_format', 'mcap')
            if self.storage_format not in ['mcap', 'sqlite3']:
                self._log_warn(f"⚠️  Invalid storage_format '{self.storage_format}', defaulting to 'mcap'")
                self.storage_format = 'mcap'
            
            # MCAP configuration (only used if storage_format is mcap)
            self.mcap_config = config.get('mcap', {})
            self.preset_profile = config.get('preset_profile', None)
            
            self._log_info(f"✅ Loaded recording config from {config_path}")
            self._log_debug(f"Storage format: {self.storage_format}")
            if self.storage_format == 'mcap':
                if self.preset_profile:
                    self._log_debug(f"MCAP preset profile: {self.preset_profile}")
                else:
                    self._log_debug(f"MCAP config: {self.mcap_config}")
        
        except FileNotFoundError:
            self._log_warn(f"⚠️  Config file not found: {config_path}, using defaults")
            self.storage_format = 'mcap'
            self.mcap_config = {}
            self.preset_profile = None
        except Exception as e:
            self._log_error(f"❌ Error loading config: {e}, using defaults")
            self.storage_format = 'mcap'
            self.mcap_config = {}
            self.preset_profile = None

    def _create_mcap_config_file(self) -> str:
        """Create a temporary YAML file with MCAP writer configuration
        
        Note: This method should only be called when NOT using preset profiles.
        When using preset profiles, --storage-preset-profile is used instead.
        """
        try:
            # Create temporary file
            fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix='mcap_config_', text=True)
            
            with os.fdopen(fd, 'w') as f:
                # Write manual MCAP configuration
                yaml.dump(self.mcap_config, f, default_flow_style=False)
            
            self._log_debug(f"Created temporary MCAP config file: {temp_path}")
            return temp_path
        
        except Exception as e:
            self._log_error(f"❌ Error creating MCAP config file: {e}")
            return None

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
            self._log_debug(
                f"Service response: success={response.success}, message='{response.message}'")
            return response

        self._log_debug("Calling start_recording() method...")
        success = self.start_recording()
        self._log_debug(f"start_recording() returned: {success}")

        if success:
            response.success = True
            response.message = f"Recording started successfully - ~/argo/bags/{self.current_bag_name}"
            self._log_info("✅ Recording started successfully")
        else:
            response.success = False
            response.message = "Failed to start recording"
            self._log_error("❌ Failed to start recording")

        self._log_debug(
            f"Service response: success={response.success}, message='{response.message}'")
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
            response.message = f"Recording stopped successfully - ~/argo/bags/{self.stopped_bag_name}"
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
                self._log_debug(
                    "start_recording() returning False - already recording")
                return False

            # Generate new bag name
            self.current_bag_name = self.generate_bag_name()
            self.bag_path = os.path.join(
                self.bagfiles_dir, self.current_bag_name)
            self._log_debug(f"Generated bag name: {self.current_bag_name}")
            self._log_debug(f"Bag path: {self.bag_path}")

            bag_path_display = f"~/argo/bags/{self.current_bag_name}"
            self._log_info(f"🚀 Starting recording: {bag_path_display}")
            self._log_info(f"📁 Bag path: {bag_path_display}")
            self._log_info(f"🕐 Start time: {datetime.now()}")
            
            # Log available topics before recording starts
            try:
                topics_cmd = ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 topic list']
                topics_result = subprocess.run(topics_cmd, capture_output=True, text=True, timeout=15)
                if topics_result.returncode == 0:
                    topic_count = len([line for line in topics_result.stdout.strip().split('\n') if line.strip()])
                    self._log_info(f"📡 Found {topic_count} topics available for recording")
                    self._log_debug(f"Available topics: {topics_result.stdout.strip()}")
                else:
                    self._log_warn("⚠️  Could not list available topics")
            except Exception as e:
                self._log_warn(f"⚠️  Error listing topics: {e}")

            # Build rosbag recording command with storage format and configuration
            base_cmd = f'source /opt/ros/humble/setup.bash && ros2 bag record -a --include-hidden-topics -o {self.current_bag_name}'
            
            # Add storage format option
            if self.storage_format == 'mcap':
                base_cmd += ' -s mcap'
                
                # Configure MCAP recording
                if self.preset_profile:
                    # Use preset profile (no config file needed)
                    base_cmd += f' --storage-preset-profile {self.preset_profile}'
                    self._log_info(f"📝 Using MCAP preset profile: {self.preset_profile}")
                else:
                    # Create and use config file for manual configuration
                    self.mcap_config_file = self._create_mcap_config_file()
                    if self.mcap_config_file:
                        base_cmd += f' --storage-config-file {self.mcap_config_file}'
                        self._log_info("📝 Using MCAP manual configuration")
                    else:
                        self._log_warn("⚠️  Could not create MCAP config file, using defaults")
            # For sqlite3, no storage format option needed (it's the default)
            elif self.storage_format == 'sqlite3':
                self._log_info("📝 Using standard ROS2 SQLite3 bag format")
            
            cmd = ['bash', '-c', base_cmd]
            self._log_debug(f"Rosbag command: {' '.join(cmd)}")
            self._log_debug("Creating subprocess for rosbag recording...")
            self.recording_process = subprocess.Popen(
                cmd,
                cwd=self.bagfiles_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,  # Capture stderr separately for better error reporting
                universal_newlines=True,
                bufsize=1
            )
            self._log_debug(
                f"Subprocess created with PID: {self.recording_process.pid}")

            self.is_recording = True
            self._log_debug("Setting is_recording = True")

            # Update health status - record node is only healthy when recording
            self.set_healthy(f"Recording active: {self.current_bag_name}")

            # Publish status updates
            self.publish_status()
            self.publish_info(f"Recording started - ~/argo/bags/{self.current_bag_name}")
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

    def _cleanup_on_exit(self):
        """Clean up any remaining resources"""
        try:
            if self.recording_process:
                self._log_info("🧹 Cleaning up recording process...")
                if self.recording_process.poll() is None:  # Process still running
                    self.recording_process.terminate()
                    try:
                        self.recording_process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.recording_process.kill()
                        self.recording_process.wait()
                self.recording_process = None
            
            # Clean up temporary MCAP config file
            if self.mcap_config_file and os.path.exists(self.mcap_config_file):
                try:
                    os.remove(self.mcap_config_file)
                    self._log_debug(f"🧹 Cleaned up temporary MCAP config file: {self.mcap_config_file}")
                except Exception as e:
                    self._log_warn(f"⚠️  Could not remove MCAP config file: {e}")
                self.mcap_config_file = None
        except Exception as e:
            self._log_warn(f"⚠️  Error during cleanup: {e}")

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

            # Clean up temporary MCAP config file
            if self.mcap_config_file and os.path.exists(self.mcap_config_file):
                try:
                    os.remove(self.mcap_config_file)
                    self._log_debug(f"🧹 Cleaned up temporary MCAP config file: {self.mcap_config_file}")
                except Exception as e:
                    self._log_warn(f"⚠️  Could not remove MCAP config file: {e}")
                self.mcap_config_file = None

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

                        bag_path_display = f"~/argo/bags/{self.current_bag_name}"
                        self._log_info(
                            f"📦 Final bag location: {bag_path_display}")
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
            self.stopped_bag_name = self.current_bag_name

            self.is_recording = False
            self.current_bag_name = None
            self.bag_path = None

            # Update health status - record node is only healthy when recording
            self.set_unhealthy("Recording inactive")

            # Publish status updates
            self.publish_status()

            # Publish detailed info about the stopped recording
            if self.stopped_bag_name:
                bag_path = os.path.join(
                    self.bagfiles_dir, self.stopped_bag_name)
                if os.path.exists(bag_path):
                    try:
                        # Get bag size
                        size_cmd = ['du', '-sh', bag_path]
                        size_result = subprocess.run(
                            size_cmd, capture_output=True, text=True)
                        bag_size = size_result.stdout.split(
                        )[0] if size_result.returncode == 0 else "Unknown"

                        # Publish detailed info with bag path and size
                        detailed_info = f"Recording stopped - Bag: ~/argo/bags/{self.stopped_bag_name} ({bag_size})"
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
            stdout_lines = []
            stderr_lines = []
            
            # Read stdout and stderr in parallel
            import threading
            
            def read_output(pipe, output_list, label):
                """Read from pipe and append to output list"""
                try:
                    for line in iter(pipe.readline, ''):
                        if line:
                            line_stripped = line.strip()
                            output_list.append(line_stripped)
                            # Log recording output (can be verbose, so use debug level)
                            self._log_debug(f"rosbag {label}: {line_stripped}")
                except Exception as e:
                    self._log_debug(f"Error reading {label}: {e}")
                finally:
                    pipe.close()
            
            # Start threads to read stdout and stderr
            stdout_thread = threading.Thread(target=read_output, args=(self.recording_process.stdout, stdout_lines, "stdout"))
            stderr_thread = threading.Thread(target=read_output, args=(self.recording_process.stderr, stderr_lines, "stderr"))
            stdout_thread.daemon = True
            stderr_thread.daemon = True
            stdout_thread.start()
            stderr_thread.start()
            
            # Wait for process to complete
            return_code = self.recording_process.wait()
            
            # Wait for threads to finish reading
            stdout_thread.join(timeout=1.0)
            stderr_thread.join(timeout=1.0)
            
            if return_code != 0:
                # Log full error output
                error_msg = f"❌ Recording process ended with code {return_code}"
                self._log_error(error_msg)
                
                # Log stderr output (usually contains the actual error)
                if stderr_lines:
                    self._log_error("📋 Error output from rosbag:")
                    for line in stderr_lines:
                        self._log_error(f"   {line}")
                
                # Log stdout output (may contain additional context)
                if stdout_lines:
                    # Only log stdout if it's not too verbose (last 10 lines)
                    relevant_lines = stdout_lines[-10:] if len(stdout_lines) > 10 else stdout_lines
                    if relevant_lines:
                        self._log_error("📋 Output from rosbag (last lines):")
                        for line in relevant_lines:
                            self._log_error(f"   {line}")
                
                # Combine all error output for the info message
                all_error_output = '\n'.join(stderr_lines) if stderr_lines else '\n'.join(stdout_lines[-5:]) if stdout_lines else "Unknown error"
                self.is_recording = False
                self.set_unhealthy("Recording failed")
                self.publish_status()
                self.publish_info(f"Recording failed (exit code {return_code}): {all_error_output[:200]}")
            else:
                self._log_info("✅ Recording process completed normally")
                self.is_recording = False
                self.set_unhealthy("Recording completed")
                self.publish_status()
                self.publish_info("Recording completed")

        except Exception as e:
            self._log_error(f"❌ Error monitoring recording: {e}")
            self.is_recording = False
            self.set_unhealthy("Recording monitoring error")
            self.publish_status()

    def publish_status(self):
        """Publish current recording status (called periodically every 5s and on state changes)"""
        try:
            status_msg = Bool()
            status_msg.data = self.is_recording
            self.status_publisher.publish(status_msg)
            # Debug log only in debug mode to avoid log spam
            self._log_debug(f"📡 Published recording status: {'ACTIVE' if self.is_recording else 'INACTIVE'}")
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
    # Use ArgoBaseNode's standardized runner
    ArgoBaseNode.run_node(ArgoRecordingNode, args)


if __name__ == '__main__':
    main()
