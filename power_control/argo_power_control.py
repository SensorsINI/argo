#!/usr/bin/env python3
#
# argo_power_control.py
#
# Argo Power Control System
# =====================================
#
# DESCRIPTION:
#   Intelligent power control system for Orange Pi Zero 2W using external relays
#   and GPIO pins. Implements safe shutdown procedures, hardware interrupt-based
#   power button monitoring, and LED status indicators. Power relay control is
#   handled by dedicated startup/shutdown hooks for safety.
#
# HARDWARE CONFIGURATION:
#   - PI3 (Pin 40): !POW - Open drain output to control power relay
#   - PI9 (Pin 28): !POW_BUT - Input from power button (external pullup required)
#   - PH4 (Pin 18): Green LED in power button (system running indicator)
#   - PI1 (Pin 12): Blue LED in power button (status/warning indicator)
#   - Red LED: Directly connected to power button (not GPIO controlled)
#
# POWER CIRCUIT DESIGN:
#   The system uses two PFETs to control the power relay:
#   1. Direct PFET: Connected to power button for immediate power-on
#   2. GPIO PFET: Connected to !POW pin for software-controlled power management
#
#   When !POW is pulled low (open drain), the relay is energized, providing power.
#   When !POW goes high, the relay is de-energized, cutting power.
#
# POWER BUTTON BEHAVIOR:
#   - Quadruple tap: Toggle Argo service
#   - Double tap: Toggle recording
#   - Long press (>= threshold DEFAULT_SHUTDOWN_THRESHOLD_S): Initiate shutdown sequence
#   - Only new button presses after service startup are detected (prevents shutdown during boot)
#
# LED INDICATORS:
#   - Green LED: Heartbeat when system is running
#   - Both LEDs: Countdown pattern during button press (5,4,3,2,1 flashes at 3Hz, then 10 rapid flashes at 10Hz)
#   - Both LEDs: Alternating short-long pattern during shutdown sequence
#   - Red LED: Directly connected to power button (not controlled by GPIO)
#
# SHUTDOWN SEQUENCE:
#   1. Broadcast wall message to all users
#   2. Wait 5 seconds for users to see notification
#   3. Start alternating short-long LED pattern (shutdown indicator)
#   4. Execute 'sudo shutdown -h now' for graceful shutdown
#   5. argo_poweroff.shutdown hook handles final power relay control during shutdown
#
# USAGE:
#   ./argo_power_control.py [--help] [--test-mode] [--threshold SECONDS]
#
# OPTIONS:
#   --help              Show this help message and exit
#   --test-mode         Run in test mode (disable actual shutdown and power control, reports actions)
#   --threshold SECONDS Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
#
# EXAMPLES:
#   ./argo_power_control.py                         # Normal operation
#   ./argo_power_control.py --test-mode             # Test mode (safe, reports button actions)
#   ./argo_power_control.py --threshold 2.0         # 2-second threshold
#
# REQUIREMENTS:
#   - User must be root or member of 'gpio' group (for GPIO access)
#   - python3-gpiod library installed
#   - External pullup resistor on power button input
#   - Proper hardware connections as described above
#
# SAFETY FEATURES:
#   - Power relay control separated from this service to prevent accidental power cuts
#   - Graceful shutdown ensures proper system shutdown before cutting power
#   - External pullup resistor requirement prevents floating inputs
#   - GPIO group membership requirement for controlled GPIO access
#
# AUTHOR: Generated for Orange Pi Zero 2W Power Control System
# VERSION: 1.0
# DATE: September 2024

import gpiod
import time
import signal
import sys
import subprocess
import threading
import logging
import os
import argparse
from pathlib import Path
from datetime import datetime
import select
import tty
import termios

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
# Removed QoS imports - using default QoS only

# =============================================================================
# CONSTANTS
# =============================================================================

# ANSI Color codes for LED visualization


class Colors:
    GREEN = '\033[92m'      # Green LED
    BLUE = '\033[94m'       # Blue LED
    RED = '\033[91m'        # Red LED (not GPIO controlled)
    YELLOW = '\033[93m'     # Warning/Info
    CYAN = '\033[96m'       # Info
    MAGENTA = '\033[95m'    # Debug
    WHITE = '\033[97m'      # Normal text
    BLACK = '\033[30m'      # Black text
    BOLD = '\033[1m'        # Bold text
    RESET = '\033[0m'       # Reset all formatting
    BG_GREEN = '\033[42m'   # Green background
    BG_BLUE = '\033[44m'    # Blue background
    BG_RED = '\033[41m'     # Red background


# Button Press Configuration
# Default button hold time for shutdown (seconds)
DEFAULT_SHUTDOWN_THRESHOLD_S = 5.0
# Warn users this many seconds before shutdown to release the button
SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S = 3.0
# Button release polling frequency during press (10 Hz - only used during button press)
BUTTON_ERROR_RECOVERY_DELAY_S = 0.1     # Delay on button read error (seconds)
# Maximum duration for multiple taps to toggle states (seconds)
MULTI_TAP_MAX_DURATION_S = 3

# LED Heartbeat Configuration
LED_HEARTBEAT_HZ = 1.0                  # Green LED heartbeat frequency (1 Hz)

# Button Press Pattern Configuration
LED_PRESS_START_FREQUENCY_HZ = 2.0      # Starting flash frequency (2 Hz)
LED_PRESS_END_FREQUENCY_HZ = 20.0       # Ending flash frequency (20 Hz)
LED_PRESS_DUTY_CYCLE = 0.5              # 50% duty cycle during button press

# Shutdown Pattern Configuration
LED_SHUTDOWN_FREQUENCY_HZ = 1.0         # Shutdown flash frequency (1 Hz)
LED_SHUTDOWN_DUTY_CYCLE = 0.05           # 5% duty cycle during shutdown

# Shutdown Sequence Timing
# No delay - shutdown immediately after wall message
# Simulated shutdown delay in test mode (seconds)
TEST_MODE_SHUTDOWN_DELAY_S = 5

# Notification Timeouts
# Desktop notification timeout (seconds)
DESKTOP_NOTIFICATION_TIMEOUT_S = 5
WALL_MESSAGE_TIMEOUT_S = 5              # Wall message timeout (seconds)
# Standard notification expire time (5 seconds)
NOTIFICATION_EXPIRE_TIME_MS = 5000
# Final shutdown notification (no timeout - stays until dismissed)
FINAL_SHUTDOWN_NOTIFICATION_MS = 0

# Main Loop Timing
# Main control loop sleep interval (seconds)
MAIN_LOOP_SLEEP_S = 1

# Battery Monitoring
LOW_BATTERY_THRESHOLD_V = 7.6      # Low battery warning threshold (SOS LED pattern)
CRITICAL_BATTERY_THRESHOLD_V = 7.2  # Critical battery voltage threshold (halt system)
BATTERY_MONITORING_INTERVAL_S = 30  # Check battery voltage interval (seconds)
# Flag file for shutdown hook
CRITICAL_BATTERY_FLAG_FILE = '/tmp/argo_critical_battery'

# Configure logging


def setup_logging():
    """Setup logging configuration"""
    handlers = [logging.StreamHandler()]

    # Only add file handler if we have write permission to /var/log/
    try:
        # Test if we can write to /var/log/
        test_file = '/var/log/argo_power_control.log'
        with open(test_file, 'a') as f:
            pass
        handlers.append(logging.FileHandler(test_file))
    except (PermissionError, OSError):
        # Fall back to local log file or no file logging
        try:
            handlers.append(logging.FileHandler('argo_power_control.log'))
        except (PermissionError, OSError):
            pass  # No file logging if we can't write anywhere

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=handlers
    )


setup_logging()
logger = logging.getLogger('argo_power_control')


class PowerController:
    def __init__(self, test_mode=False, threshold=1.0):
        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        self.gpio_available = False  # Will be set to True if GPIO initialization succeeds

        # Button state tracking
        self.initial_button_state = None
        # Flag to track when button detection should be active
        self.button_detection_active = False
        # Flag to track if warning notification has been sent for current button press
        self.warning_notification_sent = False

        # Tap detection for various commands
        self.tap_times = []  # List to store tap timestamps
        self.tap_timeout_timer = None  # Timer for tap detection timeout
        self.tap_timeout_duration = 0.5  # Wait 500ms after last tap before processing

        # Argo service status tracking
        self.argo_service_running = False

        # LED state tracking
        self.green_led_state = False
        self.blue_led_state = False

        # Desktop notification caching
        self.cached_desktop_user = None
        self.cached_display_env = None
        self.desktop_user_detection_failed = False

        # Battery monitoring state
        self.low_battery_detected = False
        self.critical_battery_detected = False
        self.last_battery_check_time = 0.0
        self.battery_monitoring_active = False
        self.sos_led_active = False

        # GPIO Configuration
        self.GPIO_CHIP = '/dev/gpiochip0'

        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 265   # PI9 (Pin 28) - !POW_BUT
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED
        self.BLUE_LED_LINE = 257       # PI1 (Pin 12) - Blue LED

        # Button press threshold (seconds)
        self.SHUTDOWN_THRESHOLD = threshold

        # No ROS2 setup needed - using subprocess calls only

        # Setup signal handlers for graceful service shutdown
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGQUIT, self.signal_handler)

        # Check if Argo service is already running at startup
        logger.info("Checking if Argo service is already running...")
        self.check_argo_service_status()
        if self.argo_service_running:
            logger.info(
                "Argo service is already running - checking recording service availability...")
            # Give ROS2 a moment to discover services
            import time
            time.sleep(2)
            self.check_recording_service_availability()
            # Query current recording status at startup
            self.query_current_recording_status()

        logger.info("Power controller initialized")

        # Initialize desktop user detection and caching
        self._detect_and_cache_desktop_user()

        # Initialize GPIO first to read initial button state
        self.init_gpio()

        # Record initial button state during boot (must be done after GPIO init but before event config)
        self.initial_button_state = self.read_initial_button_state()
        logger.info(
            f"Initial button state recorded: {self.initial_button_state}")

        # Set button detection flag based on initial state
        if self.initial_button_state == 1:  # Button not pressed at startup
            self.button_detection_active = True  # Can detect presses immediately
            logger.info("Button detection active - ready to detect presses")
        else:  # Button pressed at startup
            self.button_detection_active = False  # Wait for first release
            logger.info(
                "Button pressed at startup - waiting for release before detection starts")

        # Now reconfigure the button line for interrupt monitoring
        self.configure_button_for_interrupts()

    def _detect_and_cache_desktop_user(self):
        """Detect and cache desktop user and display information"""
        try:
            logger.info(
                "Detecting desktop user and display for notification caching...")

            # Find the user who owns the desktop session
            desktop_user = None
            display_env = None

            try:
                # Look for logged-in users to find the desktop user and display
                result = subprocess.run(
                    ['who'], capture_output=True, text=True, timeout=2)
                logger.debug(f"who command output: {result.stdout.strip()}")
                if result.returncode == 0:
                    for line in result.stdout.strip().split('\n'):
                        if line and ':' in line and not '(:' in line:
                            parts = line.split()
                            for part in parts:
                                if part.startswith(':'):
                                    # Remove the leading ':'
                                    display = part[1:]
                                    if display.isdigit() or '.' in display:
                                        # Found a display, get the user from the line
                                        if len(parts) > 0:
                                            potential_user = parts[0]
                                            if potential_user != 'root':
                                                desktop_user = potential_user
                                                display_env = f':{display}'
                                                logger.info(
                                                    f"Detected desktop_user: {desktop_user}, display_env: {display_env}")
                                                break
                            if desktop_user:
                                break
                        elif line and '(:' in line and ')' in line:
                            # Handle format like "orangepi tty7 2025-10-03 13:05 (:0)"
                            parts = line.split()
                            if len(parts) > 0:
                                potential_user = parts[0]
                                # Extract display from parentheses
                                for part in parts:
                                    if part.startswith('(:') and part.endswith(')'):
                                        # Remove (: and )
                                        display = part[2:-1]
                                        if display.isdigit() or '.' in display:
                                            if potential_user != 'root':
                                                desktop_user = potential_user
                                                display_env = f':{display}'
                                                logger.info(
                                                    f"Detected desktop_user: {desktop_user}, display_env: {display_env}")
                                                break
                            if desktop_user:
                                break
            except Exception as e:
                logger.info(f"Could not determine desktop user from who: {e}")

            # Fallback to the user who invoked sudo or the current user
            if not desktop_user:
                # When running as root, try to find the actual desktop user
                if os.geteuid() == 0:  # Running as root
                    # Try to find the user who owns the X session
                    try:
                        result = subprocess.run(
                            ['ps', 'aux'], capture_output=True, text=True, timeout=2)
                        if result.returncode == 0:
                            for line in result.stdout.split('\n'):
                                if 'Xorg' in line or 'X' in line:
                                    parts = line.split()
                                    if len(parts) > 0:
                                        potential_user = parts[0]
                                        if potential_user != 'root':
                                            desktop_user = potential_user
                                            logger.info(
                                                f"Found desktop user from ps: {desktop_user}")
                                            break
                    except Exception as e:
                        logger.info(
                            f"Could not find desktop user from ps: {e}")

                # Final fallback
                if not desktop_user:
                    desktop_user = os.environ.get('SUDO_USER') or os.environ.get(
                        'USER') or os.environ.get('LOGNAME') or 'orangepi'
                    logger.info(f"Using fallback desktop_user: {desktop_user}")

            if not display_env:
                display_env = ':0'  # Default display
                logger.info(f"Using default display_env: {display_env}")

            # Cache the detected values
            self.cached_desktop_user = desktop_user
            self.cached_display_env = display_env
            self.desktop_user_detection_failed = False

            logger.info(
                f"Cached desktop user: {self.cached_desktop_user}, display: {self.cached_display_env}")

        except Exception as e:
            logger.warning(f"Failed to detect desktop user and display: {e}")
            self.desktop_user_detection_failed = True
            # Set fallback values
            self.cached_desktop_user = os.environ.get(
                'SUDO_USER') or os.environ.get('USER') or 'orangepi'
            self.cached_display_env = ':0'

    def init_gpio(self):
        """Initialize GPIO pins"""
        try:
            logger.info(f"Attempting to open GPIO chip: {self.GPIO_CHIP}")
            self.chip = gpiod.Chip(self.GPIO_CHIP)
            logger.info(f"Successfully opened GPIO chip: {self.GPIO_CHIP}")

            # Get line objects
            # NOTE: power_relay_line (GPIO 259) is NOT controlled by this service
            # Power relay control is handled exclusively by the shutdown hook
            self.power_button_line = self.chip.get_line(self.POWER_BUTTON_LINE)
            self.green_led_line = self.chip.get_line(self.GREEN_LED_LINE)
            self.blue_led_line = self.chip.get_line(self.BLUE_LED_LINE)

            # Initially request power button line as input to read initial state
            # Will be reconfigured for interrupts after reading initial state
            self.power_button_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
            )

            # Request LED lines
            self.green_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]  # Start with LED off
            )
            self.blue_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]  # Start with LED off
            )

            logger.info("GPIO pins configured successfully")
            self.gpio_available = True
        except Exception as e:
            if self.test_mode:
                logger.warning(f"GPIO not available in test mode: {e}")
                logger.warning(
                    "Continuing in test mode without GPIO functionality")
                self.gpio_available = False
                # Set dummy values for test mode
                self.chip = None
                self.power_button_line = None
                self.green_led_line = None
                self.blue_led_line = None
            else:
                logger.error(f"Failed to initialize GPIO: {e}")
                logger.error(f"GPIO chip path attempted: {self.GPIO_CHIP}")
                logger.error(f"Full error details: {type(e).__name__}: {e}")
                raise

        # No ROS2 setup needed - using subprocess calls only

        # Simple state tracking - initialize these early for test mode compatibility
        self.recording_active = False
        self.argo_service_running = False
        self.recording_service_available = False

    def start_recording(self):
        """Start recording via ROS2 service using subprocess call"""
        if self.recording_active:
            logger.warning("Recording is active - skipping start recording")
            self.send_desktop_notification(
                "Recording Error",
                "Recording is active - skipping start recording",
                "warning"
            )
            return False

        try:
            # Check if Argo service is running (like ar alias does)
            result = subprocess.run(
                ['sudo', 'systemctl', 'is-active', 'argo-launch.service'],
                capture_output=True, text=True, timeout=4
            )
            if result.returncode != 0 or result.stdout.strip() != 'active':
                logger.error(
                    "Argo service is not running - cannot start recording")
                self.send_desktop_notification(
                    "Recording Error",
                    "Argo service is not running - start it first with: al",
                    "critical"
                )
                return False

            # Call the service using subprocess with proper ROS2 environment (like ar alias)
            logger.info("🎬 Calling recording start service via subprocess...")

            # Use subprocess to call the ROS2 service with proper environment sourcing (like ar alias)
            cmd = [
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && ros2 service call /argo/recording/start std_srvs/srv/Trigger'
            ]

            logger.info(f"Executing command: {' '.join(cmd)}")
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30.0
            )

            logger.info(
                f"Service call completed - return code: {result.returncode}")
            logger.info(f"stdout: {result.stdout}")
            if result.stderr:
                logger.info(f"stderr: {result.stderr}")

            if result.returncode == 0:
                # Parse the response from stdout - handle both formats
                success = False
                message = "Recording started successfully"

                # Check for success in different formats
                if "success: True" in result.stdout or "success=True" in result.stdout:
                    success = True
                elif "success: False" in result.stdout or "success=False" in result.stdout:
                    success = False
                else:
                    # Default to success if return code is 0 and no explicit failure
                    success = True

                if success:
                    # Extract message from response
                    if "message:" in result.stdout:
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message:" in line:
                                message = line.split("message:")[
                                    1].strip().strip('"')
                                break
                    elif "message=" in result.stdout:
                        # Handle std_srvs.srv.Trigger_Response format
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message=" in line:
                                # Extract from message='...' format
                                start = line.find("message='") + 9
                                end = line.find("'", start)
                                if start > 8 and end > start:
                                    message = line[start:end]
                                break

                    logger.info(f"✅ Recording started: {message}")
                    self.recording_active = True  # Update state
                    self.send_desktop_notification(
                        "Recording Started",
                        message,
                        "normal"
                    )
                    return True
                else:
                    # Extract error message
                    error_msg = "Recording start failed"
                    if "message:" in result.stdout:
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message:" in line:
                                error_msg = line.split("message:")[
                                    1].strip().strip('"')
                                break
                    elif "message=" in result.stdout:
                        # Handle std_srvs.srv.Trigger_Response format
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message=" in line:
                                # Extract from message='...' format
                                start = line.find("message='") + 9
                                end = line.find("'", start)
                                if start > 8 and end > start:
                                    error_msg = line[start:end]
                                break

                    # Check if recording is already in progress
                    if "already in progress" in error_msg.lower() or "already active" in error_msg.lower():
                        logger.info(
                            f"📹 Recording already in progress: {error_msg}")
                        self.recording_active = True  # Update state to match reality
                        self.send_desktop_notification(
                            "Recording Status",
                            f"Recording was already active: {error_msg}",
                            "normal"
                        )
                        return True  # Return success since recording is active
                    else:
                        logger.error(f"❌ Recording start failed: {error_msg}")
                        self.send_desktop_notification(
                            "Recording Error",
                            error_msg,
                            "critical"
                        )
                        return False
            else:
                logger.error(
                    f"Service call failed with return code {result.returncode}")
                logger.error(f"Error output: {result.stderr}")
                self.send_desktop_notification(
                    "Recording Error",
                    f"Recording start service call failed: {result.stderr}",
                    "critical"
                )
                return False

        except subprocess.TimeoutExpired:
            logger.error("Service call timed out after 15 seconds")
            self.send_desktop_notification(
                "Recording Error",
                "Recording start service call timed out after 15 seconds",
                "critical"
            )
            return False
        except Exception as e:
            logger.error(f"Error starting recording: {e}")
            self.send_desktop_notification(
                "Recording Error",
                f"Error starting recording: {e}",
                "critical"
            )
            return False

    def stop_recording(self):
        """Stop recording via ROS2 service using subprocess call"""
        if not self.recording_active:
            logger.warning("Recording is not active - skipping stop recording")
            self.send_desktop_notification(
                "Recording Error",
                "Recording is not active - skipping stop recording",
                "warning"
            )
            return False

        try:
            # Check if recording service is available using our flag
            if not self.recording_service_available:
                logger.warning(
                    "Recording service not available - Argo system may not be running")
                return False

            # Call the service using subprocess with proper ROS2 environment
            logger.info("⏹️ Calling recording stop service via subprocess...")

            # Use subprocess to call the ROS2 service with proper environment sourcing (like ac alias)
            cmd = [
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && ros2 service call /argo/recording/stop std_srvs/srv/Trigger'
            ]

            logger.info(f"Executing command: {' '.join(cmd)}")
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30.0
            )

            logger.info(
                f"Service call completed - return code: {result.returncode}")
            logger.info(f"stdout: {result.stdout}")
            if result.stderr:
                logger.info(f"stderr: {result.stderr}")

            if result.returncode == 0:
                # Parse the response from stdout - handle both formats
                success = False
                message = "Recording stopped successfully"

                # Check for success in different formats
                if "success: True" in result.stdout or "success=True" in result.stdout:
                    success = True
                elif "success: False" in result.stdout or "success=False" in result.stdout:
                    success = False
                else:
                    # Default to success if return code is 0 and no explicit failure
                    success = True

                if success:
                    # Extract message from response
                    if "message:" in result.stdout:
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message:" in line:
                                message = line.split("message:")[
                                    1].strip().strip('"')
                                break
                    elif "message=" in result.stdout:
                        # Handle std_srvs.srv.Trigger_Response format
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message=" in line:
                                # Extract from message='...' format
                                start = line.find("message='") + 9
                                end = line.find("'", start)
                                if start > 8 and end > start:
                                    message = line[start:end]
                                break

                    logger.info(f"✅ Recording stopped: {message}")
                    self.recording_active = False  # Update state
                    self.send_desktop_notification(
                        "Recording Stopped",
                        message,
                        "normal"
                    )
                    return True
                else:
                    # Extract error message
                    error_msg = "Recording stop failed"
                    if "message:" in result.stdout:
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message:" in line:
                                error_msg = line.split("message:")[
                                    1].strip().strip('"')
                                break
                    elif "message=" in result.stdout:
                        # Handle std_srvs.srv.Trigger_Response format
                        lines = result.stdout.split('\n')
                        for line in lines:
                            if "message=" in line:
                                # Extract from message='...' format
                                start = line.find("message='") + 9
                                end = line.find("'", start)
                                if start > 8 and end > start:
                                    error_msg = line[start:end]
                                break

                    logger.error(f"❌ Recording stop failed: {error_msg}")
                    self.send_desktop_notification(
                        "Recording Error",
                        error_msg,
                        "critical"
                    )
                    return False
            else:
                logger.error(
                    f"Service call failed with return code {result.returncode}")
                logger.error(f"Error output: {result.stderr}")
                self.send_desktop_notification(
                    "Recording Error",
                    f"Recording stop service call failed: {result.stderr}",
                    "critical"
                )
                return False

        except subprocess.TimeoutExpired:
            logger.error("Service call timed out after 15 seconds")
            self.send_desktop_notification(
                "Recording Error",
                "Recording stop service call timed out after 15 seconds",
                "critical"
            )
            return False
        except Exception as e:
            logger.error(f"Error stopping recording: {e}")
            self.send_desktop_notification(
                "Recording Error",
                f"Error stopping recording: {e}",
                "critical"
            )
            return False

    def check_recording_service_availability(self):
        """Check if recording service is available and update flag"""
        # In test mode, always check the actual service status
        if self.test_mode:
            # Check if Argo service is actually running
            try:
                result = subprocess.run(
                    ['sudo', 'systemctl', 'is-active', 'argo-launch.service'],
                    capture_output=True, text=True, timeout=4
                )
                is_running = result.returncode == 0 and result.stdout.strip() == 'active'
                if is_running:
                    self.argo_service_running = True
                    logger.info(
                        "Argo service detected as running in test mode")
                else:
                    self.argo_service_running = False
                    logger.info("Argo service not running in test mode")
            except Exception as e:
                logger.warning(
                    f"Error checking Argo service status in test mode: {e}")
                self.argo_service_running = False

        if not self.argo_service_running:
            self.recording_service_available = False
            logger.info(
                "Recording service is not available because Argo service is not running")
            return False

        # Test if the recording service is available using subprocess with retry
        max_retries = 3
        retry_delay = 2  # seconds

        for attempt in range(max_retries):
            try:
                # Use subprocess to test if the service is available with proper ROS2 environment
                result = subprocess.run([
                    'bash', '-c',
                    'source /opt/ros/humble/setup.bash && ros2 service list'
                ], capture_output=True, text=True, timeout=5)

                logger.info(
                    f"ROS2 service list command result (attempt {attempt + 1}/{max_retries}) - return code: {result.returncode}")
                if attempt == 0:  # Only log full output on first attempt to avoid spam
                    logger.info(f"ROS2 service list stdout: {result.stdout}")
                    if result.stderr:
                        logger.info(
                            f"ROS2 service list stderr: {result.stderr}")

                if result.returncode == 0 and '/argo/recording/start' in result.stdout:
                    self.recording_service_available = True
                    logger.info("Recording service is available")
                    return True
                else:
                    if attempt < max_retries - 1:
                        logger.info(
                            f"Recording service not found, retrying in {retry_delay} seconds...")
                        time.sleep(retry_delay)
                    else:
                        self.recording_service_available = False
                        logger.info(
                            "Recording service is not available - service not found in service list after all retries")
                        return False
            except Exception as e:
                if attempt < max_retries - 1:
                    logger.warning(
                        f"Error checking recording service availability (attempt {attempt + 1}): {e}, retrying...")
                    time.sleep(retry_delay)
                else:
                    self.recording_service_available = False
                    logger.info(
                        f"Error checking recording service availability after all retries: {e}")
                    return False

    def query_current_recording_status(self):
        """Query current recording status at startup to sync with actual state"""
        if not self.recording_service_available:
            logger.info(
                "Recording service not available - skipping status query")
            return

        try:
            # Query the current recording status using subprocess
            result = subprocess.run([
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /argo/recording/status --once'
            ], capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                # Parse the output to get the recording status
                old_state = self.recording_active

                if "data: true" in result.stdout:
                    self.recording_active = True
                    if old_state != self.recording_active:
                        logger.info(
                            "📹 Recording status changed: INACTIVE → ACTIVE (state synchronized)")
                    else:
                        logger.debug("Recording status: ACTIVE (confirmed)")
                elif "data: false" in result.stdout:
                    self.recording_active = False
                    if old_state != self.recording_active:
                        logger.info(
                            "📹 Recording status changed: ACTIVE → INACTIVE (state synchronized)")
                    else:
                        logger.debug("Recording status: INACTIVE (confirmed)")
                else:
                    logger.warning(
                        f"Could not parse recording status from: {result.stdout}")
            else:
                logger.warning(
                    f"Failed to query recording status: {result.stderr}")
        except Exception as e:
            logger.warning(f"Error querying current recording status: {e}")

    def is_argo_system_running(self):
        """Check if the Argo system is running using simple flag approach"""
        # Check recording service availability if Argo service is running
        if self.argo_service_running:
            self.check_recording_service_availability()

        logger.info(
            f"Argo service running: {self.argo_service_running}, Recording service available: {self.recording_service_available}")
        return self.argo_service_running and self.recording_service_available

    def check_argo_service_status(self):
        """Check if the Argo launch service is running"""
        try:
            result = subprocess.run(
                ['sudo', 'systemctl', 'is-active', 'argo-launch.service'],
                capture_output=True, text=True, timeout=4
            )
            is_running = result.returncode == 0 and result.stdout.strip() == 'active'
            if is_running != self.argo_service_running:
                self.argo_service_running = is_running
                logger.info(
                    f"Argo service status changed: {'RUNNING' if is_running else 'STOPPED'}")

                # When Argo service stops, recording service is not available
                if not is_running:
                    self.recording_service_available = False
                    logger.info("Recording service marked as unavailable")

            return is_running
        except Exception as e:
            logger.debug(f"Error checking Argo service status: {e}")
            return False

    def start_argo_service(self):
        """Start the Argo launch service"""
        try:
            if self.test_mode:
                logger.info("TEST MODE: Would start Argo launch service")
                return True
            logger.info("Starting Argo launch service...")
            result = subprocess.run(
                ['sudo', 'systemctl', 'start', 'argo-launch.service'],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                logger.info(
                    "✅ Argo launch service started, waiting for recording service to initialize...")
                # Set Argo service flag, but recording service needs time to initialize
                self.argo_service_running = True
                # pause to allow record.py to start
                time.sleep(7)
                # check recording service availability in loop until it is available
                self.check_recording_service_availability()
                while not self.recording_service_available:
                    logger.info(
                        "Recording service not available, waiting for initialization...")
                    time.sleep(1)
                    self.check_recording_service_availability()
                logger.info(
                    "Recording service initialized, setting available flag")
                self.recording_service_available = True
                return True
            else:
                logger.error(
                    f"❌ Failed to start Argo service: {result.stderr}")
                return False
        except Exception as e:
            logger.error(f"Error starting Argo service: {e}")
            return False

    def stop_argo_service(self):
        """Stop the Argo launch service"""
        try:
            if self.test_mode:
                logger.info("TEST MODE: Would stop Argo launch service")
                return True

            result = subprocess.run(
                ['sudo', 'systemctl', 'stop', 'argo-launch.service'],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode == 0:
                logger.info("✅ Argo launch service stopped")
                # Set flags to indicate Argo system is stopped
                self.argo_service_running = False
                self.recording_service_available = False
                return True
            else:
                logger.error(
                    f"❌ Failed to stop Argo service: {result.stderr}")
                return False
        except Exception as e:
            logger.error(f"Error stopping Argo service: {e}")
            return False

    def toggle_argo_service(self):
        """Toggle the Argo launch service on/off"""
        try:
            # First check the actual service status to ensure our flag is accurate
            actual_status = self.check_argo_service_status()

            if actual_status and self.argo_service_running:
                logger.info("Double tap detected - stopping Argo service")
                success = self.stop_argo_service()
                if success:
                    self.send_desktop_notification(
                        "Argo Service Stopped",
                        "Argo launch service stopped by double tap",
                        "normal"
                    )
                else:
                    self.send_desktop_notification(
                        "Argo Service Error",
                        "Failed to stop Argo service",
                        "critical"
                    )
            elif not actual_status and not self.argo_service_running:
                logger.info("Double tap detected - starting Argo service")
                success = self.start_argo_service()
                if success:
                    self.send_desktop_notification(
                        "Argo Service Started",
                        "Argo launch service started by double tap",
                        "normal"
                    )
                else:
                    self.send_desktop_notification(
                        "Argo Service Error",
                        "Failed to start Argo service",
                        "critical"
                    )
            else:
                # Service status mismatch - update our flag and try again
                logger.info(
                    f"Service status mismatch - actual: {actual_status}, flag: {self.argo_service_running}")
                self.argo_service_running = actual_status
                if actual_status:
                    logger.info(
                        "Argo service is already running - no action needed")
                    self.send_desktop_notification(
                        "Argo Service Status",
                        "Argo service is already running",
                        "normal"
                    )
                else:
                    logger.info("Argo service is not running - starting it")
                    success = self.start_argo_service()
                    if success:
                        self.send_desktop_notification(
                            "Argo Service Started",
                            "Argo launch service started by double tap",
                            "normal"
                        )
                    else:
                        self.send_desktop_notification(
                            "Argo Service Error",
                            "Failed to start Argo service",
                            "critical"
                        )
        except Exception as e:
            logger.error(f"Error toggling Argo service: {e}")
            self.send_desktop_notification(
                "Argo Service Error",
                f"Error toggling Argo service: {e}",
                "critical"
            )

    def signal_handler(self, signum, frame):
        """Handle shutdown signals with graceful termination"""
        logger.info(
            f"Received signal {signum}, initiating graceful shutdown...")
        self.running = False

        # Give the main loop a few seconds to exit gracefully
        logger.info("Waiting for graceful shutdown (5 seconds)...")
        time.sleep(5)

        # If we're still here, force exit
        logger.warning("Graceful shutdown timeout - forcing exit")
        try:
            self.cleanup()
        except Exception as e:
            logger.error(f"Error during forced cleanup: {e}")

        logger.info("Forcing process exit")
        os._exit(0)

    def read_initial_button_state(self):
        """Read initial power button state during startup (before interrupt configuration)"""
        if not self.gpio_available:
            logger.info(
                "GPIO not available - assuming button released in test mode")
            return 1  # Assume released in test mode
        try:
            state = self.power_button_line.get_value()
            logger.info(
                f"Initial button state read: {state} ({'pressed' if state == 0 else 'released'})")
            return state
        except Exception as e:
            logger.error(f"Error reading initial button state: {e}")
            return 1  # Assume released on error

    def configure_button_for_interrupts(self):
        """Reconfigure button line for interrupt-based monitoring"""
        if not self.gpio_available:
            logger.info(
                "GPIO not available - skipping button interrupt configuration in test mode")
            return
        try:
            # Release the current configuration
            self.power_button_line.release()

            # Reconfigure for falling edge interrupts
            # Currently configured for falling edge (POW_BUT going low)
            # TODO: Change to gpiod.LINE_REQ_EV_RISING_EDGE for future PCB changes where POW_BUT goes high on press
            self.power_button_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_EV_FALLING_EDGE,  # Detect POW_BUT going low
                flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
            )
            logger.info(
                "Button line reconfigured for falling edge interrupt monitoring")

        except Exception as e:
            logger.error(f"Error configuring button for interrupts: {e}")
            raise

    def read_power_button(self):
        """Read power button state - NOTE: Limited use with interrupt-based monitoring"""
        # This method is kept for compatibility but has limited use in interrupt mode
        # In interrupt mode, we primarily rely on events, but this can be used for release detection
        try:
            return self.power_button_line.get_value()
        except Exception as e:
            logger.error(f"Error reading power button: {e}")
            return 1  # Assume not pressed on error

    # def set_power_relay(self, state):
    #     """Control power relay (True = energized/on, False = de-energized/off)"""
    #     """
    #     DISABLED: Power relay control removed from this service for safety.
    #     Power relay (GPIO line 259) is now controlled exclusively by:
    #     - argo_poweron.startup: Sets relay ON during boot
    #     - argo_poweroff.shutdown: Sets relay OFF during shutdown only
    #     This prevents accidental power cuts during normal operation or reboots.
    #     """
    #     try:
    #         # For open drain: 0 = energized (relay on), 1 = de-energized (relay off)
    #         value = 0 if state else 1
    #         self.power_relay_line.set_value(value)
    #         logger.info(f"Power relay set to {'ON' if state else 'OFF'}")
    #     except Exception as e:
    #         logger.error(f"Error controlling power relay: {e}")
    pass

    def set_green_led(self, state):
        """Control green LED (system running indicator)"""
        if not self.gpio_available:
            logger.debug(
                f"GPIO not available - simulating green LED {'ON' if state else 'OFF'} in test mode")
            self.green_led_state = state
            return
        try:
            value = 1 if state else 0
            self.green_led_line.set_value(value)
            self.green_led_state = state
            logger.debug(f"Green LED set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED (status/warning indicator)"""
        if not self.gpio_available:
            logger.debug(
                f"GPIO not available - simulating blue LED {'ON' if state else 'OFF'} in test mode")
            self.blue_led_state = state
            return
        try:
            value = 1 if state else 0
            self.blue_led_line.set_value(value)
            self.blue_led_state = state
            logger.info(f"Blue LED set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling blue LED: {e}")

    def test_recording_function(self):
        """Test the recording function with visual feedback"""
        if True:
            return

        print(f"\n{Colors.MAGENTA}🎬 Testing Recording Function{Colors.RESET}")
        print(f"{Colors.CYAN}Starting recording simulation...{Colors.RESET}")

        # Simulate recording start with LED patterns
        for i in range(3):
            self.set_green_led(True)
            self.set_blue_led(True)
            time.sleep(0.2)
            self.set_green_led(False)
            self.set_blue_led(False)
            time.sleep(0.2)

        print(f"{Colors.GREEN}✅ Recording started (simulated){Colors.RESET}")
        print(f"{Colors.YELLOW}Recording indicators: Both LEDs flashing{Colors.RESET}")

        # Simulate recording in progress
        for i in range(10):
            self.set_green_led(True)
            time.sleep(0.5)
            self.set_green_led(False)
            time.sleep(0.5)
            print(f"{Colors.CYAN}Recording... {i+1}/10{Colors.RESET}",
                  end="\r", flush=True)

        print(f"\n{Colors.RED}⏹️  Recording stopped (simulated){Colors.RESET}")
        print(f"{Colors.CYAN}Recording test completed{Colors.RESET}\n")

    def led_blink_pattern(self, led_func, duration, blink_frequency_hz):
        """Blink LED with specified pattern

        Args:
            led_func: Function to call to control LED state
            duration: How long to blink (seconds)
            blink_frequency_hz: Blink frequency in Hz
        """
        start_time = time.time()
        led_state = False
        blink_interval = 1.0 / blink_frequency_hz

        while time.time() - start_time < duration and self.running:
            led_func(led_state)
            led_state = not led_state
            time.sleep(blink_interval)

    # TODO for rev3 PCB, LEDs are turned on by low signal on pin because LEDs in button are connected with positive side connected to 5V and negative to the pin, so we need to invert the driving logic signal
    def green_led_heartbeat(self):
        """Green LED heartbeat - normal pulse when idle, 2X faster when Argo running, 3-flash pattern when recording"""
        while self.running:
            if self.recording_active:
                # Recording mode: 3 quick flashes followed by pause
                # Total period matches normal heartbeat (1 second)

                # 3 quick flashes (150ms on, 50ms off each = 200ms per flash)
                for flash in range(3):
                    if not self.running:
                        break

                    # Flash on (150ms)
                    self.set_green_led(True)
                    self._heartbeat_sleep(0.15)  # 150ms on

                    if not self.running:
                        break

                    # Flash off (50ms between flashes, except after last flash)
                    self.set_green_led(False)
                    if flash < 2:  # Don't add gap after last flash
                        self._heartbeat_sleep(0.05)  # 50ms off between flashes

                # Longer pause to complete the 1-second period
                # 3 flashes took: 3 * (150ms + 50ms) - 50ms = 550ms
                # Remaining time: 1000ms - 550ms = 450ms
                if self.running:
                    self._heartbeat_sleep(0.45)  # 450ms pause

            else:

                # Determine heartbeat frequency based on Argo service status
                if self.argo_service_running:
                    # Argo running: 2X faster heartbeat (2Hz)
                    heartbeat_frequency = LED_HEARTBEAT_HZ * 2.0
                else:
                    # Argo stopped: normal heartbeat (1Hz)
                    heartbeat_frequency = LED_HEARTBEAT_HZ

                heartbeat_interval = 1.0 / heartbeat_frequency
                half_period = heartbeat_interval / 2.0

                # LED on for first half
                self.set_green_led(True)
                self._heartbeat_sleep(half_period)

                if not self.running:
                    break

                # LED off for second half
                self.set_green_led(False)
                self._heartbeat_sleep(half_period)

    def _heartbeat_sleep(self, duration):
        """Sleep for heartbeat timing with responsive shutdown checking"""
        sleep_time = 0
        increment = 0.05  # Check every 50ms for responsive shutdown
        while sleep_time < duration and self.running:
            time.sleep(increment)
            sleep_time += increment

    def sos_led_pattern(self):
        """SOS LED pattern for low battery warning - Blue LED blinks SOS in Morse code"""
        logger.info("Starting SOS LED pattern for low battery warning")
        self.sos_led_active = True

        # SOS in Morse code: ... --- ... (short-short-short, long-long-long, short-short-short)
        # Timing: short = 0.2s, long = 0.6s, pause between letters = 0.6s, pause between SOS = 1.8s
        sos_pattern = [
            # S: ... (short-short-short)
            (0.2, True),   # Short on
            (0.2, False),  # Short off
            (0.2, True),   # Short on
            (0.2, False),  # Short off
            (0.2, True),   # Short on
            (0.6, False),  # Long pause between letters
            
            # O: --- (long-long-long)
            (0.6, True),   # Long on
            (0.2, False),  # Short off
            (0.6, True),   # Long on
            (0.2, False),  # Short off
            (0.6, True),   # Long on
            (0.6, False),  # Long pause between letters
            
            # S: ... (short-short-short)
            (0.2, True),   # Short on
            (0.2, False),  # Short off
            (0.2, True),   # Short on
            (0.2, False),  # Short off
            (0.2, True),   # Short on
            (1.8, False),  # Long pause between SOS cycles
        ]

        while self.running and self.sos_led_active and self.low_battery_detected and not self.critical_battery_detected:
            try:
                for duration, led_state in sos_pattern:
                    if not self.running or not self.sos_led_active or not self.low_battery_detected or self.critical_battery_detected:
                        break
                    
                    # Set blue LED state (red LED is not GPIO controlled, so we use blue LED)
                    self.set_blue_led(led_state)
                    
                    # Sleep in small increments for responsive shutdown
                    sleep_time = 0
                    while sleep_time < duration and self.running and self.sos_led_active and self.low_battery_detected and not self.critical_battery_detected:
                        time.sleep(0.01)  # Sleep in 10ms increments
                        sleep_time += 0.01

            except Exception as e:
                logger.error(f"Error in SOS LED pattern: {e}")
                break

        # Turn off blue LED when done
        self.set_blue_led(False)
        self.sos_led_active = False
        logger.info("SOS LED pattern completed")

    def shutdown_led_pattern(self):
        """1Hz LED pattern with configurable duty cycle during shutdown sequence"""
        logger.info(
            f"Starting shutdown LED pattern (1Hz, {LED_SHUTDOWN_DUTY_CYCLE*100:.0f}% duty cycle)")

        # Calculate timing for 1Hz with configurable duty cycle
        period = 1.0 / LED_SHUTDOWN_FREQUENCY_HZ  # 1 second
        on_time = period * LED_SHUTDOWN_DUTY_CYCLE  # Configurable on time
        # Remaining off time
        off_time = period * (1.0 - LED_SHUTDOWN_DUTY_CYCLE)

        while self.running and self.shutdown_initiated:
            try:
                # Turn on both LEDs
                logger.info("Shutdown LED: ON")
                self.set_green_led(True)
                self.set_blue_led(True)

                # Sleep in small increments to allow for responsive shutdown
                sleep_time = 0
                while sleep_time < on_time and self.running and self.shutdown_initiated:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01

                if not self.running or not self.shutdown_initiated:
                    break

                # Turn off both LEDs
                logger.info("Shutdown LED: OFF")
                self.set_green_led(False)
                self.set_blue_led(False)

                # Sleep in small increments to allow for responsive shutdown
                sleep_time = 0
                while sleep_time < off_time and self.running and self.shutdown_initiated:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01

            except Exception as e:
                logger.error(f"Error in shutdown LED pattern: {e}")
                break

        # Turn off LEDs when done
        self.set_green_led(False)
        self.set_blue_led(False)
        logger.info("Shutdown LED pattern completed")

    def gradual_frequency_pattern(self):
        """Gradual frequency increase LED pattern during button press - 2Hz to 20Hz over threshold time"""
        start_time = time.time()
        logger.info(
            f"Starting gradual frequency pattern for {self.SHUTDOWN_THRESHOLD}s threshold")

        while self.running and self.power_button_pressed and not self.shutdown_initiated:
            # Calculate elapsed time and progress (0.0 to 1.0)
            elapsed_time = time.time() - start_time
            progress = min(elapsed_time / self.SHUTDOWN_THRESHOLD, 1.0)

            # Calculate current frequency (2Hz to 20Hz)
            current_frequency = LED_PRESS_START_FREQUENCY_HZ + \
                (LED_PRESS_END_FREQUENCY_HZ - LED_PRESS_START_FREQUENCY_HZ) * progress

            # Calculate timing for 50% duty cycle
            period = 1.0 / current_frequency
            on_time = period * LED_PRESS_DUTY_CYCLE
            off_time = period * (1.0 - LED_PRESS_DUTY_CYCLE)

            # Turn on both LEDs
            self.set_green_led(True)
            self.set_blue_led(True)

            # Sleep in small increments to allow for responsive shutdown
            sleep_time = 0
            while sleep_time < on_time and self.running and self.power_button_pressed and not self.shutdown_initiated:
                time.sleep(0.01)  # Sleep in 10ms increments
                sleep_time += 0.01

            if not self.running or not self.power_button_pressed or self.shutdown_initiated:
                break

            # Turn off both LEDs
            self.set_green_led(False)
            self.set_blue_led(False)

            # Sleep in small increments to allow for responsive shutdown
            sleep_time = 0
            while sleep_time < off_time and self.running and self.power_button_pressed and not self.shutdown_initiated:
                time.sleep(0.01)  # Sleep in 10ms increments
                sleep_time += 0.01

        # Turn off LEDs when pattern stops
        self.set_green_led(False)
        self.set_blue_led(False)

    def monitor_power_button_interrupts(self):
        """Hardware interrupt-based button monitoring - efficient edge detection with minimal CPU usage"""
        if not self.gpio_available:
            logger.info(
                "GPIO not available - skipping button monitoring in test mode")
            # In test mode without GPIO, just sleep to keep the thread alive
            while self.running:
                time.sleep(1.0)
            return

        logger.info("Using hardware interrupt-based button monitoring")
        logger.info("Monitoring for falling edge events (POW_BUT going low)")

        # IMPORTANT: Do NOT override button_detection_active here!
        # The __init__ method already set this correctly based on initial button state
        # If button was pressed at startup, detection remains inactive until first release
        logger.info(
            f"Button detection active status: {self.button_detection_active}")

        while self.running:
            try:
                # Wait for GPIO interrupt events using select() - this is the key to low CPU usage!
                # The process sleeps until a hardware interrupt occurs
                ready, _, _ = select.select(
                    [self.power_button_line.event_get_fd()], [], [], 1.0)

                if ready:
                    # Hardware interrupt occurred - read the event
                    try:
                        event = self.power_button_line.event_read()

                        # We're configured for falling edge, so this is a button press
                        if event.type == gpiod.LineEvent.FALLING_EDGE:
                            if self.button_detection_active:
                                self.button_press_start_time = time.time()
                                self.power_button_pressed = True
                                # Reset warning flag for new button press
                                self.warning_notification_sent = False
                                logger.info(
                                    "Power button pressed (hardware interrupt)")

                                # Start gradual frequency LED pattern
                                threading.Thread(
                                    target=self.gradual_frequency_pattern,
                                    daemon=True
                                ).start()

                                # Start monitoring for button release or long press timeout
                                threading.Thread(
                                    target=self.monitor_button_release_timeout,
                                    daemon=True
                                ).start()
                            else:
                                logger.info(
                                    "Button press detected but detection not yet active - ignoring")
                        else:
                            logger.warning(
                                f"Unexpected event type: {event.type}")

                    except Exception as e:
                        logger.error(f"Error reading GPIO event: {e}")

                else:
                    # Timeout - no events, check if we need to activate button detection
                    # If button was pressed at startup and detection is inactive, check for release
                    if not self.button_detection_active:
                        try:
                            current_state = self.get_current_button_state()
                            if current_state == 1:  # Button released
                                self.button_detection_active = True
                                logger.info(
                                    "Button released after startup - button detection now active")
                        except Exception as e:
                            logger.error(
                                f"Error checking button state for activation: {e}")

                    # This timeout happens every 1 second and uses minimal CPU

            except Exception as e:
                logger.error(
                    f"Error in interrupt-based button monitoring: {e}")
                time.sleep(BUTTON_ERROR_RECOVERY_DELAY_S)

    def monitor_button_release_timeout(self):
        """Monitor for button release by checking current state and handle long press timeout"""
        # Since we only get falling edge interrupts, we need to poll for release
        # But this thread only runs during button press, so overall CPU usage is still very low
        poll_interval = 0.1  # 10Hz polling during button press only

        while self.running and self.power_button_pressed:
            try:
                # Read current button state to detect release
                # Note: This requires switching the line back to input mode temporarily
                current_state = self.get_current_button_state()

                # Check for button release (rising edge - button goes high)
                if current_state == 1:  # Button released
                    press_duration = time.time() - self.button_press_start_time
                    logger.info(
                        f"Power button released after {press_duration:.2f} seconds (interrupt + polling)")

                    if press_duration >= self.SHUTDOWN_THRESHOLD:
                        logger.info(
                            "Long press detected, initiating shutdown...")
                        self.initiate_shutdown()
                    else:
                        # Check if button was released after warning but before shutdown (shutdown cancelled)
                        if (press_duration >= SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S and
                            self.warning_notification_sent and
                                not self.shutdown_initiated):
                            logger.info(
                                "Shutdown cancelled - button released after warning threshold")
                            self.send_desktop_notification(
                                "Shutdown Cancelled",
                                f"Power button released - shutdown cancelled!\nButton was held for {press_duration:.1f}s",
                                "normal"
                            )
                        else:
                            # Short press - add to tap detection system
                            self.add_tap(press_duration)

                    self.power_button_pressed = False
                    self.button_press_start_time = None
                    # Reset warning flag when button is released
                    self.warning_notification_sent = False
                    break  # Exit this monitoring thread

                # Check for long press timeout while button is still held
                else:
                    press_duration = time.time() - self.button_press_start_time

                    # Check for warning threshold - send notification to warn user to release button
                    if (press_duration >= SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S and
                        not self.warning_notification_sent and
                            not self.shutdown_initiated):
                        logger.info(
                            f"Button press reached warning threshold ({SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S}s) - sending warning notification")
                        self.send_desktop_notification(
                            "Power Button Warning",
                            f"Release the power button now to abort shutdown!\nButton held for {press_duration:.1f}s (shutdown in {self.SHUTDOWN_THRESHOLD - press_duration:.1f}s)",
                            "critical",
                            FINAL_SHUTDOWN_NOTIFICATION_MS  # No timeout - stays until dismissed
                        )
                        self.warning_notification_sent = True

                    # Check for shutdown threshold
                    if press_duration >= self.SHUTDOWN_THRESHOLD:
                        if not self.shutdown_initiated:
                            logger.info(
                                "Long press threshold reached, initiating shutdown...")
                            self.initiate_shutdown()
                        break  # Exit monitoring loop

                time.sleep(poll_interval)

            except Exception as e:
                logger.error(f"Error in button release monitoring: {e}")
                time.sleep(BUTTON_ERROR_RECOVERY_DELAY_S)
                break

    def get_current_button_state(self):
        """Get current button state - works with interrupt-configured line"""
        if not self.gpio_available:
            logger.debug(
                "GPIO not available - assuming button released in test mode")
            return 1  # Assume released in test mode
        try:
            # The line is configured for events, but we can still read the current value
            # This is needed to detect button release since we only get falling edge interrupts
            return self.power_button_line.get_value()
        except Exception as e:
            logger.error(f"Error reading current button state: {e}")
            return 1  # Assume released on error

    def add_tap(self, press_duration):
        """Add a tap to the detection system and start timeout timer"""
        current_time = time.time()

        # Only consider short presses as taps (< 0.5 seconds)
        if press_duration > 0.5:
            # Long press - clear tap history and cancel any pending timeout
            self.tap_times.clear()
            if self.tap_timeout_timer:
                self.tap_timeout_timer.cancel()
                self.tap_timeout_timer = None
            logger.info("Long press detected - clearing tap history")
            return

        # Add this tap to the history
        self.tap_times.append(current_time)
        logger.info(
            f"Tap {len(self.tap_times)} detected (duration: {press_duration:.2f}s)")

        # Cancel any existing timeout timer
        if self.tap_timeout_timer:
            self.tap_timeout_timer.cancel()

        # Start a new timeout timer
        self.tap_timeout_timer = threading.Timer(
            self.tap_timeout_duration,
            self.process_tap_sequence
        )
        self.tap_timeout_timer.start()

    def process_tap_sequence(self):
        """Process the tap sequence after timeout period"""
        if not self.tap_times:
            return

        current_time = time.time()

        # Clean up old taps outside the time window
        cutoff_time = current_time - MULTI_TAP_MAX_DURATION_S
        old_count = len(self.tap_times)
        self.tap_times = [t for t in self.tap_times if t > cutoff_time]

        if old_count != len(self.tap_times):
            logger.info(
                f"Cleaned up {old_count - len(self.tap_times)} old taps")

        # Process tap sequence based on count
        tap_count = len(self.tap_times)

        if tap_count < 2:
            # Not enough taps - ignore
            logger.info(f"Not enough taps ({tap_count}) - ignoring")
            self.tap_times.clear()
            return

        if tap_count > 5:
            # Too many taps - ignore
            logger.info(f"Too many taps ({tap_count}) - ignoring")
            self.tap_times.clear()
            return

        # Check if all taps are within the time window
        if tap_count >= 2:
            oldest_tap = min(self.tap_times)
            newest_tap = max(self.tap_times)
            duration = newest_tap - oldest_tap

            if duration <= MULTI_TAP_MAX_DURATION_S + 0.01:
                # Valid tap sequence detected
                if tap_count == 2:
                    logger.info(
                        f"Double tap detected! ({duration:.2f}s duration)")
                    self.tap_times.clear()
                    self.toggle_argo_service()
                elif tap_count == 3:
                    logger.info(
                        f"Triple tap detected! ({duration:.2f}s duration)")
                    self.tap_times.clear()
                    self.toggle_recording()
                elif tap_count == 4:
                    logger.info(
                        f"Quadruple tap detected! ({duration:.2f}s duration)")
                    self.tap_times.clear()
                    # Future: could be used for another command
                elif tap_count == 5:
                    logger.info(
                        f"Quintuple tap detected! ({duration:.2f}s duration)")
                    self.tap_times.clear()
                    # Future: could be used for another command
            else:
                logger.info(
                    f"Tap sequence too slow: {duration:.3f}s > {MULTI_TAP_MAX_DURATION_S}s")
                self.tap_times.clear()

        # Clear the timeout timer reference
        self.tap_timeout_timer = None

    def toggle_recording(self):
        """Toggle rosbag recording on/off using ROS2 service pattern from tutorial"""
        if self.test_mode:
            # Test mode - actually test the service call but with better error handling
            new_state = not self.recording_active
            logger.info(
                f"TEST MODE: Testing recording service call from {self.recording_active} to {new_state}")
            print(
                f"🎬 TEST: Testing recording {'START' if new_state else 'STOP'} service call")

            # Actually test the service call in test mode
            if new_state:
                success = self.start_recording()
                if success:
                    logger.info(
                        "TEST MODE: Recording start service call succeeded")
                    self.send_desktop_notification(
                        "Recording Test Success",
                        "Recording start service call succeeded in test mode",
                        "normal"
                    )
                else:
                    logger.error(
                        "TEST MODE: Recording start service call failed")
                    self.send_desktop_notification(
                        "Recording Test Failed",
                        "Recording start service call failed in test mode",
                        "critical"
                    )
            else:
                success = self.stop_recording()
                if success:
                    logger.info(
                        "TEST MODE: Recording stop service call succeeded")
                    self.send_desktop_notification(
                        "Recording Test Success",
                        "Recording stop service call succeeded in test mode",
                        "normal"
                    )
                else:
                    logger.error(
                        "TEST MODE: Recording stop service call failed")
                    self.send_desktop_notification(
                        "Recording Test Failed",
                        "Recording stop service call failed in test mode",
                        "critical"
                    )
            return

        # Real mode - ensure Argo service is running first
        # Always check the actual service status before proceeding
        self.check_argo_service_status()
        # Also check recording service availability after Argo service check
        self.check_recording_service_availability()
        if not self.argo_service_running:
            logger.info(
                "Triple tap detected - Argo not running, starting it first")
            self.send_desktop_notification(
                "Starting Argo Service",
                "Starting Argo service before recording...",
                "normal"
            )
            success = self.start_argo_service()
            if not success:
                self.send_desktop_notification(
                    "Recording Error",
                    "Failed to start Argo service - cannot record",
                    "critical"
                )
                return

            # Wait for recording service to become available
            logger.info("Waiting for recording service to become available...")
            import time
            max_wait = 15  # Wait up to 15 seconds
            start_time = time.time()
            while time.time() - start_time < max_wait:
                self.check_recording_service_availability()
                if self.recording_service_available:
                    logger.info("Recording service is now available")
                    break
                time.sleep(0.5)

            if not self.recording_service_available:
                self.send_desktop_notification(
                    "Recording Error",
                    "Recording service did not become available in time",
                    "critical"
                )
                return

        # Check if recording service is available
        if not self.recording_service_available:
            logger.warning(
                "Triple tap detected but recording service is not available")
            self.send_desktop_notification(
                "Recording Unavailable",
                "Recording service is not ready - please wait",
                "critical"
            )
            return

        # Toggle recording using the service pattern from ROS2 tutorial
        if self.recording_active:
            logger.info("Triple tap detected - stopping recording")
            success = self.stop_recording()
            if success:
                # Success notification will come from recording_info_callback when recording actually stops
                self.send_desktop_notification(
                    "Stopping Recording",
                    "Recording stop requested...",
                    "normal"
                )
            else:
                self.send_desktop_notification(
                    "Recording Error",
                    "Failed to send stop recording request",
                    "critical"
                )
        else:
            logger.info("Triple tap detected - starting recording")
            success = self.start_recording()
            if success:
                # Show immediate feedback that request was sent
                self.send_desktop_notification(
                    "Starting Recording",
                    "Recording start requested...",
                    "normal"
                )
            else:
                self.send_desktop_notification(
                    "Recording Error",
                    "Failed to send start recording request",
                    "critical"
                )

    def send_desktop_notification(self, title, message, urgency="normal", expire_time_ms=None):
        """Send desktop notification using notify-send with cached user/display info"""
        try:
            # Check if desktop user detection failed previously
            if self.desktop_user_detection_failed:
                logger.debug(
                    "Desktop user detection previously failed - skipping notification")
                return

            # Modify title and message for test mode
            if self.test_mode:
                test_title = f"[TEST] {title}"
                test_message = f"{message}\n\n(This is a test notification - no actual action taken)"
                logger.info(
                    f"TEST MODE: Sending desktop notification - {title}: {message}")
            else:
                test_title = title
                test_message = message

            # Use custom expire time or default
            expire_time = expire_time_ms if expire_time_ms is not None else NOTIFICATION_EXPIRE_TIME_MS

            # Check if we're running as root (systemd service)
            if os.geteuid() == 0:
                # Running as root - use cached desktop user and display
                if not self.cached_desktop_user or not self.cached_display_env:
                    logger.debug(
                        "No cached desktop user/display available - skipping notification")
                    return

                logger.debug(
                    f"Using cached desktop_user: {self.cached_desktop_user}, display: {self.cached_display_env}")

                cmd = [
                    'sudo', '-u', self.cached_desktop_user,
                    'DISPLAY=' + self.cached_display_env,
                    'notify-send',
                    '--urgency', urgency,
                    '--expire-time', str(expire_time),
                    test_title,
                    test_message
                ]

            else:
                # Running as regular user - use normal method
                # Set up environment for desktop notifications
                env = os.environ.copy()

                # Try to find the display for logged-in users
                try:
                    # Get list of logged-in users and their displays
                    result = subprocess.run(
                        ['who'], capture_output=True, text=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                    if result.returncode == 0:
                        for line in result.stdout.strip().split('\n'):
                            if line and ':' in line:
                                parts = line.split()
                                for part in parts:
                                    if part.startswith(':'):
                                        # Remove the leading ':'
                                        display = part[1:]
                                        if display.isdigit() or '.' in display:
                                            env['DISPLAY'] = f':{display}'
                                            break
                                if 'DISPLAY' in env:
                                    break
                except Exception as e:
                    logger.info(
                        f"Could not determine display from 'who' command: {e}")

                # If no display found, try common defaults
                if 'DISPLAY' not in env:
                    env['DISPLAY'] = ':0'

                cmd = [
                    'notify-send',
                    '--urgency', urgency,
                    '--expire-time', str(expire_time),
                    test_title,
                    test_message
                ]

            # Execute the notification command
            if os.geteuid() == 0:
                # For root execution, don't use env parameter with sudo
                subprocess.run(cmd, check=True,
                               timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            else:
                # For user execution, use env parameter
                subprocess.run(cmd, check=True,
                               timeout=DESKTOP_NOTIFICATION_TIMEOUT_S, env=env)

            if self.test_mode:
                logger.info(f"TEST MODE: Desktop notification sent: {title}")
            else:
                logger.info(f"Desktop notification sent: {title}")

        except subprocess.TimeoutExpired:
            logger.warning("Desktop notification timed out")
        except subprocess.CalledProcessError as e:
            logger.warning(f"Failed to send desktop notification: {e}")
            # Mark desktop user detection as failed to avoid repeated attempts
            self.desktop_user_detection_failed = True
        except Exception as e:
            logger.warning(
                f"Unexpected error sending desktop notification: {e}")
            # Mark desktop user detection as failed to avoid repeated attempts
            self.desktop_user_detection_failed = True

    def broadcast_shutdown_message(self):
        """Broadcast shutdown message to all logged-in users"""
        try:
            # Create shutdown message with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message = f"SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nShutting down now..."

            # Use wall command to broadcast message to all users
            try:
                if self.test_mode:
                    # Add test prefix to wall message
                    test_message = f"TEST: {message}"
                    subprocess.run(['wall', test_message],
                                   check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
                    logger.info(f"TEST MODE: Wall message sent: {message}")
                else:
                    # Use wall command to send message to all logged-in users
                    subprocess.run(['wall', message], check=True,
                                   timeout=WALL_MESSAGE_TIMEOUT_S)
                    logger.info("Shutdown message broadcasted to all users")
            except subprocess.TimeoutExpired:
                logger.warning(
                    "Wall command timed out - message may not have been sent")
            except subprocess.CalledProcessError as e:
                logger.warning(f"Failed to broadcast wall message: {e}")
            except Exception as e:
                logger.warning(
                    f"Unexpected error broadcasting wall message: {e}")

        except Exception as e:
            logger.error(f"Error preparing shutdown message: {e}")

    def initiate_shutdown(self):
        """Initiate system shutdown sequence"""
        if self.shutdown_initiated:
            return

        self.shutdown_initiated = True

        logger.info("Initiating shutdown sequence...")

        # Start 1Hz LED pattern immediately to show shutdown initiated
        threading.Thread(
            target=self.shutdown_led_pattern,
            daemon=True
        ).start()

        # Broadcast wall message to all users
        self.broadcast_shutdown_message()

        # Send final desktop notification
        self.send_desktop_notification(
            "System Shutdown",
            "Shutdown initiated by power button - system shutting down now",
            "critical"
        )

        # Execute shutdown command
        if self.test_mode:
            logger.info(
                "TEST MODE: Shutdown command disabled - would normally execute: shutdown -h now")
            # In test mode, let the shutdown LED pattern run for demonstration
            logger.info(
                f"TEST MODE: Running shutdown LED pattern for {TEST_MODE_SHUTDOWN_DELAY_S} seconds...")
            # Let the pattern run for demonstration
            time.sleep(TEST_MODE_SHUTDOWN_DELAY_S)
            self.running = False  # Stop running after LED pattern demonstration
        else:
            logger.info("Executing shutdown command: shutdown -h now")
            subprocess.run(['shutdown', '-h', 'now'], check=True)
            logger.info("Shutdown command executed successfully")
            # Don't set self.running = False here - let the system shutdown
            # GPIO pins will automatically revert to input state on halt, de-energizing relay

    def run(self):
        """Main control loop"""
        logger.info("Power controller starting...")
        logger.info("Button monitoring active")

        # NOTE: Power relay control removed from this service
        # Relay is energized by argo_poweron.startup during boot

        # Start power button monitoring in separate thread
        logger.info("Starting hardware interrupt-based button detection")
        # TODO for rev3 PCB, button presses will generate a high going edge, so we need to invert the logic
        button_thread = threading.Thread(
            target=self.monitor_power_button_interrupts, daemon=True)
        button_thread.start()

        # Start green LED heartbeat in separate thread
        logger.info("Starting green LED heartbeat thread")
        heartbeat_thread = threading.Thread(
            target=self.green_led_heartbeat, daemon=True)
        heartbeat_thread.start()

        # Start critical battery monitoring in separate thread
        logger.info("Starting critical battery monitoring thread")
        battery_thread = threading.Thread(
            target=self.monitor_critical_battery, daemon=True)
        battery_thread.start()

        try:
            # Main loop - just keep running while monitoring threads handle GPIO
            last_sync_time = 0
            sync_interval = 300  # Sync every 30 seconds

            while self.running:
                current_time = time.time()

                # Periodic state synchronization
                if current_time - last_sync_time >= sync_interval:
                    if self.argo_service_running and self.recording_service_available:
                        logger.debug(
                            "Performing periodic recording state synchronization...")
                        self.query_current_recording_status()
                    last_sync_time = current_time

                # Sleep in smaller increments to be more responsive to shutdown signals
                for _ in range(10):  # 10 * 0.1s = 1s total, but check running flag every 0.1s
                    if not self.running:
                        break
                    time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
            self.running = False

        finally:
            self.cleanup()

    def monitor_critical_battery(self):
        """Monitor battery voltage every 30 seconds for critical low voltage"""
        logger.info("Starting critical battery monitoring thread")
        self.battery_monitoring_active = True

        while self.running and self.battery_monitoring_active:
            try:
                # Always attempt to check battery, even if argo-launch is stopped
                # This is CRITICAL for safety - battery monitoring must never stop

                # Call battery service to get voltage
                battery_data = self._call_battery_service()
                if battery_data:
                    battery_voltage = battery_data.get('battery_voltage', 0)
                    
                    # CRITICAL SAFETY CHECK: Validate battery voltage is reasonable
                    # Never halt on obviously invalid readings (0V, negative, or extremely low)
                    if battery_voltage <= 0 or battery_voltage < 3.0:
                        logger.error(f"Invalid battery voltage reading: {battery_voltage:.3f}V - ignoring (likely I2C failure)")
                        # Don't process this reading - it's clearly invalid
                        continue
                    
                    logger.info(
                        f"Battery voltage check: {battery_voltage:.3f}V (low: {LOW_BATTERY_THRESHOLD_V}V, critical: {CRITICAL_BATTERY_THRESHOLD_V}V)")

                    # Check for critical battery first (highest priority)
                    if battery_voltage < CRITICAL_BATTERY_THRESHOLD_V:
                        if not self.critical_battery_detected:
                            logger.critical(
                                f"CRITICAL BATTERY DETECTED: {battery_voltage:.3f}V < {CRITICAL_BATTERY_THRESHOLD_V}V")
                            self.critical_battery_detected = True
                            # Stop SOS pattern if running (critical takes priority)
                            if self.sos_led_active:
                                self.sos_led_active = False
                            self.initiate_critical_battery_halt(battery_voltage)
                    
                    # Check for low battery (SOS warning)
                    elif battery_voltage < LOW_BATTERY_THRESHOLD_V:
                        if not self.low_battery_detected:
                            logger.warning(
                                f"LOW BATTERY DETECTED: {battery_voltage:.3f}V < {LOW_BATTERY_THRESHOLD_V}V - Starting SOS LED pattern")
                            self.low_battery_detected = True
                            # Start SOS LED pattern in separate thread
                            threading.Thread(target=self.sos_led_pattern, daemon=True).start()
                            # Send desktop notification
                            self.send_desktop_notification(
                                "Low Battery Warning",
                                f"Battery voltage low: {battery_voltage:.3f}V\nSOS LED pattern activated\nReturn to shore or charge battery",
                                "critical"
                            )
                    
                    # Battery voltage recovered above low threshold
                    else:
                        # Check if we were in low battery state
                        if self.low_battery_detected:
                            logger.info(
                                f"Battery voltage recovered from low: {battery_voltage:.3f}V >= {LOW_BATTERY_THRESHOLD_V}V")
                            self.low_battery_detected = False
                            self.sos_led_active = False  # Stop SOS pattern
                            self.send_desktop_notification(
                                "Battery Recovered",
                                f"Battery voltage recovered: {battery_voltage:.3f}V\nSOS LED pattern stopped",
                                "normal"
                            )
                        
                        # Check if we were in critical battery state
                        if self.critical_battery_detected:
                            logger.info(
                                f"Battery voltage recovered from critical: {battery_voltage:.3f}V >= {CRITICAL_BATTERY_THRESHOLD_V}V")
                            self.critical_battery_detected = False
                            self._clear_critical_battery_flag()
                else:
                    # Service unavailable - log at info level to see what's happening
                    # This is expected when argo-launch.service is stopped
                    logger.info(
                        "Could not retrieve battery data - battery_water service may not be running")

                self.last_battery_check_time = time.time()

            except Exception as e:
                logger.error(f"Error in critical battery monitoring: {e}")

            # Sleep for monitoring interval
            sleep_time = 0
            while sleep_time < BATTERY_MONITORING_INTERVAL_S and self.running and self.battery_monitoring_active:
                time.sleep(1.0)
                sleep_time += 1.0

        logger.info("Critical battery monitoring thread stopped")

    def _call_battery_service(self):
        """Call battery_water service to get current battery data"""
        try:
            # Use subprocess to call the battery service with proper ROS2 environment
            cmd = [
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && ros2 service call /battery_status std_srvs/srv/Trigger'
            ]

            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                # Parse the JSON response from the service
                try:
                    import json
                    # Extract message content from ROS2 service response
                    lines = result.stdout.strip().split('\n')
                    message_content = None

                    logger.info(
                        f"Battery service response has {len(lines)} lines")

                    for line in lines:
                        line = line.strip()
                        # Look for Trigger_Response format: message='...'
                        if 'message=' in line:
                            # Extract content between message=' and the closing '
                            import re
                            match = re.search(
                                r"message='(.*)'", line, re.DOTALL)
                            if match:
                                message_part = match.group(1)
                                # Unescape newlines and quotes
                                message_part = message_part.replace(
                                    '\\n', '\n').replace('\\"', '"')
                                message_content = message_part
                                logger.info(
                                    "Found message content in Trigger_Response")
                                break
                        # Also support old format: message:
                        elif line.startswith('message:'):
                            # Extract the JSON content from the message field
                            message_part = line.split(':', 1)[1].strip()
                            # Remove quotes if present
                            if message_part.startswith('"') and message_part.endswith('"'):
                                message_part = message_part[1:-1]
                            # Unescape newlines and quotes
                            message_part = message_part.replace(
                                '\\n', '\n').replace('\\"', '"')
                            message_content = message_part
                            logger.info("Found message content in old format")
                            break

                    if message_content:
                        # Parse the JSON content
                        response_data = json.loads(message_content)
                        # Extract raw_data from the response
                        raw_data = response_data.get('raw_data', {})
                        logger.info(
                            f"Successfully parsed battery data: {raw_data.get('battery_voltage', 'N/A')}V")
                        return raw_data
                    else:
                        logger.error("No message content found in response")
                        logger.error(f"Response stdout: {result.stdout[:500]}")
                        return None

                except (json.JSONDecodeError, KeyError) as e:
                    logger.error(
                        f"Error parsing battery service response: {e}")
                    logger.error(
                        f"Message content was: {message_content[:200] if message_content else 'None'}")
                    return None
            else:
                logger.error(
                    f"Battery service call failed with return code {result.returncode}")
                logger.error(f"Stdout: {result.stdout[:200]}")
                logger.error(f"Stderr: {result.stderr[:200]}")
                return None

        except subprocess.TimeoutExpired:
            logger.error("Battery service call timed out after 10 seconds")
            return None
        except Exception as e:
            logger.error(f"Error calling battery service: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            return None

    def show_critical_battery_confirmation_dialog(self, battery_voltage):
        """Show desktop confirmation dialog for critical battery shutdown"""
        try:
            # Try to show a zenity dialog first (most reliable on desktop)
            dialog_cmd = [
                'zenity',
                '--question',
                '--title=CRITICAL BATTERY SHUTDOWN',
                f'--text=Battery voltage critically low: {battery_voltage:.3f}V\n\nSystem will halt in 30 seconds to preserve power for manual sailing.\n\nClick "Cancel" to abort shutdown.\nClick "OK" to proceed with immediate shutdown.',
                '--ok-label=Shutdown Now',
                '--cancel-label=Cancel Shutdown',
                '--timeout=30',
                '--width=500',
                '--height=300'
            ]
            
            logger.info("Showing critical battery confirmation dialog...")
            result = subprocess.run(dialog_cmd, capture_output=True, text=True, timeout=35)
            
            if result.returncode == 0:
                logger.info("User confirmed critical battery shutdown")
                return True
            elif result.returncode == 1:
                logger.info("User cancelled critical battery shutdown")
                return False
            else:
                logger.warning(f"Dialog returned unexpected code: {result.returncode}")
                return True  # Default to shutdown if dialog fails
                
        except subprocess.TimeoutExpired:
            logger.info("Critical battery dialog timed out - proceeding with shutdown")
            return True
        except FileNotFoundError:
            logger.warning("zenity not found - trying kdialog...")
            try:
                # Fallback to kdialog
                dialog_cmd = [
                    'kdialog',
                    '--title=CRITICAL BATTERY SHUTDOWN',
                    '--yesno=Battery voltage critically low: {battery_voltage:.3f}V\n\nSystem will halt in 30 seconds to preserve power for manual sailing.\n\nClick "No" to abort shutdown.\nClick "Yes" to proceed with immediate shutdown.',
                    '--yes-label=Shutdown Now',
                    '--no-label=Cancel Shutdown'
                ]
                
                result = subprocess.run(dialog_cmd, capture_output=True, text=True, timeout=35)
                
                if result.returncode == 0:
                    logger.info("User confirmed critical battery shutdown (kdialog)")
                    return True
                else:
                    logger.info("User cancelled critical battery shutdown (kdialog)")
                    return False
                    
            except (FileNotFoundError, subprocess.TimeoutExpired):
                logger.warning("kdialog also not available or timed out - trying simple yad...")
                try:
                    # Fallback to yad (Yet Another Dialog)
                    dialog_cmd = [
                        'yad',
                        '--title=CRITICAL BATTERY SHUTDOWN',
                        '--text=Battery voltage critically low: {battery_voltage:.3f}V\n\nSystem will halt in 30 seconds to preserve power for manual sailing.',
                        '--button=Shutdown Now:0',
                        '--button=Cancel Shutdown:1',
                        '--timeout=30',
                        '--width=500',
                        '--height=300'
                    ]
                    
                    result = subprocess.run(dialog_cmd, capture_output=True, text=True, timeout=35)
                    return result.returncode == 0
                    
                except (FileNotFoundError, subprocess.TimeoutExpired):
                    logger.warning("No dialog tools available - proceeding with shutdown after pause")
                    return True
        except Exception as e:
            logger.error(f"Error showing critical battery dialog: {e}")
            logger.info("Proceeding with shutdown after pause due to dialog error")
            return True

    def initiate_critical_battery_halt(self, battery_voltage):
        """Initiate critical battery halt sequence with confirmation dialog"""
        logger.critical(
            f"CRITICAL BATTERY HALT: {battery_voltage:.3f}V - System will halt to preserve power")

        # Set critical battery flag for shutdown hook
        self._set_critical_battery_flag()

        # Send critical notification
        self.send_desktop_notification(
            "CRITICAL BATTERY",
            f"Battery voltage critically low: {battery_voltage:.3f}V\nSystem will halt in 30 seconds to preserve power for manual sailing",
            "critical",
            30000  # 30 second timeout for notification
        )

        # Broadcast wall message
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message = f"CRITICAL BATTERY: {battery_voltage:.3f}V at {timestamp}\nSystem will halt in 30 seconds to preserve power for manual sailing via radio control"
            subprocess.run(['wall', message], check=True,
                           timeout=WALL_MESSAGE_TIMEOUT_S)
            logger.info("Critical battery wall message broadcasted")
        except Exception as e:
            logger.error(f"Failed to broadcast critical battery message: {e}")

        # Show confirmation dialog and wait for user response
        logger.info("Showing critical battery confirmation dialog...")
        should_shutdown = self.show_critical_battery_confirmation_dialog(battery_voltage)
        
        if not should_shutdown:
            logger.info("Critical battery shutdown cancelled by user")
            # Clear the critical battery flag since we're not shutting down
            self._clear_critical_battery_flag()
            # Send cancellation notification
            self.send_desktop_notification(
                "SHUTDOWN CANCELLED",
                "Critical battery shutdown was cancelled by user",
                "normal",
                5000
            )
            return  # Exit without shutting down

        # Stop battery monitoring to prevent repeated alerts
        self.battery_monitoring_active = False

        # Execute halt command (not shutdown - preserves power relay)
        if self.test_mode:
            logger.info(
                "TEST MODE: Would execute 'sudo halt' command for critical battery")
            logger.info("TEST MODE: Critical battery halt sequence completed")
        else:
            logger.critical(
                "Executing halt command for critical battery preservation")
            subprocess.run(['sudo', 'halt'], check=True)
            logger.critical("Halt command executed - system should halt now")

    def _set_critical_battery_flag(self):
        """Set critical battery flag file for shutdown hook"""
        try:
            with open(CRITICAL_BATTERY_FLAG_FILE, 'w') as f:
                f.write(
                    f"CRITICAL_BATTERY_DETECTED\nTimestamp: {datetime.now().isoformat()}\n")
            logger.info(
                f"Critical battery flag set: {CRITICAL_BATTERY_FLAG_FILE}")
        except Exception as e:
            logger.error(f"Error setting critical battery flag: {e}")

    def _clear_critical_battery_flag(self):
        """Clear critical battery flag file"""
        try:
            if os.path.exists(CRITICAL_BATTERY_FLAG_FILE):
                os.remove(CRITICAL_BATTERY_FLAG_FILE)
                logger.info("Critical battery flag cleared")
        except Exception as e:
            logger.error(f"Error clearing critical battery flag: {e}")

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up power controller...")

        try:
            # Stop battery monitoring
            self.battery_monitoring_active = False

            # Stop SOS LED pattern if active
            self.sos_led_active = False

            # Clear critical battery flag if set
            if self.critical_battery_detected:
                self._clear_critical_battery_flag()

            # Cancel any pending tap timeout timer
            if self.tap_timeout_timer:
                self.tap_timeout_timer.cancel()
                self.tap_timeout_timer = None

            # Turn off LEDs
            self.set_green_led(False)
            self.set_blue_led(False)

            # Close GPIO chip if available
            if hasattr(self, 'chip') and self.chip is not None:
                self.chip.close()

            # ROS2 cleanup
            try:
                logger.info("Shutting down ROS2 components...")
                self.destroy_node()
            except Exception as e:
                logger.error(f"Error during ROS2 cleanup: {e}")

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

        logger.info("Power controller cleanup complete")


def test_gradual_frequency_pattern(threshold):
    """Test the gradual frequency pattern"""
    print(f"Testing gradual frequency pattern for {threshold}s threshold...")
    print(
        f"Frequency will increase from {LED_PRESS_START_FREQUENCY_HZ}Hz to {LED_PRESS_END_FREQUENCY_HZ}Hz")
    print(f"Duty cycle: {LED_PRESS_DUTY_CYCLE * 100}%")

    start_time = time.time()

    while time.time() - start_time < threshold:
        # Calculate elapsed time and progress (0.0 to 1.0)
        elapsed_time = time.time() - start_time
        progress = min(elapsed_time / threshold, 1.0)

        # Calculate current frequency (2Hz to 20Hz)
        current_frequency = LED_PRESS_START_FREQUENCY_HZ + \
            (LED_PRESS_END_FREQUENCY_HZ - LED_PRESS_START_FREQUENCY_HZ) * progress

        # Calculate timing for 50% duty cycle
        period = 1.0 / current_frequency
        on_time = period * LED_PRESS_DUTY_CYCLE
        off_time = period * (1.0 - LED_PRESS_DUTY_CYCLE)

        # Show current frequency every 0.5 seconds
        if int(elapsed_time * 2) != int((elapsed_time - 0.01) * 2):
            print(
                f"Time: {elapsed_time:.1f}s, Frequency: {current_frequency:.1f}Hz", flush=True)

        # Turn on both LEDs
        print("LED ON", flush=True)
        time.sleep(on_time)

        # Turn off both LEDs
        print("LED OFF", flush=True)
        time.sleep(off_time)

    print("LED OFF - Pattern complete")


def test_shutdown_pattern():
    """Test the shutdown LED pattern"""
    print("Testing shutdown LED pattern (1Hz, 20% duty cycle)...")

    # Calculate timing for 1Hz with 20% duty cycle
    period = 1.0 / LED_SHUTDOWN_FREQUENCY_HZ  # 1 second
    on_time = period * LED_SHUTDOWN_DUTY_CYCLE  # 0.2 seconds
    off_time = period * (1.0 - LED_SHUTDOWN_DUTY_CYCLE)  # 0.8 seconds

    print(f"Period: {period}s, On time: {on_time}s, Off time: {off_time}s")

    for cycle in range(5):  # Show 5 cycles
        print(f"Cycle {cycle + 1}: LED ON", flush=True)
        time.sleep(on_time)
        print(f"Cycle {cycle + 1}: LED OFF", flush=True)
        time.sleep(off_time)

    print("LED OFF - Shutdown pattern complete")


def print_help():
    """Print help message"""
    help_text = """
Orange Pi Zero 2W Power Control System
=====================================

DESCRIPTION:
  Intelligent power control system for Orange Pi Zero 2W using external relays
  and GPIO pins. Implements safe shutdown procedures, power button monitoring,
  and LED status indicators. Relay de-energization is handled automatically
  by GPIO pin state reversion on system halt.

HARDWARE CONFIGURATION:
  - PI3 (Pin 40): !POW - Open drain output to control power relay
  - PI9 (Pin 28): !POW_BUT - Input from power button (external pullup required)
  - PH4 (Pin 18): Green LED in power button (system running indicator)
  - PI1 (Pin 12): Blue LED in power button (status/warning indicator)
  - Red LED: Directly connected to power button (not GPIO controlled)

POWER BUTTON BEHAVIOR:
  - Short press (< threshold DEFAULT_SHUTDOWN_THRESHOLD_S): No action
  - Double tap (2 quick taps within 1.5s): Toggle Argo launch service (start/stop all nodes)
  - Triple tap (3 quick taps within 1.5s): Toggle recording on/off
  - Long press (>= threshold DEFAULT_SHUTDOWN_THRESHOLD_S): Initiate shutdown sequence
  - Only new button presses after service startup are detected (prevents shutdown during boot)

LED INDICATORS:
  - Green LED: Heartbeat when system is running (1Hz normal, 2Hz when Argo nodes running, 3-flash pattern when recording)
  - Both LEDs: Countdown pattern during button press (5,4,3,2,1 flashes at 3Hz, then 10 rapid flashes at 10Hz)
  - Both LEDs: Alternating short-long pattern during shutdown sequence
  - Red LED: Directly connected to power button (not controlled by GPIO)

USAGE:
  ./argo_power_control.py [OPTIONS]

OPTIONS:
  --help, -h          Show this help message and exit
  --test-mode, -t     Run in test mode (disable actual shutdown and power control)
  --threshold, -T     Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
  --test-wall-message, -w  Test wall message functionality and exit (safe for testing)
  --test-notification, -n  Test desktop notification functionality and exit (safe for testing)
  --debug-shutdown-flashing, -d  Test shutdown countdown LED pattern and exit (safe for testing)
  --simulate-double-tap          Simulate a double tap to toggle Argo service
  --simulate-triple-tap          Simulate a triple tap to toggle recording
  --simulate-critical-battery    Simulate critical battery condition for testing
  --simulate-low-battery         Simulate low battery condition (SOS LED pattern) for testing
EXAMPLES:
  ./argo_power_control.py                         # Normal operation
  ./argo_power_control.py --test-mode             # Test mode (safe)
  ./argo_power_control.py -t                      # Test mode (short form)
  ./argo_power_control.py --threshold 2.0         # 2-second threshold
  ./argo_power_control.py -T 2.0                  # 2-second threshold (short form)
  ./argo_power_control.py --test-wall-message     # Test wall message (safe)
  ./argo_power_control.py -w                      # Test wall message (short form)
  ./argo_power_control.py --test-notification     # Test desktop notification (safe)
  ./argo_power_control.py -n                      # Test desktop notification (short form)
  ./argo_power_control.py --debug-shutdown-flashing # Test countdown LED pattern (safe)
  ./argo_power_control.py -d                      # Test countdown LED pattern (short form)
  ./argo_power_control.py --simulate-double-tap   # Simulate double tap (Argo service toggle)
  ./argo_power_control.py --simulate-triple-tap   # Simulate triple tap (recording toggle)
  ./argo_power_control.py --simulate-critical-battery  # Test critical battery halt sequence
  ./argo_power_control.py --simulate-low-battery   # Test low battery SOS LED pattern

REQUIREMENTS:
  - User must be member of 'gpio' group (for GPIO access)
  - python3-gpiod library installed
  - External pullup resistor on power button input
  - Proper hardware connections as described above

SAFETY FEATURES:
  - Open drain configuration prevents damage from multiple control sources
  - Graceful shutdown ensures proper system shutdown before cutting power
  - GPIO pins automatically revert to input state on halt, de-energizing relay
  - External pullup resistor requirement prevents floating inputs
  - GPIO group membership requirement for controlled GPIO access
"""
    print(help_text)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Orange Pi Zero 2W Power Control System',
        add_help=False
    )
    parser.add_argument('--help', '-h', action='store_true',
                        help='Show help message and exit')
    parser.add_argument('--test-mode', '-t', action='store_true',
                        help='Run in test mode (disable actual shutdown and power control)')
    parser.add_argument('--threshold', '-T', type=float, default=DEFAULT_SHUTDOWN_THRESHOLD_S,
                        help=f'Set button press threshold for shutdown in seconds (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})')
    parser.add_argument('--test-wall-message', '-w', action='store_true',
                        help='Test wall message functionality and exit (safe for testing)')
    parser.add_argument('--test-notification', '-n', action='store_true',
                        help='Test desktop notification functionality and exit (safe for testing)')
    parser.add_argument('--debug-shutdown-flashing', '-d', action='store_true',
                        help='Test shutdown countdown LED pattern and exit (safe for testing)')
    parser.add_argument('--simulate-double-tap', action='store_true',
                        help='Simulate a double tap to toggle Argo service')
    parser.add_argument('--simulate-triple-tap', action='store_true',
                        help='Simulate a triple tap to toggle recording')
    parser.add_argument('--simulate-critical-battery', action='store_true',
                        help='Simulate critical battery condition for testing')
    parser.add_argument('--simulate-low-battery', action='store_true',
                        help='Simulate low battery condition (SOS LED pattern) for testing')

    args = parser.parse_args()

    # Handle help
    if args.help:
        print_help()
        sys.exit(0)

    # Handle wall message test
    if args.test_wall_message:
        print("Testing wall message functionality...")
        try:
            # Test wall message without GPIO initialization
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message = f"TEST: SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nThis is a test message - the system will NOT shutdown (would shutdown immediately)."

            print(f"Broadcasting test message: {message}")
            subprocess.run(['wall', message], check=True,
                           timeout=WALL_MESSAGE_TIMEOUT_S)
            print("Wall message test completed successfully!")
            print("Check your terminal - you should have received the wall message.")
        except subprocess.TimeoutExpired:
            print("Wall command timed out - message may not have been sent")
            sys.exit(1)
        except subprocess.CalledProcessError as e:
            print(f"Failed to broadcast wall message: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error during wall message test: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle notification test
    if args.test_notification:
        print("Testing desktop notification functionality...")
        try:
            # Test different notification types
            print("Sending test notifications...")

            # Test normal notification
            subprocess.run([
                'notify-send',
                '--urgency', 'normal',
                '--expire-time', '3000',
                'Power Control Test',
                'Normal notification test - power button functionality'
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            print("✅ Normal notification sent")

            # Test critical notification
            subprocess.run([
                'notify-send',
                '--urgency', 'critical',
                '--expire-time', '5000',
                'System Shutdown',
                'Critical notification test - system shutdown warning'
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            print("✅ Critical notification sent")

            # Test low priority notification
            subprocess.run([
                'notify-send',
                '--urgency', 'low',
                '--expire-time', '2000',
                'Power Button',
                'Low priority notification test - short press detected'
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            print("✅ Low priority notification sent")

            print("Desktop notification test completed successfully!")
            print("Check your desktop - you should have received 3 test notifications.")
        except subprocess.TimeoutExpired:
            print("Desktop notification timed out - check if notify-send is working")
            sys.exit(1)
        except subprocess.CalledProcessError as e:
            print(f"Failed to send desktop notification: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error during desktop notification test: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle debug shutdown flashing test
    if args.debug_shutdown_flashing:
        print("Testing LED patterns...")
        try:
            # Test gradual frequency pattern with different thresholds
            test_thresholds = [2.0, 3.0, 5.0]

            for threshold in test_thresholds:
                print(
                    f"\n=== Testing gradual frequency pattern with {threshold}s threshold ===")
                test_gradual_frequency_pattern(threshold)
                print(f"=== End {threshold}s threshold test ===\n")
                time.sleep(1)  # Brief pause between tests

            # Test shutdown pattern
            print("\n=== Testing shutdown LED pattern ===")
            test_shutdown_pattern()
            print("=== End shutdown pattern test ===\n")

            print("LED pattern tests completed successfully!")
            print("Check the output above to verify the patterns work correctly.")
        except Exception as e:
            print(f"Error during LED pattern test: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate double tap (Argo service toggle)
    if args.simulate_double_tap:
        print("Simulating double tap - Toggle Argo service")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            # Manually check and set service availability for test mode
            controller.check_recording_service_availability()
            controller.toggle_argo_service()
            print("✅ Double tap simulation completed")
        except Exception as e:
            print(f"Error simulating double tap: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate triple tap (Recording toggle)
    if args.simulate_triple_tap:
        print("Simulating triple tap - Toggle recording")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            controller.toggle_recording()
            print("✅ Triple tap simulation completed")
        except Exception as e:
            print(f"Error simulating triple tap: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate critical battery
    if args.simulate_critical_battery:
        print("Simulating critical battery condition with confirmation dialog")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            # Simulate critical battery voltage
            test_voltage = 6.2  # Below 7.2V threshold
            print(f"Simulating critical battery voltage: {test_voltage}V")
            print("This will show the confirmation dialog - you can test cancellation")
            controller.initiate_critical_battery_halt(test_voltage)
            print("✅ Critical battery simulation completed")
        except Exception as e:
            print(f"Error simulating critical battery: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate low battery
    if args.simulate_low_battery:
        print("Simulating low battery condition (SOS LED pattern)")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            # Simulate low battery voltage
            test_voltage = 7.4  # Below 7.6V threshold but above 7.2V critical
            print(f"Simulating low battery voltage: {test_voltage}V")
            controller.low_battery_detected = True
            # Start SOS LED pattern in separate thread
            import threading
            sos_thread = threading.Thread(target=controller.sos_led_pattern, daemon=True)
            sos_thread.start()
            print("✅ Low battery simulation completed - SOS LED pattern running")
            print("Press Ctrl+C to stop the SOS pattern test")
            # Let the SOS pattern run for demonstration
            try:
                while controller.sos_led_active:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nStopping SOS pattern test...")
                controller.sos_led_active = False
                controller.set_blue_led(False)
        except Exception as e:
            print(f"Error simulating low battery: {e}")
            sys.exit(1)
        sys.exit(0)

    # Check if user is in the gpio group (required for GPIO access)
    # Skip this check if running as root (systemd service)
    if os.geteuid() != 0:
        import grp
        try:
            gpio_group = grp.getgrnam('gpio')
            if gpio_group.gr_gid not in os.getgroups():
                print("Error: This script requires GPIO access")
                print("Your user must be a member of the 'gpio' group")
                print("Run: sudo usermod -a -G gpio $USER")
                print("Then log out and log back in, or run: newgrp gpio")
                sys.exit(1)
        except KeyError:
            print("Error: GPIO group not found")
            print("Please ensure the 'gpio' group exists and you are a member")
            sys.exit(1)
    else:
        print("Running as root - skipping GPIO group check")

    # Validate threshold
    if args.threshold <= 0:
        print("Error: Threshold must be greater than 0")
        sys.exit(1)

    # Show startup information
    if args.test_mode:
        print("Starting power control system in TEST MODE")
        print("  - Shutdown commands will be simulated (not executed)")
        print("  - Power relay control is DISABLED")
        print("  - Desktop notifications will be sent")
        print("  - Button presses and actions will be reported")
        print(
            f"  - Double tap (2 quick taps within {MULTI_TAP_MAX_DURATION_S}s) toggles Argo service")
        print(
            f"  - Triple tap (3 quick taps within {MULTI_TAP_MAX_DURATION_S}s) toggles recording")
    else:
        print("Starting power control system in NORMAL MODE")
        print("  - Full power control enabled")
        print("  - System will shutdown and cut power on long button press")
        print(
            f"  - Double tap (2 quick taps within {MULTI_TAP_MAX_DURATION_S}) toggles Argo service")
        print(
            f"  - Triple tap (3 quick taps within {MULTI_TAP_MAX_DURATION_S}) toggles recording")

    print(f"  - Button press threshold: {args.threshold} seconds")
    print(f"  - Button detection mode: Hardware interrupts (efficient)")
    print(
        f"  - Battery monitoring: Low warning {LOW_BATTERY_THRESHOLD_V}V (SOS LED), Critical {CRITICAL_BATTERY_THRESHOLD_V}V (halt)")
    print("  - Press Ctrl+C to stop")
    print()

    try:
        controller = PowerController(
            test_mode=args.test_mode, threshold=args.threshold)

        # Run the main GPIO monitoring loop
        controller.run()

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        # Cleanup
        try:
            controller.cleanup()
        except:
            pass


if __name__ == "__main__":
    main()
