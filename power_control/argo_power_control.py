#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
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
# HARDWARE CONFIGURATION (Rev3 PCB):
#   - PI3 (Pin 40): POW_OFF - Output for power relay control (active HIGH pulse to reset relay)
#   - PI9 (Pin 28): POW_BUT - Input from power button (active HIGH when pressed)
#   - PH4 (Pin 18): Green LED in power button (active LOW - cathode control)
#   - PI1 (Pin 12): Blue LED in power button (active LOW - cathode control)
#   - PI16 (Pin 37): Red LED: GPIO controlled (active LOW - cathode control) 
#   - LEDs: Common anode RGB LED, GPIO controls cathode (LOW = ON, HIGH = OFF)
#
# POWER CIRCUIT DESIGN (Rev3 PCB):
#   The system uses a latching relay with SET/RESET coils:
#   1. SET coil: Activated by power button press to latch power ON
#   2. RESET coil: Activated by active-HIGH pulse on POW_OFF to unlatch power
#
#   Button press activates SET coil → relay latches → system powers on
#   Software shutdown sends HIGH pulse to POW_OFF → RESET coil deactivates relay
#
# POWER BUTTON BEHAVIOR (Rev3 PCB - Active HIGH):
#   - Single tap: Toggle controller pause mode
#   - Double tap: Toggle recording
#   - Quadruple tap: Restart Argo launch service
#   - Long press (>= threshold DEFAULT_SHUTDOWN_THRESHOLD_S): Initiate shutdown sequence
#   - Button press at boot activates SET coil; by software start, button is already released
#
# LED INDICATORS (Rev3 PCB - Active LOW):
#   - Green LED: Heartbeat when system is running (1Hz normal, 2Hz with Argo, 3-flash when recording)
#   - Both LEDs: Gradual frequency increase during button press (2Hz → 20Hz)
#   - Both LEDs: Short-long pattern during shutdown sequence (1Hz, 5% duty)
#   - Blue LED: Show charging status via battery !CHARGING signal reported by the battery monitor
#   - Red LED: SOS patterns for low battery warning (slow SOS for low battery, fast SOS for critical battery)
#   - Red/Green LEDs: Alternating pattern when WiFi connectivity is lost (0.5Hz alternating)
#   - All LEDs: Active LOW control (GPIO LOW = LED ON, GPIO HIGH = LED OFF)
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
#
# SERVICE CLIENT PERFORMANCE OPTIMIZATION ALTERNATIVES
# ===================================================
#
# CURRENT IMPLEMENTATION: Ephemeral Service Clients
# - Creates and destroys ROS2 service clients for each call
# - Each call takes 2-4 seconds due to service discovery overhead
# - Reliable but slow for repeated calls
# - Used in _call_trigger_service() method
#
# ALTERNATIVE 1: Persistent Service Clients
# =========================================
# Pros:
#   - Much faster subsequent calls (~0.1-0.5s vs 2-4s)
#   - Better performance for frequent calls (battery monitoring every 30s)
#   - Reduced ROS2 daemon load
#
# Cons:
#   - Service discovery issues if services restart during development
#   - Stale references to services that no longer exist
#   - ROS2 daemon restart can invalidate clients
#   - Memory usage and file descriptor overhead
#   - May not detect service availability changes
#
# Implementation:
#   def __init__(self):
#       self.recording_start_client = self.ros2_node.create_client(Trigger, '/argo/recording/start')
#       self.recording_stop_client = self.ros2_node.create_client(Trigger, '/argo/recording/stop')
#       # ... other clients
#
#   def _call_persistent_service(self, client, timeout_sec=2.0):
#       if not client.service_is_ready():
#           return False, "Service not ready"
#       # ... make call
#
# ALTERNATIVE 2: Hybrid Approach with Fallback
# ============================================
# - Try persistent client first, fallback to ephemeral if needed
# - Best of both worlds: speed when possible, reliability when needed
#
# Implementation:
#   def _call_trigger_service(self, service_name, timeout_sec=2.0):
#       # Try persistent client first
#       if hasattr(self, f'{service_name.replace("/", "_")}_client'):
#           client = getattr(self, f'{service_name.replace("/", "_")}_client')
#           if client and client.service_is_ready():
#               return self._call_persistent_service(client, timeout_sec)
#       # Fallback to ephemeral client
#       return self._call_ephemeral_service(service_name, timeout_sec)
#
# ALTERNATIVE 3: Client Pool with TTL
# ==================================
# - Maintains clients for a limited time (e.g., 5 minutes)
# - Automatically recreates stale clients
# - Balances performance and reliability
#
# Implementation:
#   class ServiceClientPool:
#       def __init__(self, ros2_node):
#           self.clients = {}
#           self.last_used = {}
#           self.max_age = 300  # 5 minutes
#       
#       def get_client(self, service_name):
#           now = time.time()
#           if service_name in self.clients:
#               if now - self.last_used[service_name] < self.max_age:
#                   return self.clients[service_name]
#               else:
#                   # Recreate expired client
#                   self._recreate_client(service_name)
#           return self._create_client(service_name)
#
# ALTERNATIVE 4: Health-Checked Persistent Clients
# ===============================================
# - Persistent clients with automatic health checking
# - Recreates clients when they become unhealthy
# - Most robust but most complex
#
# Implementation:
#   def _check_client_health(self, client):
#       try:
#           return client.service_is_ready()
#       except:
#           return False
#
#   def _recreate_client_if_needed(self, service_name):
#       client_attr = f'{service_name.replace("/", "_")}_client'
#       if hasattr(self, client_attr):
#           client = getattr(self, client_attr)
#           if not self._check_client_health(client):
#               logger.warning(f"Recreating unhealthy client for {service_name}")
#               self.ros2_node.destroy_client(client)
#               setattr(self, client_attr, self.ros2_node.create_client(Trigger, service_name))
#
# PERFORMANCE IMPACT ANALYSIS
# ==========================
# Current (Ephemeral):     First call: 2-4s, Subsequent: 2-4s
# Persistent:              First call: 2-4s, Subsequent: 0.1-0.5s
# Client Pool:             First call: 2-4s, Subsequent: 0.1-0.5s (within TTL)
# Hybrid:                  First call: 2-4s, Subsequent: 0.1-0.5s (when persistent works)
#
# RECOMMENDATION FOR ARGO POWER CONTROL
# =====================================
# Current ephemeral approach is acceptable because:
# - Recording toggles are infrequent (user-initiated)
# - Controller pause toggles are occasional (user-initiated)
# - Battery monitoring every 30s is acceptable
# - System prioritizes reliability over speed
# - Development environment benefits from fresh clients
#
# Consider implementing Client Pool (Alternative 3) if:
# - Battery monitoring becomes more frequent
# - Service calls become a bottleneck
# - Performance testing shows significant delays
#
# To implement any alternative:
# 1. Replace _call_trigger_service() method
# 2. Add client management in __init__() and cleanup()
# 3. Test thoroughly in development environment
# 4. Monitor for service discovery issues
# 5. Add health checking if using persistent clients

import time
import signal
import sys
import subprocess
import threading
import logging
import os
import argparse
import argcomplete
from pathlib import Path
from datetime import datetime
import select
import tty
import termios
import json
from typing import Optional, Dict, Any

# isort: off

# ROS2 imports for service client
try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    Node = None
    Trigger = None
    ROS2_AVAILABLE = False

# Conditional GPIO import - required for production, optional for test mode
try:
    import gpiod
    _HAS_GPIOD = True
except ImportError:
    gpiod = None
    _HAS_GPIOD = False

# isort: on

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger, SetBool
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
DEFAULT_SHUTDOWN_THRESHOLD_S = 8.0
# Warn users this many seconds before shutdown to release the button
SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S = 4.0
# Button release polling frequency during press (10 Hz - only used during button press)
BUTTON_ERROR_RECOVERY_DELAY_S = 0.1     # Delay on button read error (seconds)
# Maximum duration for multiple taps to toggle states (seconds)
MULTI_TAP_MAX_DURATION_S = 3

# LED Heartbeat Configuration
LED_HEARTBEAT_HZ = 1.0                  # Green LED heartbeat frequency (1 Hz)
LED_PAUSED_HEARTBEAT_HZ = 0.5           # Slower heartbeat when controller is paused (0.5 Hz)

# Button Press Pattern Configuration
LED_PRESS_START_FREQUENCY_HZ = 2.0      # Starting flash frequency (2 Hz)
LED_PRESS_END_FREQUENCY_HZ = 20.0       # Ending flash frequency (20 Hz)
LED_PRESS_DUTY_CYCLE = 0.5              # 50% duty cycle during button press

# Shutdown Pattern Configuration
LED_SHUTDOWN_FREQUENCY_HZ = 1.0         # Shutdown flash frequency (1 Hz)
LED_SHUTDOWN_DUTY_CYCLE = 0.05           # 5% duty cycle during shutdown

# Button Action LED Feedback Configuration
LED_ACTION_FEEDBACK_DURATION_S = 2.0    # Duration for immediate action feedback (2 seconds)
LED_SERVICE_WAIT_FREQUENCY_HZ = 3.0     # R+G flashing frequency during service calls (3 Hz)
LED_SERVICE_WAIT_DUTY_CYCLE = 0.5       # 50% duty cycle for R+G flashing

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
BATTERY_LOG_THRESHOLD_V = 0.05     # Only log battery voltage if it changes by more than 50mV
# Flag file for shutdown hook
CRITICAL_BATTERY_FLAG_FILE = '/tmp/argo_critical_battery'

# WiFi Monitoring
WIFI_MONITORING_INTERVAL_S = 10     # Check WiFi status interval (seconds)
WIFI_CONNECTIVITY_TIMEOUT_S = 5     # Timeout for WiFi connectivity tests (seconds)
WIFI_LOSS_LED_FREQUENCY_HZ = 0.5    # Frequency for alternating red/green pattern (0.5Hz = 2 second cycle)

# DEVELOPMENT FLAG: Critical Battery Behavior
# Set to True to use normal shutdown (cuts power) instead of halt (preserves power)
# This is useful during development/testing when you want the system to fully power off
# on critical battery instead of preserving power for manual sailing.
# Production default: False (use halt to preserve power for manual sailing)
CRITICAL_BATTERY_USE_SHUTDOWN = True  # True = shutdown (cuts power), False = halt (preserves power)

# Hardware Polarity Configuration (Rev3 PCB)
# Power button: Active HIGH (1 = pressed, 0 = released)
BUTTON_PRESSED_STATE = 1
BUTTON_RELEASED_STATE = 0
# LEDs: Active LOW (0 = ON, 1 = OFF) - Common anode RGB LED with cathode control
LED_ON_STATE = 0
LED_OFF_STATE = 1
# Power relay control pulse polarity for shutdown hook (active HIGH to reset relay)
POWER_OFF_PULSE_STATE = 1

# Configure logging


def setup_logging(debug=False):
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

    # Set logging level based on debug flag
    log_level = logging.DEBUG if debug else logging.INFO
    
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=handlers
    )


# setup_logging() will be called in main() after parsing arguments
logger = logging.getLogger('argo_power_control')


class PowerController:
    def __init__(self, test_mode=False, threshold=1.0):
        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        self.gpio_available = False  # Will be set to True if GPIO initialization succeeds
        
        # CRITICAL: Verify gpiod is available in production mode
        if not test_mode and not _HAS_GPIOD:
            logger.critical("FATAL: gpiod library not available - required for production mode!")
            logger.critical("This system CANNOT run safely without GPIO control.")
            logger.critical("Install gpiod: sudo apt-get install python3-libgpiod")
            logger.critical("If testing remotely, use --test-mode flag")
            raise RuntimeError("gpiod library not available - cannot run in production mode without GPIO control")
        
        if test_mode and not _HAS_GPIOD:
            logger.warning("TEST MODE: gpiod library not available - GPIO functionality will be simulated")
            logger.warning("This is OK for remote testing, but NOT for production deployment!")

        # Button state tracking
        self.initial_button_state = None
        # Flag to track when button detection should be active
        self.button_detection_active = False
        # Flag to track if warning notification has been sent for current button press
        self.warning_notification_sent = False

        # Tap detection for various commands
        self.tap_times = []  # List to store tap timestamps
        self.tap_timeout_timer = None  # Timer for tap detection timeout
        self.tap_timeout_duration = 1.0  # Wait 1 second after last tap before processing

        # Argo service status tracking
        self.argo_service_running = False

        # LED state tracking
        self.green_led_state = False
        self.blue_led_state = False

        # Heartbeat control
        self.heartbeat_paused = False
        self.heartbeat_pause_event = threading.Event()
        self.heartbeat_pause_event.set()  # Start un-paused
        self.heartbeat_frequency_hz = LED_HEARTBEAT_HZ

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
        self.last_logged_battery_voltage = None  # Track last logged voltage for throttling

        # WiFi monitoring state
        self.wifi_connected = True  # Assume connected at startup
        self.wifi_loss_detected = False
        self.last_wifi_check_time = 0.0
        self.wifi_monitoring_active = False
        self.wifi_loss_led_active = False

        # Service wait pattern state
        self.service_wait_active = False

        # GPIO Configuration
        self.GPIO_CHIP = '/dev/gpiochip0'

        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 265   # PI9 (Pin 28) - !POW_BUT
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED
        self.BLUE_LED_LINE = 257       # PI1 (Pin 12) - Blue LED
        self.RED_LED_LINE = 272        # PI16 (Pin 37) - Red LED

        # Button press threshold (seconds)
        self.SHUTDOWN_THRESHOLD = threshold

        # Initialize ROS2 for service clients (allow in test mode for service testing)
        self.ros2_node = None
        self.power_services_created = False
        self.power_status_publisher = None
        self.button_event_publisher = None
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                # Use a unique name to avoid conflicts if other nodes are on the same process
                self.ros2_node = Node('argo_power_control', allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
                logger.info("ROS2 node initialized for power control")
            except Exception as e:
                logger.error(f"Failed to initialize ROS2 node: {e}")
                self.ros2_node = None
        elif not ROS2_AVAILABLE:
            logger.warning("rclpy not found. ROS2 service calls will be disabled.")

        # Setup signal handlers for graceful service shutdown
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGQUIT, self.signal_handler)

        # Simple state tracking - initialize these early for test mode compatibility
        self.recording_active = False
        self.recording_service_available = False
        
        # Throttled logging for recording service availability
        self.last_logged_recording_service_state = None
        
        # Throttling for frequent checks
        self.last_recording_check_time = 0.0
        self.last_logged_system_state = None
        
        # Controller pause state tracking
        self.controller_pause_state = False

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
        
        # Initialize GPIO and configure button interrupts
        # Note: Button press activates SET coil which powers on the system,
        # so by the time software starts, button has already been released.
        # No need to check initial state - button detection is always active.
        self.init_gpio()
        
        # Configure button line for interrupt-based monitoring
        if self.gpio_available:
            self.configure_button_for_interrupts()
        
        self.button_detection_active = True
        logger.info("Button detection active - ready to detect presses")

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
        # Skip GPIO initialization if gpiod is not available in test mode
        if not _HAS_GPIOD:
            if self.test_mode:
                logger.info("Skipping GPIO initialization - gpiod not available (test mode)")
                self.gpio_available = False
                # Set dummy values for test mode
                self.chip = None
                self.power_button_line = None
                self.green_led_line = None
                self.blue_led_line = None
                self.red_led_line = None
                return
            else:
                # Should never reach here due to check in __init__
                raise RuntimeError("gpiod not available in production mode!")
        
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
            self.red_led_line = self.chip.get_line(self.RED_LED_LINE)

            # Initially request power button line as input to read initial state
            # Will be reconfigured for interrupts after reading initial state
            self.power_button_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
            )

            # Request LED lines (active-low: HIGH = OFF)
            self.green_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[LED_OFF_STATE]  # Start with LED off (HIGH for active-low)
            )
            self.blue_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[LED_OFF_STATE]  # Start with LED off (HIGH for active-low)
            )
            self.red_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[LED_OFF_STATE]  # Start with LED off (HIGH for active-low)
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
                self.red_led_line = None
            else:
                logger.error(f"Failed to initialize GPIO: {e}")
                logger.error(f"GPIO chip path attempted: {self.GPIO_CHIP}")
                logger.error(f"Full error details: {type(e).__name__}: {e}")
                raise

        # No ROS2 setup needed - using subprocess calls only

    def start_recording(self):
        """Start recording via ROS2 service call"""
        if self.recording_active:
            logger.warning("Recording is already active - skipping start recording")
            self.send_desktop_notification(
                "Recording Error", "Recording is already active", "warning")
            return False

        # Check if Argo service is running (required for ROS2 services to be up)
        if not self.check_argo_service_status():
            logger.error(
                "Argo service is not running - cannot start recording")
            self.send_desktop_notification(
                "Recording Error", "Argo service is not running", "critical")
            return False

        logger.info("🎬 Calling recording start service...")
        success, message = self._call_trigger_service(
            '/argo/recording/start', timeout_sec=6.0)

        if success:
            logger.info(f"✅ Recording started: {message}")
            self.recording_active = True
            self.send_desktop_notification("Recording Started", message, "normal")
            return True
        else:
            # Check if it failed because it was already running
            if "already in progress" in message.lower() or "already active" in message.lower():
                logger.info(f"📹 Recording was already in progress: {message}")
                self.recording_active = True  # Sync state
                self.send_desktop_notification(
                    "Recording Status", f"Recording was already active: {message}", "normal")
                return True
            else:
                logger.error(f"❌ Recording start failed: {message}")
                self.send_desktop_notification(
                    "Recording Error", f"Failed to start recording: {message}", "critical")
                return False

    def stop_recording(self):
        """Stop recording via ROS2 service call"""
        if not self.recording_active:
            logger.warning("Recording is not active - skipping stop recording")
            self.send_desktop_notification(
                "Recording Error", "Recording is not active", "warning")
            return False

        logger.info("⏹️ Calling recording stop service...")
        success, message = self._call_trigger_service(
            '/argo/recording/stop', timeout_sec=15.0)

        if success:
            logger.info(f"✅ Recording stopped: {message}")
            self.recording_active = False
            self.send_desktop_notification("Recording Stopped", message, "normal")
            return True
        else:
            logger.error(f"❌ Recording stop failed: {message}")
            self.send_desktop_notification(
                "Recording Error", f"Failed to stop recording: {message}", "critical")
            return False

    def check_recording_service_availability(self):
        """Check if recording service is available using a service client."""
        if not self.ros2_node:
            self.recording_service_available = False
            return False
        if not self.check_argo_service_status():
            self.recording_service_available = False
            return False

        # Use a more reliable approach - try to call the get_status service
        # This both checks availability and gets current state
        try:
            success, message = self._call_trigger_service(
                '/argo/recording/get_status', timeout_sec=2.0)
            if success is not None:  # Service responded (regardless of recording state)
                # Throttled logging - only log when state changes
                if self.last_logged_recording_service_state != True:
                    logger.info("Recording service is available")
                    self.last_logged_recording_service_state = True
                self.recording_service_available = True
                return True
            else:
                # Throttled logging - only log when state changes
                if self.last_logged_recording_service_state != False:
                    logger.info("Recording service not available (no response)")
                    self.last_logged_recording_service_state = False
                self.recording_service_available = False
                return False
        except Exception as e:
            # Throttled logging - only log when state changes
            if self.last_logged_recording_service_state != False:
                logger.info(f"Recording service not available: {e}")
                self.last_logged_recording_service_state = False
            self.recording_service_available = False
            return False

    def query_current_recording_status(self):
        """Query current recording status by calling the get_status service."""
        if not self.check_recording_service_availability():
            # Throttled logging - only log when state changes
            if self.last_logged_recording_service_state != False:
                logger.info(
                    "Recording service not available - skipping status query")
            self.recording_active = False  # Assume not recording
            return

        success, message = self._call_trigger_service(
            '/argo/recording/get_status', timeout_sec=2.0)

        # The 'success' field of the Trigger response indicates the recording state
        old_state = self.recording_active
        self.recording_active = success

        if old_state != self.recording_active:
            logger.info(
                f"📹 Recording status changed: {'INACTIVE' if old_state else 'ACTIVE'} → {'ACTIVE' if self.recording_active else 'INACTIVE'} (state synchronized)")

    def force_recording_state_sync(self):
        """Force synchronization of recording state - useful before critical operations"""
        logger.info("🔄 Forcing recording state synchronization...")
        self.query_current_recording_status()
        logger.info(f"📹 Current recording state: {'ACTIVE' if self.recording_active else 'INACTIVE'}")
        return self.recording_active

    def is_argo_system_running(self):
        """Check if the Argo system is running using simple flag approach"""
        # Only check recording service availability if we haven't checked recently
        # This reduces frequent service calls
        current_time = time.time()
        if (self.argo_service_running and 
            (current_time - self.last_recording_check_time) > 10.0):  # Check every 10 seconds max
            self.check_recording_service_availability()
            self.last_recording_check_time = current_time

        # Only log when state changes to reduce log spam
        current_state = self.argo_service_running and self.recording_service_available
        if current_state != self.last_logged_system_state:
            logger.info(
                f"Argo system state: {'RUNNING' if current_state else 'STOPPED'} "
                f"(service: {self.argo_service_running}, recording: {self.recording_service_available})")
            self.last_logged_system_state = current_state
            
        return current_state

    def check_argo_service_status(self):
        """Check if the Argo launch service is running via ROS2 service call"""
        try:
            # Use ROS2 service call instead of systemctl for better integration
            if self.ros2_node:
                success, message = self._call_trigger_service('/argo/lifecycle/status', timeout_sec=1.0)
                if success:
                    # Parse the status response to determine if system is running
                    import json
                    try:
                        status_data = json.loads(message)
                        running_count = status_data.get('running_count', 0)
                        total_count = status_data.get('total_count', 0)
                        # Consider system running if at least 3 nodes are active
                        is_running = running_count >= 3
                        
                        if is_running != self.argo_service_running:
                            self.argo_service_running = is_running
                            logger.info(
                                f"Argo service status changed: {'RUNNING' if is_running else 'STOPPED'} ({running_count}/{total_count} nodes)")
                            # Update LED heartbeat when Argo service state changes
                            self._update_led_heartbeat_for_pause_state()

                            # When Argo service stops, recording service is not available
                            if not is_running:
                                self.recording_service_available = False
                                logger.info("Recording service marked as unavailable")
                        
                        return is_running
                    except (json.JSONDecodeError, KeyError):
                        # Fallback to systemctl if ROS2 service fails
                        pass
            
            # Fallback to systemctl check (less frequent, shorter timeout)
            result = subprocess.run(
                ['sudo', 'systemctl', 'is-active', 'argo_launch.service'],
                capture_output=True, text=True, timeout=2
            )
            is_running = result.returncode == 0 and result.stdout.strip() == 'active'
            if is_running != self.argo_service_running:
                self.argo_service_running = is_running
                logger.info(
                    f"Argo service status changed: {'RUNNING' if is_running else 'STOPPED'}")
                # Update LED heartbeat when Argo service state changes
                self._update_led_heartbeat_for_pause_state()

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
                ['sudo', 'systemctl', 'start', 'argo_launch.service'],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                logger.info(
                    "✅ Argo launch service started, waiting for recording service to initialize...")
                # Set Argo service flag, but recording service needs time to initialize
                self.argo_service_running = True
                # Update LED heartbeat when Argo service starts
                self._update_led_heartbeat_for_pause_state()
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
                ['sudo', 'systemctl', 'stop', 'argo_launch.service'],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode == 0:
                logger.info("✅ Argo launch service stopped")
                # Set flags to indicate Argo system is stopped
                self.argo_service_running = False
                self.recording_service_available = False
                # Update LED heartbeat when Argo service stops
                self._update_led_heartbeat_for_pause_state()
                return True
            else:
                logger.error(
                    f"❌ Failed to stop Argo service: {result.stderr}")
                return False
        except Exception as e:
            logger.error(f"Error stopping Argo service: {e}")
            return False

    def toggle_controller_pause(self):
        """Toggle controller pause mode via ROS2 service call"""
        try:
            # Check if Argo service is running (required for controller node)
            if not self.check_argo_service_status():
                logger.warning("Single tap detected but Argo service is not running")
                self.send_desktop_notification(
                    "Controller Pause Unavailable",
                    "Argo service must be running to toggle pause mode",
                    "warning"
                )
                return
            
            logger.info("Single tap detected - toggling controller pause mode")
            self.send_desktop_notification(
                "Controller Pause Control",
                "Toggling controller pause mode...",
                "normal"
            )
            
            # Start service wait pattern for delayed service call
            self.service_wait_active = True
            service_wait_thread = threading.Thread(
                target=self.service_wait_pattern,
                daemon=True
            )
            service_wait_thread.start()
            
            # Get current pause state and toggle it
            current_pause_state = self._get_controller_pause_state()
            new_pause_state = not current_pause_state
            success, message = self._call_controller_pause_service(new_pause_state)
            
            # Stop service wait pattern
            self.service_wait_active = False
            
            if success:
                # The service response message should indicate the new state
                logger.info(f"✅ Controller pause toggled: {message}")
                self.send_desktop_notification(
                    "Controller Pause Toggled",
                    message,
                    "normal"
                )
                
                # Update LED heartbeat frequency based on new pause state
                self._update_led_heartbeat_for_pause_state()
            else:
                logger.error(f"❌ Failed to toggle controller pause: {message}")
                self.send_desktop_notification(
                    "Controller Pause Error",
                    f"Failed to toggle pause mode: {message}",
                    "critical"
                )
        except Exception as e:
            logger.error(f"Error toggling controller pause: {e}")
            self.send_desktop_notification(
                "Controller Pause Error",
                f"Error toggling pause mode: {e}",
                "critical"
            )

    def _get_controller_pause_state(self) -> bool:
        """Get current controller pause state from cached topic data."""
        try:
            # Use the cached pause state from the topic subscription
            # This is more reliable than trying to query the service
            return getattr(self, 'controller_pause_state', False)
                
        except Exception as e:
            logger.error(f"Error getting controller pause state: {e}")
            return False

    def _call_controller_pause_service(self, pause_state: bool) -> tuple[bool, str]:
        """Call the controller pause service with the specified state."""
        try:
            if not self.ros2_node:
                return False, "ROS2 node not available"
            
            # Create service client for controller pause service
            pause_client = self.ros2_node.create_client(SetBool, '/controller_node/pause')
            
            if not pause_client.wait_for_service(timeout_sec=2.0):
                return False, "Controller pause service not available"
            
            # Create request
            request = SetBool.Request()
            request.data = pause_state
            
            # Call service
            future = pause_client.call_async(request)
            rclpy.spin_until_future_complete(self.ros2_node, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                return response.success, response.message
            else:
                return False, "Service call timed out"
                
        except Exception as e:
            return False, f"Error calling controller pause service: {e}"

    def restart_argo_service(self):
        """Restart the Argo launch service"""
        try:
            logger.info("Quadruple tap detected - restarting Argo launch service")
            self.send_desktop_notification(
                "Argo Service Control",
                "Restarting Argo nodes...",
                "normal"
            )
            
            if self.test_mode:
                logger.info("TEST MODE: Would restart Argo launch service")
                self.send_desktop_notification(
                    "Argo Service Restarted",
                    "Argo nodes restarted by quadruple tap (test mode)",
                    "normal"
                )
                return True
            
            # Start service wait pattern for delayed service restart
            self.service_wait_active = True
            service_wait_thread = threading.Thread(
                target=self.service_wait_pattern,
                daemon=True
            )
            service_wait_thread.start()
            
            # Stop service first
            result = subprocess.run(
                ['sudo', 'systemctl', 'stop', 'argo_launch.service'],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode != 0:
                logger.error(f"❌ Failed to stop Argo service: {result.stderr}")
                self.send_desktop_notification(
                    "Argo Service Error",
                    "Failed to stop Argo nodes for restart",
                    "critical"
                )
                # Stop service wait pattern
                self.service_wait_active = False
                return False
            
            # Wait a moment for clean shutdown
            time.sleep(2)
            
            # Start service
            result = subprocess.run(
                ['sudo', 'systemctl', 'start', 'argo_launch.service'],
                capture_output=True, text=True, timeout=15
            )
            
            # Stop service wait pattern
            self.service_wait_active = False
            if result.returncode == 0:
                logger.info("✅ Argo launch service restarted")
                self.argo_service_running = True
                self.send_desktop_notification(
                    "Argo Service Restarted",
                    "Argo nodes restarted by quadruple tap",
                    "normal"
                )
                # Wait for recording service to initialize
                time.sleep(7)
                self.check_recording_service_availability()
                return True
            else:
                logger.error(f"❌ Failed to start Argo service after restart: {result.stderr}")
                self.argo_service_running = False
                self.send_desktop_notification(
                    "Argo Service Error",
                    "Failed to start Argo nodes after restart",
                    "critical"
                )
                return False
        except Exception as e:
            logger.error(f"Error restarting Argo service: {e}")
            self.send_desktop_notification(
                "Argo Service Error",
                f"Error restarting Argo nodes: {e}",
                "critical"
            )
            return False

    def toggle_argo_service(self):
        """Toggle the Argo launch service on/off with immediate feedback"""
        try:
            # First check the actual service status to ensure our flag is accurate
            is_currently_running = self.check_argo_service_status()
            old_state_running = self.argo_service_running
            self.argo_service_running = is_currently_running

            if is_currently_running:
                logger.info("Double tap detected - requesting to stop Argo service")
                self.send_desktop_notification(
                    "Argo Service Control",
                    "Requesting to stop Argo nodes...",
                    "normal"
                )
                # Immediately update LED heartbeat to reflect pending state change
                self.set_heartbeat_frequency(LED_HEARTBEAT_HZ)  # Pending stopped state

                success = self.stop_argo_service()
                if success:
                    self.send_desktop_notification(
                        "Argo Service Stopped",
                        "Argo nodes stopped by double tap",
                        "normal"
                    )
                else:
                    self.send_desktop_notification(
                        "Argo Service Error",
                        "Failed to stop Argo nodes",
                        "critical"
                    )
                    # Revert state on failure
                    self.argo_service_running = True
                    self.set_heartbeat_frequency(LED_HEARTBEAT_HZ * 2.0)
            else:
                logger.info("Double tap detected - requesting to start Argo service")
                self.send_desktop_notification(
                    "Argo Service Control",
                    "Requesting to start Argo nodes...",
                    "normal"
                )
                # Immediately update LED heartbeat to reflect pending state change
                self.set_heartbeat_frequency(LED_HEARTBEAT_HZ * 2.0)  # Pending running state

                success = self.start_argo_service()
                if success:
                    self.send_desktop_notification(
                        "Argo Nodes Started",
                        "Argo nodes started by double tap",
                        "normal"
                    )
                else:
                    self.send_desktop_notification(
                        "Argo Service Error",
                        "Failed to start Argo nodes",
                        "critical"
                    )
                    # Revert state on failure
                    self.argo_service_running = False
                    self.set_heartbeat_frequency(LED_HEARTBEAT_HZ)

        except Exception as e:
            logger.error(f"Error toggling Argo service: {e}")
            self.send_desktop_notification(
                "Argo Service Error",
                f"Error toggling Argo nodes launch service: {e}",
                "critical"
            )
            # Revert to old state on exception
            self.argo_service_running = old_state_running
            self.set_heartbeat_frequency(
                LED_HEARTBEAT_HZ * 2.0 if old_state_running else LED_HEARTBEAT_HZ)

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

    def configure_button_for_interrupts(self):
        """Configure button line for interrupt-based monitoring (Rev3 PCB: active-high)"""
        if not self.gpio_available:
            logger.info(
                "GPIO not available - skipping button interrupt configuration in test mode")
            return
        try:
            # Release the initial input configuration
            self.power_button_line.release()

            # Configure for rising edge interrupts (Rev3 PCB: active-high button)
            # Button press = rising edge (0 → 1 transition)
            # Note: Button activates SET coil to power on system, so it's always
            # released by the time software starts. No startup state checking needed.
            self.power_button_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_EV_RISING_EDGE,  # Detect POW_BUT going high (pressed)
                flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
            )
            logger.info(
                "Button line configured for rising edge interrupt monitoring (active-high)")

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
            return BUTTON_RELEASED_STATE  # Assume not pressed on error

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
            self.green_led_state = state
            return
        try:
            # Active-low LED: state=True (ON) → GPIO=0 (LOW), state=False (OFF) → GPIO=1 (HIGH)
            value = LED_ON_STATE if state else LED_OFF_STATE
            self.green_led_line.set_value(value)
            self.green_led_state = state
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED (charging indicator)"""
        if not self.gpio_available:
            self.blue_led_state = state
            return
        try:
            # Active-low LED: state=True (ON) → GPIO=0 (LOW), state=False (OFF) → GPIO=1 (HIGH)
            value = LED_ON_STATE if state else LED_OFF_STATE
            self.blue_led_line.set_value(value)
            self.blue_led_state = state
        except Exception as e:
            logger.error(f"Error controlling blue LED: {e}")

    def set_red_led(self, state):
        """Control red LED (battery warning/SOS indicator)"""
        if not self.gpio_available:
            return
        try:
            # Active-low LED: state=True (ON) → GPIO=0 (LOW), state=False (OFF) → GPIO=1 (HIGH)
            value = LED_ON_STATE if state else LED_OFF_STATE
            self.red_led_line.set_value(value)
        except Exception as e:
            logger.error(f"Error controlling red LED: {e}")

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
            self.heartbeat_pause_event.wait()  # Block here if paused

            if not self.running:
                break

            # Skip heartbeat if WiFi loss pattern is active (it uses both LEDs)
            if self.wifi_loss_led_active:
                time.sleep(0.1)  # Short sleep to avoid busy waiting
                continue

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

                # Use the current heartbeat frequency (set by _update_led_heartbeat_for_pause_state)
                # This allows for pause state to control the frequency

                heartbeat_interval = 1.0 / self.heartbeat_frequency_hz
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
        """SOS LED pattern for low battery warning - Red LED blinks SOS in Morse code"""
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
                    
                    # Set red LED state for SOS warning
                    self.set_red_led(led_state)
                    
                    # Sleep in small increments for responsive shutdown
                    sleep_time = 0
                    while sleep_time < duration and self.running and self.sos_led_active and self.low_battery_detected and not self.critical_battery_detected:
                        time.sleep(0.01)  # Sleep in 10ms increments
                        sleep_time += 0.01

            except Exception as e:
                logger.error(f"Error in SOS LED pattern: {e}")
                break

        # Turn off red LED when done
        self.set_red_led(False)
        self.sos_led_active = False
        logger.info("SOS LED pattern completed")

    def wifi_loss_led_pattern(self):
        """Alternating red/green LED pattern for WiFi loss indication"""
        logger.info("Starting alternating red/green LED pattern for WiFi loss")
        self.wifi_loss_led_active = True

        # Calculate timing for alternating pattern (0.5Hz = 2 second cycle)
        # Each LED is on for 1 second, then off for 1 second, alternating
        cycle_duration = 1.0 / WIFI_LOSS_LED_FREQUENCY_HZ  # 2 seconds total cycle
        led_duration = cycle_duration / 2.0  # 1 second per LED

        while self.running and self.wifi_loss_led_active and self.wifi_loss_detected and not self.critical_battery_detected:
            try:
                # Red LED on, Green LED off
                self.set_red_led(True)
                self.set_green_led(False)
                
                # Sleep in small increments for responsive shutdown
                sleep_time = 0
                while sleep_time < led_duration and self.running and self.wifi_loss_led_active and self.wifi_loss_detected and not self.critical_battery_detected:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01

                if not self.running or not self.wifi_loss_led_active or not self.wifi_loss_detected or self.critical_battery_detected:
                    break

                # Green LED on, Red LED off
                self.set_green_led(True)
                self.set_red_led(False)
                
                # Sleep in small increments for responsive shutdown
                sleep_time = 0
                while sleep_time < led_duration and self.running and self.wifi_loss_led_active and self.wifi_loss_detected and not self.critical_battery_detected:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01

            except Exception as e:
                logger.error(f"Error in WiFi loss LED pattern: {e}")
                break

        # Turn off LEDs when done
        self.set_red_led(False)
        self.set_green_led(False)
        self.wifi_loss_led_active = False
        logger.info("WiFi loss LED pattern completed")

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

    def immediate_action_feedback_pattern(self, action_type):
        """Immediate LED feedback pattern for button actions
        
        Args:
            action_type: Type of action ('single', 'double', 'quadruple')
        """
        logger.debug(f"Starting immediate feedback pattern for {action_type} tap")
        
        # Define patterns for each action type
        patterns = {
            'single': [  # Controller pause toggle - 1 quick flash
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
            ],
            'double': [  # Recording toggle - 2 quick flashes
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
            ],
            'quadruple': [  # Service restart - 4 quick flashes
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
            ]
        }
        
        if action_type not in patterns:
            logger.warning(f"Unknown action type: {action_type}")
            return
            
        pattern = patterns[action_type]
        
        # Execute the pattern
        for duration, led_state in pattern:
            if not self.running:
                break
            self.set_green_led(led_state)
            time.sleep(duration)
        
        # Ensure LED is off at the end
        self.set_green_led(False)
        logger.debug(f"Immediate feedback pattern for {action_type} tap completed")

    def service_wait_pattern(self):
        """R+G alternating flash pattern during service calls"""
        logger.debug("Starting service wait pattern (R+G alternating)")
        
        # Calculate timing for alternating pattern
        period = 1.0 / LED_SERVICE_WAIT_FREQUENCY_HZ
        half_period = period / 2.0
        
        while self.running and hasattr(self, 'service_wait_active') and self.service_wait_active:
            try:
                # Red LED on, Green LED off
                self.set_red_led(True)
                self.set_green_led(False)
                time.sleep(half_period)
                
                if not self.running or not self.service_wait_active:
                    break
                
                # Green LED on, Red LED off
                self.set_red_led(False)
                self.set_green_led(True)
                time.sleep(half_period)
                
            except Exception as e:
                logger.error(f"Error in service wait pattern: {e}")
                break
        
        # Turn off both LEDs when done
        self.set_red_led(False)
        self.set_green_led(False)
        logger.debug("Service wait pattern completed")

    def gradual_frequency_pattern(self):
        """Gradual frequency increase LED pattern during button press - 2Hz to 20Hz over threshold time"""
        start_time = time.time()
        logger.debug(
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
        logger.info("Monitoring for rising edge events (POW_BUT going high - active-high button)")
        logger.info("Button detection is always active (button powers on system via SET coil)")

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

                        # We're configured for rising edge (active-high button press)
                        if event.type == gpiod.LineEvent.RISING_EDGE:
                            self.button_press_start_time = time.time()
                            self.power_button_pressed = True
                            # Reset warning flag for new button press
                            self.warning_notification_sent = False
                            logger.debug(
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
                            logger.warning(
                                f"Unexpected event type: {event.type}")

                    except Exception as e:
                        logger.error(f"Error reading GPIO event: {e}")

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

                # Check for button release (active-high button: released = low)
                if current_state == BUTTON_RELEASED_STATE:  # Button released
                    press_duration = time.time() - self.button_press_start_time
                    logger.debug(
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
            return BUTTON_RELEASED_STATE  # Assume released in test mode
        try:
            # The line is configured for events, but we can still read the current value
            # This is needed to detect button release since we only get falling edge interrupts
            return self.power_button_line.get_value()
        except Exception as e:
            logger.error(f"Error reading current button state: {e}")
            return BUTTON_RELEASED_STATE  # Assume released on error

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
            logger.debug("Long press detected - clearing tap history")
            return

        # Add this tap to the history
        self.tap_times.append(current_time)
        logger.debug(
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
            logger.debug(
                f"Cleaned up {old_count - len(self.tap_times)} old taps")

        # Process tap sequence based on count
        tap_count = len(self.tap_times)

        if tap_count < 1:
            # No taps - ignore
            logger.debug(f"No taps detected - ignoring")
            self.tap_times.clear()
            return

        if tap_count > 5:
            # Too many taps - ignore
            logger.warning(f"Too many taps ({tap_count}) - ignoring")
            self.tap_times.clear()
            return

        # Check if all taps are within the time window
        if tap_count >= 1:
            oldest_tap = min(self.tap_times)
            newest_tap = max(self.tap_times)
            duration = newest_tap - oldest_tap

            # For single tap, duration is 0 - always valid
            if tap_count == 1 or duration <= MULTI_TAP_MAX_DURATION_S + 0.01:
                # Valid tap sequence detected
                if tap_count == 1:
                    logger.info("Single tap detected!")
                    self.tap_times.clear()
                    # Publish button event
                    self._publish_button_event("single_tap")
                    # Immediate feedback for single tap
                    threading.Thread(
                        target=self.immediate_action_feedback_pattern,
                        args=('single',),
                        daemon=True
                    ).start()
                    # Immediate desktop notification
                    self.send_desktop_notification(
                        "Controller Pause", "Single tap detected - toggling controller pause...", "normal")
                    self.toggle_controller_pause()
                elif tap_count == 2:
                    logger.info(
                        f"Double tap detected! ({duration:.2f}s duration)")
                    self.tap_times.clear()
                    # Publish button event
                    self._publish_button_event("double_tap")
                    # Immediate feedback for double tap
                    threading.Thread(
                        target=self.immediate_action_feedback_pattern,
                        args=('double',),
                        daemon=True
                    ).start()
                    # Immediate desktop notification
                    self.send_desktop_notification(
                        "Recording Control", "Double tap detected - toggling recording...", "normal")
                    self.toggle_recording()
                elif tap_count == 4:
                    logger.info(
                        f"Quadruple tap detected! ({duration:.2f}s duration)")
                    self.tap_times.clear()
                    # Publish button event
                    self._publish_button_event("quadruple_tap")
                    # Immediate feedback for quadruple tap
                    threading.Thread(
                        target=self.immediate_action_feedback_pattern,
                        args=('quadruple',),
                        daemon=True
                    ).start()
                    # Immediate desktop notification
                    self.send_desktop_notification(
                        "Argo Service Control", "Quadruple tap detected - restarting Argo service...", "normal")
                    self.restart_argo_service()
                elif tap_count == 3 or tap_count == 5:
                    logger.info(
                        f"{tap_count} taps detected - no action assigned")
                    self.tap_times.clear()
            else:
                logger.warning(
                    f"Tap sequence too slow: {duration:.3f}s > {MULTI_TAP_MAX_DURATION_S}s")
                self.tap_times.clear()

        # Clear the timeout timer reference
        self.tap_timeout_timer = None

    def toggle_recording(self):
        """Toggle rosbag recording on/off with immediate feedback"""
        # --- Real mode ---
        # Force synchronization of recording state before toggling
        logger.info("Double tap detected - forcing recording state synchronization...")
        is_currently_recording = self.force_recording_state_sync()

        # Check if services are ready after state query
        if not self.check_argo_service_status() or not self.recording_service_available:
            logger.warning(
                "Double tap detected but Argo/recording service not ready.")
            self.send_desktop_notification(
                "Recording Unavailable", "Argo service is not ready for recording.", "critical")
            return

        if is_currently_recording:
            logger.info("Double tap detected - requesting to stop recording")
            self.send_desktop_notification(
                "Recording Control", "Requesting to stop recording...", "normal")

            # Start service wait pattern for delayed service call
            self.service_wait_active = True
            service_wait_thread = threading.Thread(
                target=self.service_wait_pattern,
                daemon=True
            )
            service_wait_thread.start()

            success = self.stop_recording()
            
            # Stop service wait pattern
            self.service_wait_active = False
            
            if not success:
                self.send_desktop_notification(
                    "Recording Error", "Failed to stop recording", "critical")
        else:
            logger.info("Double tap detected - requesting to start recording")
            self.send_desktop_notification(
                "Recording Control", "Requesting to start recording...", "normal")

            # Start service wait pattern for delayed service call
            self.service_wait_active = True
            service_wait_thread = threading.Thread(
                target=self.service_wait_pattern,
                daemon=True
            )
            service_wait_thread.start()

            success = self.start_recording()
            
            # Stop service wait pattern
            self.service_wait_active = False
            
            if not success:
                self.send_desktop_notification(
                    "Recording Error", "Failed to start recording", "critical")

    def send_desktop_notification(self, title, message, urgency="normal", expire_time_ms=None):
        """Send desktop notification using notify-send with proper environment for systemd"""
        try:
            # Check if desktop user detection failed previously
            if self.desktop_user_detection_failed:
                logger.debug(
                    "Desktop user detection previously failed - skipping notification")
                return

            # Modify title and message for test mode
            test_title = f"[TEST] {title}" if self.test_mode else title
            test_message = f"{message}\n\n(This is a test notification)" if self.test_mode else message
            if self.test_mode:
                logger.info(
                    f"TEST MODE: Sending notification - {title}: {message}")

            # Use custom expire time or default
            expire_time = expire_time_ms if expire_time_ms is not None else NOTIFICATION_EXPIRE_TIME_MS

            # For systemd services, we need to find the user's graphical session environment
            if os.geteuid() == 0:
                if not self.cached_desktop_user:
                    logger.debug("No cached desktop user - skipping notification")
                    return

                # Find the main process of the user's graphical session to get its environment
                session_pid = None
                try:
                    # Try each desktop environment separately (fixed pgrep pattern)
                    for process_name in ['gnome-shell', 'plasma-shell', 'xfce4-session', 
                                         'lxsession', 'i3', 'sway']:
                        pgrep_cmd = ['pgrep', '-u', self.cached_desktop_user, '-o', process_name]
                        result = subprocess.run(
                            pgrep_cmd, capture_output=True, text=True, timeout=2)
                        
                        if result.returncode == 0:
                            session_pid = result.stdout.strip()
                            logger.debug(f"Found {process_name} session with PID: {session_pid}")
                            break
                    
                    if session_pid:
                        # Get environment variables from the session process
                        # /proc/pid/environ uses null bytes as separators, not newlines
                        with open(f'/proc/{session_pid}/environ', 'r') as f:
                            environ_data = f.read()
                            env_vars = dict(
                                line.split('=', 1) 
                                for line in environ_data.split('\0') 
                                if '=' in line
                            )
                        
                        display = env_vars.get('DISPLAY')
                        dbus_address = env_vars.get('DBUS_SESSION_BUS_ADDRESS')

                        if display and dbus_address:
                            logger.debug(
                                f"Found graphical session for user {self.cached_desktop_user} (PID: {session_pid})")
                            logger.debug(
                                f"  DISPLAY={display}, DBUS_SESSION_BUS_ADDRESS={dbus_address}")

                            # Use 'env' command to properly set environment variables for sudo
                            cmd = [
                                'sudo', '-u', self.cached_desktop_user,
                                'env',
                                f'DISPLAY={display}',
                                f'DBUS_SESSION_BUS_ADDRESS={dbus_address}',
                                'notify-send', '--urgency', urgency,
                                '--expire-time', str(expire_time),
                                test_title, test_message
                            ]
                            subprocess.run(
                                cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                            logger.info(f"Desktop notification sent: {title}")
                            return
                        else:
                            logger.warning(
                                f"Could not find DISPLAY/DBUS for user {self.cached_desktop_user}")

                except (subprocess.TimeoutExpired, FileNotFoundError, PermissionError) as e:
                    logger.warning(
                        f"Failed to get graphical session environment: {e}")

                # Fallback to simple notification method with standard environment
                logger.debug("Falling back to standard environment notification method")
                cmd = [
                    'sudo', '-u', self.cached_desktop_user,
                    'env',
                    'DISPLAY=:0',
                    'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus',
                    'notify-send', '--urgency', urgency,
                    '--expire-time', str(expire_time),
                    test_title, test_message
                ]
                subprocess.run(
                    cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                logger.info(f"Desktop notification sent (fallback): {title}")
                return

            else:
                # Running as a regular user, direct call is fine
                cmd = [
                    'notify-send', '--urgency', urgency,
                    '--expire-time', str(expire_time),
                    test_title, test_message
                ]
                subprocess.run(
                    cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                logger.info(f"Desktop notification sent (user): {title}")

        except subprocess.TimeoutExpired:
            logger.warning("Desktop notification timed out")
        except subprocess.CalledProcessError as e:
            logger.warning(f"Failed to send desktop notification: {e}")
            self.desktop_user_detection_failed = True
        except Exception as e:
            logger.warning(
                f"Unexpected error sending desktop notification: {e}")
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
        
        # Publish button event
        self._publish_button_event("long_press_shutdown")

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
            # GPIO pins will automatically revert to input state on halt

    def _create_power_services(self):
        """Create ROS2 services for power control"""
        if not self.ros2_node or self.power_services_created:
            return
        
        try:
            # Create power control services
            from std_srvs.srv import Trigger
            self.start_recording_service = self.ros2_node.create_service(
                Trigger, '/argo/power/start_recording', self._handle_start_recording_service)
            self.stop_recording_service = self.ros2_node.create_service(
                Trigger, '/argo/power/stop_recording', self._handle_stop_recording_service)
            self.toggle_recording_service = self.ros2_node.create_service(
                Trigger, '/argo/power/toggle_recording', self._handle_toggle_recording_service)
            self.shutdown_service = self.ros2_node.create_service(
                Trigger, '/argo/power/shutdown', self._handle_shutdown_service)
            self.toggle_argo_service = self.ros2_node.create_service(
                Trigger, '/argo/power/toggle_argo', self._handle_toggle_argo_service)
            
            # Create publishers
            self.power_status_publisher = self.ros2_node.create_publisher(
                String, '/argo/power/status', 10)
            self.button_event_publisher = self.ros2_node.create_publisher(
                String, '/argo/power/button_events', 10)
            
            # Create subscription to controller pause state
            from std_msgs.msg import Bool
            self.controller_pause_sub = self.ros2_node.create_subscription(
                Bool, '/controller_pause_state', self._controller_pause_state_callback, 10)
            
            self.power_services_created = True
            logger.info("Power control ROS2 services created:")
            logger.info("  - /argo/power/start_recording")
            logger.info("  - /argo/power/stop_recording")
            logger.info("  - /argo/power/toggle_recording")
            logger.info("  - /argo/power/shutdown")
            logger.info("  - /argo/power/toggle_argo")
            logger.info("  - /argo/power/status (topic)")
            logger.info("  - /argo/power/button_events (topic)")
            logger.info("  - /controller_pause_state (subscription)")
        except Exception as e:
            logger.error(f"Failed to create power services: {e}")
    
    def _handle_start_recording_service(self, request, response):
        """Handle /argo/power/start_recording service request"""
        try:
            self.start_recording()
            response.success = True
            response.message = "Recording started"
        except Exception as e:
            response.success = False
            response.message = f"Error starting recording: {e}"
        return response
    
    def _handle_stop_recording_service(self, request, response):
        """Handle /argo/power/stop_recording service request"""
        try:
            self.stop_recording()
            response.success = True
            response.message = "Recording stopped"
        except Exception as e:
            response.success = False
            response.message = f"Error stopping recording: {e}"
        return response
    
    def _handle_toggle_recording_service(self, request, response):
        """Handle /argo/power/toggle_recording service request"""
        try:
            self.toggle_recording()
            response.success = True
            response.message = "Recording toggled"
        except Exception as e:
            response.success = False
            response.message = f"Error toggling recording: {e}"
        return response
    
    def _handle_shutdown_service(self, request, response):
        """Handle /argo/power/shutdown service request"""
        try:
            # Initiate shutdown in background thread to allow response
            import threading
            threading.Thread(target=self.initiate_shutdown, daemon=True).start()
            response.success = True
            response.message = "Shutdown initiated"
        except Exception as e:
            response.success = False
            response.message = f"Error initiating shutdown: {e}"
        return response
    
    def _handle_toggle_argo_service(self, request, response):
        """Handle /argo/power/toggle_argo service request"""
        try:
            self.toggle_argo_service()
            response.success = True
            response.message = "Argo service toggled"
        except Exception as e:
            response.success = False
            response.message = f"Error toggling Argo service: {e}"
        return response
    
    def _publish_power_status(self, message: str):
        """Publish power status update"""
        if self.power_status_publisher:
            try:
                msg = String()
                msg.data = message
                self.power_status_publisher.publish(msg)
            except Exception:
                pass  # Silently fail
    
    def _publish_button_event(self, event: str):
        """Publish button event"""
        if self.button_event_publisher:
            try:
                msg = String()
                msg.data = event
                self.button_event_publisher.publish(msg)
                logger.info(f"Published button event: {event}")
            except Exception as e:
                logger.debug(f"Failed to publish button event: {e}")
    
    def _controller_pause_state_callback(self, msg):
        """Receive controller pause state updates from topic"""
        old_state = self.controller_pause_state
        self.controller_pause_state = msg.data
        
        # Log state change
        if old_state != msg.data:
            logger.info(f"Controller pause state changed: {'PAUSED' if msg.data else 'UNPAUSED'}")
            # Update LED heartbeat frequency when pause state changes
            self._update_led_heartbeat_for_pause_state()
        else:
            logger.debug(f"Controller pause state: {'PAUSED' if msg.data else 'UNPAUSED'}")
    
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
        
        # Set initial LED heartbeat frequency based on current state
        self._update_led_heartbeat_for_pause_state()

        # Start critical battery monitoring in separate thread
        logger.info("Starting critical battery monitoring thread")
        battery_thread = threading.Thread(
            target=self.monitor_critical_battery, daemon=True)
        battery_thread.start()

        # Start WiFi connectivity monitoring in separate thread
        logger.info("Starting WiFi connectivity monitoring thread")
        wifi_thread = threading.Thread(
            target=self.monitor_wifi_connectivity, daemon=True)
        wifi_thread.start()

        # Create ROS2 services for remote control
        self._create_power_services()
        
        # Publish initial status
        self._publish_power_status("Power control system running")

        try:
            # Main loop - ROS2 spin with monitoring
            last_sync_time = 0
            sync_interval = 60  # Sync every 60 seconds (more frequent for better sync)
            last_recording_sync_time = 0
            recording_sync_interval = 30  # Recording state sync every 30 seconds
            last_status_publish_time = 0
            status_publish_interval = 10  # Publish status every 10 seconds

            while self.running and (rclpy.ok() if self.ros2_node else True):
                current_time = time.time()
                
                # Spin ROS2 node to process service requests (if available)
                if self.ros2_node:
                    rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
                else:
                    time.sleep(0.1)

                # Periodic state synchronization
                if current_time - last_sync_time >= sync_interval:
                    if self.argo_service_running and self.recording_service_available:
                        logger.debug(
                            "Performing periodic recording state synchronization...")
                        self.query_current_recording_status()
                    last_sync_time = current_time

                # More frequent recording state synchronization
                if (current_time - last_recording_sync_time >= recording_sync_interval and 
                    self.argo_service_running):
                    # Always try to sync recording state, even if service availability is unknown
                    logger.debug("Performing frequent recording state synchronization...")
                    self.query_current_recording_status()
                    last_recording_sync_time = current_time
                
                # Periodic status publishing
                if current_time - last_status_publish_time >= status_publish_interval:
                    status_msg = f"Running | Argo: {'ON' if self.argo_service_running else 'OFF'} | Recording: {'ON' if self.recording_active else 'OFF'}"
                    self._publish_power_status(status_msg)
                    last_status_publish_time = current_time

        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
            self.running = False

        finally:
            self._publish_power_status("Power control system stopping")
            self.cleanup()

    def monitor_critical_battery(self):
        """Monitor battery voltage every 30 seconds for critical low voltage"""
        logger.info("Starting critical battery monitoring thread")
        self.battery_monitoring_active = True
        
        # Track consecutive failures for safety
        consecutive_service_failures = 0
        consecutive_invalid_readings = 0
        MAX_CONSECUTIVE_FAILURES = 3  # 90 seconds of failures = assume critical
        
        # Startup grace period - don't count failures until battery service has been seen at least once
        # This prevents false critical alerts if argo_battery_water.service starts after power_control.service
        battery_service_ever_available = False
        startup_time = time.time()
        STARTUP_GRACE_PERIOD_S = 60.0  # 60 seconds to wait for argo_battery_water.service startup

        while self.running and self.battery_monitoring_active:
            try:
                # Always attempt to check battery, even if argo-launch is stopped
                # This is CRITICAL for safety - battery monitoring must never stop

                # Call battery service to get voltage
                battery_data = self._call_battery_service()
                if battery_data:
                    # Service call succeeded
                    consecutive_service_failures = 0
                    
                    # Mark that we've seen the battery service available at least once
                    if not battery_service_ever_available:
                        battery_service_ever_available = True
                        logger.info("Battery service is now available - critical battery monitoring active")
                    
                    battery_voltage = battery_data.get('battery_voltage', 0)
                    charging_status = battery_data.get('charging_status', None)
                    ac_power_present = battery_data.get('ac_power_present', None)
                    
                    # CRITICAL SAFETY CHECK: Validate battery voltage is reasonable
                    # Invalid readings (0V, very low, or impossibly high) indicate sensor/communication errors
                    # NEVER halt on invalid readings - only on valid low voltage readings
                    if battery_voltage <= 0 or battery_voltage < 3.0 or battery_voltage > 30.0:
                        consecutive_invalid_readings += 1
                        logger.error(
                            f"Invalid battery voltage reading: {battery_voltage:.3f}V "
                            f"(count: {consecutive_invalid_readings}/{MAX_CONSECUTIVE_FAILURES}) - likely sensor/I2C error")
                        logger.error(
                            f"⚠️  System will NOT halt on invalid readings - only on valid low voltage!")
                        
                        # Do NOT halt on invalid readings - they indicate hardware/communication problems, not battery issues
                        # Just log the error and continue monitoring
                        if consecutive_invalid_readings >= MAX_CONSECUTIVE_FAILURES:
                            logger.critical(
                                f"CRITICAL: {consecutive_invalid_readings} consecutive invalid battery readings!")
                            logger.critical(
                                f"This indicates a sensor/communication problem, NOT a battery problem!")
                            logger.critical(
                                f"System will continue running - check argo_battery_water.service status")
                        continue
                    
                    # Valid reading - reset invalid counter
                    consecutive_invalid_readings = 0
                    
                    # Check if voltage has changed significantly since last log
                    should_log_voltage = True
                    if self.last_logged_battery_voltage is not None:
                        voltage_change = abs(battery_voltage - self.last_logged_battery_voltage)
                        should_log_voltage = voltage_change >= BATTERY_LOG_THRESHOLD_V
                    
                    # Log battery status with charging information (only if voltage changed significantly)
                    if should_log_voltage:
                        charging_str = ""
                        if charging_status is not None or ac_power_present is not None:
                            charging_parts = []
                            if ac_power_present is not None:
                                charging_parts.append(f"AC Power: {'YES' if ac_power_present else 'NO'}")
                            if charging_status is not None:
                                charging_parts.append(f"Charging: {'ACTIVE' if charging_status else 'INACTIVE'}")
                            charging_str = f", {', '.join(charging_parts)}"
                        
                        logger.info(
                            f"Battery voltage check: {battery_voltage:.3f}V "
                            f"(low: {LOW_BATTERY_THRESHOLD_V}V, critical: {CRITICAL_BATTERY_THRESHOLD_V}V){charging_str}")
                        
                        # Update the last logged voltage
                        self.last_logged_battery_voltage = battery_voltage

                    # Check for critical battery first (highest priority)
                    if battery_voltage < CRITICAL_BATTERY_THRESHOLD_V:
                        if not self.critical_battery_detected:
                            # Always log critical battery events regardless of voltage change threshold
                            charging_str = ""
                            if charging_status is not None or ac_power_present is not None:
                                charging_parts = []
                                if ac_power_present is not None:
                                    charging_parts.append(f"AC Power: {'YES' if ac_power_present else 'NO'}")
                                if charging_status is not None:
                                    charging_parts.append(f"Charging: {'ACTIVE' if charging_status else 'INACTIVE'}")
                                charging_str = f", {', '.join(charging_parts)}"
                            
                            logger.critical(
                                f"CRITICAL BATTERY DETECTED: {battery_voltage:.3f}V < {CRITICAL_BATTERY_THRESHOLD_V}V{charging_str}")
                            self.critical_battery_detected = True
                            # Pause heartbeat for critical battery
                            self.pause_heartbeat()
                            # Stop SOS pattern if running (critical takes priority)
                            if self.sos_led_active:
                                self.sos_led_active = False
                            self.initiate_critical_battery_halt(battery_voltage)
                    
                    # Check for low battery (SOS warning)
                    elif battery_voltage < LOW_BATTERY_THRESHOLD_V:
                        if not self.low_battery_detected:
                            # Always log low battery events regardless of voltage change threshold
                            logger.warning(
                                f"LOW BATTERY DETECTED: {battery_voltage:.3f}V < {LOW_BATTERY_THRESHOLD_V}V - Starting SOS LED pattern")
                            self.low_battery_detected = True
                            # Pause heartbeat to make SOS pattern visible
                            self.pause_heartbeat()
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
                            # Always log battery recovery events regardless of voltage change threshold
                            logger.info(
                                f"Battery voltage recovered from low: {battery_voltage:.3f}V >= {LOW_BATTERY_THRESHOLD_V}V")
                            self.low_battery_detected = False
                            self.sos_led_active = False  # Stop SOS pattern
                            # Resume heartbeat now that battery is stable
                            self.resume_heartbeat()
                            self.send_desktop_notification(
                                "Battery Recovered",
                                f"Battery voltage recovered: {battery_voltage:.3f}V\nSOS LED pattern stopped",
                                "normal"
                            )
                        
                        # Check if we were in critical battery state
                        if self.critical_battery_detected:
                            # Always log critical battery recovery events regardless of voltage change threshold
                            logger.info(
                                f"Battery voltage recovered from critical: {battery_voltage:.3f}V >= {CRITICAL_BATTERY_THRESHOLD_V}V")
                            self.critical_battery_detected = False
                            self._clear_critical_battery_flag()
                    
                    # Log debug message when voltage change is too small to log normally
                    if not should_log_voltage and self.last_logged_battery_voltage is not None:
                        voltage_change = abs(battery_voltage - self.last_logged_battery_voltage)
                        logger.debug(
                            f"Battery voltage change too small to log: {voltage_change*1000:.1f}mV < {BATTERY_LOG_THRESHOLD_V*1000:.0f}mV "
                            f"(current: {battery_voltage:.3f}V, last logged: {self.last_logged_battery_voltage:.3f}V)")
                else:
                    # Service unavailable - handle based on startup state
                    time_since_startup = time.time() - startup_time
                    
                    if not battery_service_ever_available and time_since_startup < STARTUP_GRACE_PERIOD_S:
                        # During startup grace period and service never seen - don't count failures yet
                        logger.info(
                            f"Battery service not available yet - waiting for argo_battery_water.service startup "
                            f"(grace period: {time_since_startup:.0f}s / {STARTUP_GRACE_PERIOD_S:.0f}s)")
                    elif not battery_service_ever_available and time_since_startup >= STARTUP_GRACE_PERIOD_S:
                        # Grace period expired but service never became available - log error but DO NOT HALT
                        logger.critical(
                            f"CRITICAL: Battery service never became available after {STARTUP_GRACE_PERIOD_S}s grace period!")
                        logger.critical(
                            "Battery monitoring is DISABLED - system will continue WITHOUT battery protection!")
                        logger.critical(
                            "Check if argo_battery_water.service is installed and enabled:")
                        logger.critical("  sudo systemctl status argo_battery_water.service")
                        logger.critical(
                            "System will NOT halt - service unavailability is not a battery problem!")
                        # DO NOT HALT - missing service is a configuration problem, not a battery emergency
                    else:
                        # Service was available before but now failing - count consecutive failures
                        consecutive_service_failures += 1
                        logger.error(
                            f"Battery service failure - service was available but now unreachable "
                            f"(count: {consecutive_service_failures}/{MAX_CONSECUTIVE_FAILURES})")
                        
                        # After multiple consecutive service failures, log critical error but DO NOT HALT
                        if consecutive_service_failures >= MAX_CONSECUTIVE_FAILURES:
                            logger.critical(
                                f"CRITICAL: Battery service failed {consecutive_service_failures} times consecutively!")
                            logger.critical(
                                "Battery monitoring is NOT working - system continues WITHOUT battery protection!")
                            logger.critical(
                                "Service failures indicate a monitoring problem, NOT a battery emergency!")
                            logger.critical(
                                "Check argo_battery_water.service: sudo systemctl status argo_battery_water.service")
                            # DO NOT HALT - service failures are monitoring problems, not battery emergencies

                self.last_battery_check_time = time.time()

            except Exception as e:
                logger.error(f"Error in critical battery monitoring: {e}")

            # Sleep for monitoring interval
            sleep_time = 0
            while sleep_time < BATTERY_MONITORING_INTERVAL_S and self.running and self.battery_monitoring_active:
                time.sleep(1.0)
                sleep_time += 1.0

        logger.info("Critical battery monitoring thread stopped")

    def monitor_wifi_connectivity(self):
        """Monitor WiFi connectivity every 10 seconds and update LED patterns"""
        logger.info("Starting WiFi connectivity monitoring thread")
        self.wifi_monitoring_active = True
        
        # Track consecutive failures for stability
        consecutive_failures = 0
        consecutive_successes = 0
        MAX_CONSECUTIVE_FAILURES = 2  # Require 2 consecutive failures before declaring WiFi lost
        MAX_CONSECUTIVE_SUCCESSES = 2  # Require 2 consecutive successes before declaring WiFi restored
        
        while self.running and self.wifi_monitoring_active:
            try:
                # Check WiFi connectivity
                wifi_connected = self.check_wifi_connectivity()
                
                if wifi_connected:
                    consecutive_successes += 1
                    consecutive_failures = 0
                    
                    # Check if WiFi was previously lost and now restored
                    if self.wifi_loss_detected and consecutive_successes >= MAX_CONSECUTIVE_SUCCESSES:
                        logger.info("WiFi connectivity restored - stopping WiFi loss LED pattern")
                        self.wifi_loss_detected = False
                        self.wifi_loss_led_active = False  # Stop the alternating pattern
                        # Resume normal heartbeat
                        self.resume_heartbeat()
                        # Send notification
                        self.send_desktop_notification(
                            "WiFi Restored",
                            "WiFi connectivity has been restored",
                            "normal"
                        )
                        consecutive_successes = 0  # Reset counter
                        
                else:
                    consecutive_failures += 1
                    consecutive_successes = 0
                    
                    # Check if WiFi was previously connected and now lost
                    if not self.wifi_loss_detected and consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                        logger.warning("WiFi connectivity lost - starting alternating red/green LED pattern")
                        self.wifi_loss_detected = True
                        # Pause heartbeat to make WiFi loss pattern visible
                        self.pause_heartbeat()
                        # Start alternating red/green LED pattern in separate thread
                        threading.Thread(target=self.wifi_loss_led_pattern, daemon=True).start()
                        # Send notification
                        self.send_desktop_notification(
                            "WiFi Connection Lost",
                            "WiFi connectivity has been lost\nAlternating red/green LED pattern activated",
                            "critical"
                        )
                        consecutive_failures = 0  # Reset counter

                self.last_wifi_check_time = time.time()

            except Exception as e:
                logger.error(f"Error in WiFi connectivity monitoring: {e}")

            # Sleep for monitoring interval
            sleep_time = 0
            while sleep_time < WIFI_MONITORING_INTERVAL_S and self.running and self.wifi_monitoring_active:
                time.sleep(1.0)
                sleep_time += 1.0

        logger.info("WiFi connectivity monitoring thread stopped")

    def _call_battery_service(self) -> Optional[Dict[str, Any]]:
        """Call battery_water service to get current battery data using rclpy"""
        success, message = self._call_trigger_service(
            '/battery_status', timeout_sec=5.0)

        if success:
            try:
                response_data = json.loads(message)
                # Extract raw_data from the response, which is what the caller expects
                raw_data = response_data.get('raw_data', {})
                logger.debug(
                    f"Successfully parsed battery data: {raw_data.get('battery_voltage', 'N/A')}V")
                return raw_data
            except (json.JSONDecodeError, KeyError) as e:
                logger.error(f"Error parsing battery service response: {e}")
                return None
        else:
            logger.error(f"Battery service call failed: {message}")
            return None

    def check_wifi_connectivity(self) -> bool:
        """Check WiFi connectivity using multiple methods
        
        Returns:
            True if WiFi is connected and has internet access
            False if WiFi is disconnected or no internet access
        """
        try:
            # Method 1: Check if WiFi interface is up and has an IP address
            result = subprocess.run(
                ['ip', 'addr', 'show', 'wlan0'], 
                capture_output=True, text=True, timeout=WIFI_CONNECTIVITY_TIMEOUT_S
            )
            
            if result.returncode != 0:
                logger.debug("WiFi interface wlan0 not found or not accessible")
                return False
            
            # Check if wlan0 has an IP address (not 127.0.0.1)
            if 'inet ' in result.stdout and '127.0.0.1' not in result.stdout:
                # Method 2: Test internet connectivity with a simple ping
                ping_result = subprocess.run(
                    ['ping', '-c', '1', '-W', '3', '8.8.8.8'], 
                    capture_output=True, text=True, timeout=WIFI_CONNECTIVITY_TIMEOUT_S
                )
                
                if ping_result.returncode == 0:
                    logger.debug("WiFi connectivity confirmed: interface up and internet accessible")
                    return True
                else:
                    logger.debug("WiFi interface up but no internet connectivity")
                    return False
            else:
                logger.debug("WiFi interface wlan0 has no valid IP address")
                return False
                
        except subprocess.TimeoutExpired:
            logger.debug("WiFi connectivity check timed out")
            return False
        except FileNotFoundError:
            logger.debug("Network tools not available for WiFi check")
            return False
        except Exception as e:
            logger.debug(f"Error checking WiFi connectivity: {e}")
            return False

    def show_critical_battery_confirmation_dialog(self, battery_voltage):
        """Show desktop confirmation dialog for critical battery shutdown
        
        Returns:
            True: User actively CANCELLED the halt (clicked Cancel button)
            False: Proceed with halt (user clicked OK, or timeout occurred, or dialog failed)
        
        CRITICAL: Timeout or dialog failure = return False (proceed with halt for safety)
        """
        try:
            # Set up environment for desktop dialogs
            env = os.environ.copy()
            if self.cached_display_env:
                env['DISPLAY'] = self.cached_display_env
            else:
                # Fallback if display detection failed
                env['DISPLAY'] = ':0'

            # Try to show a zenity dialog first (most reliable on desktop)
            dialog_cmd = [
                'zenity',
                '--question',
                '--title=CRITICAL BATTERY SHUTDOWN',
                f'--text=Battery voltage critically low: {battery_voltage:.3f}V\n\nSystem will halt in 30 seconds to preserve power for manual sailing.\n\n⚠️ TIMEOUT = AUTOMATIC HALT (safe default)\n\nClick "Cancel" ONLY if you can plug in charger NOW.\nClick "Shutdown Now" to halt immediately.\nNo action = automatic halt after 30 seconds.',
                '--ok-label=Shutdown Now',
                '--cancel-label=Cancel (I will plug in charger)',
                '--timeout=30',
                '--width=600',
                '--height=350'
            ]
            
            logger.info("Showing critical battery confirmation dialog...")
            result = subprocess.run(
                dialog_cmd, capture_output=True, text=True, timeout=60, env=env)
            
            if result.returncode == 0:
                # User clicked "Shutdown Now" - proceed with halt
                logger.info("User confirmed critical battery shutdown")
                return False  # False = proceed with halt
            elif result.returncode == 1:
                # Check if this was a real cancel or a display error
                if "cannot open display" in result.stderr.lower():
                    logger.error(
                        f"Zenity failed to open display: {result.stderr.strip()}")
                    logger.error(
                        "Proceeding with halt as a safety measure.")
                    return False  # Treat display error as timeout/confirm
                else:
                    # User clicked "Cancel Shutdown" - user intervention
                    logger.warning(
                        "User actively CANCELLED critical battery shutdown")
                    return True  # True = user cancelled, don't halt
            elif result.returncode == 5:
                # Dialog timeout (zenity returns 5 for timeout) - proceed with halt (SAFE DEFAULT)
                logger.info(
                    "Critical battery dialog timed out - proceeding with halt (safe default)")
                return False  # False = proceed with halt
            else:
                # Unexpected return code - default to shutdown for safety
                logger.warning(f"Dialog returned unexpected code: {result.returncode} - defaulting to halt for safety")
                return False  # False = proceed with halt
                
        except subprocess.TimeoutExpired:
            # Subprocess timeout - proceed with halt (safe default)
            logger.info("Critical battery dialog timed out - proceeding with halt (safe default)")
            return False  # False = proceed with halt
        except FileNotFoundError:
            logger.warning("zenity not found - trying kdialog...")
            try:
                # Fallback to kdialog
                dialog_cmd = [
                    'kdialog',
                    '--title=CRITICAL BATTERY SHUTDOWN',
                    f'--yesno=Battery voltage critically low: {battery_voltage:.3f}V\n\nSystem will halt in 30 seconds to preserve power for manual sailing.\n\nTIMEOUT = AUTOMATIC HALT (safe default)\n\nClick "No" ONLY if you can plug in charger NOW.\nClick "Yes" to halt immediately.\nNo action = automatic halt after 30 seconds.',
                    '--yes-label=Shutdown Now',
                    '--no-label=Cancel (I will plug in charger)'
                ]
                
                result = subprocess.run(dialog_cmd, capture_output=True, text=True, timeout=35, env=env)
                
                if result.returncode == 0:
                    # User clicked "Yes" / "Shutdown Now"
                    logger.info("User confirmed critical battery shutdown (kdialog)")
                    return False  # False = proceed with halt
                else:
                    # User clicked "No" / "Cancel" or timeout
                    if result.returncode == 1:
                        logger.warning("User actively CANCELLED critical battery shutdown (kdialog)")
                        return True  # True = user cancelled
                    else:
                        # Check for display error
                        if "cannot open display" in result.stderr.lower():
                            logger.error(
                                f"kdialog failed to open display: {result.stderr.strip()}")
                            logger.error(
                                "Proceeding with halt as a safety measure.")
                            return False # Treat as confirm
                        else:
                            # Timeout or error - default to halt for safety
                            logger.info(
                                "kdialog timeout or error - proceeding with halt (safe default)")
                            return False  # False = proceed with halt
                    
            except (FileNotFoundError, subprocess.TimeoutExpired):
                logger.warning("kdialog also not available or timed out - trying yad...")
                try:
                    # Fallback to yad (Yet Another Dialog)
                    dialog_cmd = [
                        'yad',
                        '--title=CRITICAL BATTERY SHUTDOWN',
                        f'--text=Battery voltage critically low: {battery_voltage:.3f}V\n\nSystem will halt in 30 seconds to preserve power for manual sailing.\n\nTIMEOUT = AUTOMATIC HALT (safe default)\n\nClick "Cancel" ONLY if you can plug in charger NOW.\nNo action = automatic halt after 30 seconds.',
                        '--button=Shutdown Now:0',
                        '--button=Cancel (I will plug in charger):1',
                        '--timeout=30',
                        '--width=600',
                        '--height=350'
                    ]
                    
                    result = subprocess.run(dialog_cmd, capture_output=True, text=True, timeout=35, env=env)
                    if result.returncode == 0:
                        return False  # Proceed with halt
                    elif result.returncode == 1:
                        logger.warning("User actively CANCELLED via yad")
                        return True  # User cancelled
                    else:
                        # Check for display error
                        if "cannot open display" in result.stderr.lower():
                            logger.error(
                                f"yad failed to open display: {result.stderr.strip()}")
                            logger.error(
                                "Proceeding with halt as a safety measure.")
                        return False  # Timeout or error = proceed with halt
                    
                except (FileNotFoundError, subprocess.TimeoutExpired):
                    logger.warning("No dialog tools available - proceeding with halt (safe default)")
                    return False  # False = proceed with halt
        except Exception as e:
            logger.error(f"Error showing critical battery dialog: {e}")
            logger.info("Proceeding with halt due to dialog error (safe default)")
            return False  # False = proceed with halt

    def _pause_argo_system_for_power_conservation(self):
        """Pause all sensor nodes to put hardware in low-power shutdown state"""
        if not self.check_argo_service_status():
            logger.info(
                "Argo service not running - skipping pause for power conservation")
            return False

        logger.info(
            "Pausing Argo system for power conservation via service call...")
        success, message = self._call_trigger_service(
            '/toggle_pause', timeout_sec=10.0)

        if success:
            logger.info(
                f"✅ Argo system pause command successful: {message}")
            return True
        else:
            logger.error(f"❌ Failed to pause Argo system: {message}")
            return False

    def initiate_critical_battery_halt(self, battery_voltage):
        """Initiate critical battery halt sequence with timeout-based confirmation
        
        Shows a confirmation dialog with 30-second timeout:
        - If user clicks "Cancel": Halt is cancelled (allows intervention)
        - If user clicks "Shutdown Now": Immediate halt
        - If timeout (30s, no user action): Automatic halt proceeds (SAFE DEFAULT)
        
        This provides:
        - Autonomous safety: Timeout = halt proceeds (not cancelled)
        - Developer control: Active cancellation stops halt if user intervenes
        """
        # Log critical battery mode based on configuration flag
        if CRITICAL_BATTERY_USE_SHUTDOWN:
            logger.critical(
                f"CRITICAL BATTERY SHUTDOWN: {battery_voltage:.3f}V - System will shutdown and CUT POWER")
            logger.critical(
                "⚠️  DEVELOPMENT MODE: Using normal shutdown (cuts power) instead of halt")
            logger.critical(
                "⚠️  CRITICAL_BATTERY_USE_SHUTDOWN=True - Power will be CUT, not preserved")
            shutdown_mode = "shutdown (power cut)"
        else:
            logger.critical(
                f"CRITICAL BATTERY HALT: {battery_voltage:.3f}V - System will halt to preserve power")
            logger.critical(
                "PRODUCTION MODE: Using halt to preserve power for manual sailing")
            shutdown_mode = "halt (power preserved)"
        
        logger.critical(
            f"Showing confirmation dialog - timeout (no action) will proceed with {shutdown_mode}")

        # Set critical battery flag for shutdown hook ONLY if using halt mode
        # If using shutdown mode, we want the hook to cut power normally
        if not CRITICAL_BATTERY_USE_SHUTDOWN:
            self._set_critical_battery_flag()
            logger.info("Critical battery flag set - shutdown hook will preserve power")
        else:
            logger.info("Critical battery flag NOT set - shutdown hook will cut power normally")

        # Send critical notification with appropriate message
        notification_message = f"Battery voltage critically low: {battery_voltage:.3f}V\nSystem will {shutdown_mode} in 30 seconds unless cancelled\nTimeout = automatic shutdown (safe default)"
        self.send_desktop_notification(
            "CRITICAL BATTERY",
            notification_message,
            "critical",
            30000  # 30 second timeout
        )

        # Broadcast wall message
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if CRITICAL_BATTERY_USE_SHUTDOWN:
                action_desc = "SHUTDOWN and CUT POWER"
                mode_desc = "shutdown to fully power off the system (DEVELOPMENT MODE)"
            else:
                action_desc = "HALT to preserve battery power"
                mode_desc = "halt to preserve battery for manual sailing operation (PRODUCTION MODE)"
            
            message = f"""CRITICAL BATTERY ALERT: {battery_voltage:.3f}V at {timestamp}

⚠️  System will {action_desc} in 30 seconds.

To CANCEL the shutdown from CLI (close the confirmation dialog):
  pkill -f zenity

If you take no action within 30 seconds, the system will automatically
{mode_desc}."""
            subprocess.run(['wall', message], check=True,
                           timeout=WALL_MESSAGE_TIMEOUT_S)
            logger.info("Critical battery wall message broadcasted")
        except Exception as e:
            logger.error(f"Failed to broadcast critical battery message: {e}")

        # Show confirmation dialog with safe defaults
        # CRITICAL: Timeout or "Shutdown Now" = proceed with halt
        # Only "Cancel" button stops the halt
        logger.info("Showing critical battery confirmation dialog (timeout = halt proceeds)...")
        user_cancelled = self.show_critical_battery_confirmation_dialog(battery_voltage)
        
        if user_cancelled:
            # User ACTIVELY cancelled the halt/shutdown
            logger.warning(f"Critical battery {shutdown_mode} CANCELLED by user intervention")
            logger.warning("User has taken responsibility - they must plug in charger or take action!")
            # Clear the critical battery flag since we're not shutting down (if it was set)
            if not CRITICAL_BATTERY_USE_SHUTDOWN:
                self._clear_critical_battery_flag()
            # Resume heartbeat since shutdown was cancelled
            self.resume_heartbeat()
            # Send cancellation notification
            self.send_desktop_notification(
                "SHUTDOWN CANCELLED BY USER",
                f"Critical battery {shutdown_mode} was cancelled by user\nBattery: {battery_voltage:.3f}V - PLUG IN CHARGER NOW!",
                "critical",
                0  # Stays visible
            )
            return  # Exit without shutting down

        # If we reach here, either:
        # 1. User clicked "Shutdown Now" (proceed immediately)
        # 2. Dialog timed out (proceed automatically - SAFE DEFAULT)
        logger.critical(f"Proceeding with critical battery {shutdown_mode}")
        logger.critical("Either user confirmed OR timeout occurred (automatic action for safety)")
        
        # Stop battery monitoring to prevent repeated alerts
        self.battery_monitoring_active = False

        # CRITICAL POWER CONSERVATION: Pause Argo system to put sensors in shutdown state
        logger.critical("Pausing Argo system to conserve battery power...")
        pause_success = self._pause_argo_system_for_power_conservation()
        
        if pause_success:
            logger.info("Sensors paused - waiting 2 seconds for hardware shutdown...")
            time.sleep(2)  # Give sensors time to enter hardware shutdown state
        else:
            logger.warning("Could not pause sensors - proceeding with halt anyway")

        # Brief delay to allow final notifications
        logger.critical("Waiting 3 seconds for final notifications...")
        time.sleep(3)
        
        # Execute command based on configuration flag
        if self.test_mode:
            if CRITICAL_BATTERY_USE_SHUTDOWN:
                logger.info(
                    "TEST MODE: Would execute 'shutdown -h now' command for critical battery")
                logger.info("TEST MODE: Critical battery shutdown sequence completed - system would SHUTDOWN and CUT POWER NOW")
            else:
                logger.info(
                    "TEST MODE: Would execute 'sudo halt' command for critical battery")
                logger.info("TEST MODE: Critical battery halt sequence completed - system would HALT and PRESERVE POWER NOW")
        else:
            if CRITICAL_BATTERY_USE_SHUTDOWN:
                logger.critical(
                    "⚠️  DEVELOPMENT MODE: Executing shutdown command NOW - POWER WILL BE CUT")
                logger.critical(
                    "⚠️  CRITICAL_BATTERY_USE_SHUTDOWN=True - Using shutdown instead of halt")
                subprocess.run(['shutdown', '-h', 'now'], check=True)
                logger.critical("Shutdown command executed - system will shutdown and cut power")
            else:
                logger.critical(
                    "PRODUCTION MODE: Executing halt command NOW for critical battery preservation")
                logger.critical(
                    "Power relay will be preserved for manual sailing - shutdown hook will NOT cut power")
                subprocess.run(['sudo', 'halt'], check=True)
                logger.critical("Halt command executed - system should halt immediately with power preserved")

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

            # Stop WiFi monitoring
            self.wifi_monitoring_active = False

            # Stop SOS LED pattern if active
            self.sos_led_active = False

            # Stop WiFi loss LED pattern if active
            self.wifi_loss_led_active = False

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
            self.set_red_led(False)

            # Close GPIO chip if available
            if hasattr(self, 'chip') and self.chip is not None:
                self.chip.close()

            # ROS2 cleanup
            self._cleanup_ros2()

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

        logger.info("Power controller cleanup complete")

    def pause_heartbeat(self):
        """Pause the green LED heartbeat"""
        if not self.heartbeat_paused:
            logger.info("Pausing green LED heartbeat")
            self.heartbeat_paused = True
            self.heartbeat_pause_event.clear()
            # Ensure LED is off
            self.set_green_led(False)

    def resume_heartbeat(self):
        """Resume the green LED heartbeat"""
        if self.heartbeat_paused:
            logger.info("Resuming green LED heartbeat")
            self.heartbeat_paused = False
            self.heartbeat_pause_event.set()

    def set_heartbeat_frequency(self, frequency_hz):
        """Set the heartbeat frequency"""
        if frequency_hz > 0:
            logger.info(f"Setting heartbeat frequency to {frequency_hz} Hz")
            self.heartbeat_frequency_hz = frequency_hz
        else:
            logger.warning(f"Invalid heartbeat frequency: {frequency_hz}")
    
    def _update_led_heartbeat_for_pause_state(self):
        """Update LED heartbeat frequency based on controller pause state and Argo service state"""
        try:
            # Only update if Argo service is running (controller is available)
            if not self.argo_service_running:
                # Argo not running - use normal heartbeat
                self.set_heartbeat_frequency(LED_HEARTBEAT_HZ)
                logger.info("LED heartbeat: Normal (1Hz) - Argo service not running")
                return
            
            # Argo is running - check controller pause state
            if self.controller_pause_state:
                # Controller is paused - use slow heartbeat
                self.set_heartbeat_frequency(LED_PAUSED_HEARTBEAT_HZ)
                logger.info("LED heartbeat: Slow (0.5Hz) - Controller is PAUSED")
            else:
                # Controller is running - use fast heartbeat
                self.set_heartbeat_frequency(LED_HEARTBEAT_HZ * 2.0)
                logger.info("LED heartbeat: Fast (2Hz) - Controller is RUNNING")
                
        except Exception as e:
            logger.error(f"Error updating LED heartbeat for pause state: {e}")

    def _cleanup_ros2(self):
        """Clean up ROS2 resources"""
        if self.ros2_node:
            try:
                self.ros2_node.destroy_node()
            except Exception:
                pass
            self.ros2_node = None
        if rclpy and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _call_trigger_service(self, service_name: str, timeout_sec: float = 2.0) -> tuple[bool, str]:
        """Centralized function for calling ROS2 Trigger services."""
        if not self.ros2_node:
            return False, "ROS2 node not initialized"

        client = None  # Ensure client is defined in outer scope
        try:
            client = self.ros2_node.create_client(Trigger, service_name)
            if not client.wait_for_service(timeout_sec=2.0):
                logger.error(f"Service {service_name} not available")
                return False, f"Service {service_name} not available"

            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self.ros2_node, future, timeout_sec=timeout_sec)

            if future.done():
                response = future.result()
                if response is not None:
                    return response.success, response.message
                else:
                    # This can happen if the service call fails without an exception
                    return False, "Service call failed: received no response"
            else:
                logger.error(
                    f"Service call to {service_name} timed out after {timeout_sec}s")
                return False, "Service call timed out"
        except Exception as e:
            logger.error(f"Exception calling service {service_name}: {e}")
            return False, str(e)
        finally:
            if client is not None and self.ros2_node:
                self.ros2_node.destroy_client(client)


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

HARDWARE CONFIGURATION (Rev3 PCB):
  - PI3 (Pin 40): POW_OFF - Output for power relay RESET coil (active HIGH pulse)
  - PI9 (Pin 28): POW_BUT - Input from power button (active HIGH when pressed)
  - PH4 (Pin 18): Green LED in power button (active LOW - cathode control)
  - PI1 (Pin 12): Blue LED in power button (active LOW - cathode control)
  - Red LED: Not GPIO controlled (common anode RGB LED)
  - LEDs: Active LOW control (GPIO LOW = ON, GPIO HIGH = OFF)

POWER BUTTON BEHAVIOR (Active HIGH):
  - Single tap: Toggle controller pause mode
  - Double tap (2 quick taps): Toggle recording on/off
  - Quadruple tap (4 quick taps): Restart Argo launch service
  - Long press (>= threshold): Initiate shutdown sequence
  - Button press at boot activates SET coil; software always sees button released

LED INDICATORS (Active LOW):
  - Green LED: Heartbeat (1Hz normal, 2Hz when Argo running, 3-flash when recording)
  - Both LEDs: Gradual frequency increase during button press (2Hz → 20Hz)
  - Both LEDs: Short-long pattern during shutdown sequence (1Hz, 5% duty)
  - Blue LED: Show charging status via battery !CHARGING signal
  - Red LED: SOS patterns for low battery warning (slow SOS for low battery, fast SOS for critical battery)
  - Red/Green LEDs: Alternating pattern when WiFi connectivity is lost (0.5Hz alternating)

USAGE:
  ./argo_power_control.py [OPTIONS]

OPTIONS:
  --help, -h          Show this help message and exit
  --test-mode, -t     Run in test mode (disable actual shutdown and power control)
  --threshold, -T     Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
  --test-wall-message, -w  Test wall message functionality and exit (safe for testing)
  --test-notification, -n  Test desktop notification functionality and exit (safe for testing)
  --test-led-patterns, -d  Test shutdown countdown LED pattern and exit (safe for testing)
  --simulate-single-tap          Simulate a single tap to toggle controller pause mode
  --simulate-double-tap          Simulate a double tap to toggle recording
  --simulate-quadruple-tap       Simulate a quadruple tap to restart Argo service
  --simulate-critical-battery    Simulate critical battery condition for testing
  --simulate-low-battery         Simulate low battery condition (SOS LED pattern) for testing
  --simulate-wifi-loss           Simulate WiFi connectivity loss (alternating red/green LED pattern) for testing
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
  ./argo_power_control.py --test-led-patterns # Test countdown LED pattern (safe)
  ./argo_power_control.py -d                      # Test countdown LED pattern (short form)
  ./argo_power_control.py --simulate-single-tap   # Simulate single tap (controller pause toggle)
  ./argo_power_control.py --simulate-double-tap   # Simulate double tap (recording toggle)
  ./argo_power_control.py --simulate-quadruple-tap # Simulate quadruple tap (restart Argo service)
  ./argo_power_control.py --simulate-critical-battery  # Test critical battery halt sequence
  ./argo_power_control.py --simulate-low-battery   # Test low battery SOS LED pattern
  ./argo_power_control.py --simulate-wifi-loss     # Test WiFi loss alternating red/green LED pattern

REQUIREMENTS:
  - User must be member of 'gpio' group (for GPIO access)
  - python3-gpiod library installed
  - External pullup resistor on power button input
  - Proper hardware connections as described above

SAFETY FEATURES:
  - Open drain configuration prevents damage from multiple control sources
  - Graceful shutdown ensures proper system shutdown before cutting power
  - GPIO pins automatically revert to input state on halt
  - External pulldown resistor on power relay output prevents floating output
  - GPIO group membership requirement for controlled GPIO access

DEVELOPMENT CONFIGURATION:
  - CRITICAL_BATTERY_USE_SHUTDOWN flag in source code (line ~206):
    * False (default): Use halt to preserve power for manual sailing
    * True: Use shutdown to cut power completely (useful for development)
  - This flag controls behavior when battery voltage drops below {CRITICAL_BATTERY_THRESHOLD_V}V
  - Setting is logged at startup and when critical battery condition occurs
  - Battery voltage logging is throttled to only log when voltage changes by more than {BATTERY_LOG_THRESHOLD_V*1000:.0f}mV
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
    parser.add_argument('--test-mode',  action='store_true',
                        help='Run in test mode (disable actual shutdown and power control)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging (verbose output)')
    parser.add_argument('--threshold',  type=float, default=DEFAULT_SHUTDOWN_THRESHOLD_S,
                        help=f'Set button press threshold for shutdown in seconds (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})')
    parser.add_argument('--test-wall-message',  action='store_true',
                        help='Test wall message functionality and exit (safe for testing)')
    parser.add_argument('--test-notification',  action='store_true',
                        help='Test desktop notification functionality and exit (safe for testing)')
    parser.add_argument('--test-led-patterns',  action='store_true',
                        help='Test shutdown countdown LED pattern and exit (safe for testing)')
    parser.add_argument('--simulate-single-tap', action='store_true',
                        help='Simulate a single tap to toggle controller pause mode')
    parser.add_argument('--simulate-double-tap', action='store_true',
                        help='Simulate a double tap to toggle recording')
    parser.add_argument('--simulate-quadruple-tap', action='store_true',
                        help='Simulate a quadruple tap to restart Argo service')
    parser.add_argument('--simulate-critical-battery', action='store_true',
                        help='Simulate critical battery condition for testing')
    parser.add_argument('--simulate-low-battery', action='store_true',
                        help='Simulate low battery condition (SOS LED pattern) for testing')
    parser.add_argument('--simulate-wifi-loss', action='store_true',
                        help='Simulate WiFi connectivity loss (alternating red/green LED pattern) for testing')

    # Enable bash completion for command-line arguments
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    # Setup logging with debug level if requested
    setup_logging(debug=args.debug)

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
    if args.test_led_patterns:
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

    # Handle simulate single tap (Controller pause toggle)
    if args.simulate_single_tap:
        print("Simulating single tap - Toggle controller pause mode")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            controller.toggle_controller_pause()
            print("✅ Single tap simulation completed")
        except Exception as e:
            print(f"Error simulating single tap: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate double tap (Recording toggle)
    if args.simulate_double_tap:
        print("Simulating double tap - Toggle recording")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            controller.toggle_recording()
            print("✅ Double tap simulation completed")
        except Exception as e:
            print(f"Error simulating double tap: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate quadruple tap (Restart Argo service)
    if args.simulate_quadruple_tap:
        print("Simulating quadruple tap - Restart Argo service")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            controller.restart_argo_service()
            print("✅ Quadruple tap simulation completed")
        except Exception as e:
            print(f"Error simulating quadruple tap: {e}")
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
                controller.set_red_led(False)
        except Exception as e:
            print(f"Error simulating low battery: {e}")
            sys.exit(1)
        sys.exit(0)

    # Handle simulate WiFi loss
    if args.simulate_wifi_loss:
        print("Simulating WiFi connectivity loss (alternating red/green LED pattern)")
        try:
            # Create a temporary power controller instance for testing
            controller = PowerController(
                test_mode=True, threshold=args.threshold)
            # Simulate WiFi loss
            print("Simulating WiFi connectivity loss...")
            controller.wifi_loss_detected = True
            # Start alternating red/green LED pattern in separate thread
            import threading
            wifi_loss_thread = threading.Thread(target=controller.wifi_loss_led_pattern, daemon=True)
            wifi_loss_thread.start()
            print("✅ WiFi loss simulation completed - alternating red/green LED pattern running")
            print("Press Ctrl+C to stop the WiFi loss pattern test")
            # Let the pattern run for demonstration
            try:
                while controller.wifi_loss_led_active:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nStopping WiFi loss pattern test...")
                controller.wifi_loss_led_active = False
                controller.set_red_led(False)
                controller.set_green_led(False)
        except Exception as e:
            print(f"Error simulating WiFi loss: {e}")
            sys.exit(1)
        sys.exit(0)

    # Check if user is in the gpio group (required for GPIO access)
    # Skip this check if:
    #  - Running as root (systemd service)
    #  - Running in test mode without gpiod (remote testing)
    if os.geteuid() != 0 and not (args.test_mode and not _HAS_GPIOD):
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
    elif os.geteuid() == 0:
        print("Running as root - skipping GPIO group check")
    elif args.test_mode and not _HAS_GPIOD:
        print("TEST MODE: Skipping GPIO group check (gpiod not available - safe for remote testing)")

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
            f"  - Single tap toggles controller pause mode")
        print(
            f"  - Double tap (2 quick taps within {MULTI_TAP_MAX_DURATION_S}s) toggles recording")
        print(
            f"  - Quadruple tap (4 quick taps within {MULTI_TAP_MAX_DURATION_S}s) restarts Argo service")
    else:
        print("Starting power control system in NORMAL MODE")
        print("  - Full power control enabled")
        print("  - System will shutdown and cut power on long button press")
        print(
            f"  - Single tap toggles controller pause mode")
        print(
            f"  - Double tap (2 quick taps within {MULTI_TAP_MAX_DURATION_S}s) toggles recording")
        print(
            f"  - Quadruple tap (4 quick taps within {MULTI_TAP_MAX_DURATION_S}s) restarts Argo service")

    print(f"  - Button press threshold: {args.threshold} seconds")
    print(f"  - Button detection mode: Hardware interrupts (efficient)")
    
    # Show battery monitoring configuration
    if CRITICAL_BATTERY_USE_SHUTDOWN:
        critical_mode = "shutdown (CUTS POWER) - DEVELOPMENT MODE"
    else:
        critical_mode = "halt (PRESERVES POWER) - PRODUCTION MODE"
    print(
        f"  - Battery monitoring: Low warning {LOW_BATTERY_THRESHOLD_V}V (SOS LED), Critical {CRITICAL_BATTERY_THRESHOLD_V}V ({critical_mode})")
    print(f"  - Battery logging: Throttled to log only when voltage changes by >{BATTERY_LOG_THRESHOLD_V*1000:.0f}mV")
    print(f"  - WiFi monitoring: Check every {WIFI_MONITORING_INTERVAL_S}s, alternating red/green LED when lost")
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
