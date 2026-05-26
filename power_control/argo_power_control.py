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
#   - PI3 (Pin 40, wPi 27, GPIO 259): POW_OFF - Output for power relay control (active HIGH pulse to reset relay)
#   - PH0 (Pin 8, wPi 3, GPIO 224): POW_BUT - Input from power button (active HIGH when pressed)
#   - PI2 (Pin 35, wPi 24, GPIO 258): BUZZER - Pulsed via scripts/abeep.sh when needed
#   - PH4 (Pin 18, wPi 10, GPIO 228): Green LED in power button
#     Hardware: NFET control (GPIO HIGH = LED ON, GPIO LOW = LED OFF)
#     Device tree: GPIO_ACTIVE_LOW (kernel driver inverts polarity)
#     Software: Inverts brightness values (state=True → brightness=0, state=False → brightness=1)
#   - PI1 (Pin 12, wPi 6, GPIO 257): Blue LED in power button (active LOW - cathode control)
#   - PI16 (Pin 37, wPi 25, GPIO 272): Red LED: GPIO controlled (active LOW - cathode control)
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
#   - Single tap: Temporarily toggle controller_type to human (HumanController) for manual
#     servo/radio setup; second single tap restores the previous autonomous type (/tmp file)
#   - Double tap: Toggle rosbag recording on/off
#   - Quadruple tap: Restart Argo launch service
#   - Long press (>= threshold DEFAULT_SHUTDOWN_THRESHOLD_S): Initiate shutdown sequence
#   - Button press at boot activates SET coil; by software start, button is already released
#
# LED INDICATORS (Rev3 PCB - Active LOW):
#   - Green LED: Heartbeat when system is running (1Hz normal, 2Hz with Argo, 3-flash when recording)
#                OR charging status when AC power is present (via battery !CHARGING signal)
#   - Both LEDs: Gradual frequency increase during button press (2Hz → 20Hz)
#   - Both LEDs: Short-long pattern during shutdown sequence (1Hz, 5% duty)
#   - Red LED: SOS patterns for low battery warning (slow SOS for low battery, fast SOS for critical battery)
#   - Red/Green LEDs: Alternating pattern when WiFi connectivity is lost (0.5Hz alternating)
#   - All LEDs: Active LOW control (GPIO LOW = LED ON, GPIO HIGH = LED OFF)
#   - RGB State Publishing: LED RGB state is published to /argo/power_button/rgb for mast head LED mirroring
#                           (geometry_msgs/Vector3: x=red, y=green, z=blue as normalized 0.0-1.0)
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
import shlex
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
import math
from typing import Optional, Dict, Any
from logging.handlers import RotatingFileHandler
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError

# isort: off
# Add the support directory to the sys.path for ArgoBaseNode import
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../nodes/support')))

# ROS2 imports for service client
try:
    import rclpy
    from rclpy.logging import LoggingSeverity
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
    from argo_base_node import ArgoBaseNode # Import the base node
    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    # Define a dummy Node class for when ROS2 is not available
    class ArgoBaseNode:
        def __init__(self, node_name):
            pass
        def get_logger(self):
            return logging.getLogger(node_name)

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
from std_srvs.srv import Empty, Trigger
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
DEFAULT_SHUTDOWN_THRESHOLD_S = 4.0
# Warn users this many seconds before shutdown to release the button
SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S = 2.0
# Button release polling frequency during press (10 Hz - only used during button press)
BUTTON_ERROR_RECOVERY_DELAY_S = 0.1     # Delay on button read error (seconds)
# Maximum duration for multiple taps to toggle states (seconds)
MULTI_TAP_MAX_DURATION_S = 3

# LED Heartbeat Configuration
LED_HEARTBEAT_HZ = 1.0                  # Green LED heartbeat frequency (1 Hz)
LED_PAUSED_HEARTBEAT_HZ = 0.5           # Slower heartbeat when HumanController is active (0.5 Hz)
CONTROLLER_RESUME_TYPE_FILE = Path('/tmp/argo_power_controller_resume_type')

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
# Minimum interval between desktop user re-detection attempts (seconds)
DESKTOP_REDETECT_INTERVAL_S = 60
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
# AC power timeout: If AC power was plugged in more than this time ago, charger may have timed out
AC_POWER_TIMEOUT_HOURS = 20.0       # Hours - if AC power plug-in time is older than this, trigger shutdown
AC_POWER_TIMEOUT_S = AC_POWER_TIMEOUT_HOURS * 3600.0  # Convert to seconds
BATTERY_SLOPES_JSON_PATH = Path(__file__).parent.parent / 'nodes' / 'battery_slopes.json'  # Path to battery slopes JSON file
# Flag file for shutdown hook
CRITICAL_BATTERY_FLAG_FILE = '/tmp/argo_critical_battery'
# Storage rundown: discharge to this voltage then shut down (mode set via /tmp flag, cleared on reboot)
STORAGE_RUNDOWN_FLAG_FILE = '/tmp/argo_battery_storage_rundown'
STORAGE_VOLTAGE_V = 7.6  # Target voltage for storage; shutdown when reached

# SOS Pattern Configuration
SOS_PATTERN_DURATION_S = 2.0       # Total duration of one SOS pattern cycle (seconds)

# Charging State LED Pattern Configuration
# LED to use for charging state indication (easy to change later)
CHARGE_STATE_LED = 'blue'  # Options: 'red', 'green', 'blue' (currently using blue LED)
CHARGE_STATE_PERIOD_S = 3.0        # Total period for charge state flash pattern (seconds)
CHARGE_STATE_DUTY_CYCLE = 0.2      # Duty cycle for charge state flashes (20% on, 80% off) - increased from 0.1 for better visibility
CHARGE_STATE_LOG_INTERVAL_S = 30.0 # Log charge state pattern every 30 seconds when active
# Battery voltage range for percentage calculation (2S LiPo typical values)
BATTERY_VOLTAGE_MIN_V = 6.2        # Fully discharged voltage (2 cells × 3.0V)
BATTERY_VOLTAGE_MAX_V = 8.4        # Fully charged voltage (2 cells × 4.2V)

# WiFi Monitoring
WIFI_MONITORING_INTERVAL_S = 10     # Check WiFi status interval (seconds)
WIFI_CONNECTIVITY_TIMEOUT_S = 5     # Timeout for WiFi connectivity tests (seconds)
WIFI_CHECK_TIMEOUT_S = 8            # Timeout wrapper for WiFi check calls to prevent hangs (seconds)

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

# Constants
LOG_DIR = Path("/var/log.hdd/persistent")
LOG_FILE = LOG_DIR / "argo-power-control.log"

def setup_logging(debug=False):
    """Setup logging configuration to stream to stdout for journalctl."""
    # Get the root logger
    root_logger = logging.getLogger()

    # Remove any existing handlers to prevent duplicate logs or logging to files
    # This is important to override any default logging configured by rclpy.
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # Set logging level based on debug flag
    log_level = logging.DEBUG if debug else logging.INFO
    root_logger.setLevel(log_level)

    # Add a stream handler to log to stdout
    # This is important for systemd services to capture logs in journalctl
    handler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)

    # Add the new handler to the root logger
    root_logger.addHandler(handler)


# setup_logging() will be called in main() after parsing arguments
logger = logging.getLogger(__name__)


class PowerController(ArgoBaseNode):
    def __init__(self, test_mode=False, threshold=1.0):
        # Initialize the ROS2 node first
        super().__init__('argo_power_control')
        self.get_logger().info("=== POWER_CONTROL_NODE STARTUP ===")

        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        self.gpio_available = False  # Will be set to True if GPIO initialization succeeds

        # CRITICAL: Verify gpiod is available in production mode
        if not test_mode and not _HAS_GPIOD:
            logger.error("FATAL: gpiod library not available - required for production mode!")
            logger.error("This system CANNOT run safely without GPIO control.")
            logger.error("Install gpiod: sudo apt-get install python3-libgpiod")
            logger.error("If testing remotely, use --test-mode flag")
            raise RuntimeError("gpiod library not available - cannot run in production mode without GPIO control")

        if test_mode and not _HAS_GPIOD:
            logger.warning("TEST MODE: gpiod library not available - GPIO functionality will be simulated")
            logger.warning("This is OK for remote testing, but NOT for production deployment!")

        # Button state tracking
        self.initial_button_state = None
        # Flag to track when button detection should be active
        self.button_detection_active = False
        self._button_monitor_thread = None
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
        self.red_led_state = False  # Track red LED state for RGB publishing
        self.green_led_line = None  # Initialize to None, will be set if GPIO control is needed

        # Sysfs LED control paths (for kernel overlay)
        # Support both legacy/custom overlay label and generic kernel LED label.
        self.green_led_sysfs_candidates = [
            Path("/sys/class/leds/argo:green:heartbeat"),
            Path("/sys/class/leds/green_led"),
        ]
        self.green_led_sysfs_path = None
        self.green_led_brightness_path = None
        self.green_led_trigger_path = None
        self.green_led_available = False
        self.green_led_using_sysfs = False  # Track if we're using sysfs control

        # Heartbeat control
        self.heartbeat_frequency_hz = 1.0/3.0  # 3-second cycle
        self.heartbeat_duty_cycle = 0.1    # 10% duty cycle per slot
        self.heartbeat_paused = False  # Flag to pause heartbeat for special patterns
        self.heartbeat_pause_reason = None
        self.heartbeat_pause_time = 0.0
        self.last_heartbeat_pause_log = 0.0
        self.heartbeat_pause_log_interval = 5.0
        
        # LED pattern logging
        self.last_logged_pattern = None
        self.last_pattern_log_time = 0.0
        self.pattern_log_interval = 60.0  # Log at least once per minute
        self.paused_status_log_interval = 20.0  # When heartbeat paused (SOS etc.): 3 logs per minute
        
        # System status monitoring (updated via topic callbacks)
        self.anemometer_healthy = True
        self.gps_healthy = True
        self.lora_healthy = True
        self.network_healthy = True
        self.last_anemometer_update = 0.0
        self.last_gps_update = 0.0
        self.last_lora_update = 0.0
        self.last_network_check = 0.0
        self.status_timeout = 10.0  # Consider unhealthy if no update for 10 seconds
        
        # Health monitor integration for comprehensive node health
        self.health_monitor_unhealthy_count = 0
        self.health_monitor_unhealthy_nodes = []
        self.health_monitor_query_interval = 5.0  # Query every 5 seconds
        self.last_health_monitor_query = 0.0

        # Desktop notification caching
        self.cached_desktop_user = None
        self.cached_display_env = None
        self.desktop_user_detection_failed = False
        self.last_desktop_redetect_time = 0.0

        # Battery monitoring state
        self.low_battery_detected = False
        self.critical_battery_detected = False
        self.last_battery_check_time = 0.0
        self.battery_monitoring_active = False
        self.sos_led_active = False
        self.last_logged_battery_voltage = None  # Track last logged voltage for throttling
        self.last_battery_voltage = None  # Last valid battery voltage reading
        
        # I2C failure monitoring state
        self.i2c_failure_detected = False  # True when critical I2C failure is active
        self.i2c_failure_sos_active = False  # True when SOS pattern is active due to I2C failure
        
        # Charging state monitoring
        self.charging_state_active = False  # True when charging or fully charged
        self.charge_state_led_active = False  # True when charge state LED pattern is running
        self.last_charge_state_log_time = 0.0  # Track last charge state log for throttling
        self.current_charge_percentage = None  # Current battery charge percentage (0-100)
        self.is_charging = False  # True when actively charging
        self.is_fully_charged = False  # True when fully charged
        self.ac_power_from_topic = None  # AC power status from topic (for fast updates)

        # WiFi monitoring state
        self.wifi_connected = True  # Assume connected at startup
        self.last_wifi_check_time = 0.0
        self.wifi_monitoring_active = False

        # Service wait pattern state
        self.service_wait_active = False

        # GPIO Configuration
        self.GPIO_CHIP = '/dev/gpiochip0'
        self.abeep_script = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'scripts', 'abeep.sh')
        )

        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 224   # PH0 (Pin 8) - POW_BUT
        # GREEN_LED_LINE: GPIO 228 (PH4) - Green LED
        # Controlled via sysfs (no direct GPIO access needed when using sysfs)
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED (kernel overlay, controlled via sysfs)
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
                
                # Explicitly re-add a StreamHandler to the root logger AFTER rclpy.init()
                # This ensures that output is directed to the console/journal, overriding
                # any potential logging reconfiguration done by rclpy.
                root_logger = logging.getLogger()
                # Use sys.stdout to ensure it goes to the journal
                handler = logging.StreamHandler(sys.stdout) 
                formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
                handler.setFormatter(formatter)
                
                # Avoid adding duplicate handlers
                if not any(isinstance(h, logging.StreamHandler) for h in root_logger.handlers):
                    root_logger.addHandler(handler)

            except Exception as e:
                logger.error(f"Failed to initialize ROS2 node: {e}")
                self.ros2_node = None
        elif not ROS2_AVAILABLE:
            logger.warning("rclpy not found. ROS2 service calls will be disabled.")

        # NOTE: Signal handlers removed - executor.spin() handles KeyboardInterrupt properly
        # Signal handling is now done in main() function with try/except KeyboardInterrupt

        # Simple state tracking - initialize these early for test mode compatibility
        self.recording_active = False
        self.recording_service_available = False

        # Throttled logging for recording service availability
        self.last_logged_recording_service_state = None

        # Throttling for frequent checks
        self.last_recording_check_time = 0.0
        self.last_logged_system_state = None

        # Controller runtime mode (HumanController = manual-friendly LED cadence)
        self.controller_runtime_is_human = False
        self._controller_get_params_client = None
        self._controller_set_params_client = None

        # Check if Argo service is already running at startup
        self.get_logger().info("Checking if Argo service is already running...")
        self.check_argo_service_status()
        if self.argo_service_running:
            self.get_logger().info(
                "Argo service is already running - checking recording service availability...")
            # Give ROS2 a moment to discover services
            import time
            time.sleep(2)
            self.check_recording_service_availability()
            # Query current recording status at startup
            self.query_current_recording_status()

        self.get_logger().info("Power controller initialized")

        # Initialize desktop user detection and caching
        self._detect_and_cache_desktop_user()

        # Initialize and take control of sysfs-managed Green LED (GPIO 228) FIRST
        # This unbinds the kernel LED driver BEFORE we request GPIO 228
        self.init_sysfs_led()

        # Initialize GPIO for button and other LEDs (after unbinding LED driver)
        self.init_gpio()

        # Configure button line for interrupt-based monitoring
        if self.gpio_available:
            self.configure_button_for_interrupts()

        self.button_detection_active = True
        self.get_logger().info("Button detection active - ready to detect presses")

    def _detect_and_cache_desktop_user(self):
        """Detect and cache desktop user and display information"""
        try:
            self.get_logger().info(
                "Detecting desktop user and display for notification caching...")

            # Find the user who owns the desktop session
            desktop_user = None
            display_env = None

            try:
                # Look for logged-in users to find the desktop user and display
                result = subprocess.run(
                    ['who'], capture_output=True, text=True, timeout=2)
                self.get_logger().debug(f"who command output: {result.stdout.strip()}")
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
                                                self.get_logger().info(
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
                                                self.get_logger().info(
                                                    f"Detected desktop_user: {desktop_user}, display_env: {display_env}")
                                                break
                            if desktop_user:
                                break
            except Exception as e:
                self.get_logger().info(f"Could not determine desktop user from who: {e}")

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
                                            self.get_logger().info(
                                                f"Found desktop user from ps: {desktop_user}")
                                            break
                    except Exception as e:
                        self.get_logger().info(
                            f"Could not find desktop user from ps: {e}")

                # Final fallback
                if not desktop_user:
                    desktop_user = os.environ.get('SUDO_USER') or os.environ.get(
                        'USER') or os.environ.get('LOGNAME') or 'orangepi'
                    self.get_logger().info(f"Using fallback desktop_user: {desktop_user}")

            if not display_env:
                display_env = ':0'  # Default display
                self.get_logger().info(f"Using default display_env: {display_env}")

            # Cache the detected values
            self.cached_desktop_user = desktop_user
            self.cached_display_env = display_env
            self.desktop_user_detection_failed = False

            self.get_logger().info(
                f"Cached desktop user: {self.cached_desktop_user}, display: {self.cached_display_env}")

        except Exception as e:
            self.get_logger().warning(f"Failed to detect desktop user and display: {e}")
            self.desktop_user_detection_failed = True
            # Set fallback values
            self.cached_desktop_user = os.environ.get(
                'SUDO_USER') or os.environ.get('USER') or 'orangepi'
            self.cached_display_env = ':0'

    def _handle_gpio_conflicts(self):
        """Handle GPIO conflicts by stopping conflicting services"""
        try:
            # Check if boot indicator service is running and stop it
            result = subprocess.run(
                ['sudo', 'systemctl', 'is-active', 'argo_boot_indicator.service'],
                capture_output=True, text=True, timeout=5
            )
            
            if result.returncode == 0 and result.stdout.strip() == 'active':
                self.get_logger().warning("Boot indicator service is still running - attempting to stop it")
                
                # Force stop the boot indicator service
                stop_result = subprocess.run(
                    ['sudo', 'systemctl', 'stop', 'argo_boot_indicator.service'],
                    capture_output=True, text=True, timeout=10
                )
                
                if stop_result.returncode == 0:
                    self.get_logger().info("✅ Boot indicator service stopped successfully")
                    # Give it a moment to release GPIO resources
                    time.sleep(0.5)
                else:
                    self.get_logger().error(f"❌ Failed to stop boot indicator service: {stop_result.stderr}")
                    
                    # Try killing the process directly as fallback
                    self.get_logger().warning("Attempting to kill boot indicator process directly")
                    kill_result = subprocess.run(
                        ['sudo', 'pkill', '-f', 'argo_boot_indicator.py'],
                        capture_output=True, text=True, timeout=5
                    )
                    
                    if kill_result.returncode == 0:
                        self.get_logger().info("✅ Boot indicator process killed successfully")
                        time.sleep(0.5)
                    else:
                        self.get_logger().error("❌ Failed to kill boot indicator process")
                        
        except Exception as e:
            self.get_logger().error(f"Error handling GPIO conflicts: {e}")

    def init_sysfs_led(self):
        """Initialize and take control of the green LED via sysfs (no driver unbinding needed)."""
        # GPIO 228 (PH4) is managed by kernel LED driver via overlay
        # We control it via sysfs by setting trigger to "none" and controlling brightness
        # This approach works with the permissions set by argo-ph4-led-perms.service
        try:
            self.green_led_sysfs_path = None
            for candidate in self.green_led_sysfs_candidates:
                if candidate.exists():
                    self.green_led_sysfs_path = candidate
                    break

            if self.green_led_sysfs_path is None:
                candidate_str = ", ".join(str(p) for p in self.green_led_sysfs_candidates)
                self.get_logger().warning(
                    f"Green LED sysfs path not found (checked: {candidate_str}) - "
                    "kernel overlay/LED label may not be loaded"
                )
                self.green_led_available = False
                self.green_led_using_sysfs = False
                return

            self.green_led_brightness_path = self.green_led_sysfs_path / "brightness"
            self.green_led_trigger_path = self.green_led_sysfs_path / "trigger"
            self.get_logger().info(
                f"Using green LED sysfs path: {self.green_led_sysfs_path}"
            )

            # Check if brightness and trigger files are accessible
            if not self.green_led_brightness_path.exists() or not self.green_led_trigger_path.exists():
                self.get_logger().warning("Green LED sysfs control files not found")
                self.green_led_available = False
                self.green_led_using_sysfs = False
                return
            
            # Set trigger to "none" to disable kernel heartbeat and allow manual control
            try:
                with open(self.green_led_trigger_path, 'w') as f:
                    f.write('none')
                self.get_logger().info("Set LED trigger to 'none' - kernel heartbeat disabled, sysfs control enabled")
                self.green_led_available = True
                self.green_led_using_sysfs = True
            except PermissionError:
                self.get_logger().error(
                    "Permission denied setting LED trigger - "
                    "check argo-ph4-led-perms.service for current LED label"
                )
                self.green_led_available = False
                self.green_led_using_sysfs = False
                return
            except Exception as e:
                self.get_logger().error(f"Failed to set LED trigger: {e}")
                self.green_led_available = False
                self.green_led_using_sysfs = False
                return
            
            if self.green_led_available:
                self.get_logger().info(
                    f"Successfully initialized sysfs LED control for GPIO 228 (PH4) "
                    f"via {self.green_led_sysfs_path}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sysfs LED control: {e}")
            self.green_led_available = False
            self.green_led_using_sysfs = False

    def release_sysfs_led(self):
        """Release control of the green LED by restoring kernel heartbeat trigger."""
        if not self.green_led_using_sysfs:
            return
        try:
            # Turn off LED first
            if self.green_led_brightness_path.exists():
                try:
                    with open(self.green_led_brightness_path, 'w') as f:
                        f.write('0')
                except Exception as e:
                    self.get_logger().debug(f"Could not turn off LED before restoring trigger: {e}")
            
            # Restore kernel heartbeat trigger
            if self.green_led_trigger_path.exists():
                with open(self.green_led_trigger_path, 'w') as f:
                    f.write('heartbeat')
                self.get_logger().info("Restored kernel heartbeat trigger for green LED")
            
            self.green_led_using_sysfs = False
            self.green_led_available = False
        except Exception as e:
            self.get_logger().error(f"Failed to restore kernel heartbeat trigger: {e}")

    def init_gpio(self):
        """Initialize GPIO pins with conflict resolution"""
        # Skip GPIO initialization if gpiod is not available in test mode
        if not _HAS_GPIOD:
            if self.test_mode:
                self.get_logger().info("Skipping GPIO initialization - gpiod not available (test mode)")
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

        # Handle GPIO conflicts before attempting to open chip
        self._handle_gpio_conflicts()

        # Retry logic for GPIO initialization
        max_retries = 3
        retry_delay = 0.5
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Attempting to open GPIO chip: {self.GPIO_CHIP} (attempt {attempt + 1}/{max_retries})")
                self.chip = gpiod.Chip(self.GPIO_CHIP)
                self.get_logger().info(f"Successfully opened GPIO chip: {self.GPIO_CHIP}")
                break
            except OSError as e:
                if "Device or resource busy" in str(e) and attempt < max_retries - 1:
                    self.get_logger().warning(f"GPIO chip busy (attempt {attempt + 1}/{max_retries}) - retrying in {retry_delay}s")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                    continue
                else:
                    raise

        try:
            # Get line objects
            # NOTE: power_relay_line (GPIO 259) is NOT controlled by this service
            # Power relay control is handled exclusively by the shutdown hook
            self.power_button_line = self.chip.get_line(self.POWER_BUTTON_LINE)
            # Green LED is controlled via sysfs (no GPIO line needed when using sysfs)
            # Only request GPIO line if sysfs control is not available
            if not self.green_led_available or not self.green_led_using_sysfs:
                self.get_logger().warning("Green LED sysfs control not available - GPIO 228 (PH4) will remain under kernel control")
                # Don't request GPIO 228 - it's managed by kernel overlay
                self.green_led_line = None
            # Blue and red LEDs are always controlled via direct GPIO
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
            # Green LED is controlled via sysfs, so no GPIO line request needed
            # Only request GPIO line if sysfs control failed
            if self.green_led_line is not None:
                self.green_led_line.request(
                    consumer="argo_power_control.py",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[LED_OFF_STATE]  # Start with LED off (HIGH for active-low)
                )

            self.red_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[LED_OFF_STATE]  # Start with LED off (HIGH for active-low)
            )
            self.get_logger().info("GPIO pins configured successfully")
            self.gpio_available = True
        except Exception as e:
            # Release any GPIO lines that were successfully requested before the error
            self._release_gpio_lines_safe()
            
            if self.test_mode:
                self.get_logger().warning(f"GPIO not available in test mode: {e}")
                self.get_logger().warning(
                    "Continuing in test mode without GPIO functionality")
                self.gpio_available = False
                # Set dummy values for test mode
                self.chip = None
                self.power_button_line = None
                self.green_led_line = None
                self.blue_led_line = None
                self.red_led_line = None
            else:
                self.get_logger().error(f"Failed to initialize GPIO: {e}")
                self.get_logger().error(f"GPIO chip path attempted: {self.GPIO_CHIP}")
                self.get_logger().error(f"Full error details: {type(e).__name__}: {e}")
                raise

        # No ROS2 setup needed - using subprocess calls only

    def start_recording(self):
        """Start recording via ROS2 service call"""
        if self.recording_active:
            self.get_logger().warning("Recording is already active - skipping start recording")
            self.send_desktop_notification(
                "Recording Error", "Recording is already active", "warning")
            return False

        # Check if Argo service is running (required for ROS2 services to be up)
        if not self.check_argo_service_status():
            self.get_logger().error(
                "Argo service is not running - cannot start recording")
            self.send_desktop_notification(
                "Recording Error", "Argo service is not running", "critical")
            return False

        self.get_logger().info("🎬 Calling recording start service...")
        success, message = self._call_trigger_service(
            '/argo/recording/start', timeout_sec=30.0)

        if success:
            self.get_logger().info(f"✅ Recording started: {message}")
            self.recording_active = True
            self.send_desktop_notification("Recording Started", message, "normal")
            return True
        else:
            # Check if it failed because it was already running
            if "already in progress" in message.lower() or "already active" in message.lower():
                self.get_logger().info(f"📹 Recording was already in progress: {message}")
                self.recording_active = True  # Sync state
                self.send_desktop_notification(
                    "Recording Status", f"Recording was already active: {message}", "normal")
                return True
            else:
                self.get_logger().error(f"❌ Recording start failed: {message}")
                self.send_desktop_notification(
                    "Recording Error", f"Failed to start recording: {message}", "critical")
                return False

    def stop_recording(self):
        """Stop recording via ROS2 service call"""
        if not self.recording_active:
            self.get_logger().warning("Recording is not active - skipping stop recording")
            self.send_desktop_notification(
                "Recording Error", "Recording is not active", "warning")
            return False

        self.get_logger().info("⏹️ Calling recording stop service...")
        success, message = self._call_trigger_service(
            '/argo/recording/stop', timeout_sec=15.0)

        if success:
            self.get_logger().info(f"✅ Recording stopped: {message}")
            self.recording_active = False
            self.send_desktop_notification("Recording Stopped", message, "normal")
            return True
        else:
            self.get_logger().error(f"❌ Recording stop failed: {message}")
            self.send_desktop_notification(
                "Recording Error", f"Failed to stop recording: {message}", "critical")
            return False

    def check_recording_service_availability(self):
        """Check if recording service is available using a service client."""
        if not rclpy.ok(): # Use rclpy.ok() instead of self.ros2_node
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
                    self.get_logger().info("Recording service is available")
                    self.last_logged_recording_service_state = True
                self.recording_service_available = True
                return True
            else:
                # Throttled logging - only log when state changes
                if self.last_logged_recording_service_state != False:
                    self.get_logger().info("Recording service not available (no response)")
                    self.last_logged_recording_service_state = False
                self.recording_service_available = False
                return False
        except Exception as e:
            # Throttled logging - only log when state changes
            if self.last_logged_recording_service_state != False:
                self.get_logger().info(f"Recording service not available: {e}")
                self.last_logged_recording_service_state = False
            self.recording_service_available = False
            return False

    def _check_critical_nodes_running(self) -> bool:
        """Check if critical nodes are running before allowing recording."""
        # Simply check if the Argo launch service is running
        # The launch system handles critical node management
        if not self.argo_service_running:
            self.get_logger().warning("Argo service not running for recording")
            return False
        
        # Service is running, allow recording
        return True

    def query_current_recording_status(self):
        """Query current recording status by calling the get_status service."""
        if not self.check_recording_service_availability():
            # Throttled logging - only log when state changes
            if self.last_logged_recording_service_state != False:
                self.get_logger().info(
                    "Recording service not available - skipping status query")
            self.recording_active = False  # Assume not recording
            return

        success, message = self._call_trigger_service(
            '/argo/recording/get_status', timeout_sec=2.0)

        # The 'success' field of the Trigger response indicates the recording state
        old_state = self.recording_active
        self.recording_active = success

        if old_state != self.recording_active:
            self.get_logger().info(
                f"📹 Recording status changed: {'INACTIVE' if old_state else 'ACTIVE'} → {'ACTIVE' if self.recording_active else 'INACTIVE'} (state synchronized)")

    def force_recording_state_sync(self):
        """Force synchronization of recording state - useful before critical operations"""
        self.get_logger().info("🔄 Forcing recording state synchronization...")
        self.query_current_recording_status()
        self.get_logger().info(f"📹 Current recording state: {'ACTIVE' if self.recording_active else 'INACTIVE'}")
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
            self.get_logger().info(
                f"Argo system state: {'RUNNING' if current_state else 'STOPPED'} "
                f"(service: {self.argo_service_running}, recording: {self.recording_service_available})")
            self.last_logged_system_state = current_state

        return current_state

    def check_argo_service_status(self):
        """Check if the Argo launch service is running via systemctl"""
        try:
            # Check service status via systemctl
            result = subprocess.run(
                ['sudo', 'systemctl', 'is-active', 'argo_launch_standard.service'],
                capture_output=True, text=True, timeout=2
            )
            is_running = result.returncode == 0 and result.stdout.strip() == 'active'
            if is_running != self.argo_service_running:
                self.argo_service_running = is_running
                self.get_logger().info(
                    f"Argo service status changed: {'RUNNING' if is_running else 'STOPPED'}")
                # Update LED heartbeat when Argo service state changes
                self._update_led_heartbeat_for_controller_mode()

                # When Argo service stops, recording service is not available
                if not is_running:
                    self.recording_service_available = False
                    self.get_logger().info("Recording service marked as unavailable")

            return is_running
        except Exception as e:
            self.get_logger().debug(f"Error checking Argo service status: {e}")
            return False

    def start_argo_service(self):
        """Start the Argo launch service"""
        try:
            if self.test_mode:
                self.get_logger().info("TEST MODE: Would start Argo launch service")
                return True
            self.get_logger().info("Starting Argo launch service...")
            result = subprocess.run(
                ['sudo', 'systemctl', 'start', 'argo_launch_standard.service'],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                self.get_logger().info(
                    "✅ Argo launch service started, waiting for recording service to initialize...")
                # Set Argo service flag, but recording service needs time to initialize
                self.argo_service_running = True
                # Update LED heartbeat when Argo service starts
                self._update_led_heartbeat_for_controller_mode()
                # pause to allow record.py to start
                time.sleep(7)
                # check recording service availability in loop until it is available
                self.check_recording_service_availability()
                while not self.recording_service_available:
                    self.get_logger().info(
                        "Recording service not available, waiting for initialization...")
                    time.sleep(1)
                    self.check_recording_service_availability()
                self.get_logger().info(
                    "Recording service initialized, setting available flag")
                self.recording_service_available = True
                return True
            else:
                self.get_logger().error(
                    f"❌ Failed to start Argo service: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error starting Argo service: {e}")
            return False

    def stop_argo_service(self):
        """Stop the Argo launch service"""
        try:
            if self.test_mode:
                self.get_logger().info("TEST MODE: Would stop Argo launch service")
                return True

            result = subprocess.run(
                ['sudo', 'systemctl', 'stop', 'argo_launch_standard.service'],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode == 0:
                self.get_logger().info("✅ Argo launch service stopped")
                # Set flags to indicate Argo system is stopped
                self.argo_service_running = False
                self.recording_service_available = False
                # Update LED heartbeat when Argo service stops
                self._update_led_heartbeat_for_controller_mode()
                return True
            else:
                self.get_logger().error(
                    f"❌ Failed to stop Argo service: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error stopping Argo service: {e}")
            return False

    def toggle_controller_pause(self):
        """Single-tap action: temporarily switch to HumanController for manual setup, or restore."""
        try:
            if not self.check_argo_service_status():
                self.get_logger().warning("Single tap detected but Argo service is not running")
                self.send_desktop_notification(
                    "Controller toggle unavailable",
                    "Argo service must be running to switch controller mode",
                    "warning"
                )
                return

            self.get_logger().info("Single tap detected - toggling human vs autonomous controller")
            self.send_desktop_notification(
                "Controller mode",
                "Toggling human vs autonomous controller...",
                "normal"
            )

            self.service_wait_active = True
            service_wait_thread = threading.Thread(
                target=self.service_wait_pattern,
                daemon=True
            )
            service_wait_thread.start()

            success, message = self._toggle_controller_human_via_ros_param()

            self.service_wait_active = False

            if success:
                self.get_logger().info(f"✅ {message}")
                self.send_desktop_notification("Controller mode", message, "normal")
                self._update_led_heartbeat_for_controller_mode()
            else:
                self.get_logger().error(f"❌ {message}")
                self.send_desktop_notification(
                    "Controller toggle error",
                    message,
                    "critical"
                )
        except Exception as e:
            self.service_wait_active = False
            self.get_logger().error(f"Error toggling controller mode: {e}")
            self.send_desktop_notification(
                "Controller toggle error",
                f"Error: {e}",
                "critical"
            )

    def _ensure_controller_param_clients(self) -> bool:
        """Create ROS2 parameter service clients (in-process; ros2 CLI fails when rclpy is running)."""
        if not ROS2_AVAILABLE or not rclpy.ok():
            return False
        if self._controller_get_params_client is None:
            self._controller_get_params_client = self.create_client(
                GetParameters, '/controller_node/get_parameters')
            self._controller_set_params_client = self.create_client(
                SetParameters, '/controller_node/set_parameters')
        return True

    def _ros2_get_controller_type_param(self) -> Optional[str]:
        """Read controller_type from controller_node via parameter service."""
        if not self._ensure_controller_param_clients():
            return None
        client = self._controller_get_params_client
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning(
                    "controller_node get_parameters service not available")
                return None
            request = GetParameters.Request()
            request.names = ['controller_type']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                self.get_logger().warning("controller_type get timed out")
                return None
            result = future.result()
            if result is None or not result.values:
                return None
            val = result.values[0]
            if val.type == ParameterType.PARAMETER_STRING:
                return val.string_value.strip().lower()
            return None
        except Exception as e:
            self.get_logger().warning(f"Failed to get controller_type: {e}")
            return None

    def _ros2_set_controller_type_param(self, controller_type: str) -> tuple[bool, str]:
        """Set controller_type on controller_node via parameter service."""
        safe = (controller_type or '').strip().lower()
        allowed = (
            'proportional', 'wind_aware', 'return_to_home', 'crosser', 'human',
        )
        if safe not in allowed:
            return False, f"Refusing to set invalid controller_type: {controller_type!r}"
        if not self._ensure_controller_param_clients():
            return False, "ROS2 not available for parameter set"
        client = self._controller_set_params_client
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                return False, "controller_node set_parameters service not available"
            param_msg = ParameterMsg()
            param_msg.name = 'controller_type'
            param_msg.value = ParameterValue()
            param_msg.value.type = ParameterType.PARAMETER_STRING
            param_msg.value.string_value = safe
            request = SetParameters.Request()
            request.parameters = [param_msg]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                return False, "controller_type set timed out"
            result = future.result()
            if result is None or not result.results or not result.results[0].successful:
                reason = (
                    result.results[0].reason
                    if result and result.results else "no response"
                )
                return False, f"Failed to set controller_type: {reason}"
            return True, f"controller_type set to {safe}"
        except Exception as e:
            return False, str(e)

    def _toggle_controller_human_via_ros_param(self) -> tuple[bool, str]:
        """Set controller_type to human (nodes/controllers/human.py); restore prior type on second tap."""
        cur = self._ros2_get_controller_type_param()
        if cur is None:
            return False, "Could not read controller_type (is controller_node running?)"

        if cur == 'human':
            restore = 'crosser'
            try:
                if CONTROLLER_RESUME_TYPE_FILE.is_file():
                    txt = CONTROLLER_RESUME_TYPE_FILE.read_text().strip().lower()
                    if txt in ('proportional', 'wind_aware', 'return_to_home', 'crosser'):
                        restore = txt
            except OSError:
                pass
            ok, msg = self._ros2_set_controller_type_param(restore)
            if ok:
                try:
                    CONTROLLER_RESUME_TYPE_FILE.unlink(missing_ok=True)
                except OSError:
                    pass
            return ok, msg if ok else msg

        try:
            CONTROLLER_RESUME_TYPE_FILE.write_text(cur)
        except OSError as e:
            self.get_logger().warning(f"Could not persist resume controller type: {e}")
        return self._ros2_set_controller_type_param('human')

    def _get_controller_pause_state(self) -> bool:
        """Deprecated: pause removed; kept for API compatibility."""
        return False

    def _call_controller_pause_service(self, pause_state: bool) -> tuple[bool, str]:
        """Deprecated: pause service removed."""
        return False, "Controller pause service has been removed; use human controller mode"

    def restart_argo_service(self):
        """Restart the Argo launch service"""
        try:
            self.get_logger().info("Quadruple tap detected - restarting Argo launch service")
            self.send_desktop_notification(
                "Argo Service Control",
                "Restarting Argo nodes...",
                "normal"
            )

            if self.test_mode:
                self.get_logger().info("TEST MODE: Would restart Argo launch service")
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
                ['sudo', 'systemctl', 'stop', 'argo_launch_standard.service'],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode != 0:
                self.get_logger().error(f"❌ Failed to stop Argo service: {result.stderr}")
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
                ['sudo', 'systemctl', 'start', 'argo_launch_standard.service'],
                capture_output=True, text=True, timeout=15
            )

            # Stop service wait pattern
            self.service_wait_active = False
            if result.returncode == 0:
                self.get_logger().info("✅ Argo launch service restarted")
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
                self.get_logger().error(f"❌ Failed to start Argo service after restart: {result.stderr}")
                self.argo_service_running = False
                self.send_desktop_notification(
                    "Argo Service Error",
                    "Failed to start Argo nodes after restart",
                    "critical"
                )
                return False
        except Exception as e:
            # Always stop service wait pattern on exception
            self.service_wait_active = False
            self.get_logger().error(f"Error restarting Argo service: {e}")
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
                self.get_logger().info("Double tap detected - requesting to stop Argo service")
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
                self.get_logger().info("Double tap detected - requesting to start Argo service")
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
            self.get_logger().error(f"Error toggling Argo service: {e}")
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
        """
        DEPRECATED: Signal handler not used in current executor-based architecture.
        
        Previously attempted to handle shutdown signals, but this interfered with
        executor.spin() interrupt handling. Signal handling is now done in main()
        via KeyboardInterrupt exception, which properly integrates with ROS2 executor.
        
        This method is kept for reference but is not actively registered.
        """
        self.get_logger().warning("signal_handler() called but is deprecated - shutdown via main() instead")
        # Deprecated - do not use
        pass

    def configure_button_for_interrupts(self):
        """Configure button line for interrupt-based monitoring (Rev3 PCB: active-high)"""
        if not self.gpio_available:
            self.get_logger().info(
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

            # Wait a moment for the GPIO line to stabilize after reconfiguration
            time.sleep(0.1)

            # Check initial state and clear any false events that might have been triggered
            # during the reconfiguration
            try:
                initial_state = self.power_button_line.get_value()
                self.get_logger().info(f"Initial button state after interrupt configuration: {initial_state} "
                          f"({'PRESSED' if initial_state == BUTTON_PRESSED_STATE else 'RELEASED'})")

                # If the button appears to be pressed initially, it's likely a false detection
                # from the GPIO reconfiguration. Clear any pending events.
                if initial_state == BUTTON_PRESSED_STATE:
                    self.get_logger().warning("Button appears pressed after GPIO reconfiguration - "
                                 "this is likely a false detection. Clearing any pending events.")
                    # Try to clear any pending events by reading them
                    try:
                        # This will clear any events that were queued during reconfiguration
                        while True:
                            ready, _, _ = select.select([self.power_button_line.event_get_fd()], [], [], 0.0)
                            if not ready:
                                break
                            event = self.power_button_line.event_read()
                            self.get_logger().debug(f"Cleared false event during startup: {event.type}")
                    except Exception as e:
                        self.get_logger().debug(f"No events to clear: {e}")

            except Exception as e:
                self.get_logger().warning(f"Could not check initial button state: {e}")

            self.get_logger().info(
                "Button line configured for rising edge interrupt monitoring (active-high)")

        except Exception as e:
            self.get_logger().error(f"Error configuring button for interrupts: {e}")
            raise

    def read_power_button(self):
        """Read power button state - NOTE: Limited use with interrupt-based monitoring"""
        # This method is kept for compatibility but has limited use in interrupt mode
        # In interrupt mode, we primarily rely on events, but this can be used for release detection
        try:
            return self.power_button_line.get_value()
        except Exception as e:
            self.get_logger().error(f"Error reading power button: {e}")
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
        """Control green LED (system running indicator) via sysfs or direct GPIO.
        
        CRITICAL POLARITY NOTE:
        - Hardware: PH4 controls green LED via NFET (GPIO HIGH = LED ON, GPIO LOW = LED OFF)
        - Device tree: Configured as GPIO_ACTIVE_LOW, causing kernel driver to invert polarity
        - Kernel behavior: brightness=1 drives GPIO LOW, brightness=0 drives GPIO HIGH
        - Software workaround: Invert brightness values to match hardware requirements
          * state=True (LED ON) → brightness=0 (kernel drives GPIO HIGH → LED ON) ✓
          * state=False (LED OFF) → brightness=1 (kernel drives GPIO LOW → LED OFF) ✓
        """
        # GPIO 228 (PH4) is controlled via sysfs when available (preferred method)
        # Falls back to direct GPIO if sysfs is not available
        self.green_led_state = state
        # Note: RGB state publishing removed - now publishes pattern once per cycle
        
        # Use sysfs control if available (preferred method)
        if self.green_led_available and self.green_led_using_sysfs:
            try:
                # POLARITY WORKAROUND: Invert brightness values due to kernel driver polarity inversion
                # Hardware needs: GPIO HIGH = ON, GPIO LOW = OFF
                # Kernel driver with GPIO_ACTIVE_LOW: brightness=1 → GPIO LOW, brightness=0 → GPIO HIGH
                # Solution: Write inverted brightness (state=True → brightness=0, state=False → brightness=1)
                brightness_value = 0 if state else 1
                
                # Write with explicit flush to ensure it takes effect immediately
                with open(self.green_led_brightness_path, 'w') as f:
                    f.write(str(brightness_value))
                    f.flush()
                    import os
                    os.fsync(f.fileno())  # Force write to disk
                
                # Verify the write took effect and check for trigger changes
                try:
                    with open(self.green_led_brightness_path, 'r') as f:
                        actual_brightness = int(f.read().strip())
                    
                    # Only log brightness mismatches (warnings)
                    if actual_brightness != brightness_value:
                        self.get_logger().warning(
                            f"PH4 brightness mismatch! Wrote {brightness_value}, read back {actual_brightness}")
                    
                    # Check if trigger changed (kernel might have taken control back)
                    with open(self.green_led_trigger_path, 'r') as f:
                        trigger_content = f.read()
                        if '[none]' not in trigger_content:
                            self.get_logger().warning(
                                f"PH4 trigger changed! Expected [none], got: {trigger_content[:100]}")
                            # Try to restore control
                            with open(self.green_led_trigger_path, 'w') as f2:
                                f2.write('none')
                                f2.flush()
                                os.fsync(f2.fileno())
                except Exception as verify_e:
                    self.get_logger().debug(f"Could not verify PH4 state: {verify_e}")
                
                return
            except Exception as e:
                self.get_logger().error(f"Error controlling green LED via sysfs: {e}")
                # Fall through to GPIO control if sysfs fails
        
        # Fallback to direct GPIO control (if sysfs not available or failed)
        if self.gpio_available and self.green_led_line is not None:
            try:
                # Active-low LED: state=True (ON) -> GPIO=0 (LOW), state=False (OFF) -> GPIO=1 (HIGH)
                value = LED_ON_STATE if state else LED_OFF_STATE
                self.green_led_line.set_value(value)
            except Exception as e:
                self.get_logger().error(f"Error controlling green LED via GPIO: {e}")

    # def set_blue_led(self, state):
    #     """Control blue LED (charging indicator)"""
    #     if not self.gpio_available:
    #         self.blue_led_state = state
    #         return
    #     try:
    #         # Active-low LED: state=True (ON) → GPIO=0 (LOW), state=False (OFF) → GPIO=1 (HIGH)
    #         value = LED_ON_STATE if state else LED_OFF_STATE
    #         self.blue_led_line.set_value(value)
    #         self.blue_led_state = state
    #     except Exception as e:
    #         self.get_logger().error(f"Error controlling blue LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED (charging indicator)"""
        # HACK: This function is temporarily disabled. The blue LED pin is being
        # used for the green heartbeat signal due to hardware issues with PH4/PH0.
        # Still track state and publish RGB for future use when blue LED is enabled
        self.blue_led_state = state
        # Note: RGB state publishing removed - now publishes pattern once per cycle

    def set_red_led(self, state):
        """Control red LED (battery warning/SOS indicator)"""
        if not self.gpio_available:
            return
        if getattr(self, 'red_led_line', None) is None:
            self.red_led_state = state  # Keep state consistent
            return
        try:
            # Active-low LED: state=True (ON) → GPIO=0 (LOW), state=False (OFF) → GPIO=1 (HIGH)
            value = LED_ON_STATE if state else LED_OFF_STATE
            self.red_led_line.set_value(value)
            self.red_led_state = state  # Track red LED state
            # Note: RGB state publishing removed - now publishes pattern once per cycle
        except Exception as e:
            self.get_logger().error(f"Error controlling red LED: {e}")

    def _run_abeep(self, duration_s: float = 0.5, reason: str = ""):
        """Pulse buzzer using scripts/abeep.sh in a best-effort way."""
        script = self.abeep_script
        if not os.path.isfile(script):
            self.get_logger().warning(f"abeep script missing: {script}")
            return
        # --wait: blocking pulse in foreground so the shell does not exit while a
        # background subshell is still driving GPIO (avoids truncated or silent beeps).
        cmd = ['bash', script, '--wait', f"{duration_s:.3f}"]
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(15.0, duration_s + 5.0)
            )
            if result.returncode == 0:
                if reason:
                    self.get_logger().info(f"Buzzer pulse complete ({reason})")
                else:
                    self.get_logger().info("Buzzer pulse complete")
            else:
                stderr = (result.stderr or "").strip()
                stdout = (result.stdout or "").strip()
                detail = stderr or stdout or f"exit {result.returncode}"
                if reason:
                    self.get_logger().warning(f"Buzzer pulse failed ({reason}): {detail}")
                else:
                    self.get_logger().warning(f"Buzzer pulse failed: {detail}")
        except Exception as e:
            if reason:
                self.get_logger().warning(f"Buzzer pulse exception ({reason}): {e}")
            else:
                self.get_logger().warning(f"Buzzer pulse exception: {e}")

    def _run_abeep_async(self, duration_s: float = 0.5, reason: str = ""):
        """Run buzzer pulse asynchronously so control flow is not blocked."""
        threading.Thread(
            target=self._run_abeep,
            args=(duration_s, reason),
            daemon=True
        ).start()

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
    def _check_ph4_hardware_state(self):
        """Check actual PH4 hardware state and compare with expected state"""
        try:
            expected_state = self.green_led_state
            actual_state = None
            method = None
            
            sysfs_state = None
            gpio_state = None
            
            # Check sysfs state if available
            if self.green_led_available and self.green_led_using_sysfs:
                try:
                    # Check trigger first - should be "none" for manual control
                    try:
                        with open(self.green_led_trigger_path, 'r') as f:
                            trigger_content = f.read().strip()
                            # Trigger file shows available triggers with [none] indicating current
                            # Format is usually: "[none] heartbeat ..." where [none] means it's active
                            trigger_is_none = '[none]' in trigger_content
                            if not trigger_is_none:
                                # Check if it's set to heartbeat (kernel control)
                                if 'heartbeat' in trigger_content.lower() and '[heartbeat]' in trigger_content:
                                    self.get_logger().warning(
                                        f"PH4 trigger changed to heartbeat! Kernel may be controlling LED. "
                                        f"Trigger content: {trigger_content[:200]}"
                                    )
                                else:
                                    self.get_logger().warning(
                                        f"PH4 trigger may have changed! Current trigger content: {trigger_content[:200]}"
                                    )
                    except Exception as e:
                        self.get_logger().debug(f"Could not read PH4 trigger: {e}")
                    
                    # Check brightness
                    with open(self.green_led_brightness_path, 'r') as f:
                        brightness_str = f.read().strip()
                        actual_brightness = int(brightness_str) if brightness_str.isdigit() else None
                        if actual_brightness is not None:
                            sysfs_state = (actual_brightness == 1)
                except Exception as e:
                    self.get_logger().debug(f"Could not read PH4 sysfs state: {e}")
            
            # ALWAYS check actual GPIO pin state when available (even if using sysfs)
            # This detects if kernel driver is not properly controlling the GPIO
            if self.gpio_available and self.green_led_line is not None:
                try:
                    gpio_value = self.green_led_line.get_value()
                    # Active-low: GPIO 0 (LOW) = LED ON, GPIO 1 (HIGH) = LED OFF
                    gpio_state = (gpio_value == LED_ON_STATE)
                except Exception as e:
                    self.get_logger().debug(f"Could not read PH4 GPIO state: {e}")
            
            # Use GPIO state as primary (actual hardware), sysfs as secondary (what kernel thinks)
            if gpio_state is not None:
                actual_state = gpio_state
                method = "gpio"
                # Compare with sysfs if both available
                if sysfs_state is not None and sysfs_state != gpio_state:
                    self.get_logger().warning(
                        f"PH4 sysfs/GPIO mismatch! Sysfs brightness={sysfs_state}, "
                        f"GPIO pin={gpio_state} (GPIO value={gpio_value}). "
                        f"Kernel LED driver may not be controlling GPIO correctly."
                    )
            elif sysfs_state is not None:
                actual_state = sysfs_state
                method = "sysfs"
            
            # Log mismatch if detected
            if actual_state is not None and actual_state != expected_state:
                self.get_logger().warning(
                    f"PH4 state mismatch detected! Expected={expected_state}, "
                    f"Actual={actual_state} (via {method}), "
                    f"GPIO228 may be controlled by another process"
                )
            elif actual_state is not None:
                self.get_logger().debug(
                    f"PH4 state check OK: expected={expected_state}, actual={actual_state} (via {method})"
                )
        except Exception as e:
            self.get_logger().debug(f"Error checking PH4 hardware state: {e}")

    def green_led_heartbeat(self):
        """LED slot-based heartbeat - 3-second cycle with 10 slots of 0.3s each"""
        while self.running:

            if not self.running:
                break

            current_time = time.time()

            # Check network status periodically (only one that needs active checking)
            self._check_network_status()

            # Skip heartbeat if paused (for SOS, service wait, or other special patterns)
            if self.heartbeat_paused:
                if current_time - self.last_heartbeat_pause_log >= self.heartbeat_pause_log_interval:
                    reason = self.heartbeat_pause_reason or "unspecified"
                    self.get_logger().debug(f"Heartbeat paused (reason: {reason})")
                    self.last_heartbeat_pause_log = current_time
                # Periodic INFO log so SOS/special state is visible in logs (3 per minute when paused)
                if current_time - self.last_pattern_log_time >= self.paused_status_log_interval:
                    self._log_paused_state_status()
                    self.last_pattern_log_time = current_time
                time.sleep(0.1)
                continue

            # Generate LED pattern for this cycle
            pattern = self._generate_led_pattern()
            
            # Log pattern changes and periodic updates
            pattern_changed = pattern != self.last_logged_pattern
            time_since_log = current_time - self.last_pattern_log_time
            
            # Log if pattern changed or if it's been more than a minute since last log
            if pattern_changed or time_since_log >= self.pattern_log_interval:
                explanation = self._explain_led_pattern(pattern)
                if pattern_changed:
                    self.get_logger().info(f"LED pattern changed to {pattern}: ({explanation})")
                else:
                    self.get_logger().info(f"LED pattern {pattern}: ({explanation})")
                self.last_logged_pattern = pattern
                self.last_pattern_log_time = current_time
            
            # Execute the pattern over 10 slots (3 seconds total)
            self._execute_led_pattern(pattern)

    def _generate_led_pattern(self) -> str:
        """Generate LED pattern string for current system status"""
        pattern = ""
        
        # Determine status flashes needed
        status_flashes = 1  # Default: basic system alive
        if self.argo_service_running:
            status_flashes = 2  # Argo nodes launched
        if self.recording_active:
            status_flashes = 3  # Recording active

        # Add green status flashes
        for i in range(min(status_flashes, 3)):  # Max 3 green flashes
            pattern += "g"
        
        # Add red failure flashes
        failure_count = self.get_failure_count()
        for i in range(min(failure_count, 4)):  # Max 4 red flashes
            pattern += "r"
        
        # Add blue status if needed (for special states)
        if self.critical_battery_detected:
            pattern += "b"  # Blue for critical battery
        
        # Pad with dots to show remaining slots
        while len(pattern) < 10:
            pattern += "."
        
        # Truncate to exactly 10 slots
        pattern = pattern[:10]
        
        return pattern
    
    def _explain_led_pattern(self, pattern: str) -> str:
        """Generate a detailed explanation of the LED pattern"""
        parts = []
        
        # Count pattern components
        green_count = pattern.count('g')
        red_count = pattern.count('r')
        blue_count = pattern.count('b')
        off_count = pattern.count('.')
        
        # Explain green status flashes
        if green_count == 0:
            parts.append("system stopped")
        elif green_count == 1:
            parts.append("system alive")
        elif green_count == 2:
            parts.append("launch active")
            if self.recording_active:
                # This shouldn't happen with current logic, but handle it
                parts.append("(recording state unclear)")
        elif green_count >= 3:
            parts.append("recording active")
        
        # Explain red failure flashes
        if red_count > 0:
            failure_count = self.get_failure_count()
            # Use health monitor count if available, otherwise show local checks
            if self.health_monitor_unhealthy_count > 0:
                if self.health_monitor_unhealthy_nodes:
                    names_str = ", ".join(self.health_monitor_unhealthy_nodes)
                    parts.append(f"{failure_count} unhealthy: {names_str}")
                else:
                    parts.append(f"{failure_count} unhealthy nodes")
            else:
                # Fallback to local component checks
                unhealthy_components = []
                if not self.anemometer_healthy:
                    unhealthy_components.append("anem")
                if not self.network_healthy:
                    unhealthy_components.append("network")
                if not self.gps_healthy:
                    unhealthy_components.append("gps")
                if not self.lora_healthy:
                    unhealthy_components.append("lora")
                
                if unhealthy_components:
                    parts.append(f"{failure_count} unhealthy: {', '.join(unhealthy_components)}")
                else:
                    parts.append(f"{failure_count} unverified")
        
        # Explain blue status
        if blue_count > 0:
            parts.append("critical battery")
        
        # Explain off slots
        if off_count > 0:
            parts.append(f"{off_count} idle slots")
        
        return "; ".join(parts)

    def _execute_led_pattern(self, pattern: str):
        """Execute LED pattern over 10 slots of 0.3s each"""
        # Track LED states for both ON and OFF periods within each slot
        # Each slot has: ON period (0.03s) + OFF period (0.27s) = 0.3s total
        # Total: 10 slots × 2 periods = 20 states tracked
        slot_states = {'R': [], 'G': [], 'B': []}  # Track red, green, blue for each period
        
        for slot in range(10):
            if not self.running:
                break
                
            if slot < len(pattern):
                led_state = pattern[slot]
                self._set_leds_for_slot(led_state)
            else:
                # Turn off all LEDs for empty slots
                self.set_green_led(False)
                self.set_red_led(False)
                self.set_blue_led(False)
            
            # Record LED states during ON period (0.03s)
            slot_states['R'].append('r' if self.red_led_state else '-')
            slot_states['G'].append('g' if self.green_led_state else '-')
            slot_states['B'].append('b' if self.blue_led_state else '-')
            
            # Each slot: 0.03s on + 0.27s off = 0.3s total (10% duty cycle)
            self._heartbeat_sleep(0.03)  # LED on for 0.03s (10% of 0.3s)
            
            if not self.running:
                break
                
            # Turn off all LEDs for 0.27s
            self.set_green_led(False)
            self.set_red_led(False)
            self.set_blue_led(False)
            
            # Record LED states during OFF period (0.27s) - should all be OFF
            slot_states['R'].append('r' if self.red_led_state else '-')
            slot_states['G'].append('g' if self.green_led_state else '-')
            slot_states['B'].append('b' if self.blue_led_state else '-')
            
            self._heartbeat_sleep(0.27)  # LED off for 0.27s (90% of 0.3s)
        
        # Publish pattern message once per cycle for mast LED mirroring
        self._publish_led_pattern(pattern)
        
        # Print summary of LED states across all periods (20 total: 10 slots × 2 periods each)
        # Only format and log if debug level is enabled to avoid CPU overhead
        if len(slot_states['R']) == 20:
            logger = self.get_logger()
            # Check if debug logging is enabled before formatting (avoids string formatting overhead)
            # Only format strings if DEBUG level is actually enabled
            if logger.get_effective_level() <= logging.DEBUG:
                r_line = f"R:{''.join(slot_states['R'])}"
                g_line = f"G:{''.join(slot_states['G'])}"
                b_line = f"B:{''.join(slot_states['B'])}"
                logger.debug(f"LED pattern execution summary (20 periods: 10 slots × [ON,OFF]):")
                logger.debug(r_line)
                logger.debug(g_line)
                logger.debug(b_line)

    def _set_leds_for_slot(self, led_state: str):
        """Set LED states for a single slot based on character"""
        # Turn off all LEDs first
        self.set_green_led(False)
        self.set_red_led(False)
        self.set_blue_led(False)
        
        if led_state == 'g':  # Green only
            self.set_green_led(True)
        elif led_state == 'r':  # Red only
            self.set_red_led(True)
        elif led_state == 'b':  # Blue only
            self.set_blue_led(True)
        elif led_state == 'y':  # Yellow (red + green)
            self.set_red_led(True)
            self.set_green_led(True)
        elif led_state == 'p':  # Purple (red + blue)
            self.set_red_led(True)
            self.set_blue_led(True)
        elif led_state == 'c':  # Cyan (green + blue)
            self.set_green_led(True)
            self.set_blue_led(True)
        # For '.' or any other character, all LEDs remain off

    def _heartbeat_sleep(self, duration):
        """Sleep for heartbeat timing with responsive shutdown checking"""
        sleep_time = 0
        increment = 0.1  # Check every 100ms for responsive shutdown
        while sleep_time < duration and self.running:
            time.sleep(increment)
            sleep_time += increment

    def sos_led_pattern(self):
        """SOS LED pattern for low battery warning - Red LED blinks SOS in Morse code"""
        self.get_logger().info("Starting SOS LED pattern for low battery warning")
        self.sos_led_active = True
        # Clear green from last heartbeat slot (otherwise SOS mixes with green on the button)
        self.set_green_led(False)
        self.set_blue_led(False)
        # Mast mirrors /argo/power_button/pattern only once per heartbeat cycle; while paused, publish SOS + RGB
        try:
            self._publish_led_pattern("..........")
        except Exception:
            pass
        self._publish_power_button_rgb(0.0, 0.0, 0.0)

        # SOS in Morse code: ... --- ... (short-short-short, long-long-long, short-short-short)
        # Base timing: short = 0.2s, long = 0.6s, pause between letters = 0.6s, pause between SOS = 1.8s
        # Total base duration: 3.6s (scaled to SOS_PATTERN_DURATION_S to maintain duty cycle)
        base_duration = 3.6
        scale_factor = SOS_PATTERN_DURATION_S / base_duration
        
        sos_pattern = [
            # S: ... (short-short-short)
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (0.6 * scale_factor, False),  # Long pause between letters

            # O: --- (long-long-long)
            (0.6 * scale_factor, True),   # Long on
            (0.2 * scale_factor, False),  # Short off
            (0.6 * scale_factor, True),   # Long on
            (0.2 * scale_factor, False),  # Short off
            (0.6 * scale_factor, True),   # Long on
            (0.6 * scale_factor, False),  # Long pause between letters

            # S: ... (short-short-short)
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (1.8 * scale_factor, False),  # Long pause between SOS cycles
        ]

        while self.running and self.sos_led_active and self.low_battery_detected and not self.critical_battery_detected:
            try:
                for duration, led_state in sos_pattern:
                    if not self.running or not self.sos_led_active or not self.low_battery_detected or self.critical_battery_detected:
                        break

                    # Set red LED state for SOS warning
                    self.set_red_led(led_state)
                    self._publish_power_button_rgb(1.0 if led_state else 0.0, 0.0, 0.0)

                    # Sleep in small increments for responsive shutdown
                    sleep_time = 0
                    while sleep_time < duration and self.running and self.sos_led_active and self.low_battery_detected and not self.critical_battery_detected:
                        time.sleep(0.01)  # Sleep in 10ms increments
                        sleep_time += 0.01

            except Exception as e:
                self.get_logger().error(f"Error in SOS LED pattern: {e}")
                break

        # Turn off red LED when done
        self.set_red_led(False)
        self._publish_power_button_rgb(0.0, 0.0, 0.0)
        self.sos_led_active = False
        self.get_logger().info("SOS LED pattern completed")

    def sos_led_pattern_i2c_failure(self):
        """SOS LED pattern for I2C failure warning - Red LED blinks SOS in Morse code"""
        self.get_logger().info("Starting SOS LED pattern for I2C failure warning")
        self.i2c_failure_sos_active = True
        self.set_green_led(False)
        self.set_blue_led(False)
        try:
            self._publish_led_pattern("..........")
        except Exception:
            pass
        self._publish_power_button_rgb(0.0, 0.0, 0.0)

        # SOS in Morse code: ... --- ... (short-short-short, long-long-long, short-short-short)
        # Base timing: short = 0.2s, long = 0.6s, pause between letters = 0.6s, pause between SOS = 1.8s
        # Total base duration: 3.6s (scaled to SOS_PATTERN_DURATION_S to maintain duty cycle)
        base_duration = 3.6
        scale_factor = SOS_PATTERN_DURATION_S / base_duration
        
        sos_pattern = [
            # S: ... (short-short-short)
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (0.6 * scale_factor, False),  # Long pause between letters

            # O: --- (long-long-long)
            (0.6 * scale_factor, True),   # Long on
            (0.2 * scale_factor, False),  # Short off
            (0.6 * scale_factor, True),   # Long on
            (0.2 * scale_factor, False),  # Short off
            (0.6 * scale_factor, True),   # Long on
            (0.6 * scale_factor, False),  # Long pause between letters

            # S: ... (short-short-short)
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (0.2 * scale_factor, False),  # Short off
            (0.2 * scale_factor, True),   # Short on
            (1.8 * scale_factor, False),  # Long pause between SOS cycles
        ]

        while self.running and self.i2c_failure_sos_active and self.i2c_failure_detected and not self.critical_battery_detected:
            try:
                for duration, led_state in sos_pattern:
                    if not self.running or not self.i2c_failure_sos_active or not self.i2c_failure_detected or self.critical_battery_detected:
                        break

                    # Set red LED state for SOS warning
                    self.set_red_led(led_state)
                    self._publish_power_button_rgb(1.0 if led_state else 0.0, 0.0, 0.0)

                    # Sleep in small increments for responsive shutdown
                    sleep_time = 0
                    while sleep_time < duration and self.running and self.i2c_failure_sos_active and self.i2c_failure_detected and not self.critical_battery_detected:
                        time.sleep(0.01)  # Sleep in 10ms increments
                        sleep_time += 0.01

            except Exception as e:
                self.get_logger().error(f"Error in I2C failure SOS LED pattern: {e}")
                break

        # Turn off red LED when done
        self.set_red_led(False)
        self._publish_power_button_rgb(0.0, 0.0, 0.0)
        self.i2c_failure_sos_active = False
        self.get_logger().info("I2C failure SOS LED pattern completed")

    def _calculate_battery_percentage(self, voltage: float) -> float:
        """Calculate battery percentage from voltage
        
        Args:
            voltage: Battery voltage in volts
            
        Returns:
            Battery percentage (0-100), clamped to valid range
        """
        if voltage <= BATTERY_VOLTAGE_MIN_V:
            return 0.0
        elif voltage >= BATTERY_VOLTAGE_MAX_V:
            return 100.0
        else:
            # Linear interpolation between min and max voltage
            percentage = ((voltage - BATTERY_VOLTAGE_MIN_V) / 
                         (BATTERY_VOLTAGE_MAX_V - BATTERY_VOLTAGE_MIN_V)) * 100.0
            return max(0.0, min(100.0, percentage))

    def _get_charge_state_led_setter(self):
        """Get the LED setter function based on CHARGE_STATE_LED constant
        
        Returns:
            Function to set LED state (True/False)
        """
        if CHARGE_STATE_LED == 'red':
            return self.set_red_led
        elif CHARGE_STATE_LED == 'green':
            return self.set_green_led
        elif CHARGE_STATE_LED == 'blue':
            return self.set_blue_led
        else:
            self.get_logger().warning(f"Unknown CHARGE_STATE_LED: {CHARGE_STATE_LED}, defaulting to red")
            return self.set_red_led

    def _get_flash_count_from_percentage(self, percentage: float) -> int:
        """Get number of flashes based on battery percentage
        
        Args:
            percentage: Battery percentage (0-100)
            
        Returns:
            Number of flashes: 1 (0-25%), 2 (25-50%), 3 (50-75%), 4 (75-99%), 0 (100% = steady on)
        """
        if percentage >= 100.0:
            return 0  # Steady on for fully charged
        elif percentage >= 75.0:
            return 4  # 4 flashes for 75-99%
        elif percentage >= 50.0:
            return 3  # 3 flashes for 50-75%
        elif percentage >= 25.0:
            return 2  # 2 flashes for 25-50%
        else:
            return 1  # 1 flash for 0-25%

    def charge_state_led_pattern(self):
        """LED pattern for charging state indication
        
        Pattern:
        - The LED duty cycle directly represents battery charge percentage
        - Over a 3-second period (CHARGE_STATE_PERIOD_S):
          * 0% charge: LED on for 0% of period (essentially off)
          * 50% charge: LED on for 50% of period (1.5s on, 1.5s off)
          * 100% charge: LED on for 100% of period (steady ON)
        
        Uses the LED specified by CHARGE_STATE_LED constant (currently green).
        """
        self.get_logger().info("Starting charge state LED pattern")
        self.charge_state_led_active = True
        
        led_setter = self._get_charge_state_led_setter()
        
        while self.running and self.charge_state_led_active and self.charging_state_active:
            try:
                # Get current charge percentage and fully charged status (updated by battery monitoring thread)
                percentage = self.current_charge_percentage
                is_fully_charged = getattr(self, 'is_fully_charged', False)
                
                if percentage is None:
                    # No percentage data yet - wait a bit
                    time.sleep(0.5)
                    continue
                
                # Clamp percentage to valid range (0-100)
                percentage = max(0.0, min(100.0, percentage))
                
                # If battery service reports fully charged, show steady ON regardless of percentage
                if is_fully_charged or percentage >= 100.0:
                    led_setter(True)
                    # Sleep for the period, checking periodically for shutdown
                    sleep_time = 0
                    while sleep_time < CHARGE_STATE_PERIOD_S and self.running and self.charge_state_led_active and self.charging_state_active:
                        time.sleep(0.1)
                        sleep_time += 0.1
                    continue
                
                # Calculate duty cycle directly from percentage
                # duty_cycle = percentage / 100.0 (e.g., 50% charge = 0.5 duty cycle)
                duty_cycle = percentage / 100.0
                
                # Calculate on_time and off_time based on duty cycle
                on_time = CHARGE_STATE_PERIOD_S * duty_cycle
                off_time = CHARGE_STATE_PERIOD_S * (1.0 - duty_cycle)
                
                # Log the pattern for debugging
                self.get_logger().debug(
                    f"Charge state pattern: {percentage:.1f}% charge = {duty_cycle*100:.1f}% duty cycle, "
                    f"on_time={on_time*1000:.1f}ms, off_time={off_time*1000:.1f}ms")
                
                # Execute pattern: LED ON for on_time, then OFF for off_time
                # If percentage is very low (near 0%), on_time will be very short
                # If percentage is high (near 100%), off_time will be very short
                
                # LED ON phase
                if on_time > 0.01:  # Only turn on if on_time is significant (>10ms)
                    led_setter(True)
                    sleep_time = 0
                    while sleep_time < on_time and self.running and self.charge_state_led_active and self.charging_state_active:
                        time.sleep(0.01)
                        sleep_time += 0.01
                
                if not self.running or not self.charge_state_led_active or not self.charging_state_active:
                    break
                
                # LED OFF phase
                if off_time > 0.01:  # Only turn off if off_time is significant (>10ms)
                    led_setter(False)
                    sleep_time = 0
                    while sleep_time < off_time and self.running and self.charge_state_led_active and self.charging_state_active:
                        time.sleep(0.01)
                        sleep_time += 0.01
                else:
                    # If off_time is very short, LED stays on (near 100% charge)
                    # This ensures smooth transition to steady ON at 100%
                    pass
                    
            except Exception as e:
                self.get_logger().error(f"Error in charge state LED pattern: {e}")
                break
        
        # Turn off LED when done
        led_setter(False)
        self.charge_state_led_active = False
        self.get_logger().info("Charge state LED pattern completed")

    def shutdown_led_pattern(self):
        """1Hz LED pattern with configurable duty cycle during shutdown sequence"""
        self.get_logger().info(
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
                self.get_logger().error(f"Error in shutdown LED pattern: {e}")
                break

        # Turn off LEDs when done
        self.set_green_led(False)
        self.set_blue_led(False)
        self.get_logger().info("Shutdown LED pattern completed")

    def immediate_action_feedback_pattern(self, action_type):
        """Immediate LED feedback pattern for button actions

        Args:
            action_type: Type of action ('single', 'double', 'quadruple')
        """
        self.get_logger().debug(f"Starting immediate feedback pattern for {action_type} tap")

        # Define patterns for each action type
        patterns = {
            'single': [  # Single tap: temporary HumanController for manual setup
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off
                (0.1, True),   # 100ms on
                (0.1, False),  # 100ms off,
            ],
            'double': [  # Double tap: recording toggle
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
            self.get_logger().warning(f"Unknown action type: {action_type}")
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
        self.get_logger().debug(f"Immediate feedback pattern for {action_type} tap completed")

    def service_wait_pattern(self):
        """R+G alternating flash pattern during service calls"""
        self.get_logger().debug("Starting service wait pattern (R+G alternating)")

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
                self.get_logger().error(f"Error in service wait pattern: {e}")
                break

        # Turn off both LEDs when done
        self.set_red_led(False)
        self.set_green_led(False)
        self.get_logger().debug("Service wait pattern completed")

    def gradual_frequency_pattern(self):
        """Gradual frequency increase LED pattern during button press - 2Hz to 20Hz over threshold time"""
        start_time = time.time()
        self.get_logger().debug(
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
            self.get_logger().info(
                "GPIO not available - skipping button monitoring in test mode")
            # In test mode without GPIO, just sleep to keep the thread alive
            while self.running:
                time.sleep(1.0)
            return

        self.get_logger().info("Using hardware interrupt-based button monitoring")
        self.get_logger().info("Monitoring for rising edge events (POW_BUT going high - active-high button)")
        self.get_logger().info("Button detection is always active (button powers on system via SET coil)")

        while self.running:
            if not self.gpio_available or self.power_button_line is None:
                break

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
                            self.get_logger().debug(
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
                            self.get_logger().warning(
                                f"Unexpected event type: {event.type}")

                    except Exception as e:
                        self.get_logger().error(f"Error reading GPIO event: {e}")

                # This timeout happens every 1 second and uses minimal CPU

            except Exception as e:
                if not self.running or not self.gpio_available or self.power_button_line is None:
                    break
                self.get_logger().error(
                    f"Error in interrupt-based button monitoring: {e}")
                time.sleep(BUTTON_ERROR_RECOVERY_DELAY_S)

        self.get_logger().info("Hardware interrupt-based button monitoring stopped")

    def monitor_button_release_timeout(self):
        """Monitor for button release by checking current state and handle long press timeout"""
        # Rising-edge interrupt marks press; poll line value to detect release
        poll_interval = 0.1  # 10Hz polling during button press only
        released_normally = False

        try:
            while self.running and self.power_button_pressed:
                try:
                    # Read current button state to detect release
                    current_state = self.get_current_button_state()

                    # Check for button release (active-high button: released = low)
                    if current_state == BUTTON_RELEASED_STATE:  # Button released
                        press_duration = time.time() - self.button_press_start_time
                        self.get_logger().debug(
                            f"Power button released after {press_duration:.2f} seconds (interrupt + polling)")

                        if press_duration >= self.SHUTDOWN_THRESHOLD:
                            self.get_logger().info(
                                "Long press detected, initiating shutdown...")
                            self.initiate_shutdown()
                        else:
                            # Check if button was released after warning but before shutdown (shutdown cancelled)
                            if (press_duration >= SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S and
                                self.warning_notification_sent and
                                    not self.shutdown_initiated):
                                self.get_logger().info(
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
                        released_normally = True
                        break  # Exit this monitoring thread

                    # Check for long press timeout while button is still held
                    else:
                        press_duration = time.time() - self.button_press_start_time

                        # Check for warning threshold - send notification to warn user to release button
                        if (press_duration >= SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S and
                            not self.warning_notification_sent and
                                not self.shutdown_initiated):
                            self.get_logger().info(
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
                                self.get_logger().info(
                                    "Long press threshold reached, initiating shutdown...")
                                self.initiate_shutdown()
                            released_normally = True
                            break  # Exit monitoring loop

                    time.sleep(poll_interval)

                except Exception as e:
                    self.get_logger().error(f"Error in button release monitoring: {e}")
                    time.sleep(BUTTON_ERROR_RECOVERY_DELAY_S)
                    break
        finally:
            if not released_normally and self.power_button_pressed:
                self.get_logger().warning(
                    "Button release monitor exited abnormally - clearing pressed state")
                self.power_button_pressed = False
                self.button_press_start_time = None
                self.warning_notification_sent = False

    def get_current_button_state(self):
        """Get current button state - works with interrupt-configured line"""
        if not self.gpio_available:
            self.get_logger().debug(
                "GPIO not available - assuming button released in test mode")
            return BUTTON_RELEASED_STATE  # Assume released in test mode
        try:
            # The line is configured for events, but we can still read the current value
            # This is needed to detect button release since we only get falling edge interrupts
            return self.power_button_line.get_value()
        except Exception as e:
            self.get_logger().error(f"Error reading current button state: {e}")
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
            self.get_logger().debug("Long press detected - clearing tap history")
            return

        # Add this tap to the history
        self.tap_times.append(current_time)
        self.get_logger().debug(
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
            self.get_logger().debug(
                f"Cleaned up {old_count - len(self.tap_times)} old taps")

        # Process tap sequence based on count
        tap_count = len(self.tap_times)

        if tap_count < 1:
            # No taps - ignore
            self.get_logger().debug(f"No taps detected - ignoring")
            self.tap_times.clear()
            return

        if tap_count > 5:
            # Too many taps - ignore
            self.get_logger().warning(f"Too many taps ({tap_count}) - ignoring")
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
                    self.get_logger().info("Single tap detected!")
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
                        "Controller mode",
                        "Single tap — temporary HumanController for manual servo setup...",
                        "normal")
                    self.toggle_controller_pause()
                elif tap_count == 2:
                    self.get_logger().info(
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
                    self.get_logger().info(
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
                    self.get_logger().info(
                        f"{tap_count} taps detected - no action assigned")
                    self.tap_times.clear()
            else:
                self.get_logger().warning(
                    f"Tap sequence too slow: {duration:.3f}s > {MULTI_TAP_MAX_DURATION_S}s")
                self.tap_times.clear()

        # Clear the timeout timer reference
        self.tap_timeout_timer = None

    def toggle_recording(self):
        """Toggle rosbag recording on/off with immediate feedback"""
        # --- Real mode ---
        # Force synchronization of recording state before toggling
        self.get_logger().info("Double tap detected - forcing recording state synchronization...")
        is_currently_recording = self.force_recording_state_sync()

        # Check if services are ready after state query
        if not self.check_argo_service_status() or not self.recording_service_available:
            self.get_logger().warning(
                "Double tap detected but Argo/recording service not ready.")
            self.send_desktop_notification(
                "Recording Unavailable", "Argo service is not ready for recording.", "critical")
            return
        
        # NEW: Check if critical nodes are running before allowing recording
        if not self._check_critical_nodes_running():
            self.get_logger().warning(
                "Double tap detected but critical nodes not running.")
            self.send_desktop_notification(
                "Recording Unavailable", "Critical nodes are not running.", "critical")
            return

        if is_currently_recording:
            self.get_logger().info("Double tap detected - requesting to stop recording")
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
            self.get_logger().info("Double tap detected - requesting to start recording")
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
        """Send desktop notification using notify-send with proper environment for systemd.

        If a previous send failed, desktop_user_detection_failed is set and all notifications
        are skipped until we re-detect (throttled to DESKTOP_REDETECT_INTERVAL_S). On send
        failure we re-detect once and retry before setting the sticky failure flag.
        """
        # If we previously failed, try re-detecting desktop user (throttled) so notifications can recover
        if self.desktop_user_detection_failed:
            now = time.time()
            if (now - self.last_desktop_redetect_time) >= DESKTOP_REDETECT_INTERVAL_S:
                self.get_logger().debug("Re-detecting desktop user after previous notification failure")
                self.last_desktop_redetect_time = now
                self._detect_and_cache_desktop_user()
            if self.desktop_user_detection_failed:
                self.get_logger().debug(
                    "Desktop user detection previously failed - skipping notification")
                return

        # Modify title and message for test mode
        test_title = f"[TEST] {title}" if self.test_mode else title
        test_message = f"{message}\n\n(This is a test notification)" if self.test_mode else message
        if self.test_mode:
            self.get_logger().debug(
                f"TEST MODE: Sending notification - {title}: {message}")

        expire_time = expire_time_ms if expire_time_ms is not None else NOTIFICATION_EXPIRE_TIME_MS

        for attempt in range(2):
            failed = False
            try:
                if self._do_send_desktop_notification(test_title, test_message, urgency, expire_time, title):
                    return
                # _do_send returned False (e.g. no cached desktop user)
                failed = True
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, Exception) as e:
                self.get_logger().warning(f"Desktop notification failed: {e}")
                failed = True
            if failed:
                if attempt == 0:
                    self.get_logger().info("Re-detecting desktop user and retrying notification")
                    self._detect_and_cache_desktop_user()
                    if not self.desktop_user_detection_failed:
                        continue
                self.desktop_user_detection_failed = True
                return
        self.desktop_user_detection_failed = True

    def _do_send_desktop_notification(self, test_title, test_message, urgency, expire_time, log_title):
        """Perform one attempt to send a desktop notification. Returns True on success.
        Raises on failure (caller handles retry and desktop_user_detection_failed)."""
        if os.geteuid() == 0:
            if not self.cached_desktop_user:
                self.get_logger().debug("No cached desktop user - skipping notification")
                return False

            session_pid = None
            try:
                for process_name in ['gnome-shell', 'plasma-shell', 'xfce4-session',
                                     'lxsession', 'i3', 'sway']:
                    pgrep_cmd = ['pgrep', '-u', self.cached_desktop_user, '-o', process_name]
                    result = subprocess.run(
                        pgrep_cmd, capture_output=True, text=True, timeout=2)
                    if result.returncode == 0:
                        session_pid = result.stdout.strip()
                        self.get_logger().debug(f"Found {process_name} session with PID: {session_pid}")
                        break

                if session_pid:
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
                        self.get_logger().debug(
                            f"Found graphical session for user {self.cached_desktop_user} (PID: {session_pid})")
                        cmd = [
                            'sudo', '-u', self.cached_desktop_user,
                            'env',
                            f'DISPLAY={display}',
                            f'DBUS_SESSION_BUS_ADDRESS={dbus_address}',
                            'notify-send', '--urgency', urgency,
                            '--expire-time', str(expire_time),
                            test_title, test_message
                        ]
                        subprocess.run(cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                        self.get_logger().debug(f"Desktop notification sent: {log_title}")
                        return True
                    self.get_logger().warning(
                        f"Could not find DISPLAY/DBUS for user {self.cached_desktop_user}")
            except (subprocess.TimeoutExpired, FileNotFoundError, PermissionError) as e:
                self.get_logger().warning(f"Failed to get graphical session environment: {e}")

            self.get_logger().debug("Falling back to standard environment notification method")
            cmd = [
                'sudo', '-u', self.cached_desktop_user,
                'env',
                'DISPLAY=:0',
                'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus',
                'notify-send', '--urgency', urgency,
                '--expire-time', str(expire_time),
                test_title, test_message
            ]
            subprocess.run(cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            self.get_logger().debug(f"Desktop notification sent (fallback): {log_title}")
            return True

        # Running as regular user
        cmd = [
            'notify-send', '--urgency', urgency,
            '--expire-time', str(expire_time),
            test_title, test_message
        ]
        subprocess.run(cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
        self.get_logger().debug(f"Desktop notification sent (user): {log_title}")
        return True

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
                    self.get_logger().info(f"TEST MODE: Wall message sent: {message}")
                else:
                    # Use wall command to send message to all logged-in users
                    subprocess.run(['wall', message], check=True,
                                   timeout=WALL_MESSAGE_TIMEOUT_S)
                    self.get_logger().info("Shutdown message broadcasted to all users")
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    "Wall command timed out - message may not have been sent")
            except subprocess.CalledProcessError as e:
                self.get_logger().warning(f"Failed to broadcast wall message: {e}")
            except Exception as e:
                self.get_logger().warning(
                    f"Unexpected error broadcasting wall message: {e}")

        except Exception as e:
            self.get_logger().error(f"Error preparing shutdown message: {e}")

    def initiate_shutdown(self):
        """Initiate system shutdown sequence"""
        if self.shutdown_initiated:
            return

        self.shutdown_initiated = True

        self.get_logger().info("Initiating shutdown sequence...")
        self._run_abeep_async(0.5, "shutdown_initiated")

        # IMMEDIATE: Start shutdown LED pattern first for instant visual feedback
        # This runs in a daemon thread and does not block other actions
        shutdown_pattern_thread = threading.Thread(
            target=self.shutdown_led_pattern,
            daemon=True
        )
        shutdown_pattern_thread.start()

        # Publish button event
        self._publish_button_event("long_press_shutdown")

        # NEW: Stop recording before shutdown to prevent hangs
        if self.recording_active:
            self.get_logger().warning("Recording is active - stopping recording before shutdown...")
            self.send_desktop_notification(
                "Shutdown Preparations",
                "Stopping recording before shutdown...",
                "critical"
            )
            success = self.stop_recording()
            if success:
                self.get_logger().info("✅ Recording stopped successfully before shutdown")
                time.sleep(2)  # Allow recording to fully stop
            else:
                self.get_logger().warning("⚠️  Failed to stop recording before shutdown, proceeding anyway...")

        # Stop background monitoring threads early in shutdown sequence
        self.get_logger().info("Stopping background monitoring threads...")
        self.battery_monitoring_active = False
        self.wifi_monitoring_active = False
        self.sos_led_active = False
        self.charge_state_led_active = False
        self.charging_state_active = False
        self.service_wait_active = False

        # Clear critical battery flag if set
        if self.critical_battery_detected:
            self._clear_critical_battery_flag()

        # Brief delay to show shutdown pattern, then release control to kernel
        time.sleep(2)  # Show shutdown pattern for 2 seconds
        
        # CRITICAL: Release control of green LED back to kernel EARLY in shutdown
        # This ensures kernel heartbeat is restored before process is killed
        # The shutdown LED pattern will stop working after this, but kernel heartbeat takes over
        self.get_logger().info("Releasing green LED control to kernel for heartbeat...")
        self.release_sysfs_led()

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
            self.get_logger().info(
                "TEST MODE: Shutdown command disabled - would normally execute: shutdown -h now")
            # In test mode, let the shutdown LED pattern run for demonstration
            self.get_logger().info(
                f"TEST MODE: Running shutdown LED pattern for {TEST_MODE_SHUTDOWN_DELAY_S} seconds...")
            # Let the pattern run for demonstration
            time.sleep(TEST_MODE_SHUTDOWN_DELAY_S)
            self.running = False  # Stop running after LED pattern demonstration
        else:
            self.get_logger().error("Executing shutdown command: shutdown -h now")
            subprocess.run(['shutdown', '-h', 'now'], check=True)
            self.get_logger().info("Shutdown command executed successfully")
            # Don't set self.running = False here - let the system shutdown
            # GPIO pins will automatically revert to input state on halt

    def _create_power_services(self):
        """Create ROS2 services for power control"""
        if not rclpy.ok() or self.power_services_created:
            return

        try:
            # Create power control services
            from std_srvs.srv import Trigger
            from std_msgs.msg import Bool, UInt8, Int32
            from geometry_msgs.msg import Vector3
            self.start_recording_service = self.create_service(
                Trigger, '/argo/power/start_recording', self._handle_start_recording_service)
            self.stop_recording_service = self.create_service(
                Trigger, '/argo/power/stop_recording', self._handle_stop_recording_service)
            self.toggle_recording_service = self.create_service(
                Trigger, '/argo/power/toggle_recording', self._handle_toggle_recording_service)
            self.shutdown_service = self.create_service(
                Trigger, '/argo/power/shutdown', self._handle_shutdown_service)
            self.toggle_argo_service_service = self.create_service( # Renamed to avoid conflict
                Trigger, '/argo/power/toggle_argo', self._handle_toggle_argo_service)

            # Create publishers
            self.power_status_publisher = self.create_publisher(
                String, '/argo/power/status', 10)
            self.button_event_publisher = self.create_publisher(
                String, '/argo/power/button_events', 10)
            # LED pattern publisher for mast head LED mirroring (publishes once per cycle)
            # Format: JSON string with category, pattern, cycle_duration, slot_duration
            self.power_button_pattern_publisher = self.create_publisher(
                String, '/argo/power_button/pattern', 10)
            self.power_button_rgb_publisher = self.create_publisher(
                Vector3, '/argo/power_button/rgb', 10)

            # Runtime controller class name from controller_node (HumanController = manual)
            self.controller_state_sub = self.create_subscription(
                String, '/controller_state', self._controller_runtime_state_callback, 10)

            # Create subscriptions for system status monitoring
            
            # Anemometer health monitoring
            self.anem_health_sub = self.create_subscription(
                Bool, '/anem_node_health', self._anem_health_callback, 10)
            
            # GPS status monitoring (satellite count)
            self.gps_satellites_sub = self.create_subscription(
                UInt8, '/gps_num_satellites', self._gps_satellites_callback, 10)
            
            # LoRa signal strength monitoring
            self.lora_signal_sub = self.create_subscription(
                Int32, '/lora_signal_strength', self._lora_signal_callback, 10)
            
            # AC power status monitoring (for fast charge state updates)
            self.ac_power_sub = self.create_subscription(
                Bool, '/ac_power_present', self._ac_power_callback, 10)
            
            # I2C failure monitoring (critical - battery monitoring unavailable)
            self.i2c_failure_sub = self.create_subscription(
                Bool, '/argo/critical/i2c_failure', self._i2c_failure_callback, 10)
            
            # Health monitor service client for comprehensive health status
            self.health_monitor_client = self.create_client(
                Trigger, '/argo/health/status')

            self.power_services_created = True
            self.get_logger().info("Power control ROS2 services created:")
            self.get_logger().info("  - /argo/power/start_recording")
            self.get_logger().info("  - /argo/power/stop_recording")
            self.get_logger().info("  - /argo/power/toggle_recording")
            self.get_logger().info("  - /argo/power/shutdown")
            self.get_logger().info("  - /argo/power/toggle_argo")
            self.get_logger().info("  - /argo/power/status (topic)")
            self.get_logger().info("  - /argo/power/button_events (topic)")
            self.get_logger().info("  - /argo/power_button/pattern (mast slot patterns)")
            self.get_logger().info("  - /argo/power_button/rgb (mast mirrors SOS / instant RGB)")
            self.get_logger().info("  - /controller_state (subscription)")
            self.get_logger().info("  - /ac_power_present (subscription - for fast charge state updates)")
            self.get_logger().info("  - /argo/critical/i2c_failure (subscription - for critical I2C failure detection)")
        except Exception as e:
            self.get_logger().error(f"Failed to create power services: {e}")

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
                self.get_logger().info(f"Published button event: {event}")
            except Exception as e:
                self.get_logger().debug(f"Failed to publish button event: {e}")

    def _publish_led_pattern(self, pattern: str):
        """Publish LED pattern for mast head LED mirroring (once per cycle).
        
        Publishes a JSON message with:
        - category: Pattern category (heartbeat, sos, charging, shutdown, wifi_loss)
        - pattern: Pattern string (e.g., "ggrrrr....")
        - cycle_duration: Total cycle duration in seconds (3.0 for heartbeat)
        - slot_duration: Duration of each slot in seconds (0.3 for heartbeat)
        - on_period: ON period duration in seconds (0.03 for heartbeat)
        - off_period: OFF period duration in seconds (0.27 for heartbeat)
        """
        if not hasattr(self, 'power_button_pattern_publisher') or not self.power_button_pattern_publisher:
            return  # Publisher not yet created (during initialization)
        
        try:
            import json
            from std_msgs.msg import String
            
            # Determine pattern category
            category = "heartbeat"
            if self.heartbeat_paused:
                if self.sos_led_active:
                    category = "sos"
                elif self.i2c_failure_sos_active:
                    category = "sos"
                elif self.charge_state_led_active:
                    category = "charging"
                elif self.shutdown_initiated:
                    category = "shutdown"
                elif self.wifi_monitoring_active and not self.wifi_connected:
                    category = "wifi_loss"
                else:
                    category = self.heartbeat_pause_reason or "unknown"
            
            # Create pattern message
            pattern_data = {
                "category": category,
                "pattern": pattern,
                "cycle_duration": 3.0,  # 10 slots × 0.3s = 3.0s
                "slot_duration": 0.3,   # Each slot is 0.3s
                "on_period": 0.03,      # ON period within each slot
                "off_period": 0.27      # OFF period within each slot
            }
            
            msg = String()
            msg.data = json.dumps(pattern_data)
            self.power_button_pattern_publisher.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"Failed to publish LED pattern: {e}")

    def _publish_power_button_rgb(self, r: float, g: float, b: float):
        """Instant RGB for mast mirroring (required when heartbeat is paused — no slot pattern published)."""
        pub = getattr(self, 'power_button_rgb_publisher', None)
        if not pub:
            return
        try:
            from geometry_msgs.msg import Vector3 as Vector3Msg
            msg = Vector3Msg(x=float(r), y=float(g), z=float(b))
            pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"Failed to publish RGB mirror: {e}")

    def _controller_runtime_state_callback(self, msg):
        """Track active controller class name for LED cadence (HumanController = human mode)."""
        name = (msg.data or '').strip()
        is_human = name == 'HumanController'
        old = getattr(self, 'controller_runtime_is_human', False)
        self.controller_runtime_is_human = is_human
        if old != is_human:
            self.get_logger().info(
                f"Controller runtime state: {'HumanController (manual)' if is_human else (name or 'unknown')}"
            )
            self._update_led_heartbeat_for_controller_mode()
        else:
            self.get_logger().debug(f"Controller runtime state: {name or 'unknown'}")

    def _anem_health_callback(self, msg):
        """Receive anemometer health status updates"""
        import time
        self.last_anemometer_update = time.time()
        old_healthy = self.anemometer_healthy
        self.anemometer_healthy = msg.data
        
        if old_healthy != self.anemometer_healthy:
            self.get_logger().info(f"Anemometer health changed: {'HEALTHY' if msg.data else 'UNHEALTHY'}")

    def _gps_satellites_callback(self, msg):
        """Receive GPS satellite count updates"""
        import time
        self.last_gps_update = time.time()
        old_healthy = self.gps_healthy
        self.gps_healthy = msg.data >= 4  # Need at least 4 satellites for good fix
        
        if old_healthy != self.gps_healthy:
            self.get_logger().info(f"GPS health changed: {'HEALTHY' if self.gps_healthy else 'UNHEALTHY'} ({msg.data} satellites)")

    def _lora_signal_callback(self, msg):
        """Receive LoRa signal strength updates"""
        import time
        self.last_lora_update = time.time()
        old_healthy = self.lora_healthy
        self.lora_healthy = msg.data > -90  # Good signal threshold
        
        if old_healthy != self.lora_healthy:
            self.get_logger().info(f"LoRa health changed: {'HEALTHY' if self.lora_healthy else 'UNHEALTHY'} (signal: {msg.data} dBm)")

    def _ac_power_callback(self, msg):
        """Receive AC power status updates from topic - triggers immediate charge state update"""
        import time
        old_ac_power = self.ac_power_from_topic
        self.ac_power_from_topic = msg.data
        
        # Only trigger update if AC power status actually changed
        if old_ac_power is not None and old_ac_power != msg.data:
            self.get_logger().info(f"AC power status changed (from topic): {old_ac_power} -> {msg.data}")
            
            # Trigger immediate charge state update by calling battery service
            # This will update charge state pattern immediately instead of waiting for next polling cycle
            if self.battery_monitoring_active:
                # Use a thread to avoid blocking the callback
                threading.Thread(
                    target=self._immediate_charge_state_update,
                    daemon=True
                ).start()
        elif old_ac_power is None:
            # First update - just log
            self.get_logger().debug(f"AC power status (from topic): {msg.data}")

    def _i2c_failure_callback(self, msg):
        """Receive I2C failure status updates from topic - triggers SOS pattern for critical failures"""
        old_i2c_failure = self.i2c_failure_detected
        self.i2c_failure_detected = msg.data
        
        # Always clear SOS pattern if receiving False, even if state didn't change
        # This handles the case where power control starts after battery service and misses initial False
        if not self.i2c_failure_detected and self.i2c_failure_sos_active:
            self.get_logger().info("✅ I2C failure cleared (received False) - stopping SOS pattern")
            self.i2c_failure_sos_active = False
            self.resume_heartbeat(reason="i2c_failure_cleared")
            return
        
        # Only trigger SOS pattern if I2C failure status actually changed
        if old_i2c_failure != self.i2c_failure_detected:
            if self.i2c_failure_detected:
                # Critical I2C failure detected - start SOS pattern
                self.get_logger().error("🔴 CRITICAL I2C FAILURE detected - battery monitoring unavailable!")
                self.get_logger().error("Starting SOS LED pattern for I2C failure warning")
                
                # Stop charge state pattern if running (I2C failure takes priority)
                if self.charge_state_led_active:
                    self.charge_state_led_active = False
                    self.charging_state_active = False
                    # Immediately turn off charge state LED
                    led_setter = self._get_charge_state_led_setter()
                    led_setter(False)
                
                # Pause heartbeat to make SOS pattern visible
                self.pause_heartbeat(reason="i2c_failure")
                
                # Start SOS LED pattern in separate thread (if not already active)
                if not self.i2c_failure_sos_active and not self.sos_led_active:
                    self.i2c_failure_sos_active = True
                    threading.Thread(target=self.sos_led_pattern_i2c_failure, daemon=True).start()
                
                # Send desktop notification
                self.send_desktop_notification(
                    "CRITICAL I2C FAILURE",
                    "Battery monitoring unavailable due to I2C bus failure\nSOS LED pattern activated\nCheck I2C connections and argo_battery_water.service",
                    "critical"
                )
            else:
                # I2C failure recovered - stop SOS pattern
                self.get_logger().info("✅ I2C failure recovered - battery monitoring restored")
                self.i2c_failure_sos_active = False
                
                # Resume heartbeat now that I2C failure is cleared
                self.resume_heartbeat(reason="i2c_failure_recovered")
                
                # Send desktop notification
                self.send_desktop_notification(
                    "I2C Recovery",
                    "I2C communication restored - battery monitoring active",
                    "normal"
                )
    
    def _check_initial_i2c_failure_state(self):
        """Check initial I2C failure state from battery service on startup.
        This ensures we have the correct state even if we missed the initial topic message."""
        try:
            # Wait a moment for services to be ready
            time.sleep(2)
            
            # Call battery service to get current I2C failure state
            battery_data = self._call_battery_service()
            if battery_data:
                i2c_failure = battery_data.get('i2c_failure', False)
                self.get_logger().info(f"Initial I2C failure state from battery service: {i2c_failure}")
                
                # Update state and handle accordingly
                old_state = self.i2c_failure_detected
                self.i2c_failure_detected = i2c_failure
                
                # If I2C is working (False) but SOS is active, clear it
                if not i2c_failure and self.i2c_failure_sos_active:
                    self.get_logger().info("Clearing I2C failure SOS pattern based on battery service status")
                    self.i2c_failure_sos_active = False
                    self.resume_heartbeat(reason="i2c_failure_cleared_on_startup")
                # If I2C is failing (True) but SOS is not active, start it
                elif i2c_failure and not self.i2c_failure_sos_active and not self.sos_led_active:
                    self.get_logger().warning("Starting I2C failure SOS pattern based on battery service status")
                    self.pause_heartbeat(reason="i2c_failure")
                    self.i2c_failure_sos_active = True
                    threading.Thread(target=self.sos_led_pattern_i2c_failure, daemon=True).start()
            else:
                self.get_logger().warning("Could not get initial I2C failure state from battery service")
        except Exception as e:
            self.get_logger().warning(f"Error checking initial I2C failure state: {e}")

    def _immediate_charge_state_update(self):
        """Immediately update charge state based on current AC power and battery status"""
        try:
            # Call battery service to get current status
            battery_data = self._call_battery_service()
            if battery_data:
                battery_voltage = battery_data.get('battery_voltage', 0)
                charging_status = battery_data.get('charging_status', None)
                ac_power_present = battery_data.get('ac_power_present', None)
                
                # Use topic value if available, otherwise use service value
                ac_power = self.ac_power_from_topic if self.ac_power_from_topic is not None else ac_power_present
                
                # Validate voltage
                if battery_voltage <= 0 or battery_voltage < 3.0 or battery_voltage > 30.0:
                    return  # Invalid reading, skip update
                
                # Calculate percentage
                percentage = self._calculate_battery_percentage(battery_voltage)
                self.current_charge_percentage = percentage
                
                # Determine charging state
                is_in_charging_state = (charging_status is True) or (ac_power is True)
                is_fully_charged_now = battery_voltage >= (BATTERY_VOLTAGE_MAX_V * 0.98)
                is_charging_now = is_in_charging_state and not is_fully_charged_now
                
                self.is_charging = is_charging_now
                self.is_fully_charged = is_fully_charged_now
                
                # Update charge state based on AC power and battery status
                # CRITICAL: Charge state pattern should ONLY be active when AC power is present
                # Even if battery is fully charged, don't show charge state pattern without AC power
                charge_state_should_be_active = (ac_power is True) and (is_charging_now or is_fully_charged_now)
                
                # Handle low battery + AC power transition
                if self.low_battery_detected and not self.critical_battery_detected:
                    if ac_power is not True:
                        # No AC power - ensure SOS is active
                        if not self.sos_led_active:
                            self.get_logger().info("AC power removed - starting SOS pattern")
                            self.pause_heartbeat(reason="low_battery")
                            if self.charge_state_led_active:
                                self.charge_state_led_active = False
                                self.charging_state_active = False
                                # Immediately turn off charge state LED
                                led_setter = self._get_charge_state_led_setter()
                                led_setter(False)
                            threading.Thread(target=self.sos_led_pattern, daemon=True).start()
                    else:
                        # AC power present - stop SOS, start charge state
                        if self.sos_led_active:
                            self.get_logger().info("AC power detected - stopping SOS, starting charge state")
                            self.sos_led_active = False
                            self.resume_heartbeat(reason="ac_power_detected")
                
                # Update charge state pattern
                if not self.critical_battery_detected:
                    if charge_state_should_be_active and not self.charging_state_active:
                        self.get_logger().info(
                            f"Immediate charge state update: charging={is_charging_now}, "
                            f"fully_charged={is_fully_charged_now}, voltage={battery_voltage:.3f}V, "
                            f"percentage={percentage:.1f}%")
                        self.charging_state_active = True
                        self.pause_heartbeat(reason="charging_state")
                        if not self.charge_state_led_active:
                            threading.Thread(target=self.charge_state_led_pattern, daemon=True).start()
                    elif not charge_state_should_be_active and self.charging_state_active:
                        self.get_logger().info("Immediate charge state update: charging ended")
                        self.charging_state_active = False
                        self.charge_state_led_active = False
                        
                        # CRITICAL: Immediately turn off charge state LED to prevent it staying on
                        # The pattern thread will exit, but we need to turn off LED now to avoid delay
                        led_setter = self._get_charge_state_led_setter()
                        led_setter(False)
                        
                        # When AC power is removed, check if we should show SOS (low battery) or resume heartbeat
                        if battery_voltage < LOW_BATTERY_THRESHOLD_V and not self.critical_battery_detected:
                            # Battery is low - start SOS pattern
                            if not self.sos_led_active:
                                self.get_logger().info(
                                    f"AC power removed with low battery ({battery_voltage:.3f}V) - starting SOS pattern")
                                self.pause_heartbeat(reason="low_battery")
                                threading.Thread(target=self.sos_led_pattern, daemon=True).start()
                        else:
                            # Battery is OK - resume normal heartbeat
                            self.resume_heartbeat(reason="charging_state_ended")
        except Exception as e:
            self.get_logger().error(f"Error in immediate charge state update: {e}")

    def run_threads(self):
        """Start all background monitoring threads."""
        self.get_logger().info("Power controller starting background threads...")
        self.get_logger().info("Button monitoring active")

        # Start power button monitoring in separate thread
        self.get_logger().info("Starting hardware interrupt-based button detection")
        self._button_monitor_thread = threading.Thread(
            target=self.monitor_power_button_interrupts, daemon=True)
        self._button_monitor_thread.start()

        # Start green LED heartbeat in separate thread
        self.get_logger().info("Starting green LED heartbeat thread")
        heartbeat_thread = threading.Thread(
            target=self.green_led_heartbeat, daemon=True)
        heartbeat_thread.start()

        # Set initial LED heartbeat frequency based on current state
        self._update_led_heartbeat_for_controller_mode()

        # Start critical battery monitoring in separate thread
        self.get_logger().info("Starting critical battery monitoring thread")
        battery_thread = threading.Thread(
            target=self.monitor_critical_battery, daemon=True)
        battery_thread.start()

        # Start WiFi connectivity monitoring in separate thread
        self.get_logger().info("Starting WiFi connectivity monitoring thread")
        wifi_thread = threading.Thread(
            target=self.monitor_wifi_connectivity, daemon=True)
        wifi_thread.start()

        # Create ROS2 services for remote control
        self._create_power_services()
        
        # Check initial I2C failure state from battery service (in case we missed the topic message)
        self._check_initial_i2c_failure_state()

        # Publish initial status
        self._publish_power_status("Power control system running")

    def run(self):
        """
        Main control loop - DEPRECATED.
        This logic is now handled by the MultiThreadedExecutor in main().
        This method is kept for potential future use or for non-ROS execution modes.
        """
        self.get_logger().warning("run() method is deprecated. Use executor.spin() in main.")
        # The original while loop is no longer needed here as executor.spin() handles it.
        pass

    def monitor_critical_battery(self):
        """Monitor battery voltage every 30 seconds for critical low voltage"""
        self.get_logger().info("Starting critical battery monitoring thread")
        self.battery_monitoring_active = True

        # Unified timeout for both service failures and invalid readings (I2C failures)
        # Both represent inability to monitor battery voltage, so same timeout applies
        BATTERY_SERVICE_OR_READING_FAILURE_TIMEOUT = 3600.0  # 1 hour = 3600 seconds
        failure_start_time = None  # Track when failures started (service OR reading failure)
        failure_cause = None  # Track cause: "service_failure" or "i2c_failure"
        last_valid_reading_time = None  # Track when last valid reading occurred
        
        # Throttling for failure log messages (1 per minute)
        last_failure_log_time = 0.0
        FAILURE_LOG_INTERVAL = 60.0  # Log every 60 seconds during failures

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
                current_time = time.time()
                
                if battery_data:
                    # Service call succeeded - check if reading is valid
                    battery_voltage = battery_data.get('battery_voltage', 0)
                    charging_status = battery_data.get('charging_status', None)
                    ac_power_present = battery_data.get('ac_power_present', None)
                    # CRITICAL: Extract I2C failure and stale data flags to distinguish I2C failures from actual low voltage
                    i2c_failure = battery_data.get('i2c_failure', False)
                    stale_data = battery_data.get('stale_data', False)
                    storage_rundown = battery_data.get('battery_storage_rundown', False)
                    shutdown_threshold_v = STORAGE_VOLTAGE_V if storage_rundown else CRITICAL_BATTERY_THRESHOLD_V
                    # Prefer topic value if available (faster updates), otherwise use service value
                    ac_power = self.ac_power_from_topic if self.ac_power_from_topic is not None else ac_power_present
                    self.last_battery_voltage = battery_voltage
                    self.get_logger().debug(
                        f"Battery data received: {battery_voltage:.3f}V (charging={charging_status}, ac_power={ac_power_present}, "
                        f"i2c_failure={i2c_failure}, stale_data={stale_data})")

                    # Mark that we've seen the battery service available at least once
                    if not battery_service_ever_available:
                        battery_service_ever_available = True
                        self.get_logger().info("Battery service is now available - critical battery monitoring active")

                    # Check if reading is invalid (I2C failure, stale data, NaN, or 0.0V)
                    is_invalid_reading = (i2c_failure or stale_data or 
                                         math.isnan(battery_voltage) or battery_voltage <= 0.0)
                    
                    if is_invalid_reading:
                        # Invalid reading detected (I2C failure, stale data, NaN, or 0.0V)
                        # Determine specific failure type for logging
                        if i2c_failure:
                            failure_type = "I2C failure"
                        elif stale_data:
                            failure_type = "stale data"
                        elif math.isnan(battery_voltage):
                            failure_type = "NaN reading"
                        else:
                            failure_type = "invalid reading (0.0V)"
                        
                        # Track failure start time and cause
                        if failure_start_time is None:
                            failure_start_time = current_time
                            failure_cause = "i2c_failure"  # Service responded but reading is invalid
                            voltage_str = f"{battery_voltage:.3f}V" if not math.isnan(battery_voltage) else "NaN"
                            self.get_logger().warn(
                                f"Battery monitoring failure detected: {failure_type} ({voltage_str}) - "
                                f"treating as invalid reading. Shutdown after 1h sustained failure.")
                        
                        # Check if failure has persisted for >= timeout
                        if failure_start_time is not None:
                            failure_duration = current_time - failure_start_time
                            
                            # Validate duration is reasonable (detect stale timestamps)
                            # Only reset if duration is negative or impossibly large (> 24 hours)
                            # This allows legitimate failures up to the timeout period
                            MAX_REASONABLE_DURATION = 86400.0  # 24 hours - catches truly invalid timestamps
                            
                            if failure_duration < 0 or failure_duration > MAX_REASONABLE_DURATION:
                                self.get_logger().error(
                                    f"⚠️  Invalid duration calculation: {failure_duration:.1f}s - resetting timer "
                                    f"(expected range: 0-{MAX_REASONABLE_DURATION/3600:.1f}h)")
                                failure_start_time = current_time
                                failure_duration = 0.0
                            
                            # Throttled logging (1 per minute)
                            if current_time - last_failure_log_time >= FAILURE_LOG_INTERVAL:
                                failure_duration_hours = failure_duration / 3600.0
                                time_since_valid_str = ""
                                if last_valid_reading_time is not None:
                                    time_since_valid = current_time - last_valid_reading_time
                                    if time_since_valid < 60:
                                        time_since_valid_str = f" ({time_since_valid:.1f}s since last valid)"
                                    elif time_since_valid < 3600:
                                        time_since_valid_str = f" ({time_since_valid/60:.1f}min since last valid)"
                                    else:
                                        time_since_valid_str = f" ({time_since_valid/3600:.2f}h since last valid)"
                                
                                voltage_str = f"{battery_voltage:.3f}V" if not math.isnan(battery_voltage) else "NaN"
                                self.get_logger().warn(
                                    f"Battery monitoring failure continuing: {failure_type} ({voltage_str}) - "
                                    f"{failure_duration_hours:.2f}h duration{time_since_valid_str} - "
                                    f"shutdown after 1h sustained failure")
                                last_failure_log_time = current_time
                            
                            # Check timeout
                            if failure_duration >= BATTERY_SERVICE_OR_READING_FAILURE_TIMEOUT:
                                failure_duration_hours = failure_duration / 3600.0
                                
                                # Log shutdown cause
                                self.get_logger().error(
                                    f"🔴 CRITICAL: Battery monitoring failure has persisted for {failure_duration_hours:.2f}h")
                                self.get_logger().error(
                                    f"Failure cause: {failure_type} - battery monitoring unavailable")
                                self.get_logger().fatal(
                                    f"⚠️  SAFETY SHUTDOWN: Initiating safe shutdown to prevent battery damage")
                                
                                # Sync logs to ensure persistent storage
                                try:
                                    import os
                                    os.sync()
                                    self.get_logger().info("Filesystem synced - failure log written to persistent storage")
                                except Exception as e:
                                    self.get_logger().error(f"Failed to sync filesystem: {e}")
                                
                                # Broadcast wall message explaining shutdown
                                try:
                                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                    wall_message = (
                                        f"\n"
                                        f"═══════════════════════════════════════════════════════════\n"
                                        f"SYSTEM SHUTDOWN INITIATED: Battery Monitoring Failure\n"
                                        f"═══════════════════════════════════════════════════════════\n"
                                        f"Time: {timestamp}\n"
                                        f"Reason: Battery monitoring unavailable for {failure_duration_hours:.2f} hours\n"
                                        f"Failure cause: {failure_type}\n"
                                        f"\n"
                                        f"System cannot verify battery voltage - initiating safe shutdown\n"
                                        f"to prevent potential battery damage.\n"
                                        f"\n"
                                        f"Check:\n"
                                        f"  - argo_battery_water.service status\n"
                                        f"  - I2C bus connectivity\n"
                                        f"  - Sensor hardware connections\n"
                                        f"═══════════════════════════════════════════════════════════\n"
                                    )
                                    subprocess.run(['wall', wall_message], check=True, timeout=10.0)
                                    self.get_logger().info("Shutdown wall message broadcasted")
                                except Exception as e:
                                    self.get_logger().error(f"Failed to broadcast wall message: {e}")
                                
                                # Initiate shutdown with specific reason
                                self.initiate_critical_battery_halt(
                                    battery_voltage=0.0,
                                    reason=f"battery_monitoring_failure_{failure_cause}_{failure_duration_hours:.1f}h")
                                break  # Exit monitoring loop after initiating shutdown
                        
                        continue  # Skip critical battery check - this is monitoring failure, not low voltage

                    # Valid reading - reset failure tracking, update last valid reading time
                    if failure_start_time is not None:
                        # Failure recovered - log recovery
                        recovery_duration = current_time - failure_start_time
                        recovery_duration_hours = recovery_duration / 3600.0
                        self.get_logger().info(
                            f"Battery monitoring recovered after {recovery_duration_hours:.2f}h - "
                            f"valid reading received: {battery_voltage:.3f}V")
                        failure_start_time = None
                        failure_cause = None
                    last_valid_reading_time = current_time  # Update last valid reading time

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

                        # Handle NaN in logging
                        voltage_str = f"{battery_voltage:.3f}V" if not math.isnan(battery_voltage) else "NaN (I2C failure)"
                        storage_note = " | Storage rundown to 7.6V" if storage_rundown else ""
                        self.get_logger().info(
                            f"Battery voltage check: {voltage_str} "
                            f"(low: {LOW_BATTERY_THRESHOLD_V}V, critical: {CRITICAL_BATTERY_THRESHOLD_V}V){charging_str}{storage_note}")

                        # Update the last logged voltage
                        self.last_logged_battery_voltage = battery_voltage

                    # Check for critical battery first (highest priority)
                    # In storage rundown mode use 7.6V threshold; otherwise 7.2V
                    # Safety check: Skip if NaN (shouldn't happen here, but defensive programming)
                    if not math.isnan(battery_voltage) and battery_voltage < shutdown_threshold_v:
                        # Storage rundown: always shutdown at 7.6V (no AC bypass) so test with supply works
                        if storage_rundown:
                            if not self.critical_battery_detected:
                                self.get_logger().info(
                                    f"Storage rundown: voltage {battery_voltage:.3f}V reached {STORAGE_VOLTAGE_V}V target; initiating shutdown.")
                                self.critical_battery_detected = True
                                self.pause_heartbeat(reason="storage_rundown")
                                if self.sos_led_active:
                                    self.sos_led_active = False
                                if self.i2c_failure_sos_active:
                                    self.i2c_failure_sos_active = False
                                if self.charge_state_led_active:
                                    self.charge_state_led_active = False
                                    self.charging_state_active = False
                                    led_setter = self._get_charge_state_led_setter()
                                    led_setter(False)
                                self.initiate_critical_battery_halt(battery_voltage, reason="storage_rundown_7.6V")
                            continue
                        # CRITICAL FIX: If AC power is present and charging is active, don't trigger shutdown
                        # The battery is being charged and will recover - shutdown is not needed
                        # Use topic value (ac_power) for faster updates, fallback to service value
                        is_in_charging_state = (charging_status is True) or (ac_power is True)
                        
                        # Check if AC power plug-in time has exceeded timeout (charger may have timed out)
                        ac_power_timed_out, plug_in_time_str = self._check_ac_power_timeout()
                        
                        if is_in_charging_state and not ac_power_timed_out:
                            # AC power present and charging active - battery will recover, don't shutdown
                            if self.critical_battery_detected:
                                # Critical battery was previously detected but AC power is now present - clear the flag
                                charging_str = ""
                                if charging_status is not None or ac_power_present is not None:
                                    charging_parts = []
                                    if ac_power_present is not None:
                                        charging_parts.append(f"AC Power: {'YES' if ac_power_present else 'NO'}")
                                    if charging_status is not None:
                                        charging_parts.append(f"Charging: {'ACTIVE' if charging_status else 'INACTIVE'}")
                                    charging_str = f", {', '.join(charging_parts)}"
                                
                                self.get_logger().info(
                                    f"CRITICAL BATTERY FLAG CLEARED - AC POWER DETECTED: {battery_voltage:.3f}V < {CRITICAL_BATTERY_THRESHOLD_V}V{charging_str}")
                                self.get_logger().info(
                                    "Battery is being charged - critical battery shutdown cancelled. Voltage will recover during charging.")
                                self.critical_battery_detected = False
                                self._clear_critical_battery_flag()
                                # Resume heartbeat - let charge state pattern handle LED indication
                                self.resume_heartbeat(reason="ac_power_detected_during_critical")
                            elif not self.critical_battery_detected:
                                # First time detecting critical voltage with AC power present
                                charging_str = ""
                                if charging_status is not None or ac_power_present is not None:
                                    charging_parts = []
                                    if ac_power_present is not None:
                                        charging_parts.append(f"AC Power: {'YES' if ac_power_present else 'NO'}")
                                    if charging_status is not None:
                                        charging_parts.append(f"Charging: {'ACTIVE' if charging_status else 'INACTIVE'}")
                                    charging_str = f", {', '.join(charging_parts)}"
                                
                                self.get_logger().warning(
                                    f"CRITICAL BATTERY VOLTAGE DETECTED but AC POWER PRESENT: {battery_voltage:.3f}V < {CRITICAL_BATTERY_THRESHOLD_V}V{charging_str}")
                                self.get_logger().warning(
                                    "Battery is being charged - shutdown cancelled. Voltage will recover during charging.")
                                # Don't set critical_battery_detected flag - allow normal operation to continue
                                # Don't pause heartbeat - let charge state pattern handle LED indication
                                # Don't stop charge state pattern - it should continue showing charging status
                            # Continue monitoring - voltage should increase during charging
                        elif is_in_charging_state and ac_power_timed_out:
                            # AC power is present but charger has timed out (> 20h) - trigger shutdown
                            if not self.critical_battery_detected:
                                charging_str = ""
                                if charging_status is not None or ac_power_present is not None:
                                    charging_parts = []
                                    if ac_power_present is not None:
                                        charging_parts.append(f"AC Power: {'YES' if ac_power_present else 'NO'}")
                                    if charging_status is not None:
                                        charging_parts.append(f"Charging: {'ACTIVE' if charging_status else 'INACTIVE'}")
                                    charging_str = f", {', '.join(charging_parts)}"
                                
                                hours_ago = (time.time() - datetime.fromisoformat(plug_in_time_str).timestamp()) / 3600.0 if plug_in_time_str else None
                                hours_str = f" ({hours_ago:.1f}h ago)" if hours_ago else ""
                                
                                self.get_logger().error(
                                    f"CRITICAL BATTERY DETECTED: {battery_voltage:.3f}V < {CRITICAL_BATTERY_THRESHOLD_V}V{charging_str}")
                                self.get_logger().error(
                                    f"AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h) - "
                                    f"charger may have timed out. Shutdown required to allow user to replug charger.")
                                self.critical_battery_detected = True
                                # Pause heartbeat for critical battery
                                self.pause_heartbeat(reason="critical_battery")
                                # Stop SOS pattern if running (critical takes priority)
                                if self.sos_led_active:
                                    self.sos_led_active = False
                                # Stop I2C failure SOS pattern if running (critical takes priority)
                                if self.i2c_failure_sos_active:
                                    self.i2c_failure_sos_active = False
                                # Stop charge state pattern if running (critical takes priority)
                                if self.charge_state_led_active:
                                    self.charge_state_led_active = False
                                    self.charging_state_active = False
                                    # Immediately turn off charge state LED
                                    led_setter = self._get_charge_state_led_setter()
                                    led_setter(False)
                                self.initiate_critical_battery_halt(
                                    battery_voltage, 
                                    reason=f"ac_power_timeout_{hours_ago:.1f}h" if hours_ago else "ac_power_timeout")
                        else:
                            # No AC power or not charging - critical battery shutdown is needed
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

                                self.get_logger().error(
                                    f"CRITICAL BATTERY DETECTED: {battery_voltage:.3f}V < {CRITICAL_BATTERY_THRESHOLD_V}V{charging_str}")
                                self.critical_battery_detected = True
                                # Pause heartbeat for critical battery
                                self.pause_heartbeat(reason="critical_battery")
                                # Stop SOS pattern if running (critical takes priority)
                                if self.sos_led_active:
                                    self.sos_led_active = False
                                # Stop I2C failure SOS pattern if running (critical takes priority)
                                if self.i2c_failure_sos_active:
                                    self.i2c_failure_sos_active = False
                                # Stop charge state pattern if running (critical takes priority)
                                if self.charge_state_led_active:
                                    self.charge_state_led_active = False
                                    self.charging_state_active = False
                                    # Immediately turn off charge state LED
                                    led_setter = self._get_charge_state_led_setter()
                                    led_setter(False)
                                self.initiate_critical_battery_halt(battery_voltage)

                    # Check for low battery (SOS warning)
                    # SOS pattern only shows when AC power is NOT present
                    elif battery_voltage < LOW_BATTERY_THRESHOLD_V:
                        if not self.low_battery_detected:
                            # Always log low battery events regardless of voltage change threshold
                            self.get_logger().warning(
                                f"LOW BATTERY DETECTED: {battery_voltage:.3f}V < {LOW_BATTERY_THRESHOLD_V}V")
                            self.low_battery_detected = True
                        
                        # Only show SOS pattern if AC power is NOT present
                        # If AC power is present, charge state pattern will be shown instead
                        if ac_power is not True:
                            # No AC power - show SOS warning
                            if not self.sos_led_active:
                                self.get_logger().warning("Starting SOS LED pattern (no AC power)")
                                # Pause heartbeat to make SOS pattern visible
                                self.pause_heartbeat(reason="low_battery")
                                # Stop charge state pattern if running (SOS takes priority when no AC)
                                if self.charge_state_led_active:
                                    self.charge_state_led_active = False
                                    self.charging_state_active = False
                                    # Immediately turn off charge state LED
                                    led_setter = self._get_charge_state_led_setter()
                                    led_setter(False)
                                # Start SOS LED pattern in separate thread
                                threading.Thread(target=self.sos_led_pattern, daemon=True).start()
                                # Send desktop notification
                                self.send_desktop_notification(
                                    "Low Battery Warning",
                                    f"Battery voltage low: {battery_voltage:.3f}V\nSOS LED pattern activated\nReturn to shore or charge battery",
                                    "critical"
                                )
                        else:
                            # AC power is present - stop SOS and let charge state pattern handle it
                            if self.sos_led_active:
                                self.get_logger().info("AC power detected - stopping SOS pattern, showing charge state instead")
                                self.sos_led_active = False
                                # Resume heartbeat so charge state pattern can pause it
                                self.resume_heartbeat(reason="ac_power_detected")
                            # Charge state pattern will be started/updated by the charge state check below

                    # Check for charging state (only if not in critical battery state)
                    # Charging state takes precedence over normal heartbeat and SOS (when AC power is present)
                    # Charge state can run even when low_battery_detected is True (if AC power is present)
                    if not self.critical_battery_detected:
                        # Determine if charging or fully charged
                        # Consider in charging state if: charging_status is True OR ac_power is True
                        # Fully charged if voltage is >= 98% of max voltage (regardless of charging_status)
                        # Use topic value (ac_power) for faster updates
                        is_in_charging_state = (charging_status is True) or (ac_power is True)
                        is_fully_charged_now = battery_voltage >= (BATTERY_VOLTAGE_MAX_V * 0.98)  # ~98% of max = fully charged
                        is_charging_now = is_in_charging_state and not is_fully_charged_now
                        
                        # Calculate battery percentage
                        percentage = self._calculate_battery_percentage(battery_voltage)
                        self.current_charge_percentage = percentage
                        self.is_charging = is_charging_now
                        self.is_fully_charged = is_fully_charged_now
                        
                        # Determine if charge state should be active
                        # CRITICAL: Charge state pattern should ONLY be active when AC power is present
                        # Even if battery is fully charged, don't show charge state pattern without AC power
                        charge_state_should_be_active = (ac_power is True) and (is_charging_now or is_fully_charged_now)
                        
                        # Start charge state pattern if needed
                        if charge_state_should_be_active and not self.charging_state_active:
                            self.get_logger().info(
                                f"Charging state detected: charging={is_charging_now}, fully_charged={is_fully_charged_now}, "
                                f"voltage={battery_voltage:.3f}V, percentage={percentage:.1f}%")
                            self.charging_state_active = True
                            # Pause heartbeat to make charge state pattern visible
                            self.pause_heartbeat(reason="charging_state")
                            # Start charge state LED pattern in separate thread
                            if not self.charge_state_led_active:
                                threading.Thread(target=self.charge_state_led_pattern, daemon=True).start()
                        
                        # Update charge state pattern if already active
                        if self.charging_state_active:
                            # Periodic logging for debugging
                            current_time = time.time()
                            if current_time - self.last_charge_state_log_time >= CHARGE_STATE_LOG_INTERVAL_S:
                                # If battery service reports fully charged, show steady ON in log
                                if is_fully_charged_now or percentage >= 100.0:
                                    pattern_desc = "steady ON (fully charged)"
                                else:
                                    # Duty cycle directly represents charge percentage
                                    duty_cycle = percentage / 100.0
                                    on_time = CHARGE_STATE_PERIOD_S * duty_cycle
                                    pattern_desc = f"{percentage:.1f}% charge = {duty_cycle*100:.1f}% duty cycle ({on_time*1000:.0f}ms on, {CHARGE_STATE_PERIOD_S*1000 - on_time*1000:.0f}ms off)"
                                
                                self.get_logger().info(
                                    f"Charge state pattern: {pattern_desc}, "
                                    f"voltage={battery_voltage:.3f}V, "
                                    f"charging={is_charging_now}, "
                                    f"fully_charged={is_fully_charged_now}, "
                                    f"LED={CHARGE_STATE_LED}")
                                self.last_charge_state_log_time = current_time
                        
                        # Stop charge state pattern if no longer charging/charged
                        if not charge_state_should_be_active and self.charging_state_active:
                            self.get_logger().info(
                                f"Charging state ended: voltage={battery_voltage:.3f}V, percentage={percentage:.1f}%")
                            self.charging_state_active = False
                            self.charge_state_led_active = False  # This will stop the pattern thread
                            # Immediately turn off charge state LED
                            led_setter = self._get_charge_state_led_setter()
                            led_setter(False)
                            # Resume heartbeat now that charging state is inactive
                            self.resume_heartbeat(reason="charging_state_ended")

                    # Battery voltage recovered above low threshold (run whenever voltage is OK, not only when critical was set)
                    if battery_voltage >= LOW_BATTERY_THRESHOLD_V:
                        # Clear low battery state and stop SOS if we were in it
                        if self.low_battery_detected:
                            # Always log battery recovery events regardless of voltage change threshold
                            self.get_logger().info(
                                f"Battery voltage recovered from low: {battery_voltage:.3f}V >= {LOW_BATTERY_THRESHOLD_V}V")
                            self.low_battery_detected = False
                            self.sos_led_active = False  # Stop SOS pattern
                            # Resume heartbeat now that battery is stable
                            self.resume_heartbeat(reason="low_battery_recovered")
                            self.send_desktop_notification(
                                "Battery Recovered",
                                f"Battery voltage recovered: {battery_voltage:.3f}V\nSOS LED pattern stopped",
                                "normal"
                            )

                        # Clear critical battery state if we were in it
                        if self.critical_battery_detected:
                            # Always log critical battery recovery events regardless of voltage change threshold
                            self.get_logger().info(
                                f"Battery voltage recovered from critical: {battery_voltage:.3f}V >= {CRITICAL_BATTERY_THRESHOLD_V}V")
                            self.critical_battery_detected = False
                            self._clear_critical_battery_flag()

                    # Log debug message when voltage change is too small to log normally
                    if not should_log_voltage and self.last_logged_battery_voltage is not None:
                        voltage_change = abs(battery_voltage - self.last_logged_battery_voltage)
                        self.get_logger().debug(
                            f"Battery voltage change too small to log: {voltage_change*1000:.1f}mV < {BATTERY_LOG_THRESHOLD_V*1000:.0f}mV "
                            f"(current: {battery_voltage:.3f}V, last logged: {self.last_logged_battery_voltage:.3f}V)")
                else:
                    # Service unavailable - handle based on startup state
                    current_time = time.time()
                    time_since_startup = current_time - startup_time

                    if not battery_service_ever_available and time_since_startup < STARTUP_GRACE_PERIOD_S:
                        # During startup grace period and service never seen - don't count failures yet
                        self.get_logger().info(
                            f"Battery service not available yet - waiting for argo_battery_water.service startup "
                            f"(grace period: {time_since_startup:.0f}s / {STARTUP_GRACE_PERIOD_S:.0f}s)")
                    elif not battery_service_ever_available and time_since_startup >= STARTUP_GRACE_PERIOD_S:
                        # Grace period expired but service never became available - log error but DO NOT HALT
                        self.get_logger().error(
                            f"CRITICAL: Battery service never became available after {STARTUP_GRACE_PERIOD_S}s grace period!")
                        self.get_logger().error(
                            "Battery monitoring is DISABLED - system will continue WITHOUT battery protection!")
                        self.get_logger().error(
                            "Check if argo_battery_water.service is installed and enabled:")
                        self.get_logger().error("  sudo systemctl status argo_battery_water.service")
                        self.get_logger().error(
                            "System will NOT halt - service unavailability is not a battery problem!")
                        # DO NOT HALT - missing service is a configuration problem, not a battery emergency
                    else:
                        # Service was available before but now failing - track failure start time
                        if failure_start_time is None:
                            failure_start_time = current_time
                            failure_cause = "service_failure"  # Service call failed
                            # Format timestamp for "since when"
                            from datetime import datetime
                            failure_start_str = datetime.fromtimestamp(current_time).strftime("%Y-%m-%d %H:%M:%S")
                            time_since_valid_str = ""
                            if last_valid_reading_time is not None:
                                time_since_valid = current_time - last_valid_reading_time
                                if time_since_valid < 60:
                                    time_since_valid_str = f" (last valid reading {time_since_valid:.1f}s ago)"
                                elif time_since_valid < 3600:
                                    time_since_valid_str = f" (last valid reading {time_since_valid/60:.1f}min ago)"
                                else:
                                    time_since_valid_str = f" (last valid reading {time_since_valid/3600:.2f}h ago)"
                            self.get_logger().error(
                                f"Battery service failure detected - service was available but now unreachable "
                                f"since {failure_start_str}{time_since_valid_str}")
                        else:
                            # Failure already tracked - check if duration is reasonable (detect stale timestamps)
                            # Only reset if duration is negative or impossibly large (> 24 hours)
                            # This allows legitimate failures up to the timeout period
                            failure_duration = current_time - failure_start_time
                            MAX_REASONABLE_DURATION = 86400.0  # 24 hours - catches truly invalid timestamps
                            
                            if failure_duration < 0 or failure_duration > MAX_REASONABLE_DURATION:
                                self.get_logger().error(
                                    f"⚠️  Invalid duration calculation: {failure_duration:.1f}s - resetting timer "
                                    f"(expected range: 0-{MAX_REASONABLE_DURATION/3600:.1f}h)")
                                failure_start_time = current_time
                                failure_duration = 0.0
                        
                        # Check if failure has persisted for >= timeout
                        if failure_start_time is not None:
                            failure_duration = current_time - failure_start_time
                            
                            # Throttled logging (1 per minute)
                            if current_time - last_failure_log_time >= FAILURE_LOG_INTERVAL:
                                failure_duration_hours = failure_duration / 3600.0
                                failure_duration_min = failure_duration / 60.0
                                failure_duration_sec = failure_duration
                                
                                # Format duration string
                                if failure_duration < 60:
                                    duration_str = f"{failure_duration_sec:.0f}s"
                                elif failure_duration < 3600:
                                    duration_str = f"{failure_duration_min:.1f}min"
                                else:
                                    duration_str = f"{failure_duration_hours:.2f}h"
                                
                                # Format "since when" timestamp
                                from datetime import datetime
                                failure_start_str = datetime.fromtimestamp(failure_start_time).strftime("%H:%M:%S")
                                
                                time_since_valid_str = ""
                                if last_valid_reading_time is not None:
                                    time_since_valid = current_time - last_valid_reading_time
                                    if time_since_valid < 60:
                                        time_since_valid_str = f" (last valid reading {time_since_valid:.1f}s ago)"
                                    elif time_since_valid < 3600:
                                        time_since_valid_str = f" (last valid reading {time_since_valid/60:.1f}min ago)"
                                    else:
                                        time_since_valid_str = f" (last valid reading {time_since_valid/3600:.2f}h ago)"
                                
                                self.get_logger().error(
                                    f"Battery service unavailable - unreachable for {duration_str} "
                                    f"(since {failure_start_str}){time_since_valid_str} - shutdown after 1h sustained failure")
                                last_failure_log_time = current_time
                            
                            # Check timeout
                            if failure_duration >= BATTERY_SERVICE_OR_READING_FAILURE_TIMEOUT:
                                failure_duration_hours = failure_duration / 3600.0
                                
                                # Log shutdown cause
                                self.get_logger().error(
                                    f"🔴 CRITICAL: Battery service has been failing for {failure_duration_hours:.2f}h")
                                self.get_logger().error(
                                    f"Failure cause: service unavailable - battery monitoring unavailable")
                                self.get_logger().fatal(
                                    f"⚠️  SAFETY SHUTDOWN: Initiating safe shutdown to prevent battery damage")
                                self.get_logger().error(
                                    f"Check: journalctl -u argo_battery_water.service -n 50")
                                
                                # Sync logs to ensure persistent storage
                                try:
                                    import os
                                    os.sync()
                                    self.get_logger().info("Filesystem synced - failure log written to persistent storage")
                                except Exception as e:
                                    self.get_logger().error(f"Failed to sync filesystem: {e}")
                                
                                # Broadcast wall message explaining shutdown
                                try:
                                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                    wall_message = (
                                        f"\n"
                                        f"═══════════════════════════════════════════════════════════\n"
                                        f"SYSTEM SHUTDOWN INITIATED: Battery Service Failure\n"
                                        f"═══════════════════════════════════════════════════════════\n"
                                        f"Time: {timestamp}\n"
                                        f"Reason: Battery service unavailable for {failure_duration_hours:.2f} hours\n"
                                        f"Failure cause: Service unreachable (service call failed)\n"
                                        f"\n"
                                        f"System cannot verify battery voltage - initiating safe shutdown\n"
                                        f"to prevent potential battery damage.\n"
                                        f"\n"
                                        f"Check:\n"
                                        f"  - argo_battery_water.service status\n"
                                        f"  - Service logs: journalctl -u argo_battery_water.service -n 50\n"
                                        f"  - ROS2 service discovery\n"
                                        f"═══════════════════════════════════════════════════════════\n"
                                    )
                                    subprocess.run(['wall', wall_message], check=True, timeout=10.0)
                                    self.get_logger().info("Shutdown wall message broadcasted")
                                except Exception as e:
                                    self.get_logger().error(f"Failed to broadcast wall message: {e}")
                                
                                # Initiate shutdown with specific reason
                                self.initiate_critical_battery_halt(
                                    battery_voltage=0.0,
                                    reason=f"battery_monitoring_failure_{failure_cause}_{failure_duration_hours:.1f}h")
                                break  # Exit monitoring loop after initiating shutdown

                self.last_battery_check_time = time.time()

            except Exception as e:
                self.get_logger().error(f"Error in critical battery monitoring: {e}")

            # Sleep for monitoring interval
            sleep_time = 0
            while sleep_time < BATTERY_MONITORING_INTERVAL_S and self.running and self.battery_monitoring_active:
                time.sleep(1.0)
                sleep_time += 1.0

        self.get_logger().info("Critical battery monitoring thread stopped")

    def monitor_wifi_connectivity(self):
        """Monitor WiFi connectivity every 10 seconds and update network_healthy status"""
        self.get_logger().info("Starting WiFi connectivity monitoring thread")
        self.wifi_monitoring_active = True

        # Track consecutive failures for stability
        consecutive_failures = 0
        consecutive_successes = 0
        MAX_CONSECUTIVE_FAILURES = 2  # Require 2 consecutive failures before declaring WiFi lost
        MAX_CONSECUTIVE_SUCCESSES = 2  # Require 2 consecutive successes before declaring WiFi restored

        while self.running and self.wifi_monitoring_active:
            try:
                # Check WiFi connectivity with timeout wrapper to prevent hangs
                try:
                    with ThreadPoolExecutor(max_workers=1) as executor:
                        future = executor.submit(self.check_wifi_connectivity)
                        wifi_connected = future.result(timeout=WIFI_CHECK_TIMEOUT_S)
                except FutureTimeoutError:
                    self.get_logger().warning(
                        f"WiFi connectivity check timed out after {WIFI_CHECK_TIMEOUT_S}s in monitoring thread")
                    wifi_connected = False
                except Exception as e:
                    self.get_logger().error(f"Error in WiFi connectivity check: {e}")
                    wifi_connected = False

                if wifi_connected:
                    consecutive_successes += 1
                    consecutive_failures = 0

                    # Check if WiFi was previously lost and now restored
                    if not self.network_healthy and consecutive_successes >= MAX_CONSECUTIVE_SUCCESSES:
                        self.get_logger().info("WiFi connectivity restored")
                        self.network_healthy = True
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
                    if self.network_healthy and consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                        self.get_logger().warning("WiFi connectivity lost")
                        self.network_healthy = False
                        # Send notification
                        self.send_desktop_notification(
                            "WiFi Connection Lost",
                            "WiFi connectivity has been lost",
                            "critical"
                        )
                        consecutive_failures = 0  # Reset counter

                self.last_wifi_check_time = time.time()

            except Exception as e:
                self.get_logger().error(f"Error in WiFi connectivity monitoring: {e}")

            # Sleep for monitoring interval
            sleep_time = 0
            while sleep_time < WIFI_MONITORING_INTERVAL_S and self.running and self.wifi_monitoring_active:
                time.sleep(1.0)
                sleep_time += 1.0

        self.get_logger().info("WiFi connectivity monitoring thread stopped")

    def _check_ac_power_timeout(self) -> tuple[bool, Optional[str]]:
        """Check if AC power plug-in time is older than AC_POWER_TIMEOUT_HOURS
        
        Returns:
            tuple: (is_timed_out: bool, plug_in_time_str: Optional[str])
                - is_timed_out: True if AC power was plugged in more than AC_POWER_TIMEOUT_HOURS ago
                - plug_in_time_str: ISO format timestamp string of when AC power was plugged in, or None if not found
        """
        try:
            if not BATTERY_SLOPES_JSON_PATH.exists():
                self.get_logger().debug(f"Battery slopes file not found: {BATTERY_SLOPES_JSON_PATH}")
                return False, None
            
            with open(BATTERY_SLOPES_JSON_PATH, 'r') as f:
                slopes_data = json.load(f)
            
            plug_in_time_str = slopes_data.get('ac_power_plug_in_time')
            if not plug_in_time_str:
                self.get_logger().debug("No ac_power_plug_in_time found in battery_slopes.json")
                return False, None
            
            # Parse ISO format timestamp
            plug_in_time = datetime.fromisoformat(plug_in_time_str)
            time_since_plug_in = time.time() - plug_in_time.timestamp()
            
            is_timed_out = time_since_plug_in > AC_POWER_TIMEOUT_S
            
            if is_timed_out:
                hours_ago = time_since_plug_in / 3600.0
                self.get_logger().warning(
                    f"AC power plug-in time exceeded timeout: {plug_in_time_str} "
                    f"({hours_ago:.1f} hours ago, timeout: {AC_POWER_TIMEOUT_HOURS}h)")
            
            return is_timed_out, plug_in_time_str
            
        except (json.JSONDecodeError, KeyError, ValueError, OSError) as e:
            self.get_logger().debug(f"Error checking AC power timeout: {e}")
            return False, None

    def _call_battery_service(self) -> Optional[Dict[str, Any]]:
        """Call argo_battery_water service to get current battery data using rclpy"""
        success, message = self._call_trigger_service(
            '/battery_status', timeout_sec=5.0)

        if success:
            try:
                response_data = json.loads(message)
                # Extract raw_data from the response, which is what the caller expects
                raw_data = response_data.get('raw_data', {})
                self.get_logger().debug(
                    f"Successfully parsed battery data: {raw_data.get('battery_voltage', 'N/A')}V")
                return raw_data
            except (json.JSONDecodeError, KeyError) as e:
                self.get_logger().error(f"Error parsing battery service response: {e}")
                return None
        else:
            # Error message already logged by _call_trigger_service, no need to duplicate
            return None

    def check_wifi_connectivity(self) -> bool:
        """Check WiFi connectivity using multiple methods

        Returns:
            True if WiFi is connected and functional (has IP address)
            False if WiFi is disconnected or no valid connection
        """
        try:
            # Method 1: Check if WiFi interface is up and has an IP address
            result = subprocess.run(
                ['ip', 'addr', 'show', 'wlan0'],
                capture_output=True, text=True, timeout=WIFI_CONNECTIVITY_TIMEOUT_S
            )

            if result.returncode != 0:
                self.get_logger().debug("WiFi interface wlan0 not found or not accessible")
                return False

            # Check if wlan0 has an IP address (not 127.0.0.1)
            if 'inet ' in result.stdout and '127.0.0.1' not in result.stdout:
                # Extract the connection name to check if it's a known functional network
                nm_result = subprocess.run(
                    ['nmcli', '-t', '-f', 'NAME', 'connection', 'show', '--active'],
                    capture_output=True, text=True, timeout=WIFI_CONNECTIVITY_TIMEOUT_S
                )

                if nm_result.returncode == 0:
                    active_connection = nm_result.stdout.strip()
                    self.get_logger().debug(f"Active WiFi connection: {active_connection}")

                    # For networks without internet access (like uzh-iot), check DNS resolution instead
                    if active_connection == 'uzh-iot':
                        # Test DNS resolution instead of internet ping for uzh-iot
                        dns_result = subprocess.run(
                            ['nslookup', 'google.com'],
                            capture_output=True, text=True, timeout=WIFI_CONNECTIVITY_TIMEOUT_S
                        )
                        if dns_result.returncode == 0:
                            self.get_logger().debug("WiFi connectivity confirmed: uzh-iot connected with DNS access")
                            return True
                        else:
                            self.get_logger().debug("WiFi interface up but uzh-iot DNS not working")
                            return False
                    else:
                        # For other networks (like tobi-wlan), test internet connectivity
                        ping_result = subprocess.run(
                            ['ping', '-c', '1', '-W', '3', '8.8.8.8'],
                            capture_output=True, text=True, timeout=WIFI_CONNECTIVITY_TIMEOUT_S
                        )

                        if ping_result.returncode == 0:
                            self.get_logger().debug("WiFi connectivity confirmed: interface up and internet accessible")
                            return True
                        else:
                            self.get_logger().debug("WiFi interface up but no internet connectivity")
                            return False
                else:
                    self.get_logger().debug("Could not determine active connection name")
                    return False
            else:
                self.get_logger().debug("WiFi interface wlan0 has no valid IP address")
                return False

        except subprocess.TimeoutExpired:
            self.get_logger().debug("WiFi connectivity check timed out")
            return False
        except FileNotFoundError:
            self.get_logger().debug("Network tools not available for WiFi check")
            return False
        except Exception as e:
            self.get_logger().debug(f"Error checking WiFi connectivity: {e}")
            return False

    def _check_network_status(self):
        """Check network status periodically (only one that needs active checking)
        
        Wrapped with timeout to prevent hangs in the LED heartbeat loop.
        """
        import time
        current_time = time.time()
        if current_time - self.last_network_check < 5.0:  # Check every 5 seconds
            return
        
        self.last_network_check = current_time
        old_healthy = self.network_healthy
        
        # Wrap WiFi check in timeout to prevent hangs in LED heartbeat loop
        try:
            with ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(self.check_wifi_connectivity)
                self.network_healthy = future.result(timeout=WIFI_CHECK_TIMEOUT_S)
        except FutureTimeoutError:
            self.get_logger().warning(
                f"WiFi connectivity check timed out after {WIFI_CHECK_TIMEOUT_S}s - assuming UNHEALTHY")
            self.network_healthy = False
        except Exception as e:
            self.get_logger().error(f"Error checking network status: {e}")
            self.network_healthy = False
        
        if old_healthy != self.network_healthy:
            self.get_logger().info(f"Network health changed: {'HEALTHY' if self.network_healthy else 'UNHEALTHY'}")

    def _check_status_timeouts(self):
        """Check if any status updates have timed out"""
        import time
        current_time = time.time()
        
        # Check if any status hasn't been updated recently
        if current_time - self.last_anemometer_update > self.status_timeout:
            self.anemometer_healthy = False
        if current_time - self.last_gps_update > self.status_timeout:
            self.gps_healthy = False
        if current_time - self.last_lora_update > self.status_timeout:
            self.lora_healthy = False

    def _query_health_monitor(self):
        """Query health monitor service for comprehensive node health status"""
        import time
        import json
        current_time = time.time()
        
        # Throttle queries to avoid excessive service calls
        if current_time - self.last_health_monitor_query < self.health_monitor_query_interval:
            return
        
        # Only query if ROS2 is available and health monitor client exists
        if not rclpy.ok() or not hasattr(self, 'health_monitor_client') or not self.health_monitor_client:
            return
        
        try:
            # Check if service is available
            if not self.health_monitor_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().debug("Health monitor service not available")
                return
            
            # Make service call
            request = Trigger.Request()
            future = self.health_monitor_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    try:
                        health_data = json.loads(response.message)
                        nodes_data = health_data.get('nodes', {})
                        
                        # Count unhealthy nodes and capture their names for logging
                        unhealthy_count = 0
                        unhealthy_nodes = []
                        for node_name, node_info in nodes_data.items():
                            health_status = node_info.get('healthy')
                            if health_status is False:
                                unhealthy_count += 1
                                unhealthy_nodes.append(
                                    self._format_health_monitor_node_name(node_name, node_info)
                                )
                        
                        self.health_monitor_unhealthy_count = unhealthy_count
                        self.health_monitor_unhealthy_nodes = unhealthy_nodes
                        self.last_health_monitor_query = current_time
                        self.get_logger().debug(
                            f"Health monitor query: {unhealthy_count} unhealthy nodes"
                            f"{' (' + ', '.join(unhealthy_nodes) + ')' if unhealthy_nodes else ''}"
                        )
                    except (json.JSONDecodeError, KeyError) as e:
                        self.get_logger().debug(f"Error parsing health monitor response: {e}")
        except Exception as e:
            self.get_logger().debug(f"Error querying health monitor: {e}")
    
    def _format_health_monitor_node_name(self, node_key: str, node_info: dict) -> str:
        """Format node name from health monitor data for human-readable output."""
        candidate = node_info.get('display_name')
        if candidate:
            return candidate
        
        normalized = os.path.basename(node_key.strip())
        if normalized.endswith('.py'):
            normalized = normalized[:-3]
        return normalized
    
    def get_failure_count(self) -> int:
        """Get the number of system failures for LED indication"""
        # Query health monitor for comprehensive health status
        self._query_health_monitor()
        
        # Use health monitor count if we've successfully queried it (even if count is 0)
        # This ensures we use comprehensive health status when available
        if self.last_health_monitor_query > 0:
            return self.health_monitor_unhealthy_count
        
        # Fallback to local health checks (for backward compatibility when health monitor unavailable)
        # Check for timeouts first
        self._check_status_timeouts()
        
        failures = 0
        if not self.anemometer_healthy:
            failures += 1
        if not self.network_healthy:
            failures += 1
        if not self.gps_healthy:
            failures += 1
        if not self.lora_healthy:
            failures += 1
        return failures

    def show_critical_battery_confirmation_dialog(self, battery_voltage, reason=None):
        """Show desktop confirmation dialog for critical battery shutdown

        Args:
            battery_voltage: Battery voltage (for display)
            reason: Optional reason string (e.g., "ac_power_timeout_20.5h")

        Returns:
            True: User actively CANCELLED the halt (clicked Cancel button)
            False: Proceed with halt (user clicked OK, or timeout occurred, or dialog failed)

        CRITICAL: Timeout or dialog failure = return False (proceed with halt for safety)
        """
        # Check if this is an AC power timeout situation
        is_ac_power_timeout = reason and reason.startswith("ac_power_timeout")
        hours_ago = None
        if is_ac_power_timeout and "_" in reason:
            try:
                # Extract hours from reason string like "ac_power_timeout_20.5h"
                hours_str = reason.split("_")[-1].rstrip("h")
                hours_ago = float(hours_str)
            except (ValueError, IndexError):
                pass
        
        # Format voltage string
        if battery_voltage > 0:
            voltage_str = f"{battery_voltage:.3f}V"
        else:
            voltage_str = "UNKNOWN"
        
        try:
            # Set up environment for desktop dialogs
            env = os.environ.copy()
            if self.cached_display_env:
                env['DISPLAY'] = self.cached_display_env
            else:
                # Fallback if display detection failed
                env['DISPLAY'] = ':0'

            # Customize dialog message based on reason
            if is_ac_power_timeout and hours_ago:
                dialog_title = "AC POWER TIMEOUT - CRITICAL BATTERY"
                dialog_text = (
                    f"Battery voltage critically low: {voltage_str}\n\n"
                    f"⚠️ AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h)\n"
                    f"Charger chip may have timed out charging or supplemental power.\n"
                    f"System will shutdown in 30 seconds to allow you to replug charger.\n\n"
                    f"⚠️ TIMEOUT = AUTOMATIC SHUTDOWN (safe default)\n\n"
                    f"Click 'Cancel' ONLY if you can replug charger NOW.\n"
                    f"Click 'Shutdown Now' to shutdown immediately.\n"
                    f"No action = automatic shutdown after 30 seconds.\n\n"
                    f"To abort from CLI: pkill -f zenity")
            else:
                dialog_title = "CRITICAL BATTERY SHUTDOWN"
                dialog_text = (
                    f"Battery voltage critically low: {voltage_str}\n\n"
                    f"System will halt in 30 seconds to preserve power for manual sailing.\n\n"
                    f"⚠️ TIMEOUT = AUTOMATIC HALT (safe default)\n\n"
                    f"Click 'Cancel' ONLY if you can plug in charger NOW.\n"
                    f"Click 'Shutdown Now' to halt immediately.\n"
                    f"No action = automatic halt after 30 seconds.\n\n"
                    f"To abort from CLI: pkill -f zenity")

            # Try to show a zenity dialog first (most reliable on desktop)
            dialog_cmd = [
                'zenity',
                '--question',
                f'--title={dialog_title}',
                f'--text={dialog_text}',
                '--ok-label=Shutdown Now',
                '--cancel-label=Cancel (I will plug in charger)',
                '--timeout=30',
                '--width=600',
                '--height=350'
            ]

            self.get_logger().info("Showing critical battery confirmation dialog...")
            result = subprocess.run(
                dialog_cmd, capture_output=True, text=True, timeout=60, env=env)

            if result.returncode == 0:
                # User clicked "Shutdown Now" - proceed with halt
                self.get_logger().info("User confirmed critical battery shutdown")
                return False  # False = proceed with halt
            elif result.returncode == 1:
                # Check if this was a real cancel or a display error
                if "cannot open display" in result.stderr.lower():
                    self.get_logger().error(
                        f"Zenity failed to open display: {result.stderr.strip()}")
                    self.get_logger().error(
                        "Proceeding with halt as a safety measure.")
                    return False  # Treat display error as timeout/confirm
                else:
                    # User clicked "Cancel Shutdown" - user intervention
                    self.get_logger().warning(
                        "User actively CANCELLED critical battery shutdown")
                    return True  # True = user cancelled, don't halt
            elif result.returncode == 5:
                # Dialog timeout (zenity returns 5 for timeout) - proceed with halt (SAFE DEFAULT)
                self.get_logger().info(
                    "Critical battery dialog timed out - proceeding with halt (safe default)")
                return False  # False = proceed with halt
            else:
                # Unexpected return code - default to shutdown for safety
                self.get_logger().warning(f"Dialog returned unexpected code: {result.returncode} - defaulting to halt for safety")
                return False  # False = proceed with halt

        except subprocess.TimeoutExpired:
            # Subprocess timeout - proceed with halt (safe default)
            self.get_logger().info("Critical battery dialog timed out - proceeding with halt (safe default)")
            return False  # False = proceed with halt
        except FileNotFoundError:
            self.get_logger().warning("zenity not found - trying kdialog...")
            try:
                # Fallback to kdialog - use same customized messages
                if is_ac_power_timeout and hours_ago:
                    kdialog_title = "AC POWER TIMEOUT - CRITICAL BATTERY"
                    kdialog_text = (
                        f"Battery voltage critically low: {voltage_str}\n\n"
                        f"⚠️ AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h)\n"
                        f"Charger chip may have timed out charging or supplemental power.\n"
                        f"System will shutdown in 30 seconds to allow you to replug charger.\n\n"
                        f"TIMEOUT = AUTOMATIC SHUTDOWN (safe default)\n\n"
                        f"Click 'No' ONLY if you can replug charger NOW.\n"
                        f"Click 'Yes' to shutdown immediately.\n"
                        f"No action = automatic shutdown after 30 seconds.\n\n"
                        f"To abort from CLI: pkill -f kdialog")
                else:
                    kdialog_title = "CRITICAL BATTERY SHUTDOWN"
                    kdialog_text = (
                        f"Battery voltage critically low: {voltage_str}\n\n"
                        f"System will halt in 30 seconds to preserve power for manual sailing.\n\n"
                        f"TIMEOUT = AUTOMATIC HALT (safe default)\n\n"
                        f"Click 'No' ONLY if you can plug in charger NOW.\n"
                        f"Click 'Yes' to halt immediately.\n"
                        f"No action = automatic halt after 30 seconds.\n\n"
                        f"To abort from CLI: pkill -f kdialog")
                
                dialog_cmd = [
                    'kdialog',
                    f'--title={kdialog_title}',
                    f'--yesno={kdialog_text}',
                    '--yes-label=Shutdown Now',
                    '--no-label=Cancel (I will plug in charger)'
                ]

                result = subprocess.run(dialog_cmd, capture_output=True, text=True, timeout=35, env=env)

                if result.returncode == 0:
                    # User clicked "Yes" / "Shutdown Now"
                    self.get_logger().info("User confirmed critical battery shutdown (kdialog)")
                    return False  # False = proceed with halt
                else:
                    # User clicked "No" / "Cancel" or timeout
                    if result.returncode == 1:
                        self.get_logger().warning("User actively CANCELLED critical battery shutdown (kdialog)")
                        return True  # True = user cancelled
                    else:
                        # Check for display error
                        if "cannot open display" in result.stderr.lower():
                            self.get_logger().error(
                                f"kdialog failed to open display: {result.stderr.strip()}")
                            self.get_logger().error(
                                "Proceeding with halt as a safety measure.")
                            return False # Treat as confirm
                        else:
                            # Timeout or error - default to halt for safety
                            self.get_logger().info(
                                "kdialog timeout or error - proceeding with halt (safe default)")
                            return False  # False = proceed with halt

            except (FileNotFoundError, subprocess.TimeoutExpired):
                self.get_logger().warning("kdialog also not available or timed out - trying yad...")
                try:
                    # Fallback to yad (Yet Another Dialog) - use same customized messages
                    if is_ac_power_timeout and hours_ago:
                        yad_title = "AC POWER TIMEOUT - CRITICAL BATTERY"
                        yad_text = (
                            f"Battery voltage critically low: {voltage_str}\n\n"
                            f"⚠️ AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h)\n"
                            f"Charger chip may have timed out charging or supplemental power.\n"
                            f"System will shutdown in 30 seconds to allow you to replug charger.\n\n"
                            f"TIMEOUT = AUTOMATIC SHUTDOWN (safe default)\n\n"
                            f"Click 'Cancel' ONLY if you can replug charger NOW.\n"
                            f"No action = automatic shutdown after 30 seconds.\n\n"
                            f"To abort from CLI: pkill -f yad")
                    else:
                        yad_title = "CRITICAL BATTERY SHUTDOWN"
                        yad_text = (
                            f"Battery voltage critically low: {voltage_str}\n\n"
                            f"System will halt in 30 seconds to preserve power for manual sailing.\n\n"
                            f"TIMEOUT = AUTOMATIC HALT (safe default)\n\n"
                            f"Click 'Cancel' ONLY if you can plug in charger NOW.\n"
                            f"No action = automatic halt after 30 seconds.\n\n"
                            f"To abort from CLI: pkill -f yad")
                    
                    dialog_cmd = [
                        'yad',
                        f'--title={yad_title}',
                        f'--text={yad_text}',
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
                        self.get_logger().warning("User actively CANCELLED via yad")
                        return True  # User cancelled
                    else:
                        # Check for display error
                        if "cannot open display" in result.stderr.lower():
                            self.get_logger().error(
                                f"yad failed to open display: {result.stderr.strip()}")
                            self.get_logger().error(
                                "Proceeding with halt as a safety measure.")
                        return False  # Timeout or error = proceed with halt

                except (FileNotFoundError, subprocess.TimeoutExpired):
                    self.get_logger().warning("No dialog tools available - proceeding with halt (safe default)")
                    return False  # False = proceed with halt
        except Exception as e:
            self.get_logger().error(f"Error showing critical battery dialog: {e}")
            self.get_logger().info("Proceeding with halt due to dialog error (safe default)")
            return False  # False = proceed with halt

    def _pause_argo_system_for_power_conservation(self):
        """Best-effort quieting before halt; legacy /toggle_pause path removed."""
        if not self.check_argo_service_status():
            self.get_logger().info(
                "Argo service not running - skipping pre-halt coordination")
            return False

        self.get_logger().info(
            "Pre-halt: /toggle_pause removed from Argo; proceeding without remote pause service")
        return True

    def initiate_critical_battery_halt(self, battery_voltage, reason=None):
        """Initiate critical battery halt sequence with timeout-based confirmation

        Args:
            battery_voltage: Battery voltage (use 0.0 if voltage is unknown/unavailable)
            reason: Optional reason string for logging (e.g., "battery_service_failure_timeout")

        Shows a confirmation dialog with 30-second timeout:
        - If user clicks "Cancel": Halt is cancelled (allows intervention)
        - If user clicks "Shutdown Now": Immediate halt
        - If timeout (30s, no user action): Automatic halt proceeds (SAFE DEFAULT)

        This provides:
        - Autonomous safety: Timeout = halt proceeds (not cancelled)
        - Developer control: Active cancellation stops halt if user intervenes
        """
        # Format voltage string (handle unknown voltage)
        if battery_voltage > 0:
            voltage_str = f"{battery_voltage:.3f}V"
        else:
            voltage_str = "UNKNOWN (monitoring unavailable)"
        
        # Format reason string
        reason_str = f" - {reason}" if reason else ""
        
        # Log critical battery mode based on configuration flag
        if CRITICAL_BATTERY_USE_SHUTDOWN:
            self.get_logger().error(
                f"CRITICAL BATTERY SHUTDOWN: {voltage_str}{reason_str} - System will shutdown and CUT POWER")
            self.get_logger().error(
                "⚠️  DEVELOPMENT MODE: Using normal shutdown (cuts power) instead of halt")
            self.get_logger().error(
                "⚠️  CRITICAL_BATTERY_USE_SHUTDOWN=True - Power will be CUT, not preserved")
            shutdown_mode = "shutdown (power cut)"
        else:
            self.get_logger().error(
                f"CRITICAL BATTERY HALT: {voltage_str}{reason_str} - System will halt to preserve power")
            self.get_logger().error(
                "PRODUCTION MODE: Using halt to preserve power for manual sailing")
            shutdown_mode = "halt (power preserved)"

        self.get_logger().error(
            f"Showing confirmation dialog - timeout (no action) will proceed with {shutdown_mode}")

        # Set critical battery flag for shutdown hook ONLY if using halt mode
        # If using shutdown mode, we want the hook to cut power normally
        if not CRITICAL_BATTERY_USE_SHUTDOWN:
            self._set_critical_battery_flag()
            self.get_logger().info("Critical battery flag set - shutdown hook will preserve power")
        else:
            self.get_logger().info("Critical battery flag NOT set - shutdown hook will cut power normally")

        # Check if this is an AC power timeout situation
        is_ac_power_timeout = reason and reason.startswith("ac_power_timeout")
        hours_ago = None
        if is_ac_power_timeout and "_" in reason:
            try:
                # Extract hours from reason string like "ac_power_timeout_20.5h"
                hours_str = reason.split("_")[-1].rstrip("h")
                hours_ago = float(hours_str)
            except (ValueError, IndexError):
                pass
        
        # Send critical notification with appropriate message
        if is_ac_power_timeout:
            if battery_voltage > 0:
                notification_message = (
                    f"Battery voltage critically low: {battery_voltage:.3f}V\n"
                    f"AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h)\n"
                    f"Charger may have timed out - replug charger to resume charging\n"
                    f"System will {shutdown_mode} in 30 seconds unless cancelled\n"
                    f"Timeout = automatic shutdown (safe default)")
            else:
                notification_message = (
                    f"AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h)\n"
                    f"Charger may have timed out - replug charger to resume charging\n"
                    f"System will {shutdown_mode} in 30 seconds unless cancelled\n"
                    f"Timeout = automatic shutdown (safe default)")
        else:
            if battery_voltage > 0:
                notification_message = f"Battery voltage critically low: {battery_voltage:.3f}V\nSystem will {shutdown_mode} in 30 seconds unless cancelled\nTimeout = automatic shutdown (safe default)"
            else:
                notification_message = f"Battery monitoring unavailable{reason_str}\nSystem will {shutdown_mode} in 30 seconds unless cancelled\nTimeout = automatic shutdown (safe default)"
        
        self.send_desktop_notification(
            "CRITICAL BATTERY" if not is_ac_power_timeout else "AC POWER TIMEOUT",
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

            if is_ac_power_timeout:
                message = f"""AC POWER TIMEOUT ALERT: {voltage_str}{reason_str} at {timestamp}

⚠️  AC POWER TIMEOUT: Charger plugged in {hours_ago:.1f}h ago (timeout: {AC_POWER_TIMEOUT_HOURS}h)
   Charger chip may have timed out charging or supplemental power.
   System will {action_desc} in 30 seconds to allow user to replug charger.

To CANCEL the shutdown from CLI (close the confirmation dialog):
  pkill -f zenity
  # Or kill all dialog processes:
  pkill -f "zenity|kdialog|yad"

If you take no action within 30 seconds, the system will automatically
{mode_desc}.

After shutdown, replug the AC charger to resume charging."""
            else:
                message = f"""CRITICAL BATTERY ALERT: {voltage_str}{reason_str} at {timestamp}

⚠️  System will {action_desc} in 30 seconds.

To CANCEL the shutdown from CLI (close the confirmation dialog):
  pkill -f zenity
  # Or kill all dialog processes:
  pkill -f "zenity|kdialog|yad"

If you take no action within 30 seconds, the system will automatically
{mode_desc}."""
            subprocess.run(['wall', message], check=True,
                           timeout=WALL_MESSAGE_TIMEOUT_S)
            self.get_logger().info("Critical battery wall message broadcasted")
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast critical battery message: {e}")

        # Show confirmation dialog with safe defaults
        # CRITICAL: Timeout or "Shutdown Now" = proceed with halt
        # Only "Cancel" button stops the halt
        self.get_logger().info("Showing critical battery confirmation dialog (timeout = halt proceeds)...")
        user_cancelled = self.show_critical_battery_confirmation_dialog(battery_voltage, reason=reason)

        if user_cancelled:
            # User ACTIVELY cancelled the halt/shutdown
            self.get_logger().warning(f"Critical battery {shutdown_mode} CANCELLED by user intervention")
            self.get_logger().warning("User has taken responsibility - they must plug in charger or take action!")
            # Clear the critical battery flag since we're not shutting down (if it was set)
            if not CRITICAL_BATTERY_USE_SHUTDOWN:
                self._clear_critical_battery_flag()
            # Resume heartbeat since shutdown was cancelled
            self.resume_heartbeat(reason="critical_battery_cancelled")
            # Send cancellation notification
            self.send_desktop_notification(
                "SHUTDOWN CANCELLED BY USER",
                f"Critical battery {shutdown_mode} was cancelled by user\nBattery: {voltage_str} - PLUG IN CHARGER NOW!",
                "critical",
                0  # Stays visible
            )
            return  # Exit without shutting down

        # If we reach here, either:
        # 1. User clicked "Shutdown Now" (proceed immediately)
        # 2. Dialog timed out (proceed automatically - SAFE DEFAULT)
        self.get_logger().error(f"Proceeding with critical battery {shutdown_mode}")
        self.get_logger().error("Either user confirmed OR timeout occurred (automatic action for safety)")

        # Stop battery monitoring to prevent repeated alerts
        self.battery_monitoring_active = False

        # CRITICAL POWER CONSERVATION: Pause Argo system to put sensors in shutdown state
        self.get_logger().error("Pausing Argo system to conserve battery power...")
        pause_success = self._pause_argo_system_for_power_conservation()

        if pause_success:
            self.get_logger().info("Sensors paused - waiting 2 seconds for hardware shutdown...")
            time.sleep(2)  # Give sensors time to enter hardware shutdown state
        else:
            self.get_logger().warning("Could not pause sensors - proceeding with halt anyway")

        # Brief delay to allow final notifications
        self.get_logger().error("Waiting 3 seconds for final notifications...")
        time.sleep(3)

        # Execute command based on configuration flag
        if self.test_mode:
            if CRITICAL_BATTERY_USE_SHUTDOWN:
                self.get_logger().info(
                    "TEST MODE: Would execute 'shutdown -h now' command for critical battery")
                self.get_logger().info("TEST MODE: Critical battery shutdown sequence completed - system would SHUTDOWN and CUT POWER NOW")
            else:
                self.get_logger().info(
                    "TEST MODE: Would execute 'sudo halt' command for critical battery")
                self.get_logger().info("TEST MODE: Critical battery halt sequence completed - system would HALT and PRESERVE POWER NOW")
        else:
            if CRITICAL_BATTERY_USE_SHUTDOWN:
                self.get_logger().error(
                    "⚠️  DEVELOPMENT MODE: Executing systemctl poweroff NOW - POWER WILL BE CUT")
                self.get_logger().error(
                    "⚠️  CRITICAL_BATTERY_USE_SHUTDOWN=True - Using systemctl poweroff instead of halt")
                subprocess.run(
                    ['systemctl', 'poweroff', '--message', 'ARGO critical battery shutdown'],
                    check=True
                )
                self.get_logger().error("Poweroff command executed - system will shutdown and cut power")
            else:
                self.get_logger().error(
                    "PRODUCTION MODE: Executing halt command NOW for critical battery preservation")
                self.get_logger().error(
                    "Power relay will be preserved for manual sailing - shutdown hook will NOT cut power")
                subprocess.run(['sudo', 'halt'], check=True)
                self.get_logger().error("Halt command executed - system should halt immediately with power preserved")

    def _set_critical_battery_flag(self):
        """Set critical battery flag file for shutdown hook"""
        try:
            with open(CRITICAL_BATTERY_FLAG_FILE, 'w') as f:
                f.write(
                    f"CRITICAL_BATTERY_DETECTED\nTimestamp: {datetime.now().isoformat()}\n")
            self.get_logger().info(
                f"Critical battery flag set: {CRITICAL_BATTERY_FLAG_FILE}")
        except Exception as e:
            self.get_logger().error(f"Error setting critical battery flag: {e}")

    def _clear_critical_battery_flag(self):
        """Clear critical battery flag file"""
        try:
            if os.path.exists(CRITICAL_BATTERY_FLAG_FILE):
                os.remove(CRITICAL_BATTERY_FLAG_FILE)
                self.get_logger().info("Critical battery flag cleared")
        except Exception as e:
            self.get_logger().error(f"Error clearing critical battery flag: {e}")

    def cleanup(self):
        """Clean up resources"""
        self.get_logger().info("=== POWER_CONTROL_NODE CLEANUP ===")
        self.get_logger().info("Cleaning up power controller...")

        try:
            # Stop button monitor before releasing GPIO (avoids NoneType on event_get_fd)
            self.running = False
            self.button_detection_active = False
            if self._button_monitor_thread is not None:
                self._button_monitor_thread.join(timeout=2.0)
                if self._button_monitor_thread.is_alive():
                    self.get_logger().warning(
                        "Button monitor thread did not exit before GPIO release")

            # Stop battery monitoring
            self.battery_monitoring_active = False

            # Stop WiFi monitoring
            self.wifi_monitoring_active = False

            # Stop SOS LED pattern if active
            self.sos_led_active = False
            self.i2c_failure_sos_active = False

            # Stop charge state LED pattern if active
            self.charge_state_led_active = False
            self.charging_state_active = False
            # Immediately turn off charge state LED
            try:
                led_setter = self._get_charge_state_led_setter()
                led_setter(False)
            except Exception:
                pass  # Ignore errors during cleanup

            # Stop service wait pattern if active
            self.service_wait_active = False


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

            # Release GPIO lines before closing chip
            self.gpio_available = False
            self._release_gpio_lines_safe()

            # Release sysfs LED control back to kernel
            self.release_sysfs_led()

            # Close GPIO chip if available
            if hasattr(self, 'chip') and self.chip is not None:
                try:
                    self.chip.close()
                except Exception as e:
                    self.get_logger().warning(f"Error closing GPIO chip: {e}")

            # ROS2 cleanup
            self._cleanup_ros2()

        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

        self.get_logger().info("Power controller cleanup complete")

    def _release_gpio_lines_safe(self):
        """Safely release all GPIO lines without raising exceptions"""
        try:
            if hasattr(self, 'power_button_line') and self.power_button_line is not None:
                self.power_button_line.release()
                self.power_button_line = None
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().warning(f"Error releasing power_button_line: {e}")
        
        try:
            if hasattr(self, 'green_led_line') and self.green_led_line is not None:
                self.green_led_line.release()
                self.green_led_line = None
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().warning(f"Error releasing green_led_line: {e}")
        
        try:
            if hasattr(self, 'red_led_line') and self.red_led_line is not None:
                self.red_led_line.release()
                self.red_led_line = None
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().warning(f"Error releasing red_led_line: {e}")
        
        try:
            if hasattr(self, 'blue_led_line') and self.blue_led_line is not None:
                self.blue_led_line.release()
                self.blue_led_line = None
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().warning(f"Error releasing blue_led_line: {e}")

    def set_heartbeat_frequency(self, frequency_hz):
        """Set the heartbeat frequency"""
        if frequency_hz > 0:
            self.get_logger().info(f"Setting heartbeat frequency to {frequency_hz} Hz")
            self.heartbeat_frequency_hz = frequency_hz
        else:
            self.get_logger().warning(f"Invalid heartbeat frequency: {frequency_hz}")

    def pause_heartbeat(self, reason: Optional[str] = None):
        """Pause the heartbeat to allow special LED patterns to be visible"""
        reason = reason or "unspecified"
        if not self.heartbeat_paused or self.heartbeat_pause_reason != reason:
            self.get_logger().info(f"Pausing heartbeat (reason: {reason})")
        else:
            self.get_logger().debug(f"Heartbeat already paused (reason: {reason})")
        self.heartbeat_paused = True
        self.heartbeat_pause_reason = reason
        self.heartbeat_pause_time = time.time()
        self.last_heartbeat_pause_log = 0.0

    def resume_heartbeat(self, reason: Optional[str] = None):
        """Resume the heartbeat after special LED patterns are done"""
        resume_reason = reason or self.heartbeat_pause_reason or "unspecified"
        if self.heartbeat_paused:
            self.get_logger().info(f"Resuming heartbeat (reason: {resume_reason})")
        else:
            self.get_logger().debug(f"Heartbeat resume requested while already running (requested reason: {resume_reason})")
        self.heartbeat_paused = False
        self.heartbeat_pause_reason = None
        self.heartbeat_pause_time = 0.0
        self.last_heartbeat_pause_log = 0.0

    def _log_paused_state_status(self):
        """Log a throttled INFO message describing current paused state (SOS, charging, etc.)."""
        reason = self.heartbeat_pause_reason or "unspecified"
        if reason == "low_battery":
            volt = f"{self.last_battery_voltage:.2f}V" if self.last_battery_voltage is not None else "?"
            ac = "yes" if self.ac_power_from_topic is True else "no"
            self.get_logger().info(
                f"Low battery SOS active | Battery: {volt} | AC: {ac} | Red LED blinking SOS"
            )
        elif reason == "i2c_failure" or reason == "i2c_failure_recovered":
            self.get_logger().info(
                "I2C failure SOS active | Battery monitoring unavailable | Red LED blinking SOS"
            )
        elif reason == "charging_state":
            self.get_logger().info(
                "Charging state pattern active | AC connected | Green LED showing charge state"
            )
        else:
            self.get_logger().info(f"LED heartbeat paused (reason: {reason})")

    def _update_led_heartbeat_for_controller_mode(self):
        """Set LED heartbeat cadence from HumanController vs autonomous (Argo running)."""
        try:
            # Only update if Argo service is running (controller is available)
            if not self.argo_service_running:
                # Argo not running - use normal heartbeat
                self.set_heartbeat_frequency(LED_HEARTBEAT_HZ)
                self.get_logger().info("LED heartbeat: Normal (1Hz) - Argo service not running")
                return

            # Argo is running - slow heartbeat when HumanController is active (manual / radio focus)
            if self.controller_runtime_is_human:
                self.set_heartbeat_frequency(LED_PAUSED_HEARTBEAT_HZ)
                self.get_logger().info("LED heartbeat: Slow (0.5Hz) - HumanController active")
            else:
                self.set_heartbeat_frequency(LED_HEARTBEAT_HZ * 2.0)
                self.get_logger().info("LED heartbeat: Fast (2Hz) - Autonomous controller active")

        except Exception as e:
            self.get_logger().error(f"Error updating LED heartbeat for controller mode: {e}")

    def _cleanup_ros2(self):
        """Clean up ROS2 resources"""
        try:
            self.destroy_node()
        except Exception:
            pass
        # rclpy.shutdown() is called in the main function

    def _call_trigger_service(self, service_name: str, timeout_sec: float = 2.0) -> tuple[bool, str]:
        """Centralized function for calling ROS2 Trigger services."""
        if not rclpy.ok():
            return False, "ROS2 node not initialized"

        client = None  # Ensure client is defined in outer scope
        try:
            client = self.create_client(Trigger, service_name)
            if not client.wait_for_service(timeout_sec=2.0):
                # Error message - duration context will be added in monitoring loop when failure persists
                self.get_logger().error(f"Service {service_name} not available")
                return False, f"Service {service_name} not available"

            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=timeout_sec)

            if future.done():
                response = future.result()
                if response is not None:
                    return response.success, response.message
                else:
                    # This can happen if the service call fails without an exception
                    return False, "Service call failed: received no response"
            else:
                self.get_logger().error(
                    f"Service call to {service_name} timed out after {timeout_sec}s")
                return False, "Service call timed out"
        except Exception as e:
            self.get_logger().error(f"Exception calling service {service_name}: {e}")
            return False, str(e)
        finally:
            if client is not None and rclpy.ok():
                self.destroy_client(client)


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
  - PH0 (Pin 8): POW_BUT - Input from power button (active HIGH when pressed)
  - PI2 (Pin 35): BUZZER - Pulsed via scripts/abeep.sh when needed
  - PH4 (Pin 18): Green LED in power button
      Hardware: NFET control (GPIO HIGH = LED ON, GPIO LOW = LED OFF)
      Device tree: GPIO_ACTIVE_LOW (kernel driver inverts polarity)
      Software: Inverts brightness values in set_green_led() to compensate
      Note: Rev3 PCB uses NFET control, different from rev2 direct cathode control.
  - PI1 (Pin 12): Blue LED in power button (active LOW - cathode control)
  - Red LED: Not GPIO controlled (common anode RGB LED)
  - LEDs: Active LOW control (GPIO LOW = ON, GPIO HIGH = OFF)

POWER BUTTON BEHAVIOR (Active HIGH):
  - Single tap: Temporarily switch to HumanController (manual servo/radio); tap again to restore
  - Double tap (2 quick taps): Toggle rosbag recording on/off
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
  - RGB State Publishing: LED RGB state is published to /argo/power_button/rgb for mast head LED mirroring
                           (geometry_msgs/Vector3: x=red, y=green, z=blue as normalized 0.0-1.0)

USAGE:
  ./argo_power_control.py [OPTIONS]

OPTIONS:
  --help, -h          Show this help message and exit
  --test-mode, -t     Run in test mode (disable actual shutdown and power control)
  --threshold, -T     Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
  --test-wall-message, -w  Test wall message functionality and exit (safe for testing)
  --test-notification, -n  Test desktop notification functionality and exit (safe for testing)
  --test-led-patterns, -d  Test shutdown countdown LED pattern and exit (safe for testing)
  --simulate-single-tap          Simulate a single tap (human vs autonomous controller)
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
  ./argo_power_control.py --simulate-single-tap   # Simulate single tap (controller human toggle)
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
    # Initialize rclpy early so we can create a temporary node for argument parsing if needed
    if ROS2_AVAILABLE:
        # Pass all arguments except ROS-specific ones to rclpy.init()
        ros_args = rclpy.utilities.remove_ros_args(args=sys.argv)
        rclpy.init(args=ros_args)

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
                        help='Simulate a single tap (toggle human vs autonomous controller)')
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

    argcomplete.autocomplete(parser)
    # Use the filtered arguments for argparse
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    # Setup logging to stdout for journalctl compatibility
    setup_logging(debug=args.debug)

    # Create a temporary logger for the main function before the node is fully initialized
    temp_logger = rclpy.logging.get_logger('argo_power_control_main')

    # Handle help
    if args.help:
        print_help()
        sys.exit(0)

    # Check if any test/simulation flag is present
    is_test_or_sim_run = any([
        args.test_wall_message,
        args.test_notification,
        args.test_led_patterns,
        args.simulate_single_tap,
        args.simulate_double_tap,
        args.simulate_quadruple_tap,
        args.simulate_critical_battery,
        args.simulate_low_battery,
        args.simulate_wifi_loss
    ])

    if is_test_or_sim_run:
        # --- STANDALONE TEST/SIMULATION MODE ---
        temp_logger.info("Running in standalone test/simulation mode...")
        controller = None
        try:
            # Create a controller instance for testing
            controller = PowerController(test_mode=True, threshold=args.threshold)
            
            if args.test_wall_message:
                temp_logger.info("Testing wall message...")
                controller.broadcast_shutdown_message()
            elif args.test_notification:
                temp_logger.info("Testing desktop notification...")
                controller.send_desktop_notification("Test Notification", "This is a test message.", "normal")
            elif args.test_led_patterns:
                temp_logger.info("Testing LED patterns...")
                test_shutdown_pattern()
                test_gradual_frequency_pattern(args.threshold)
            elif args.simulate_single_tap:
                controller.toggle_controller_pause()
            elif args.simulate_double_tap:
                controller.toggle_recording()
            elif args.simulate_quadruple_tap:
                controller.restart_argo_service()
            elif args.simulate_critical_battery:
                controller.initiate_critical_battery_halt(CRITICAL_BATTERY_THRESHOLD_V - 0.1)
            elif args.simulate_low_battery:
                # Need to run the SOS pattern in a thread to see it
                sos_thread = threading.Thread(target=controller.sos_led_pattern, daemon=True)
                controller.low_battery_detected = True
                sos_thread.start()
                time.sleep(10) # Let it run for 10 seconds
                controller.low_battery_detected = False # Stop the pattern
                sos_thread.join()
            elif args.simulate_wifi_loss:
                # This requires the main loop to run, so it's harder to simulate here
                temp_logger.warning("WiFi loss simulation is best tested in service mode.")

        except Exception as e:
            temp_logger.error(f"Error during simulation: {e}")
        finally:
            if controller:
                controller.cleanup()
            if ROS2_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
        sys.exit(0)


    # --- SERVICE MODE ---
    # If no test/simulation flags are provided, run as a full ROS2 node.
    temp_logger.info("=== POWER_CONTROL_NODE STARTUP ===")
    temp_logger.info("Starting power control system in ROS2 Node mode...")
    
    node = None
    try:
        node = PowerController(
            test_mode=args.test_mode, threshold=args.threshold)
        
        # Set ROS2 logger level to DEBUG if debug flag is enabled
        if args.debug and ROS2_AVAILABLE:
            node.get_logger().set_level(LoggingSeverity.DEBUG)
            node.get_logger().info('Debug logging enabled for ROS2 logger')
        
        # Start background threads
        node.run_threads()
        
        # Spin the node - standard ROS2 pattern
        temp_logger.info("Spinning node...")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        temp_logger.info('=== POWER_CONTROL_NODE SHUTDOWN START ===')
        temp_logger.info('Keyboard interrupt, shutting down.')
    except rclpy.executors.ExternalShutdownException:
        temp_logger.info('=== POWER_CONTROL_NODE SHUTDOWN START ===')
        temp_logger.info('External shutdown requested.')
    finally:
        if node:
            temp_logger.info("Entering cleanup phase...")
            temp_logger.info("Cleaning up node...")
            node.cleanup()
            node.destroy_node()
        if ROS2_AVAILABLE and rclpy.ok():
            temp_logger.info("Shutting down rclpy...")
            rclpy.shutdown()
        temp_logger.info("=== POWER_CONTROL_NODE SHUTDOWN COMPLETE ===")


if __name__ == "__main__":
    main()
