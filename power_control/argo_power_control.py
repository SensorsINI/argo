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
#   - Short press (< threshold DEFAULT_SHUTDOWN_THRESHOLD_S): No action
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
#   - User must be member of 'gpio' group (for GPIO access)
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

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
DEFAULT_SHUTDOWN_THRESHOLD_S = 5.0      # Default button hold time for shutdown (seconds)
SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S = 3.0       # Warn users this many seconds before shutdown to release the button
BUTTON_POLLING_HZ = 10.0                # Button release polling frequency during press (10 Hz - only used during button press)
BUTTON_ERROR_RECOVERY_DELAY_S = 0.1     # Delay on button read error (seconds)
TRIPLE_TAP_MAX_DURATION_S = 1.5         # Maximum duration for 3 quick taps to toggle recording (seconds)

# Button Detection Configuration
# No boot delay - only detect new button presses after service starts

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
TEST_MODE_SHUTDOWN_DELAY_S = 5          # Simulated shutdown delay in test mode (seconds)

# Notification Timeouts
DESKTOP_NOTIFICATION_TIMEOUT_S = 5      # Desktop notification timeout (seconds)
WALL_MESSAGE_TIMEOUT_S = 5              # Wall message timeout (seconds)
NOTIFICATION_EXPIRE_TIME_MS = 5000      # Standard notification expire time (5 seconds)
FINAL_SHUTDOWN_NOTIFICATION_MS = 0      # Final shutdown notification (no timeout - stays until dismissed)

# Main Loop Timing
MAIN_LOOP_SLEEP_S = 1                   # Main control loop sleep interval (seconds)

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

class PowerController(Node):
    def __init__(self, test_mode=False, threshold=1.0):
        super().__init__('argo_power_control')
        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        
        # Button state tracking
        self.initial_button_state = None
        self.button_detection_active = False  # Flag to track when button detection should be active
        self.warning_notification_sent = False  # Flag to track if warning notification has been sent for current button press
        
        # Triple tap detection for recording toggle
        self.tap_times = []  # List to store tap timestamps
        self.max_tap_count = 3  # Number of taps needed
        self.last_tap_time = 0  # Time of last tap
        
        # LED state tracking
        self.green_led_state = False
        self.blue_led_state = False
        
        # GPIO Configuration
        self.GPIO_CHIP = '/dev/gpiochip0'
        
        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 265   # PI9 (Pin 28) - !POW_BUT  
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED
        self.BLUE_LED_LINE = 257       # PI1 (Pin 12) - Blue LED
        
        # Button press threshold (seconds)
        self.SHUTDOWN_THRESHOLD = threshold
        
        # ROS2 Setup
        self.setup_ros2()
        
        # Setup signal handlers for graceful service shutdown
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGQUIT, self.signal_handler)
        
        logger.info("Power controller initialized")
        
        # Initialize GPIO first to read initial button state
        self.init_gpio()
        
        # Record initial button state during boot (must be done after GPIO init but before event config)
        self.initial_button_state = self.read_initial_button_state()
        logger.info(f"Initial button state recorded: {self.initial_button_state}")
        
        # Set button detection flag based on initial state
        if self.initial_button_state == 1:  # Button not pressed at startup
            self.button_detection_active = True  # Can detect presses immediately
            logger.info("Button detection active - ready to detect presses")
        else:  # Button pressed at startup
            self.button_detection_active = False  # Wait for first release
            logger.info("Button pressed at startup - waiting for release before detection starts")
            
        # Now reconfigure the button line for interrupt monitoring
        self.configure_button_for_interrupts()


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
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO: {e}")
            logger.error(f"GPIO chip path attempted: {self.GPIO_CHIP}")
            logger.error(f"Full error details: {type(e).__name__}: {e}")
            raise

    def setup_ros2(self):
        """Setup ROS2 publishers, subscribers, and service clients"""
        try:
            self.get_logger().info("Setting up ROS2 components...")
            
            # QoS profile for reliable communication
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
            
            # Publishers
            self.led_status_pub = self.create_publisher(
                String, 
                '/argo/power_control/led_status', 
                qos_profile
            )
            
            self.node_health_pub = self.create_publisher(
                DiagnosticArray,
                '/argo/power_control/node_health',
                qos_profile
            )
            
            # Subscriber for recording status (match publisher's TRANSIENT_LOCAL QoS)
            recording_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
            self.recording_status_sub = self.create_subscription(
                Bool,
                '/argo/recording/status',
                self.recording_status_callback,
                recording_qos
            )
            
            # Service clients for recording control
            self.start_recording_client = self.create_client(Empty, '/argo/recording/start')
            self.stop_recording_client = self.create_client(Empty, '/argo/recording/stop')
            
            # Recording state tracking
            self.recording_active = False
            
            # ROS2 timers for periodic tasks
            self.status_timer = self.create_timer(1.0, self.publish_status)  # 1Hz status updates
            self.health_timer = self.create_timer(5.0, self.publish_health)  # 5s health checks
            
            self.get_logger().info("ROS2 components initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup ROS2 components: {e}")
            raise

    def recording_status_callback(self, msg):
        """Callback for recording status updates"""
        self.get_logger().info(f"📡 Recording status callback received: {msg.data}")
        was_recording = self.recording_active
        self.recording_active = msg.data
        
        if was_recording != self.recording_active:
            if self.recording_active:
                self.get_logger().info("🎬 Recording started - LED heartbeat shows 3-flash pattern")
            else:
                self.get_logger().info("⏹️ Recording stopped - LED heartbeat returns to normal pulse")
        else:
            self.get_logger().info(f"📡 Recording status unchanged: {self.recording_active}")

    def publish_status(self):
        """Publish LED status information"""
        try:
            status_msg = String()
            led_states = []
            if hasattr(self, 'green_led_state') and self.green_led_state:
                led_states.append("GREEN")
            if hasattr(self, 'blue_led_state') and self.blue_led_state:
                led_states.append("BLUE")
            
            status_msg.data = f"LEDs: {','.join(led_states) if led_states else 'OFF'} | Recording: {'ON' if self.recording_active else 'OFF'}"
            self.led_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def publish_health(self):
        """Publish node health diagnostics"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Power control health status
            status = DiagnosticStatus()
            status.name = "argo_power_control"
            status.level = DiagnosticStatus.OK
            status.message = "Power control system operational"
            
            # Add key-value pairs
            status.values.append(KeyValue(key="test_mode", value=str(self.test_mode)))
            status.values.append(KeyValue(key="shutdown_threshold", value=str(self.SHUTDOWN_THRESHOLD)))
            status.values.append(KeyValue(key="recording_active", value=str(self.recording_active)))
            status.values.append(KeyValue(key="button_detection_active", value=str(self.button_detection_active)))
            
            diag_array.status.append(status)
            self.node_health_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing health: {e}")

    def start_recording(self):
        """Start recording via ROS2 service"""
        try:
            if not self.start_recording_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Recording start service not available")
                return False
                
            request = Empty.Request()
            future = self.start_recording_client.call_async(request)
            self.get_logger().info("🎬 Requesting recording start...")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error starting recording: {e}")
            return False

    def stop_recording(self):
        """Stop recording via ROS2 service"""
        try:
            if not self.stop_recording_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Recording stop service not available")
                return False
                
            request = Empty.Request()
            future = self.stop_recording_client.call_async(request)
            self.get_logger().info("⏹️ Requesting recording stop...")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")
            return False

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, stopping power controller...")
        self.running = False


    def read_initial_button_state(self):
        """Read initial power button state during startup (before interrupt configuration)"""
        try:
            state = self.power_button_line.get_value()
            logger.info(f"Initial button state read: {state} ({'pressed' if state == 0 else 'released'})")
            return state
        except Exception as e:
            logger.error(f"Error reading initial button state: {e}")
            return 1  # Assume released on error
    
    def configure_button_for_interrupts(self):
        """Reconfigure button line for interrupt-based monitoring"""
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
            logger.info("Button line reconfigured for falling edge interrupt monitoring")
            
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
        try:
            value = 1 if state else 0
            self.green_led_line.set_value(value)
            self.green_led_state = state
            logger.debug(f"Green LED set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED (status/warning indicator)"""
        try:
            value = 1 if state else 0
            self.blue_led_line.set_value(value)
            self.blue_led_state = state
            logger.debug(f"Blue LED set to {'ON' if state else 'OFF'}")
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
            print(f"{Colors.CYAN}Recording... {i+1}/10{Colors.RESET}", end="\r", flush=True)
        
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

    def green_led_heartbeat(self):
        """Green LED heartbeat - normal pulse when idle, 3-flash pattern when recording"""
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
                # Normal mode: simple on/off heartbeat at 1Hz
                heartbeat_interval = 1.0 / LED_HEARTBEAT_HZ  # 1 second period
                half_period = heartbeat_interval / 2.0  # 500ms on, 500ms off
                
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


    def shutdown_led_pattern(self):
        """1Hz LED pattern with configurable duty cycle during shutdown sequence"""
        logger.info(f"Starting shutdown LED pattern (1Hz, {LED_SHUTDOWN_DUTY_CYCLE*100:.0f}% duty cycle)")
        
        # Calculate timing for 1Hz with configurable duty cycle
        period = 1.0 / LED_SHUTDOWN_FREQUENCY_HZ  # 1 second
        on_time = period * LED_SHUTDOWN_DUTY_CYCLE  # Configurable on time
        off_time = period * (1.0 - LED_SHUTDOWN_DUTY_CYCLE)  # Remaining off time
        
        while self.running and self.shutdown_initiated:
            try:
                # Turn on both LEDs
                logger.debug("Shutdown LED: ON")
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
                logger.debug("Shutdown LED: OFF")
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
        logger.debug(f"Starting gradual frequency pattern for {self.SHUTDOWN_THRESHOLD}s threshold")
        
        while self.running and self.power_button_pressed and not self.shutdown_initiated:
            # Calculate elapsed time and progress (0.0 to 1.0)
            elapsed_time = time.time() - start_time
            progress = min(elapsed_time / self.SHUTDOWN_THRESHOLD, 1.0)
            
            # Calculate current frequency (2Hz to 20Hz)
            current_frequency = LED_PRESS_START_FREQUENCY_HZ + (LED_PRESS_END_FREQUENCY_HZ - LED_PRESS_START_FREQUENCY_HZ) * progress
            
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
        logger.info("Using hardware interrupt-based button monitoring")
        logger.info("Monitoring for falling edge events (POW_BUT going low)")
        
        # IMPORTANT: Do NOT override button_detection_active here!
        # The __init__ method already set this correctly based on initial button state
        # If button was pressed at startup, detection remains inactive until first release
        logger.info(f"Button detection active status: {self.button_detection_active}")
        
        while self.running:
            try:
                # Wait for GPIO interrupt events using select() - this is the key to low CPU usage!
                # The process sleeps until a hardware interrupt occurs
                ready, _, _ = select.select([self.power_button_line.event_get_fd()], [], [], 1.0)
                
                if ready:
                    # Hardware interrupt occurred - read the event
                    try:
                        event = self.power_button_line.event_read()
                        
                        # We're configured for falling edge, so this is a button press
                        if event.type == gpiod.LineEvent.FALLING_EDGE:
                            if self.button_detection_active:
                                self.button_press_start_time = time.time()
                                self.power_button_pressed = True
                                self.warning_notification_sent = False  # Reset warning flag for new button press
                                logger.info("Power button pressed (hardware interrupt)")
                                
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
                                logger.debug("Button press detected but detection not yet active - ignoring")
                        else:
                            logger.warning(f"Unexpected event type: {event.type}")
                            
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
                                logger.info("Button released after startup - button detection now active")
                        except Exception as e:
                            logger.error(f"Error checking button state for activation: {e}")
                    
                    # This timeout happens every 1 second and uses minimal CPU
                    
            except Exception as e:
                logger.error(f"Error in interrupt-based button monitoring: {e}")
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
                    logger.info(f"Power button released after {press_duration:.2f} seconds (interrupt + polling)")
                    
                    if press_duration >= self.SHUTDOWN_THRESHOLD:
                        logger.info("Long press detected, initiating shutdown...")
                        self.initiate_shutdown()
                    else:
                        # Check if button was released after warning but before shutdown (shutdown cancelled)
                        if (press_duration >= SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S and 
                            self.warning_notification_sent and 
                            not self.shutdown_initiated):
                            logger.info("Shutdown cancelled - button released after warning threshold")
                            self.send_desktop_notification(
                                "Shutdown Cancelled",
                                f"Power button released - shutdown cancelled!\nButton was held for {press_duration:.1f}s",
                                "normal"
                            )
                        else:
                            # Short press - check for triple tap
                            if self.handle_triple_tap_detection(press_duration):
                                # Triple tap detected - toggle recording
                                self.toggle_recording()
                            else:
                                logger.info(f"Short press detected ({len(self.tap_times)}/3 taps)")
                    
                    self.power_button_pressed = False
                    self.button_press_start_time = None
                    self.warning_notification_sent = False  # Reset warning flag when button is released
                    break  # Exit this monitoring thread
                
                # Check for long press timeout while button is still held
                else:
                    press_duration = time.time() - self.button_press_start_time
                    
                    # Check for warning threshold - send notification to warn user to release button
                    if (press_duration >= SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S and 
                        not self.warning_notification_sent and 
                        not self.shutdown_initiated):
                        logger.info(f"Button press reached warning threshold ({SHUTDOWN_WARNING_BEFORE_SHUTDOWN_S}s) - sending warning notification")
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
                            logger.info("Long press threshold reached, initiating shutdown...")
                            self.initiate_shutdown()
                        break  # Exit monitoring loop
                
                time.sleep(poll_interval)
                
            except Exception as e:
                logger.error(f"Error in button release monitoring: {e}")
                time.sleep(BUTTON_ERROR_RECOVERY_DELAY_S)
                break
    
    def get_current_button_state(self):
        """Get current button state - works with interrupt-configured line"""
        try:
            # The line is configured for events, but we can still read the current value
            # This is needed to detect button release since we only get falling edge interrupts
            return self.power_button_line.get_value()
        except Exception as e:
            logger.error(f"Error reading current button state: {e}")
            return 1  # Assume released on error
    
    def handle_triple_tap_detection(self, press_duration):
        """Handle triple tap detection for recording toggle"""
        current_time = time.time()
        
        # Only consider short presses as taps (< 0.5 seconds)
        if press_duration > 0.5:
            # Long press - clear tap history
            self.tap_times.clear()
            logger.debug("Long press detected - clearing tap history")
            return False
        
        # Add this tap to the history
        self.tap_times.append(current_time)
        self.last_tap_time = current_time
        
        logger.debug(f"Tap {len(self.tap_times)} detected (duration: {press_duration:.2f}s)")
        
        # Check if we have 3 or more taps - check for triple tap BEFORE cleanup
        if len(self.tap_times) >= self.max_tap_count:
            # Check that the last 3 taps occurred within the time window
            last_three_taps = self.tap_times[-self.max_tap_count:]
            oldest_tap = min(last_three_taps)
            newest_tap = max(last_three_taps)
            
            if newest_tap - oldest_tap <= TRIPLE_TAP_MAX_DURATION_S + 0.01:  # Add small tolerance for floating point precision
                logger.info(f"Triple tap detected! ({newest_tap - oldest_tap:.2f}s duration)")
                self.tap_times.clear()  # Clear history after successful detection
                return True
        
        # Clean up old taps outside the time window (only if no triple tap detected)
        cutoff_time = current_time - TRIPLE_TAP_MAX_DURATION_S
        old_count = len(self.tap_times)
        self.tap_times = [t for t in self.tap_times if t > cutoff_time]
        
        if old_count != len(self.tap_times):
            logger.debug(f"Cleaned up {old_count - len(self.tap_times)} old taps")
        
        return False
    
    def toggle_recording(self):
        """Toggle rosbag recording on/off"""
        if self.test_mode:
            # Test mode - just show what would happen
            new_state = not self.recording_active
            logger.info(f"TEST MODE: Would toggle recording from {self.recording_active} to {new_state}")
            print(f"🎬 TEST: Recording would be {'STARTED' if new_state else 'STOPPED'}")
            
            # Send test notification
            self.send_desktop_notification(
                "Recording Toggle (Test)",
                f"Recording would be {'started' if new_state else 'stopped'} by triple tap",
                "normal"
            )
            
            # Update state for testing
            self.recording_active = new_state
            return
        
        # Real mode - actually toggle recording
        if self.recording_active:
            logger.info("Triple tap detected - stopping recording")
            self.stop_recording()
            self.send_desktop_notification(
                "Recording Stopped",
                "Recording stopped by triple tap",
                "normal"
            )
        else:
            logger.info("Triple tap detected - starting recording")
            self.start_recording()
            self.send_desktop_notification(
                "Recording Started", 
                "Recording started by triple tap",
                "normal"
            )

    def send_desktop_notification(self, title, message, urgency="normal", expire_time_ms=None):
        """Send desktop notification using notify-send"""
        try:
            # Modify title and message for test mode
            if self.test_mode:
                test_title = f"[TEST] {title}"
                test_message = f"{message}\n\n(This is a test notification - no actual action taken)"
                logger.info(f"TEST MODE: Sending desktop notification - {title}: {message}")
            else:
                test_title = title
                test_message = message
            
            # Use custom expire time or default
            expire_time = expire_time_ms if expire_time_ms is not None else NOTIFICATION_EXPIRE_TIME_MS
            
            # Check if we're running as root (systemd service)
            if os.geteuid() == 0:
                # Running as root - need to send notification as the desktop user
                # Find the user who owns the desktop session
                desktop_user = None
                display_env = None
                
                try:
                    # Look for X server processes to find the desktop user and display
                    result = subprocess.run(['ps', 'aux'], capture_output=True, text=True, timeout=2)
                    if result.returncode == 0:
                        for line in result.stdout.split('\n'):
                            # Look for Xorg process to get display and user
                            if 'Xorg' in line and ':0' in line:
                                parts = line.split()
                                if len(parts) > 0:
                                    # Get user from first column
                                    potential_user = parts[0]
                                    if potential_user != 'root':
                                        desktop_user = potential_user
                                        # Extract display from Xorg command line
                                        if ':0' in line:
                                            display_env = ':0'
                                        break
                except Exception as e:
                    logger.debug(f"Could not determine desktop user from ps: {e}")
                
                # Fallback to the user who invoked sudo or the current user
                if not desktop_user:
                    desktop_user = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.environ.get('LOGNAME') or 'root'
                if not display_env:
                    display_env = ':0'  # Default display
                
                # Run notify-send as the desktop user with proper environment
                cmd = [
                    'sudo', '-u', desktop_user,
                    'DISPLAY=' + display_env,
                    'notify-send',
                    '--urgency', urgency,
                    '--expire-time', str(expire_time),
                    test_title,
                    test_message
                ]
                
                logger.debug(f"Sending notification as user {desktop_user} with DISPLAY={display_env}")
                
            else:
                # Running as regular user - use normal method
                # Set up environment for desktop notifications
                env = os.environ.copy()
                
                # Try to find the display for logged-in users
                try:
                    # Get list of logged-in users and their displays
                    result = subprocess.run(['who'], capture_output=True, text=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                    if result.returncode == 0:
                        for line in result.stdout.strip().split('\n'):
                            if line and ':' in line:
                                parts = line.split()
                                for part in parts:
                                    if part.startswith(':'):
                                        display = part[1:]  # Remove the leading ':'
                                        if display.isdigit() or '.' in display:
                                            env['DISPLAY'] = f':{display}'
                                            break
                                if 'DISPLAY' in env:
                                    break
                except Exception as e:
                    logger.debug(f"Could not determine display from 'who' command: {e}")
                
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
                subprocess.run(cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            else:
                # For user execution, use env parameter
                subprocess.run(cmd, check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S, env=env)
            
            if self.test_mode:
                logger.info(f"TEST MODE: Desktop notification sent: {title}")
            else:
                logger.info(f"Desktop notification sent: {title}")
                
        except subprocess.TimeoutExpired:
            logger.warning("Desktop notification timed out")
        except subprocess.CalledProcessError as e:
            logger.warning(f"Failed to send desktop notification: {e}")
        except Exception as e:
            logger.warning(f"Unexpected error sending desktop notification: {e}")

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
                    subprocess.run(['wall', test_message], check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
                    logger.info(f"TEST MODE: Wall message sent: {message}")
                else:
                    # Use wall command to send message to all logged-in users
                    subprocess.run(['wall', message], check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
                    logger.info("Shutdown message broadcasted to all users")
            except subprocess.TimeoutExpired:
                logger.warning("Wall command timed out - message may not have been sent")
            except subprocess.CalledProcessError as e:
                logger.warning(f"Failed to broadcast wall message: {e}")
            except Exception as e:
                logger.warning(f"Unexpected error broadcasting wall message: {e}")
                    
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
            logger.info("TEST MODE: Shutdown command disabled - would normally execute: shutdown -h now")
            # In test mode, let the shutdown LED pattern run for demonstration
            logger.info(f"TEST MODE: Running shutdown LED pattern for {TEST_MODE_SHUTDOWN_DELAY_S} seconds...")
            time.sleep(TEST_MODE_SHUTDOWN_DELAY_S)  # Let the pattern run for demonstration
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
        logger.info("Button monitoring active - only new button presses after startup will trigger shutdown")
        
        # NOTE: Power relay control removed from this service
        # Relay is energized by argo_poweron.startup during boot
        
        # Start power button monitoring in separate thread
        logger.info("Starting hardware interrupt-based button detection")
        button_thread = threading.Thread(target=self.monitor_power_button_interrupts, daemon=True)
        button_thread.start()
        
        # Start green LED heartbeat in separate thread
        logger.info("Starting green LED heartbeat thread")
        heartbeat_thread = threading.Thread(target=self.green_led_heartbeat, daemon=True)
        heartbeat_thread.start()
        
        try:
            # Main loop - just keep running while monitoring threads handle GPIO
            while self.running:
                time.sleep(1.0)  # Sleep and let the interrupt threads do the work
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
            self.running = False
        
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up power controller...")
        
        try:
            # Turn off LEDs
            self.set_green_led(False)
            self.set_blue_led(False)
            
            # Close GPIO chip
            if hasattr(self, 'chip'):
                self.chip.close()
            
            # ROS2 cleanup
            try:
                self.get_logger().info("Shutting down ROS2 components...")
                self.destroy_node()
            except Exception as e:
                logger.error(f"Error during ROS2 cleanup: {e}")
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
        
        logger.info("Power controller cleanup complete")

def test_gradual_frequency_pattern(threshold):
    """Test the gradual frequency pattern"""
    print(f"Testing gradual frequency pattern for {threshold}s threshold...")
    print(f"Frequency will increase from {LED_PRESS_START_FREQUENCY_HZ}Hz to {LED_PRESS_END_FREQUENCY_HZ}Hz")
    print(f"Duty cycle: {LED_PRESS_DUTY_CYCLE * 100}%")
    
    start_time = time.time()
    
    while time.time() - start_time < threshold:
        # Calculate elapsed time and progress (0.0 to 1.0)
        elapsed_time = time.time() - start_time
        progress = min(elapsed_time / threshold, 1.0)
        
        # Calculate current frequency (2Hz to 20Hz)
        current_frequency = LED_PRESS_START_FREQUENCY_HZ + (LED_PRESS_END_FREQUENCY_HZ - LED_PRESS_START_FREQUENCY_HZ) * progress
        
        # Calculate timing for 50% duty cycle
        period = 1.0 / current_frequency
        on_time = period * LED_PRESS_DUTY_CYCLE
        off_time = period * (1.0 - LED_PRESS_DUTY_CYCLE)
        
        # Show current frequency every 0.5 seconds
        if int(elapsed_time * 2) != int((elapsed_time - 0.01) * 2):
            print(f"Time: {elapsed_time:.1f}s, Frequency: {current_frequency:.1f}Hz", flush=True)
        
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
  - Long press (>= threshold DEFAULT_SHUTDOWN_THRESHOLD_S): Initiate shutdown sequence
  - Only new button presses after service startup are detected (prevents shutdown during boot)

LED INDICATORS:
  - Green LED: Heartbeat when system is running
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
            subprocess.run(['wall', message], check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
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
                print(f"\n=== Testing gradual frequency pattern with {threshold}s threshold ===")
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
        print("  - Triple tap (3 quick taps within 1.5s) toggles recording")
    else:
        print("Starting power control system in NORMAL MODE")
        print("  - Full power control enabled")
        print("  - System will shutdown and cut power on long button press")
        print("  - Triple tap (3 quick taps within 1.5s) toggles recording")
    
    print(f"  - Button press threshold: {args.threshold} seconds")
    print(f"  - Button detection mode: Hardware interrupts (efficient)")
    print("  - Press Ctrl+C to stop")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        controller = PowerController(test_mode=args.test_mode, threshold=args.threshold)
        
        # Use MultiThreadedExecutor to handle ROS2 callbacks while running GPIO monitoring
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        # Start ROS2 executor in a separate thread (not daemon so it stays alive)
        executor_thread = threading.Thread(target=executor.spin, daemon=False)
        executor_thread.start()
        
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
        rclpy.shutdown()

if __name__ == "__main__":
    main()
