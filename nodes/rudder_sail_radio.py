#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
ROS2 Rudder/Sail Control Node - Combined Hardware Interface and Control Arbitration

This node provides a unified interface for rudder/sail control, combining hardware
interaction with the argo_radio_servo_module kernel module and intelligent control
arbitration between human and autonomous control systems.

Hardware Interface:
- Uses argo_radio_servo_module kernel module for hardware PWM capture/generation
- Interfaces via sysfs at /sys/kernel/argo_radio_servo/
- Supports Orange Pi Zero 2W with custom GPIO/PWM configuration
- Real-time radio control input capture and normalization
- Safe servo output with pulse width validation (900-2100µs range)
- HIGH IMPEDANCE MODE: Writing 0 to servo control files disables PWM output,
  allowing radio control to pass through directly to servos via resistor network

Safety features:
- The radio channels are connected directly to the servo outputs by resistors. This way, if
  radio_sail_servo is not running, the PWM output pins are set to high impedance, so that the radio 
  directly drives the servo outputs. When radio_sail_servo is running, the radio inputs are passed through when
  the boat is human controlled. Under argo auto mode, the servos are controlled by controller.py.
- FAIL-SAFE DESIGN: Servo outputs start in high impedance mode (PWM disabled) for maximum safety
- HIGH IMPEDANCE MODE: Servos automatically switch to high impedance when not actively controlled

Control Arbitration:
- Intelligent human/robot control switching with human priority
- Automatic detection of human activity based on radio input changes
- Configurable timeout-based handover between control modes
- Safety limits and deadband filtering for robust operation
- High-frequency control loop (20Hz) for responsive arbitration

Key Features:
- Combined hardware interface and control logic in single node
- Default QoS for all topics (compatible with systemd and normal processes)
- Throttled logging to minimize system load
- Built-in safety features with high impedance defaults
- Graceful handling of hardware disconnection
- HIGH IMPEDANCE MODE: Automatic switching between PWM control and radio passthrough

Topics Published:
- /rudder_sail_radio: Vector3 with normalized radio inputs (-1 to +1)
  * x: rudder position (-1=full left, +1=full right, looking down at boat)
  * y: sail position (-1=pulled in fully, +1=let out fully)
  * z: reserved (currently 0)
  
- /rudder_sail_servo: Vector3 with actual servo commands being sent to hardware
  * Same format as radio topic but reflects actual servo positions
  
- /human_controlled: Bool indicating current control state (default QoS)
  * True: Human has control (radio inputs active)
  * False: Computer has control (autonomous navigation active)
  
- /control_authority: Vector3 with detailed control status
  * x: Current authority (1=human, 0=robot)
  * y: Time since human activity
  * z: Time since auto command
  
- /rudder_sail_radio_health: Bool indicating node health status
  * true: node is healthy and hardware is accessible
  * false: node has failed or hardware is not accessible

Topics Subscribed:
- /rudder_sail_cmd: Vector3 with autonomous navigation commands (-1 to +1 range)
  * Commands from controller.py or other autonomous navigation nodes
  * Only applied when human_controlled is False

Hardware Requirements:
- argo_radio_servo_module kernel module loaded (v0.5+ with high impedance support)
- GPIO pins configured for radio input capture (PI11, PI13)
- PWM outputs configured for servo control (PI12=PWM2, PI14=PWM4)
- RC receiver connected and calibrated for 1000-2000µs pulse width range

Safety Features:
- HIGH IMPEDANCE MODE: Servo outputs default to high impedance (PWM disabled)
- Pulse width validation and clamping (900-2100µs hardware range)
- Outlier radio input filtering (500-2500µs acceptance range)
- Automatic fallback to high impedance mode on invalid inputs
- Throttled error logging to prevent log spam
- Graceful handling of hardware disconnection
- FAIL-SAFE: Radio control always works when PWM is disabled

Author: Tobi Delbruck (original pwm.py), Enhanced with control arbitration and high impedance mode
License: MIT
Version: 3.2 - Enhanced control logging, fail-safe exit handling, and human control timeout constant
"""

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))

# Import ArgoBaseNode for standardized functionality
from argo_base_node import ArgoBaseNode

# Removed toggle_pause_service import - rudder_sail_radio node doesn't need pause functionality
import rclpy
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import argparse
import argcomplete
from pathlib import Path
import time
import numpy as np
import sys
import signal
import tty
import termios
import select
import cProfile
import pstats
from datetime import datetime

# --- Hardware Configuration ---
SYS_BASE_PATH = Path("/sys/kernel/argo_radio_servo")
RADIO_RUDDER_PATH = SYS_BASE_PATH / "radio_rudder_pw_us"
RADIO_SAIL_PATH = SYS_BASE_PATH / "radio_sail_pw_us"
SERVO_RUDDER_PATH = SYS_BASE_PATH / "servo_rudder_pw_us"
SERVO_SAIL_PATH = SYS_BASE_PATH / "servo_sail_pw_us"

# Kernel module enforces 900-2100µs range for servo outputs
SERVO_MIN_PW_US = 900
SERVO_MAX_PW_US = 2100

# Throttle logging for clamped values to once per minute
CLAMP_LOG_THROTTLE_S = 60.0

# Throttle logging for outlier radio PWM messages to once every 10 seconds
OUTLIER_LOG_THROTTLE_S = 60.0

# Human control timeout - seconds after last human activity before robot can take control
HUMAN_CONTROL_TIMEOUT_S = 2.0

# Default parameter values
# NOTE: These defaults are only used if the parameter is not set in the YAML configuration file.
# The launch system loads parameters from nodes/argo.yaml via --params-file,
# which overrides these defaults. To change the value, edit the YAML file or use ros2 param set.
# YAML file location: nodes/argo.yaml (rudder_sail_radio_node section)
DEFAULT_DEADBAND_THRESHOLD = 0.1875 # servo command threshold away from zero rudder on scale -1 to +1 for human control to take over (5σ measured)
DEFAULT_SAFETY_MAX_RUDDER = 1.0 # limit rudder comamnd to this magnitude in case of mechanical constraints
DEFAULT_SAFETY_MAX_SAIL = 1.0 # limit sail comamnd to this magnitude in case of mechanical constraints

# Control loop timing constants
DEFAULT_CONTROL_LOOP_PERIOD = 0.05  # 20 Hz for responsive control (active)
DEFAULT_CONTROL_LOOP_PERIOD_IDLE = 0.20  # 5 Hz when idle (reduces executor overhead)
DEFAULT_STATUS_PERIOD = 0.1  # 10 Hz status updates (active)
DEFAULT_STATUS_PERIOD_IDLE = 1.0  # 1 Hz when idle (reduces executor overhead)

# Test mode configuration
TEST_MIN_PW = 900
TEST_MAX_PW = 2100
TEST_STEP = 100
TEST_DELAY_S = 0.5
TEST_DEFAULT_PW = 1500


def cmd_to_pw_us(cmd: float) -> int:
    """Converts a normalized command (-1 to +1) to a pulse width in microseconds (1000 to 2000).
    
    Rudder convention: -1 = full left, 0 = center, +1 = full right (looking down at boat)
    Sail convention: -1 = pulled in fully, 0 = half out, +1 = let out fully
    PWM convention: 1000us = -1, 1500us = 0, 2000us = +1
    """
    # Clamp command to [-1, 1]
    cmd = max(-1.0, min(1.0, cmd))
    # Linear interpolation: 1500 is center, 500 is range on each side
    pw = 1500 + 500 * cmd
    return int(pw)


def pw_us_to_cmd(pw_us: float) -> float:
    """Converts a pulse width in microseconds (1000 to 2000) to a normalized command (-1 to +1).
    
    Rudder convention: -1 = full left, 0 = center, +1 = full right (looking down at boat)
    Sail convention: -1 = pulled in fully, 0 = half out, +1 = let out fully
    PWM convention: 1000us = -1, 1500us = 0, 2000us = +1
    """
    # Clamp pulse width to [1000, 2000] for stable conversion
    pw_us = max(1000.0, min(2000.0, pw_us))
    # Linear interpolation
    cmd = (pw_us - 1500.0) / 500.0
    return cmd

# Test mode helper functions


def get_initial_pw_test(path: Path) -> int:
    """Reads the initial pulse width from a sysfs file, or returns a default."""
    try:
        return int(path.read_text().strip())
    except (IOError, ValueError, FileNotFoundError):
        print(
            f"Warning: Could not read {path}, using default {TEST_DEFAULT_PW} us.", file=sys.stderr)
        return TEST_DEFAULT_PW


def read_sysfs_pw_test(path: Path) -> str:
    """Reads a pulse width from a sysfs file for display."""
    try:
        return path.read_text().strip()
    except (IOError, FileNotFoundError):
        return "N/A"


def write_sysfs_pw_test(path: Path, value: int):
    """Writes a pulse width to a sysfs file for test mode."""
    # Apply same clamping as normal mode
    value = max(SERVO_MIN_PW_US, min(SERVO_MAX_PW_US, value))
    try:
        path.write_text(str(value))
    except IOError as e:
        print(f"\nError writing to {path}: {e}", file=sys.stderr)


def display_test_status(rudder_pw: int, sail_pw: int, paused: bool):
    """Clears the screen and displays the current test status."""
    # ANSI escape code to clear screen and move cursor to top-left
    print("\033[H\033[J", end="")

    print("--- Radio Control Input Pulse Widths ---")
    print(f"Radio Rudder: {read_sysfs_pw_test(RADIO_RUDDER_PATH)} us")
    print(f"Radio Sail:   {read_sysfs_pw_test(RADIO_SAIL_PATH)} us")
    print("----------------------------------------")
    print("--- Servo Motor Output Pulse Widths (Sweeping) ---")
    print(f"Rudder (PWM2): {rudder_pw} us")
    print(f"Sail (PWM4):   {sail_pw} us")
    print("--------------------------------------------------")
    if paused:
        print("STATUS: PAUSED (Press Spacebar to RESUME)")
    else:
        print("STATUS: RUNNING (Press Spacebar to PAUSE)")
    sys.stdout.flush()


def get_key_non_blocking() -> str | None:
    """Reads a single key press without blocking. Returns None if no key is pressed."""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None


def get_key_blocking() -> str:
    """Reads a single key press, blocking until one is received."""
    return sys.stdin.read(1)


def run_test_mode():
    """Runs the PWM test mode - sweeps servo outputs and displays radio inputs."""
    # Check if sysfs path exists for better error reporting
    if not SYS_BASE_PATH.is_dir():
        print(f"Error: Sysfs path {SYS_BASE_PATH} not found.", file=sys.stderr)
        print("Is the 'argo_radio_servo_module' kernel module loaded?", file=sys.stderr)
        sys.exit(1)

    # Initialize state variables
    current_rudder_pw = get_initial_pw_test(SERVO_RUDDER_PATH)
    current_sail_pw = get_initial_pw_test(SERVO_SAIL_PATH)
    direction_rudder = 1  # 1 for increasing, -1 for decreasing
    direction_sail = 1
    paused = False

    print("Starting Argo Radio Servo PWM Test Mode...")
    print("Monitoring input pulse widths and sweeping output servo positions.")
    print("Press Spacebar to PAUSE/RESUME. Press Ctrl+C to STOP.")
    time.sleep(1.5)  # Give user time to read the intro message

    # Terminal Setup
    # Save original terminal settings to restore them on exit
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Set terminal to "cbreak" mode to read keys instantly without requiring Enter
        tty.setcbreak(sys.stdin.fileno())

        # Main Loop
        while True:
            display_test_status(current_rudder_pw, current_sail_pw, paused)

            # Handle Input for Pause/Resume
            key_pressed = get_key_non_blocking()

            if key_pressed == ' ':
                paused = not paused
                # Update status immediately
                display_test_status(current_rudder_pw, current_sail_pw, paused)

                if paused:
                    print("\nPAUSED. Press Spacebar to RESUME...")
                    sys.stdout.flush()
                    # When paused, enter a blocking loop that waits *only* for a spacebar.
                    while True:
                        key_to_resume = get_key_blocking()
                        if key_to_resume == ' ':
                            paused = not paused
                            display_test_status(
                                current_rudder_pw, current_sail_pw, paused)
                            break  # Exit this inner blocking loop

            if not paused:
                # Update Rudder (PWM2) Pulse Width
                current_rudder_pw += direction_rudder * TEST_STEP
                if current_rudder_pw > TEST_MAX_PW:
                    current_rudder_pw = TEST_MAX_PW
                    direction_rudder = -1
                elif current_rudder_pw < TEST_MIN_PW:
                    current_rudder_pw = TEST_MIN_PW
                    direction_rudder = 1
                write_sysfs_pw_test(SERVO_RUDDER_PATH, current_rudder_pw)

                # Update Sail (PWM4) Pulse Width
                current_sail_pw += direction_sail * TEST_STEP
                if current_sail_pw > TEST_MAX_PW:
                    current_sail_pw = TEST_MAX_PW
                    direction_sail = -1
                elif current_sail_pw < TEST_MIN_PW:
                    current_sail_pw = TEST_MIN_PW
                    direction_sail = 1
                write_sysfs_pw_test(SERVO_SAIL_PATH, current_sail_pw)

            time.sleep(TEST_DELAY_S)

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Exiting test mode.")
    finally:
        # Restore Terminal
        # This block ensures terminal settings are always restored
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Terminal settings restored.")


class RudderSailRadioNode(ArgoBaseNode):
    """
    Combined hardware interface and control arbitration node for rudder/sail control.

    Key responsibilities:
    1. Interface with argo_radio_servo_module kernel module via sysfs
    2. Read radio control inputs and normalize to -1 to +1 range
    3. Receive autonomous control commands from controller.py
    4. Arbitrate between human and robot control with human priority
    5. Write final servo commands to hardware with safety validation
    6. Publish control status and authority information
    7. HIGH IMPEDANCE MODE: Set servos to high impedance when not actively controlling
    """

    def __init__(self, debug_mode: bool = False):
        super().__init__('rudder_sail_radio_node')

        # Set default logging level to INFO
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Initialize pause service with namespaced name
        # Removed pause service - rudder_sail_radio node doesn't need pause functionality

        self.get_logger().info(
            'Rudder/Sail Radio node starting with high impedance safety mode...')

        # Check for sysfs directory
        if not SYS_BASE_PATH.is_dir():
            self.get_logger().error(f"Sysfs path {SYS_BASE_PATH} not found.")
            self.get_logger().error("Is the 'argo_radio_servo_module' kernel module loaded?")
            rclpy.shutdown()
            return

        # --- Parameters ---
        self.declare_parameter('human_override_timeout',
                               HUMAN_CONTROL_TIMEOUT_S)  # seconds
        # ignore small radio movements
        self.declare_parameter('deadband_threshold', DEFAULT_DEADBAND_THRESHOLD)
        self.declare_parameter('safety_max_rudder', DEFAULT_SAFETY_MAX_RUDDER)       # safety limits
        self.declare_parameter('safety_max_sail', DEFAULT_SAFETY_MAX_SAIL)

        # Load parameter values
        self.human_override_timeout = self.get_parameter(
            'human_override_timeout').get_parameter_value().double_value
        self.deadband_threshold = self.get_parameter(
            'deadband_threshold').get_parameter_value().double_value
        self.safety_max_rudder = self.get_parameter(
            'safety_max_rudder').get_parameter_value().double_value
        self.safety_max_sail = self.get_parameter(
            'safety_max_sail').get_parameter_value().double_value
        
        # Log parameter values for debugging
        self.get_logger().info(
            f"Control authority parameters: deadband_threshold={self.deadband_threshold:.3f}, "
            f"human_override_timeout={self.human_override_timeout:.1f}s, "
            f"safety_max_rudder={self.safety_max_rudder:.2f}, safety_max_sail={self.safety_max_sail:.2f}")
        
        # Warn if deadband_threshold doesn't match expected default
        if abs(self.deadband_threshold - DEFAULT_DEADBAND_THRESHOLD) > 0.001:
            self.get_logger().warn(
                f"⚠️  deadband_threshold={self.deadband_threshold:.3f} does not match default={DEFAULT_DEADBAND_THRESHOLD:.3f}. "
                f"ROS2 may be using a stored parameter value. To reset, use: "
                f"ros2 param set /rudder_sail_radio_node deadband_threshold {DEFAULT_DEADBAND_THRESHOLD}")
        
        # Add parameter callback to handle runtime parameter changes
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # Enhanced control logging state
        self.last_control_switch_time = time.time()
        self.control_switch_reason = "initialization"

        # --- State Variables ---
        # Radio input (from hardware via sysfs)
        self.radio_rudder = 0.0
        self.radio_sail = 0.0
        self.last_radio_update = 0.0

        # Autonomous control input (from controller.py)
        self.auto_rudder = 0.0
        self.auto_sail = 0.0
        self.last_auto_update = 0.0

        # Control arbitration state
        # Initialize based on radio input availability (will be updated after first radio read)
        self.human_controlled = False  # Start as robot control, will be set to True if radio input exists
        self.last_human_activity = 0.0  # Initialize to 0, will be set when actual radio activity is detected
        self.last_logged_control_mode = None

        # Track previous radio values for human activity detection
        self.prev_radio_rudder = 0.0
        self.prev_radio_sail = 0.0
        
        # Track if radio was previously invalid (0 or out of range) to detect radio turn-on
        self._radio_was_invalid = True  # Start assuming radio is off/invalid

        # Throttling for clamp warning logs
        self.last_clamp_warning_time = 0.0

        # Throttling for outlier radio PWM warning logs
        self.last_outlier_warning_time = 0.0
        
        # Track previous radio health state for change detection
        self.prev_radio_healthy = None  # None = unknown, True = healthy, False = unhealthy
        
        # Debug logging throttling (log 1 out of every 100 messages)
        self._debug_control_authority_counter = 0
        self._debug_human_activity_counter = 0
        self._debug_small_change_counter = 0
        
        # Change detection for publishing optimization
        self.prev_published_radio_rudder = None
        self.prev_published_radio_sail = None
        self.prev_published_servo_rudder = None
        self.prev_published_servo_sail = None
        self.last_radio_publish_time = 0.0
        self.last_servo_publish_time = 0.0
        self.last_status_publish_time = 0.0
        self.prev_published_human_controlled = None
        self.prev_published_control_authority = None
        
        # Minimum change threshold for publishing (1% of full range)
        self.PUBLISH_CHANGE_THRESHOLD = 0.01  # 1% of -1 to +1 range
        # Maximum time between publishes even if no change (ensure subscribers know we're alive)
        self.MAX_PUBLISH_INTERVAL = 5.0  # 5 seconds
        
        # Cache message objects to avoid allocation overhead
        self._radio_msg_cache = Vector3()
        self._servo_msg_cache = Vector3()
        self._human_msg_cache = Bool()
        self._authority_msg_cache = Vector3()
        self._prev_authority_cache = Vector3()  # For comparison
        
        # Sysfs read caching to reduce file I/O overhead
        self._cached_radio_rudder_pw_us = 0.0
        self._cached_radio_sail_pw_us = 0.0
        self._last_sysfs_read_time = 0.0
        self._last_radio_value_change_time = 0.0
        # Adaptive read interval: faster when values changing, slower when stable
        self.SYSFS_READ_INTERVAL_ACTIVE = 0.05  # 20Hz when active (50ms)
        self.SYSFS_READ_INTERVAL_IDLE = 0.20    # 5Hz when idle (200ms) - reduces I/O by 75%

        # --- QoS Profiles ---

        # --- Publishers ---
        # Radio input from hardware (normalized)
        self.pub_rudder_sail_radio = self.create_publisher(
            Vector3, '/rudder_sail_radio', 10)

        # Final servo commands to hardware
        self.pub_rudder_sail_servo = self.create_publisher(
            Vector3, '/rudder_sail_servo', 10)

        # Status for other nodes
        self.pub_human_controlled = self.create_publisher(
            Bool, '/human_controlled', 10)
        self.pub_control_authority = self.create_publisher(
            Vector3, '/control_authority', 10)

        # Health status is now handled by ArgoBaseNode

        # --- Subscribers ---
        # From controller.py (autonomous commands)
        self.create_subscription(
            Vector3, '/rudder_sail_cmd', self.auto_control_callback, 10)

        # --- Timers ---
        self.control_loop_period = DEFAULT_CONTROL_LOOP_PERIOD  # 20 Hz for responsive control
        self.timer = self.create_timer(
            self.control_loop_period, self.timer_callback)

        # Set initial health status as unhealthy (no radio input yet)
        self.set_unhealthy("No radio input detected yet")

        # Status publishing timer
        self.status_period = DEFAULT_STATUS_PERIOD  # 10 Hz status updates
        self.status_timer = self.create_timer(
            self.status_period, self.publish_status)
        
        # Adaptive timer frequency tracking
        self._last_activity_time = time.time()
        self._is_idle_mode = False
        self._timer_adjustment_interval = 5.0  # Check for idle mode every 5 seconds
        self._last_timer_adjustment = time.time()

        # Initialize servos to high impedance mode for safety
        self.set_servo_high_impedance()
        
        # Do an initial radio read to set human_controlled based on radio availability
        # This ensures the published topic reflects the actual radio state at startup
        try:
            initial_rudder = self.read_sysfs_pw(RADIO_RUDDER_PATH)
            initial_sail = self.read_sysfs_pw(RADIO_SAIL_PATH)
            if (500 < initial_rudder < 2500 and 500 < initial_sail < 2500):
                # Radio input is available - human has control
                self.human_controlled = True
                self.last_human_activity = time.time()
                self._radio_was_invalid = False
                self.get_logger().info("Radio input detected at startup - Human control enabled")
            else:
                # Radio input not available - robot control
                self.human_controlled = False
                self._radio_was_invalid = True
                self.get_logger().info("No radio input at startup - Robot control enabled")
        except Exception as e:
            # If we can't read radio at startup, assume it's off
            self.human_controlled = False
            self._radio_was_invalid = True
            self.get_logger().debug(f"Could not read radio at startup: {e} - assuming robot control")

        # Register cleanup handlers for graceful shutdown
        import atexit
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        atexit.register(self._ensure_safe_exit)

    def read_sysfs_pw(self, path: Path) -> float:
        """Reads a pulse width from a sysfs file using optimized file I/O."""
        try:
            # Use lower-level file operations for better performance
            # sysfs files are small (<20 bytes), so reading directly is faster
            with open(path, 'r') as f:
                return float(f.read().strip())
        except (IOError, FileNotFoundError, ValueError) as e:
            # Throttle error logging to prevent spam
            now = time.time()
            if not hasattr(self, '_last_sysfs_error_time'):
                self._last_sysfs_error_time = {}
            if path not in self._last_sysfs_error_time or (now - self._last_sysfs_error_time[path]) > 60.0:
                self.get_logger().warn(f"Could not read or parse {path}: {e}")
                self._last_sysfs_error_time[path] = now
            return 0.0  # Return a safe, invalid value

    def write_sysfs_pw(self, path: Path, value: int):
        """Writes a pulse width to a sysfs file, with special handling for high impedance mode."""
        original_value = value

        # Special case: 0 means high impedance mode (no clamping)
        if value == 0:
            try:
                path.write_text(str(value))
                self.get_logger().info(
                    f"Set {path.name} to HIGH IMPEDANCE mode (radio control active)")
                return
            except IOError as e:
                self.get_logger().error(
                    f"Error setting high impedance mode for {path}: {e}")
                return

        # For non-zero values, apply normal clamping
        value = max(SERVO_MIN_PW_US, min(SERVO_MAX_PW_US, value))

        # Log clamping with throttling (once per minute max)
        if value != original_value:
            now = time.time()
            if now - self.last_clamp_warning_time > CLAMP_LOG_THROTTLE_S:
                self.get_logger().warn(
                    f"Clamped servo pulse width from {original_value}µs to {value}µs "
                    f"(valid range: {SERVO_MIN_PW_US}-{SERVO_MAX_PW_US}µs). "
                    f"Further clamp warnings suppressed for {CLAMP_LOG_THROTTLE_S}s."
                )
                self.last_clamp_warning_time = now

        try:
            path.write_text(str(value))
            # Removed debug logging to reduce CPU overhead (runs at 20Hz)
        except IOError as e:
            self.get_logger().error(f"Error writing to {path}: {e}")

    def set_servo_high_impedance(self, rudder: bool = True, sail: bool = True):
        """Set servo outputs to high impedance mode for safety."""
        if rudder:
            self.write_sysfs_pw(SERVO_RUDDER_PATH, 0)
        if sail:
            self.write_sysfs_pw(SERVO_SAIL_PATH, 0)
        self.get_logger().info("Servo outputs set to HIGH IMPEDANCE mode (radio control active)")

    def read_radio_inputs(self):
        """Read and process radio inputs from hardware with adaptive caching to reduce I/O overhead."""
        current_time = time.time()
        
        # Determine read interval based on activity: faster when values changing, slower when idle
        time_since_last_change = current_time - self._last_radio_value_change_time
        is_active = time_since_last_change < 2.0  # Active if changed in last 2 seconds
        read_interval = self.SYSFS_READ_INTERVAL_ACTIVE if is_active else self.SYSFS_READ_INTERVAL_IDLE
        
        # Only read from sysfs if cache is stale (adaptive interval reduces I/O when idle)
        if (current_time - self._last_sysfs_read_time) >= read_interval:
            # Batch read both sysfs files together for better performance
            try:
                # Read rudder
                with open(RADIO_RUDDER_PATH, 'r') as f:
                    self._cached_radio_rudder_pw_us = float(f.read().strip())
                # Read sail
                with open(RADIO_SAIL_PATH, 'r') as f:
                    self._cached_radio_sail_pw_us = float(f.read().strip())
                self._last_sysfs_read_time = current_time
            except (IOError, FileNotFoundError, ValueError) as e:
                # Fallback to individual read methods for error handling
                self._cached_radio_rudder_pw_us = self.read_sysfs_pw(RADIO_RUDDER_PATH)
                self._cached_radio_sail_pw_us = self.read_sysfs_pw(RADIO_SAIL_PATH)
                self._last_sysfs_read_time = current_time
        
        # Use cached values
        radio_rudder_pw_us = self._cached_radio_rudder_pw_us
        radio_sail_pw_us = self._cached_radio_sail_pw_us

        # Validate and normalize radio inputs
        if not (500 < radio_rudder_pw_us < 2500 and 500 < radio_sail_pw_us < 2500):
            # Mark radio as invalid (off or out of range)
            self._radio_was_invalid = True
            
            # Determine the specific cause of health failure
            # Use normalized messages to prevent log spam from changing values
            if radio_rudder_pw_us == 0.0 and radio_sail_pw_us == 0.0:
                failure_reason = "Radio transmitter off or disconnected (both channels 0.0us)"
            elif radio_rudder_pw_us == 0.0:
                failure_reason = "Radio rudder channel off/disconnected (0.0us)"
            elif radio_sail_pw_us == 0.0:
                failure_reason = "Radio sail channel off/disconnected (0.0us)"
            else:
                failure_reason = f"Outlier radio PWM: rudder={radio_rudder_pw_us:.1f}us, sail={radio_sail_pw_us:.1f}us"

            # Check if health state changed
            health_state_changed = self.prev_radio_healthy is not False
            now = time.time()
            
            # Log immediately on state change, otherwise throttle to 1 per minute
            if health_state_changed or (now - self.last_outlier_warning_time > OUTLIER_LOG_THROTTLE_S):
                self.get_logger().warn(
                    f"Radio input validation failed: {failure_reason}")
                self.last_outlier_warning_time = now
            
            # Update health state tracking
            self.prev_radio_healthy = False
            
            # Update health status (ArgoBaseNode will only log if status/details changed)
            self.set_unhealthy(failure_reason)
            return False

        # Convert radio inputs using standard servo convention:
        # Rudder: 1000us -> -1 (full left), 1500us -> 0 (center), 2000us -> +1 (full right)
        self.radio_rudder = pw_us_to_cmd(radio_rudder_pw_us)
        # Sail: 1000us -> -1 (pulled in), 1500us -> 0 (half out), 2000us -> +1 (let out)
        self.radio_sail = pw_us_to_cmd(radio_sail_pw_us)

        self.last_radio_update = time.time()

        # Check if radio just switched on (transition from invalid/0 to valid)
        # This immediately grants human control when radio is turned on
        if self._radio_was_invalid:
            # Radio was previously invalid/off, now it's valid - human just turned on radio
            self.last_human_activity = time.time()
            self.human_controlled = True
            self._radio_was_invalid = False
            self.get_logger().info(
                f"Radio input detected - Human control immediately granted (rudder: {self.radio_rudder:.3f}, sail: {self.radio_sail:.3f})")
            # Update previous values to current so we don't trigger false change detection
            self.prev_radio_rudder = self.radio_rudder
            self.prev_radio_sail = self.radio_sail
            return True

        # Detect human activity based on significant radio input changes
        rudder_change = abs(self.radio_rudder - self.prev_radio_rudder)
        sail_change = abs(self.radio_sail - self.prev_radio_sail)
        
        # Store change values for periodic status logging
        self._last_rudder_change = rudder_change
        self._last_sail_change = sail_change

        if rudder_change > self.deadband_threshold or sail_change > self.deadband_threshold:
            self.last_human_activity = time.time()
            self._last_radio_value_change_time = time.time()  # Track for adaptive read frequency
            self._last_activity_time = time.time()  # Track for adaptive timer frequencies (wake up!)
            if not self.human_controlled:
                self.get_logger().info(
                    f"Human activity detected (rudder: {rudder_change:.3f}, sail: {sail_change:.3f}, deadband: {self.deadband_threshold:.3f})")
            else:
                # Throttle debug logging to 1 in 100 messages
                self._debug_human_activity_counter += 1
                if self._debug_human_activity_counter % 100 == 0:
                    self.get_logger().debug(
                        f"Human activity continues (rudder: {rudder_change:.3f}, sail: {sail_change:.3f}, deadband: {self.deadband_threshold:.3f})")
        elif rudder_change > 0.001 or sail_change > 0.001:  # Small changes also update timestamp
            self._last_radio_value_change_time = time.time()  # Track any change for adaptive reads
            self._last_activity_time = time.time()  # Even small radio changes wake up timers
            # Throttle debug logging to 1 in 100 messages
            self._debug_small_change_counter += 1
            if self._debug_small_change_counter % 100 == 0:
                self.get_logger().debug(
                    f"Small radio change (rudder: {rudder_change:.3f}, sail: {sail_change:.3f}, deadband: {self.deadband_threshold:.3f}) - not enough to update human_activity")

        # Update previous values for next comparison
        self.prev_radio_rudder = self.radio_rudder
        self.prev_radio_sail = self.radio_sail

        # Set health status to healthy when radio inputs are valid
        # ArgoBaseNode will log automatically when state changes from unhealthy to healthy
        self.set_healthy("Radio inputs within valid range")
        
        # Update health state tracking
        self.prev_radio_healthy = True

        return True

    def auto_control_callback(self, msg):
        """Receive autonomous control commands from controller.py."""
        # Always process autonomous commands, even when paused (for safety)
        self.auto_rudder = msg.x
        self.auto_sail = msg.y
        self.last_auto_update = time.time()
        self._last_activity_time = time.time()  # Track activity for adaptive timers

    def determine_control_authority(self):
        """
        Determine who has control authority based on human activity.

        Human gets priority when:
        1. Recent human activity detected (within timeout period)
        2. Radio input is available and changing

        Robot gets control when:
        1. No human activity for timeout period
        2. Autonomous commands are being received

        Returns:
            tuple: (human_controlled: bool, reason: str)
        """
        current_time = time.time()

        # Check for recent human activity
        if self.last_human_activity == 0.0:
            # No human activity detected yet, robot can take control immediately
            reason = "no_human_activity_detected_yet"
            # Throttle debug logging to 1 in 100 messages
            self._debug_control_authority_counter += 1
            if self._debug_control_authority_counter % 100 == 0:
                self.get_logger().debug(f"Control authority: ROBOT (last_human_activity=0.0, deadband_threshold={self.deadband_threshold:.3f})")
            return False, reason  # Robot control
        
        time_since_human_activity = current_time - self.last_human_activity
        
        # Human has control if there's been recent activity
        if time_since_human_activity < self.human_override_timeout:
            reason = f"human_activity_within_{self.human_override_timeout:.1f}s (last_activity: {time_since_human_activity:.1f}s ago)"
            # Throttle debug logging to 1 in 100 messages
            self._debug_control_authority_counter += 1
            if self._debug_control_authority_counter % 100 == 0:
                self.get_logger().debug(f"Control authority: HUMAN (time_since={time_since_human_activity:.2f}s < timeout={self.human_override_timeout:.1f}s, deadband={self.deadband_threshold:.3f})")
            return True, reason  # Human control

        # Robot can take control after human timeout expires
        # This allows testing control handover behavior even when controller.py is not running
        reason = f"timeout_handover_after_{time_since_human_activity:.1f}s"

        # Check if we have recent autonomous commands to provide more context
        time_since_auto_update = current_time - self.last_auto_update
        if time_since_auto_update < 1.0:  # Auto commands are fresh
            reason += " (autonomous_commands_active)"
        elif time_since_auto_update < 30.0:  # Had autonomous commands recently
            reason += " (autonomous_standby_mode)"
        else:
            reason += " (no_autonomous_controller)"

        # Throttle debug logging to 1 in 100 messages
        self._debug_control_authority_counter += 1
        if self._debug_control_authority_counter % 100 == 0:
            self.get_logger().debug(f"Control authority: ROBOT (time_since={time_since_human_activity:.2f}s >= timeout={self.human_override_timeout:.1f}s, deadband={self.deadband_threshold:.3f})")
        return False, reason  # Robot control

    def _on_parameters_set(self, parameters):
        """Callback for parameter changes - handles runtime parameter updates."""
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            param_name = param.name
            
            # Get parameter value - handle both ParameterValue object and direct value
            if hasattr(param, 'value'):
                param_value = param.value
            else:
                param_value = param.get_parameter_value()
            
            # Extract double value from ParameterValue object or use directly
            if hasattr(param_value, 'double_value'):
                double_value = param_value.double_value
            elif isinstance(param_value, (int, float)):
                double_value = float(param_value)
            else:
                self.get_logger().warn(f"Parameter {param_name} has unexpected value type: {type(param_value)}")
                continue
            
            if param_name == 'deadband_threshold':
                old_value = self.deadband_threshold
                self.deadband_threshold = double_value
                self.get_logger().info(
                    f"✅ Parameter updated: deadband_threshold changed from {old_value:.3f} to {double_value:.3f}")
                
            elif param_name == 'human_override_timeout':
                old_value = self.human_override_timeout
                self.human_override_timeout = double_value
                self.get_logger().info(
                    f"✅ Parameter updated: human_override_timeout changed from {old_value:.1f}s to {double_value:.1f}s")
                
            elif param_name == 'safety_max_rudder':
                old_value = self.safety_max_rudder
                self.safety_max_rudder = double_value
                self.get_logger().info(
                    f"✅ Parameter updated: safety_max_rudder changed from {old_value:.2f} to {double_value:.2f}")
                
            elif param_name == 'safety_max_sail':
                old_value = self.safety_max_sail
                self.safety_max_sail = double_value
                self.get_logger().info(
                    f"✅ Parameter updated: safety_max_sail changed from {old_value:.2f} to {double_value:.2f}")
        
        return result

    def apply_safety_limits(self, rudder, sail):
        """Apply safety limits to control commands."""
        rudder = np.clip(rudder, -self.safety_max_rudder,
                         self.safety_max_rudder)
        sail = np.clip(sail, -self.safety_max_sail, self.safety_max_sail)
        return rudder, sail

    # Health status publishing is now handled by ArgoBaseNode

    def _adjust_timer_frequencies(self, current_time):
        """Adjust timer frequencies based on system activity to reduce executor overhead."""
        # Only check periodically to avoid overhead
        if (current_time - self._last_timer_adjustment) < self._timer_adjustment_interval:
            return
        
        self._last_timer_adjustment = current_time
        
        # Determine if system is idle (no recent activity)
        time_since_activity = current_time - self._last_activity_time
        should_be_idle = time_since_activity > 5.0  # Idle if no activity for 5 seconds
        
        # Adjust timers if state changed
        if should_be_idle != self._is_idle_mode:
            self._is_idle_mode = should_be_idle
            
            if self._is_idle_mode:
                # Switch to idle frequencies (reduces executor overhead by ~75%)
                new_control_period = DEFAULT_CONTROL_LOOP_PERIOD_IDLE
                new_status_period = DEFAULT_STATUS_PERIOD_IDLE
                self.get_logger().debug(f"Switching to idle mode: control={1/new_control_period:.1f}Hz, status={1/new_status_period:.1f}Hz")
            else:
                # Switch to active frequencies
                new_control_period = DEFAULT_CONTROL_LOOP_PERIOD
                new_status_period = DEFAULT_STATUS_PERIOD
                self.get_logger().debug(f"Switching to active mode: control={1/new_control_period:.1f}Hz, status={1/new_status_period:.1f}Hz")
            
            # Update timer periods (must cancel and recreate)
            if abs(self.control_loop_period - new_control_period) > 0.001:
                self.timer.cancel()
                self.control_loop_period = new_control_period
                self.timer = self.create_timer(self.control_loop_period, self.timer_callback)
            
            if abs(self.status_period - new_status_period) > 0.001:
                self.status_timer.cancel()
                self.status_period = new_status_period
                self.status_timer = self.create_timer(self.status_period, self.publish_status)

    def timer_callback(self):
        """Main control arbitration and hardware interface loop."""
        # Removed pause functionality - rudder_sail_radio runs continuously
        current_time = time.time()
        
        # Adjust timer frequencies based on activity (reduces executor overhead when idle)
        self._adjust_timer_frequencies(current_time)

        # 1. Read radio inputs from hardware
        if not self.read_radio_inputs():
            return  # Skip this cycle if radio inputs are invalid

        # 2. Publish normalized radio inputs (only on significant change or timeout)
        radio_changed = (
            self.prev_published_radio_rudder is None or
            abs(self.radio_rudder - self.prev_published_radio_rudder) > self.PUBLISH_CHANGE_THRESHOLD or
            abs(self.radio_sail - self.prev_published_radio_sail) > self.PUBLISH_CHANGE_THRESHOLD or
            (current_time - self.last_radio_publish_time) > self.MAX_PUBLISH_INTERVAL
        )
        
        if radio_changed:
            self._radio_msg_cache.x = self.radio_rudder
            self._radio_msg_cache.y = self.radio_sail
            self._radio_msg_cache.z = 0.0
            self.pub_rudder_sail_radio.publish(self._radio_msg_cache)
            self.prev_published_radio_rudder = self.radio_rudder
            self.prev_published_radio_sail = self.radio_sail
            self.last_radio_publish_time = current_time
            self._last_activity_time = current_time  # Track activity for adaptive timers

        # 3. Determine control authority
        new_human_controlled, authority_reason = self.determine_control_authority()

        # 4. Log control mode changes with detailed reason
        if new_human_controlled != self.last_logged_control_mode:
            time_since_last_switch = current_time - self.last_control_switch_time

            if new_human_controlled:
                self.get_logger().info(
                    f"HUMAN has taken control authority - Reason: {authority_reason} (previous mode duration: {time_since_last_switch:.1f}s)")
            else:
                self.get_logger().info(
                    f"ROBOT has taken control authority - Reason: {authority_reason} (previous mode duration: {time_since_last_switch:.1f}s)")

            self.last_logged_control_mode = new_human_controlled
            self.last_control_switch_time = current_time
            self.control_switch_reason = authority_reason
            self._last_activity_time = current_time  # Track activity for adaptive timers

        self.human_controlled = new_human_controlled
        
        # Log periodic status (every 10 seconds) to help debug control authority
        if not hasattr(self, '_last_status_log_time'):
            self._last_status_log_time = 0.0
        if current_time - self._last_status_log_time > 10.0:
            time_since_human = current_time - self.last_human_activity if self.last_human_activity > 0 else float('inf')
            rudder_change = getattr(self, '_last_rudder_change', 0.0)
            sail_change = getattr(self, '_last_sail_change', 0.0)
            self.get_logger().info(
                f"Control status: human_controlled={self.human_controlled}, "
                f"last_human_activity={time_since_human:.1f}s ago, "
                f"deadband={self.deadband_threshold:.3f}, "
                f"timeout={self.human_override_timeout:.1f}s, "
                f"rudder_change={rudder_change:.3f}, "
                f"sail_change={sail_change:.3f}")
            self._last_status_log_time = current_time

        # 5. Select control commands based on authority
        if self.human_controlled:
            # Use radio commands
            cmd_rudder = self.radio_rudder
            cmd_sail = self.radio_sail
        else:
            # Use autonomous commands
            cmd_rudder = self.auto_rudder
            cmd_sail = self.auto_sail

        # 6. Apply safety limits
        cmd_rudder, cmd_sail = self.apply_safety_limits(cmd_rudder, cmd_sail)

        # 7. Convert commands to pulse widths and write to hardware
        servo_rudder_pw_us = cmd_to_pw_us(cmd_rudder)
        servo_sail_pw_us = cmd_to_pw_us(cmd_sail)

        self.write_sysfs_pw(SERVO_RUDDER_PATH, servo_rudder_pw_us)
        self.write_sysfs_pw(SERVO_SAIL_PATH, servo_sail_pw_us)

        # 8. Publish actual servo commands (only on significant change or timeout)
        servo_changed = (
            self.prev_published_servo_rudder is None or
            abs(cmd_rudder - self.prev_published_servo_rudder) > self.PUBLISH_CHANGE_THRESHOLD or
            abs(cmd_sail - self.prev_published_servo_sail) > self.PUBLISH_CHANGE_THRESHOLD or
            (current_time - self.last_servo_publish_time) > self.MAX_PUBLISH_INTERVAL
        )
        
        if servo_changed:
            self._servo_msg_cache.x = cmd_rudder
            self._servo_msg_cache.y = cmd_sail
            self._servo_msg_cache.z = 0.0
            self.pub_rudder_sail_servo.publish(self._servo_msg_cache)
            self.prev_published_servo_rudder = cmd_rudder
            self.prev_published_servo_sail = cmd_sail
            self.last_servo_publish_time = current_time
            self._last_activity_time = current_time  # Track activity for adaptive timers

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals by ensuring safe exit."""
        self.get_logger().info(
            f"Received signal {signum}, initiating safe shutdown...")
        self._ensure_safe_exit()
        # Allow normal signal handling to proceed
        if signum == signal.SIGINT:
            raise KeyboardInterrupt()
        elif signum == signal.SIGTERM:
            sys.exit(0)

    def _ensure_safe_exit(self):
        """Ensure servos are in high impedance mode before exit."""
        try:
            if hasattr(self, 'get_logger'):
                self.get_logger().info("Setting servos to HIGH IMPEDANCE mode for safe exit...")
            else:
                print(
                    "Setting servos to HIGH IMPEDANCE mode for safe exit...", file=sys.stderr)

            # Set servos to high impedance mode (radio control active)
            if SERVO_RUDDER_PATH.exists():
                SERVO_RUDDER_PATH.write_text("0")
            if SERVO_SAIL_PATH.exists():
                SERVO_SAIL_PATH.write_text("0")

            if hasattr(self, 'get_logger'):
                self.get_logger().info(
                    "Servos successfully set to HIGH IMPEDANCE mode. Radio control is now active.")
            else:
                print(
                    "Servos successfully set to HIGH IMPEDANCE mode. Radio control is now active.", file=sys.stderr)

        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"Error during safe exit: {e}")
            else:
                print(f"Error during safe exit: {e}", file=sys.stderr)

    def _cleanup_on_exit(self):
        """Rudder/sail radio specific cleanup on exit"""
        self._ensure_safe_exit()

    def publish_status(self):
        """Publish control status for other nodes (only on change or timeout)."""
        current_time = time.time()
        time_since_human = current_time - self.last_human_activity
        time_since_auto = current_time - self.last_auto_update
        
        # Publish human control status (only on change or timeout)
        human_changed = (
            self.prev_published_human_controlled is None or
            self.human_controlled != self.prev_published_human_controlled or
            (current_time - self.last_status_publish_time) > self.MAX_PUBLISH_INTERVAL
        )
        
        if human_changed:
            self._human_msg_cache.data = self.human_controlled
            self.pub_human_controlled.publish(self._human_msg_cache)
            self.prev_published_human_controlled = self.human_controlled
            self.last_status_publish_time = current_time  # Update timestamp for timeout tracking

        # Publish detailed control authority info (only on change or timeout)
        authority_x = 1.0 if self.human_controlled else 0.0
        authority_changed = (
            self.prev_published_control_authority is None or
            abs(authority_x - self._prev_authority_cache.x) > 0.01 or
            abs(time_since_human - self._prev_authority_cache.y) > 0.5 or
            abs(time_since_auto - self._prev_authority_cache.z) > 0.5 or
            (current_time - self.last_status_publish_time) > self.MAX_PUBLISH_INTERVAL
        )
        
        if authority_changed:
            self._authority_msg_cache.x = authority_x
            self._authority_msg_cache.y = time_since_human
            self._authority_msg_cache.z = time_since_auto
            self.pub_control_authority.publish(self._authority_msg_cache)
            
            # Update previous values for comparison
            self._prev_authority_cache.x = authority_x
            self._prev_authority_cache.y = time_since_human
            self._prev_authority_cache.z = time_since_auto
            self.prev_published_control_authority = True  # Mark as initialized
            
            self.last_status_publish_time = current_time

        # Note: Health status is now managed only in read_radio_inputs() to prevent toggling


def main(args=None):
    """Main function using ArgoBaseNode standardized approach"""
    parser = ArgoBaseNode.create_standard_parser(
        'Rudder/Sail Control Node - Combined hardware interface and control arbitration with high impedance safety mode',
        epilog="""
This ROS2 node provides unified rudder/sail control combining hardware interface
with intelligent control arbitration between human and autonomous systems:

HARDWARE INTERFACE:
- Interfaces with argo_radio_servo_module kernel module via sysfs
- Reads radio control inputs and normalizes to -1 to +1 range
- Writes servo commands to hardware with safety validation
- Handles pulse width conversion (1000-2000µs ↔ -1 to +1)
- HIGH IMPEDANCE MODE: Writing 0 to servo control files disables PWM output

CONTROL ARBITRATION:
- Human gets priority when radio input activity is detected
- Robot gets control after human_override_timeout seconds of no activity
- Deadband threshold prevents noise from triggering human activity
- Safety limits applied to all commands

HIGH IMPEDANCE SAFETY MODE:
- Servo outputs start in high impedance mode (PWM disabled) for safety
- Radio control passes through directly to servos when PWM is disabled
- Automatic switching between PWM control and radio passthrough
- Fail-safe design ensures radio control always works

TOPICS:
  Publishes:
    /rudder_sail_radio: Vector3 - Normalized radio inputs from hardware
      x: rudder (-1=full left, +1=full right, looking down at boat)
      y: sail (-1=pulled in fully, +1=let out fully)
      z: reserved (0)
    /rudder_sail_servo: Vector3 - Final commands sent to hardware
      x: rudder (-1=full left, +1=full right, looking down at boat)
      y: sail (-1=pulled in fully, +1=let out fully)
      z: reserved (0)
    /human_controlled: Bool - Current control authority status (default QoS)
    /control_authority: Vector3 - Detailed control status (authority, time_since_human, time_since_auto)
    /rudder_sail_radio_health: Bool - Node health status (ArgoBaseNode)

  Subscribes:
    /rudder_sail_cmd: Vector3 - Autonomous commands from controller.py
      x: rudder (-1=full left, +1=full right, looking down at boat)
      y: sail (-1=pulled in fully, +1=let out fully)

SERVICES:
  /rudder_sail_radio_node/health: Trigger - Health status service endpoint

PARAMETERS:
  human_override_timeout: Seconds after last human activity before robot can take control (default: 2.0)
  deadband_threshold: Minimum change in radio input to count as human activity (default: 0.05)
  safety_max_rudder: Maximum rudder command magnitude (default: 1.0)
  safety_max_sail: Maximum sail command magnitude (default: 1.0)

HARDWARE REQUIREMENTS:
- argo_radio_servo_module kernel module loaded (v0.5+ with high impedance support)
- Sysfs interface at /sys/kernel/argo_radio_servo/
- GPIO pins: PI11, PI13 (radio input), PI12, PI14 (servo output)
- RC receiver calibrated for 1000-2000µs pulse width range

ROBUSTNESS FEATURES:
- Human priority: Any radio activity immediately grants human control
- Timeout-based handover: Smooth transition to robot control when human is inactive
- Safety limits: All commands clamped to safe ranges
- High-frequency control loop: 20Hz for responsive arbitration
- Hardware validation: Pulse width clamping and outlier filtering
- Default QoS: Compatible communication between systemd and normal processes
- HIGH IMPEDANCE MODE: Automatic fail-safe switching to radio control

TEST MODE:
- Use --test flag to run servo sweep test without ROS2
- Continuously sweeps servo outputs from 900-2100µs
- Displays real-time radio input pulse widths
- Press spacebar to pause/resume, Ctrl+C to exit
        """
    )
    
    parser.add_argument('--test', action='store_true',
                        help='Run in test mode: sweep servo outputs and display radio inputs (no ROS2)')
    parser.add_argument('--profile', action='store_true',
                        help='Enable CPU profiling and save results to profile file')

    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args(args)

    # Check for invalid arguments that aren't ROS2-related
    # ROS2 arguments typically start with --ros-args, -r, or are node-specific
    valid_ros2_prefixes = ['--ros-args', '-r',
                           '--node-name', '--namespace', '--remap', '--param']
    invalid_args = []

    for arg in unknown_args:
        # Skip ROS2 arguments
        is_ros2_arg = any(arg.startswith(prefix)
                          for prefix in valid_ros2_prefixes)
        # Also skip single character flags that might be ROS2 related
        if not is_ros2_arg and (arg.startswith('-') or arg.startswith('--')):
            invalid_args.append(arg)

    if invalid_args:
        parser.print_usage()
        print(f"error: unrecognized arguments: {' '.join(invalid_args)}")
        print("Use --help for more information.")
        sys.exit(2)

    # Check if test mode is requested
    if parsed_args.test:
        run_test_mode()
        return

    # Setup CPU profiling if requested
    profiler = None
    profile_file = None
    if parsed_args.profile:
        profiler = cProfile.Profile()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        profile_file = f"rudder_sail_radio_profile_{timestamp}.prof"
        print(f"🔍 CPU profiling enabled. Results will be saved to: {profile_file}")
        print("   Run for at least 30 seconds to get meaningful results.")
        print("   Press Ctrl+C to stop and save profile.")

    try:
        if profiler:
            # Run with profiling - wrap the entire execution
            profiler.enable()
            ArgoBaseNode.run_node(RudderSailRadioNode, args, parser)
        else:
            # Normal run
            ArgoBaseNode.run_node(RudderSailRadioNode, args, parser)
    except Exception as e:
        # Handle rudder/sail radio specific errors
        print(f"CRITICAL: Failed to initialize Rudder/Sail Radio node: {e}")
        print("CRITICAL: Check kernel module and sysfs interface.")
        # Final safety check - ensure servos are in high impedance mode
        try:
            if SERVO_RUDDER_PATH.exists():
                SERVO_RUDDER_PATH.write_text("0")
            if SERVO_SAIL_PATH.exists():
                SERVO_SAIL_PATH.write_text("0")
            print("Emergency: Servos set to HIGH IMPEDANCE mode. Radio control is active.")
        except Exception as emergency_e:
            print(f"CRITICAL: Could not set high impedance mode during emergency: {emergency_e}")
        sys.exit(1)
    finally:
        # Save profiling data if enabled
        if profiler:
            profiler.disable()
            if profile_file:
                profiler.dump_stats(profile_file)
                print(f"\n📊 Profile data saved to: {profile_file}")
                print(f"\n🔍 To analyze the profile, run:")
                print(f"   python3 -m pstats {profile_file}")
                print(f"\n   Or use this command for top 30 functions:")
                print(f"   python3 -c \"import pstats; p=pstats.Stats('{profile_file}'); p.sort_stats('cumulative').print_stats(30)\"")
                print(f"\n   Or sort by time per call:")
                print(f"   python3 -c \"import pstats; p=pstats.Stats('{profile_file}'); p.sort_stats('tottime').print_stats(30)\"")


if __name__ == '__main__':
    main()
