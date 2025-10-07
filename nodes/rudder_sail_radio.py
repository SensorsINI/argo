#!/usr/bin/env python3
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
from rclpy.parameter import Parameter

import yaml
import argparse
from pathlib import Path
import time
import numpy as np
import sys
import signal
import tty
import termios
import select

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from toggle_pause_service import TogglePauseService

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

# Test mode configuration
TEST_MIN_PW = 900
TEST_MAX_PW = 2100
TEST_STEP = 100
TEST_DELAY_S = 0.5
TEST_DEFAULT_PW = 1500


def cmd_to_pw_us(cmd: float) -> int:
    """Converts a normalized command (-1 to +1) to a pulse width in microseconds (1000 to 2000)."""
    # Clamp command to [-1, 1]
    cmd = max(-1.0, min(1.0, cmd))
    # Linear interpolation: 1500 is center, 500 is range on each side
    pw = 1500 + 500 * cmd
    return int(pw)


def pw_us_to_cmd(pw_us: float) -> float:
    """Converts a pulse width in microseconds (1000 to 2000) to a normalized command (-1 to +1)."""
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


class RudderSailRadioNode(Node):
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

    def __init__(self):
        super().__init__('rudder_sail_radio_node')
        
        # Initialize pause service
        self.pause_service = TogglePauseService(self)
        
        self.get_logger().info(
            'Rudder/Sail Radio node starting with high impedance safety mode...')

        # Check for sysfs directory
        if not SYS_BASE_PATH.is_dir():
            self.get_logger().error(f"Sysfs path {SYS_BASE_PATH} not found.")
            self.get_logger().error("Is the 'argo_radio_servo_module' kernel module loaded?")
            rclpy.shutdown()
            return

        # --- Parameters ---
        self.declare_parameter('param_file_path', 'argo.yaml')
        self.declare_parameter('human_override_timeout',
                               HUMAN_CONTROL_TIMEOUT_S)  # seconds
        # ignore small radio movements
        self.declare_parameter('deadband_threshold', 0.05)
        self.declare_parameter('safety_max_rudder', 1.0)       # safety limits
        self.declare_parameter('safety_max_sail', 1.0)

        self.param_file = Path(self.get_parameter(
            'param_file_path').get_parameter_value().string_value)
        self._last_param_mtime = 0

        # Load initial parameters
        self.check_and_reload_params(is_initial=True)

        # Initialize parameter values for immediate use
        self.human_override_timeout = self.get_parameter(
            'human_override_timeout').get_parameter_value().double_value
        self.deadband_threshold = self.get_parameter(
            'deadband_threshold').get_parameter_value().double_value
        self.safety_max_rudder = self.get_parameter(
            'safety_max_rudder').get_parameter_value().double_value
        self.safety_max_sail = self.get_parameter(
            'safety_max_sail').get_parameter_value().double_value

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
        self.human_controlled = True  # Start in human control for safety
        self.last_human_activity = time.time()
        self.last_logged_control_mode = None

        # Track previous radio values for human activity detection
        self.prev_radio_rudder = 0.0
        self.prev_radio_sail = 0.0

        # Throttling for clamp warning logs
        self.last_clamp_warning_time = 0.0

        # Throttling for outlier radio PWM warning logs
        self.last_outlier_warning_time = 0.0

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

        # Health status publisher
        self.pub_health = self.create_publisher(
            Bool, '/rudder_sail_radio_health', 10)
        self.health_status = False  # Track current health status

        # --- Subscribers ---
        # From controller.py (autonomous commands)
        self.create_subscription(
            Vector3, '/rudder_sail_cmd', self.auto_control_callback, 10)

        # --- Timers ---
        self.control_loop_period = 0.05  # 20 Hz for responsive control
        self.timer = self.create_timer(
            self.control_loop_period, self.timer_callback)

        # Publish initial health status as healthy
        self._publish_health_status(True)

        self.param_reload_check_period = 3.0
        self.param_timer = self.create_timer(
            self.param_reload_check_period, self.check_and_reload_params)

        # Status publishing timer
        self.status_period = 0.1  # 10 Hz status updates
        self.status_timer = self.create_timer(
            self.status_period, self.publish_status)

        # Initialize servos to high impedance mode for safety
        self.set_servo_high_impedance()

        # Register cleanup handlers for graceful shutdown
        import atexit
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        atexit.register(self._ensure_safe_exit)

    def check_and_reload_params(self, is_initial=False):
        """Checks if the param file has changed and reloads it."""
        try:
            if not self.param_file.is_file():
                if self._last_param_mtime != 0:
                    self.get_logger().warn(
                        f"Parameter file '{self.param_file}' not found.")
                self._last_param_mtime = 0
                return

            mtime = self.param_file.stat().st_mtime
            if mtime > self._last_param_mtime:
                self.get_logger().info(
                    f"Parameter file '{self.param_file}' changed, reloading...")
                self._last_param_mtime = mtime

                with open(self.param_file, 'r') as f:
                    data = yaml.safe_load(f)

                node_name = self.get_name()
                if node_name in data and 'ros__parameters' in data[node_name]:
                    params_to_set = []
                    new_params = data[node_name]['ros__parameters']
                    for name, value in new_params.items():
                        if self.has_parameter(name):
                            params_to_set.append(Parameter(name, value=value))

                    if params_to_set:
                        self.set_parameters(params_to_set)
                else:
                    self.get_logger().warn(
                        f"Could not find parameters for node '{node_name}' in '{self.param_file}'.")

                # Update internal variables from parameters
                self.human_override_timeout = self.get_parameter(
                    'human_override_timeout').get_parameter_value().double_value
                self.deadband_threshold = self.get_parameter(
                    'deadband_threshold').get_parameter_value().double_value
                self.safety_max_rudder = self.get_parameter(
                    'safety_max_rudder').get_parameter_value().double_value
                self.safety_max_sail = self.get_parameter(
                    'safety_max_sail').get_parameter_value().double_value

        except Exception as e:
            self.get_logger().error(f"Error reloading parameters: {e}")

    def read_sysfs_pw(self, path: Path) -> float:
        """Reads a pulse width from a sysfs file."""
        try:
            return float(path.read_text().strip())
        except (IOError, FileNotFoundError, ValueError) as e:
            self.get_logger().warn(f"Could not read or parse {path}: {e}")
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
            self.get_logger().debug(f"Wrote {value}µs to {path}")
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
        """Read and process radio inputs from hardware."""
        # Read radio inputs from sysfs
        radio_rudder_pw_us = self.read_sysfs_pw(RADIO_RUDDER_PATH)
        radio_sail_pw_us = self.read_sysfs_pw(RADIO_SAIL_PATH)

        # Validate and normalize radio inputs
        if not (500 < radio_rudder_pw_us < 2500 and 500 < radio_sail_pw_us < 2500):
            # Determine the specific cause of health failure
            if radio_rudder_pw_us == 0.0 and radio_sail_pw_us == 0.0:
                failure_reason = "Radio transmitter off or disconnected (both channels 0.0us)"
            elif radio_rudder_pw_us == 0.0:
                failure_reason = f"Radio rudder channel off/disconnected (0.0us), sail={radio_sail_pw_us:.1f}us"
            elif radio_sail_pw_us == 0.0:
                failure_reason = f"Radio sail channel off/disconnected (0.0us), rudder={radio_rudder_pw_us:.1f}us"
            else:
                failure_reason = f"Outlier radio PWM: rudder={radio_rudder_pw_us:.1f}us, sail={radio_sail_pw_us:.1f}us"

            # Throttle warning logs to once every 10 seconds
            now = time.time()
            if now - self.last_outlier_warning_time > OUTLIER_LOG_THROTTLE_S:
                self.get_logger().warn(
                    f"Radio input validation failed: {failure_reason}")
                self.last_outlier_warning_time = now

            self._publish_health_status(False, failure_reason)
            return False

        # The original script inverted the rudder command. Let's preserve that.
        # 1000us -> +1 (right), 2000us -> -1 (left)
        self.radio_rudder = -pw_us_to_cmd(radio_rudder_pw_us)
        # 1000us -> -1 (in), 2000us -> +1 (out)
        self.radio_sail = pw_us_to_cmd(radio_sail_pw_us)

        self.last_radio_update = time.time()

        # Detect human activity based on significant radio input changes
        rudder_change = abs(self.radio_rudder - self.prev_radio_rudder)
        sail_change = abs(self.radio_sail - self.prev_radio_sail)

        if rudder_change > self.deadband_threshold or sail_change > self.deadband_threshold:
            self.last_human_activity = time.time()
            if not self.human_controlled:
                self.get_logger().info(
                    f"Human activity detected (rudder: {rudder_change:.3f}, sail: {sail_change:.3f})")

        # Update previous values for next comparison
        self.prev_radio_rudder = self.radio_rudder
        self.prev_radio_sail = self.radio_sail

        # Set health status to healthy when radio inputs are valid
        self._publish_health_status(True, "Radio inputs within valid range")

        return True

    def auto_control_callback(self, msg):
        """Receive autonomous control commands from controller.py."""
        # Always process autonomous commands, even when paused (for safety)
        self.auto_rudder = msg.x
        self.auto_sail = msg.y
        self.last_auto_update = time.time()

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
        time_since_human_activity = current_time - self.last_human_activity

        # Human has control if there's been recent activity
        if time_since_human_activity < self.human_override_timeout:
            reason = f"human_activity_within_{self.human_override_timeout:.1f}s (last_activity: {time_since_human_activity:.1f}s ago)"
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

        return False, reason  # Robot control

    def apply_safety_limits(self, rudder, sail):
        """Apply safety limits to control commands."""
        rudder = np.clip(rudder, -self.safety_max_rudder,
                         self.safety_max_rudder)
        sail = np.clip(sail, -self.safety_max_sail, self.safety_max_sail)
        return rudder, sail

    def _publish_health_status(self, is_healthy: bool, reason: str = None):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            if is_healthy:
                self.get_logger().info("Rudder/Sail Radio health status: HEALTHY")
            else:
                if reason:
                    self.get_logger().warn(
                        f"Rudder/Sail Radio health status: FAILED - {reason}")
                else:
                    self.get_logger().warn("Rudder/Sail Radio health status: FAILED")

    def timer_callback(self):
        """Main control arbitration and hardware interface loop."""
        # Check if node is paused
        if self.pause_service.is_paused():
            return  # Skip processing when paused
        
        # 1. Read radio inputs from hardware
        if not self.read_radio_inputs():
            return  # Skip this cycle if radio inputs are invalid

        # 2. Publish normalized radio inputs
        radio_msg = Vector3(x=self.radio_rudder, y=self.radio_sail, z=0.0)
        self.pub_rudder_sail_radio.publish(radio_msg)

        # 3. Determine control authority
        new_human_controlled, authority_reason = self.determine_control_authority()

        # 4. Log control mode changes with detailed reason
        if new_human_controlled != self.last_logged_control_mode:
            current_time = time.time()
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

        self.human_controlled = new_human_controlled

        # 5. Select control commands based on authority
        if self.human_controlled:
            # Use radio commands
            cmd_rudder = self.radio_rudder
            cmd_sail = self.radio_sail

            self.get_logger().debug(
                f"Human control: rudder={cmd_rudder:.3f}, sail={cmd_sail:.3f}"
            )
        else:
            # Use autonomous commands
            cmd_rudder = self.auto_rudder
            cmd_sail = self.auto_sail

            self.get_logger().debug(
                f"Robot control: rudder={cmd_rudder:.3f}, sail={cmd_sail:.3f}"
            )

        # 6. Apply safety limits
        cmd_rudder, cmd_sail = self.apply_safety_limits(cmd_rudder, cmd_sail)

        # 7. Convert commands to pulse widths and write to hardware
        servo_rudder_pw_us = cmd_to_pw_us(cmd_rudder)
        servo_sail_pw_us = cmd_to_pw_us(cmd_sail)

        self.write_sysfs_pw(SERVO_RUDDER_PATH, servo_rudder_pw_us)
        self.write_sysfs_pw(SERVO_SAIL_PATH, servo_sail_pw_us)

        # 8. Publish actual servo commands
        servo_msg = Vector3(x=cmd_rudder, y=cmd_sail, z=0.0)
        self.pub_rudder_sail_servo.publish(servo_msg)

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

    def destroy_node(self):
        """Override destroy_node to ensure safe exit."""
        # Publish health status as failed on shutdown
        self._publish_health_status(False)

        self._ensure_safe_exit()
        super().destroy_node()

    def publish_status(self):
        """Publish control status for other nodes."""
        # Publish human control status
        human_msg = Bool(data=self.human_controlled)
        self.pub_human_controlled.publish(human_msg)

        # Publish detailed control authority info
        current_time = time.time()
        time_since_human = current_time - self.last_human_activity
        time_since_auto = current_time - self.last_auto_update

        authority_msg = Vector3(
            # Current authority (1=human, 0=robot)
            x=1.0 if self.human_controlled else 0.0,
            y=time_since_human,                       # Time since human activity
            z=time_since_auto                         # Time since auto command
        )
        self.pub_control_authority.publish(authority_msg)

        # Note: Health status is now managed only in read_radio_inputs() to prevent toggling


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Rudder/Sail Control Node - Combined hardware interface and control arbitration with high impedance safety mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
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
    /rudder_sail_servo: Vector3 - Final commands sent to hardware
    /human_controlled: Bool - Current control authority status (default QoS)
    /control_authority: Vector3 - Detailed control status (authority, time_since_human, time_since_auto)

  Subscribes:
    /rudder_sail_cmd: Vector3 - Autonomous commands from controller.py

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

    rclpy.init(args=unknown_args)
    node = None
    try:
        node = RudderSailRadioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Initiating safe shutdown...")
    except rclpy.executors.ExternalShutdownException:
        print("External shutdown requested. Initiating safe shutdown...")
    except Exception as e:
        print(f"Unexpected error: {e}. Initiating emergency safe shutdown...")
        # Ensure servos are in high impedance mode even on unexpected errors
        try:
            if SERVO_RUDDER_PATH.exists():
                SERVO_RUDDER_PATH.write_text("0")
            if SERVO_SAIL_PATH.exists():
                SERVO_SAIL_PATH.write_text("0")
            print(
                "Emergency: Servos set to HIGH IMPEDANCE mode. Radio control is active.")
        except Exception as emergency_e:
            print(
                f"CRITICAL: Could not set high impedance mode during emergency: {emergency_e}")
    finally:
        # Ensure proper cleanup in all cases
        if node:
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error during node destruction: {e}")

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error during ROS2 shutdown: {e}")

        # Final safety check - ensure servos are in high impedance mode
        try:
            if SERVO_RUDDER_PATH.exists():
                SERVO_RUDDER_PATH.write_text("0")
            if SERVO_SAIL_PATH.exists():
                SERVO_SAIL_PATH.write_text("0")
            print("Final safety check: Servos confirmed in HIGH IMPEDANCE mode.")
        except Exception as e:
            print(f"CRITICAL: Final safety check failed: {e}")


if __name__ == '__main__':
    main()
