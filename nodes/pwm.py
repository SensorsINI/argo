#!/usr/bin/env python3
"""
ROS2 PWM Node - Autonomous Boat Radio Control and Servo Interface

This node provides the interface between radio control hardware and the autonomous
navigation system. It captures PWM signals from RC receivers, manages human/computer
control switching, and controls servo outputs for rudder and sail actuators.

Hardware Interface:
- Uses argo_radio_servo_module kernel module for hardware PWM capture/generation
- Interfaces via sysfs at /sys/kernel/argo_radio_servo/
- Supports Orange Pi Zero 2W with custom GPIO/PWM configuration

Key Features:
- Real-time radio control input capture and normalization
- Automatic human/computer control switching based on stick activity
- Safe servo output with pulse width validation (900-2100µs range)
- Throttled logging to minimize system load
- Built-in safety features with neutral position defaults

Control Logic:
- Human takes control when rudder stick moves beyond threshold
- Computer regains control after timeout period of stick inactivity
- All servo outputs are validated and clamped to safe hardware ranges
- Invalid radio inputs are filtered to prevent system instability

Topics Published:
- /rudder_sail_radio: Vector3 with normalized radio inputs (-1 to +1)
  * x: rudder position (-1=full left, +1=full right, looking down at boat)
  * y: sail position (-1=pulled in fully, +1=let out fully)
  * z: reserved (currently 0)
  
- /rudder_sail_servo: Vector3 with actual servo commands being sent to hardware
  * Same format as radio topic but reflects actual servo positions
  
- /human_controlled: Bool indicating current control state
  * True: Human has control (radio inputs active)
  * False: Computer has control (autonomous navigation active)

Topics Subscribed:
- /rudder_sail_cmd: Vector3 with autonomous navigation commands (-1 to +1 range)
  * Commands from control.py or other autonomous navigation nodes
  * Only applied when human_controlled is False

Hardware Requirements:
- argo_radio_servo_module kernel module loaded
- GPIO pins configured for radio input capture (PI11, PI13)
- PWM outputs configured for servo control (PI12=PWM2, PI14=PWM4)
- RC receiver connected and calibrated for 1000-2000µs pulse width range

Safety Features:
- Pulse width validation and clamping (900-2100µs hardware range)
- Outlier radio input filtering (500-2500µs acceptance range)
- Automatic fallback to neutral positions on invalid inputs
- Throttled error logging to prevent log spam
- Graceful handling of hardware disconnection

Author: Tobi Delbruck
License: MIT
Version: 2.0 - Added pulse width clamping and improved error handling
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

import time
import argparse
from pathlib import Path
import sys
import tty
import termios
import select

# --- Configuration ---
SYS_BASE_PATH = Path("/sys/kernel/argo_radio_servo")
RADIO_RUDDER_PATH = SYS_BASE_PATH / "radio_rudder_pw_us"
RADIO_SAIL_PATH = SYS_BASE_PATH / "radio_sail_pw_us"
SERVO_RUDDER_PATH = SYS_BASE_PATH / "servo_rudder_pw_us"
SERVO_SAIL_PATH = SYS_BASE_PATH / "servo_sail_pw_us"

# Kernel module enforces 900-2100µs range for servo outputs
SERVO_MIN_PW_US = 900
SERVO_MAX_PW_US = 2100

HUMAN_CONTROL_TIMEOUT_S = 2.0
# time in seconds that human takes control after rudder command deviates by
# more than HUMAN_CONTROL_THRESHOLD
HUMAN_CONTROL_THRESHOLD = 0.2
# threshold deviation from zero for radio rudder input that human takes control

# Throttle logging for clamped values to once per minute
CLAMP_LOG_THROTTLE_S = 60.0

# Throttle logging for outlier radio PWM messages to once every 10 seconds
OUTLIER_LOG_THROTTLE_S = 10.0

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

class PwmNode(Node):
    def __init__(self):
        super().__init__('pwm_node')
        self.get_logger().info("pwm_node starting...")

        # Check for sysfs directory
        if not SYS_BASE_PATH.is_dir():
            self.get_logger().error(f"Sysfs path {SYS_BASE_PATH} not found.")
            self.get_logger().error("Is the 'argo_radio_servo_module' kernel module loaded?")
            rclpy.shutdown()
            return

        # QoS Profiles
        # Standard QoS for real-time data
        self.standard_qos = 10
        
        # Persistent QoS for critical control status
        # Late-joining nodes get immediate access to current control authority
        self.persistent_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1  # Keep only the latest value
        )

        # Publishers
        self.pub_rudder_sail_radio = self.create_publisher(Vector3, 'rudder_sail_radio', self.standard_qos)
        self.pub_rudder_sail_servo = self.create_publisher(Vector3, 'rudder_sail_servo', self.standard_qos)
        self.pub_human_controlled = self.create_publisher(Bool, 'human_controlled', self.persistent_qos)

        # Subscriber
        self.sub_rudder_sail_cmd = self.create_subscription(
            Vector3,
            '/rudder_sail_cmd',
            self.rudder_sail_cmd_callback,
            10)

        # State variables
        self.cmd_rudder = None
        self.cmd_sail = None
        self.time_last_human_cmd = time.time()
        self.human_control = True
        
        # Throttling for clamp warning logs
        self.last_clamp_warning_time = 0.0
        
        # Throttling for outlier radio PWM warning logs
        self.last_outlier_warning_time = 0.0

        self.get_logger().info(f"Human control timeout={HUMAN_CONTROL_TIMEOUT_S:.1f}s, threshold={HUMAN_CONTROL_THRESHOLD:.1f}")

        # Main loop timer
        self.timer = self.create_timer(0.1, self.main_loop) # 10 Hz

    def rudder_sail_cmd_callback(self, msg):
        self.get_logger().debug(f"Received /rudder_sail_cmd: {msg}")
        self.cmd_rudder = msg.x
        self.cmd_sail = msg.y

    def read_sysfs_pw(self, path: Path) -> float:
        """Reads a pulse width from a sysfs file."""
        try:
            return float(path.read_text().strip())
        except (IOError, FileNotFoundError, ValueError) as e:
            self.get_logger().warn(f"Could not read or parse {path}: {e}")
            return 0.0 # Return a safe, invalid value

    def write_sysfs_pw(self, path: Path, value: int):
        """Writes a pulse width to a sysfs file, clamping to valid range."""
        original_value = value
        
        # Clamp to kernel module's valid range (900-2100µs)
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

    def main_loop(self):
        # 1. Read radio inputs from sysfs
        radio_rudder_pw_us = self.read_sysfs_pw(RADIO_RUDDER_PATH)
        radio_sail_pw_us = self.read_sysfs_pw(RADIO_SAIL_PATH)

        # 2. Validate and normalize radio inputs
        if not (500 < radio_rudder_pw_us < 2500 and 500 < radio_sail_pw_us < 2500):
            # Throttle outlier warning logs to once every 10 seconds
            now = time.time()
            if now - self.last_outlier_warning_time > OUTLIER_LOG_THROTTLE_S:
                self.get_logger().warn(f"Outlier radio PWM: rudder={radio_rudder_pw_us:.1f}us, sail={radio_sail_pw_us:.1f}us")
                self.last_outlier_warning_time = now
            return

        # The original script inverted the rudder command. Let's preserve that.
        # 1000us -> +1 (right), 2000us -> -1 (left)
        radio_rudder_normalized = -pw_us_to_cmd(radio_rudder_pw_us)
        # 1000us -> -1 (in), 2000us -> +1 (out)
        radio_sail_normalized = pw_us_to_cmd(radio_sail_pw_us)

        self.pub_rudder_sail_radio.publish(
            Vector3(x=radio_rudder_normalized, y=radio_sail_normalized, z=0.0)
        )

        # 3. Human control logic
        # If the human moves the rudder stick, reset the control timer.
        if abs(radio_rudder_normalized) > HUMAN_CONTROL_THRESHOLD:
            self.time_last_human_cmd = time.time()

        # Determine the current control state based on the timeout.
        human_control_now = (time.time() - self.time_last_human_cmd) < HUMAN_CONTROL_TIMEOUT_S

        # Only log and publish if the control state has changed.
        if human_control_now != self.human_control:
            self.get_logger().info("Human took control" if human_control_now else "Computer took control")
            self.pub_human_controlled.publish(Bool(data=human_control_now))

        self.human_control = human_control_now

        # 4. Set servo outputs
        servo_rudder_pw_us = 0
        servo_sail_pw_us = 0

        if self.human_control or self.cmd_rudder is None:
            # Pass through radio commands
            servo_rudder_pw_us = int(radio_rudder_pw_us)
            servo_sail_pw_us = int(radio_sail_pw_us)
        else:
            # Use computer commands
            
            servo_rudder_pw_us = cmd_to_pw_us(self.cmd_rudder)
            servo_sail_pw_us = cmd_to_pw_us(self.cmd_sail)

        self.write_sysfs_pw(SERVO_RUDDER_PATH, servo_rudder_pw_us)
        self.write_sysfs_pw(SERVO_SAIL_PATH, servo_sail_pw_us)

        # 5. Publish actual servo commands and human control state
        servo_rudder_normalized = pw_us_to_cmd(float(servo_rudder_pw_us))
        servo_sail_normalized = pw_us_to_cmd(float(servo_sail_pw_us))
        self.pub_rudder_sail_servo.publish(
            Vector3(x=servo_rudder_normalized, y=servo_sail_normalized, z=0.0)
        )

# Test mode helper functions
def get_initial_pw_test(path: Path) -> int:
    """Reads the initial pulse width from a sysfs file, or returns a default."""
    try:
        return int(path.read_text().strip())
    except (IOError, ValueError, FileNotFoundError):
        print(f"Warning: Could not read {path}, using default {TEST_DEFAULT_PW} us.", file=sys.stderr)
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
                display_test_status(current_rudder_pw, current_sail_pw, paused)  # Update status immediately

                if paused:
                    print("\nPAUSED. Press Spacebar to RESUME...")
                    sys.stdout.flush()
                    # When paused, enter a blocking loop that waits *only* for a spacebar.
                    while True:
                        key_to_resume = get_key_blocking()
                        if key_to_resume == ' ':
                            paused = not paused
                            display_test_status(current_rudder_pw, current_sail_pw, paused)
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

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='PWM Node for ROS2 - Captures and publishes rudder and sail servo positions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node interfaces with the argo_radio_servo_module kernel module to:
- Read radio control inputs for rudder and sail from sysfs
- Publish normalized servo commands (-1 to +1) to ROS topics
- Handle human vs computer control switching
- Write servo commands back to the kernel module

Topics:
  Publishes:
    /rudder_sail_radio: Vector3 with normalized radio inputs (x=rudder, y=sail)
    /rudder_sail_servo: Vector3 with actual servo commands sent to hardware
    /human_controlled: Bool indicating if human has control (based on rudder input)

  Subscribes:
    /rudder_sail_cmd: Vector3 with computer-generated servo commands

Hardware:
  Requires argo_radio_servo_module kernel module loaded
  Sysfs interface at /sys/kernel/argo_radio_servo/
  Pulse width range: 1000-2000 microseconds (1500 = center)
        """
    )
    
    parser.add_argument('--test', action='store_true', 
                        help='Run in test mode: sweep servo outputs and display radio inputs (no ROS2)')
    
    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args(args)
    
    # Check for invalid arguments that aren't ROS2-related
    # ROS2 arguments typically start with --ros-args, -r, or are node-specific
    valid_ros2_prefixes = ['--ros-args', '-r', '--node-name', '--namespace', '--remap', '--param']
    invalid_args = []
    
    for arg in unknown_args:
        # Skip ROS2 arguments
        is_ros2_arg = any(arg.startswith(prefix) for prefix in valid_ros2_prefixes)
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
    
    # Initialize ROS2 with remaining arguments
    rclpy.init(args=unknown_args)
    pwm_node = PwmNode()
    
    if rclpy.ok():
        try:
            rclpy.spin(pwm_node)
        except KeyboardInterrupt:
            # The rclpy signal handler initiates shutdown on Ctrl+C.
            # We can't use the logger here as the context may be invalid.
            print("\nKeyboard interrupt, shutting down.")
        except rclpy.executors.ExternalShutdownException:
            # External shutdown (e.g., systemd service stop) - this is normal
            print("External shutdown signal received, exiting gracefully.")
        finally:
            try:
                pwm_node.destroy_node()
            except Exception:
                # Node may already be destroyed
                pass
            # rclpy.shutdown() is not called here to avoid the "context already shutdown" error.

if __name__ == '__main__':
    main()
