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

Control Arbitration:
- Intelligent human/robot control switching with human priority
- Automatic detection of human activity based on radio input changes
- Configurable timeout-based handover between control modes
- Safety limits and deadband filtering for robust operation
- High-frequency control loop (20Hz) for responsive arbitration

Key Features:
- Combined hardware interface and control logic in single node
- Persistent QoS for critical control status (immediate access for late-joining nodes)
- Throttled logging to minimize system load
- Built-in safety features with neutral position defaults
- Graceful handling of hardware disconnection

Topics Published:
- /rudder_sail_radio: Vector3 with normalized radio inputs (-1 to +1)
  * x: rudder position (-1=full left, +1=full right, looking down at boat)
  * y: sail position (-1=pulled in fully, +1=let out fully)
  * z: reserved (currently 0)
  
- /rudder_sail_servo: Vector3 with actual servo commands being sent to hardware
  * Same format as radio topic but reflects actual servo positions
  
- /human_controlled: Bool indicating current control state (persistent QoS)
  * True: Human has control (radio inputs active)
  * False: Computer has control (autonomous navigation active)
  
- /control_authority: Vector3 with detailed control status
  * x: Current authority (1=human, 0=robot)
  * y: Time since human activity
  * z: Time since auto command

Topics Subscribed:
- /rudder_sail_cmd: Vector3 with autonomous navigation commands (-1 to +1 range)
  * Commands from controller.py or other autonomous navigation nodes
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

Author: Tobi Delbruck (original pwm.py), Enhanced with control arbitration
License: MIT
Version: 3.0 - Combined hardware interface and control arbitration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
from rclpy.parameter import Parameter

import yaml
import argparse
from pathlib import Path
import time
import numpy as np
import sys

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
OUTLIER_LOG_THROTTLE_S = 10.0

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
    """
    
    def __init__(self):
        super().__init__('rudder_sail_radio_node')
        self.get_logger().info('Rudder/Sail Radio node starting...')
        
        # Check for sysfs directory
        if not SYS_BASE_PATH.is_dir():
            self.get_logger().error(f"Sysfs path {SYS_BASE_PATH} not found.")
            self.get_logger().error("Is the 'argo_radio_servo_module' kernel module loaded?")
            rclpy.shutdown()
            return

        # --- Parameters ---
        self.declare_parameter('param_file_path', 'argo.yaml')
        self.declare_parameter('human_override_timeout', 2.0)  # seconds
        self.declare_parameter('deadband_threshold', 0.05)     # ignore small radio movements
        self.declare_parameter('safety_max_rudder', 1.0)       # safety limits
        self.declare_parameter('safety_max_sail', 1.0)
        
        self.param_file = Path(self.get_parameter('param_file_path').get_parameter_value().string_value)
        self._last_param_mtime = 0
        
        # Load initial parameters
        self.check_and_reload_params(is_initial=True)
        
        # Initialize parameter values for immediate use
        self.human_override_timeout = self.get_parameter('human_override_timeout').get_parameter_value().double_value
        self.deadband_threshold = self.get_parameter('deadband_threshold').get_parameter_value().double_value
        self.safety_max_rudder = self.get_parameter('safety_max_rudder').get_parameter_value().double_value
        self.safety_max_sail = self.get_parameter('safety_max_sail').get_parameter_value().double_value
        
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
        # Standard QoS for real-time data
        self.standard_qos = 10
        
        # Persistent QoS for critical control status
        # Late-joining nodes get immediate access to current control authority
        self.persistent_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1  # Keep only the latest value
        )
        
        # --- Publishers ---
        # Radio input from hardware (normalized)
        self.pub_rudder_sail_radio = self.create_publisher(Vector3, '/rudder_sail_radio', self.standard_qos)
        
        # Final servo commands to hardware
        self.pub_rudder_sail_servo = self.create_publisher(Vector3, '/rudder_sail_servo', self.standard_qos)
        
        # Status for other nodes (use persistent QoS for critical status)
        self.pub_human_controlled = self.create_publisher(Bool, '/human_controlled', self.persistent_qos)
        self.pub_control_authority = self.create_publisher(Vector3, '/control_authority', self.standard_qos)
        
        # --- Subscribers ---
        # From controller.py (autonomous commands)
        self.create_subscription(Vector3, '/rudder_sail_cmd', self.auto_control_callback, 10)
        
        # --- Timers ---
        self.control_loop_period = 0.05  # 20 Hz for responsive control
        self.timer = self.create_timer(self.control_loop_period, self.timer_callback)
        
        self.param_reload_check_period = 3.0
        self.param_timer = self.create_timer(self.param_reload_check_period, self.check_and_reload_params)
        
        # Status publishing timer
        self.status_period = 0.1  # 10 Hz status updates
        self.status_timer = self.create_timer(self.status_period, self.publish_status)
    
    def check_and_reload_params(self, is_initial=False):
        """Checks if the param file has changed and reloads it."""
        try:
            if not self.param_file.is_file():
                if self._last_param_mtime != 0:
                    self.get_logger().warn(f"Parameter file '{self.param_file}' not found.")
                self._last_param_mtime = 0
                return

            mtime = self.param_file.stat().st_mtime
            if mtime > self._last_param_mtime:
                self.get_logger().info(f"Parameter file '{self.param_file}' changed, reloading...")
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
                    self.get_logger().warn(f"Could not find parameters for node '{node_name}' in '{self.param_file}'.")

                # Update internal variables from parameters
                self.human_override_timeout = self.get_parameter('human_override_timeout').get_parameter_value().double_value
                self.deadband_threshold = self.get_parameter('deadband_threshold').get_parameter_value().double_value
                self.safety_max_rudder = self.get_parameter('safety_max_rudder').get_parameter_value().double_value
                self.safety_max_sail = self.get_parameter('safety_max_sail').get_parameter_value().double_value

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

    def read_radio_inputs(self):
        """Read and process radio inputs from hardware."""
        # Read radio inputs from sysfs
        radio_rudder_pw_us = self.read_sysfs_pw(RADIO_RUDDER_PATH)
        radio_sail_pw_us = self.read_sysfs_pw(RADIO_SAIL_PATH)

        # Validate and normalize radio inputs
        if not (500 < radio_rudder_pw_us < 2500 and 500 < radio_sail_pw_us < 2500):
            # Throttle outlier warning logs to once every 10 seconds
            now = time.time()
            if now - self.last_outlier_warning_time > OUTLIER_LOG_THROTTLE_S:
                self.get_logger().warn(f"Outlier radio PWM: rudder={radio_rudder_pw_us:.1f}us, sail={radio_sail_pw_us:.1f}us")
                self.last_outlier_warning_time = now
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
                self.get_logger().info(f"Human activity detected (rudder: {rudder_change:.3f}, sail: {sail_change:.3f})")
        
        # Update previous values for next comparison
        self.prev_radio_rudder = self.radio_rudder
        self.prev_radio_sail = self.radio_sail
        
        return True
    
    def auto_control_callback(self, msg):
        """Receive autonomous control commands from controller.py."""
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
        """
        current_time = time.time()
        
        # Check for recent human activity
        time_since_human_activity = current_time - self.last_human_activity
        
        # Human has control if there's been recent activity
        if time_since_human_activity < self.human_override_timeout:
            return True  # Human control
        
        # Check if we have recent autonomous commands
        time_since_auto_update = current_time - self.last_auto_update
        if time_since_auto_update < 1.0:  # Auto commands are fresh
            return False  # Robot control
        
        # Default to human control for safety if no recent commands
        return True
    
    def apply_safety_limits(self, rudder, sail):
        """Apply safety limits to control commands."""
        rudder = np.clip(rudder, -self.safety_max_rudder, self.safety_max_rudder)
        sail = np.clip(sail, -self.safety_max_sail, self.safety_max_sail)
        return rudder, sail
    
    def timer_callback(self):
        """Main control arbitration and hardware interface loop."""
        # 1. Read radio inputs from hardware
        if not self.read_radio_inputs():
            return  # Skip this cycle if radio inputs are invalid
        
        # 2. Publish normalized radio inputs
        radio_msg = Vector3(x=self.radio_rudder, y=self.radio_sail, z=0.0)
        self.pub_rudder_sail_radio.publish(radio_msg)
        
        # 3. Determine control authority
        self.human_controlled = self.determine_control_authority()
        
        # 4. Log control mode changes
        if self.human_controlled != self.last_logged_control_mode:
            if self.human_controlled:
                self.get_logger().info("HUMAN has control authority")
            else:
                self.get_logger().info("ROBOT has control authority")
            self.last_logged_control_mode = self.human_controlled
        
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
            x=1.0 if self.human_controlled else 0.0,  # Current authority (1=human, 0=robot)
            y=time_since_human,                       # Time since human activity
            z=time_since_auto                         # Time since auto command
        )
        self.pub_control_authority.publish(authority_msg)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Rudder/Sail Control Node - Combined hardware interface and control arbitration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node provides unified rudder/sail control combining hardware interface
with intelligent control arbitration between human and autonomous systems:

HARDWARE INTERFACE:
- Interfaces with argo_radio_servo_module kernel module via sysfs
- Reads radio control inputs and normalizes to -1 to +1 range
- Writes servo commands to hardware with safety validation
- Handles pulse width conversion (1000-2000µs ↔ -1 to +1)

CONTROL ARBITRATION:
- Human gets priority when radio input activity is detected
- Robot gets control after human_override_timeout seconds of no activity
- Deadband threshold prevents noise from triggering human activity
- Safety limits applied to all commands

TOPICS:
  Publishes:
    /rudder_sail_radio: Vector3 - Normalized radio inputs from hardware
    /rudder_sail_servo: Vector3 - Final commands sent to hardware
    /human_controlled: Bool - Current control authority status (persistent QoS)
    /control_authority: Vector3 - Detailed control status (authority, time_since_human, time_since_auto)

  Subscribes:
    /rudder_sail_cmd: Vector3 - Autonomous commands from controller.py

PARAMETERS:
  human_override_timeout: Seconds after last human activity before robot can take control (default: 2.0)
  deadband_threshold: Minimum change in radio input to count as human activity (default: 0.05)
  safety_max_rudder: Maximum rudder command magnitude (default: 1.0)
  safety_max_sail: Maximum sail command magnitude (default: 1.0)

HARDWARE REQUIREMENTS:
- argo_radio_servo_module kernel module loaded
- Sysfs interface at /sys/kernel/argo_radio_servo/
- GPIO pins: PI11, PI13 (radio input), PI12, PI14 (servo output)
- RC receiver calibrated for 1000-2000µs pulse width range

ROBUSTNESS FEATURES:
- Human priority: Any radio activity immediately grants human control
- Timeout-based handover: Smooth transition to robot control when human is inactive
- Safety limits: All commands clamped to safe ranges
- High-frequency control loop: 20Hz for responsive arbitration
- Hardware validation: Pulse width clamping and outlier filtering
- Persistent QoS: Late-joining nodes get immediate access to control status
        """
    )
    
    parsed_args, unknown_args = parser.parse_known_args(args)
    
    rclpy.init(args=unknown_args)
    node = None
    try:
        node = RudderSailRadioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()