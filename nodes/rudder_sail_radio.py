#!/usr/bin/env python3
# ROS2 node for low-level rudder/sail control and human/robot arbitration
# Handles hardware control and prioritizes human input when detected

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

class RudderSailRadioNode(Node):
    """
    Low-level node for rudder/sail control and human/robot arbitration.
    
    Key responsibilities:
    1. Receive radio input from PWM hardware
    2. Receive autonomous control commands from controller.py
    3. Arbitrate between human and robot control with human priority
    4. Publish final rudder/sail commands to PWM hardware
    5. Publish control authority status
    """
    
    def __init__(self):
        super().__init__('rudder_sail_radio_node')
        self.get_logger().info('Rudder/Sail Radio node starting...')

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
        
        # --- State Variables ---
        # Radio input (from PWM hardware)
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
        # To PWM hardware (real-time commands)
        self.pub_rudder_sail_servo = self.create_publisher(Vector3, '/rudder_sail_servo', self.standard_qos)
        
        # Status for other nodes (use persistent QoS for critical status)
        self.pub_human_controlled = self.create_publisher(Bool, '/human_controlled', self.persistent_qos)
        self.pub_control_authority = self.create_publisher(Vector3, '/control_authority', self.standard_qos)
        
        # --- Subscribers ---
        # From PWM hardware (radio input)
        self.create_subscription(Vector3, '/rudder_sail_radio', self.radio_callback, 10)
        
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
    
    def radio_callback(self, msg):
        """Receive radio input from PWM hardware."""
        self.radio_rudder = msg.x
        self.radio_sail = msg.y
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
        """Main control arbitration loop."""
        # Determine control authority
        self.human_controlled = self.determine_control_authority()
        
        # Log control mode changes
        if self.human_controlled != self.last_logged_control_mode:
            if self.human_controlled:
                self.get_logger().info("HUMAN has control authority")
            else:
                self.get_logger().info("ROBOT has control authority")
            self.last_logged_control_mode = self.human_controlled
        
        # Select control commands based on authority
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
        
        # Apply safety limits
        cmd_rudder, cmd_sail = self.apply_safety_limits(cmd_rudder, cmd_sail)
        
        # Publish commands to PWM hardware
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
        description='Rudder/Sail Radio Node - Low-level control arbitration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node handles low-level rudder/sail control and human/robot arbitration:

CONTROL ARBITRATION:
- Human gets priority when radio input activity is detected
- Robot gets control after human_override_timeout seconds of no activity
- Deadband threshold prevents noise from triggering human activity
- Safety limits applied to all commands

TOPICS:
  Publishes:
    /rudder_sail_servo: Vector3 - Final commands to PWM hardware
    /human_controlled: Bool - Current control authority status
    /control_authority: Vector3 - Detailed control status (authority, time_since_human, time_since_auto)

  Subscribes:
    /rudder_sail_radio: Vector3 - Radio input from PWM hardware
    /rudder_sail_cmd: Vector3 - Autonomous commands from controller.py

PARAMETERS:
  human_override_timeout: Seconds after last human activity before robot can take control (default: 2.0)
  deadband_threshold: Minimum change in radio input to count as human activity (default: 0.05)
  safety_max_rudder: Maximum rudder command magnitude (default: 1.0)
  safety_max_sail: Maximum sail command magnitude (default: 1.0)

ROBUSTNESS FEATURES:
- Human priority: Any radio activity immediately grants human control
- Timeout-based handover: Smooth transition to robot control when human is inactive
- Safety limits: All commands clamped to safe ranges
- High-frequency control loop: 20Hz for responsive arbitration
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
