#!/usr/bin/env python3
# ROS2 version of control.py
# Controls Argo based on sensor inputs.

# topics
# publishes 
# /rudder_sail_cmd,
#   Vector3 with .x=rudder command -1:1=left:right, .y=sail command -1:in, +1:out
# 
# subscribes to 
# /rudder_sail_radio: Radio commands from pwm.py
# /human_controlled: Boolean indicating if human has control
# /pose: IMU data, specifically compass heading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from rclpy.parameter import Parameter

import yaml
from pathlib import Path
import math
import time
import numpy as np

def signed_angle_difference_degrees(angle1_deg, angle2_deg):
    """
    Computes the signed difference between two angles in degrees,
    returning a result in the range [-180, 180].
    """
    diff_deg = angle1_deg - angle2_deg
    return (diff_deg + 180.0) % 360.0 - 180.0

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info('Control node starting...')

        # --- Parameters ---
        self.declare_parameter('rudder_gain', 1.0)
        self.declare_parameter('rudder_full_scale_deg', 60.0)
        self.declare_parameter('param_file_path', 'argo.yaml')
        
        self.param_file = Path(self.get_parameter('param_file_path').get_parameter_value().string_value)
        self._last_param_mtime = 0
        
        # Load initial parameters and get values
        self.check_and_reload_params(is_initial=True)

        # --- State Variables ---
        self.compass = None          # Current compass heading in degrees
        self.target_compass = None   # Target heading in degrees
        self.radio_sail = 0.0        # Sail command from radio (-1:in, +1:out)
        self.human_control = True    # Start in human control mode for safety
        self.last_logged_human_control = None

        # --- Publishers ---
        self.pub_rudder_sail_cmd = self.create_publisher(Vector3, 'rudder_sail_cmd', 10)

        # --- Subscribers ---
        self.create_subscription(Vector3, '/rudder_sail_radio', self.rudder_sail_radio_callback, 10)
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Bool, '/human_controlled', self.human_control_callback, 10)

        # --- Main Timer ---
        self.control_loop_period = 0.1 # seconds (10 Hz)
        self.timer = self.create_timer(self.control_loop_period, self.timer_callback)
        
        # --- Parameter Reload Timer ---
        self.param_reload_check_period = 3.0 # seconds
        self.param_timer = self.create_timer(self.param_reload_check_period, self.check_and_reload_params)

    def check_and_reload_params(self, is_initial=False):
        """Checks if the param file has changed and reloads it."""
        try:
            if not self.param_file.is_file():
                if self._last_param_mtime != 0: # Log only if file was previously found
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

                # Update internal variables from the parameter server
                self.rudder_gain = self.get_parameter('rudder_gain').get_parameter_value().double_value
                self.RUDDER_FULL_SCALE_DEG = self.get_parameter('rudder_full_scale_deg').get_parameter_value().double_value
                log_level = self.get_logger().info if not is_initial else self.get_logger().debug
                log_level(f"Using params: rudder_gain={self.rudder_gain:.2f}, rudder_full_scale_deg={self.RUDDER_FULL_SCALE_DEG:.1f}")

        except Exception as e:
            self.get_logger().error(f"Error reloading parameters: {e}")

    def rudder_sail_radio_callback(self, msg):
        self.radio_sail = msg.y

    def pose_callback(self, msg):
        self.compass = msg.z # z component is the magnetic compass heading in deg

    def human_control_callback(self, msg):
        self.human_control = msg.data

    def timer_callback(self):
        """Main control loop, executed at 10Hz."""
        # Log human control state changes only once
        if self.human_control != self.last_logged_human_control:
            if self.human_control:
                self.get_logger().info("Human has taken control.")
            else:
                # Log when computer takes control. The target heading is set from the
                # current heading in the logic below.
                self.get_logger().info("Computer has taken control.")
            self.last_logged_human_control = self.human_control

        # --- Main Control Logic ---
        if self.compass is None:
            self.get_logger().debug("Waiting for initial compass heading...", throttle_duration_sec=5)
            return

        if self.human_control:
            # While human is in control, continuously update the target heading.
            # When control is switched to computer, this last heading will be the target.
            self.target_compass = self.compass
            # No command is published; pwm.py will pass through radio commands.
        else:  # Computer control
            if self.target_compass is None:
                self.get_logger().warn("Computer control active, but no target heading. Waiting for human to set a course.", throttle_duration_sec=5)
                return

            # Calculate heading error (target - current). A positive error means we need to turn right (CW).
            compass_err = signed_angle_difference_degrees(self.target_compass, self.compass)

            # Proportional controller for the rudder
            cmd_rudder = self.rudder_gain * (compass_err / self.RUDDER_FULL_SCALE_DEG)
            
            # Clamp command to [-1, 1] range
            cmd_rudder = np.clip(cmd_rudder, -1.0, 1.0)

            # Sail command is passed through from the radio
            cmd_sail = self.radio_sail

            self.get_logger().debug(
                f"Target: {self.target_compass:.1f}, Current: {self.compass:.1f}, "
                f"Err: {compass_err:.1f} -> Rudder Cmd: {cmd_rudder:.2f}"
            )
            
            self.pub_rudder_sail_cmd.publish(Vector3(x=cmd_rudder, y=cmd_sail, z=0.0))

def main(args=None):
    rclpy.init(args=args)
    control_node = None
    try:
        control_node = ControlNode()
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if control_node:
            control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
