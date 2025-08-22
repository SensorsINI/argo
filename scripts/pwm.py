#!/usr/bin/env python
# ROS2 version of pwm.py
# Captures and publishes the rudder and sail winch servo positions.
# Uses the argo_radio_servo_module sysfs interface.

# topics
# publishes
# /rudder_sail_radio, -1:1 Vector3 with .x=rudder, .y=sail, .z reserved (currently 0)
#   values are normalized to -1:+1 assuming range of pulse width is 1000us to 2000us
#   rudder -1 means full left rudder (turn CCW looking down on boat), +1 is full right rudder
#   sail -1 means pulled in fully, +1 let out fully
# /rudder_sail_servo, -1:1 Vector3 with similar normalized actual servo commands
# /human_controlled: True if human has taken control, False if rudder is left near neutral for some time

# subscribes to
# /rudder_sail_cmd: -1:+1 range Vector3 rudder and sail commands (from control.py)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

import time
from pathlib import Path

# --- Configuration ---
SYS_BASE_PATH = Path("/sys/kernel/argo_radio_servo")
RADIO_RUDDER_PATH = SYS_BASE_PATH / "radio_rudder_pw_us"
RADIO_SAIL_PATH = SYS_BASE_PATH / "radio_sail_pw_us"
SERVO_RUDDER_PATH = SYS_BASE_PATH / "servo_rudder_pw_us"
SERVO_SAIL_PATH = SYS_BASE_PATH / "servo_sail_pw_us"

HUMAN_CONTROL_TIMEOUT_S = 2.0
# time in seconds that human takes control after rudder command deviates by
# more than HUMAN_CONTROL_THRESHOLD
HUMAN_CONTROL_THRESHOLD = 0.2
# threshold deviation from zero for radio rudder input that human takes control

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

        # Publishers
        self.pub_rudder_sail_radio = self.create_publisher(Vector3, 'rudder_sail_radio', 10)
        self.pub_rudder_sail_servo = self.create_publisher(Vector3, 'rudder_sail_servo', 10)
        self.pub_human_controlled = self.create_publisher(Bool, 'human_controlled', 10)

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
        """Writes a pulse width to a sysfs file."""
        try:
            path.write_text(str(value))
            self.get_logger().debug(f"Wrote {value} to {path}")
        except IOError as e:
            self.get_logger().error(f"Error writing to {path}: {e}")

    def main_loop(self):
        # 1. Read radio inputs from sysfs
        radio_rudder_pw_us = self.read_sysfs_pw(RADIO_RUDDER_PATH)
        radio_sail_pw_us = self.read_sysfs_pw(RADIO_SAIL_PATH)

        # 2. Validate and normalize radio inputs
        if not (500 < radio_rudder_pw_us < 2500 and 500 < radio_sail_pw_us < 2500):
            self.get_logger().warn(f"Outlier radio PWM: rudder={radio_rudder_pw_us:.1f}us, sail={radio_sail_pw_us:.1f}us")
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
        if abs(radio_rudder_normalized) > HUMAN_CONTROL_THRESHOLD:
            self.time_last_human_cmd = time.time()
            if not self.human_control:
                self.get_logger().info("Human took control")
                self.pub_human_controlled.publish(Bool(data=True))

        human_control_now = (time.time() - self.time_last_human_cmd) < HUMAN_CONTROL_TIMEOUT_S
        if not human_control_now and self.human_control:
            self.get_logger().info("Computer took control")
            self.pub_human_controlled.publish(Bool(data=False))
        
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
        self.pub_human_controlled.publish(Bool(data=self.human_control))

def main(args=None):
    rclpy.init(args=args)
    pwm_node = PwmNode()
    
    if rclpy.ok():
        try:
            rclpy.spin(pwm_node)
        except KeyboardInterrupt:
            # The rclpy signal handler initiates shutdown on Ctrl+C.
            # We can't use the logger here as the context may be invalid.
            print("\nKeyboard interrupt, shutting down.")
        finally:
            pwm_node.destroy_node()
            # rclpy.shutdown() is not called here to avoid the "context already shutdown" error.

if __name__ == '__main__':
    main()
