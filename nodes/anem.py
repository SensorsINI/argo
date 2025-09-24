#!/usr/bin/env python
# ROS2 Anemometer Node - Wind Speed and Direction Sensor

# Reading three Sensirion SDP3x differential pressure sensors
# Dev by JJ Slabbert, modified by tobi
# This ROS2 node communicates with three differential pressure sensors over I2C
# to determine wind speed and direction using directional wind meter principles.
#
# Hardware Setup:
# - Uses I2C bus 0 (not bus 1)
# - Three sensors at addresses 0x21 (CCW), 0x22 (CW), 0x23 (center)
# - Run "sudo i2cdetect -y 0" to verify sensor connections
# - Sensors show as addresses 21, 22, 23 in hex
#
# Algorithm based on Sensirion's directional wind meter application:
# https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/
#
# Features:
# - Automatic sensor reconnection on I2C errors
# - Optional visual debug mode with ASCII wind vector display
# - Temperature compensation and averaging
# - Configurable logging levels
#
# Command line options:
# --debug: Enable debug logging of sensor values
# --debug_visually: Show real-time ASCII visualization of wind vector
#
# Topics Published:
# /anem_speed_angle_temp (geometry_msgs/Vector3):
#   x: wind speed in m/s
#   y: wind angle in degrees CW from front of boat (looking down)
#   z: average temperature in celsius
# /anem_diffpressure (geometry_msgs/Vector3):
#   x: differential pressure from sensor 1 (CCW) in Pascals
#   y: differential pressure from sensor 2 (center) in Pascals  
#   z: differential pressure from sensor 3 (CW) in Pascals

import rclpy
from rclpy.node import Node
from  geometry_msgs.msg import Vector3
import smbus
import time
import numpy as np
import argparse
from rclpy.logging import LoggingSeverity
import math
import sys

# from https://stackoverflow.com/questions/49906101/byte-array-to-int-in-python-2-x-using-standard-libraries
# This function is compatible with Python 3.
def int_from_bytes(b):
    '''Convert big-endian signed integer bytearray to int

    int_from_bytes(b) == int.from_bytes(b, 'big', signed=True)'''
    if not b: # special-case 0 to avoid b[0] raising
        return 0
    n = b[0] & 0x7f # skip sign bit
    for by in b[1:]:
        n = n * 256 + by
    if b[0] & 0x80: # if sign bit is set, 2's complement
        bits = 8*len(b)
        offset = 2**(bits-1)
        return n - offset
    else:
        return n
    

# following from sensirion https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/
  
def calculate_angle_deg(dp1, dp2, dp3):
    # parameter for sinus curve, estimated on one measurement at 7.2 m/s
    b=0.64
    s1 = dp1+dp2
    s2 = dp2+dp3

    if s1!=0:
        g1 = s2/s1
    else:
        g1=1e12
    if g1!=0:
        g2 = 1/g1
    else:
        g2=1e12 # arg large value

    # |g1|==|g2| for omega = 3b/2
    if np.abs(g1)<(3*b/2):
        # lookup based on g1
        magn = 2*np.sqrt(1-g1+g1**2) # Not used in this calculation
        w = np.pi/4+np.arctan((2*g1-1)/np.sqrt(3))-(np.sign(s1)-1)*np.pi/2
    else:
    # lookup based on g2
        w = (np.pi/2-np.arctan((2*g2-1)/np.sqrt(3))-(np.sign(s2)-1)*np.pi/2)
    w=w*(180./np.pi)
    return w

def calculate_speed_mps(dp1,dp2, dp3):
    rho = 1.2
    # scale factor sqrt(2) estimated by measurements
    # A in differential pressure
    A = np.sqrt(2)*(np.abs(dp1) + np.abs(dp2) + np.abs(dp3))
    # A in m/s
    A = np.sqrt(2 * rho * A)
    return A

class AnemNode(Node):
    def __init__(self, debug_visually: bool = False):
        super().__init__('anem_node')
        self.get_logger().info('Initializing Anemometer node...')

        # Publishers
        self.pub_diff_pressure = self.create_publisher(Vector3, 'anem_diffpressure', 10)
        self.pub_wind_temp = self.create_publisher(Vector3, 'anem_speed_angle_temp', 10)

        # Visual debug mode flag
        self.debug_visually = debug_visually

        # I2C setup
        # 21 is CCW, 23 is center, 22 is clockwise (viewed from top)
        self.i2cAddr = (0x21, 0x23, 0x22)
        self.bus = None
        self.sensors_ready = False
        self._last_error_log_time = 0.0
        
        try:
            self.bus = smbus.SMBus(0) # The default i2c bus
            self.get_logger().info('Opened i2c SMBus')
        except FileNotFoundError:
            self.get_logger().error("CRITICAL: I2C bus not found. Is I2C enabled? Exiting.")
            sys.exit(1)

        if self.bus is not None:
            if not self.setup_sensors():
                self.get_logger().fatal("FATAL: Failed to setup anemometer sensors. Exiting.")
                sys.exit(1)
            else:
                self.sensors_ready = True

        # Main loop timer
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        
        # Report actual sensor status
        if self.sensors_ready:
            self.get_logger().info("Initialization of anemometer wind sensor completed successfully.")

        # Visual mode init
        self._vis_initialized = False
        self._vis_width = 41
        self._vis_height = 21
        # Full-scale visual radius corresponds to 15 knots ≈ 7.72 m/s
        self._vis_speed_ref = 15 * 0.514444  # ≈ 7.7167 m/s
        if self.debug_visually:
            self._init_visual()

    def _init_visual(self):
        # Setup terminal for in-place drawing (no scrolling)
        if self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[?25l')  # hide cursor
            sys.stdout.write('\x1b[2J')    # clear screen
            sys.stdout.write('\x1b[H')     # move cursor home
            sys.stdout.flush()
            self._vis_initialized = True
        except Exception:
            self._vis_initialized = False

    def _teardown_visual(self):
        if not self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')   # reset attributes
            sys.stdout.write('\x1b[2J')   # clear
            sys.stdout.write('\x1b[H')    # home
            sys.stdout.write('\x1b[?25h') # show cursor
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False

    def _render_visual(self, speed_mps: float, angle_deg: float, temp_c: float, dp_tuple):
        if not self._vis_initialized:
            self._init_visual()
        width = self._vis_width
        height = self._vis_height
        cx = width // 2
        cy = height // 2

        # Compute endpoint based on angle (CW from forward) and speed
        # Forward is up; x = sin(theta), y = -cos(theta)
        theta = math.radians(angle_deg)
        radius = min(cx - 2, cy - 2)
        scale = radius / max(self._vis_speed_ref, 0.1)
        r = max(0, min(radius, int(round(speed_mps * scale))))
        ex = cx + int(round(r * math.sin(theta)))
        ey = cy + int(round(-r * math.cos(theta)))

        # Build grid
        grid = [[' ' for _ in range(width)] for _ in range(height)]
        # Axes
        for x in range(width):
            grid[cy][x] = '-' if grid[cy][x] == ' ' else grid[cy][x]
        for y in range(height):
            grid[y][cx] = '|' if grid[y][cx] == ' ' else grid[y][cx]
        grid[cy][cx] = '+'

        # Mark endpoint
        if 0 <= ey < height and 0 <= ex < width:
            grid[ey][ex] = 'o'

        # Header lines (fixed count to avoid scrolling)
        header = [
            f"Wind v={speed_mps:5.2f} m/s  angle={angle_deg:6.2f} deg  temp={temp_c:5.1f} C",
            f"dp(Pa)=({dp_tuple[0]:.2f}, {dp_tuple[1]:.2f}, {dp_tuple[2]:.2f})  scale~{self._vis_speed_ref:.1f} m/s->radius",
            "Use Ctrl+C to exit visual mode"
        ]

        # Render
        try:
            sys.stdout.write('\x1b[H')  # move home
            for line in header:
                sys.stdout.write(line.ljust(width) + '\n')
            for row in grid:
                sys.stdout.write(''.join(row) + '\n')
            # Ensure we always write the same number of lines
            sys.stdout.flush()
        except Exception:
            pass

    def setup_sensors(self):
        # First try to communicate with sensors to see if they exist
        sensors_detected = []
        for a in self.i2cAddr:
            try:
                # Try a simple read to test communication
                self.bus.read_byte(a)
                sensors_detected.append(hex(a))
            except IOError:
                pass  # Sensor not detected, continue checking others
        
        if sensors_detected:
            self.get_logger().info(f'Stopping existing continuous measurements on detected sensors: {sensors_detected}')
        else:
            self.get_logger().warn('Wind sensor not detected on I2C bus')
            return False
            
        for a in self.i2cAddr:
            try:
                self.bus.write_i2c_block_data(a, 0x3F, [0xF9]) # Stop any cont measurement
            except IOError as e:
                self.get_logger().error(f"Failed to communicate with sensor at address {hex(a)}: {e}")
                return False
        
        time.sleep(0.8)

        # Start Continuous Measurement (5.3.1 in Data sheet)
        self.get_logger().info('Starting 0x3615 continuous measurement with average till read')
        ##Command code (Hex)        Temperature compensation            Averaging
        ##0x3603                    Mass flow                           Average  till read
        ##0x3608                    Mass flow None                      Update rate 0.5ms
        ##0x3615                    Differential pressure               Average till read
        ##0x361E                    Differential pressure None          Update rate 0.5ms
        for a in self.i2cAddr:
            try:
                self.bus.write_i2c_block_data(a, 0x36, [0x15])
            except IOError as e:
                self.get_logger().error(f"Failed to start measurement on sensor at address {hex(a)}: {e}")
                return False
        
        time.sleep(0.1)
        return True


    def timer_callback(self):
        dp = []
        temps = []
        try:
            if not self.sensors_ready:
                self.get_logger().error("CRITICAL: Anemometer sensors not ready. Exiting.")
                sys.exit(1)
            for a in self.i2cAddr:
                b = self.bus.read_i2c_block_data(a, 0, 9)
                v = int_from_bytes([b[0], b[1]]) / 240.  # convert to Pascals diff pressure
                temp = int_from_bytes([b[3], b[4]]) / 200.  # convert to deg celsius
                dp.append(v)
                temps.append(temp)
            
            # Publish differential pressure
            self.pub_diff_pressure.publish(Vector3(x=float(dp[0]), y=float(dp[1]), z=float(dp[2])))

            # Calculate and publish wind speed, angle, and temperature
            angle_deg = calculate_angle_deg(dp[0], dp[1], dp[2])
            speed_mps = calculate_speed_mps(dp[0], dp[1], dp[2])
            temp_celsius = (temps[0] + temps[1] + temps[2]) / 3.0
            
            self.pub_wind_temp.publish(Vector3(x=float(speed_mps), y=float(angle_deg), z=float(temp_celsius)))
            
            self.get_logger().debug(
                f"Anemometer: speed(m/s)={speed_mps:.2f} angle(deg)={angle_deg:.1f} "
                f"temp(C)={temp_celsius:.1f} dp(pascal)=({dp[0]:.4f}, {dp[1]:.4f}, {dp[2]:.4f})"
            )

            if self.debug_visually:
                self._render_visual(speed_mps, angle_deg, temp_celsius, (dp[0], dp[1], dp[2]))

        except IOError as e:
            # Critical I2C error - exit immediately
            self.get_logger().error(f"CRITICAL: I2C read error: {e}. Exiting.")
            sys.exit(1)
        except IndexError as e:
            self.get_logger().error(f"Data parsing error: {e}. Received incomplete data from sensor.")


    def destroy_node(self):
        # This is the recommended way to perform cleanup in ROS2.
        # It gets called automatically when the node is destroyed.
        self.get_logger().info('Stopping existing continuous measurements on shutdown.')
        if self.debug_visually:
            self._teardown_visual()
        if self.bus:
            for a in self.i2cAddr:
                try:
                    self.bus.write_i2c_block_data(a, 0x3F, [0xF9]) # Stop any cont measurement
                except IOError:
                    self.get_logger().warn(f"Could not stop sensor at address {hex(a)} on shutdown.")
        super().destroy_node()

def main(args=None):
    # Parse CLI args for this script first, pass the remainder to ROS 2
    parser = argparse.ArgumentParser(
        description='Anemometer Node for ROS2 - Wind Speed and Direction Sensor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node reads three Sensirion SDP3x differential pressure sensors over I2C
to determine wind speed and direction using directional wind meter principles.

Hardware Setup:
  - Uses I2C bus 0 (not bus 1)
  - Three sensors at addresses 0x21 (CCW), 0x22 (CW), 0x23 (center)
  - Run "sudo i2cdetect -y 0" to verify sensor connections
  - Sensors show as addresses 21, 22, 23 in hex

Algorithm based on Sensirion's directional wind meter application:
  https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/

Features:
  - Automatic sensor reconnection on I2C errors
  - Optional visual debug mode with ASCII wind vector display
  - Temperature compensation and averaging
  - Configurable logging levels

Topics Published:
  /anem_speed_angle_temp (geometry_msgs/Vector3):
    x: wind speed in m/s
    y: wind angle in degrees CW from front of boat (looking down)
    z: average temperature in celsius
  /anem_diffpressure (geometry_msgs/Vector3):
    x: differential pressure from sensor 1 (CCW) in Pascals
    y: differential pressure from sensor 2 (center) in Pascals  
    z: differential pressure from sensor 3 (CW) in Pascals
        """
    )
    parser.add_argument('--debug', action='store_true', help='Log sensor values to the terminal')
    parser.add_argument('--debug_visually', action='store_true', help='Show test-mode ASCII visualization of wind vector')
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    anem_node = AnemNode(debug_visually=parsed_args.debug_visually)

    if parsed_args.debug_visually:
        # Suppress routine logs to avoid interfering with visual display
        anem_node.get_logger().set_level(LoggingSeverity.WARN)
    elif parsed_args.debug:
        anem_node.get_logger().set_level(LoggingSeverity.DEBUG)
        anem_node.get_logger().info('Debug logging enabled; sensor values will be printed.')

    if rclpy.ok():
        try:
            rclpy.spin(anem_node)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt, shutting down anemometer node.")
        except rclpy.executors.ExternalShutdownException:
            print("External shutdown signal received, exiting gracefully.")
        finally:
            try:
                # Cleanup is handled in destroy_node
                anem_node.destroy_node()
            except Exception:
                pass  # Ignore errors during shutdown
            # rclpy.shutdown() is not called here to avoid "context already shutdown" error
            # when rclpy.spin is interrupted.

if __name__ == '__main__':
    main()
