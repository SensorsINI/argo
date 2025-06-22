#!/usr/bin/env python
# ROS2 version of anem.py

#Reading the Sensirion SDP32 sensor
#Dev by JJ SlabbertSDP810_example / modified by tobi
#This script is based on the original anem.py for ROS1.
#It communicates with three differential pressure sensors over I2C to
#determine wind speed and direction.
#Run sudo i2cdetect -y 1 in the terminal, to see if the sensor is connected. it will show address 25
#Check the datasheet at https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf
#The sensor i2c address is 0x21 to 0x23 (Not user programable).

# scripts runs but not yet tested with wind sensor connected

# topics
# publishes
# /anem_speed_angle_temp, Vector3 with:
#   x is speed in m/s
#   y is angle in deg CW from front of boat looking down on boat
#   z is temperature in celsius
# /anem_diffpressure, Vector3 with:
#   .x is from sensor 1 (CCW)
#   .y from sensor 2 (center)
#   .z from sensor 3 (CW)

import rclpy
from rclpy.node import Node
from  geometry_msgs.msg import Vector3
import smbus
import time
import numpy as np

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
    def __init__(self):
        super().__init__('anem_node')
        self.get_logger().info('Initializing Anemometer node...')

        # Publishers
        self.pub_diff_pressure = self.create_publisher(Vector3, 'anem_diffpressure', 10)
        self.pub_wind_temp = self.create_publisher(Vector3, 'anem_speed_angle_temp', 10)

        # I2C setup
        # 21 is CCW, 23 is center, 22 is clockwise (viewed from top)
        self.i2cAddr = (0x21, 0x23, 0x22)
        self.bus = None
        
        try:
            self.bus = smbus.SMBus(1) # The default i2c bus
            self.get_logger().info('Opened i2c SMBus')
        except FileNotFoundError:
            self.get_logger().error("I2C bus not found. Is I2C enabled? Shutting down.")
            rclpy.shutdown()
            return

        if not self.setup_sensors():
            self.get_logger().error("Failed to setup sensors. Shutting down.")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Main loop timer
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info("Initialization of anemometer wind sensor completed.")

    def setup_sensors(self):
        self.get_logger().info('Stopping existing continuous measurements')
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

        except IOError as e:
            self.get_logger().error(f"I2C read error: {e}. Check sensor connections.")
        except IndexError as e:
            self.get_logger().error(f"Data parsing error: {e}. Received incomplete data from sensor.")

    def destroy_node(self):
        # This is the recommended way to perform cleanup in ROS2.
        # It gets called automatically when the node is destroyed.
        self.get_logger().info('Stopping existing continuous measurements on shutdown.')
        if self.bus:
            for a in self.i2cAddr:
                try:
                    self.bus.write_i2c_block_data(a, 0x3F, [0xF9]) # Stop any cont measurement
                except IOError:
                    self.get_logger().warn(f"Could not stop sensor at address {hex(a)} on shutdown.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    anem_node = AnemNode()

    if rclpy.ok():
        try:
            rclpy.spin(anem_node)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt, shutting down anemometer node.")
        finally:
            # Cleanup is handled in destroy_node
            anem_node.destroy_node()
            # rclpy.shutdown() is not called here to avoid "context already shutdown" error
            # when rclpy.spin is interrupted.

if __name__ == '__main__':
    main()
