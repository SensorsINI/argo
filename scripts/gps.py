#!/usr/bin/env python3
# ROS2 version of gps.py

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import serial
import time
import operator
from functools import reduce

class GpsNode(Node):
    """
    A ROS2 node to interface with a NMEA-compliant GPS device over a serial port.
    This node reads raw NMEA sentences from the GPS, sends initialization and
    shutdown commands, and publishes the raw sentences to the /gps_data topic.

    This script is a migration of a ROS1 script. It intentionally keeps the
    functionality of publishing raw NMEA strings. For a more robust solution
    that publishes standard sensor_msgs/NavSatFix messages, it is highly
    recommended to use the standard ROS2 `nmea_navsat_driver` package.
    This node can be used to feed the raw data to `nmea_navsat_driver`.
    """
    def __init__(self):
        super().__init__('gps_node')
        self.get_logger().info('Initializing GPS node...')

        # Declare and get parameters
        # The GPS Sparkfun NEO uses UART5, but this does not appear at ttyS5 as one might expect, but rather at /dev/ttyS0 using the
        # standard overlay (in orangepiEnv.txt) that has "overlays=ph-uart5"
        # not confirmed, since I have not yet seen output from the NEO

        # the problem is that I did not rewire the GPS RX and TX lines to the new serial port UART5 which are
        # pins TX=11 (PH2) and RX=13 (PH3) on the OPi. 
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('gps_frame_id', 'argo_gps')

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.gps_frame_id = self.get_parameter('gps_frame_id').get_parameter_value().string_value
 
        # Publisher for raw NMEA data
        self.pub_data = self.create_publisher(String, 'gps_data', 10)

        self.serial_port = None
        try:
            self.serial_port = serial.Serial(
                self.serial_port_name,
                baudrate=self.baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1.0 # Add a timeout for reads
            )
            self.get_logger().info(f"Serial connection established on {self.serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            self.get_logger().error("Shutting down GPS node.")
            # Don't call rclpy.shutdown() in constructor, just let it fail to initialize
            raise e # re-raise to stop node creation

        self.setup_gps()

        # Timer for the main loop to read from the serial port
        self.timer = self.create_timer(0.1, self.read_and_publish) # 10 Hz
        self.get_logger().info("GPS setup completed. Reading data...")

    def checksum(self, sentence: str) -> int:
        """Calculates the NMEA checksum for a sentence."""
        sentence = sentence.strip('\n\r').replace('$', '')
        if '*' in sentence:
            nmeadata, _ = sentence.split('*', 1)
        else:
            nmeadata = sentence
        
        calc_cksum = reduce(operator.xor, (ord(s) for s in nmeadata), 0)
        return calc_cksum

    def send_cmd(self, msg: str):
        """Sends a command to the GPS, adding a checksum if needed."""
        if not '*' in msg:  # add checksum
            cs = self.checksum(msg)
            msg = f"{msg}*{cs:02X}"
        
        if not msg.endswith('\r\n'):
            msg = msg + '\r\n'
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.write(msg.encode('ascii'))
                time.sleep(0.1) # Give device time to process
                reply_bytes = self.serial_port.readline()
                reply = reply_bytes.decode('ascii', errors='ignore').strip()
                self.get_logger().info(f"Sent: {msg.strip()} | Received: {reply}")
                return reply
            except serial.SerialException as e:
                self.get_logger().warn(f"GPS serial port exception while sending command: {e}")
        return None

    def setup_gps(self):
        """Sends initialization commands to the GPS."""
        self.get_logger().info("Getting GPS version info...")
        hwVersion = self.send_cmd("$PTNLQVR,H")
        swVersion = self.send_cmd("$PTNLQVR,S")
        navVersion = self.send_cmd("$PTNLQVR,Nn")
        
        if hwVersion is None or swVersion is None or navVersion is None:
            self.get_logger().warn('Could not get all version information.')
        else:
            self.get_logger().info(f"GPS version info: HW={hwVersion} SW={swVersion} NAV={navVersion}")

        # The original script had many commented-out commands.
        # They are omitted here for clarity. If needed, they can be added back.
        self.get_logger().info("Setup commands sent.")

    def read_and_publish(self):
        """Reads data from the serial port and publishes it."""
        # The `in_waiting` check is not strictly necessary because `readline()`
        # with a timeout will block until a line is received or the timeout occurs.
        # We just need to ensure the port is open.
        if self.serial_port and self.serial_port.is_open:
            try:
                # Readline() will read until a newline or timeout
                data_bytes = self.serial_port.readline()
                data_str = data_bytes.decode('ascii', errors='ignore').strip()
                
                if data_str:
                    self.get_logger().debug(f"GPS Raw: {data_str}")
                    msg = String()
                    msg.data = data_str
                    self.pub_data.publish(msg)
                
                # The original script performed manual parsing of NMEA sentences
                # and used a ROS1-specific library (libnmea_navsat_driver).
                # This functionality is removed because:
                # 1. The library is not available in ROS2.
                # 2. The standard `nmea_navsat_driver` ROS2 package should be used
                #    for parsing NMEA and publishing standard sensor messages.
                # This node's primary purpose is now to provide the raw data stream.

            except serial.SerialException as e:
                self.get_logger().warn(f'Could not read data from serial port: {e}')
            except Exception as e:
                self.get_logger().error(f'An unexpected error occurred in read_and_publish: {e}')

    def destroy_node(self):
        """Gracefully shutdown the node and the GPS device."""
        self.get_logger().info("Shutting down, sending final command to GPS.")
        if self.serial_port and self.serial_port.is_open:
            # Shutdown, store data to flash, wakeup on next serial port on A or B ports
            self.send_cmd("$PTNLSRT,S,2,3")
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsNode()
        rclpy.spin(node)
    except serial.SerialException as e:
        # This will catch the exception raised from the constructor if the port fails to open.
        rclpy.logging.get_logger('gps_main').fatal(f"Failed to initialize GPS node: {e}")
    except (KeyboardInterrupt, ExternalShutdownException):
        # This handles Ctrl+C or external shutdown requests gracefully.
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
