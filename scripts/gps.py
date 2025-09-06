#!/usr/bin/env python3
# ROS2 version of gps.py

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import serial
import time
import operator
import argparse
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
    def __init__(self, debug_mode=False):
        super().__init__('gps_node')
        
        # Set logger level to DEBUG if debug mode is enabled
        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self.get_logger().debug('Debug logging enabled')
        
        self.get_logger().info('Initializing GPS node...')

        # Declare and get parameters
        # The GPS Sparkfun NEO uses UART5, which appears at /dev/ttyS5 on Orange Pi Zero 2W
        # UART5 is enabled via the "ph-uart5" overlay in orangepiEnv.txt
        # UART5 pins are TX=11 (PH2) and RX=13 (PH3) on the Orange Pi Zero 2W
        self.declare_parameter('serial_port', '/dev/ttyS5')
        self.declare_parameter('baud_rate', 38400)  # u-blox NEO-M9N default baud rate
        self.declare_parameter('gps_frame_id', 'argo_gps')

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.gps_frame_id = self.get_parameter('gps_frame_id').get_parameter_value().string_value
 
        # Publisher for raw NMEA data
        self.pub_data = self.create_publisher(String, 'gps_data', 10)

        self.serial_port = None
        try:
            self.get_logger().debug(f"Attempting to open serial port {self.serial_port_name} at {self.baud_rate} baud")
            self.serial_port = serial.Serial(
                self.serial_port_name,
                baudrate=self.baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1.0 # Add a timeout for reads
            )
            self.get_logger().info(f"Serial connection established on {self.serial_port_name}")
            self.get_logger().debug(f"Serial port settings: {self.baud_rate} baud, 8N1, 1.0s timeout")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            self.get_logger().error("Shutting down GPS node.")
            # Don't call rclpy.shutdown() in constructor, just let it fail to initialize
            raise e # re-raise to stop node creation

        self.setup_gps()

        # Initialize data counter for debugging
        self.data_count = 0
        self.last_data_log_time = time.time()
        
        # Timer for the main loop to read from the serial port
        self.timer = self.create_timer(0.1, self.read_and_publish) # 10 Hz
        self.get_logger().info("GPS node ready. Listening for NMEA data on /gps_data topic...")

    def checksum(self, sentence: str) -> int:
        """Calculates the NMEA checksum for a sentence."""
        sentence = sentence.strip('\n\r').replace('$', '')
        if '*' in sentence:
            nmeadata, _ = sentence.split('*', 1)
        else:
            nmeadata = sentence
        
        calc_cksum = reduce(operator.xor, (ord(s) for s in nmeadata), 0)
        return calc_cksum

    def send_cmd(self, msg: str, timeout=0.5):
        """Sends a command to the GPS, adding a checksum if needed."""
        if not '*' in msg:  # add checksum
            cs = self.checksum(msg)
            msg = f"{msg}*{cs:02X}"
        
        if not msg.endswith('\r\n'):
            msg = msg + '\r\n'
        
        if self.serial_port and self.serial_port.is_open:
            try:
                # Store original timeout and set a shorter one for commands
                original_timeout = self.serial_port.timeout
                self.serial_port.timeout = timeout
                
                self.serial_port.reset_input_buffer()
                self.serial_port.write(msg.encode('ascii'))
                self.get_logger().debug(f"Sent command: {msg.strip()}")
                
                # Try to read response with short timeout
                reply_bytes = self.serial_port.readline()
                reply = reply_bytes.decode('ascii', errors='ignore').strip()
                
                # Restore original timeout
                self.serial_port.timeout = original_timeout
                
                if reply:
                    self.get_logger().debug(f"Received: {reply}")
                    return reply
                else:
                    self.get_logger().debug("No response received")
                    return None
                    
            except serial.SerialException as e:
                self.get_logger().warn(f"GPS serial port exception while sending command: {e}")
                # Restore original timeout on error
                self.serial_port.timeout = original_timeout
        return None

    def setup_gps(self):
        """Sends initialization commands to the GPS and verifies communication."""
        self.get_logger().info("Setting up u-blox NEO-N9M GPS...")
        
        # First, listen briefly for any automatic output
        self.get_logger().debug("Listening for automatic GPS output...")
        time.sleep(0.5)  # Brief pause to let any automatic data come through
        
        # Check if there's any data waiting
        if self.serial_port.in_waiting > 0:
            try:
                waiting_data = self.serial_port.read(self.serial_port.in_waiting).decode('ascii', errors='ignore')
                if waiting_data.strip():
                    self.get_logger().info("✓ GPS is outputting data automatically!")
                    self.get_logger().debug(f"Sample output: {waiting_data[:100]}...")
                    self.get_logger().info("GPS setup completed. Device is working correctly.")
                    return
            except Exception as e:
                self.get_logger().debug(f"Error reading automatic data: {e}")
        
        # If no automatic data, try communication tests
        communication_ok = False
        self.get_logger().debug("No automatic output detected. Testing GPS communication...")
        
        # Test 1: Request software version (works on most GPS modules)
        self.get_logger().debug("Sending software version query...")
        response = self.send_cmd("$PMTK605", timeout=1.0)  # MTK command for version info
        if response and len(response) > 0:
            self.get_logger().info(f"✓ GPS responded to version query: {response}")
            communication_ok = True
        
        # Test 2: Try u-blox specific version query if MTK didn't work
        if not communication_ok:
            self.get_logger().debug("Trying u-blox position query...")
            response = self.send_cmd("$PUBX,00", timeout=1.0)  # u-blox position query
            if response and len(response) > 0:
                self.get_logger().info(f"✓ GPS responded to u-blox query: {response}")
                communication_ok = True
        
        # Test 3: Try requesting NMEA output rate
        if not communication_ok:
            self.get_logger().debug("Trying NMEA output rate query...")
            response = self.send_cmd("$PMTK414", timeout=1.0)  # MTK query output rate
            if response and len(response) > 0:
                self.get_logger().info(f"✓ GPS responded to rate query: {response}")
                communication_ok = True
        
        # Log communication status
        if communication_ok:
            self.get_logger().info("✓ GPS communication verified - device is responding correctly")
            self.get_logger().info("GPS setup completed. Waiting for NMEA data...")
        else:
            self.get_logger().warn("⚠ GPS communication test failed - no responses received")
            self.get_logger().warn("GPS may still work if it outputs data automatically")
            self.get_logger().info("Continuing to listen for automatic NMEA output...")
        
        self.get_logger().debug("Expected NMEA sentences: GGA, GLL, GSA, GSV, RMC, VTG")

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
                    self.data_count += 1
                    self.get_logger().debug(f"GPS Raw: {data_str}")
                    msg = String()
                    msg.data = data_str
                    self.pub_data.publish(msg)
                    
                    # Log data reception every 10 seconds for debugging
                    current_time = time.time()
                    if current_time - self.last_data_log_time >= 10.0:
                        self.get_logger().info(f"GPS data flowing: {self.data_count} messages received so far")
                        self.last_data_log_time = current_time
                
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
        self.get_logger().info("Shutting down GPS node.")
        if self.serial_port and self.serial_port.is_open:
            # For u-blox modules, we don't need special shutdown commands
            # Just close the serial port cleanly
            self.get_logger().debug("Closing serial connection to GPS")
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='GPS Node for ROS2')
    parser.add_argument('--debug', action='store_true', 
                       help='Enable debug logging')
    
    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args()
    
    # Initialize ROS2 with remaining arguments
    rclpy.init(args=unknown_args)
    node = None
    try:
        node = GpsNode(debug_mode=parsed_args.debug)
        rclpy.spin(node)
    except serial.SerialException as e:
        # This will catch the exception raised from the constructor if the port fails to open.
        rclpy.logging.get_logger('gps_main').fatal(f"Failed to initialize GPS node: {e}")
    except (KeyboardInterrupt, ExternalShutdownException):
        # This handles Ctrl+C or external shutdown requests gracefully.
        if node:
            node.get_logger().info("GPS node interrupted, shutting down gracefully...")
    except Exception as e:
        # Handle any other unexpected exceptions
        if node:
            node.get_logger().error(f"Unexpected error: {e}")
        else:
            print(f"Error before node creation: {e}")
    finally:
        # Clean shutdown
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass  # Suppress any errors during node destruction
        
        try:
            # Check if ROS is still initialized before shutdown
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Suppress ROS shutdown errors (like "already called" errors)
            pass

if __name__ == '__main__':
    main()
