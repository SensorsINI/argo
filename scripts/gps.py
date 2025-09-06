#!/usr/bin/env python3
# ROS2 version of gps.py

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3
import serial
import time
import operator
import argparse
import re
import math
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
        
        # Publishers for navigation data
        self.pub_sog = self.create_publisher(Float64, 'gps_sog', 10)  # Speed Over Ground (knots)
        self.pub_cog = self.create_publisher(Float64, 'gps_cog', 10)  # Course Over Ground (degrees)
        self.pub_velocity = self.create_publisher(Vector3, 'gps_velocity', 10)  # Combined velocity vector
        
        # Navigation data storage
        self.current_sog = None  # Speed in knots
        self.current_cog = None  # Course in degrees true
        self.gps_fix_valid = False
        self.current_latitude = None  # Latitude in decimal degrees
        self.current_longitude = None  # Longitude in decimal degrees
        self.satellites_used = 0  # Number of satellites used in fix
        
        # Periodic logging control
        self.debug_mode = debug_mode
        self.last_fix_log_time = time.time()
        self.last_no_fix_log_time = time.time()

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
            # Enable SOG/COG sentences since we have communication
            self.enable_nmea_sentences()
            self.get_logger().info("GPS setup completed. Waiting for NMEA data...")
        else:
            self.get_logger().warn("⚠ GPS communication test failed - no responses received")
            self.get_logger().warn("GPS may still work if it outputs data automatically")
            # Try to enable sentences anyway - GPS might still accept configuration
            self.get_logger().info("Attempting to enable SOG/COG sentences...")
            self.enable_nmea_sentences()
            self.get_logger().info("Continuing to listen for automatic NMEA output...")
        
        self.get_logger().debug("Expected NMEA sentences: GGA, GLL, GSA, GSV, RMC, VTG")
        self.get_logger().info("Publishing SOG (Speed Over Ground) to /gps_sog topic")
        self.get_logger().info("Publishing COG (Course Over Ground) to /gps_cog topic")
        self.get_logger().info("Publishing combined velocity vector to /gps_velocity topic")

    def enable_nmea_sentences(self):
        """Enable RMC and VTG NMEA sentences on u-blox NEO-N9M for SOG/COG data."""
        self.get_logger().info("Configuring u-blox NEO-N9M to enable SOG/COG sentences...")
        
        # UBX-CFG-MSG commands to enable NMEA sentences
        # Format: Class ID, Message ID, Rate for each port (DDC, UART1, UART2, USB, SPI, Reserved)
        
        # Enable NMEA RMC (Recommended Minimum Course) - Class 0xF0, ID 0x04
        # Rate 1 = output every measurement cycle
        rmc_enable = bytes([0xB5, 0x62,  # Sync chars
                           0x06, 0x01,  # Class: CFG, ID: MSG
                           0x08, 0x00,  # Length: 8 bytes
                           0xF0, 0x04,  # NMEA RMC message
                           0x00,        # Rate on DDC (I2C)
                           0x01,        # Rate on UART1 (our connection)
                           0x00,        # Rate on UART2
                           0x01,        # Rate on USB
                           0x00,        # Rate on SPI
                           0x00])       # Reserved
        
        # Calculate checksum for RMC command
        rmc_ck_a = 0
        rmc_ck_b = 0
        for byte in rmc_enable[2:]:  # Skip sync chars
            rmc_ck_a = (rmc_ck_a + byte) & 0xFF
            rmc_ck_b = (rmc_ck_b + rmc_ck_a) & 0xFF
        rmc_enable += bytes([rmc_ck_a, rmc_ck_b])
        
        # Enable NMEA VTG (Course Over Ground) - Class 0xF0, ID 0x05
        vtg_enable = bytes([0xB5, 0x62,  # Sync chars
                           0x06, 0x01,  # Class: CFG, ID: MSG
                           0x08, 0x00,  # Length: 8 bytes
                           0xF0, 0x05,  # NMEA VTG message
                           0x00,        # Rate on DDC (I2C)
                           0x01,        # Rate on UART1 (our connection)
                           0x00,        # Rate on UART2
                           0x01,        # Rate on USB
                           0x00,        # Rate on SPI
                           0x00])       # Reserved
        
        # Calculate checksum for VTG command
        vtg_ck_a = 0
        vtg_ck_b = 0
        for byte in vtg_enable[2:]:  # Skip sync chars
            vtg_ck_a = (vtg_ck_a + byte) & 0xFF
            vtg_ck_b = (vtg_ck_b + vtg_ck_a) & 0xFF
        vtg_enable += bytes([vtg_ck_a, vtg_ck_b])
        
        # Send configuration commands
        if self.serial_port and self.serial_port.is_open:
            try:
                self.get_logger().debug("Enabling NMEA RMC sentences...")
                self.serial_port.write(rmc_enable)
                time.sleep(0.1)
                
                self.get_logger().debug("Enabling NMEA VTG sentences...")
                self.serial_port.write(vtg_enable)
                time.sleep(0.1)
                
                self.get_logger().info("✓ SOG/COG sentences enabled on GPS module")
                return True
                
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to configure GPS for SOG/COG: {e}")
                return False
        return False

    def parse_rmc_sentence(self, sentence):
        """Parse GNRMC sentence for SOG and COG data."""
        try:
            # GNRMC format: $GNRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir,mode*checksum
            parts = sentence.split(',')
            if len(parts) >= 9:
                status = parts[2]  # A = valid, V = invalid
                speed_knots = parts[7]  # Speed over ground in knots
                course_true = parts[8]  # Course over ground in degrees true
                
                if status == 'A' and speed_knots and course_true:
                    self.current_sog = float(speed_knots)
                    self.current_cog = float(course_true)
                    self.gps_fix_valid = True
                    self.get_logger().debug(f"RMC: SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°")
                    return True
                else:
                    self.gps_fix_valid = False
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Error parsing RMC sentence: {e}")
        return False

    def parse_vtg_sentence(self, sentence):
        """Parse GNVTG sentence for additional SOG and COG data."""
        try:
            # GNVTG format: $GNVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode*checksum
            parts = sentence.split(',')
            if len(parts) >= 8:
                course_true = parts[1]  # Course over ground, degrees true
                speed_knots = parts[5]  # Speed over ground in knots
                
                if course_true and speed_knots:
                    # Only update if we don't have valid data from RMC or if this is more recent
                    if not self.gps_fix_valid or self.current_sog is None:
                        self.current_sog = float(speed_knots)
                        self.current_cog = float(course_true)
                        self.get_logger().debug(f"VTG: SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°")
                    return True
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Error parsing VTG sentence: {e}")
        return False

    def parse_gga_sentence(self, sentence):
        """Parse GNGGA sentence for position and satellite data."""
        try:
            # GNGGA format: $GNGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,geoid,geoid_unit,dgps_time,dgps_id*checksum
            parts = sentence.split(',')
            if len(parts) >= 15:
                latitude_raw = parts[2]  # Latitude in DDMM.MMMM format
                lat_dir = parts[3]       # N or S
                longitude_raw = parts[4] # Longitude in DDDMM.MMMM format
                lon_dir = parts[5]       # E or W
                fix_quality = parts[6]   # 0=invalid, 1=GPS, 2=DGPS, etc.
                num_sats = parts[7]      # Number of satellites used
                
                if fix_quality and int(fix_quality) > 0 and latitude_raw and longitude_raw and num_sats:
                    # Convert DDMM.MMMM to decimal degrees
                    if latitude_raw:
                        lat_deg = int(latitude_raw[:2])
                        lat_min = float(latitude_raw[2:])
                        self.current_latitude = lat_deg + lat_min / 60.0
                        if lat_dir == 'S':
                            self.current_latitude = -self.current_latitude
                    
                    if longitude_raw:
                        lon_deg = int(longitude_raw[:3])
                        lon_min = float(longitude_raw[3:])
                        self.current_longitude = lon_deg + lon_min / 60.0
                        if lon_dir == 'W':
                            self.current_longitude = -self.current_longitude
                    
                    self.satellites_used = int(num_sats)
                    self.get_logger().debug(f"GGA: Position {self.current_latitude:.6f}°, {self.current_longitude:.6f}°, {self.satellites_used} sats")
                    return True
                    
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Error parsing GGA sentence: {e}")
        return False

    def periodic_status_logging(self):
        """Handle periodic status logging for normal operation."""
        if self.debug_mode:
            return  # Skip periodic logging in debug mode (already has detailed logs)
            
        current_time = time.time()
        
        if self.gps_fix_valid and self.current_latitude is not None and self.current_longitude is not None:
            # Log GPS fix status every 30 seconds
            if current_time - self.last_fix_log_time >= 30.0:
                # Convert SOG from knots to m/s (1 knot = 0.514444 m/s)
                sog_ms = self.current_sog * 0.514444 if self.current_sog is not None else 0.0
                
                # Format coordinates with proper hemisphere indicators
                lat_str = f"{abs(self.current_latitude):.6f}°{'N' if self.current_latitude >= 0 else 'S'}"
                lon_str = f"{abs(self.current_longitude):.6f}°{'E' if self.current_longitude >= 0 else 'W'}"
                
                self.get_logger().info(
                    f"GPS Fix: {self.satellites_used} satellites, "
                    f"Position: {lat_str} {lon_str}, "
                    f"COG: {self.current_cog:.1f}°, SOG: {sog_ms:.2f} m/s"
                )
                self.last_fix_log_time = current_time
                
        else:
            # Log no GPS fix every 5 seconds
            if current_time - self.last_no_fix_log_time >= 5.0:
                self.get_logger().info("GPS: No fix - searching for satellites...")
                self.last_no_fix_log_time = current_time

    def publish_navigation_data(self):
        """Publish SOG and COG data to ROS topics."""
        if self.current_sog is not None and self.current_cog is not None:
            # Publish individual topics
            sog_msg = Float64()
            sog_msg.data = self.current_sog
            self.pub_sog.publish(sog_msg)
            
            cog_msg = Float64()
            cog_msg.data = self.current_cog
            self.pub_cog.publish(cog_msg)
            
            # Publish combined velocity vector (x=north component, y=east component, z=speed magnitude)
            velocity_msg = Vector3()
            # Convert course (degrees from north) and speed to velocity components
            cog_rad = math.radians(self.current_cog)
            velocity_msg.x = self.current_sog * math.cos(cog_rad)  # North component
            velocity_msg.y = self.current_sog * math.sin(cog_rad)  # East component  
            velocity_msg.z = self.current_sog  # Speed magnitude
            self.pub_velocity.publish(velocity_msg)
            
            self.get_logger().debug(f"Published nav data: SOG={self.current_sog:.2f}kt, COG={self.current_cog:.1f}°")

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
                    
                    # Publish raw NMEA data
                    msg = String()
                    msg.data = data_str
                    self.pub_data.publish(msg)
                    
                    # Parse navigation data from specific sentences
                    if data_str.startswith('$GNRMC') or data_str.startswith('$GPRMC'):
                        if self.parse_rmc_sentence(data_str):
                            self.publish_navigation_data()
                    elif data_str.startswith('$GNVTG') or data_str.startswith('$GPVTG'):
                        if self.parse_vtg_sentence(data_str):
                            self.publish_navigation_data()
                    elif data_str.startswith('$GNGGA') or data_str.startswith('$GPGGA'):
                        self.parse_gga_sentence(data_str)
                    
                    # Handle periodic status logging for normal operation
                    self.periodic_status_logging()
                    
                    # Log data reception every 10 seconds for debugging (debug mode only)
                    if self.debug_mode:
                        current_time = time.time()
                        if current_time - self.last_data_log_time >= 10.0:
                            self.get_logger().info(f"GPS data flowing: {self.data_count} messages received so far")
                            if self.gps_fix_valid:
                                self.get_logger().info(f"Current navigation: SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°")
                            else:
                                self.get_logger().info("No GPS fix - navigation data not available")
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
