#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
# ROS2 version of gps.py

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
# Removed toggle_pause_service import - GPS node doesn't need pause functionality
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Float64, UInt8, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import time
import operator
import argparse
import argcomplete
import re
import math
from functools import reduce
import pynmea2



class GpsNode(Node):
    """
    ROS2 GPS node for Argo autonomous sailboat navigation and 3D visualization.

    Interfaces with u-blox NEO-M9N GPS module via UART5 (/dev/ttyS5) and publishes
    comprehensive navigation data for both autonomous control and Foxglove 3D mapping.

    Published Topics:
    - /gps_data (std_msgs/String): Raw NMEA sentences from GPS module
    - /gps_sog (std_msgs/Float64): Speed over ground in knots
    - /gps_cog (std_msgs/Float64): Course over ground in degrees true (0-360°)
    - /gps_velocity (geometry_msgs/Vector3): Velocity vector (x=north, y=east, z=speed)
    - /gps_num_satellites (std_msgs/UInt8): Number of satellites used in GPS fix
    - /fix (sensor_msgs/NavSatFix): Standard GPS fix for mapping applications
    - /gps_health (std_msgs/Bool): Node health status (true=healthy, false=failed)

    Hardware Configuration:
    - GPS: u-blox NEO-M9N via UART5 (/dev/ttyS5)
    - Baud Rate: 38400 (u-blox default)
    - Frame ID: 'argo_gps' (configurable parameter)

    Key Features:
    - Automatic GPS communication verification and setup
    - NMEA sentence parsing (GGA, RMC, VTG) for position and navigation data
    - Standard NavSatFix messages for 3D mapping in Foxglove
    - Velocity vector decomposition for course over ground visualization
    - Robust error handling with communication timeout detection
    - Configurable debug logging for troubleshooting

    For 3D Visualization:
    The /fix topic provides GPS coordinates that can be used directly in Foxglove's
    3D panel for mapping. Combined with /gps_velocity and /gps_cog topics, this
    enables comprehensive boat tracking and navigation visualization.

    Command Line Options:
    --debug: Enable detailed debug logging of GPS data and communication
    """

    def __init__(self, debug_mode=False):
        super().__init__('gps_node')

        # Removed pause service - GPS node doesn't need pause functionality

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
        # u-blox NEO-M9N default baud rate
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('gps_frame_id', 'argo_gps')

        self.serial_port_name = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter(
            'baud_rate').get_parameter_value().integer_value
        self.gps_frame_id = self.get_parameter(
            'gps_frame_id').get_parameter_value().string_value

        # Publisher for raw NMEA data
        self.pub_data = self.create_publisher(String, 'gps_data', 10)

        # Publishers for navigation data
        self.pub_sog = self.create_publisher(
            Float64, 'gps_sog', 10)  # Speed Over Ground (knots)
        self.pub_cog = self.create_publisher(
            Float64, 'gps_cog', 10)  # Course Over Ground (degrees)
        self.pub_velocity = self.create_publisher(
            Vector3, 'gps_velocity', 10)  # Combined velocity vector
        self.pub_satellites = self.create_publisher(
            UInt8, 'gps_num_satellites', 10)  # Number of satellites used

        # Publisher for standard ROS NavSatFix messages (for mapping)
        self.pub_navsat = self.create_publisher(
            NavSatFix, 'fix', 10)  # Standard GPS fix for mapping

        # Health status publisher
        self.pub_health = self.create_publisher(Bool, 'gps_health', 10)
        self.health_status = False  # Track current health status

        # Navigation data storage
        self.current_sog = None  # Speed in knots
        self.current_cog = None  # Course in degrees true
        self.gps_fix_valid = False
        self.current_latitude = None  # Latitude in decimal degrees
        self.current_longitude = None  # Longitude in decimal degrees
        self.satellites_used = 0  # Number of satellites used in fix

        # NavSatFix data storage
        self.current_altitude = None  # Altitude in meters above WGS84 ellipsoid
        self.position_covariance = [0.0] * 9  # Position covariance matrix
        self.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_status = NavSatStatus.STATUS_NO_FIX
        self.navsat_service = NavSatStatus.SERVICE_GPS

        # Periodic logging control
        self.debug_mode = debug_mode
        self.last_fix_log_time = time.time()
        self.last_no_fix_log_time = time.time()

        self.serial_port = None
        try:
            self.get_logger().debug(
                f"Attempting to open serial port {self.serial_port_name} at {self.baud_rate} baud")
            self.serial_port = serial.Serial(
                self.serial_port_name,
                baudrate=self.baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1.0  # Add a timeout for reads
            )
            self.get_logger().info(
                f"Serial connection established on {self.serial_port_name}")
            self.get_logger().debug(
                f"Serial port settings: {self.baud_rate} baud, 8N1, 1.0s timeout")
        except serial.SerialException as e:
            self.get_logger().error(
                f"CRITICAL: Failed to open serial port {self.serial_port_name}: {e}")
            self.get_logger().error("CRITICAL: GPS device not accessible. Exiting.")
            self._publish_health_status(False)
            import sys
            sys.exit(1)

        self.setup_gps()

        # Initialize data counter for debugging
        self.data_count = 0
        self.last_data_log_time = time.time()
        
        # Track NMEA sentence types received
        self.sentence_types_seen = set()
        self.last_sentence_type_log_time = time.time()

        # GPS communication timeout tracking
        self.last_data_received_time = time.time()
        self.gps_timeout_seconds = 30.0  # Fail if no data received for 30 seconds

        # Timer for the main loop to read from the serial port
        self.timer = self.create_timer(0.1, self.read_and_publish)  # 10 Hz
        self.get_logger().info("GPS node ready. Listening for NMEA data on /gps_data topic...")

        # Publish initial health status as unhealthy (no GPS fix yet)
        self._publish_health_status(False)

        # Timer to periodically publish satellite count (ensures zero is published when no fix)
        self.sat_timer = self.create_timer(1.0, self.publish_satellite_count)  # 1 Hz

        # Track pause state to manage power save transitions (PSMOO)
        self._prev_paused = False

    # --- UBX helpers for power save control (PSMOO via CFG-RXM) ---
    def _ubx_checksum(self, payload: bytes) -> bytes:
        ck_a = 0
        ck_b = 0
        for b in payload:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return bytes([ck_a, ck_b])

    def _send_ubx(self, ubx_class: int, ubx_id: int, payload: bytes, expect_ack: bool = True, timeout: float = 0.2) -> bool:
        """Send a UBX message over the existing serial port. Optionally wait for ACK.

        UBX frame: 0xB5 0x62 | class | id | length(LSB,MSB) | payload | CK_A | CK_B
        """
        if not (self.serial_port and self.serial_port.is_open):
            return False
        length = len(payload)
        header = bytes([0xB5, 0x62, ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF])
        ck = self._ubx_checksum(bytes([ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF]) + payload)
        frame = header + payload + ck
        try:
            self.serial_port.write(frame)
            if not expect_ack:
                return True
            # Simple ACK wait loop (reads a small number of bytes with short timeout)
            original_timeout = self.serial_port.timeout
            self.serial_port.timeout = timeout
            resp = self.serial_port.read(32)
            self.serial_port.timeout = original_timeout
            # Look for UBX-ACK-ACK (class 0x05, id 0x01) matching our class/id
            # ACK payload: clsID, msgID
            for i in range(max(0, len(resp) - 10)):
                if i + 10 <= len(resp) and resp[i:i+2] == b"\xB5\x62" and resp[i+2] == 0x05:
                    if resp[i+3] in (0x01, 0x00):  # ACK-ACK or ACK-NAK
                        # length should be 2
                        if i+6 < len(resp) and resp[i+4] == 0x02 and resp[i+5] == 0x00:
                            if i+8 < len(resp) and resp[i+6] == (ubx_class & 0xFF) and resp[i+7] == (ubx_id & 0xFF):
                                return resp[i+3] == 0x01  # True if ACK-ACK
            return False
        except Exception as e:
            self.get_logger().warn(f"UBX send failed: class=0x{ubx_class:02X} id=0x{ubx_id:02X}: {e}")
            return False

    def _cfg_rxm(self, lp_mode: int) -> bool:
        """Send UBX-CFG-RXM to set low power (PSM) mode.

        Payload (length=2): reserved1=0x00, lpMode={0=continuous,1=power save}
        """
        payload = bytes([0x00, 0x01 if lp_mode else 0x00])
        ok = self._send_ubx(0x06, 0x11, payload, expect_ack=True)
        if not ok:
            self.get_logger().warn(f"UBX-CFG-RXM lpMode={lp_mode} not acknowledged")
        else:
            self.get_logger().info(f"Set GPS power mode via CFG-RXM lpMode={lp_mode}")
        return ok

    def _enter_power_save_mode(self) -> None:
        """Enable GPS power save (PSMOO) on pause."""
        try:
            self._cfg_rxm(1)
        except Exception as e:
            self.get_logger().warn(f"Failed to enter GPS power save mode: {e}")

    def _exit_pause_mode(self) -> None:
        """Restore continuous tracking and NMEA output on unpause."""
        try:
            self._cfg_rxm(0)
            # Re-ensure output configuration
            self.enable_nmea_sentences()
        except Exception as e:
            self.get_logger().warn(f"Failed to restore GPS continuous mode: {e}")

    def _publish_health_status(self, is_healthy: bool):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            if is_healthy:
                self.get_logger().info("GPS health status: HEALTHY")
            else:
                self.get_logger().warn("GPS health status: FAILED")

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
                self.get_logger().warn(
                    f"GPS serial port exception while sending command: {e}")
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
                waiting_data = self.serial_port.read(
                    self.serial_port.in_waiting).decode('ascii', errors='ignore')
                if waiting_data.strip():
                    self.get_logger().info("✓ GPS is outputting data automatically!")
                    self.get_logger().debug(
                        f"Sample output: {waiting_data[:100]}...")
                    self.get_logger().info("GPS setup completed. Device is working correctly.")
                    return
            except Exception as e:
                self.get_logger().debug(f"Error reading automatic data: {e}")

        # If no automatic data, try communication tests
        communication_ok = False
        self.get_logger().debug("No automatic output detected. Testing GPS communication...")

        # Test 1: Request software version (works on most GPS modules)
        self.get_logger().debug("Sending software version query...")
        # MTK command for version info
        response = self.send_cmd("$PMTK605", timeout=1.0)
        if response and len(response) > 0:
            self.get_logger().info(
                f"✓ GPS responded to version query: {response}")
            communication_ok = True

        # Test 2: Try u-blox specific version query if MTK didn't work
        if not communication_ok:
            self.get_logger().debug("Trying u-blox position query...")
            # u-blox position query
            response = self.send_cmd("$PUBX,00", timeout=1.0)
            if response and len(response) > 0:
                self.get_logger().info(
                    f"✓ GPS responded to u-blox query: {response}")
                communication_ok = True

        # Test 3: Try requesting NMEA output rate
        if not communication_ok:
            self.get_logger().debug("Trying NMEA output rate query...")
            # MTK query output rate
            response = self.send_cmd("$PMTK414", timeout=1.0)
            if response and len(response) > 0:
                self.get_logger().info(
                    f"✓ GPS responded to rate query: {response}")
                communication_ok = True

        # Log communication status
        if communication_ok:
            self.get_logger().info("✓ GPS communication verified - device is responding correctly")
            # Enable SOG/COG sentences since we have communication
            self.enable_nmea_sentences()
            self.get_logger().info("GPS setup completed. Waiting for NMEA data...")
        else:
            self.get_logger().error("CRITICAL: GPS communication test failed - no responses received")
            self.get_logger().error("CRITICAL: GPS device is not communicating properly. Exiting.")
            self._publish_health_status(False)
            import sys
            sys.exit(1)

        self.get_logger().debug("Expected NMEA sentences: GGA (position/satellites), RMC (speed/course), VTG (course)")
        self.get_logger().info("Publishing SOG (Speed Over Ground) to /gps_sog topic")
        self.get_logger().info("Publishing COG (Course Over Ground) to /gps_cog topic")
        self.get_logger().info("Publishing combined velocity vector to /gps_velocity topic")
        self.get_logger().info("Publishing satellite count to /gps_num_satellites topic")
        self.get_logger().info(
            "Publishing standard NavSatFix messages to /fix topic (for mapping)")

    def enable_nmea_sentences(self):
        """Enable RMC, VTG, and GGA NMEA sentences on u-blox NEO-N9M for navigation and satellite data."""
        self.get_logger().info("Configuring u-blox NEO-N9M to enable navigation sentences...")

        # UBX-CFG-MSG commands to enable NMEA sentences
        # Format: Class ID, Message ID, Rate for each port (DDC, UART1, UART2, USB, SPI, Reserved)

        # Enable NMEA GGA (Global Positioning System Fix Data) - Class 0xF0, ID 0x00
        # This provides position, altitude, and satellite count
        gga_enable = bytes([0xB5, 0x62,  # Sync chars
                           0x06, 0x01,  # Class: CFG, ID: MSG
                           0x08, 0x00,  # Length: 8 bytes
                           0xF0, 0x00,  # NMEA GGA message
                           0x00,        # Rate on DDC (I2C)
                           0x01,        # Rate on UART1 (our connection)
                           0x00,        # Rate on UART2
                           0x01,        # Rate on USB
                           0x00,        # Rate on SPI
                           0x00])       # Reserved

        # Calculate checksum for GGA command
        gga_ck_a = 0
        gga_ck_b = 0
        for byte in gga_enable[2:]:  # Skip sync chars
            gga_ck_a = (gga_ck_a + byte) & 0xFF
            gga_ck_b = (gga_ck_b + gga_ck_a) & 0xFF
        gga_enable += bytes([gga_ck_a, gga_ck_b])

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
                self.get_logger().debug("Enabling NMEA GGA sentences...")
                self.serial_port.write(gga_enable)
                time.sleep(0.1)

                self.get_logger().debug("Enabling NMEA RMC sentences...")
                self.serial_port.write(rmc_enable)
                time.sleep(0.1)

                self.get_logger().debug("Enabling NMEA VTG sentences...")
                self.serial_port.write(vtg_enable)
                time.sleep(0.1)

                self.get_logger().info("✓ Navigation sentences (GGA, RMC, VTG) enabled on GPS module")
                return True

            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Failed to configure GPS navigation sentences: {e}")
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
                course_true = parts[8]  # Course over ground in degrees true (can be empty when stationary)

                if status == 'A' and speed_knots:
                    # We have a valid fix with speed data
                    self.current_sog = float(speed_knots)
                    
                    # Course may be empty when stationary (speed near zero)
                    if course_true:
                        self.current_cog = float(course_true)
                        self.get_logger().debug(
                            f"RMC: SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°")
                    else:
                        # Keep previous COG value or set to None if stationary
                        # Don't clear it - vessel may have stopped but still has heading
                        self.get_logger().debug(
                            f"RMC: SOG={self.current_sog:.2f} knots, COG=N/A (stationary)")
                    
                    self.gps_fix_valid = True
                    # Set NavSat status to indicate we have a fix
                    if self.navsat_status == NavSatStatus.STATUS_NO_FIX:
                        self.navsat_status = NavSatStatus.STATUS_FIX
                        # Update health status when we get a fix
                        self._publish_health_status(True)
                    return True
                else:
                    # Invalid status - no GPS fix
                    if self.gps_fix_valid:  # Only update health if fix status changed
                        self._publish_health_status(False)
                    self.gps_fix_valid = False
                    self.navsat_status = NavSatStatus.STATUS_NO_FIX
                    # Clear navigation data when fix is lost
                    self.current_sog = None
                    self.current_cog = None
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Error parsing RMC sentence: {e}")
        return False

    def parse_vtg_sentence(self, sentence):
        """Parse GNVTG sentence for additional SOG and COG data."""
        try:
            # GNVTG format: $GNVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode*checksum
            parts = sentence.split(',')
            if len(parts) >= 8:
                course_true = parts[1]  # Course over ground, degrees true (can be empty when stationary)
                speed_knots = parts[5]  # Speed over ground in knots

                # Only process VTG data if we have a valid GPS fix
                # VTG sentences can contain data even without a fix, so we need to check fix status
                if speed_knots and self.gps_fix_valid:
                    # Only update if we don't have valid data from RMC
                    if self.current_sog is None:
                        self.current_sog = float(speed_knots)
                        
                        # Course may be empty when stationary
                        if course_true:
                            self.current_cog = float(course_true)
                            self.get_logger().debug(
                                f"VTG: SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°")
                        else:
                            self.get_logger().debug(
                                f"VTG: SOG={self.current_sog:.2f} knots, COG=N/A (stationary)")
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
                longitude_raw = parts[4]  # Longitude in DDDMM.MMMM format
                lon_dir = parts[5]       # E or W
                fix_quality = parts[6]   # 0=invalid, 1=GPS, 2=DGPS, etc.
                num_sats = parts[7]      # Number of satellites used
                hdop = parts[8]          # Horizontal dilution of precision
                altitude_raw = parts[9]  # Altitude above mean sea level
                # Altitude units (usually M for meters)
                alt_unit = parts[10]

                # Always extract satellite count for GPS health monitoring, even without fix
                if num_sats is not None and num_sats != '':
                    self.satellites_used = int(num_sats)

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

                    # Extract altitude
                    if altitude_raw:
                        self.current_altitude = float(altitude_raw)
                        # Convert to meters if needed (usually already in meters)
                        if alt_unit != 'M':
                            self.get_logger().debug(
                                f"Unexpected altitude unit: {alt_unit}")

                    # Update NavSat status and GPS fix validity based on fix quality
                    if not self.gps_fix_valid:  # Only update health if fix status changed
                        self._publish_health_status(True)
                    if int(fix_quality) == 1:
                        self.navsat_status = NavSatStatus.STATUS_FIX
                        self.gps_fix_valid = True
                    elif int(fix_quality) == 2:
                        self.navsat_status = NavSatStatus.STATUS_SBAS_FIX
                        self.gps_fix_valid = True
                    elif int(fix_quality) >= 4:
                        self.navsat_status = NavSatStatus.STATUS_GBAS_FIX
                        self.gps_fix_valid = True
                    else:
                        self.navsat_status = NavSatStatus.STATUS_FIX
                        self.gps_fix_valid = True

                    # Update position covariance based on HDOP if available
                    if hdop:
                        hdop_val = float(hdop)
                        # Rough conversion from HDOP to position variance (in meters^2)
                        # This is a simplified approximation
                        variance = (hdop_val * 2.0) ** 2
                        self.position_covariance[0] = variance  # East-East
                        self.position_covariance[4] = variance  # North-North
                        # Up-Up (usually worse than horizontal)
                        self.position_covariance[8] = variance * 2
                        self.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                    self.get_logger().debug(
                        f"GGA: Position {self.current_latitude:.6f}°, {self.current_longitude:.6f}°, Alt {self.current_altitude}m, {self.satellites_used} sats")
                    return True
                else:
                    # No valid fix - clear GPS fix status
                    if self.gps_fix_valid:  # Only update health if fix status changed
                        self._publish_health_status(False)
                    self.gps_fix_valid = False
                    self.navsat_status = NavSatStatus.STATUS_NO_FIX
                    # Clear navigation data when fix is lost
                    self.current_sog = None
                    self.current_cog = None
                    self.current_latitude = None
                    self.current_longitude = None
                    self.current_altitude = None

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Error parsing GGA sentence: {e}")
        return False

    def periodic_status_logging(self):
        """Handle periodic status logging for normal operation."""
        if self.debug_mode:
            # Skip periodic logging in debug mode (already has detailed logs)
            return

        current_time = time.time()

        if self.gps_fix_valid and self.current_latitude is not None and self.current_longitude is not None:
            # Log GPS fix status every 30 seconds
            if current_time - self.last_fix_log_time >= 30.0:
                # Convert SOG from knots to m/s (1 knot = 0.514444 m/s)
                sog_ms = self.current_sog * 0.514444 if self.current_sog is not None else 0.0

                # Format coordinates with proper hemisphere indicators
                lat_str = f"{abs(self.current_latitude):.6f}°{'N' if self.current_latitude >= 0 else 'S'}"
                lon_str = f"{abs(self.current_longitude):.6f}°{'E' if self.current_longitude >= 0 else 'W'}"

                alt_str = f", Alt: {self.current_altitude:.1f}m" if self.current_altitude is not None else ""
                cog_str = f"COG: {self.current_cog:.1f}°" if self.current_cog is not None else "COG: N/A"
                self.get_logger().info(
                    f"GPS Fix: {self.satellites_used} satellites, "
                    f"Position: {lat_str} {lon_str}{alt_str}, "
                    f"{cog_str}, SOG: {sog_ms:.2f} m/s"
                )
                self.last_fix_log_time = current_time

        else:
            # Log no GPS fix every 5 seconds
            # After startup, log "no fix" every 5s for the first 3 messages, then every 30s
            if not hasattr(self, 'no_fix_log_count'):
                self.no_fix_log_count = 0

            # Determine log interval based on current count
            if self.no_fix_log_count < 3:
                log_interval = 5.0
            else:
                log_interval = 60.0

            # Check if enough time has passed since last log
            if current_time - self.last_no_fix_log_time >= log_interval:
                self.get_logger().info("GPS: No fix - searching for satellites...")
                self.last_no_fix_log_time = current_time
                self.no_fix_log_count += 1

    def publish_navigation_data(self):
        """Publish SOG and COG data to ROS topics."""
        if self.current_sog is not None and self.gps_fix_valid:
            # Always publish speed if we have a valid fix
            sog_msg = Float64()
            sog_msg.data = self.current_sog
            self.pub_sog.publish(sog_msg)

            # Publish velocity vector
            velocity_msg = Vector3()
            if self.current_cog is not None:
                # We have course - publish COG and calculate velocity components
                cog_msg = Float64()
                cog_msg.data = self.current_cog
                self.pub_cog.publish(cog_msg)

                # Convert course (degrees from north) and speed to velocity components
                cog_rad = math.radians(self.current_cog)
                velocity_msg.x = self.current_sog * math.cos(cog_rad)  # North component
                velocity_msg.y = self.current_sog * math.sin(cog_rad)  # East component
                velocity_msg.z = self.current_sog  # Speed magnitude
                
                self.get_logger().debug(
                    f"Published nav data: SOG={self.current_sog:.2f}kt, COG={self.current_cog:.1f}°")
            else:
                # Stationary or no course available - velocity components are zero
                velocity_msg.x = 0.0  # North component
                velocity_msg.y = 0.0  # East component
                velocity_msg.z = self.current_sog  # Speed magnitude (usually very small when stationary)
                
                self.get_logger().debug(
                    f"Published nav data: SOG={self.current_sog:.2f}kt, COG=N/A (stationary)")
            
            self.pub_velocity.publish(velocity_msg)
        else:
            # Debug: Log why navigation data is not being published
            self.get_logger().debug(
                f"NOT publishing nav data: SOG={self.current_sog}, fix_valid={self.gps_fix_valid}")

    def publish_satellite_count(self):
        """Publish satellite count for GPS health monitoring (called at 1 Hz)."""
        # Always publish satellite count, even if zero (valuable for GPS health monitoring)
        sat_msg = UInt8()
        sat_msg.data = self.satellites_used
        self.pub_satellites.publish(sat_msg)
        
        # Log satellite count periodically in debug mode
        if self.debug_mode:
            self.get_logger().debug(f"Published satellite count: {self.satellites_used}")

    def publish_navsat_fix(self):
        """Publish NavSatFix message for mapping applications."""
        if (self.current_latitude is not None and
            self.current_longitude is not None and
                self.gps_fix_valid):

            # Create NavSatFix message
            navsat_msg = NavSatFix()

            # Header
            navsat_msg.header.stamp = self.get_clock().now().to_msg()
            navsat_msg.header.frame_id = self.gps_frame_id

            # Status
            navsat_msg.status.status = self.navsat_status
            navsat_msg.status.service = self.navsat_service

            # Position
            navsat_msg.latitude = self.current_latitude
            navsat_msg.longitude = self.current_longitude
            navsat_msg.altitude = self.current_altitude if self.current_altitude is not None else 0.0

            # Covariance
            navsat_msg.position_covariance = self.position_covariance
            navsat_msg.position_covariance_type = self.position_covariance_type

            # Publish
            self.pub_navsat.publish(navsat_msg)

            self.get_logger().debug(
                f"Published NavSatFix: {self.current_latitude:.6f}°, {self.current_longitude:.6f}°, Alt: {self.current_altitude}m")

    def read_and_publish(self):
        """Reads data from the serial port and publishes it."""
        # Removed pause functionality - GPS node runs continuously

        # GPS node runs continuously without pause functionality

        # Check for GPS communication timeout
        current_time = time.time()
        if current_time - self.last_data_received_time > self.gps_timeout_seconds:
            self.get_logger().error(
                f"CRITICAL: GPS communication timeout - no data received for {self.gps_timeout_seconds} seconds")
            self.get_logger().error(
                "CRITICAL: GPS device appears to have stopped communicating. Exiting.")
            self._publish_health_status(False)
            import sys
            sys.exit(1)

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
                    self.last_data_received_time = current_time  # Reset timeout
                    self.get_logger().debug(f"GPS Raw: {data_str}")
                    
                    # Track sentence types seen
                    if data_str.startswith('$'):
                        sentence_type = data_str.split(',')[0]
                        self.sentence_types_seen.add(sentence_type)

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
                        # Always parse to extract satellite count
                        self.parse_gga_sentence(data_str)
                        self.publish_navsat_fix()  # Only publishes if valid fix
                        # Note: satellite count published by periodic timer (1 Hz)

                    # Handle periodic status logging for normal operation
                    self.periodic_status_logging()

                    # Log sentence types seen every 15 seconds
                    if current_time - self.last_sentence_type_log_time >= 15.0:
                        if self.sentence_types_seen:
                            sentence_list = sorted(list(self.sentence_types_seen))
                            self.get_logger().debug(f"NMEA sentence types received: {', '.join(sentence_list)}")
                            has_gga = any('GGA' in s for s in self.sentence_types_seen)
                            if not has_gga:
                                self.get_logger().warn("No GGA sentences received - satellite count unavailable")
                        self.last_sentence_type_log_time = current_time
                    
                    # Log data reception every 10 seconds for debugging (debug mode only)
                    if self.debug_mode:
                        if current_time - self.last_data_log_time >= 10.0:
                            self.get_logger().info(
                                f"GPS data flowing: {self.data_count} messages received so far")
                            if self.gps_fix_valid:
                                if self.current_sog is not None and self.current_cog is not None:
                                    self.get_logger().info(
                                        f"GPS Fix: SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°")
                                elif self.current_sog is not None:
                                    self.get_logger().info(
                                        f"GPS Fix: SOG={self.current_sog:.2f} knots, COG=N/A (stationary)")
                                else:
                                    self.get_logger().info("GPS Fix obtained, waiting for navigation data")
                            else:
                                self.get_logger().info("No GPS fix - searching for satellites...")
                            self.last_data_log_time = current_time

                # The original script performed manual parsing of NMEA sentences
                # and used a ROS1-specific library (libnmea_navsat_driver).
                # This functionality is removed because:
                # 1. The library is not available in ROS2.
                # 2. The standard `nmea_navsat_driver` ROS2 package should be used
                #    for parsing NMEA and publishing standard sensor messages.
                # This node's primary purpose is now to provide the raw data stream.

            except serial.SerialException as e:
                self.get_logger().error(f'CRITICAL: Serial port error: {e}')
                self.get_logger().error('CRITICAL: GPS device communication lost. Exiting.')
                self._publish_health_status(False)
                import sys
                sys.exit(1)
            except Exception as e:
                # Log unexpected errors as warnings but don't exit the node
                # This allows the node to recover from transient errors
                self.get_logger().warn(
                    f'Unexpected error in GPS processing: {e}', throttle_duration_sec=5.0)
                # Don't exit - continue operation and try to recover

    def destroy_node(self):
        """Gracefully shutdown the node and the GPS device."""
        self.get_logger().info("Shutting down GPS node.")

        # Publish health status as failed on shutdown
        self._publish_health_status(False)

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
        rclpy.logging.get_logger('gps_main').fatal(
            f"Failed to initialize GPS node: {e}")
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
