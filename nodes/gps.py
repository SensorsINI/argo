#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
# ROS2 version of gps.py

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
# Import ArgoBaseNode for standardized functionality
from argo_base_node import ArgoBaseNode
import rclpy
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



class GpsNode(ArgoBaseNode):
    """
    ROS2 GPS node for Argo autonomous sailboat navigation and 3D visualization.

    Interfaces with u-blox NEO-M9N GPS module via UART5 (/dev/ttyS5) and publishes
    comprehensive navigation data for both autonomous control and Foxglove 3D mapping.
    
    Protocol Reference: u-blox M9-MDR-2.16 Interface Description (UBX-22037308), Protocol version 35.16

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
    - Baud Rate: 9600 (temporarily changed for testing - normally 38400)
    - Frame ID: 'argo_gps' (configurable parameter)
    - PPS Output: 1Hz pulse, 100ms duration, active when GPS locked

    Key Features:
    - Automatic GPS communication verification and setup
    - Hot start configuration for fast GPS acquisition (CFG-BAT, CFG-NAV5, CFG-PMS)
    - NMEA sentence parsing (GGA, RMC, VTG) for position and navigation data
    - Standard NavSatFix messages for 3D mapping in Foxglove
    - Velocity vector decomposition for course over ground visualization
    - Real-time satellite count monitoring with immediate change detection
    - GPS fix status monitoring with PPS correlation logging
    - Position accuracy reporting (HDOP and 95% confidence circle)
      HDOP = Horizontal Dilution of Precision (lower values indicate better accuracy)
    - Fix quality reporting (GPS Fix, SBAS Fix, GBAS Fix)
    - Robust error handling with communication timeout detection
    - Configurable debug logging for troubleshooting

    Real-time Monitoring:
    - Satellite count changes are logged and published immediately
    - GPS fix acquisition/loss triggers immediate PPS status logging
    - Position accuracy and fix quality included in status logs
    - First GPS fix is logged immediately upon acquisition

    Hot Start Capability:
    - Configures backup battery for almanac/ephemeris retention
    - Optimizes navigation engine settings for faster acquisition
    - Enables power management for continuous operation
    - Reduces cold start delays when power is maintained

    GPS Reset Mechanisms and Startup Modes:
    
    The NEO-M9N supports multiple reset types and startup modes:
    
    1. Cold Start (navBbrMask=0x0004):
       - Clears ALL data: ephemeris, almanac, position, and time
       - GPS must search entire time/frequency space and all satellite numbers
       - Time-To-First-Fix (TTFF): ~24-36 seconds under good conditions
       - Used when GPS has been without fix for extended periods (10+ minutes)
       - Automatic cold start reset after 10 minutes without fix (configurable)
    
    2. Warm Start (navBbrMask=0x0002):
       - Clears ephemeris and almanac, but keeps position and time
       - GPS has approximate location and time, but needs fresh satellite data
       - TTFF: ~15-30 seconds (faster than cold start)
       - Used when GPS has valid time but satellite data is stale (4+ hours)
    
    3. Hot Start (navBbrMask=0x0001):
       - Clears ephemeris only, keeps almanac, position, and time
       - GPS knows its location, time, and which satellites should be visible
       - TTFF: ~2 seconds (fastest acquisition)
       - Requires backup battery to maintain data during power cycles
       - Used when GPS was recently powered down (<4 hours)
    
    4. GNSS-Only Restart (resetType=0x02):
       - Restarts only GNSS tasks without reloading configuration
       - Fastest reset option, doesn't clear any data
       - Useful for recovering from temporary GNSS task errors
    
    Automatic Reset:
    - If GPS runs without a fix for 10 minutes, automatic cold start reset is triggered
    - Minimum 2 minutes between resets to prevent reset loops
    - Resets clear stale data that may prevent satellite acquisition
    - After reset, GPS reinitializes and should acquire satellites within 30-60 seconds
    
    Why GPS May Fail to Acquire Satellites After Extended No-Fix:
    - Stale ephemeris data becomes invalid after ~4 hours
    - Stale almanac data becomes invalid after several days
    - GPS search algorithms may get stuck in ineffective search patterns
    - Power management states may interfere with acquisition attempts
    - Solution: Cold start reset clears all stale data and forces fresh acquisition

    For 3D Visualization:
    The /fix topic provides GPS coordinates that can be used directly in Foxglove's
    3D panel for mapping. Combined with /gps_velocity and /gps_cog topics, this
    enables comprehensive boat tracking and navigation visualization.

    Command Line Options:
    --debug: Enable detailed debug logging of GPS data and communication
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
        # u-blox NEO-M9N default baud rate (temporarily 9600 for testing)
        self.declare_parameter('baud_rate', 9600)
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

        # Health status is now handled by ArgoBaseNode

        # Navigation data storage
        self.current_sog = None  # Speed in knots
        self.current_cog = None  # Course in degrees true
        self.gps_fix_valid = False
        self.current_latitude = None  # Latitude in decimal degrees
        self.current_longitude = None  # Longitude in decimal degrees
        self.satellites_used = 0  # Number of satellites used in fix
        self.last_satellite_count = 0  # Track previous satellite count for change detection
        self.last_fix_status = False  # Track previous fix status for change detection

        # NavSatFix data storage
        self.current_altitude = None  # Altitude in meters above WGS84 ellipsoid
        self.current_hdop = None  # Horizontal Dilution of Precision
        self.position_covariance = [0.0] * 9  # Position covariance matrix
        self.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_status = NavSatStatus.STATUS_NO_FIX
        self.navsat_service = NavSatStatus.SERVICE_GPS

        # Periodic logging control
        self.debug_mode = debug_mode
        self.last_fix_log_time = 0.0  # Initialize to 0 so first fix is logged immediately
        self.last_no_fix_log_time = time.time()
        self.first_fix_logged = False  # Track if we've logged the first fix

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
            self.set_unhealthy("GPS device not accessible")
            import sys
            sys.exit(1)

        self.setup_gps()

        # Initialize data counter for debugging
        self.data_count = 0
        self.last_data_log_time = time.time()
        
        # Track NMEA sentence types received
        self.sentence_types_seen = set()
        self.last_sentence_type_log_time = time.time()
        # Track first reception of expected NMEA sentences for debugging
        self.first_gga_received = False
        self.first_rmc_received = False
        self.first_vtg_received = False

        # GPS communication timeout tracking
        self.last_data_received_time = time.time()
        # Increased timeout: GPS may take time to output NMEA, especially indoors without fix
        # But we need to detect actual communication failures
        self.gps_timeout_seconds = 60.0  # Fail if no valid NMEA received for 60 seconds (when no satellites)
        self.last_satellites_seen_time = None  # Track when we last saw satellites (satellites_used > 0)

        # GPS reset tracking for extended no-fix periods
        self.last_fix_time = time.time()  # Initialize to startup time (will be cleared when fix acquired)
        self.no_fix_reset_timeout_seconds = 600.0  # Auto-reset after 10 minutes without fix
        self.last_reset_time = None  # Track when we last reset to avoid reset loops
        self.min_reset_interval_seconds = 120.0  # Minimum 2 minutes between resets

        # Timer for the main loop to read from the serial port
        self.timer = self.create_timer(0.1, self.read_and_publish)  # 10 Hz
        self.get_logger().info("GPS node ready. Listening for NMEA data on /gps_data topic...")

        # Set initial health status as unhealthy (no GPS fix yet)
        self.set_unhealthy("No GPS fix yet")

        # Timer to periodically publish satellite count (ensures zero is published when no fix)
        self.sat_timer = self.create_timer(1.0, self.publish_satellite_count)  # 1 Hz
        
        # Timer to periodically publish NavSatFix at 1Hz (for consistent visualization rate)
        self.navsat_timer = self.create_timer(1.0, self.publish_navsat_fix)  # 1 Hz

        # Timer to check for extended no-fix condition and trigger reset if needed
        self.reset_check_timer = self.create_timer(60.0, self.check_and_reset_if_needed)  # Check every 60 seconds

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
            # Flush input buffer before sending to avoid reading stale data
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)
            
            self.serial_port.write(frame)
            if self.debug_mode:
                self.get_logger().debug(f"Sent UBX: class=0x{ubx_class:02X} id=0x{ubx_id:02X} payload_len={length}")
            
            if not expect_ack:
                return True
            
            # Wait for ACK with multiple read attempts (GPS may be slow to respond)
            original_timeout = self.serial_port.timeout
            self.serial_port.timeout = 0.1  # Short timeout per read
            resp = bytearray()
            start_time = time.time()
            
            # Read multiple times up to the timeout
            while time.time() - start_time < timeout:
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    resp.extend(chunk)
                time.sleep(0.05)  # Small delay between reads
            
            self.serial_port.timeout = original_timeout
            
            if self.debug_mode and len(resp) > 0:
                self.get_logger().debug(f"UBX response ({len(resp)} bytes): {resp.hex()[:80]}")
            
            # Look for UBX-ACK-ACK (class 0x05, id 0x01) or ACK-NAK (class 0x05, id 0x00) matching our class/id
            # ACK frame: 0xB5 0x62 0x05 0x01/0x00 length(LSB,MSB) clsID msgID CK_A CK_B
            # Total length: 10 bytes (2 sync + 1 class + 1 id + 2 length + 2 payload + 2 checksum)
            # Convert bytearray to bytes for easier slicing
            resp_bytes = bytes(resp) if isinstance(resp, bytearray) else resp
            
            for i in range(len(resp_bytes) - 9):  # Need at least 10 bytes, so i can go up to len-10
                if i + 10 <= len(resp_bytes) and resp_bytes[i:i+2] == b"\xB5\x62" and resp_bytes[i+2] == 0x05:
                    # Check if this is an ACK (0x01) or NAK (0x00)
                    if resp_bytes[i+3] in (0x01, 0x00):
                        # Check length is 2 bytes (little-endian: LSB=0x02, MSB=0x00)
                        if resp_bytes[i+4] == 0x02 and resp_bytes[i+5] == 0x00:
                            # Check if this ACK matches our command (class and id)
                            if resp_bytes[i+6] == (ubx_class & 0xFF) and resp_bytes[i+7] == (ubx_id & 0xFF):
                                # Verify checksum
                                ack_payload = resp_bytes[i+2:i+8]  # Class through msgID
                                ck_a, ck_b = self._ubx_checksum(ack_payload)
                                if resp_bytes[i+8] == ck_a and resp_bytes[i+9] == ck_b:
                                    is_ack = resp_bytes[i+3] == 0x01
                                    if self.debug_mode:
                                        ack_type = "ACK-ACK" if is_ack else "ACK-NAK"
                                        self.get_logger().debug(f"Received UBX {ack_type} for class=0x{ubx_class:02X} id=0x{ubx_id:02X}")
                                    return is_ack
                                elif self.debug_mode:
                                    self.get_logger().debug(f"ACK checksum mismatch: expected {ck_a:02X}{ck_b:02X}, got {resp_bytes[i+8]:02X}{resp_bytes[i+9]:02X}")
                            elif self.debug_mode:
                                self.get_logger().debug(f"ACK class/id mismatch: expected class=0x{ubx_class:02X} id=0x{ubx_id:02X}, got class=0x{resp_bytes[i+6]:02X} id=0x{resp_bytes[i+7]:02X}")
            
            if self.debug_mode and len(resp) > 0:
                self.get_logger().debug(f"No matching ACK found in response. Looking for class=0x{ubx_class:02X} id=0x{ubx_id:02X}")
            
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

    # --- UBX reset mechanisms (CFG-RST) ---
    def gps_reset(self, nav_bbr_mask: int = 0x0000, reset_mode: int = 0x08) -> bool:
        """
        Send UBX-CFG-RST to reset the GPS module.
        
        Args:
            nav_bbr_mask: Navigation BBR (battery-backed RAM) mask:
                          - 0x0001: Hot start (clear ephemeris, keep almanac/position/time)
                          - 0x0002: Warm start (clear ephemeris/almanac, keep position/time)
                          - 0x0004: Cold start (clear all data: ephemeris, almanac, position, time)
            reset_mode: Reset mode:
                          - 0x08: Controlled software reset (GNSS stops then restarts, preserves config)
                          - 0x09: Hardware reset immediately (full reset, may lose config)
        
        Returns:
            True if reset command was sent successfully (note: GPS will reset, so ACK may not be received)
        
        Note:
            For u-blox NEO-M9N, UBX-CFG-RST does not have a separate resetType field.
            The resetMode determines the type of reset (controlled vs hardware).
        """
        if not (self.serial_port and self.serial_port.is_open):
            return False
        
        # UBX-CFG-RST payload: navBbrMask (2 bytes), resetMode (1 byte), reserved1 (1 byte)
        payload = bytes([
            (nav_bbr_mask >> 8) & 0xFF,  # navBbrMask MSB
            nav_bbr_mask & 0xFF,          # navBbrMask LSB
            reset_mode & 0xFF,           # resetMode (0x08=controlled, 0x09=hardware)
            0x00                          # reserved1
        ])
        
        # Send reset command (don't wait for ACK as GPS will reset)
        ok = self._send_ubx(0x06, 0x04, payload, expect_ack=False, timeout=0.1)
        return ok

    def gps_cold_start(self) -> bool:
        """Perform a cold start reset - clears all data (ephemeris, almanac, position, time).
        
        NOTE: This performs a controlled software reset which may temporarily stop NMEA output.
        After calling this, ensure enable_nmea_sentences() is called to restore NMEA output.
        Time-To-First-Fix (TTFF) after cold start: ~24-29 seconds under good conditions.
        """
        self.get_logger().info("Performing GPS cold start reset (clearing all data: ephemeris, almanac, position, time)...")
        return self.gps_reset(nav_bbr_mask=0x0004, reset_mode=0x08)

    def clear_navigation_data(self) -> bool:
        """Clear navigation BBR data (ephemeris, almanac) without doing a full reset.
        
        This uses UBX-CFG-CFG to clear only the navigation BBR, preserving port configuration
        and other settings. This is safer than cold_start() as it doesn't stop NMEA output.
        
        Returns:
            True if command was sent successfully
        """
        if not (self.serial_port and self.serial_port.is_open):
            return False
        
        self.get_logger().info("Clearing navigation data (ephemeris, almanac) without reset...")
        
        # UBX-CFG-CFG: Clear navigation BBR (battery-backed RAM)
        # This clears ephemeris and almanac data without doing a full reset
        # Device mask: clear all (0xFFFFFFFF)
        # Clear mask: navigation BBR only (0x00000001 = navBbr)
        device_mask = bytes([0xFF, 0xFF, 0xFF, 0xFF])  # All devices
        clear_mask = bytes([0x01, 0x00, 0x00, 0x00])   # Clear navBBR only
        save_mask = bytes([0x00, 0x00, 0x00, 0x00])    # Don't save to flash
        load_mask = bytes([0x00, 0x00, 0x00, 0x00])    # Don't load from flash
        
        payload = device_mask + clear_mask + save_mask + load_mask
        
        ok = self._send_ubx(0x06, 0x09, payload, expect_ack=True, timeout=2.0)
        if ok:
            self.get_logger().info("Navigation data cleared. GPS will reacquire fresh satellite data...")
        else:
            self.get_logger().warn("Navigation data clear command may have failed (no ACK received)")
        
        return ok

    def gps_warm_start(self) -> bool:
        """Perform a warm start reset - clears ephemeris and almanac, keeps position and time.
        
        Time-To-First-Fix (TTFF) after warm start: ~15-30 seconds.
        """
        self.get_logger().info("Performing GPS warm start reset (clearing ephemeris/almanac, keeping position/time)...")
        return self.gps_reset(nav_bbr_mask=0x0002, reset_mode=0x08)

    def gps_hot_start_reset(self) -> bool:
        """Perform a hot start reset - clears ephemeris only, keeps almanac, position, and time.
        
        Time-To-First-Fix (TTFF) after hot start: ~2 seconds (fastest).
        """
        self.get_logger().info("Performing GPS hot start reset (clearing ephemeris only, keeping almanac/position/time)...")
        return self.gps_reset(nav_bbr_mask=0x0001, reset_mode=0x08)

    def gps_gnss_restart(self) -> bool:
        """Perform a GNSS-only restart - restarts GNSS tasks without reloading configuration.
        
        This uses a controlled software reset with no BBR clear (navBbrMask=0x0000).
        Fastest reset option, doesn't clear any data.
        """
        self.get_logger().info("Performing GPS GNSS-only restart (no data cleared)...")
        return self.gps_reset(nav_bbr_mask=0x0000, reset_mode=0x08)

    def gps_factory_reset(self) -> bool:
        """Perform a hardware factory reset - completely restores factory defaults.
        
        WARNING: This will clear ALL configuration including:
        - Port settings (baud rate, protocol masks)
        - Message rates
        - Navigation settings
        - Power management settings
        
        After this reset, the GPS must be fully re-configured.
        """
        self.get_logger().warn("Performing GPS hardware factory reset (restoring factory defaults)...")
        self.get_logger().warn("WARNING: All configuration will be lost! GPS must be re-configured after reset.")
        # Hardware reset immediately (reset_mode=0x09) with cold start mask
        # This performs a full hardware reset, not just a controlled software reset
        return self.gps_reset(nav_bbr_mask=0x0004, reset_mode=0x09)

    def check_and_reset_if_needed(self):
        """Check if GPS has been without a fix for too long and trigger reset if needed.
        
        Also detects the condition where many satellites are visible but no fix is acquired,
        which often indicates stale almanac/ephemeris data requiring a cold start.
        """
        current_time = time.time()
        
        # Only check if we don't have a fix
        if not self.gps_fix_valid:
            # Check for "many satellites but no fix" condition (stale satellite data)
            # This is a strong indicator that the GPS has stale almanac/ephemeris
            # that prevents it from computing a position despite seeing satellites
            if (self.satellites_used >= 8 and 
                self.last_satellites_seen_time is not None and
                (self.last_reset_time is None or 
                 (current_time - self.last_reset_time) >= self.min_reset_interval_seconds)):
                # Give GPS a reasonable time to acquire fix with satellites (2 minutes)
                # If it still hasn't fixed after 2 minutes with 8+ satellites, something is wrong
                time_with_satellites = current_time - self.last_satellites_seen_time
                if time_with_satellites >= 120.0:  # 2 minutes with satellites but no fix
                    self.get_logger().warn(
                        f"GPS has {self.satellites_used} satellites in view but no fix after "
                        f"{time_with_satellites/60:.1f} minutes. "
                        f"This likely indicates stale satellite data. Clearing navigation data...")
                    # Use clear_navigation_data() for this case (satellites visible but no fix)
                    # This is less aggressive than cold start and preserves NMEA output
                    if self.clear_navigation_data():
                        self.last_reset_time = current_time
                        self.last_fix_time = current_time
                        self.get_logger().info("Navigation data cleared. Waiting 25 seconds for GPS to process clear command...")
                        time.sleep(25.0)  # Allow time for GPS to clear BBR and start reacquiring
                        self.get_logger().info("Navigation data clear completed. GPS should reacquire fix with fresh satellite data...")
                    else:
                        self.get_logger().error("Failed to clear navigation data, trying warm start instead...")
                        # Fallback to warm start if CFG-CFG fails
                        if self.gps_warm_start():
                            self.last_reset_time = current_time
                            self.last_fix_time = current_time
                            self.get_logger().info("Warm start reset sent. Waiting 15 seconds for GPS to reinitialize...")
                            time.sleep(15.0)  # Allow time for warm start TTFF + NMEA re-enable
                            self.enable_nmea_sentences()
                            self.get_logger().info("Warm start completed. GPS will reacquire fresh satellite data...")
                    return
            
            # Calculate time without fix (from last fix time or startup)
            time_without_fix = current_time - self.last_fix_time
            
            # Check if we need to reset (only if enough time has passed since last reset)
            if time_without_fix >= self.no_fix_reset_timeout_seconds:
                if (self.last_reset_time is None or 
                    (current_time - self.last_reset_time) >= self.min_reset_interval_seconds):
                    self.get_logger().warn(
                        f"GPS has been without fix for {time_without_fix/60:.1f} minutes. "
                        f"Performing hardware factory reset to clear all state and recover from potential firmware issues...")
                    # For extended no-fix periods (10+ minutes), use hardware factory reset to clear ALL state
                    # This is more aggressive than cold start - it performs a full hardware reset (like power cycle)
                    # which can recover from firmware bugs, illegal commands, or hung states
                    # Hardware reset (0x09) clears ALL configuration, so GPS must be fully reconfigured
                    if self.gps_factory_reset():
                        self.last_reset_time = current_time
                        self.last_fix_time = current_time
                        self.get_logger().warn("Hardware factory reset sent. GPS will reboot and all configuration will be cleared.")
                        self.get_logger().info("Waiting 5 seconds for GPS to reboot after hardware reset...")
                        time.sleep(5.0)  # Give GPS time to reboot after hardware reset
                        # Close and reopen serial port (GPS may have reset to default baud rate)
                        if self.serial_port and self.serial_port.is_open:
                            self.serial_port.close()
                            self.get_logger().debug("Closed serial port for GPS reboot")
                        time.sleep(2.0)  # Additional wait for GPS to fully boot
                        # Reopen serial port (may need to try different baud rates)
                        try:
                            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1.0)
                            time.sleep(1.0)  # Allow serial port to stabilize
                            self.get_logger().info("Serial port reopened after GPS reboot")
                        except Exception as e:
                            self.get_logger().error(f"Failed to reopen serial port after factory reset: {e}")
                            # Try to recover by reopening with setup
                            self.setup_gps()
                            return
                        # Reconfigure GPS completely (factory reset cleared all config)
                        self.get_logger().info("Reconfiguring GPS after factory reset...")
                        self.setup_gps()  # Full reconfiguration (port, NMEA, hot start, PPS)
                        self.get_logger().info("Hardware factory reset completed. GPS should reacquire satellites within 30-60 seconds.")
                    else:
                        # Fallback to warm start if cold start fails
                        self.get_logger().warn("Cold start failed, trying warm start reset...")
                        if self.gps_warm_start():
                            self.last_reset_time = current_time
                            self.last_fix_time = current_time
                            self.get_logger().info("Warm start reset sent. Waiting 15 seconds for GPS to reinitialize...")
                            time.sleep(15.0)  # Allow time for warm start TTFF (~15-30s) + NMEA re-enable + margin
                            self.enable_nmea_sentences()
                            self.get_logger().info("Warm start completed. GPS should reacquire satellites within 30-60 seconds.")
                        else:
                            self.get_logger().error("Failed to send GPS reset command")
        else:
            # We have a fix - we already updated last_fix_time when fix was acquired
            pass

    # Health status publishing is now handled by ArgoBaseNode

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
        time.sleep(1.0)  # Brief pause to let any automatic data come through

        # Check if there's any data waiting
        if self.serial_port.in_waiting > 0:
            try:
                waiting_data = self.serial_port.read(
                    self.serial_port.in_waiting).decode('ascii', errors='ignore')
                if waiting_data.strip():
                    self.get_logger().info("✓ GPS is outputting data automatically!")
                    self.get_logger().debug(
                        f"Sample output: {waiting_data[:100]}...")
                    # Still configure NMEA sentences and hot start even with automatic output
                    self.enable_nmea_sentences()
                    self.get_logger().info("GPS setup completed. Device is working correctly.")
                    return
            except Exception as e:
                self.get_logger().debug(f"Error reading automatic data: {e}")

        # If no automatic data, try communication tests
        communication_ok = False
        self.get_logger().debug("No automatic output detected. Testing GPS communication...")

        # Test 1: Request software version (works on NEO-N9M GPS modules)
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
            
            # Test if GPS accepts UBX commands and query software version
            self.get_logger().debug("Testing UBX command acceptance and querying software version...")
            # Send UBX MON-VER (version query) - no ACK expected for this query
            ubx_test = self._send_ubx(0x0A, 0x04, b'', expect_ack=False, timeout=1.0)
            time.sleep(0.5)
            
            if self.serial_port.in_waiting > 0:
                ubx_resp = self.serial_port.read(self.serial_port.in_waiting)
                if len(ubx_resp) >= 8 and ubx_resp[0:2] == b'\xB5\x62':
                    self.get_logger().info("✓ GPS accepts UBX commands")
                    
                    # Parse MON-VER response if we got one
                    if ubx_resp[2] == 0x0A and ubx_resp[3] == 0x04:  # MON-VER response
                        # MON-VER format: sync(2) class(1) id(1) length(2) software_version(30) hardware_version(10) extension...
                        # Length is little-endian at bytes 4-5
                        length = ubx_resp[4] + (ubx_resp[5] << 8)
                        if length >= 30 and len(ubx_resp) >= 8 + length:
                            # Software version is null-terminated string starting at byte 6
                            sw_version_end = ubx_resp.find(b'\x00', 6, 6 + 30)
                            if sw_version_end > 6:
                                sw_version = ubx_resp[6:sw_version_end].decode('ascii', errors='ignore')
                                self.get_logger().info(f"✓ GPS Software Version: {sw_version}")
                                
                                # Check for bootloader mode or firmware issues
                                if "ROM BOOT" in sw_version.upper() or "BOOT" in sw_version.upper():
                                    self.get_logger().error("✗ CRITICAL: GPS appears to be in BOOTLOADER mode!")
                                    self.get_logger().error("✗ GPS firmware may be corrupted or GPS needs firmware update")
                                    self.get_logger().error("✗ GPS will NOT acquire satellites or output NMEA in bootloader mode")
                                    self.get_logger().error("✗ This explains why there are no PPS pulses - GPS is not running normal firmware")
                            
                            # Hardware version is null-terminated string starting after software version
                            hw_start = 6 + 30
                            if len(ubx_resp) >= hw_start + 10:
                                hw_version_end = ubx_resp.find(b'\x00', hw_start, hw_start + 10)
                                if hw_version_end > hw_start:
                                    hw_version = ubx_resp[hw_start:hw_version_end].decode('ascii', errors='ignore')
                                    self.get_logger().info(f"✓ GPS Hardware Version: {hw_version}")
                            
                            # Extension strings (if present)
                            ext_start = 6 + 30 + 10
                            if len(ubx_resp) > ext_start:
                                # Extension strings are null-terminated, multiple strings possible
                                ext_data = ubx_resp[ext_start:]
                                ext_strings = []
                                i = 0
                                while i < len(ext_data):
                                    if ext_data[i] == 0:
                                        i += 1
                                        continue
                                    ext_end = ext_data.find(b'\x00', i)
                                    if ext_end > i:
                                        ext_str = ext_data[i:ext_end].decode('ascii', errors='ignore')
                                        if ext_str:
                                            ext_strings.append(ext_str)
                                        i = ext_end + 1
                                    else:
                                        break
                                if ext_strings:
                                    self.get_logger().info(f"✓ GPS Extension Info: {', '.join(ext_strings)}")
                        else:
                            self.get_logger().debug(f"MON-VER response too short: length={length}, received={len(ubx_resp)}")
                    else:
                        self.get_logger().debug(f"Received UBX response but not MON-VER: class=0x{ubx_resp[2]:02X} id=0x{ubx_resp[3]:02X}")
                else:
                    self.get_logger().warn("⚠ GPS may not be accepting UBX commands - responses may not be UBX format")
            else:
                self.get_logger().warn("⚠ No UBX response received - GPS may not accept UBX commands")
            
            # Enable SOG/COG sentences since we have communication
            self.enable_nmea_sentences()
            self.get_logger().info("GPS setup completed. Waiting for NMEA data...")
        else:
            self.get_logger().error("CRITICAL: GPS communication test failed - no responses received")
            self.get_logger().error("CRITICAL: GPS device is not communicating properly. Exiting.")
            self.set_unhealthy("GPS device not accessible")
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

        # CRITICAL: First configure UART port (CFG-PRT) to ensure NMEA protocol is enabled
        # Reference: u-blox M9-MDR-2.16 Interface Description (UBX-22037308), Protocol version 35.16
        # The port must have NMEA output protocol enabled, or no NMEA sentences will be sent
        # Even if CFG-MSG enables specific NMEA messages, they won't output if protocol is disabled
        # Format: portID=1 (UART1), txReady=0, mode=4 bytes (8N1, 9600 baud),
        #         inProtoMask=0x07 (UBX + NMEA + RTCM3 input), outProtoMask=0x07 (UBX + NMEA + RTCM3 output)
        # Mode format: [charLen(4 bits)|reserved(4 bits), parity(2 bits)|nStopBits(2 bits)|reserved(4 bits), 
        #               baudRate(16 bits, little-endian)]
        # For 9600 baud 8N1: charLen=8 (0x08), parity=none (0x0), nStopBits=1 (0x0), baudRate=9600 (0x2580)
        # Note: Temporarily using 9600 baud for testing (normally 38400)
        # Protocol masks: 0x07 = UBX(bit0=1) + NMEA(bit1=1) + RTCM3(bit2=1) = all protocols enabled
        uart1_cfg_prt = bytes([0xB5, 0x62,  # Sync chars
                              0x06, 0x00,  # Class: CFG, ID: PRT
                              0x14, 0x00,  # Length: 20 bytes
                              0x01,        # Port ID: 1 (UART1)
                              0x00,        # Reserved
                              0x00, 0x00,  # TX Ready: disabled (little-endian)
                              0x08,        # Mode[0]: charLen=8 (bits 0-3), reserved (bits 4-7)
                              0x00,        # Mode[1]: parity=none (bits 0-1), nStopBits=1 (bits 2-3), reserved (bits 4-7)
                              0x80, 0x25,  # Mode[2:3]: baudRate=9600 (0x2580, little-endian 16-bit)
                              0x00, 0x00,  # Reserved
                              0x07, 0x00,  # In Protocol Mask: 0x07 = UBX(1) + NMEA(2) + RTCM3(4) input enabled
                              0x07, 0x00,  # Out Protocol Mask: 0x07 = UBX(1) + NMEA(2) + RTCM3(4) output enabled
                              0x00, 0x00,  # Flags
                              0x00, 0x00, 0x00, 0x00])  # Reserved
        
        # Calculate checksum for CFG-PRT
        prt_ck_a = 0
        prt_ck_b = 0
        for byte in uart1_cfg_prt[2:]:  # Skip sync chars
            prt_ck_a = (prt_ck_a + byte) & 0xFF
            prt_ck_b = (prt_ck_b + prt_ck_a) & 0xFF
        uart1_cfg_prt += bytes([prt_ck_a, prt_ck_b])

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
                # CRITICAL: Configure UART port to enable NMEA protocol output first
                # Without this, even if sentences are enabled, they won't be output
                # Reference: u-blox M9-MDR-2.16 Interface Description (UBX-22037308), Protocol version 35.16
                self.get_logger().debug("Configuring UART1 port to enable NMEA protocol...")
                # Extract payload (skip sync chars and checksum)
                prt_payload = uart1_cfg_prt[6:-2]
                prt_result = self._send_ubx(0x06, 0x00, prt_payload, expect_ack=True, timeout=2.0)
                if prt_result:
                    self.get_logger().debug("✓ UART1 port configuration ACK received")
                else:
                    # GPS rejected port configuration - this can happen if:
                    # 1. GPS is already configured correctly (factory defaults may match)
                    # 2. GPS doesn't allow port reconfiguration in current state
                    # 3. Configuration parameters are invalid
                    self.get_logger().warn("⚠ UART1 port configuration rejected (ACK-NAK)")
                    self.get_logger().info("  GPS may already be configured correctly, or using factory defaults")
                    self.get_logger().info("  Continuing with NMEA sentence configuration...")
                
                self.get_logger().debug("Enabling NMEA GGA sentences...")
                gga_payload = gga_enable[6:-2]
                if self._send_ubx(0x06, 0x01, gga_payload, expect_ack=True, timeout=1.0):
                    self.get_logger().debug("✓ GGA enable ACK received")
                else:
                    self.get_logger().warn("⚠ GGA enable sent but no ACK received")
                time.sleep(0.1)

                self.get_logger().debug("Enabling NMEA RMC sentences...")
                rmc_payload = rmc_enable[6:-2]
                if self._send_ubx(0x06, 0x01, rmc_payload, expect_ack=True, timeout=1.0):
                    self.get_logger().debug("✓ RMC enable ACK received")
                else:
                    self.get_logger().warn("⚠ RMC enable sent but no ACK received")
                time.sleep(0.1)
                
                self.get_logger().debug("Enabling NMEA VTG sentences...")
                vtg_payload = vtg_enable[6:-2]
                if self._send_ubx(0x06, 0x01, vtg_payload, expect_ack=True, timeout=1.0):
                    self.get_logger().debug("✓ VTG enable ACK received")
                else:
                    self.get_logger().warn("⚠ VTG enable sent but no ACK received")
                time.sleep(0.1)

                self.get_logger().info("✓ Navigation sentences (GGA, RMC, VTG) enabled on GPS module")
                
                # Flush any pending UBX ACK/NAK messages from the configuration commands
                # These are binary messages that would otherwise be read as invalid data
                time.sleep(0.5)  # Give GPS time to send ACKs
                try:
                    if self.serial_port.in_waiting > 0:
                        # Read and discard pending UBX ACK messages
                        self.serial_port.read(self.serial_port.in_waiting)
                        self.get_logger().debug("Flushed pending UBX ACK messages")
                except Exception:
                    pass
                
                # Configure GPS for hot start capability
                self.configure_hot_start()
                
                # Final flush after all configuration
                time.sleep(0.5)
                try:
                    if self.serial_port.in_waiting > 0:
                        self.serial_port.read(self.serial_port.in_waiting)
                        self.get_logger().debug("Flushed final UBX ACK messages")
                except Exception:
                    pass
                
                return True

            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Failed to configure GPS navigation sentences: {e}")
                return False
        return False

    def configure_hot_start(self):
        """Configure GPS for hot start capability with backup battery and almanac retention."""
        self.get_logger().info("Configuring GPS for hot start capability...")
        
        try:
            # Configure backup battery for hot start (CFG-BAT)
            # This enables the GPS to maintain almanac/ephemeris data during power cycles
            backup_battery_cfg = bytes([0xB5, 0x62,  # Sync chars
                                       0x06, 0x09,  # Class: CFG, ID: BAT
                                       0x04, 0x00,  # Length: 4 bytes
                                       0x01,        # Enable backup battery
                                       0x00,        # Reserved
                                       0x00,        # Reserved
                                       0x00])       # Reserved
            
            # Calculate checksum
            backup_battery_cfg += self._ubx_checksum(backup_battery_cfg[2:])
            
            # Send backup battery configuration
            self.serial_port.write(backup_battery_cfg)
            time.sleep(0.1)
            self.get_logger().debug("✓ Backup battery configuration sent")
            
            # Configure navigation engine settings for hot start (CFG-NAV5)
            # This optimizes the GPS for faster acquisition using cached data
            nav5_cfg = bytes([0xB5, 0x62,  # Sync chars
                             0x06, 0x24,  # Class: CFG, ID: NAV5
                             0x24, 0x00,  # Length: 36 bytes
                             0xFF, 0xFF,  # Mask: apply all settings
                             0x06,        # Dynamic model: automotive
                             0x03,        # Fix mode: auto 2D/3D
                             0x00, 0x00,  # Fixed altitude (not used)
                             0x10, 0x27,  # Fixed altitude variance (not used)
                             0x05,        # Minimum elevation: 5 degrees
                             0x00,        # Reserved
                             0xFA, 0x00,  # Position DOP mask: 250 (DOP = Dilution of Precision)
                             0xFA, 0x00,  # Time DOP mask: 250 (DOP = Dilution of Precision)
                             0x64, 0x00,  # Position accuracy mask: 100m
                             0x2C, 0x01,  # Time accuracy mask: 300s
                             0x00,        # Static hold threshold: 0 cm/s
                             0x00,        # DGNSS timeout: 0s
                             0x00,        # CNO threshold: 0 dBHz
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00,        # Reserved
                             0x00])       # Reserved
            
            # Calculate checksum
            nav5_cfg += self._ubx_checksum(nav5_cfg[2:])
            
            # Send navigation configuration
            self.serial_port.write(nav5_cfg)
            time.sleep(0.1)
            self.get_logger().debug("✓ Navigation engine configuration sent")
            
            # Configure power management for hot start (CFG-PMS)
            # This ensures the GPS maintains satellite data during low power states
            pms_cfg = bytes([0xB5, 0x62,  # Sync chars
                            0x06, 0x86,  # Class: CFG, ID: PMS
                            0x08, 0x00,  # Length: 8 bytes
                            0x00,        # Power setup value: full power
                            0x00,        # Period: 0 (continuous)
                            0x00,        # On time: 0 (continuous)
                            0x00,        # Reserved
                            0x00,        # Reserved
                            0x00,        # Reserved
                            0x00,        # Reserved
                            0x00])       # Reserved
            
            # Calculate checksum
            pms_cfg += self._ubx_checksum(pms_cfg[2:])
            
            # Send power management configuration
            self.serial_port.write(pms_cfg)
            time.sleep(0.1)
            self.get_logger().debug("✓ Power management configuration sent")
            
            # Configure PPS (Pulse Per Second) output
            self.configure_pps_output()
            
            # Query GPS status to diagnose hardware issues
            self.query_gps_status()
            
            self.get_logger().info("✓ GPS configured for hot start capability")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to configure GPS hot start: {e}")

    def configure_pps_output(self):
        """Configure PPS (Pulse Per Second) output on the GPS module."""
        self.get_logger().info("Configuring PPS output...")
        
        try:
            # Configure PPS output (CFG-TP5)
            # This enables the PPS signal that drives the LED
            pps_cfg = bytearray([0xB5, 0x62,  # Sync chars
                                0x06, 0x31,  # Class: CFG, ID: TP5
                                0x20, 0x00,  # Length: 32 bytes
                                0x00,        # TP index: 0 (first timepulse)
                                0x00,        # Version: 0
                                0x00,        # Reserved
                                0x00,        # Reserved
                                0x00, 0x00, 0x00, 0x00,  # Antenna cable delay: 0ns
                                0x00, 0x00, 0x00, 0x00,  # RF group delay: 0ns
                                0x40, 0x42, 0x0F, 0x00,  # Frequency period: 1Hz (1,000,000µs)
                                0x40, 0x42, 0x0F, 0x00,  # Frequency period lock: 1Hz
                                0xA0, 0x86, 0x01, 0x00,  # Pulse length: 100ms (100,000µs)
                                0xA0, 0x86, 0x01, 0x00,  # Pulse length lock: 100ms
                                0x00, 0x00, 0x00, 0x00,  # User configurable delay: 0ns
                                0x00, 0x00, 0x00, 0x00,  # User configurable delay lock: 0ns
                                0x06, 0x00, 0x00, 0x00,  # Flags: active, locked, polarity high
                                0x00, 0x00, 0x00, 0x00]) # Reserved
            
            # Calculate checksum
            pps_cfg += self._ubx_checksum(pps_cfg[2:])
            
            # Send PPS configuration
            self.serial_port.write(pps_cfg)
            time.sleep(0.1)
            self.get_logger().debug("✓ PPS output configuration sent")
            
            self.get_logger().info("✓ PPS output configured (1Hz, 100ms pulse, active when locked)")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to configure PPS output: {e}")
    
    def query_gps_status(self):
        """Query GPS status to diagnose hardware and fix acquisition issues."""
        self.get_logger().info("Querying GPS status for diagnostics...")
        
        try:
            # Query NAV-STATUS to check fix status
            # UBX-NAV-STATUS: Class 0x01, ID 0x03
            self.get_logger().debug("Querying NAV-STATUS...")
            # Flush buffer before query
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)
            status_result = self._send_ubx(0x01, 0x03, b'', expect_ack=False, timeout=1.5)
            time.sleep(0.5)  # Give GPS time to respond
            
            if self.serial_port.in_waiting > 0:
                status_resp = self.serial_port.read(self.serial_port.in_waiting)
                # Search for NAV-STATUS message in response (might be mixed with other data)
                for i in range(len(status_resp) - 15):
                    if (i + 16 <= len(status_resp) and 
                        status_resp[i:i+2] == b'\xB5\x62' and 
                        status_resp[i+2] == 0x01 and status_resp[i+3] == 0x03):
                        # Parse NAV-STATUS: iTOW(4), gpsFix(1), flags(1), fixStat(1), flags2(1), ttff(4), msss(4)
                        # Skip sync(2) + class(1) + id(1) + length(2) + iTOW(4) = 10 bytes
                        gps_fix = status_resp[i+10]  # gpsFix byte
                        flags = status_resp[i+11]    # flags byte
                        gps_fix_ok = (flags & 0x01) != 0  # gpsFixOk flag
                        diff_soln = (flags & 0x02) != 0   # diffSoln flag
                        
                        fix_names = {0: "No Fix", 1: "Dead Reckoning", 2: "2D Fix", 3: "3D Fix", 4: "GPS+DR", 5: "Time Only"}
                        fix_name = fix_names.get(gps_fix, f"Unknown({gps_fix})")
                        
                        self.get_logger().info(f"  GPS Fix Status: {fix_name} (0x{gps_fix:02X})")
                        self.get_logger().info(f"  GPS Fix OK: {gps_fix_ok}, Differential Solution: {diff_soln}")
                        
                        if gps_fix == 0:
                            self.get_logger().warn("  ⚠ GPS has NO FIX - PPS will not be active")
                            self.get_logger().warn("  ⚠ This may indicate hardware fault if outside with clear sky")
                        break
                else:
                    self.get_logger().warn("  ⚠ NAV-STATUS response not found in received data")
            
            # Query NAV-SAT to check satellites in view
            # UBX-NAV-SAT: Class 0x01, ID 0x35
            self.get_logger().debug("Querying NAV-SAT (satellites in view)...")
            # Flush buffer before query
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)
            sat_result = self._send_ubx(0x01, 0x35, b'', expect_ack=False, timeout=1.5)
            time.sleep(0.5)  # Give GPS time to respond
            
            if self.serial_port.in_waiting > 0:
                sat_resp = self.serial_port.read(self.serial_port.in_waiting)
                # Search for NAV-SAT message in response
                for i in range(len(sat_resp) - 11):
                    if (i + 12 <= len(sat_resp) and 
                        sat_resp[i:i+2] == b'\xB5\x62' and 
                        sat_resp[i+2] == 0x01 and sat_resp[i+3] == 0x35):
                        # Parse NAV-SAT header: iTOW(4), version(1), numSvs(1), reserved1(2), reserved2(4)
                        # Skip sync(2) + class(1) + id(1) + length(2) + iTOW(4) + version(1) = 11 bytes
                        num_svs = sat_resp[i+11]  # numSvs byte
                        self.get_logger().info(f"  Satellites in view: {num_svs}")
                        
                        if num_svs == 0:
                            self.get_logger().error("  ✗ NO SATELLITES IN VIEW - Hardware fault likely!")
                            self.get_logger().error("  ✗ Check: antenna connection, antenna power, GPS module power")
                        elif num_svs < 4:
                            self.get_logger().warn(f"  ⚠ Only {num_svs} satellites in view - may not be enough for fix")
                        else:
                            self.get_logger().info(f"  ✓ {num_svs} satellites visible - should be able to acquire fix")
                        break
                else:
                    self.get_logger().warn("  ⚠ NAV-SAT response not found in received data")
            
            # Query MON-HW to check hardware status
            # UBX-MON-HW: Class 0x0A, ID 0x09
            self.get_logger().debug("Querying MON-HW (hardware status)...")
            # Flush buffer before query
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)
            hw_result = self._send_ubx(0x0A, 0x09, b'', expect_ack=False, timeout=1.5)
            time.sleep(0.5)  # Give GPS time to respond
            
            if self.serial_port.in_waiting > 0:
                hw_resp = self.serial_port.read(self.serial_port.in_waiting)
                # Search for MON-HW message in response
                for i in range(len(hw_resp) - 59):
                    if (i + 60 <= len(hw_resp) and 
                        hw_resp[i:i+2] == b'\xB5\x62' and 
                        hw_resp[i+2] == 0x0A and hw_resp[i+3] == 0x09):
                        # Parse MON-HW: pinSel(4), pinBank(4), pinDir(4), pinVal(4), noisePerMS(2), agcCnt(2), 
                        #              aStatus(1), aPower(1), flags(1), reserved1(1), usedMask(4), VP(25), jamInd(1), reserved2(2), pinIrq(4), pullH(4), pullL(4)
                        # Skip sync(2) + class(1) + id(1) + length(2) + pinSel(4) + pinBank(4) + pinDir(4) + pinVal(4) + noisePerMS(2) + agcCnt(2) = 24 bytes
                        a_status = hw_resp[i+24]  # aStatus byte (antenna status)
                        a_power = hw_resp[i+25]   # aPower byte (antenna power status)
                    
                        ant_status_names = {0: "Init", 1: "Dont know", 2: "OK", 3: "Short", 4: "Open"}
                        ant_status = ant_status_names.get(a_status, f"Unknown({a_status})")
                        ant_power = "ON" if a_power == 1 else "OFF"
                        
                        self.get_logger().info(f"  Antenna Status: {ant_status} (0x{a_status:02X})")
                        self.get_logger().info(f"  Antenna Power: {ant_power}")
                        
                        if a_status == 3:  # Short
                            self.get_logger().error("  ✗ ANTENNA SHORT DETECTED - Hardware fault!")
                        elif a_status == 4:  # Open
                            self.get_logger().error("  ✗ ANTENNA OPEN DETECTED - Check antenna connection!")
                        elif a_status == 2:  # OK
                            self.get_logger().info("  ✓ Antenna status OK")
                        
                        if a_power == 0:
                            self.get_logger().error("  ✗ ANTENNA POWER OFF - Check power supply!")
                        break
                else:
                    self.get_logger().warn("  ⚠ MON-HW response not found in received data")
        
        except Exception as e:
            self.get_logger().warn(f"Failed to query GPS status: {e}")

    def parse_rmc_sentence(self, sentence):
        """
        Parse RMC (Recommended Minimum) sentence for speed, course, and fix status.
        
        RMC sentence format: $GNRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum
        - status: 'A' = valid fix, 'V' = invalid fix
        - speed: Speed over ground in knots
        - course: Course over ground in degrees true
        - Most comprehensive navigation sentence - contains position, speed, course, and fix status
        """
        try:
            # GNRMC format: $GNRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir,mode*checksum
            parts = sentence.split(',')
            if len(parts) >= 9:
                status = parts[2]  # A = valid, V = invalid
                speed_knots = parts[7]  # Speed over ground in knots
                course_true = parts[8]  # Course over ground in degrees true (can be empty when stationary)

                if status == 'A':
                    # We have a valid fix
                    if not self.gps_fix_valid:
                        self.gps_fix_valid = True
                        self.last_fix_time = time.time()  # Record fix acquisition time
                        # Handle immediate logging for fix status change
                        self.handle_fix_status_change(True)
                    
                    # Set NavSat status to indicate we have a fix
                    if self.navsat_status == NavSatStatus.STATUS_NO_FIX:
                        self.navsat_status = NavSatStatus.STATUS_FIX
                        # Update health status when we get a fix (RMC doesn't have satellite count)
                        self.set_healthy("GPS fix valid")
                    
                    # Process speed data if available
                    if speed_knots:
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
                    else:
                        # No speed data available (stationary or no speed info)
                        self.get_logger().debug("RMC: Valid fix, no speed data available")
                    
                    return True
                else:
                    # Invalid status - no GPS fix
                    if self.gps_fix_valid:  # Only update health if fix status changed
                        self.gps_fix_valid = False
                        # Handle immediate logging for fix status change
                        self.handle_fix_status_change(False)
                        self.set_unhealthy("GPS fix lost")
                    self.navsat_status = NavSatStatus.STATUS_NO_FIX
                    # Clear navigation data when fix is lost
                    self.current_sog = None
                    self.current_cog = None
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Error parsing RMC sentence: {e}")
        return False

    def parse_vtg_sentence(self, sentence):
        """
        Parse VTG (Track Made Good and Ground Speed) sentence for course and speed data.
        
        VTG sentence format: $GNVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode*checksum
        - course_true: Course over ground in degrees true
        - course_mag: Course over ground in degrees magnetic
        - speed_knots: Speed over ground in knots
        - speed_kmh: Speed over ground in km/h
        - Alternative to RMC for navigation data, provides course and speed information
        """
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
        """
        Parse GGA (Global Positioning System Fix Data) sentence for position and satellite data.
        
        GGA sentence format: $GNGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,geoid,geoid_unit,dgps_time,dgps_id*checksum
        - quality: 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK, 5=RTK Float, 6=Estimated, 7=Manual, 8=Simulation
        - sats: Number of satellites used in position calculation
        - hdop: Horizontal Dilution of Precision (lower is better)
          HDOP = Horizontal Dilution of Precision (measure of horizontal position accuracy)
        - alt: Altitude above mean sea level
        - Most detailed position data - contains position, altitude, satellite count, and accuracy metrics
        """
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
                    new_sat_count = int(num_sats)
                    if new_sat_count != self.satellites_used:
                        self.satellites_used = new_sat_count
                        # Handle immediate logging and publishing for satellite count changes
                        self.handle_satellite_count_change(new_sat_count)
                        # Update timeout behavior: if satellites now in view, GPS is working
                        if new_sat_count > 0 and self.last_satellites_seen_time is None:
                            self.get_logger().info(
                                f"Satellites detected ({new_sat_count}) - GPS is working, timeout disabled until satellites lost")
                    else:
                        self.satellites_used = new_sat_count

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
                        self.gps_fix_valid = True
                        self.last_fix_time = time.time()  # Record fix acquisition time
                        # Handle immediate logging for fix status change
                        self.handle_fix_status_change(True)
                        self.set_healthy(f"GPS fix valid - {self.satellites_used} satellites")
                    if int(fix_quality) == 1:
                        self.navsat_status = NavSatStatus.STATUS_FIX
                    elif int(fix_quality) == 2:
                        self.navsat_status = NavSatStatus.STATUS_SBAS_FIX
                    elif int(fix_quality) >= 4:
                        self.navsat_status = NavSatStatus.STATUS_GBAS_FIX
                    else:
                        self.navsat_status = NavSatStatus.STATUS_FIX

                    # Update position covariance based on HDOP if available
                    # HDOP = Horizontal Dilution of Precision (lower values = better horizontal accuracy)
                    if hdop:
                        hdop_val = float(hdop)
                        self.current_hdop = hdop_val  # Store HDOP for logging
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
                        self.gps_fix_valid = False
                        # Handle immediate logging for fix status change
                        self.handle_fix_status_change(False)
                        self.set_unhealthy("GPS fix lost")
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

    def get_fix_quality_description(self):
        """Get human-readable description of GPS fix quality."""
        if self.navsat_status == NavSatStatus.STATUS_NO_FIX:
            return "No Fix"
        elif self.navsat_status == NavSatStatus.STATUS_FIX:
            return "GPS Fix"
        elif self.navsat_status == NavSatStatus.STATUS_SBAS_FIX:
            return "SBAS Fix"
        elif self.navsat_status == NavSatStatus.STATUS_GBAS_FIX:
            return "GBAS Fix"
        else:
            return "Unknown Fix"

    def get_position_accuracy_info(self):
        """
        Get position accuracy information from covariance and HDOP.
        
        HDOP = Horizontal Dilution of Precision (lower values indicate better horizontal accuracy)
        DOP = Dilution of Precision (measure of satellite geometry quality)
        """
        accuracy_info = []
        
        # Add HDOP if available
        if self.current_hdop is not None:
            accuracy_info.append(f"HDOP: {self.current_hdop:.2f}")
        
        # Calculate position accuracy from covariance matrix
        if (self.position_covariance_type == NavSatFix.COVARIANCE_TYPE_APPROXIMATED and 
            self.position_covariance[0] > 0 and self.position_covariance[4] > 0):
            # Calculate 95% confidence circle radius (2-sigma)
            # Using the larger of East-East and North-North variances
            max_variance = max(self.position_covariance[0], self.position_covariance[4])
            accuracy_95 = 2.0 * math.sqrt(max_variance)  # 2-sigma for 95% confidence
            accuracy_info.append(f"Accuracy: ±{accuracy_95:.1f}m (95%)")
        
        return ", ".join(accuracy_info) if accuracy_info else "Accuracy: Unknown"

    def handle_satellite_count_change(self, new_count):
        """Handle immediate logging and publishing when satellite count changes."""
        if new_count != self.last_satellite_count:
            # Log the change immediately
            if new_count > self.last_satellite_count:
                self.get_logger().info(f"Satellite count increased: {self.last_satellite_count} → {new_count}")
            elif new_count < self.last_satellite_count:
                self.get_logger().info(f"Satellite count decreased: {self.last_satellite_count} → {new_count}")
            else:
                self.get_logger().info(f"Satellite count changed: {self.last_satellite_count} → {new_count}")
            
            # Update tracking
            self.last_satellite_count = new_count
            
            # Publish immediately (don't wait for periodic timer)
            sat_msg = UInt8()
            sat_msg.data = new_count
            self.pub_satellites.publish(sat_msg)
            
            # Log in debug mode
            if self.debug_mode:
                self.get_logger().debug(f"Immediately published satellite count: {new_count}")

    def handle_fix_status_change(self, new_fix_status):
        """Handle immediate logging when GPS fix status changes (PPS indicator)."""
        if new_fix_status != self.last_fix_status:
            if new_fix_status:
                self.get_logger().info("GPS FIX ACQUIRED - PPS should be active")
            else:
                self.get_logger().info("GPS FIX LOST - PPS should be inactive")
            
            # Update tracking
            self.last_fix_status = new_fix_status
            
            # Log in debug mode
            if self.debug_mode:
                self.get_logger().debug(f"Fix status changed: {not new_fix_status} → {new_fix_status}")

    def periodic_status_logging(self):
        """Handle periodic status logging for normal operation."""
        if self.debug_mode:
            # Skip periodic logging in debug mode (already has detailed logs)
            return

        current_time = time.time()

        if self.gps_fix_valid and self.current_latitude is not None and self.current_longitude is not None:
            # Log GPS fix status immediately on first fix, then every 30 seconds
            if not self.first_fix_logged or current_time - self.last_fix_log_time >= 30.0:
                # Convert SOG from knots to m/s (1 knot = 0.514444 m/s)
                sog_ms = self.current_sog * 0.514444 if self.current_sog is not None else 0.0

                # Format coordinates with proper hemisphere indicators
                lat_str = f"{abs(self.current_latitude):.6f}°{'N' if self.current_latitude >= 0 else 'S'}"
                lon_str = f"{abs(self.current_longitude):.6f}°{'E' if self.current_longitude >= 0 else 'W'}"

                alt_str = f", Alt: {self.current_altitude:.1f}m" if self.current_altitude is not None else ""
                cog_str = f"COG: {self.current_cog:.1f}°" if self.current_cog is not None else "COG: N/A"
                fix_quality_str = self.get_fix_quality_description()
                accuracy_str = self.get_position_accuracy_info()
                self.get_logger().info(
                    f"GPS Fix: {self.satellites_used} satellites ({fix_quality_str}), "
                    f"Position: {lat_str} {lon_str}{alt_str}, "
                    f"{cog_str}, SOG: {sog_ms:.2f} m/s, {accuracy_str}"
                )
                self.last_fix_log_time = current_time
                self.first_fix_logged = True

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
        # Only timeout if no satellites are in view - if satellites are visible, GPS is working
        # and just needs time to acquire a fix (which is normal behavior)
        current_time = time.time()
        has_satellites = self.satellites_used > 0
        
        # Update last_satellites_seen_time when we have satellites
        if has_satellites:
            if self.last_satellites_seen_time is None:
                self.get_logger().info(f"Satellites in view ({self.satellites_used}) - GPS is working, timeout disabled")
            self.last_satellites_seen_time = current_time
        elif self.last_satellites_seen_time is not None:
            # We previously had satellites but lost them - this is OK, they may come back
            self.get_logger().debug(f"Lost satellite view (previously saw {self.last_satellite_count}), waiting for re-acquisition")
        
        # Only timeout if:
        # 1. No valid NMEA data received for timeout period
        # 2. AND no satellites are currently in view (satellites_used == 0)
        # 3. AND no reset was recently performed (GPS may be reinitializing after reset)
        # 4. AND we're not within the reset check window (to allow reset to trigger)
        # This allows GPS to work normally when satellites are visible but fix acquisition takes time
        # and prevents premature exit when GPS is recovering from a reset
        time_since_last_reset = None
        if self.last_reset_time is not None:
            time_since_last_reset = current_time - self.last_reset_time
        
        # Don't exit if reset was recent (within 2 minutes) - GPS may still be reacquiring
        reset_recent = (time_since_last_reset is not None and 
                       time_since_last_reset < 120.0)
        
        # Don't exit if we're close to the reset timeout (within 1 minute) - let reset check run
        time_without_fix = current_time - self.last_fix_time
        reset_pending = (time_without_fix >= (self.no_fix_reset_timeout_seconds - 60.0))
        
        if (current_time - self.last_data_received_time > self.gps_timeout_seconds and 
            not has_satellites and
            not reset_recent and
            not reset_pending):
            self.get_logger().warn(
                f"GPS communication timeout - no data received for {self.gps_timeout_seconds} seconds")
            self.get_logger().warn(
                "GPS device appears to have stopped communicating. Attempting cold start reset...")
            self.set_unhealthy("GPS communication timeout - attempting reset")
            
            # Instead of exiting, try a cold start reset to recover
            if self.gps_cold_start():
                self.get_logger().info("Cold start reset completed. Waiting for GPS to reacquire...")
                self.last_reset_time = current_time
                self.last_data_received_time = current_time  # Reset timeout after reset
            else:
                self.get_logger().error("Cold start reset failed. Will retry on next timeout check.")
                # Don't exit - allow the reset check timer to handle further recovery attempts
        elif (current_time - self.last_data_received_time > self.gps_timeout_seconds and 
              not has_satellites):
            # Log why we're not exiting (for debugging)
            if reset_recent:
                self.get_logger().debug(
                    f"GPS timeout detected but not exiting: reset was recent ({time_since_last_reset:.1f}s ago)")
            elif reset_pending:
                self.get_logger().debug(
                    f"GPS timeout detected but not exiting: reset check pending (no fix for {time_without_fix/60:.1f} min)")

        # The `in_waiting` check is not strictly necessary because `readline()`
        # with a timeout will block until a line is received or the timeout occurs.
        # We just need to ensure the port is open.
        if self.serial_port and self.serial_port.is_open:
            try:
                # Readline() will read until a newline or timeout
                data_bytes = self.serial_port.readline()
                data_str = data_bytes.decode('ascii', errors='ignore').strip()

                if data_str:
                    # Only process valid NMEA sentences (must start with $)
                    # Invalid/corrupted data might be UBX binary, partial reads, or noise
                    # UBX messages are binary and when decoded as ASCII appear as control characters or garbage
                    if not data_str.startswith('$'):
                        # Check if this looks like UBX binary data (control characters, non-printable)
                        if any(ord(c) < 32 and c not in '\r\n\t' for c in data_str):
                            # This is likely UBX binary data - silently discard it
                            self.get_logger().debug(f"GPS UBX binary data (discarded): {repr(data_str[:50])}")
                        else:
                            # Not NMEA and not clearly binary - log for investigation
                            self.get_logger().debug(f"GPS invalid data (ignored): {repr(data_str[:100])}")
                        # Don't reset timeout for invalid data - only valid NMEA resets it
                        return  # Skip processing invalid data
                    
                    # Process valid NMEA sentence
                    self.data_count += 1
                    self.last_data_received_time = current_time  # Reset timeout only on valid NMEA
                    self.get_logger().debug(f"GPS Raw: {data_str}")
                    
                    # Track sentence types seen
                    sentence_type = data_str.split(',')[0]
                    self.sentence_types_seen.add(sentence_type)

                    # Publish raw NMEA data
                    msg = String()
                    msg.data = data_str
                    self.pub_data.publish(msg)

                    # Parse navigation data from specific NMEA sentences (only valid NMEA reaches here)
                    # NMEA sentence prefixes:
                    # - $GNRMC/$GPRMC: Recommended Minimum (RMC) - speed, course, position, fix status
                    # - $GNVTG/$GPVTG: Track Made Good and Ground Speed (VTG) - course, speed, heading
                    # - $GNGGA/$GPGGA: Global Positioning System Fix Data (GGA) - position, altitude, satellites, HDOP
                      # HDOP = Horizontal Dilution of Precision (lower values = better horizontal accuracy)
                    # - $GNGLL/$GPGLL: Geographic Position - Latitude/Longitude (GLL) - position only
                    # - $GNGSA/$GPGSA: GPS DOP and Active Satellites (GSA) - satellite selection, DOP values
                      # DOP = Dilution of Precision (measure of satellite geometry quality)
                    # - $GNGSV/$GPGSV: GPS Satellites in View (GSV) - satellite visibility information
                    
                    if data_str.startswith('$GNRMC') or data_str.startswith('$GPRMC'):
                        # RMC: Most comprehensive navigation sentence - contains position, speed, course, and fix status
                        # Distinguishing feature: Contains 'A' (valid) or 'V' (invalid) fix status
                        if not self.first_rmc_received:
                            self.first_rmc_received = True
                            self.get_logger().info(f"✓ First RMC sentence received: {data_str[:80]}")
                        if self.parse_rmc_sentence(data_str):
                            self.publish_navigation_data()
                    elif data_str.startswith('$GNVTG') or data_str.startswith('$GPVTG'):
                        # VTG: Course and speed information - alternative to RMC for navigation data
                        # Distinguishing feature: Contains course over ground (COG) and speed over ground (SOG)
                        if not self.first_vtg_received:
                            self.first_vtg_received = True
                            self.get_logger().info(f"✓ First VTG sentence received: {data_str[:80]}")
                        if self.parse_vtg_sentence(data_str):
                            self.publish_navigation_data()
                    elif data_str.startswith('$GNGGA') or data_str.startswith('$GPGGA'):
                        # GGA: Position and satellite information - most detailed position data
                        # Distinguishing feature: Contains satellite count, HDOP, and altitude
                          # HDOP = Horizontal Dilution of Precision (lower values = better accuracy)
                        if not self.first_gga_received:
                            self.first_gga_received = True
                            self.get_logger().info(f"✓ First GGA sentence received: {data_str[:80]}")
                        # Always parse to extract satellite count and position data
                        self.parse_gga_sentence(data_str)
                        # Note: NavSatFix published by periodic timer (1 Hz) for consistent rate
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
                        else:
                            self.get_logger().warn("No NMEA sentences received - GPS may not be outputting NMEA data")
                            # Debug: Check if expected sentences have been received
                            expected_status = []
                            if not self.first_gga_received:
                                expected_status.append("GGA")
                            if not self.first_rmc_received:
                                expected_status.append("RMC")
                            if not self.first_vtg_received:
                                expected_status.append("VTG")
                            if expected_status:
                                self.get_logger().warn(
                                    f"No expected NMEA sentences received yet (waiting for: {', '.join(expected_status)})")
                        self.last_sentence_type_log_time = current_time
                    
                    # Log data reception every 10 seconds for debugging (debug mode only)
                    if self.debug_mode:
                        if current_time - self.last_data_log_time >= 10.0:
                            self.get_logger().info(
                                f"GPS data flowing: {self.data_count} messages received so far")
                            if self.gps_fix_valid:
                                fix_quality_str = self.get_fix_quality_description()
                                accuracy_str = self.get_position_accuracy_info()
                                if self.current_sog is not None and self.current_cog is not None:
                                    self.get_logger().info(
                                        f"GPS Fix ({fix_quality_str}): SOG={self.current_sog:.2f} knots, COG={self.current_cog:.1f}°, {accuracy_str}")
                                elif self.current_sog is not None:
                                    self.get_logger().info(
                                        f"GPS Fix ({fix_quality_str}): SOG={self.current_sog:.2f} knots, COG=N/A (stationary), {accuracy_str}")
                                else:
                                    self.get_logger().info(f"GPS Fix ({fix_quality_str}) obtained, waiting for navigation data, {accuracy_str}")
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
                self.set_unhealthy("GPS device not accessible")
                import sys
                sys.exit(1)
            except Exception as e:
                # Log unexpected errors as warnings but don't exit the node
                # This allows the node to recover from transient errors
                self.get_logger().warn(
                    f'Unexpected error in GPS processing: {e}', throttle_duration_sec=5.0)
                # Don't exit - continue operation and try to recover

    def _cleanup_on_exit(self):
        """GPS-specific cleanup on exit"""
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.get_logger().debug("Closing serial connection to GPS")
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    """Main function using ArgoBaseNode standardized approach"""
    parser = ArgoBaseNode.create_standard_parser(
        'GPS Node for ROS2',
        epilog="""
This ROS2 node interfaces with u-blox NEO-M9N GPS module via UART5 (/dev/ttyS5) 
and publishes comprehensive navigation data for both autonomous control and Foxglove 3D mapping.

Published Topics:
- /gps_data (std_msgs/String): Raw NMEA sentences from GPS module
- /gps_sog (std_msgs/Float64): Speed over ground in knots
- /gps_cog (std_msgs/Float64): Course over ground in degrees true (0-360°)
- /gps_velocity (geometry_msgs/Vector3): Velocity vector (x=north, y=east, z=speed)
- /gps_num_satellites (std_msgs/UInt8): Number of satellites used in GPS fix
- /fix (sensor_msgs/NavSatFix): Standard GPS fix for mapping applications
- /gps_health (std_msgs/Bool): Node health status (true=healthy, false=failed)

Services:
- /gps_node/health: Health status service endpoint
        """
    )
    
    try:
        ArgoBaseNode.run_node(GpsNode, args, parser)
    except serial.SerialException as e:
        # Handle GPS-specific serial port errors
        print(f"CRITICAL: Failed to initialize GPS node: {e}")
        print("CRITICAL: GPS device not accessible. Check serial port and permissions.")
        sys.exit(1)


if __name__ == '__main__':
    main()
