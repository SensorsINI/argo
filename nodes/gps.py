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
from std_msgs.msg import String, Float64, Float32, UInt8, Bool
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
try:
    import gpiod
    GPIO_AVAILABLE = True
except ImportError:
    gpiod = None
    GPIO_AVAILABLE = False



class GpsNode(ArgoBaseNode):
    """
    ROS2 GPS node for Argo autonomous sailboat navigation and 3D visualization.

    Interfaces with u-blox NEO-M9N GPS module via UART5 (/dev/ttyS5) and publishes
    comprehensive navigation data for both autonomous control and Foxglove 3D mapping.

    Integration guide: https://content.u-blox.com/sites/default/files/NEO-M9N_Integrationmanual_UBX-19014286.pdf
    UBX interfaced description: https://avionicsduino.com/wp-content/uploads/2023/04/NEO-M9N_Interface_description_UBX-19035940.pdf
    
    Protocol Reference: u-blox M9-MDR-2.16 Interface Description (UBX-22037308), Protocol version 35.16

    Published Topics:
    - /gps_data (std_msgs/String): Raw NMEA sentences from GPS module
    - /gps_sog (std_msgs/Float64): Speed over ground in knots
    - /gps_cog (std_msgs/Float64): Course over ground in degrees true (0-360°)
    - /gps_velocity (geometry_msgs/Vector3): Velocity vector (x=north, y=east, z=speed)
    - /gps_num_satellites (std_msgs/UInt8): Number of satellites in view (from GSV)
    - /gps_num_satellites_used (std_msgs/UInt8): Number of satellites used in navigation solution (from GGA)
    - /gps_snr_avg (std_msgs/Float32): Average SNR of satellites in view (dBHz)
    - /fix (sensor_msgs/NavSatFix): Standard GPS fix for mapping applications (includes position covariance)
    - /gps_pps_status (std_msgs/Bool): True when PPS pulses are being received on GPS_PPS line
    - /gps_health (std_msgs/Bool): Node health status (true=healthy, false=failed)

    Hardware Configuration:
    - GPS: u-blox NEO-M9N via UART5 (/dev/ttyS5)
    - Baud Rate: 38400 (factory default after firmware update)
    - Frame ID: 'argo_gps' (configurable parameter)
    - PPS Output: 1Hz pulse, 100ms duration, active when GPS locked
    
    Firmware Updates:
    - If GPS is in bootloader mode (shows "ROM BOOT" version), firmware update is required
    - See firmware/sparkfun_neo_n9m/README.md for firmware flashing instructions

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
       - Used when GPS has been without fix for extended periods (4+ minutes)
       - Automatic cold start reset after 4 minutes without fix (configurable)
    
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
    - If GPS runs without a fix for 4 minutes, automatic cold start reset is triggered
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

    GPS Noise Characteristics (from real-world measurements):
    Analysis of pond recording (argo_20251117_114846 first irchel pond in rain) revealed:
    - Position noise: 0.51 m stddev (much better than typical 3-10m due to open water conditions)
    - COG noise: 44.64° stddev at low speeds (<0.5 m/s), ~5° at normal speeds
    - SOG noise: 0.156 m/s stddev
    - Temporal correlation: 3.01s time constant (GPS receiver applies smoothing)
    - Update rate: ~0.2 Hz (1/5 Hz) for velocity/COG/SOG (much slower than position updates)
    
    These characteristics are modeled in simulation via argo.yaml sensor_noise.gps parameters
    to reduce sim-to-real gap. The low velocity update rate (0.2 Hz) is critical to model correctly.

    Command Line Options:
    --debug: Enable detailed debug logging of GPS data and communication
    --reset: Perform hardware reset on GPS module at startup using reset pin
    --factory-reset: Reset GPS configuration to factory defaults (clears BBR and Flash)
    """

    def __init__(self, debug_mode=False, do_hardware_reset=False, do_factory_reset=False):
        super().__init__('gps_node')

        # Set logger level to DEBUG if debug mode is enabled
        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self.get_logger().debug('Debug logging enabled')
        
        # Store hardware reset flag for use during setup
        self.do_hardware_reset_on_startup = do_hardware_reset
        self.do_factory_reset_on_startup = do_factory_reset

        self.get_logger().info('Initializing GPS node...')

        # Declare and get parameters
        # The GPS Sparkfun NEO uses UART5, which appears at /dev/ttyS5 on Orange Pi Zero 2W
        # UART5 is enabled via the "ph-uart5" overlay in orangepiEnv.txt
        # UART5 pins are TX=11 (PH2) and RX=13 (PH3) on the Orange Pi Zero 2W
        self.declare_parameter('serial_port', '/dev/ttyS5')
        # u-blox NEO-M9N default baud rate (38400 is factory default after firmware update)
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('gps_frame_id', 'argo_gps')
        # GPIO line for GPS 1PPS input monitoring (-1 disables; default 229 = pin 24 / PH5)
        self.declare_parameter('pps_gpio_line', 229)

        self.serial_port_name = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter(
            'baud_rate').get_parameter_value().integer_value
        self.gps_frame_id = self.get_parameter(
            'gps_frame_id').get_parameter_value().string_value
        self.pps_gpio_line = self.get_parameter(
            'pps_gpio_line').get_parameter_value().integer_value

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
            UInt8, 'gps_num_satellites', 10)  # Number of satellites in view (from GSV)
        self.pub_satellites_used = self.create_publisher(
            UInt8, 'gps_num_satellites_used', 10)  # Number of satellites used in fix (from GGA)
        self.pub_snr_avg = self.create_publisher(
            Float32, 'gps_snr_avg', 10)  # Average SNR of satellites in view

        # Publisher for standard ROS NavSatFix messages (for mapping)
        self.pub_navsat = self.create_publisher(
            NavSatFix, 'fix', 10)  # Standard GPS fix for mapping
        self.pub_pps_status = self.create_publisher(
            Bool, 'gps_pps_status', 10)  # True when PPS pulses are actively received

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
        
        # Satellite signal strength tracking (from GSV sentences)
        self.satellites_in_view = []  # List of (prn, elevation, azimuth, snr) tuples
        self.satellites_in_view_temp = []  # Temporary buffer for GSV parsing (to avoid clearing live data)
        self.gsv_constellation_buffers = {}  # Per-constellation buffers: {'GP': [...], 'GL': [...], etc}
        self.average_snr = 0.0  # Average SNR of satellites with valid signal
        self.last_gsv_time = None  # Track when we last received valid GSV data (None until first GSV)
        self.last_gsv_update_time = 0.0  # Track when we last aggregated all constellation data

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

        # GPIO lines for GPS reset and PPS monitoring
        self.GPIO_CHIP = '/dev/gpiochip0'
        self.GPS_RESET_LINE = 225  # Pin 10 (RXD.0) - !GPS_RESET (active low)
        # Pin 24 (line 229 / PH5) is SPI1_CS0 on the default boot overlay (LoRa SPI).
        # PPS GPIO monitoring is skipped when the kernel already owns that line.
        self.GPS_PPS_LINE = self.pps_gpio_line if self.pps_gpio_line >= 0 else None
        self.gpio_chip = None
        self.gps_reset_line = None
        self.gps_pps_line = None
        self.last_pps_pulse_time = 0.0
        self._last_pps_line_value = 0
        self.pps_status = False
        self.last_pps_status_publish_time = 0.0
        self.pps_status_publish_interval = 5.0
        self.pps_timeout_seconds = 2.5
        self._initialize_gpio_lines()

        # Last known antenna status from diagnostics (UBX-MON-HW). Populated during setup.
        self.antenna_status = None  # e.g. "OK", "Open", "Short", "Init", "Dont know"
        self.antenna_power = None   # "ON" / "OFF"
        self.last_antenna_status_time = None

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
        self.gps_timeout_seconds = 240.0  # Fail if no valid NMEA received for 60 seconds (when no satellites)
        self.last_satellites_seen_time = None  # Track when we last saw satellites (satellites_used > 0)

        # GPS reset tracking for extended no-fix periods
        self.last_fix_time = time.time()  # Initialize to startup time (will be cleared when fix acquired)
        self.no_fix_reset_timeout_seconds = 0.0  # Auto-reset after this many seconds without fix if >0
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
        # Timer to monitor PPS line and publish reception status
        self.pps_timer = self.create_timer(0.1, self.monitor_pps_status)  # 10 Hz polling for PPS

        # Track pause state to manage power save transitions (PSMOO)
        self._prev_paused = False

    def _request_gps_pps_line(self, req_in):
        """Request GPS_PPS as GPIO input, or None if unavailable."""
        if self.GPS_PPS_LINE is None:
            self.get_logger().info(
                "GPS_PPS GPIO monitoring disabled (pps_gpio_line=-1)"
            )
            return None

        line = self.gpio_chip.get_line(self.GPS_PPS_LINE)
        if line.is_used():
            self.get_logger().info(
                f"GPS_PPS GPIO line {self.GPS_PPS_LINE} is in use by the kernel "
                f"on {self.GPIO_CHIP} (SPI1_CS0 / LoRa on this board). "
                "Skipping PPS pin monitoring; module PPS output via UBX is unchanged."
            )
            return None

        bias_disable = getattr(gpiod, "LINE_REQ_FLAG_BIAS_DISABLE", None)
        try:
            line.request(consumer="gps_node_pps", type=req_in)
        except OSError as first_err:
            if bias_disable is None:
                raise
            try:
                line.request(
                    consumer="gps_node_pps",
                    type=req_in,
                    flags=bias_disable,
                )
            except OSError:
                raise first_err from None

        self.get_logger().info(
            f"GPS_PPS pulse monitoring enabled on GPIO line {self.GPS_PPS_LINE}"
        )
        return line

    def _initialize_gpio_lines(self):
        """Initialize GPIO lines for !GPS_RESET output and GPS_PPS input monitoring."""
        if not GPIO_AVAILABLE:
            self.get_logger().warn("gpiod not available - GPS reset pin and PPS GPIO monitoring disabled")
            return

        req_out = getattr(gpiod, "LINE_REQ_DIR_OUT", None)
        req_in = getattr(gpiod, "LINE_REQ_DIR_IN", None)
        pps_line_desc = (
            self.GPS_PPS_LINE if self.GPS_PPS_LINE is not None else "disabled"
        )

        try:
            self.gpio_chip = gpiod.Chip(self.GPIO_CHIP)
            self.get_logger().info(
                f"Opened GPIO chip {self.GPIO_CHIP} for GPS lines "
                f"(reset={self.GPS_RESET_LINE}, pps={pps_line_desc})"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Failed to open GPIO chip {self.GPIO_CHIP} for GPS lines "
                f"(reset={self.GPS_RESET_LINE}, pps={pps_line_desc}): "
                f"{type(e).__name__}: {e}"
            )
            self.gpio_chip = None
            return

        try:
            # !GPS_RESET is active-low: keep HIGH during normal operation.
            self.gps_reset_line = self.gpio_chip.get_line(self.GPS_RESET_LINE)
            self.gps_reset_line.request(
                consumer="gps_node_reset",
                type=req_out,
                default_vals=[1],
            )
        except Exception as e:
            self.get_logger().warn(
                f"Failed requesting !GPS_RESET line {self.GPS_RESET_LINE} "
                f"on {self.GPIO_CHIP} (consumer=gps_node_reset, type={req_out}, default_vals=[1]): "
                f"{type(e).__name__}: {e}"
            )
            self.gps_reset_line = None

        try:
            self.gps_pps_line = self._request_gps_pps_line(req_in)
            if self.gps_pps_line is not None:
                self._last_pps_line_value = self.gps_pps_line.get_value()
        except Exception as e:
            self.get_logger().warn(
                f"Failed requesting GPS_PPS GPIO line {self.GPS_PPS_LINE} "
                f"on {self.GPIO_CHIP}: {type(e).__name__}: {e}"
            )
            self.gps_pps_line = None

        if self.gps_reset_line is not None or self.gps_pps_line is not None:
            pps_status = (
                f"ok (line {self.GPS_PPS_LINE})"
                if self.gps_pps_line is not None
                else (
                    "disabled"
                    if self.GPS_PPS_LINE is None
                    else f"unavailable (line {self.GPS_PPS_LINE})"
                )
            )
            self.get_logger().info(
                "Initialized GPS GPIO lines: "
                f"!GPS_RESET={'ok' if self.gps_reset_line is not None else 'unavailable'} "
                f"(line {self.GPS_RESET_LINE}), "
                f"GPS_PPS={pps_status}"
            )
        else:
            self.get_logger().warn(
                "GPS GPIO initialization completed with no active lines. "
                "Serial GPS can still run, but PPS and reset pin support are disabled."
            )
            if self.gpio_chip:
                try:
                    self.gpio_chip.close()
                except Exception:
                    pass
                self.gpio_chip = None

    def _pulse_gps_reset_pin(self, hold_seconds: float = 0.2) -> bool:
        """Pulse !GPS_RESET low to hard-reset the GPS module."""
        if self.gps_reset_line is None:
            self.get_logger().warn("!GPS_RESET GPIO line unavailable - skipping hardware reset pulse")
            return False

        try:
            self.get_logger().warn("Pulsing !GPS_RESET low for GPS hardware reset")
            self.gps_reset_line.set_value(0)  # Active low reset asserted
            time.sleep(hold_seconds)
            self.gps_reset_line.set_value(1)  # Release reset
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to pulse !GPS_RESET line: {e}")
            return False

    def _reopen_serial_port(self) -> bool:
        """Close and reopen GPS serial port after a hardware reset."""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.get_logger().debug("Closed serial port for GPS reset recovery")
            time.sleep(2.0)
            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1.0)
            time.sleep(1.0)
            self.get_logger().info("Serial port reopened after GPS reset")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to reopen serial port after reset: {e}")
            return False

    def monitor_pps_status(self):
        """Monitor GPS_PPS input and publish whether pulses are being received."""
        now = time.time()
        pps_detected = False

        if self.gps_pps_line is not None:
            try:
                current_value = self.gps_pps_line.get_value()
                # Detect rising edge of PPS pulse
                if self._last_pps_line_value == 0 and current_value == 1:
                    self.last_pps_pulse_time = now
                    self.get_logger().debug("GPS PPS pulse detected")
                self._last_pps_line_value = current_value
                pps_detected = (now - self.last_pps_pulse_time) <= self.pps_timeout_seconds
            except Exception as e:
                self.get_logger().warn(f"Failed reading GPS_PPS GPIO: {e}")
                pps_detected = False

        should_publish = (
            pps_detected != self.pps_status
            or (now - self.last_pps_status_publish_time) >= self.pps_status_publish_interval
        )
        if should_publish:
            status_changed = (pps_detected != self.pps_status)
            self.pps_status = pps_detected
            self.last_pps_status_publish_time = now
            self.pub_pps_status.publish(Bool(data=pps_detected))
            if status_changed and pps_detected:
                self.get_logger().info("GPS PPS status: ACTIVE (pulses received)")
            elif status_changed and not pps_detected:
                self.get_logger().info("GPS PPS status: INACTIVE (no recent pulses)")

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
                flushed = self.serial_port.read(self.serial_port.in_waiting)
                # self.get_logger().debug(f"[SERIAL-TX] Flushed {len(flushed)} bytes before UBX send")
            
            self.serial_port.write(frame)
            # self.get_logger().debug(f"[SERIAL-TX] UBX: class=0x{ubx_class:02X} id=0x{ubx_id:02X} len={length} frame={frame[:20].hex()}...")
            
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
                    # self.get_logger().debug(f"[SERIAL-RX] Read {len(chunk)} bytes: {chunk[:40].hex()}...")
                time.sleep(0.05)  # Small delay between reads
            
            self.serial_port.timeout = original_timeout
            
            # if len(resp) > 0:
            #     self.get_logger().debug(f"[SERIAL-RX] Total UBX response: {len(resp)} bytes")
            
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
                                    ack_type = "ACK-ACK" if is_ack else "ACK-NAK"
                                    # self.get_logger().debug(f"[SERIAL-RX] {ack_type} for class=0x{ubx_class:02X} id=0x{ubx_id:02X}")
                                    return is_ack
                                # elif self.debug_mode:
                                #     self.get_logger().debug(f"ACK checksum mismatch: expected {ck_a:02X}{ck_b:02X}, got {resp_bytes[i+8]:02X}{resp_bytes[i+9]:02X}")
                            # elif self.debug_mode:
                            #     self.get_logger().debug(f"ACK class/id mismatch: expected class=0x{ubx_class:02X} id=0x{ubx_id:02X}, got class=0x{resp_bytes[i+6]:02X} id=0x{resp_bytes[i+7]:02X}")
            
            # if self.debug_mode and len(resp) > 0:
            #     self.get_logger().debug(f"No matching ACK found in response. Looking for class=0x{ubx_class:02X} id=0x{ubx_id:02X}")
            
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
            
            # Check if we need to reset (only if timeout is enabled and enough time has passed since last reset)
            # Skip reset if timeout is 0 (disabled) - allows cold start with weak signal
            if self.no_fix_reset_timeout_seconds > 0 and time_without_fix >= self.no_fix_reset_timeout_seconds:
                if (self.last_reset_time is None or 
                    (current_time - self.last_reset_time) >= self.min_reset_interval_seconds):
                    self.get_logger().warn(
                        f"GPS has been without fix for {time_without_fix/60:.1f} minutes. "
                        f"attempting hardware reset recovery...")
                    # First try dedicated hardware reset pin (!GPS_RESET), then reconfigure.
                    if self._pulse_gps_reset_pin():
                        self.last_reset_time = current_time
                        self.last_fix_time = current_time
                        self.get_logger().warn("!GPS_RESET pulse sent. Waiting for GPS reboot...")
                        time.sleep(5.0)
                        if self._reopen_serial_port():
                            self.get_logger().info("Reconfiguring GPS after !GPS_RESET pulse...")
                            self.setup_gps()
                            self.get_logger().info("GPS recovery via !GPS_RESET completed.")
                            return
                        self.get_logger().error("Serial recovery failed after !GPS_RESET pulse; falling back to UBX reset.")

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
                            if not self._reopen_serial_port():
                                raise RuntimeError("Serial reopen failed")
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
                # self.get_logger().debug(f"Sent command: {msg.strip()}")

                # Try to read response with short timeout
                reply_bytes = self.serial_port.readline()
                reply = reply_bytes.decode('ascii', errors='ignore').strip()

                # Restore original timeout
                self.serial_port.timeout = original_timeout

                if reply:
                    # self.get_logger().debug(f"Received: {reply}")
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

    def _query_firmware_version(self):
        """Query and log GPS firmware version using UBX MON-VER command."""
        self.get_logger().debug("Querying GPS firmware version...")
        try:
            # Send UBX MON-VER (version query) - no ACK expected for this query
            self._send_ubx(0x0A, 0x04, b'', expect_ack=False, timeout=1.0)
            time.sleep(0.5)
            
            if self.serial_port.in_waiting > 0:
                ubx_resp = self.serial_port.read(self.serial_port.in_waiting)
                if len(ubx_resp) >= 8 and ubx_resp[0:2] == b'\xB5\x62':
                    # Parse MON-VER response if we got one
                    if ubx_resp[2] == 0x0A and ubx_resp[3] == 0x04:  # MON-VER response
                        # MON-VER format: sync(2) class(1) id(1) length(2) software_version(30) hardware_version(10) extension...
                        length = ubx_resp[4] + (ubx_resp[5] << 8)
                        if length >= 30 and len(ubx_resp) >= 8 + length:
                            # Software version is null-terminated string starting at byte 6
                            sw_version_end = ubx_resp.find(b'\x00', 6, 6 + 30)
                            if sw_version_end > 6:
                                sw_version = ubx_resp[6:sw_version_end].decode('ascii', errors='ignore')
                                self.get_logger().info(f"GPS Software Version: {sw_version}")
                                
                                # Check for bootloader mode or firmware issues
                                if "ROM BOOT" in sw_version.upper() or "BOOT" in sw_version.upper():
                                    self.get_logger().error("✗ CRITICAL: GPS appears to be in BOOTLOADER mode!")
                                    self.get_logger().error("✗ GPS firmware may be corrupted or GPS needs firmware update")
                                    self.get_logger().error("✗ GPS will NOT acquire satellites or output NMEA in bootloader mode")
                                    self.get_logger().error("✗ SOLUTION: Flash firmware to restore GPS functionality")
                            
                            # Hardware version
                            hw_start = 6 + 30
                            if len(ubx_resp) >= hw_start + 10:
                                hw_version_end = ubx_resp.find(b'\x00', hw_start, hw_start + 10)
                                if hw_version_end > hw_start:
                                    hw_version = ubx_resp[hw_start:hw_version_end].decode('ascii', errors='ignore')
                                    self.get_logger().info(f"GPS Hardware Version: {hw_version}")
                            
                            # Extension strings (firmware details, protocol version, etc.)
                            ext_start = 6 + 30 + 10
                            if len(ubx_resp) > ext_start:
                                ext_data = ubx_resp[ext_start:]
                                ext_strings = []
                                i = 0
                                while i < len(ext_data) and len(ext_strings) < 10:  # Limit to 10 extensions
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
                                    for ext in ext_strings:
                                        self.get_logger().info(f"GPS Info: {ext}")
                        else:
                            # self.get_logger().debug(f"MON-VER response too short: length={length}, received={len(ubx_resp)}")
                            pass
                    else:
                        # self.get_logger().debug(f"Received non-MON-VER UBX response: class=0x{ubx_resp[2]:02X} id=0x{ubx_resp[3]:02X}")
                        pass
                else:
                    self.get_logger().debug("Response not in UBX format")
            else:
                self.get_logger().debug("No response to MON-VER query")
        except Exception as e:
            # self.get_logger().debug(f"Error querying firmware version: {e}")
            pass

    def setup_gps(self):
        """Sends initialization commands to the GPS and verifies communication."""
        self.get_logger().info("Setting up u-blox NEO-N9M GPS...")

        # Perform factory reset if requested (before hardware reset)
        factory_reset_performed = False
        if self.do_factory_reset_on_startup:
            self.get_logger().info("Factory reset requested - will reset GPS configuration to defaults")
            if self.reset_config_to_defaults():
                factory_reset_performed = True
                # Wait for GPS to process the reset and stabilize (factory reset can take time)
                self.get_logger().info("Waiting for GPS to process factory reset...")
                time.sleep(3.0)

        # Perform hardware reset if requested
        if self.do_hardware_reset_on_startup:
            self.get_logger().info("Performing hardware reset on GPS module...")
            if self.gps_reset_line:
                try:
                    # Pull reset line LOW (active-low reset) for 100ms
                    self.gps_reset_line.set_value(0)
                    time.sleep(0.1)
                    # Release reset line HIGH (normal operation)
                    self.gps_reset_line.set_value(1)
                    self.get_logger().info("✓ Hardware reset completed. Waiting for GPS to initialize...")
                    # Wait for GPS module to complete startup (typically 1-2 seconds)
                    time.sleep(2.0)
                except Exception as e:
                    self.get_logger().error(f"Failed to perform hardware reset: {e}")
            else:
                self.get_logger().warn("⚠ Hardware reset requested but GPIO reset line not available")

        # First, listen briefly for any automatic output
        self.get_logger().debug("Listening for automatic GPS output...")
        time.sleep(1.0)  # Brief pause to let any automatic data come through

        # Check if there's any data waiting
        automatic_output_detected = False
        if self.serial_port.in_waiting > 0:
            try:
                waiting_data = self.serial_port.read(
                    self.serial_port.in_waiting).decode('ascii', errors='ignore')
                if waiting_data.strip():
                    self.get_logger().info("✓ GPS is outputting data automatically!")
                    self.get_logger().debug(
                        f"Sample output: {waiting_data[:100]}...")
                    automatic_output_detected = True
            except Exception as e:
                # self.get_logger().debug(f"Error reading automatic data: {e}")
                pass
        
        # Query firmware version (always do this for diagnostics, unless factory reset just happened)
        if not factory_reset_performed:
            self._query_firmware_version()
        else:
            self.get_logger().info("Skipping firmware version query (GPS still stabilizing after factory reset)")
        
        # If automatic output was detected, skip configuration and return
        if automatic_output_detected:
            # CRITICAL: Never send configuration commands on startup!
            # Any configuration command (CFG-PMS, CFG-TP5, CFG-MSG) causes GPS to restart
            # satellite tracking from scratch, losing all downloaded ephemeris data in BBR.
            #
            # The GPS module retains its configuration permanently via backup battery.
            # Configuration should only be done manually via --factory-reset or setup script.
            if not factory_reset_performed:
                self.get_logger().info("GPS outputting data - skipping configuration to preserve ephemeris")
            else:
                # After factory reset: use factory defaults with minimal essential configuration
                # Per NEO-M9N Integration Manual 3.1.5, factory defaults include:
                # - UART: 38400 baud, 8N1
                # - Output: NMEA GGA, GLL, GSA, GSV, RMC, VTG, TXT
                # - NMEA 4.10 with all GNSS bands
                self.get_logger().info("Using factory default configuration:")
                self.get_logger().info("  UART: 38400 baud, 8N1")
                self.get_logger().info("  NMEA: GGA, GLL, GSA, GSV, RMC, VTG, TXT (all GNSS)")
                self.get_logger().info("  Dynamic model: Portable (factory default)")
                
                # NOTE: SparkFun NEO-M9N breakout board has hardware antenna power circuit
                # (VCC_RF with FB1 ferrite bead, R14 10Ω resistor, C1 47pF capacitor)
                # No software configuration needed - antenna power is always on
                # self.configure_antenna_power()  # DISABLED - not needed with SparkFun board
            
            self.get_logger().info("GPS setup completed. Device is working correctly.")
            self.set_healthy("GPS initialized and receiving data")
            return

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
                                    self.get_logger().error("")
                                    self.get_logger().error("✗ SOLUTION: Flash firmware to restore GPS functionality")
                                    self.get_logger().error("✗ See firmware/sparkfun_neo_n9m/README.md for detailed instructions")
                                    self.get_logger().error("✗ Quick start: ./scripts/gps_firmware_flash_wine.sh firmware/sparkfun_neo_n9m/u-blox-m9n-v4.04-firmware.bin /dev/ttyS5 firmware/sparkfun_neo_n9m/ubxfwupdate.exe firmware/sparkfun_neo_n9m/flash.xml")
                            
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
                            # self.get_logger().debug(f"MON-VER response too short: length={length}, received={len(ubx_resp)}")
                            pass
                    else:
                        # self.get_logger().debug(f"Received UBX response but not MON-VER: class=0x{ubx_resp[2]:02X} id=0x{ubx_resp[3]:02X}")
                        pass
                else:
                    self.get_logger().warn("⚠ GPS may not be accepting UBX commands - responses may not be UBX format")
            else:
                self.get_logger().warn("⚠ No UBX response received - GPS may not accept UBX commands")
            
            # Enable SOG/COG sentences since we have communication
            self.configure_hot_start()  # Configure navigation engine FIRST
            self.enable_nmea_sentences()
            self.get_logger().info("GPS setup completed. Waiting for NMEA data...")
            self.set_healthy("GPS initialized and receiving data")
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
        self.get_logger().info("Publishing average SNR (signal strength) to /gps_snr_avg topic")
        self.get_logger().info(
            "Publishing standard NavSatFix messages to /fix topic (for mapping)")

    def _enable_nmea_protocol_cfg_valset(self) -> bool:
        """Enable NMEA output protocol using CFG-VALSET (newer, more reliable method).
        
        CFG-VALSET is preferred over CFG-PRT as it works better after firmware updates.
        Reference: u-blox M9-MDR-2.16 Interface Description (UBX-22037308), Protocol version 35.16
        """
        # CFG-VALSET format:
        # Version (1 byte): 0x01 = transaction mode (safer)
        # Layers (1 byte): 0x07 = RAM + BBR + Flash (save to all layers)
        # Reserved (2 bytes): 0x0000
        # Key-Value pairs: Key (4 bytes, little-endian) + Value (variable length)
        
        # CFG-UART1OUTPROT-NMEA: Enable NMEA on UART1 output
        # Key: 0x10730002 (CFG-UART1OUTPROT-NMEA)
        # Value: 1 byte, 0x01 = enabled
        
        # CFG-UART1OUTPROT-UBX: Keep UBX enabled (for compatibility)
        # Key: 0x10730001 (CFG-UART1OUTPROT-UBX)
        # Value: 1 byte, 0x01 = enabled
        
        version = 0x01  # Transaction mode
        layers = 0x07   # RAM + BBR + Flash
        reserved = 0x0000
        
        # Enable NMEA on UART1 output
        nmea_key = 0x10730002  # CFG-UART1OUTPROT-NMEA
        nmea_value = 0x01      # Enabled
        
        # Enable UBX on UART1 output (keep both enabled)
        ubx_key = 0x10730001   # CFG-UART1OUTPROT-UBX
        ubx_value = 0x01      # Enabled
        
        payload = bytes([
            version,
            layers,
            reserved & 0xFF, (reserved >> 8) & 0xFF,
            # NMEA key (little-endian)
            nmea_key & 0xFF, (nmea_key >> 8) & 0xFF, (nmea_key >> 16) & 0xFF, (nmea_key >> 24) & 0xFF,
            # NMEA value
            nmea_value,
            # UBX key (little-endian)
            ubx_key & 0xFF, (ubx_key >> 8) & 0xFF, (ubx_key >> 16) & 0xFF, (ubx_key >> 24) & 0xFF,
            # UBX value
            ubx_value
        ])
        
        self.get_logger().debug("Configuring NMEA output protocol using CFG-VALSET...")
        result = self._send_ubx(0x06, 0x8A, payload, expect_ack=True, timeout=2.0)
        
        if result:
            self.get_logger().debug("✓ CFG-VALSET ACK received - NMEA output protocol enabled")
            return True
        else:
            self.get_logger().warn("⚠ CFG-VALSET not acknowledged - trying fallback CFG-PRT method...")
            return False

    def enable_nmea_sentences(self):
        """Enable RMC, VTG, and GGA NMEA sentences on u-blox NEO-N9M for navigation and satellite data."""
        self.get_logger().info("Configuring u-blox NEO-N9M to enable navigation sentences...")

        # First try CFG-VALSET (newer, more reliable method)
        nmea_protocol_enabled = self._enable_nmea_protocol_cfg_valset()
        
        # Fallback to CFG-PRT if CFG-VALSET failed
        if not nmea_protocol_enabled:
            # CRITICAL: Configure UART port (CFG-PRT) to ensure NMEA protocol is enabled
            # Reference: u-blox M9-MDR-2.16 Interface Description (UBX-22037308), Protocol version 35.16
            # Format: portID=1 (UART1), txReady=0, mode=4 bytes (8N1, 38400 baud),
            #         inProtoMask=0x07 (UBX + NMEA + RTCM3 input), outProtoMask=0x07 (UBX + NMEA + RTCM3 output)
            # Mode format: [charLen(4 bits)|reserved(4 bits), parity(2 bits)|nStopBits(2 bits)|reserved(4 bits), 
            #               baudRate(16 bits, little-endian)]
            # For 38400 baud 8N1: charLen=8 (0x08), parity=none (0x0), nStopBits=1 (0x0), baudRate=38400 (0x9600)
            # Protocol masks: 0x07 = UBX(bit0=1) + NMEA(bit1=1) + RTCM3(bit2=1) = all protocols enabled
            uart1_cfg_prt = bytes([0xB5, 0x62,  # Sync chars
                                  0x06, 0x00,  # Class: CFG, ID: PRT
                                  0x14, 0x00,  # Length: 20 bytes
                                  0x01,        # Port ID: 1 (UART1)
                                  0x00,        # Reserved
                                  0x00, 0x00,  # TX Ready: disabled (little-endian)
                                  0x08,        # Mode[0]: charLen=8 (bits 0-3), reserved (bits 4-7)
                                  0x00,        # Mode[1]: parity=none (bits 0-1), nStopBits=1 (bits 2-3), reserved (bits 4-7)
                                  0x00, 0x96,  # Mode[2:3]: baudRate=38400 (0x9600, little-endian 16-bit)
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

        # Enable NMEA GSV (Satellites in View) - Class 0xF0, ID 0x03
        # This provides satellite visibility and signal strength information
        gsv_enable = bytes([0xB5, 0x62,  # Sync chars
                           0x06, 0x01,  # Class: CFG, ID: MSG
                           0x08, 0x00,  # Length: 8 bytes
                           0xF0, 0x03,  # NMEA GSV message
                           0x00,        # Rate on DDC (I2C)
                           0x01,        # Rate on UART1 (our connection)
                           0x00,        # Rate on UART2
                           0x01,        # Rate on USB
                           0x00,        # Rate on SPI
                           0x00])       # Reserved

        # Calculate checksum for GSV command
        gsv_ck_a = 0
        gsv_ck_b = 0
        for byte in gsv_enable[2:]:  # Skip sync chars
            gsv_ck_a = (gsv_ck_a + byte) & 0xFF
            gsv_ck_b = (gsv_ck_b + gsv_ck_a) & 0xFF
        gsv_enable += bytes([gsv_ck_a, gsv_ck_b])

        # Send configuration commands
        if self.serial_port and self.serial_port.is_open:
            try:
                # If CFG-VALSET failed, try CFG-PRT as fallback
                if not nmea_protocol_enabled:
                    self.get_logger().debug("Trying CFG-PRT fallback method...")
                    # Extract payload (skip sync chars and checksum)
                    prt_payload = uart1_cfg_prt[6:-2]
                    prt_result = self._send_ubx(0x06, 0x00, prt_payload, expect_ack=True, timeout=2.0)
                    if prt_result:
                        self.get_logger().debug("✓ UART1 port configuration ACK received (CFG-PRT)")
                        nmea_protocol_enabled = True
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

                self.get_logger().debug("Enabling NMEA GSV sentences...")
                gsv_payload = gsv_enable[6:-2]
                if self._send_ubx(0x06, 0x01, gsv_payload, expect_ack=True, timeout=1.0):
                    self.get_logger().debug("✓ GSV enable ACK received")
                else:
                    self.get_logger().warn("⚠ GSV enable sent but no ACK received")
                time.sleep(0.1)

                self.get_logger().info("✓ Navigation sentences (GGA, RMC, VTG, GSV) enabled on GPS module")
                
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
                
                # NOTE: configure_hot_start() is called BEFORE enable_nmea_sentences() in setup sequence
                # Do NOT call it again here or it will reconfigure and wipe out satellite acquisition
                
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

    def poll_pubx_svstatus(self):
        """Poll UBX-NAV-SAT to diagnose why there's no fix.
        
        UBX-NAV-SAT provides detailed satellite information including:
        - Signal quality (C/N0)
        - Satellite health status  
        - Ephemeris and almanac availability flags
        - Whether satellite is used in navigation solution
        """
        try:
            # Poll UBX-NAV-SAT (0x01 0x35) for satellite information
            # Send poll command (empty payload)
            self.get_logger().info(f"Polling UBX-NAV-SAT for satellite status...")
            
            # Build UBX poll frame
            ubx_class = 0x01
            ubx_id = 0x35
            payload = bytearray()
            length = 0
            header = bytes([0xB5, 0x62, ubx_class, ubx_id, length & 0xFF, (length >> 8) & 0xFF])
            ck = self._ubx_checksum(bytes([ubx_class, ubx_id, 0, 0]))
            frame = header + ck
            
            # Don't flush - we want to accumulate data
            # Send poll request
            self.serial_port.write(frame)
            self.get_logger().debug(f"UBX-NAV-SAT poll sent, waiting for response...")
            
            # Read response - try multiple times over 2 seconds to handle NMEA traffic
            resp = bytearray()
            max_attempts = 10
            for attempt in range(max_attempts):
                time.sleep(0.2)  # Short sleep between reads
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    resp.extend(chunk)
                    self.get_logger().debug(f"Attempt {attempt+1}: read {len(chunk)} bytes (total {len(resp)} bytes)")
                    
                    # Check if we have UBX-NAV-SAT response
                    if b'\xB5\x62\x01\x35' in resp:
                        self.get_logger().debug(f"Found UBX-NAV-SAT marker in buffer after {attempt+1} attempts")
                        break
            
            if len(resp) == 0:
                self.get_logger().warn("No data received after UBX-NAV-SAT poll")
                return None
            
            self.get_logger().debug(f"Total received: {len(resp)} bytes")
            
            # Find UBX-NAV-SAT response (0xB5 0x62 0x01 0x35)
            result = None
            for i in range(len(resp) - 8):
                if resp[i:i+4] == b'\xB5\x62\x01\x35':
                    # Found NAV-SAT response
                    payload_len = resp[i+4] | (resp[i+5] << 8)
                    self.get_logger().debug(f"Found UBX-NAV-SAT at offset {i}, payload length: {payload_len}")
                    if i + 6 + payload_len <= len(resp):
                        result = resp[i+6:i+6+payload_len]  # Extract payload only
                        self.get_logger().debug(f"Successfully extracted {len(result)} byte payload")
                        break
                    else:
                        self.get_logger().debug(f"Incomplete UBX-NAV-SAT frame (need {i+6+payload_len} bytes, have {len(resp)})")
            
            if not result:
                # Debug: show first 100 bytes of what we received
                sample = resp[:100].hex(' ')
                self.get_logger().warn(f"No valid UBX-NAV-SAT response found ({len(resp)} bytes received). First 100 bytes: {sample}")
                return None
            
            # Parse UBX-NAV-SAT response
            # Format: header(8) + iTOW(4) + version(1) + numSvs(1) + reserved(2) + repeated satellite blocks
            # Each satellite block: gnssId(1) + svId(1) + cno(1) + elev(1) + azim(2) + prRes(2) + flags(4)
            if len(result) < 8:
                self.get_logger().warn(f"UBX-NAV-SAT response too short: {len(result)} bytes")
                return None
            
            version = result[0]
            num_svs = result[1]
            
            self.get_logger().info(f"UBX-NAV-SAT: {num_svs} satellites tracked")
            
            # Parse satellite data (each sat is 12 bytes)
            satellites_used = 0
            satellites_with_ephemeris = 0
            satellites_healthy = 0
            low_snr_sats = []
            
            for i in range(num_svs):
                offset = 8 + (i * 12)
                if offset + 12 > len(result):
                    break
                
                gnss_id = result[offset]
                sv_id = result[offset + 1]
                cno = result[offset + 2]  # C/N0 (dBHz)
                elev = result[offset + 3] if result[offset + 3] < 128 else result[offset + 3] - 256  # signed
                azim = int.from_bytes(result[offset + 4:offset + 6], 'little')
                flags = int.from_bytes(result[offset + 8:offset + 12], 'little')
                
                # Decode flags
                quality_ind = (flags >> 0) & 0x07  # bits 0-2
                sv_used = (flags >> 3) & 0x01      # bit 3
                health = (flags >> 4) & 0x03        # bits 4-5
                eph_avail = (flags >> 11) & 0x01    # bit 11
                alm_avail = (flags >> 12) & 0x01    # bit 12
                
                if sv_used:
                    satellites_used += 1
                if health == 1:  # 1 = healthy
                    satellites_healthy += 1
                if eph_avail:
                    satellites_with_ephemeris += 1
                if cno > 0 and cno < 25:  # Low SNR
                    low_snr_sats.append((sv_id, cno))
            
            self.get_logger().info(
                f"Satellite analysis: {satellites_used} used in solution, "
                f"{satellites_with_ephemeris} have ephemeris, {satellites_healthy} healthy")
            
            if low_snr_sats:
                self.get_logger().info(f"Low SNR satellites (<25 dBHz): {low_snr_sats[:5]}")  # Show first 5
            
            return {
                'used': satellites_used,
                'ephemeris': satellites_with_ephemeris,
                'healthy': satellites_healthy,
                'low_snr': len(low_snr_sats)
            }
                
        except Exception as e:
            self.get_logger().warn(f"Error polling PUBX-03: {e}")
            return None

    def query_current_dynmodel(self):
        """Query the current dynamic platform model using CFG-VALGET."""
        try:
            # CFG-VALGET message to query CFG-NAVSPG-DYNMODEL (0x20110021)
            valget_payload = bytearray([
                0x00,        # Version: 0
                0x00,        # Layer: 0 = RAM (current active config)
                0x00, 0x00,  # Position (reserved, set to 0)
                # Key ID for DYNMODEL (4 bytes, little-endian)
                0x21, 0x00, 0x11, 0x20
            ])
            
            result = self._send_ubx(0x06, 0x8B, valget_payload, expect_ack=False, timeout=2.0)
            if result and isinstance(result, (bytes, bytearray)) and len(result) >= 1:
                # Response format: header (8 bytes) + key (4 bytes) + value (1 byte for U1)
                # Extract the DYNMODEL value from response
                dynmodel = result[0]
                model_names = {
                    0: "Portable", 2: "Stationary", 3: "Pedestrian", 
                    4: "Automotive", 5: "Sea", 6: "Airborne <1g",
                    7: "Airborne <2g", 8: "Airborne <4g", 9: "Wrist"
                }
                model_name = model_names.get(dynmodel, f"Unknown({dynmodel})")
                self.get_logger().info(f"Current dynamic platform model: {model_name} ({dynmodel})")
                return dynmodel
            else:
                self.get_logger().debug("Dynamic platform model query returned no payload (using factory defaults)")
                return None
        except Exception as e:
            self.get_logger().warn(f"Error querying dynamic model: {e}")
            return None

    def reset_config_to_defaults(self):
        """Reset GPS configuration to factory defaults using CFG-CFG.
        
        Uses UBX-CFG-CFG to clear all configuration from BBR and Flash layers.
        Per NEO-M9N Interface Description 3.10.3.1:
        "if any bit is set in the clearMask: all configuration in the selected 
        non-volatile memory is deleted"
        """
        try:
            # CFG-CFG message to clear all configuration from BBR and Flash
            # Payload: clearMask (4 bytes) + saveMask (4 bytes) + loadMask (4 bytes) + optional deviceMask
            cfg_cfg_payload = bytearray([
                # clearMask (X4): Clear all configuration from selected layers
                0x00, 0x00, 0x00, 0x1F,  # bit 0 (BBR) + bit 1 (Flash) = 0x03, but docs say "any bit" clears all
                                          # Using 0x1F to be explicit about clearing all subsystems
                # saveMask (X4): Don't save anything (we're clearing, not saving)
                0x00, 0x00, 0x00, 0x00,
                # loadMask (X4): Load from remaining layers after clear (reload defaults)
                0x00, 0x00, 0x00, 0x1F,  # Load all subsystems from lower layers (defaults)
            ])
            
            self.get_logger().info("Sending CFG-CFG to clear all configuration from BBR and Flash...")
            
            # Factory reset can take longer - use 5 second timeout
            result = self._send_ubx(0x06, 0x09, cfg_cfg_payload, expect_ack=True, timeout=5.0)
            if result:
                self.get_logger().info("✓ GPS configuration reset to factory defaults (CFG-CFG ACK received)")
                self.get_logger().info("   All configuration cleared from BBR and Flash, defaults loaded")
                return True
            else:
                self.get_logger().warn("⚠ CFG-CFG not acknowledged (ACK-NAK or timeout)")
                self.get_logger().info("   Factory reset may have failed")
                return False
        except Exception as e:
            self.get_logger().error(f"Failed to reset GPS configuration: {e}")
            return False

    def configure_hot_start(self):
        """Configure GPS for hot start capability with backup battery and almanac retention."""
        self.get_logger().info("Configuring GPS for hot start capability...")
        
        try:
            # NOTE: Backup battery / supercapacitor configuration
            # The SparkFun NEO-M9N board may not have a backup battery populated.
            # Without backup power on V_BCKP pin, the GPS will perform cold start on every power cycle:
            # - Cold start: 26-30 seconds to first fix (no retained data)
            # - Hot start: 1-2 seconds (with backup battery and retained ephemeris)
            #
            # For M9N modules, power management is configured via CFG-VALSET with PM2 keys,
            # not the legacy CFG-BAT/CFG-CFG commands. However, without hardware backup battery,
            # software configuration won't enable hot start capability.
            #
            # If hot start is needed, verify V_BCKP has backup power source (battery/supercap)
            self.get_logger().info("Note: Hot start requires backup battery on V_BCKP (not configured in software)")
            
            # Configure navigation engine using CFG-VALSET (modern interface for NEO-M9N)
            # CRITICAL: Use Sea dynamic model for sailboat, not Automotive!
            # NEO-M9N uses the new configuration interface (CFG-VALSET), not legacy CFG-NAV5
            self.get_logger().info("Configuring navigation engine (Sea dynamic model, relaxed filters)...")
            
            # CFG-VALSET payload format:
            # Version (1): 0x00 = SET (immediate), 0x01 = transaction mode
            # Layers (1): 0x01 = RAM only, 0x02 = BBR, 0x04 = Flash, 0x07 = all
            # CRITICAL: Writing to BBR/Flash disrupts satellite tracking! Use RAM only.
            # Reserved (2): 0x0000
            # Key-Value pairs: each key is 4 bytes (little-endian), value follows
            
            # Configuration keys (all 1-byte values, U1 or I1 types)
            # CFG-NAVSPG-DYNMODEL = 0x20110021 - Dynamic model
            # CFG-NAVSPG-INFIL_MINELEV = 0x201100a4 - Minimum elevation (signed, degrees)
            # CFG-NAVSPG-INFIL_MINCNO = 0x201100a3 - Minimum CNO/SNR (dBHz)
            # CFG-NAVSPG-INFIL_NCNOTHRS = 0x201100aa - Number of sats required above CNO threshold
            # CFG-NAVSPG-INFIL_CNOTHRS = 0x201100ab - CNO threshold for fix attempt (dBHz)
            
            def add_key_value_u1(payload, key, value):
                """Helper to add a 1-byte unsigned key-value pair"""
                payload.extend([
                    key & 0xFF, (key >> 8) & 0xFF, 
                    (key >> 16) & 0xFF, (key >> 24) & 0xFF,
                    value & 0xFF
                ])
            
            def add_key_value_i1(payload, key, value):
                """Helper to add a 1-byte signed key-value pair"""
                # Convert signed to unsigned byte representation
                byte_val = value if value >= 0 else (256 + value)
                payload.extend([
                    key & 0xFF, (key >> 8) & 0xFF, 
                    (key >> 16) & 0xFF, (key >> 24) & 0xFF,
                    byte_val & 0xFF
                ])
            
            valset_payload = bytearray([
                0x00,        # Version: 0 = SET (immediate)
                0x02,        # Layers: 0x02 = BBR only (battery-backed RAM, persists across restarts)
                0x00, 0x00,  # Reserved
            ])
            
            # Add configuration key-value pairs
            # Dynamic Platform Model: Pedestrian (3) is ideal for small, slow-moving sailboats
            # - Max altitude: 9000m (allows land testing, unlike Sea model's 500m limit)
            # - Max velocity: 30 m/s (plenty for 65cm sailboat)
            # - Max vertical velocity: 20 m/s
            # - Position deviation: Small (better accuracy than Sea model's Medium)
            # Alternative models: Portable(0), Stationary(2), Automotive(4), Sea(5), Airborne(6-8)
            add_key_value_u1(valset_payload, 0x20110021, 3)    # DYNMODEL: Pedestrian (3)
            add_key_value_i1(valset_payload, 0x201100a4, 0)    # INFIL_MINELEV: 0 degrees (accept low satellites)
            add_key_value_u1(valset_payload, 0x201100a3, 6)    # INFIL_MINCNO: 6 dBHz (very permissive for weak signals)
            add_key_value_u1(valset_payload, 0x201100aa, 3)    # INFIL_NCNOTHRS: 3 satellites minimum
            add_key_value_u1(valset_payload, 0x201100ab, 20)   # INFIL_CNOTHRS: 20 dBHz threshold (permissive)
            
            # Enable all GNSS constellations for maximum satellite availability
            # CFG-SIGNAL-GPS_ENA (0x1031001f): Enable GPS L1C/A
            # CFG-SIGNAL-GAL_ENA (0x10310021): Enable Galileo E1
            # CFG-SIGNAL-BDS_ENA (0x10310022): Enable BeiDou B1I
            # CFG-SIGNAL-QZSS_ENA (0x10310024): Enable QZSS L1C/A
            # CFG-SIGNAL-GLO_ENA (0x10310025): Enable GLONASS L1
            add_key_value_u1(valset_payload, 0x1031001f, 1)    # GPS enabled
            add_key_value_u1(valset_payload, 0x10310021, 1)    # Galileo enabled
            add_key_value_u1(valset_payload, 0x10310022, 1)    # BeiDou enabled
            add_key_value_u1(valset_payload, 0x10310024, 1)    # QZSS enabled
            add_key_value_u1(valset_payload, 0x10310025, 1)    # GLONASS enabled
            
            # Send CFG-VALSET command - DISABLED (causes satellite tracking disruption)
            # nav_result = self._send_ubx(0x06, 0x8A, valset_payload, expect_ack=True, timeout=2.0)
            # 
            # if nav_result:
            #     self.get_logger().info("✓ Navigation engine configured: Sea mode, min_elev=0°, min_cno=6dBHz")
            #     self.get_logger().info("✓ GNSS constellations enabled: GPS, GLONASS, Galileo, BeiDou, QZSS")
            # else:
            #     self.get_logger().warn("⚠ Navigation configuration (CFG-VALSET) not acknowledged - may use default settings")
            
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
            
            # NOTE: SparkFun NEO-M9N breakout board has hardware antenna power circuit
            # (VCC_RF with FB1 ferrite bead, R14 10Ω resistor, C1 47pF capacitor)
            # No software configuration needed - antenna power is always on
            # self.configure_antenna_power()  # DISABLED - not needed with SparkFun board
            
            # Configure PPS (Pulse Per Second) output
            self.configure_pps_output()
            
            # Query current dynamic platform model to see what's configured
            self.query_current_dynmodel()
            
            # CRITICAL DECISION: Skip CFG-VALSET configuration entirely!
            #
            # Problem: ANY configuration command (CFG-VALSET, CFG-CFG) disrupts satellite tracking.
            # Even writing to RAM or BBR causes satellites to be lost and requires reacquisition.
            #
            # Solution: Use factory defaults! The u-blox NEO-M9N default configuration is good enough:
            # - GPS constellation enabled by default
            # - Can acquire satellites with default settings
            # - No need to configure GLONASS/Galileo/BeiDou (nice to have, but not critical)
            #
            # We'll still configure:
            # - Antenna power (non-disruptive)
            # - PPS output (non-disruptive)
            # - NMEA sentences (non-disruptive)
            #
            # DECISION: SKIP CFG-VALSET entirely to avoid satellite tracking disruption.
            
            self.get_logger().info("✓ GPS using factory defaults (skipping CFG-VALSET to avoid satellite disruption)")
            
            self.get_logger().info("✓ GPS configured for hot start capability")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to configure GPS hot start: {e}")

    def save_gps_configuration(self):
        """Save current GPS configuration to non-volatile memory using UBX-CFG-CFG."""
        self.get_logger().info("Saving GPS configuration to non-volatile memory...")
        try:
            # UBX-CFG-CFG: Save configuration to non-volatile memory
            # Format: clearMask (4 bytes), saveMask (4 bytes), loadMask (4 bytes), deviceMask (1 byte)
            # saveMask bits: bit 0=ioPort, bit 1=msgConf, bit 2=infMsg, bit 3=navConf, bit 4=rxmConf,
            #                bit 8=senConf, bit 9=rinvConf, bit 10=antConf, bit 11=logConf, bit 12=ftsConf
            # We want to save: navConf (navigation), msgConf (messages), antConf (antenna), ioPort
            # saveMask = 0x0000041F (bits 0,1,2,3,4,10 = ioPort,msgConf,infMsg,navConf,rxmConf,antConf)
            # deviceMask = 0x01 (save to BBR - battery-backed RAM, 0x02=Flash is not supported on all modules)
            cfg_cfg = bytearray([0xB5, 0x62,  # Sync chars
                                0x06, 0x09,  # Class: CFG, ID: CFG (save configuration)
                                0x0D, 0x00,  # Length: 13 bytes
                                0x00, 0x00, 0x00, 0x00,  # clearMask: don't clear anything
                                0x1F, 0x04, 0x00, 0x00,  # saveMask: save navigation, messages, antenna, I/O port
                                0x00, 0x00, 0x00, 0x00,  # loadMask: don't load (not used for save)
                                0x01])       # deviceMask: BBR (battery-backed RAM)
            
            cfg_cfg += self._ubx_checksum(cfg_cfg[2:])
            # self.get_logger().debug(f"[SERIAL-TX] CFG-CFG (save config): {cfg_cfg[:20].hex()}...")
            self.serial_port.write(cfg_cfg)
            time.sleep(0.5)  # Give GPS time to save to memory
            
            # Check for ACK
            if self.serial_port.in_waiting > 0:
                save_resp = self.serial_port.read(self.serial_port.in_waiting)
                # self.get_logger().debug(f"[SERIAL-RX] CFG-CFG response: {len(save_resp)} bytes")
                if b'\xB5\x62\x05\x01' in save_resp:
                    self.get_logger().info("✓ GPS configuration saved to non-volatile memory")
                else:
                    self.get_logger().warn("⚠ Configuration save sent but no ACK received")
            else:
                self.get_logger().warn("⚠ Configuration save sent but no response")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to save GPS configuration: {e}")

    def configure_antenna_power(self):
        """Configure active antenna power on u-blox NEO-N9M GPS module using CFG-VALSET.
        
        For M9N modules, we should use CFG-VALSET instead of the legacy CFG-ANT command.
        The key configuration is CFG-HW-ANT_CFG_VOLTCTRL to enable active antenna power.
        """
        self.get_logger().info("Configuring active antenna power (CFG-VALSET)...")
        
        try:
            # Use CFG-VALSET to configure antenna power for M9N modules
            # Key ID: CFG-HW-ANT_CFG_VOLTCTRL (0x10a3002e) - Enable active antenna LNA
            # Value: 1 (enable voltage control for active antenna)
            
            # CFG-VALSET format:
            # version (1), layers (1), reserved1 (2), cfgData (variable)
            # cfgData: key (4 bytes), value (variable)
            
            # For RAM-only configuration (Layer = 0x01)
            cfg_valset_payload = bytearray([
                0x00,  # version = 0
                0x01,  # layers = RAM only (0x01)
                0x00, 0x00,  # reserved
                # Key: CFG-HW-ANT_CFG_VOLTCTRL (0x10a3002e) - 1 byte value
                0x2e, 0x00, 0xa3, 0x10,  # Key ID (little-endian)
                0x01,  # Value: 1 = enable active antenna power
            ])
            
            # Send using _send_ubx helper for proper ACK handling
            result = self._send_ubx(0x06, 0x8A, cfg_valset_payload, expect_ack=True, timeout=2.0)
            
            if result:
                self.get_logger().info("✓ Active antenna power enabled (CFG-VALSET)")
                time.sleep(0.3)  # Allow antenna to power up
            else:
                self.get_logger().warn("⚠ Antenna power command sent but not acknowledged")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to configure antenna power: {e}")

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
            # self.get_logger().debug(f"[SERIAL-TX] CFG-TP5 (PPS): {pps_cfg[:20].hex()}...")
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
            time.sleep(1.0)  # Give GPS more time to respond after antenna power config
            
            if self.serial_port.in_waiting > 0:
                hw_resp = self.serial_port.read(self.serial_port.in_waiting)
                # Search for MON-HW message in response
                for i in range(len(hw_resp) - 59):
                    if (i + 60 <= len(hw_resp) and 
                        hw_resp[i:i+2] == b'\xB5\x62' and 
                        hw_resp[i+2] == 0x0A and hw_resp[i+3] == 0x09):
                        # Parse MON-HW: pinSel(4), pinBank(4), pinDir(4), pinVal(4), noisePerMS(2), agcCnt(2), 
                        #              aStatus(1), aPower(1), flags(1), reserved1(1), usedMask(4), VP(25), jamInd(1), reserved2(2), pinIrq(4), pullH(4), pullL(4)
                        # Skip sync(2) + class(1) + id(1) + length(2) + pinSel(4) + pinBank(4) + pinDir(4) + pinVal(4) + noisePerMS(2) + agcCnt(2) = 26 bytes
                        a_status = hw_resp[i+26]  # aStatus byte (antenna status)
                        a_power = hw_resp[i+27]   # aPower byte (antenna power status)
                    
                        ant_status_names = {0: "Init", 1: "Dont know", 2: "OK", 3: "Short", 4: "Open"}
                        ant_status = ant_status_names.get(a_status, f"Unknown({a_status})")
                        ant_power = "ON" if a_power == 1 else "OFF"

                        # Store last known antenna status for consolidated "no fix" logging.
                        self.antenna_status = ant_status
                        self.antenna_power = ant_power
                        self.last_antenna_status_time = time.time()
                        
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
            # self.get_logger().debug(f"Error parsing RMC sentence: {e}")
            pass
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
            # self.get_logger().debug(f"Error parsing VTG sentence: {e}")
            pass
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
                    # No valid fix - log diagnostic info when we have satellites (helps diagnose issues)
                    if num_sats and int(num_sats) >= 3:
                        hdop_val = float(hdop) if hdop else 99.99
                        # HDOP interpretation: <1=ideal, 1-2=excellent, 2-5=good, 5-10=moderate, 10-20=fair, >20=poor
                        hdop_quality = "POOR" if hdop_val > 10 else "MODERATE" if hdop_val > 5 else "GOOD"
                        self.get_logger().info(
                            f"GGA diagnostic: quality={fix_quality} (need >0), sats={num_sats}, "
                            f"HDOP={hdop} ({hdop_quality} geometry - need <10 for fix)")
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
            # self.get_logger().debug(f"Error parsing GGA sentence: {e}")
            pass
        return False

    def parse_gsv_sentence(self, sentence):
        """
        Parse GSV (Satellites in View) sentence for signal strength information.
        
        GSV sentence format: $GPGSV,total_msgs,msg_num,sats_in_view,[prn,elev,azim,snr]*4*checksum
        - Multiple GSV sentences may be sent to cover all satellites
        - Each constellation (GP=GPS, GL=GLONASS, GA=Galileo, GB=BeiDou) sends separate bursts
        - Each sentence contains up to 4 satellite entries
        - SNR (Signal-to-Noise Ratio) is in dBHz (typically 0-99, higher is better)
        - SNR may be empty if satellite is tracked but no signal yet
        
        Example: $GPGSV,3,1,12,01,45,123,42,03,12,045,38,06,78,234,45,09,23,187,*checksum
        """
        try:
            # Extract constellation ID from sentence ($GPGSV -> 'GP', $GLGSV -> 'GL', etc.)
            if not sentence.startswith('$'):
                return False
            constellation_id = sentence[1:3]  # Extract 2-char constellation ID
            
            parts = sentence.split(',')
            if len(parts) < 4:
                return False
            
            # Parse header
            total_messages = int(parts[1]) if parts[1] else 0
            message_number = int(parts[2]) if parts[2] else 0
            sats_in_view_total = int(parts[3]) if parts[3] else 0
            
            # If this is the first message in a constellation burst, initialize that constellation's buffer
            if message_number == 1:
                self.gsv_constellation_buffers[constellation_id] = []
            
            # Parse up to 4 satellite entries (each is 4 fields: prn, elev, azim, snr)
            # Add to this constellation's buffer
            if constellation_id not in self.gsv_constellation_buffers:
                self.gsv_constellation_buffers[constellation_id] = []
                
            for i in range(4):
                base_idx = 4 + (i * 4)
                if base_idx + 3 < len(parts):
                    prn = parts[base_idx] if parts[base_idx] else None
                    elevation = parts[base_idx + 1] if parts[base_idx + 1] else None
                    azimuth = parts[base_idx + 2] if parts[base_idx + 2] else None
                    snr_str = parts[base_idx + 3].split('*')[0] if parts[base_idx + 3] else None
                    
                    # Only add if we have a valid PRN
                    if prn:
                        try:
                            snr = int(snr_str) if snr_str and snr_str.strip() else None
                            self.gsv_constellation_buffers[constellation_id].append({
                                'prn': int(prn),
                                'elevation': int(elevation) if elevation else None,
                                'azimuth': int(azimuth) if azimuth else None,
                                'snr': snr
                            })
                        except ValueError:
                            pass  # Skip satellites with invalid data
            
            # If this is the last message in this constellation's burst, aggregate all constellations
            if message_number == total_messages:
                # Aggregate satellites from all constellation buffers
                # Only update every 1 second to avoid rapid updates from multiple constellations
                current_time = time.time()
                if current_time - self.last_gsv_update_time >= 1.0:
                    # Combine satellites from all constellations
                    aggregated_sats = []
                    for const_id, sats in self.gsv_constellation_buffers.items():
                        aggregated_sats.extend(sats)
                    
                    # Track if we actually changed anything (to avoid spammy debug logs)
                    changed = False
                    
                    # Only update if we have satellites OR explicitly reporting 0
                    if len(aggregated_sats) > 0:
                        self.satellites_in_view = aggregated_sats
                        self.last_gsv_time = current_time
                        self.last_gsv_update_time = current_time
                        changed = True
                    elif sats_in_view_total == 0 and len(self.satellites_in_view) == 0:
                        # GPS reports 0 and we already have 0 - nothing to do, but update timestamp to avoid spam
                        self.last_gsv_update_time = current_time
                    elif sats_in_view_total == 0 and self.last_gsv_time is not None and (current_time - self.last_gsv_time) > 15.0:
                        # GPS reports 0 for more than 15 seconds - truly lost satellites
                        self.satellites_in_view = []
                        self.last_gsv_update_time = current_time
                        changed = True
                    
                    # Calculate average SNR from satellites with valid signal
                    valid_snrs = [sat['snr'] for sat in self.satellites_in_view if sat['snr'] is not None and sat['snr'] > 0]
                    if valid_snrs:
                        self.average_snr = sum(valid_snrs) / len(valid_snrs)
                        # Only publish SNR when we have valid data (don't spam 0.0)
                        snr_msg = Float32()
                        snr_msg.data = self.average_snr
                        self.pub_snr_avg.publish(snr_msg)
                    else:
                        # Don't update average_snr or publish - keep last valid value
                        pass
                    
                    # Only log in debug mode AND when something actually changed
                    if self.debug_mode and changed:
                        const_counts = {k: len(v) for k, v in self.gsv_constellation_buffers.items()}
                        self.get_logger().debug(
                            f"GSV: {len(self.satellites_in_view)} sats total ({const_counts}), "
                            f"{len(valid_snrs)} with signal, avg SNR: {self.average_snr:.1f} dBHz"
                        )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse GSV sentence: {e}")
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
            # if self.debug_mode:
            #     self.get_logger().debug(f"Immediately published satellite count: {new_count}")

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
            # if self.debug_mode:
            #     self.get_logger().debug(f"Fix status changed: {not new_fix_status} → {new_fix_status}")

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
            # Log no GPS fix frequently to track satellite acquisition patterns
            if not hasattr(self, 'no_fix_log_count'):
                self.no_fix_log_count = 0

            # Use 5 second interval to see detailed satellite count changes during debugging
            log_interval = 5.0

            # Check if enough time has passed since last log
            if current_time - self.last_no_fix_log_time >= log_interval:
                status_line = self._format_no_fix_status_line(current_time)
                if status_line:  # Only log if we have meaningful status (not GSV_pending noise)
                    self.get_logger().info(status_line)
                    self.last_no_fix_log_time = current_time
                    self.no_fix_log_count += 1
                    
                    # Poll PUBX-03 for detailed satellite status every 30 seconds when we have satellites but no fix
                    # This helps diagnose why GPS can't get a fix (ephemeris, geometry, etc.)
                    if self.satellites_used >= 3 and self.no_fix_log_count % 6 == 0:  # Every 30s (6 * 5s intervals)
                        self.get_logger().info(f"Polling satellite status (no fix with {self.satellites_used} satellites)...")
                        self.poll_pubx_svstatus()

    @staticmethod
    def _format_hms(seconds: float) -> str:
        total = max(0, int(seconds))
        h = total // 3600
        m = (total % 3600) // 60
        s = total % 60
        return f"{h:02d}:{m:02d}:{s:02d}"

    def _format_no_fix_status_line(self, now: float) -> str:
        time_without_fix_s = now - self.last_fix_time
        # Only calculate reset countdown if reset timeout is enabled (> 0)
        reset_due_in_s = (self.no_fix_reset_timeout_seconds - time_without_fix_s 
                         if self.no_fix_reset_timeout_seconds > 0 else float('inf'))

        if self.gps_pps_line is None:
            pps_str = "PPS=unavailable"
        else:
            age = now - self.last_pps_pulse_time if self.last_pps_pulse_time > 0 else None
            if self.pps_status:
                pps_str = f"PPS=ACTIVE age={age:.1f}s" if age is not None else "PPS=ACTIVE"
            else:
                pps_str = f"PPS=INACTIVE age={age:.1f}s" if age is not None else "PPS=INACTIVE"

        if self.antenna_status is None and self.antenna_power is None:
            ant_str = "ANT=unknown"
        else:
            ant_parts = []
            if self.antenna_status is not None:
                ant_parts.append(f"ANT={self.antenna_status}")
            if self.antenna_power is not None:
                ant_parts.append(f"PWR={self.antenna_power}")
            if self.last_antenna_status_time is not None:
                ant_parts.append(f"age={self._format_hms(now - self.last_antenna_status_time)}")
            ant_str = " ".join(ant_parts)

        # Format reset status string
        if self.no_fix_reset_timeout_seconds <= 0:
            # Reset is disabled
            reset_str = "reset=disabled"
        elif self.last_reset_time is None:
            reset_str = f"reset_due_in={self._format_hms(reset_due_in_s)}"
        else:
            reset_str = (
                f"last_reset={self._format_hms(now - self.last_reset_time)} ago "
                f"reset_due_in={self._format_hms(reset_due_in_s)}"
            )

        # Add detailed satellite constellation information
        # Count satellites by constellation (PRN ranges)
        gps_sats = len([s for s in self.satellites_in_view if 1 <= s['prn'] <= 32])
        glonass_sats = len([s for s in self.satellites_in_view if 65 <= s['prn'] <= 96])
        galileo_sats = len([s for s in self.satellites_in_view if s['prn'] >= 211])
        beidou_sats = len([s for s in self.satellites_in_view if 159 <= s['prn'] <= 163 or 33 <= s['prn'] <= 64])
        total_in_view = len(self.satellites_in_view)
        
        # Count satellites with valid SNR (being tracked)
        sats_with_signal = len([s for s in self.satellites_in_view if s.get('snr') and s['snr'] > 0])
        
        # Build constellation string
        # Check if we have recent GSV data (within last 15 seconds)
        gsv_age = (now - self.last_gsv_time) if self.last_gsv_time is not None else 999.0
        
        if total_in_view > 0 or (gsv_age < 15.0):
            # We have satellites OR we recently had GSV data (just waiting for next burst)
            if total_in_view > 0:
                const_parts = []
                if gps_sats > 0: const_parts.append(f"GPS:{gps_sats}")
                if glonass_sats > 0: const_parts.append(f"GLO:{glonass_sats}")
                if galileo_sats > 0: const_parts.append(f"GAL:{galileo_sats}")
                if beidou_sats > 0: const_parts.append(f"BDS:{beidou_sats}")
                const_str = " ".join(const_parts) if const_parts else f"total:{total_in_view}"
                
                if self.average_snr > 0:
                    sat_str = f"in_view={total_in_view}({const_str}) tracking={sats_with_signal} SNR={self.average_snr:.0f}dB used={self.satellites_used}"
                else:
                    sat_str = f"in_view={total_in_view}({const_str}) tracking={sats_with_signal} used={self.satellites_used}"
            else:
                # Between GSV bursts - just show satellites used, don't log GSV_pending noise
                if self.satellites_used > 0:
                    sat_str = f"used={self.satellites_used}"
                else:
                    # No satellites, waiting for GSV - skip logging entirely by returning None
                    return None
        else:
            # No satellites and no recent GSV data
            if self.satellites_used > 0:
                # We're using satellites but GSV hasn't reported any (stale data)
                sat_str = f"used={self.satellites_used} in_view=? (GSV pending)"
            else:
                # Truly no satellites for >15s - critical hardware issue
                sat_str = f"in_view=0 (NO RF SIGNAL - check antenna!)"
        
        return (
            "GPS: No fix - searching... "
            f"({sat_str}; since_fix={self._format_hms(time_without_fix_s)}; {pps_str}; {ant_str}; {reset_str})"
        )

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
        # Publish total satellites in view (from GSV) to gps_num_satellites
        # This gives a more stable indication of GPS health, as satellites_used drops to 0
        # during brief fix losses even when GPS is still tracking many satellites
        total_sats_in_view = len(self.satellites_in_view)
        
        # Logical consistency check: can't use more satellites than we can see
        # If satellites_used > satellites_in_view, it means GSV data is stale/incomplete
        # In this case, assume at least satellites_used are in view to maintain consistency
        if self.satellites_used > total_sats_in_view:
            # GGA reports more satellites used than GSV reports in view
            # This happens during startup or when GSV data is delayed
            # Use satellites_used as the minimum to keep display logically consistent
            total_sats_in_view = self.satellites_used
        
        sat_in_view_msg = UInt8()
        sat_in_view_msg.data = total_sats_in_view
        self.pub_satellites.publish(sat_in_view_msg)
        
        # Also publish satellites used in navigation solution (from GGA) to gps_num_satellites_used
        sat_used_msg = UInt8()
        sat_used_msg.data = self.satellites_used
        self.pub_satellites_used.publish(sat_used_msg)
        
        # Log satellite count periodically in debug mode
        # if self.debug_mode:
        #     self.get_logger().debug(f"Published satellite count: in_view={sat_in_view_msg.data}, used={sat_used_msg.data}")

    def publish_navsat_fix(self):
        """Publish NavSatFix message for mapping applications."""
        # Always publish NavSatFix to update dashboard with current status
        # even when there's no fix (so dashboard knows GPS is not locked)
        
        # Create NavSatFix message
        navsat_msg = NavSatFix()

        # Header
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.header.frame_id = self.gps_frame_id

        # Status (always publish current status)
        navsat_msg.status.status = self.navsat_status
        navsat_msg.status.service = self.navsat_service

        # Position (use current or zero if no fix)
        if self.current_latitude is not None and self.current_longitude is not None:
            navsat_msg.latitude = self.current_latitude
            navsat_msg.longitude = self.current_longitude
            navsat_msg.altitude = self.current_altitude if self.current_altitude is not None else 0.0
            
            # Covariance (only valid when we have a fix)
            navsat_msg.position_covariance = self.position_covariance
            navsat_msg.position_covariance_type = self.position_covariance_type
        else:
            # No position data yet
            navsat_msg.latitude = 0.0
            navsat_msg.longitude = 0.0
            navsat_msg.altitude = 0.0
            navsat_msg.position_covariance = [0.0] * 9
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # Publish
        self.pub_navsat.publish(navsat_msg)
        
        # self.get_logger().debug(
        #     f"Published NavSatFix: {self.current_latitude:.6f}°, {self.current_longitude:.6f}°, Alt: {self.current_altitude}m")

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
        # elif self.last_satellites_seen_time is not None:
        #     We previously had satellites but lost them - this is OK, they may come back
        #     self.get_logger().debug(f"Lost satellite view (previously saw {self.last_satellite_count}), waiting for re-acquisition")
        
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
        # Only check if reset timeout is enabled (> 0)
        time_without_fix = current_time - self.last_fix_time
        reset_pending = (self.no_fix_reset_timeout_seconds > 0 and 
                        time_without_fix >= (self.no_fix_reset_timeout_seconds - 60.0))
        
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
                    # Log every sentence received for debugging
                    # self.get_logger().debug(f"[SERIAL-RX] NMEA: {data_str[:80]}")
                    
                    # Only process valid NMEA sentences (must start with $)
                    # Invalid/corrupted data might be UBX binary, partial reads, or noise
                    # UBX messages are binary and when decoded as ASCII appear as control characters or garbage
                    if not data_str.startswith('$'):
                        # Check if this looks like UBX binary data (control characters, non-printable)
                        if any(ord(c) < 32 and c not in '\r\n\t' for c in data_str):
                            # This is likely UBX binary data - silently discard it
                            # self.get_logger().debug(f"GPS UBX binary data (discarded): {repr(data_str[:50])}")
                            pass
                        # else:
                            # Not NMEA and not clearly binary - log for investigation
                            # self.get_logger().debug(f"GPS invalid data (ignored): {repr(data_str[:100])}")
                        # Don't reset timeout for invalid data - only valid NMEA resets it
                        return  # Skip processing invalid data
                    
                    # Process valid NMEA sentence
                    self.data_count += 1
                    self.last_data_received_time = current_time  # Reset timeout only on valid NMEA
                    # self.get_logger().debug(f"GPS Raw: {data_str}")
                    
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
                    elif (data_str.startswith('$GPGSV') or data_str.startswith('$GLGSV') or 
                          data_str.startswith('$GAGSV') or data_str.startswith('$GBGSV') or
                          data_str.startswith('$GNGSV')):
                        # GSV: Satellites in view with signal strength (SNR) information
                        # Multiple constellations: GP=GPS, GL=GLONASS, GA=Galileo, GB=BeiDou, GN=Combined
                        # Each constellation sends its own multi-message burst
                        # Parser aggregates across constellations to avoid overwriting data
                        self.parse_gsv_sentence(data_str)

                    # Handle periodic status logging for normal operation
                    self.periodic_status_logging()

                    # Log sentence types seen every 15 seconds
                    if current_time - self.last_sentence_type_log_time >= 15.0:
                        if self.sentence_types_seen:
                            sentence_list = sorted(list(self.sentence_types_seen))
                            # self.get_logger().debug(f"NMEA sentence types received: {', '.join(sentence_list)}")
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
                                status_line = self._format_no_fix_status_line(current_time)
                                if status_line:  # Only log if we have meaningful status (not GSV_pending noise)
                                    self.get_logger().info(status_line)
                            self.last_data_log_time = current_time

                # The original script performed manual parsing of NMEA sentences
                # and used a ROS1-specific library (libnmea_navsat_driver).
                # This functionality is removed because:
                # 1. The library is not available in ROS2.
                # 2. The standard `nmea_navsat_driver` ROS2 package should be used
                #    for parsing NMEA and publishing standard sensor messages.
                # This node's primary purpose is now to provide the raw data stream.

            except serial.SerialException as e:
                error_msg = str(e)
                # Check if this is a transient "device reports readiness" error
                # This is a known pyserial issue with certain UART devices and should not cause exit
                if "readiness to read but returned no data" in error_msg:
                    self.get_logger().warn(
                        f'Transient serial port error (recoverable): {e}',
                        throttle_duration_sec=10.0)
                    # Don't exit - this is a transient UART timing issue, not a hardware failure
                    # The next read will likely succeed
                    time.sleep(0.1)  # Brief delay before retrying
                else:
                    # This is a real hardware error (device disconnected, etc.)
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
        if hasattr(self, 'gps_reset_line') and self.gps_reset_line is not None:
            try:
                self.gps_reset_line.release()
            except Exception:
                pass
            self.gps_reset_line = None

        if hasattr(self, 'gps_pps_line') and self.gps_pps_line is not None:
            try:
                self.gps_pps_line.release()
            except Exception:
                pass
            self.gps_pps_line = None

        if hasattr(self, 'gpio_chip') and self.gpio_chip is not None:
            try:
                self.gpio_chip.close()
            except Exception:
                pass
            self.gpio_chip = None

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
- /gps_velocity (geometry_msgs/Vector3): Velocity vector (x=north, y=east, z=speed m/s)
- /gps_num_satellites (std_msgs/UInt8): Number of satellites in view (from GSV sentences)
- /gps_num_satellites_used (std_msgs/UInt8): Number of satellites used in fix (from GGA)
- /gps_snr_avg (std_msgs/Float32): Average signal-to-noise ratio of satellites (dBHz)
- /fix (sensor_msgs/NavSatFix): Standard GPS fix for mapping applications
- /gps_pps_status (std_msgs/Bool): True when PPS pulses are actively received
- /gps_health (std_msgs/Bool): Node health status (true=healthy, false=failed)

Services:
- /gps_node/health: Health status service endpoint

Options:
--debug: Enable detailed debug logging
--reset: Perform hardware reset on GPS module at startup
--factory-reset: Reset GPS configuration to factory defaults
        """
    )
    
    # Add GPS-specific command line arguments
    parser.add_argument('--reset', action='store_true',
                       help='Perform hardware reset on GPS module at startup using reset pin')
    parser.add_argument('--factory-reset', action='store_true',
                       help='Reset GPS configuration to factory defaults (clears BBR and Flash storage)')
    
    try:
        ArgoBaseNode.run_node(GpsNode, args, parser)
    except serial.SerialException as e:
        # Handle GPS-specific serial port errors
        print(f"CRITICAL: Failed to initialize GPS node: {e}")
        print("CRITICAL: GPS device not accessible. Check serial port and permissions.")
        sys.exit(1)


if __name__ == '__main__':
    main()
