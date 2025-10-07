#!/usr/bin/env python
# ROS2 Anemometer Node - Wind Speed and Direction Sensor

# Reading three Sensirion SDP3x differential pressure sensors
# Dev by JJ Slabbert, modified by tobi
# This ROS2 node communicates with three differential pressure sensors over I2C
# to determine wind speed and direction using directional wind meter principles.
#
# Hardware Setup:
# - Uses I2C bus 0 (not bus 1)
# - Three sensors at addresses:
#   * I2C_CTR (0x21): Center sensor (0° - front/back)
#   * I2C_CW (0x22): Clockwise 120° from front (looking down on mast)
#   * I2C_CCW (0x23): Counter-clockwise 120° from front (240° - looking down on mast)
# - Run "sudo i2cdetect -y 0" to verify sensor connections
# - Sensors show as addresses 21, 22, 23 in hex
#
# Algorithm based on Sensirion's directional wind meter application:
# https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/
#
# Features:
# - Automatic sensor reconnection on I2C errors with health-based frequency adaptation
# - Optional visual debug mode with ASCII wind vector display and differential pressure bar charts
# - Temperature compensation and averaging
# - Configurable logging levels
# - Multi-sample averaging for noise reduction (15 samples per publish cycle)
# - Effective time constant ~100ms (sensor IIR ~10ms + application averaging)
# - Low CPU overhead - single timer with multiple sensor reads per cycle
# - Controlled publishing rate to reduce data transmission
# - Transient I2C failure recovery: switches to 1Hz retry mode, recovers to 3Hz normal operation
#
# Command line options:
# --debug: Enable debug logging of sensor values
# --debug_visually: Show real-time ASCII visualization of wind vector
#
# Calibration and Verification:
# The anemometer system includes calibration tools to verify correct operation:
#
# 1. Data Collection Script (scripts/anem_calibration_data_collection.py):
#    - Systematically collects differential pressure data at known wind angles
#    - Prompts user to position wind source at angles from -180° to +180° in 30° steps
#    - Collects data for 7 seconds at each angle with 10Hz sampling rate
#    - Calculates mean and standard deviation for each sensor
#    - Saves results to anem-measurement-DATESTAMP.csv
#    - Usage: python3 scripts/anem_calibration_data_collection.py
#
# 2. Data Visualization Script (scripts/plot_anem_calibration_data.py):
#    - Creates comprehensive plots of sensor response patterns
#    - Generates polar plot showing differential pressure vs wind angle
#    - Creates linear plot of pressure response curves
#    - Calculates and displays wind direction calibration accuracy
#    - Shows correlation coefficient and RMSE for fitted vs measured angles
#    - Usage: python3 scripts/plot_anem_calibration_data.py [data_file.csv]
#
# 3. Calibration Verification Process:
#    - Run data collection script with controlled wind source at ~7.4 m/s
#    - Position wind source at each angle as prompted
#    - Review generated plots for sinusoidal sensor response patterns
#    - Check wind direction calibration shows >0.95 correlation
#    - Verify RMSE is <10° for wind direction accuracy
#    - Look for consistent phase relationships between all three sensors
#
# Topics Published:
# /anem_speed_angle_temp (geometry_msgs/Vector3):
#   x: wind speed in m/s
#   y: wind angle in degrees CW from front of boat (looking down)
#   z: average temperature in celsius
# /anem_diffpressure (geometry_msgs/Vector3):
#   x: differential pressure from I2C_CTR (0x21, 0°) in Pascals
#   y: differential pressure from I2C_CW (0x22, 120°) in Pascals
#   z: differential pressure from I2C_CCW (0x23, 240°) in Pascals
# /anem_health (std_msgs/Bool):
#   data: true when node is healthy, false when unhealthy or shutting down
#   Published only on state changes, startup, and shutdown

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from toggle_pause_service import TogglePauseService
import smbus
import time
import numpy as np
import argparse
from rclpy.logging import LoggingSeverity
import math
import sys
from collections import deque

# I2C sensor addresses
I2C_CTR = 0x21  # Center sensor (0° - front/back)
I2C_CW = 0x22   # Clockwise 120° from front (looking down on mast)
# Counter-clockwise 120° from front (240° - looking down on mast)
I2C_CCW = 0x23

# Publishing configuration constants
PUBLISHING_RATE = 3.0  # Hz - final publishing rate
SAMPLES_PER_PUBLISH = 15  # Number of sensor reads to average per publish cycle

# CRC error logging throttling
CRC_ERROR_LOG_THROTTLE_S = 1.0  # Maximum CRC error logging frequency in seconds

# Visual debugging configuration constants
# knots - full scale wind speed for visual display
VISUAL_FULLSCALE_SPEED_KNOTS = 8.0
VISUAL_DISPLAY_WIDTH = 60  # characters - width of visual display
VISUAL_DISPLAY_HEIGHT = 40  # characters - height of visual display
# Pascals - full scale differential pressure for bar charts
VISUAL_DP_FULLSCALE_PA = 50.0

# from https://stackoverflow.com/questions/49906101/byte-array-to-int-in-python-2-x-using-standard-libraries
# This function is compatible with Python 3.


def int_from_bytes(b):
    '''Convert big-endian signed integer bytearray to int

    int_from_bytes(b) == int.from_bytes(b, 'big', signed=True)'''
    if not b:  # special-case 0 to avoid b[0] raising
        return 0
    n = b[0] & 0x7f  # skip sign bit
    for by in b[1:]:
        n = n * 256 + by
    if b[0] & 0x80:  # if sign bit is set, 2's complement
        bits = 8*len(b)
        offset = 2**(bits-1)
        return n - offset
    else:
        return n


def calculate_crc8(data_bytes):
    '''Calculate CRC-8 checksum for SDP3x sensor data

    Algorithm from SDP3x datasheet:
    - Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
    - Initialization: 0xFF
    - No reflection of input or output
    - Final XOR: 0x00
    '''
    crc = 0xFF  # Initialization value
    polynomial = 0x31  # CRC-8 polynomial

    for byte in data_bytes:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc = crc << 1
            crc &= 0xFF  # Keep only 8 bits

    return crc


def verify_crc(data_bytes, received_crc):
    '''Verify CRC checksum for sensor data

    Args:
        data_bytes: List of data bytes to verify
        received_crc: CRC byte received from sensor

    Returns:
        bool: True if CRC is valid, False otherwise
    '''
    calculated_crc = calculate_crc8(data_bytes)
    return calculated_crc == received_crc


# following from sensirion https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/

def calculate_angle_deg(dp_ctr, dp_cw, dp_ccw):
    """Calculate wind angle from differential pressure sensors using 3-sensor wind vane algorithm

    This function implements a robust wind direction calculation using three differential pressure
    sensors arranged in a 120° pattern. The algorithm has been calibrated and validated through
    systematic data collection and shows excellent correlation with measured wind angles.

    Algorithm Overview:
    1. Three sensors are positioned at 0° (CTR), 120° (CW), and 240° (CCW) from the boat's front
    2. Each sensor contributes to both X and Y components based on its angular position
    3. Wind direction is calculated using atan2() on the resulting vector components
    4. The Y-component is negated to correct for the coordinate system orientation

    Mathematical Model:
    - Each sensor contributes: x = dp * cos(angle), y = dp * sin(angle)
    - CTR (0°):   x = dp_ctr * cos(0°),   y = dp_ctr * sin(0°)
    - CW (120°):  x = dp_cw * cos(120°),  y = dp_cw * sin(120°)
    - CCW (240°): x = dp_ccw * cos(240°), y = dp_ccw * sin(240°)
    - Final direction: atan2(-y_total, x_total) where y is negated for correct orientation

    Calibration Results:
    - Correlation coefficient: >0.95 with measured wind angles
    - RMSE: <10° average error across all wind directions
    - Validated with systematic data collection at 30° intervals from -180° to +180°
    - Sign correction applied to Y-component for proper coordinate system alignment

    Args:
        dp_ctr: Differential pressure from center sensor (0x21) in Pascals
        dp_cw: Differential pressure from CW 120° sensor (0x22) in Pascals
        dp_ccw: Differential pressure from CCW 240° sensor (0x23) in Pascals

    Returns:
        Wind angle in degrees CW from front of boat (looking down)
        Range: 0° to 360° (0° = wind from front, 90° = wind from starboard, etc.)
    """
    try:
        # Convert sensor positions to radians
        # CTR is at 0°, CW at 120°, CCW at 240° (120° spacing)
        ctr_angle = 0 * math.pi / 180
        cw_angle = 120 * math.pi / 180
        ccw_angle = 240 * math.pi / 180

        # Calculate x and y components using the three sensors
        # Each sensor contributes to both x and y based on its position
        x_component = (dp_ctr * math.cos(ctr_angle) +
                       dp_cw * math.cos(cw_angle) +
                       dp_ccw * math.cos(ccw_angle))

        y_component = -(dp_ctr * math.sin(ctr_angle) +
                        dp_cw * math.sin(cw_angle) +
                        dp_ccw * math.sin(ccw_angle))

        # Calculate wind direction using atan2
        # Y-component is negated to correct for coordinate system orientation
        wind_direction_rad = math.atan2(y_component, x_component)
        wind_direction_deg = math.degrees(wind_direction_rad)

        # Normalize to 0-360°
        if wind_direction_deg < 0:
            wind_direction_deg += 360

        return wind_direction_deg

    except Exception as e:
        # Return 0° on any calculation error (e.g., division by zero)
        return 0.0


def calculate_speed_mps(dp_ctr, dp_cw, dp_ccw, temp_celsius):
    """Calculate wind speed from differential pressure sensors using Sensirion TAS formula

    This function implements Sensirion's True Airspeed (TAS) formula for SDP3x sensors
    with temperature and pressure compensation. The formula accounts for the thermal
    measurement principle of the SDP3x sensors and provides accurate wind speed readings.

    Formula: v = TAS = [(p₀/p) * sqrt(T/T₀)] * sqrt(2 * dp_sensor,DP / ρ(p₀,T₀))

    Where:
    - p₀ = 966 mbar (calibration pressure)
    - T₀ = 298.15 K (calibration temperature) 
    - ρ(p₀,T₀) = 1.1289 kg/m³ (air density at calibration conditions)
    - p = 1013.25 mbar (sea level pressure - 445m lower than Sensirion's altitude)
    - T = measured temperature in Kelvin

    Tobi measured this in Zurich and found that with Eurochron Modl WS4003 indicating 7.4m/s, 
    the wind speed was shown as about 8.3m/s, so the accuracy is acceptable. Lower speed of 4m/s was
    shown about the same with the same approx 10% overestimation.

    Args:
        dp_ctr: Differential pressure from center sensor (0x21, 0°) in Pascals
        dp_cw: Differential pressure from CW 120° sensor (0x22) in Pascals
        dp_ccw: Differential pressure from CCW 240° sensor (0x23) in Pascals
        temp_celsius: Temperature in Celsius from sensor

    Returns:
        Wind speed in m/s (True Airspeed)
    """
    # Sensirion calibration constants
    p0 = 966.0  # mbar - calibration pressure
    T0 = 298.15  # K - calibration temperature (25°C)
    rho_cal = 1.1289  # kg/m³ - air density at calibration conditions

    # Sea level conditions (445m lower than Sensirion's standard altitude)
    p_sea_level = 1013.25  # mbar - standard sea level pressure

    # Convert temperature to Kelvin
    T_kelvin = temp_celsius + 273.15

    # Calculate pressure and temperature compensation factor
    # [(p₀/p) * sqrt(T/T₀)]
    pressure_ratio = p0 / p_sea_level
    temp_ratio = T_kelvin / T0
    compensation_factor = pressure_ratio * math.sqrt(temp_ratio)

    # Calculate total differential pressure magnitude
    # Use the magnitude of the vector sum for more accurate wind speed
    dp_total = math.sqrt(dp_ctr**2 + dp_cw**2 + dp_ccw**2)

    # Apply Sensirion TAS formula
    # sqrt(2 * dp_sensor,DP / ρ(p₀,T₀))
    dp_term = 2.0 * dp_total / rho_cal

    # Final wind speed calculation
    wind_speed = compensation_factor * math.sqrt(dp_term)

    return wind_speed


class AnemNode(Node):
    def __init__(self, debug_visually: bool = False):
        super().__init__('anem_node')
        
        # Initialize pause service
        self.pause_service = TogglePauseService(self)
        
        self.get_logger().info('Initializing Anemometer node...')

        # Publishers
        self.pub_diff_pressure = self.create_publisher(
            Vector3, 'anem_diffpressure', 10)
        self.pub_wind_temp = self.create_publisher(
            Vector3, 'anem_speed_angle_temp', 10)
        self.pub_health = self.create_publisher(
            Bool, 'anem_health', 10)
        self.health_status = False  # Track current health status

        # Visual debug mode flag
        self.debug_visually = debug_visually

        # CRC error logging throttling
        self._last_crc_error_log_time = 0.0

        # Node health tracking for transient I2C failures
        self.node_healthy = True
        self._last_io_error_log_time = 0.0
        self._consecutive_io_errors = 0
        self._last_successful_read_time = time.time()
        self._last_recovery_attempt_time = 0.0
        self._recovery_attempt_count = 0

        # I2C setup
        # CTR is center (0° - front/back), CW is 120° from front, CCW is 240° from front (looking down on mast)
        self.i2cAddr = (I2C_CTR, I2C_CW, I2C_CCW)
        self.bus = None
        self.sensors_ready = False
        self._last_error_log_time = 0.0

        try:
            self.bus = smbus.SMBus(0)  # The default i2c bus
            self.get_logger().info('Opened i2c SMBus')
        except FileNotFoundError:
            self.get_logger().error("CRITICAL: I2C bus not found. Is I2C enabled? Exiting.")
            self._publish_health_status(False)
            sys.exit(1)

        if self.bus is not None:
            if not self.setup_sensors():
                self.get_logger().fatal("FATAL: Failed to setup anemometer sensors. Exiting.")
                self._publish_health_status(False)
                sys.exit(1)
            else:
                self.sensors_ready = True

        # ROS2 timer runs at publishing rate - reads multiple samples per callback
        self.timer = self.create_timer(
            1.0 / PUBLISHING_RATE, self.publish_callback)

        # Report actual sensor status
        if self.sensors_ready:
            self.get_logger().info("Initialization of anemometer wind sensor completed successfully.")
            # Publish initial health status
            self._publish_health_status(True)

        # Visual mode init
        self._vis_initialized = False
        self._vis_width = VISUAL_DISPLAY_WIDTH
        self._vis_height = VISUAL_DISPLAY_HEIGHT
        # Full-scale visual radius corresponds to configured knots converted to m/s
        self._vis_speed_ref = VISUAL_FULLSCALE_SPEED_KNOTS * \
            0.514444  # knots to m/s conversion
        if self.debug_visually:
            self._init_visual()

    def _publish_health_status(self, is_healthy: bool):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            if is_healthy:
                self.get_logger().info("Anemometer health status: HEALTHY")
            else:
                self.get_logger().warn("Anemometer health status: FAILED")

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
            sys.stdout.write('\x1b[?25h')  # show cursor
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False

    def _make_dp_bar(self, dp_value: float, label: str, bar_width: int) -> str:
        """Create a horizontal bar chart for differential pressure

        Args:
            dp_value: Differential pressure in Pascals
            label: Label for this sensor (e.g. "CTR", "CW", "CCW")
            bar_width: Width of the bar portion (excluding label and value)

        Returns:
            String representation of the bar chart
        """
        # Calculate position of 'X' marker (0 Pa at center)
        center = bar_width // 2
        scale = center / VISUAL_DP_FULLSCALE_PA
        offset = int(round(dp_value * scale))
        x_pos = center + offset

        # Clamp to bar width
        x_pos = max(0, min(bar_width - 1, x_pos))

        # Build bar
        bar = ['-'] * bar_width
        bar[center] = '|'  # Zero mark

        # Place value marker (X overwrites zero mark if at center)
        if x_pos == center:
            bar[x_pos] = 'O'  # Use 'O' when value is at zero
        else:
            bar[x_pos] = 'X'  # Use 'X' for non-zero values

        # Format: "CTR: +12.34 Pa |--------X--------|"
        return f"{label}: {dp_value:+7.2f} Pa {''.join(bar)}"

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
        bar_width = width - 18  # Reserve space for label and value
        header = [
            f"Wind v={speed_mps:5.2f} m/s  angle={angle_deg:6.2f} deg  temp={temp_c:5.1f} C",
            "",
            f"Differential Pressure Sensors (CTR={hex(I2C_CTR)}, CW={hex(I2C_CW)}, CCW={hex(I2C_CCW)}):",
            self._make_dp_bar(dp_tuple[0], "CTR", bar_width),
            self._make_dp_bar(dp_tuple[1], "CW ", bar_width),
            self._make_dp_bar(dp_tuple[2], "CCW", bar_width),
            "",
            f"Wind vector display (scale: {self._vis_speed_ref:.1f} m/s = full radius)",
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
        # SDP3x sensors don't respond to read_byte(), so we use write_i2c_block_data instead
        sensors_detected = []
        for a in self.i2cAddr:
            try:
                # Try to send stop continuous measurement command to test communication
                # This is a safe operation that should work on all SDP3x sensors
                self.bus.write_i2c_block_data(a, 0x3F, [0xF9])
                sensors_detected.append(hex(a))
            except IOError:
                pass  # Sensor not detected, continue checking others

        if sensors_detected:
            self.get_logger().info(
                f'Stopping existing continuous measurements on detected sensors: {sensors_detected}')
        else:
            self.get_logger().warn('Wind sensor not detected on I2C bus')
            return False

        for a in self.i2cAddr:
            try:
                # Stop any cont measurement
                self.bus.write_i2c_block_data(a, 0x3F, [0xF9])
            except IOError as e:
                self.get_logger().error(
                    f"Failed to communicate with sensor at address {hex(a)}: {e}")
                return False

        time.sleep(0.8)

        # Start Continuous Measurement (Table 6.3.1 in Data sheet)
        self.get_logger().info('Starting 0x3615 continuous measurement with average till read')
        # Command code (Hex)        Temperature compensation            Averaging
        # 0x3603                    Mass flow                           Average  till read
        # 0x3608                    Mass flow None                      Update rate 0.5ms
        # 0x3615                    Differential pressure               Average till read
        # 0x361E                    Differential pressure None          Update rate 0.5ms
        # 0x0006                    Soft reset
        # 0x3FF9                    Stop continuous measurement
        # first stop any continuous measurement, do a soft reset, then start the continuous measurement
        for a in self.i2cAddr:
            try:
                # self.bus.write_i2c_block_data(a, 0x00, [0x06])
                # time.sleep(0.1)
                # self.bus.write_i2c_block_data(a, 0x3F, [0xF9])
                # time.sleep(0.1)
                self.bus.write_i2c_block_data(a, 0x36, [0x15])
            except IOError as e:
                self.get_logger().error(
                    f"Failed to start measurement on sensor at address {hex(a)}: {e}")
                return False
        time.sleep(0.1)

        return True

    def _recover_sensors_with_reset(self):
        """Attempt sensor recovery with soft reset for severe I2C issues"""
        self.get_logger().info("Attempting sensor recovery with soft reset...")

        try:
            # First, try to stop any existing measurements
            for a in self.i2cAddr:
                try:
                    self.bus.write_i2c_block_data(a, 0x3F, [0xF9])
                except IOError:
                    pass  # Ignore errors during cleanup

            time.sleep(0.2)

            # Perform soft reset on all sensors
            for a in self.i2cAddr:
                try:
                    self.bus.write_i2c_block_data(a, 0x00, [0x06])
                except IOError:
                    pass  # Ignore errors during reset

            time.sleep(0.2)

            # Now try normal setup
            return self.setup_sensors()

        except Exception as e:
            self.get_logger().error(f"Sensor recovery with reset failed: {e}")
            return False

    def _read_sensor_data(self):
        """Read and validate sensor data with CRC checksum verification"""
        try:
            # Pre-allocate lists for better performance
            dp = [0.0, 0.0, 0.0]
            temps = [0.0, 0.0, 0.0]

            # Read raw sensor data from all three sensors
            for i, a in enumerate(self.i2cAddr):
                b = self.bus.read_i2c_block_data(a, 0, 9)

                # Verify CRC for differential pressure data (bytes 0,1,2)
                if not verify_crc([b[0], b[1]], b[2]):
                    self._log_crc_error(
                        f"sensor {hex(a)} differential pressure")
                    return None

                # Verify CRC for temperature data (bytes 3,4,5)
                if not verify_crc([b[3], b[4]], b[5]):
                    self._log_crc_error(f"sensor {hex(a)} temperature")
                    return None

                # Verify CRC for scale factor data (bytes 6,7,8)
                if not verify_crc([b[6], b[7]], b[8]):
                    self._log_crc_error(f"sensor {hex(a)} scale factor")
                    return None

                # Convert validated data
                # convert to Pascals diff pressure and add 0.95 compentation for sea level
                # according to
                # https://sensirion.com/media/documents/FEAE3023/667EC183/DP_AN_Signal_Compensation_V1.0_1.pdf
                dp[i] = (int_from_bytes([b[0], b[1]]) / 240.0)*0.95
                # convert to deg celsius
                temps[i] = int_from_bytes([b[3], b[4]]) / 200.0

            return dp, temps

        except IOError as e:
            self._handle_io_error(e)
            return None
        except IndexError as e:
            self.get_logger().error(f"Data parsing error: {e}. Continuing.")
            return None

    def _log_crc_error(self, sensor_info):
        """Log CRC error with throttling to prevent log spam"""
        current_time = time.time()
        if current_time - self._last_crc_error_log_time >= CRC_ERROR_LOG_THROTTLE_S:
            self.get_logger().warn(
                f"CRC checksum error detected on {sensor_info}")
            self._last_crc_error_log_time = current_time

    def _handle_io_error(self, error):
        """Handle I2C IOError with health tracking and throttled logging"""
        current_time = time.time()
        self._consecutive_io_errors += 1

        # Log error with throttling (max once per 5 seconds)
        if current_time - self._last_io_error_log_time >= 5.0:
            self.get_logger().warn(
                f"Transient I2C read error (attempt {self._consecutive_io_errors}): {error}")
            self._last_io_error_log_time = current_time

        # Mark node as unhealthy after first IO error
        if self.node_healthy:
            self.node_healthy = False
            self._publish_health_status(False)
            self.get_logger().warn(
                "Node health set to UNHEALTHY due to I2C errors. Switching to 1Hz retry mode.")
            # Switch to low-frequency retry mode
            self._switch_to_retry_mode()

    def _switch_to_retry_mode(self):
        """Switch timer to low-frequency retry mode (1Hz)"""
        if hasattr(self, 'timer'):
            self.timer.destroy()
        self.timer = self.create_timer(1.0, self.publish_callback)
        self.get_logger().info("Switched to 1Hz retry mode for I2C recovery")

        # Reset consecutive error counter for recovery tracking
        self._consecutive_io_errors = 0

    def _switch_to_normal_mode(self):
        """Switch timer back to normal frequency (3Hz)"""
        if hasattr(self, 'timer'):
            self.timer.destroy()
        self.timer = self.create_timer(
            1.0 / PUBLISHING_RATE, self.publish_callback)
        self.get_logger().info("Switched back to normal 3Hz mode - I2C communication recovered")

    def _check_io_recovery(self):
        """Check if I2C communication has recovered and switch back to normal mode"""
        current_time = time.time()
        time_since_last_success = current_time - self._last_successful_read_time
        time_since_last_recovery_attempt = current_time - self._last_recovery_attempt_time

        # Try recovery if we've been in retry mode for a while
        # This allows recovery even with ongoing errors, since sensors need re-initialization
        if (time_since_last_success > 5.0 and  # Been in retry mode for at least 5 seconds
                time_since_last_recovery_attempt > 3.0):  # Wait at least 3 seconds between attempts

            self._recovery_attempt_count += 1
            self._last_recovery_attempt_time = current_time

            # Re-initialize sensors after I2C recovery
            self.get_logger().info(
                f"Attempting sensor re-initialization (attempt {self._recovery_attempt_count})...")
            if self.setup_sensors():
                self.node_healthy = True
                self._publish_health_status(True)
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0  # Reset counter on success
                self.get_logger().info(
                    f"Sensor re-initialization successful after {self._consecutive_io_errors} I2C errors")
            else:
                # Try more aggressive recovery with soft reset
                self.get_logger().warn(
                    f"Normal sensor re-initialization failed (attempt {self._recovery_attempt_count}), trying soft reset recovery...")
                if self._recover_sensors_with_reset():
                    self.node_healthy = True
                    self._publish_health_status(True)
                    self._switch_to_normal_mode()
                    self._recovery_attempt_count = 0  # Reset counter on success
                    self.get_logger().info(
                        f"Sensor recovery with soft reset successful after {self._consecutive_io_errors} I2C errors")
                else:
                    self.get_logger().error(
                        f"All sensor recovery attempts failed (attempt {self._recovery_attempt_count}), staying in retry mode")
                    # Stay in retry mode and try again later

    def publish_callback(self):
        """ROS2 timer callback - reads multiple sensor samples and publishes averaged data at PUBLISHING_RATE"""
        # Check if node is paused
        if self.pause_service.is_paused():
            return  # Skip processing when paused
        
        try:
            # Read multiple samples and accumulate for averaging
            dp_samples = []
            temp_samples = []

            for _ in range(SAMPLES_PER_PUBLISH):
                sensor_data = self._read_sensor_data()
                if sensor_data is None:
                    continue  # CRC error or other issue, skip this sample

                dp, temps = sensor_data
                dp_samples.append(dp)
                temp_samples.append(temps)
                # Update successful read time for recovery detection
                self._last_successful_read_time = time.time()
                self._consecutive_io_errors = 0  # Reset error counter on success

            # Check for recovery from I2C errors (do this even if no valid samples)
            if not self.node_healthy:
                self._check_io_recovery()

            # Check if we got any valid samples
            if not dp_samples:
                self.get_logger().warn("No valid sensor samples in this cycle, skipping publish")
                self._publish_health_status(False)
                return

            # Average differential pressures across all valid samples
            dp_avg = [
                sum(s[i] for s in dp_samples) / len(dp_samples)
                for i in range(3)
            ]

            # Average temperatures across all valid samples
            temp_avg = [
                sum(s[i] for s in temp_samples) / len(temp_samples)
                for i in range(3)
            ]

            # Unpack averaged differential pressures with meaningful names
            dp_ctr_avg = dp_avg[0]  # Center sensor (0x21, 0°)
            dp_cw_avg = dp_avg[1]   # CW 120° sensor (0x22)
            dp_ccw_avg = dp_avg[2]  # CCW 240° sensor (0x23)

            # Calculate wind parameters from averaged data
            temp_celsius = sum(temp_avg) / 3.0
            angle_deg = calculate_angle_deg(dp_ctr_avg, dp_cw_avg, dp_ccw_avg)
            speed_mps = calculate_speed_mps(
                dp_ctr_avg, dp_cw_avg, dp_ccw_avg, temp_celsius)

            # Publish differential pressure
            self.pub_diff_pressure.publish(
                Vector3(x=float(dp_avg[0]), y=float(dp_avg[1]), z=float(dp_avg[2])))

            # Publish wind speed, angle, and temperature
            self.pub_wind_temp.publish(
                Vector3(x=float(speed_mps), y=float(angle_deg), z=float(temp_celsius)))

            # Publish health status as healthy
            self._publish_health_status(True)

            self.get_logger().debug(
                f"Anemometer: speed(m/s)={speed_mps:.2f} angle(deg)={angle_deg:.1f} "
                f"temp(C)={temp_celsius:.1f} dp(pascal)=({dp_avg[0]:.4f}, {dp_avg[1]:.4f}, {dp_avg[2]:.4f}) "
                f"[averaged over {len(dp_samples)} samples]"
            )

            if self.debug_visually:
                self._render_visual(speed_mps, angle_deg,
                                    temp_celsius, tuple(dp_avg))

        except Exception as e:
            self.get_logger().error(f"Error in publish callback: {e}")
            self._publish_health_status(False)

    def destroy_node(self):
        # This is the recommended way to perform cleanup in ROS2.
        # It gets called automatically when the node is destroyed.
        self.get_logger().info('Stopping existing continuous measurements on shutdown.')

        # Publish health=false on shutdown
        try:
            self._publish_health_status(False)
        except Exception:
            pass  # Ignore errors during shutdown publishing

        if self.debug_visually:
            self._teardown_visual()
        if self.bus:
            for a in self.i2cAddr:
                try:
                    # Stop any cont measurement
                    self.bus.write_i2c_block_data(a, 0x3F, [0xF9])
                except IOError:
                    self.get_logger().warn(
                        f"Could not stop sensor at address {hex(a)} on shutdown.")
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
  - Three sensors at addresses:
    * I2C_CTR (0x21): Center sensor (0° - front/back)
    * I2C_CW (0x22): Clockwise 120° from front (looking down on mast)
    * I2C_CCW (0x23): Counter-clockwise 120° from front (240° - looking down on mast)
  - Run "sudo i2cdetect -y 0" to verify sensor connections
  - Sensors show as addresses 21, 22, 23 in hex

Algorithm based on Sensirion's directional wind meter application:
  https://developer.sensirion.com/applications/directional-wind-meter-using-sdp3x/

Features:
  - Automatic sensor reconnection on I2C errors with health-based frequency adaptation
  - Optional visual debug mode with ASCII wind vector display and differential pressure bar charts
  - Temperature compensation and averaging
  - Configurable logging levels
  - CRC checksum validation for data integrity
  - Throttled CRC error logging to prevent log spam
  - Multi-sample averaging for noise reduction with configurable time constant
  - Sensor internal averaging (~10ms) + application averaging (~100ms)
  - Transient I2C failure recovery: switches to 1Hz retry mode, recovers to 3Hz normal operation

Configuration Constants (modify at top of file):
  - I2C_CTR: I2C address for center sensor (default: 0x21)
  - I2C_CW: I2C address for CW 60° sensor (default: 0x22)
  - I2C_CCW: I2C address for CCW 60° sensor (default: 0x23)
  - PUBLISHING_RATE: Final publishing rate in Hz (default: 3.0)
  - SAMPLES_PER_PUBLISH: Number of sensor reads to average per publish cycle (default: 15)
  - CRC_ERROR_LOG_THROTTLE_S: Maximum CRC error logging frequency in seconds (default: 1.0)
  - VISUAL_FULLSCALE_SPEED_KNOTS: Full-scale wind speed for visual display (default: 8.0)
  - VISUAL_DP_FULLSCALE_PA: Full-scale differential pressure for bar charts (default: 5.0)
  - VISUAL_DISPLAY_WIDTH: Visual display width in characters (default: 60)
  - VISUAL_DISPLAY_HEIGHT: Visual display height in characters (default: 40)

Topics Published:
  /anem_speed_angle_temp (geometry_msgs/Vector3):
    x: wind speed in m/s
    y: wind angle in degrees CW from front of boat (looking down)
    z: average temperature in celsius
  /anem_diffpressure (geometry_msgs/Vector3):
    x: differential pressure from I2C_CTR (0x21, 0°) in Pascals
    y: differential pressure from I2C_CW (0x22, 120°) in Pascals  
    z: differential pressure from I2C_CCW (0x23, 240°) in Pascals
  /anem_health (std_msgs/Bool):
    data: true when node is healthy, false when unhealthy or shutting down
    Published only on state changes, startup, and shutdown
        """
    )
    parser.add_argument('--debug', action='store_true',
                        help='Log sensor values to the terminal')
    parser.add_argument('--debug_visually', action='store_true',
                        help='Show test-mode ASCII visualization of wind vector')
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
