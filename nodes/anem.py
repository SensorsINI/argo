#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
# ROS2 Anemometer Node - Wind Speed and Direction Sensor

# Reading three Sensirion SDP3x differential pressure sensors
# Dev by JJ Slabbert, modified by tobi
# This ROS2 node communicates with three differential pressure sensors over I2C
# to determine wind speed and direction using directional wind meter principles.
#
# Hardware Setup:
# - As of argo PCB version 4, uses TWI2 on pins 27/28 (SDA.2/SCL.2)
#   for electrical isolation of the wind sensor in case of short circuit from the mast and salt water intrusion.
# - Three sensors (left to right on PCB; see pcb/WindSensor/README.md):
#   * I2C_CW (0x21): +120° CW from bow
#   * I2C_CTR (0x22): 0° bow/stern
#   * I2C_CCW (0x23): −120° CCW from bow
#   CTR: negate I2C; CW/CCW: +1 (pitot routing vs CTR, see README)
# - Resolve Linux bus index for TWI2 and verify with:
#   ls /sys/devices/platform/soc*/5002800.i2c/i2c-*
#   sudo i2cdetect -y <resolved_bus_index>
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
# Static Offset Calibration (argo.yaml):
# Each SDP3x sensor has a small zero-wind baseline differential pressure. Subtract per-sensor
# offsets (anem_node.ros__parameters.dp_offset_*) from raw I2C readings, then apply
# DP_SIGN_CORRECTION (−1 on CTR, +1 on CW/CCW) before speed/angle.
# With no airflow, run `python3 nodes/anem.py --debug` and average dp(pascal) over ~30 s.
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
#   y: wind angle in degrees CW from front (looking down); direction wind is coming FROM
#   z: average temperature in celsius
# /anem_diffpressure (geometry_msgs/Vector3) — calibrated, PCB sign-corrected:
#   x: I2C_CTR (0x22, 0°) in Pascals
#   y: I2C_CW (0x21, +120°) in Pascals
#   z: I2C_CCW (0x23, −120° / 240°) in Pascals

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))

from collections import deque
import math
from rclpy.logging import LoggingSeverity
import argparse
import argcomplete
import numpy as np
import time
import smbus
import glob
# Import ArgoBaseNode for standardized functionality
from argo_base_node import ArgoBaseNode
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

# I2C Configuration
# Wind sensors are on TWI2 (pins 27/28: PI10/PI9). The Linux adapter
# index can vary by image/kernel (often i2c-4 on Orange Pi 6.1 images).
TWI2_CONTROLLER_BASENAME = "5002800.i2c"
TWI2_DEFAULT_LINUX_BUS = 4

# I2C sensor addresses (left to right on PCB), looking down on mast.
# Sign convention (pcb/WindSensor/README.md, SDP32 datasheet):
#   I2C reports P(+) − P(−). Upper (windward) port is the − port → negate CTR.
#   ±120° pitot routing inverts sign vs CTR on the chip, so CW/CCW use × +1.
I2C_CW = 0x21   # +120° CW from bow
I2C_CTR = 0x22  # 0° bow/stern
I2C_CCW = 0x23  # −120° CCW from bow (240° CW from bow)

# Multiply (offset-subtracted) raw DP by index: [CTR, CW, CCW]
DP_SIGN_CORRECTION = (-1, 1, 1)

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
VISUAL_DP_FULLSCALE_PA = 15.0

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


def resolve_twi2_linux_bus(default_bus=TWI2_DEFAULT_LINUX_BUS):
    """Resolve Linux i2c-* index for TWI2 controller (5002800.i2c)."""
    try:
        matches = sorted(
            glob.glob(f"/sys/devices/platform/soc*/{TWI2_CONTROLLER_BASENAME}/i2c-*")
        )
        if not matches:
            return default_bus

        bus_name = os.path.basename(matches[0])  # e.g. "i2c-4"
        return int(bus_name.split("-")[1])
    except Exception:
        return default_bus


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
        dp_ctr: Calibrated DP from center sensor (0x22, 0°) in Pascals
        dp_cw: Calibrated DP from CW sensor (0x21, +120°) in Pascals
        dp_ccw: Calibrated DP from CCW sensor (0x23, −120°) in Pascals

    Returns:
        Wind angle in degrees CW from bow (looking down); direction wind is coming FROM.
        Range: 0°–360° (0° = from bow, 90° = from starboard, 180° = from stern).
    """
    try:
        # CTR at 0°, CW at +120°, CCW at 240° CW (120° spacing)
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
        dp_ctr: Calibrated DP from center sensor (0x22, 0°) in Pascals
        dp_cw: Calibrated DP from CW sensor (0x21, +120°) in Pascals
        dp_ccw: Calibrated DP from CCW sensor (0x23, −120°) in Pascals
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


class AnemNode(ArgoBaseNode):
    def __init__(self, debug_mode: bool = False, debug_visually: bool = False):
        super().__init__('anem_node')

        self.get_logger().info('Initializing Anemometer node...')

        # Zero-wind baseline offsets (Pa) — see nodes/argo.yaml anem_node section
        self.declare_parameter('dp_offset_ctr', 0.0)
        self.declare_parameter('dp_offset_cw', 0.0)
        self.declare_parameter('dp_offset_ccw', 0.0)
        self.declare_parameter('calm_speed_threshold_mps', 0.10)
        self._dp_offsets = (
            self.get_parameter('dp_offset_ctr').get_parameter_value().double_value,
            self.get_parameter('dp_offset_cw').get_parameter_value().double_value,
            self.get_parameter('dp_offset_ccw').get_parameter_value().double_value,
        )
        self._calm_speed_threshold = (
            self.get_parameter('calm_speed_threshold_mps').get_parameter_value().double_value
        )
        if any(abs(o) > 1e-9 for o in self._dp_offsets):
            self.get_logger().info(
                f"DP baseline offsets (Pa): CTR={self._dp_offsets[0]:.4f}, "
                f"CW={self._dp_offsets[1]:.4f}, CCW={self._dp_offsets[2]:.4f}"
            )

        # Publishers
        self.pub_diff_pressure = self.create_publisher(
            Vector3, 'anem_diffpressure', 10)
        self.pub_wind_temp = self.create_publisher(
            Vector3, 'anem_speed_angle_temp', 10)
        self.pub_temperature_air = self.create_publisher(
            Float32, 'temperature_air', 10)
        
        # Critical I2C failure publisher (for automatic RTH switching)
        self.pub_i2c_failure = self.create_publisher(
            Bool, '/argo/critical/i2c_failure', 10)
        self._i2c_failure_state = False  # Track current I2C failure state

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
        
        # Critical I2C failure detection (all sensors failed = critical)
        self._critical_i2c_failure = False
        self._all_sensors_failed_timeout = 30.0  # Consider critical after 30s of all sensors failing
        self._all_sensors_failed_start_time = None
        self._detected_sensors = set()  # Track which sensors are detected
        
        # Add a throttle for the "no valid samples" warning
        self._last_no_valid_samples_log_time = 0.0

        # Temperature publishing at low rate (1 per minute)
        self._last_temperature_publish_time = 0.0
        self._temperature_publish_interval = 60.0  # 60 seconds

        # I2C setup
        self.i2cAddr = (I2C_CTR, I2C_CW, I2C_CCW)
        self.bus = None
        self.i2c_bus = resolve_twi2_linux_bus()
        self.sensors_ready = False
        self._last_error_log_time = 0.0
        self.retry_timer = None
        self.main_timer = None
        self.get_logger().info(
            f"Resolved wind bus TWI2 ({TWI2_CONTROLLER_BASENAME}) to Linux i2c-{self.i2c_bus}"
        )

        # Attempt initial sensor setup
        if not self._initial_setup():
            self._start_retry_timer()
        else:
            # If setup is successful, start the main publisher timer
            self.main_timer = self.create_timer(
                1.0 / PUBLISHING_RATE, self.publish_callback)

        # Visual mode init
        self._vis_initialized = False
        # After I2C/sample failure, next successful redraw clears the whole terminal (no stale lines)
        self._vis_pending_full_clear = False
        self._vis_width = VISUAL_DISPLAY_WIDTH
        self._vis_height = VISUAL_DISPLAY_HEIGHT
        # Full-scale visual radius corresponds to configured knots converted to m/s
        self._vis_speed_ref = VISUAL_FULLSCALE_SPEED_KNOTS * \
            0.514444  # knots to m/s conversion
        if self.debug_visually:
            self._init_visual()

    def _initial_setup(self):
        """Attempts to initialize I2C bus and sensors, returns True on success."""
        try:
            if not self.bus:
                self.bus = smbus.SMBus(self.i2c_bus)
                self.get_logger().info(f'Opened I2C SMBus bus {self.i2c_bus} for wind sensors')
        except FileNotFoundError:
            # Only log this error on the first attempt in a retry cycle
            if not hasattr(self, '_is_first_retry_log') or self._is_first_retry_log:
                self.get_logger().error(
                    f"I2C bus i2c-{self.i2c_bus} not found for TWI2 wind bus. Is pi-i2c2 enabled?"
                )
            return False

        if self.bus:
            if self.setup_sensors():
                self.sensors_ready = True
                self.set_healthy("Sensors operating normally.")
                return True
            else:
                # Only log this error on the first attempt in a retry cycle
                if not hasattr(self, '_is_first_retry_log') or self._is_first_retry_log:
                    # Pass log_on_fail=True to get detailed error on first attempt
                    self.setup_sensors(log_on_fail=True)
                return False
        return False

    def _start_retry_timer(self):
        """Starts a timer to periodically retry sensor setup."""
        if self.retry_timer is None or self.retry_timer.is_canceled():
            self.get_logger().info("Starting sensor setup retry timer (0.1 Hz).")
            self.retry_timer = self.create_timer(10.0, self._retry_setup_callback)
            self._is_first_retry_log = True
            self._last_retry_log_time = 0.0

    def _retry_setup_callback(self):
        """Callback for the retry timer."""
        current_time = time.time()
        log_now = self._is_first_retry_log or (current_time - self._last_retry_log_time > 60.0)

        if log_now:
            self.get_logger().warn("Attempting to connect to anemometer sensors...")
            self._last_retry_log_time = current_time
        
        if self._initial_setup():
            self.get_logger().info("Successfully connected to anemometer sensors after retries.")
            self._is_first_retry_log = True # Reset for next potential failure
            if self.debug_visually:
                self._vis_pending_full_clear = True
            if self.retry_timer:
                self.retry_timer.cancel()
            self.main_timer = self.create_timer(1.0 / PUBLISHING_RATE, self.publish_callback)
        else:
            self.set_unhealthy("Sensors not detected on I2C bus. Retrying...")
            if log_now:
                self._is_first_retry_log = False

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
        return f"{label}: {dp_value:+9.3f} Pa {''.join(bar)}"

    def _render_visual(self, speed_mps: float, angle_deg: float, temp_c: float, dp_tuple):
        if not self._vis_initialized:
            self._init_visual()
        if self._vis_pending_full_clear:
            try:
                sys.stdout.write('\x1b[2J')   # clear entire screen
                sys.stdout.write('\x1b[H')    # cursor home
                sys.stdout.flush()
            except Exception:
                pass
            self._vis_pending_full_clear = False
        width = self._vis_width
        height = self._vis_height
        cx = width // 2
        cy = height // 2

        # Wind-from arrow: angle_deg is where wind comes FROM (0° = from bow).
        # Bow is up on screen; marker 'o' is wind-from direction (not where it blows to).
        #   ex = cx + r*sin(θ), ey = cy - r*cos(θ)  →  θ=0° places 'o' above center (up).
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
        bar_width = width - 20  # Reserve space for label and value (+9.3f Pa)
        header = [
            f"Wind v={speed_mps:5.2f} m/s  angle={angle_deg:6.2f} deg  temp={temp_c:5.1f} C",
            "",
            f"DP raw Pa (avg before offset/sign; use for argo.yaml dp_offset_*):",
            self._make_dp_bar(dp_tuple[0], "CTR", bar_width),
            self._make_dp_bar(dp_tuple[1], "CW ", bar_width),
            self._make_dp_bar(dp_tuple[2], "CCW", bar_width),
            "",
            f"Wind-from vector (scale: {self._vis_speed_ref:.1f} m/s = full radius; bow/0 deg = up)",
            "  'o' = direction wind is coming FROM (0 deg=headwind from bow, arrow points up)",
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

    def setup_sensors(self, log_on_fail=False):
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
            # Only log this if we are also logging failures, to reduce noise
            if log_on_fail:
                self.get_logger().info(
                    f'Stopping existing continuous measurements on detected sensors: {sensors_detected}')
            # Update detected sensors set
            self._detected_sensors = set(sensors_detected)
            # Reset failure timer if sensors are detected
            if self._all_sensors_failed_start_time is not None:
                self._all_sensors_failed_start_time = None
                if self._critical_i2c_failure:
                    self._critical_i2c_failure = False
                    self._publish_i2c_failure(False)
                    self.get_logger().info("✅ I2C recovery: Wind sensors detected - critical failure cleared")
                    if self.debug_visually:
                        self._vis_pending_full_clear = True
        else:
            if log_on_fail:
                self.get_logger().warn('Wind sensor not detected on I2C bus')
            # All sensors failed - start critical failure timer
            current_time = time.time()
            if self._all_sensors_failed_start_time is None:
                self._all_sensors_failed_start_time = current_time
                self.get_logger().warn("All wind sensors failed - starting critical failure timer")
            else:
                # Check if all sensors have been failing for too long (critical failure)
                failure_duration = current_time - self._all_sensors_failed_start_time
                if failure_duration >= self._all_sensors_failed_timeout and not self._critical_i2c_failure:
                    self._critical_i2c_failure = True
                    self._publish_i2c_failure(True)
                    self.get_logger().error(
                        f"🔴 CRITICAL I2C FAILURE: All wind sensors have been failing for {failure_duration:.1f}s. "
                        f"Wind monitoring unavailable - controller should switch to RTH mode.")
            return False

        for a in self.i2cAddr:
            try:
                # Stop any cont measurement
                self.bus.write_i2c_block_data(a, 0x3F, [0xF9])
            except IOError as e:
                if log_on_fail:
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
                if log_on_fail:
                    self.get_logger().error(
                        f"Failed to start measurement on sensor at address {hex(a)}: {e}")
                return False
        time.sleep(0.1)

        return True

    def _recover_sensors_with_reset(self, log_on_fail=False):
        """Attempt sensor recovery with soft reset for severe I2C issues"""
        if log_on_fail:
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
            return self.setup_sensors(log_on_fail=log_on_fail)

        except Exception as e:
            if log_on_fail:
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

        # Log error with throttling (max once per 30 seconds)
        if current_time - self._last_io_error_log_time >= 60.0:
            self.get_logger().warn(
                f"Transient I2C read error: {error} (Total consecutive errors: {self._consecutive_io_errors})")
            self._last_io_error_log_time = current_time

        # Mark node as unhealthy after first IO error
        if self.node_healthy:
            self.node_healthy = False
            self.set_unhealthy("I2C read error")
            self.get_logger().warn(
                "Node health set to UNHEALTHY due to I2C errors. Switching to 1Hz retry mode.")
            # Switch to low-frequency retry mode
            self._switch_to_retry_mode()

    def _switch_to_retry_mode(self):
        """Switch timer to low-frequency retry mode (1Hz)"""
        if hasattr(self, 'main_timer') and self.main_timer:
            self.main_timer.destroy()
        self.main_timer = self.create_timer(1.0, self.publish_callback)
        self.get_logger().info("Switched to 1Hz retry mode for I2C recovery")

        # Reset consecutive error counter for recovery tracking
        self._consecutive_io_errors = 0

    def _switch_to_normal_mode(self):
        """Switch timer back to normal frequency (3Hz)"""
        if hasattr(self, 'main_timer') and self.main_timer:
            self.main_timer.destroy()
        self.main_timer = self.create_timer(
            1.0 / PUBLISHING_RATE, self.publish_callback)
        self.get_logger().info("Switched back to normal 3Hz mode - I2C communication recovered")
    
    def _publish_i2c_failure(self, failed: bool):
        """Publish I2C failure status to critical failure topic"""
        try:
            msg = Bool(data=failed)
            self.pub_i2c_failure.publish(msg)
            self._i2c_failure_state = failed
            if failed:
                self.get_logger().error("Published CRITICAL I2C failure - controller should switch to RTH")
            else:
                self.get_logger().info("Published I2C recovery - critical failure cleared")
        except Exception as e:
            self.get_logger().error(f"Error publishing I2C failure status: {e}")

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
            
            # Throttle recovery attempt logging
            log_now = (self._recovery_attempt_count == 1) or (current_time - self._last_recovery_log_time > 60.0)
            if not hasattr(self, '_last_recovery_log_time'):
                self._last_recovery_log_time = 0.0

            if log_now:
                self.get_logger().info(f"Attempting sensor re-initialization (attempt {self._recovery_attempt_count})...")
                self._last_recovery_log_time = current_time

            if self.setup_sensors(log_on_fail=log_now):
                self.node_healthy = True
                self.set_healthy("I2C communication recovered")
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0  # Reset counter on success
                if self.debug_visually:
                    self._vis_pending_full_clear = True
                self.get_logger().info(f"Sensor re-initialization successful after {self._consecutive_io_errors} I2C errors")
            
            elif log_now: # Only log failures if we logged the attempt
                # Try more aggressive recovery with soft reset
                self.get_logger().warn(f"Normal sensor re-initialization failed, trying soft reset recovery...")
                if self._recover_sensors_with_reset(log_on_fail=log_now):
                    self.node_healthy = True
                    self.set_healthy("I2C communication recovered after soft reset")
                    self._switch_to_normal_mode()
                    self._recovery_attempt_count = 0  # Reset counter on success
                    if self.debug_visually:
                        self._vis_pending_full_clear = True
                    self.get_logger().info(f"Sensor recovery with soft reset successful after {self._consecutive_io_errors} I2C errors")
                else:
                    self.get_logger().error(f"All sensor recovery attempts failed, staying in retry mode")
                    # Stay in retry mode and try again later

    def publish_callback(self):
        """ROS2 timer callback - reads multiple sensor samples and publishes averaged data at PUBLISHING_RATE"""
        # Removed pause functionality - anem node runs continuously

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
                
                # Reset failure timer on successful read
                if self._all_sensors_failed_start_time is not None:
                    self._all_sensors_failed_start_time = None
                    if self._critical_i2c_failure:
                        self._critical_i2c_failure = False
                        self._publish_i2c_failure(False)
                        self.get_logger().info("✅ I2C recovery: Wind sensor communication restored - critical failure cleared")
                        if self.debug_visually:
                            self._vis_pending_full_clear = True

            # Check for recovery from I2C errors (do this even if no valid samples)
            if not self.node_healthy:
                self._check_io_recovery()

            # Check if we got any valid samples
            if not dp_samples:
                current_time = time.time()
                if current_time - self._last_no_valid_samples_log_time > 60.0:
                    self.get_logger().warn("No valid sensor samples in this cycle, skipping publish. Will retry.")
                    self._last_no_valid_samples_log_time = current_time
                self.set_unhealthy("No valid sensor samples")
                if self.debug_visually:
                    self._vis_pending_full_clear = True

                # Check for critical I2C failure (all sensors failed for extended period)
                if self._all_sensors_failed_start_time is None:
                    self._all_sensors_failed_start_time = current_time
                else:
                    failure_duration = current_time - self._all_sensors_failed_start_time
                    if failure_duration >= self._all_sensors_failed_timeout and not self._critical_i2c_failure:
                        self._critical_i2c_failure = True
                        self._publish_i2c_failure(True)
                        self.get_logger().error(
                            f"🔴 CRITICAL I2C FAILURE: All wind sensors have been failing for {failure_duration:.1f}s. "
                            f"Wind monitoring unavailable - controller should switch to RTH mode.")
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

            # Baseline subtract (offsets from raw I2C), then PCB sign correction
            dp_cal = [
                (dp_avg[i] - self._dp_offsets[i]) * DP_SIGN_CORRECTION[i]
                for i in range(3)
            ]

            # Unpack: i2cAddr order is (CTR, CW, CCW)
            dp_ctr_avg = dp_cal[0]  # 0x22, 0°
            dp_cw_avg = dp_cal[1]   # 0x21, +120°
            dp_ccw_avg = dp_cal[2]  # 0x23, −120°

            # Calculate wind parameters from calibrated data
            temp_celsius = sum(temp_avg) / 3.0
            angle_deg = calculate_angle_deg(dp_ctr_avg, dp_cw_avg, dp_ccw_avg)
            speed_mps = calculate_speed_mps(
                dp_ctr_avg, dp_cw_avg, dp_ccw_avg, temp_celsius)
            if speed_mps < self._calm_speed_threshold:
                speed_mps = 0.0

            # Publish calibrated differential pressure (baseline subtracted)
            self.pub_diff_pressure.publish(
                Vector3(x=float(dp_cal[0]), y=float(dp_cal[1]), z=float(dp_cal[2])))

            # Publish wind speed, angle, and temperature
            self.pub_wind_temp.publish(
                Vector3(x=float(speed_mps), y=float(angle_deg), z=float(temp_celsius)))

            # Publish air temperature at low rate (1 per minute)
            current_time = time.time()
            if current_time - self._last_temperature_publish_time >= self._temperature_publish_interval:
                self.pub_temperature_air.publish(Float32(data=float(temp_celsius)))
                self._last_temperature_publish_time = current_time

            # Publish health status as healthy
            self.set_healthy("Sensors operating normally.")

            self.get_logger().debug(
                f"Anemometer: speed(m/s)={speed_mps:.2f} angle(deg)={angle_deg:.1f} "
                f"temp(C)={temp_celsius:.1f} "
                f"dp_raw(pascal)=({dp_avg[0]:.4f}, {dp_avg[1]:.4f}, {dp_avg[2]:.4f}) "
                f"dp_cal(pascal)=({dp_cal[0]:.4f}, {dp_cal[1]:.4f}, {dp_cal[2]:.4f}) "
                f"[averaged over {len(dp_samples)} samples]"
            )

            if self.debug_visually:
                self._render_visual(speed_mps, angle_deg,
                                    temp_celsius, tuple(dp_avg))

        except Exception as e:
            self.get_logger().error(f"Error in publish callback: {e}")
            self.set_unhealthy(f"Error in publish callback: {e}")

    def _enter_sleep_mode(self):
        """Put all SDP3x sensors into sleep mode (command 0x3677).

        Per datasheet section 6.3.5/6.3.6: enter sleep from idle mode. We
        first stop continuous measurement (0x3FF9), then send sleep (0x3677).
        """
        try:
            # Stop continuous measurement on all sensors
            for a in self.i2cAddr:
                try:
                    self.bus.write_i2c_block_data(a, 0x3F, [0xF9])  # 0x3FF9
                except IOError:
                    pass  # Ignore if already idle
            time.sleep(0.01)

            # Enter sleep mode on all sensors
            for a in self.i2cAddr:
                try:
                    self.bus.write_i2c_block_data(a, 0x36, [0x77])  # 0x3677
                except IOError as e:
                    self.get_logger().warn(f"Sleep command failed for sensor {hex(a)}: {e}")

            self.get_logger().info("SDP3x sensors placed into SLEEP mode (0x3677)")
        except Exception as e:
            self.get_logger().error(f"Failed to enter sleep mode: {e}")

    def _exit_pause_mode(self):
        """Wake sensors and (re)start continuous measurement (0x3615)."""
        try:
            # Any valid write wakes the sensor; setup_sensors configures 0x3615
            if self.setup_sensors():
                self.node_healthy = True
                self.set_healthy("Sensors reinitialized after pause")
                self.get_logger().info("SDP3x sensors reinitialized after pause (continuous measurement)")
            else:
                self.get_logger().error("Failed to reinitialize SDP3x sensors after pause")
        except Exception as e:
            self.get_logger().error(f"Error exiting pause mode: {e}")

    def destroy_node(self):
        # This is the recommended way to perform cleanup in ROS2.
        # It gets called automatically when the node is destroyed.
        self.get_logger().info('Stopping existing continuous measurements on shutdown.')

        # Publish health=false on shutdown
        self.set_unhealthy("Node shutting down")

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
    # This is a bit custom, so not using ArgoBaseNode.run_node directly
    node = None
    try:
        parser = ArgoBaseNode.create_standard_parser(
            'Anemometer Node for ROS2 - Wind Speed and Direction Sensor',
            epilog="""
This ROS2 node reads three Sensirion SDP3x differential pressure sensors over I2C
to determine wind speed and direction using directional wind meter principles.

Hardware Setup:
  - Uses TWI2 on pins 27/28 (SDA.2/SCL.2)
  - Linux bus index is resolved at runtime from controller 5002800.i2c
  - Three sensors (see pcb/WindSensor/README.md):
    * I2C_CW (0x21): +120° CW from bow
    * I2C_CTR (0x22): 0° bow/stern
    * I2C_CCW (0x23): −120° CCW from bow
    * CTR I2C negated; CW/CCW not (pitot routing inverts vs center)
  - Run "ls /sys/devices/platform/soc*/5002800.i2c/i2c-*" to resolve Linux i2c-* index
  - Then run "sudo i2cdetect -y <resolved_bus_index>" to verify sensor connections
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
  - I2C_CW: 0x21 (+120°), I2C_CTR: 0x22 (0°), I2C_CCW: 0x23 (−120°)
  - DP_SIGN_CORRECTION: (-1, 1, 1) — negate CTR I2C; CW/CCW unchanged (pitot routing)
  - PUBLISHING_RATE: Final publishing rate in Hz (default: 3.0)
  - SAMPLES_PER_PUBLISH: Number of sensor reads to average per publish cycle (default: 15)
  - CRC_ERROR_LOG_THROTTLE_S: Maximum CRC error logging frequency in seconds (default: 1.0)
  - VISUAL_FULLSCALE_SPEED_KNOTS: Full-scale wind speed for visual display (default: 8.0)
  - VISUAL_DP_FULLSCALE_PA: Full-scale differential pressure for bar charts (default: 15.0)
  - VISUAL_DISPLAY_WIDTH: Visual display width in characters (default: 60)
  - VISUAL_DISPLAY_HEIGHT: Visual display height in characters (default: 40)

Topics Published:
  /anem_speed_angle_temp (geometry_msgs/Vector3):
    x: wind speed in m/s
    y: wind angle in degrees CW from front of boat (looking down)
    z: average temperature in celsius
  /temperature_air (std_msgs/Float32):
    Air temperature in celsius (published at low rate: 1 per minute)
  /anem_diffpressure (geometry_msgs/Vector3):
    x: I2C_CTR (0x22, 0°) in Pascals
    y: I2C_CW (0x21, +120°) in Pascals
    z: I2C_CCW (0x23, −120°) in Pascals
  /anem_node_health (std_msgs/Bool):
    data: true when node is healthy, false when unhealthy or shutting down
    Published only on state changes, startup, and shutdown
        """
        )
        parser.add_argument('--debug_visually', action='store_true',
                            help='Show test-mode ASCII visualization of wind vector')
        
        parsed_args, unknown_args = parser.parse_known_args(args)
        argcomplete.autocomplete(parser)

        rclpy.init(args=unknown_args)

        # Pass debug_visually to constructor
        node = AnemNode(debug_mode=parsed_args.debug, 
                        debug_visually=parsed_args.debug_visually)

        if parsed_args.debug_visually:
            # Suppress routine logs to avoid interfering with visual display
            node.get_logger().set_level(LoggingSeverity.WARN)
        elif parsed_args.debug:
            node.get_logger().set_level(LoggingSeverity.DEBUG)
            node.get_logger().info('Debug logging enabled; sensor values will be printed.')
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("\nKeyboard interrupt, shutting down anemometer node.")
    except ExternalShutdownException:
        if node:
            node.get_logger().info("External shutdown signal received, exiting gracefully.")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
