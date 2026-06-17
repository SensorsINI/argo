#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
# Battery/Water ROS2 node
# - Reads MAX11612 ADC: AIN0=battery via 27k/18k divider, AIN1=saltwater probe, AIN2=sail winch shunt
# - Reads SHT45 temperature/humidity sensor
# - Monitors MP2672GD charger status via GPIO (always) and optionally I2C (if host control enabled)
# 
# MP2672 Charger Configuration:
# - MP2672_HOST_CONTROL constant controls operating mode:
#   * False (default): Standalone mode - uses GPIO only for charging and AC power status
#   * True: Host control mode - uses I2C registers (primary) and GPIO (fallback)
# 
# MP2672 Charger I2C Support (when MP2672_HOST_CONTROL = True):
# - I2C Address: 0x4B (only accessible when USB power is present)
# - Register 0x03 (Status): Reads CHG_STAT, PPM_STAT, BATTFLOAT_STAT, THERM_STAT, VSYS_STAT
# - Register 0x04 (Fault): Reads WD_FAULT, INPUT_FAULT, THERM_SD_FAULT, TIMER_FAULT, BAT_FAULT, NTC_FAULT
# - Automatic configuration on USB connection:
#   * Disables CHG timer (REG02H bits [2:1] = 00) to prevent charging timeout during long development sessions
#   * Disables Suspend mode (REG02H bit [0] = 1) to keep boost enabled
# - During normal robot operation (battery only), MP2672 not accessible via I2C (powered by USB)
# - Battery missing/balance cable fault detection via BATTFLOAT_STAT (REG 03H bit 2)
# 
# Charging Status Priority (when MP2672_HOST_CONTROL = True):
# 1. MP2672 I2C register 0x03 (when USB power present)
# 2. GPIO status with time-window filtering (fallback when USB not present)
# 
# Charging Status (when MP2672_HOST_CONTROL = False):
# - GPIO status with time-window filtering (standalone mode)
# 
# Publishes (Float32):
# - battery_voltage (V), saltwater_voltage (V), sail_current (A), pcb_temperature (C), relative_humidity (%)
# - battery_remaining_pct (%) using per‑cell LiPo formula: soc% = S − S/(1 + (v/V0)^A)^B
# Publishes (Bool):
# - charging_status (true=charging, false=not charging) - from MP2672 I2C or GPIO
# - ac_power_present (true=AC/USB power present, false=not present) - from MP2672 I2C or GPIO
# Alerts (Bool):
# - battery_low_alert (hysteresis 50 mV around battery_low_threshold_v; warning on low, info on recover)
# - saltwater_alert (voltage >= saltwater_alert_threshold_v)
# - humidity_alert (RH% >= humidity_alert_threshold_pct)
# Health (Bool):
# - battery_water_health (true=healthy, false=failed)
# Publishing optimization (saves rosbag space):
# - First 30s: publishes all sensor data at 1Hz, logs sensor states every 5s via ROS info
# - After 30s: publishes sensor data only when values change >5% OR every 60s (whichever is shorter)
# - Alert topics always publish at 1Hz regardless of optimization for safety
# Debug:
# - ASCII terminal bars when --debug is used (disabled during --test-adc)
# Key parameters:
# - battery_low_threshold_v (default 7.2 V), saltwater_alert_threshold_v (1.0 V), humidity_alert_threshold_pct (75.0)
# - battery_series_cells (default 2)
# - soc_S (123.0), soc_V0 (3.7), soc_A (80.0), soc_B (0.165)
# Hardware:
# - I2C Bus: Exclusively uses I2C bus 0 (Orange Pi Zero 2W default I2C interface)
# - I2C Pins: SDA=PI6 (twi0-sda), SCL=PI5 (twi0-sck) - configured via pi-i2c0 overlay
# - MAX11612 ADC at I2C address 0x34, SHT45 sensor at I2C address 0x44
# - MP2672 charger at I2C address 0x4B (only accessible when USB power present and MP2672_HOST_CONTROL = True)
# - GPIO Pins: PC12 (pin 36, line 76) !CHARGING, PH9 (pin 26, line 233) !ACOK from MP2672GD; PI4 (pin 38, line 260) !PG from CH221K USB-C voltage controller Pin 3 (open-drain, active low, requires pull-up)
#   - !PG indicates successful USB Power Delivery (PD) negotiation - LOW = PD successful, HIGH = no PD (standard USB) or failed
#   - Note: With standard USB cables (non-PD), this will be HIGH, not an indication of power failure
# - HOST_CTL: GPIO 229 (pin 24) tied high via hardware pullup resistor for host control mode

import rclpy
import math
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
import time
import sys
import signal
import argparse
import argcomplete
import os
import subprocess
import csv
import json
import math
import re

# Import ArgoBaseNode
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode
from datetime import datetime
from rclpy.executors import ExternalShutdownException
from collections import deque
from typing import Optional, Tuple

# Using standard Trigger service - no custom imports needed

# Hardware configuration constants
I2C_BUS_NUMBER = 0  # Orange Pi Zero 2W default I2C bus

# MP2672 configuration
MP2672_HOST_CONTROL = False  # Set to False for standalone mode (GPIO only), True for host control mode (I2C + GPIO)
# MP2672 I2C configuration (only used when MP2672_HOST_CONTROL = True)
MP2672_I2C_ADDR = 0x4B  # MP2672 I2C address in host control mode
MP2672_REG_01 = 0x01  # Register 01H (see datasheet)
MP2672_REG_02 = 0x02  # Register 02H - Configuration register (default: 0x95, see datasheet)
MP2672_REG_STATUS = 0x03  # Register 03H - Status register (see datasheet page 31)
MP2672_REG_04 = 0x04  # Register 04H - Fault register (default: 0x00, read-only, see datasheet)
# Note: REG 05H is OTP-only and not accessible to customers

# GPIO configuration for MP2672GD charger status
CHARGING_GPIO_LINE = 76   # PC12 (pin 36) - !CHARGING from MP2672GD
ACOK_GPIO_LINE = 233      # PH9 (pin 26) - !ACOK from MP2672GD
PG_GPIO_LINE = 260        # PI4 (pin 38) - !PG from CH221K USB-C voltage controller Pin 3 (open-drain output, active low, REQUIRES external pull-up)
                         # Indicates successful USB Power Delivery (PD) negotiation: LOW = PD successful, HIGH = no PD (standard USB) or failed
                         # Note: With standard USB cables (non-PD), this will be HIGH - not an indication of power failure
HOST_CTL_GPIO_LINE = 229  # PH5/SPI1_CS0 (pin 24) - HOST_CTL to MP2672GD (not used, MPC2672GD reads CV pin on power-on only)

# Sample rate configuration - dual timers for different sensor requirements
SAIL_CURRENT_RATE_HZ = 5.0  # Hz for sail current (control critical)
BATTERY_SAFETY_INTERVAL_S = 5.0  # seconds for battery/saltwater/humidity (safety critical)

# only publish if the change is greater than this percentage
THRESHOLD_CHANGE_PCT = 0.1  # Reduced from 1.0 to 0.1% for more frequent publishing

# Battery monitoring thresholds
BATTERY_LOW_THRESHOLD_V = 7.5       # V, ~20% for 2S LiPo, triggers warning, makes node unhealthy
BATTERY_CRITICAL_THRESHOLD_V = 7.2  # V, pack empty / halt (matches power_control CRITICAL_BATTERY_THRESHOLD_V)
BATTERY_FULLY_CHARGED_THRESHOLD_V = 8.2  # V, fully charged, by observation with USB charging pluugged in under full laod (sans servos)

# Battery lifetime estimation configuration
# Quantization: 12-bit ADC, Vref=4.096V, 27k/18k divider (2.5x) -> ~2.5mV per count at battery.
# Sampling every 5s; a single step over 5s gives ~1.8 V/h apparent slope. Use more samples and
# regime sanity bands to avoid noise-dominated estimates.
BATTERY_LIFETIME_SAMPLE_WINDOW = 180  # Samples for linear regression (5s interval -> 15 min window)
BATTERY_LIFETIME_MIN_SAMPLES = 15   # Minimum samples for slope (75s window); reduces quantization noise
BATTERY_SLOPES_FILE = "battery_slopes.json"  # Persistent storage for per-regime slopes (schema 2)

# Power-state model (voltage trend is authoritative; charger GPIO/I2C is only a hint):
#   discharging    - AC absent: battery powers everything
#   plugged_drain  - AC present but net battery drain (charger insufficient or faulted)
#   plugged_charge - AC present with sustained positive voltage trend (true net charging)
POWER_STATE_DISCHARGING = 'discharging'
POWER_STATE_PLUGGED_DRAIN = 'plugged_drain'
POWER_STATE_PLUGGED_CHARGE = 'plugged_charge'

# Slope-learning regimes. Discharging splits into idle/sailing by sail-winch activity.
REGIME_DISCHARGE_IDLE = 'discharge_idle'
REGIME_DISCHARGE_SAILING = 'discharge_sailing'
REGIME_PLUGGED_NET = 'plugged_net'
REGIME_CHARGING = 'charging'

# Default slopes from field data (Jun 2026, see docs/README-battery-power.md):
# idle desk ~-0.12 V/h (-7.5 %SOC/h), sailing bags ~-0.25 V/h (-15 %SOC/h),
# plugged-in idle: slow net drain observed with faulted/insufficient charger.
DEFAULT_REGIME_SLOPES = {
    REGIME_DISCHARGE_IDLE:    {'vph': -0.12, 'soc_pct_per_h': -7.5},
    REGIME_DISCHARGE_SAILING: {'vph': -0.25, 'soc_pct_per_h': -15.0},
    REGIME_PLUGGED_NET:       {'vph': -0.04, 'soc_pct_per_h': -2.5},
    REGIME_CHARGING:          {'vph': None,  'soc_pct_per_h': None},
}

# Per-regime sanity bands (V/h); fresh regression outside its band is not persisted
REGIME_SLOPE_BANDS_VPH = {
    REGIME_DISCHARGE_IDLE:    (-0.30, -0.05),
    REGIME_DISCHARGE_SAILING: (-1.00, -0.10),
    REGIME_PLUGGED_NET:       (-0.30,  0.30),
    REGIME_CHARGING:          ( 0.10,  1.00),
}

# Idle vs sailing classification: slow EMA of |sail current| above threshold -> sailing
SAILING_SAIL_CURRENT_THRESHOLD_A = 0.05
SAIL_ACTIVITY_EMA_TAU_S = 300.0  # ~5 min time constant

# AC present is classified plugged_charge only with a sustained positive trend
PLUGGED_CHARGE_MIN_SLOPE_VPH = 0.05

# Conservative on-water TTE floor (SOC space). ~-0.5 V/h equivalent at mid-pack.
# Applied ONLY when discharging AND sailing; idle estimates use measured slopes.
SAILING_TTE_FLOOR_SOC_PCT_PER_H = 30.0
# Fresh SOC slope gentler than this is treated as "flat" -> fall back to persisted regime slope
MIN_MEANINGFUL_SOC_SLOPE_PCT_PER_H = 1.0
# Storage rundown: flag file set by astore; discharge to 7.6V then shut down (cleared on reboot)
STORAGE_RUNDOWN_FLAG_FILE = '/tmp/argo_battery_storage_rundown'
STORAGE_VOLTAGE_V = 7.6  # Target voltage for storage; time-to-empty uses this when storage mode active

# MP2672 CHG timer configuration (safety timer that disables charging after timeout)
# Default: 20 hours (bits [2:1] = 10), can be disabled (00), 8hrs (01), or 12hrs (11)
MP2672_CHG_TIMER_HOURS = 20.0  # Default 20-hour safety timer

# Humidity alert threshold
HUMIDITY_ALERT_THRESHOLD_PCT = 75.0  # %, triggers warning, makes node unhealthy

# Saltwater alert threshold
SALTWATER_ALERT_THRESHOLD_V = 1.5  # V, triggers warning, makes node unhealthy

# Charging anomaly detection threshold (negative slope while charging)
# Considered significant if voltage is falling faster than this while charging
MIN_DISCHARGING_SLOPE_FOR_ANOMALY_VPH = -0.05  # V/h

try:
    import smbus2 as smbus2
    from smbus2 import i2c_msg
except Exception:
    smbus2 = None
    i2c_msg = None

# GPIO for MP2672GD charger status monitoring
try:
    import gpiod
    _HAS_GPIO = True
except Exception:
    _HAS_GPIO = False
    gpiod = None

# tqdm for terminal bars in debug mode
try:
    from tqdm import tqdm
    _HAS_TQDM = True
except Exception:
    _HAS_TQDM = False

# matplotlib for plotting RC-decay capture (only imported when needed)
_HAS_MPL = False


class BatteryWaterNode(ArgoBaseNode):
    def __init__(self, debug_mode=False):
        super().__init__('battery_water_node')
        self.get_logger().info('=== BATTERY_WATER_NODE STARTUP ===')
        self.get_logger().info('Initializing Battery/Water node...')

        # Debug flag
        self.debug = ('--debug' in sys.argv)
        # ADC electrical test mode (RC decay of REFOUT with sampling & PNG plot)
        self.test_adc = ('--test-adc' in sys.argv)
        self._test_state = 'decay'  # not used in one-shot capture
        self._test_t0 = time.monotonic()

        # Publishers for critical battery/water monitoring
        self.pub_battery_voltage = self.create_publisher(
            Float32, 'battery_voltage', 10)
        self.pub_saltwater_voltage = self.create_publisher(
            Float32, 'saltwater_voltage', 10)
        self.pub_sail_current = self.create_publisher(
            Float32, 'sail_current', 10)
        self.pub_temperature = self.create_publisher(
            Float32, 'temperature_pcb', 10)
        self.pub_humidity = self.create_publisher(
            Float32, 'relative_humidity', 10)
        # Alert publishers for safety-critical alerts
        self.pub_battery_low_alert = self.create_publisher(
            Bool, 'battery_low_alert', 10)
        self.pub_saltwater_alert = self.create_publisher(
            Bool, 'saltwater_alert', 10)
        self.pub_humidity_alert = self.create_publisher(
            Bool, 'humidity_alert', 10)
        
        # Critical I2C failure publisher (for automatic RTH switching)
        self.pub_i2c_failure = self.create_publisher(
            Bool, '/argo/critical/i2c_failure', 10)
        self._i2c_failure_state = False  # Track current I2C failure state

        # Initialize health as unhealthy until we get readings
        self.set_unhealthy("No sensor readings yet")
        # Battery remaining percentage publisher
        self.pub_battery_remaining_pct = self.create_publisher(
            Float32, 'battery_remaining_pct', 10)
        
        # GPIO status publishers for MP2672GD charger monitoring
        self.pub_charging_status = self.create_publisher(
            Bool, 'charging_status', 10)
        self.pub_ac_power_present = self.create_publisher(
            Bool, 'ac_power_present', 10)
        
        # Time-based status tracking for MP2672GD behavior
        # Note: When battery is fully charged, MP2672GD cycles between charging
        # and supplementing modes. We track "recently charging" over a time window.
        self._charging_window_s = 30.0  # Report "charging" if seen in last 30s
        self._ac_power_window_s = 5.0   # Report "AC power" if seen in last 5s (matches publication interval)
        self._last_charging_true_time = 0.0
        self._last_ac_power_true_time = 0.0
        self._charging_conflict_log_interval = 60.0
        self._last_charging_conflict_log_time = 0.0
        
        # GPIO fault detection (for standalone mode - no I2C access)
        # MP2672 datasheet: 1Hz blinking on STAT pin indicates fault conditions:
        # - Battery missing (BATTFLOAT_STAT)
        # - Battery OVP, Timer fault, NTC hot/cold faults, Battery float
        # Track GPIO transitions to detect characteristic blinking pattern
        self._charging_gpio_transitions = deque(maxlen=20)  # Track last 20 transitions for frequency analysis
        self._last_charging_gpio_value = None
        self._last_charging_gpio_change_time = None
        self._charging_fault_detected = False  # True if 1-2 Hz blinking pattern detected
        self._charging_fault_frequency = None  # Calculated frequency if fault detected
        self._charging_fault_log_interval = 30.0  # Log fault status every 30 seconds
        self._last_charging_fault_log_time = 0.0
        self._gpio_ac_power_raw_prev = None  # Track AC transitions for fault-buffer reset
        self._charging_fault_stale_edge_s = 3.0  # No edges this long => pin stable, not fault blinking
        
        # AC power plug-in/plug-out tracking for MP2672 CHG timer monitoring
        # Track when AC power is plugged in to monitor 20-hour safety timer
        self._ac_power_plug_in_time = None  # Monotonic time when AC was plugged in (for calculations)
        self._ac_power_plug_in_time_persistent = None  # Wall clock time when AC was plugged in (for persistence)
        self._ac_power_plug_out_time_persistent = None  # Wall clock time when AC was unplugged (for persistence)
        self._mp2672_chg_timer_hours = MP2672_CHG_TIMER_HOURS  # CHG timer duration (default 20h, can be read from MP2672)
        
        # Battery lifetime estimation publisher
        self.pub_battery_lifetime_hours = self.create_publisher(
            Float32, 'battery_lifetime_hours', 10)

        # Service for on-demand battery status using standard Trigger service
        self.srv_battery_status = self.create_service(
            Trigger, 'battery_status', self.battery_status_callback)
        # Alert previous-state flags for edge-triggered logging
        self._batt_low_prev = False
        self._salt_alert_prev = False
        self._humid_alert_prev = False

        # Latest sensor values for service response (double buffer for consistency)
        # Use NaN to indicate no valid reading yet (more informative than 0.0)
        self._latest_battery_voltage = float('nan')
        self._latest_saltwater_voltage = float('nan')
        self._latest_sail_current = 0.0
        self._latest_temperature = None
        self._latest_humidity = None
        self._latest_battery_remaining_pct = None
        self._latest_battery_low_alert = False
        self._latest_saltwater_alert = False
        self._latest_humidity_alert = False
        self._latest_charging_status = None
        self._latest_ac_power_present = None
        self._latest_usb_pd_negotiated = False  # CH221K !PG pin: True = USB PD negotiation successful, False = no PD (standard USB) or failed
        self._latest_charging_fault_detected = False  # GPIO-based fault detection (standalone mode)
        self._latest_charging_fault_frequency = None  # Frequency of blinking if fault detected
        self._latest_timestamp = None
        # Track data staleness (when using last known values due to I2C failure)
        self._data_stale = False
        # Derived diagnostics
        self._latest_voltage_slope_vph = None
        self._latest_charging_anomaly = False
        self._latest_anomaly_code = None
        self._latest_anomaly_reason = None
        
        # MP2672 register information for service response
        self._mp2672_status_info = None
        self._mp2672_fault_info = None
        self._mp2672_battery_missing_fault = False
        
        # Thread-safe double buffer for service responses
        self._service_buffer = {
            'battery_voltage': float('nan'),  # Use NaN to indicate no valid reading yet
            'saltwater_voltage': float('nan'),  # Use NaN to indicate no valid reading yet
            'sail_current': 0.0,
            'temperature': None,
            'humidity': None,
            'battery_remaining_pct': None,
            'battery_low_alert': False,
            'saltwater_alert': False,
            'humidity_alert': False,
            'charging_status': None,
            'ac_power_present': None,
            'usb_pd_negotiated': False,  # CH221K !PG GPIO (PI4, pin 38): True = USB PD negotiation successful, False = no PD (standard USB) or failed
            'charging_fault_detected': False,  # GPIO-based fault detection: True if 1-2 Hz blinking pattern detected
            'charging_fault_frequency': None,  # Frequency (Hz) of blinking if fault detected
            'time_to_full_hours': None,
            'time_to_empty_hours': None,
            'voltage_slope_vph': None,
            'charging_anomaly': False,
            'anomaly_code': None,
            'anomaly_reason': None,
            # MP2672 charger information (NEW)
            'mp2672_available': False,
            'mp2672_status': None,  # Dict with parsed status register (REG 03H)
            'mp2672_faults': None,  # Dict with parsed fault register (REG 04H)
            'mp2672_battery_missing_fault': False,  # BATTFLOAT_STAT from REG 03H
            # I2C failure and data staleness tracking
            'i2c_failure': False,  # True when critical I2C failure is active
            'stale_data': False,  # True when using last known values due to I2C failure
            # AC power plug-in tracking for MP2672 CHG timer
            'charging_time_remaining_hours': None,  # Remaining time until CHG timer expires (None if AC not present or timer disabled)
            'timestamp': None
        }
        
        # Thread lock for service buffer access
        import threading
        self._buffer_lock = threading.Lock()
        
        # Battery lifetime estimation
        self.battery_lifetime_sample_window = int(
            self.declare_parameter('battery_lifetime_sample_window', BATTERY_LIFETIME_SAMPLE_WINDOW).value)
        self.battery_lifetime_min_samples = int(
            self.declare_parameter('battery_lifetime_min_samples', BATTERY_LIFETIME_MIN_SAMPLES).value)
        
        # Sample history (timestamp, value) for lifetime estimation.
        # Buffers are cleared on AC plug/unplug and idle<->sailing transitions so each
        # regression window covers a single power regime (avoids IR-step artifacts).
        self._voltage_samples = deque(maxlen=self.battery_lifetime_sample_window)
        self._soc_samples = deque(maxlen=self.battery_lifetime_sample_window)
        
        # Power-state / activity classification state
        self._latest_power_state = None    # POWER_STATE_* string
        self._prev_power_state = None
        self._latest_activity_state = None  # 'idle' | 'sailing'
        self._latest_soc_slope_pct_per_h = None  # fresh SOC regression over current buffer
        self._sail_current_ema = None      # slow EMA of |sail current| for activity classification
        self._sail_ema_last_time = None
        
        # Persistent per-regime slopes for early estimates (loaded below)
        self._regime_slopes = {r: dict(v) for r, v in DEFAULT_REGIME_SLOPES.items()}
        self._slopes_dirty = False
        self._last_slope_save_time = 0.0
        self._slopes_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), BATTERY_SLOPES_FILE)
        self._load_battery_slopes()
        
        # Latest lifetime estimates for service response and publishing
        self._latest_time_to_full_hours = None
        self._latest_time_to_empty_hours = None
        self._latest_battery_lifetime_hours = None

        # CSV logging setup
        self.csv_log_dir = "/var/log.hdd/persistent"
        self.csv_log_interval = 30.0  # Log to CSV every 30 seconds
        self._last_csv_log_time = 0.0
        self._csv_file_initialized = False
        self._init_csv_logging()

        # Sensor failure tracking
        self._adc_failure_count = 0
        self._sht_failure_count = 0
        self._sht_sensor_available = True  # Track SHT45 sensor availability
        self._sht_last_error_time = 0.0
        self._sht_error_log_interval = 60.0  # Log SHT45 errors max once per minute
        
        # I2C failure logging throttling
        self._last_i2c_error_log_time = 0.0
        self._last_reinit_error = None  # Store last re-initialization error for logging
        self._i2c_error_log_interval = 30.0  # Log I2C errors max once every 30 seconds
        self._max_failures = 3
        self._shutdown_requested = False
        
        # I2C recovery tracking (similar to imu.py)
        self.node_healthy = True
        self._consecutive_io_errors = 0
        self._last_successful_read_time = time.monotonic()
        self._last_recovery_attempt_time = 0.0
        self._recovery_attempt_count = 0
        
        # Critical I2C failure detection (ADC failure = critical)
        self._critical_i2c_failure = False
        self._adc_failure_timeout = 30.0  # Consider critical after 30s of ADC failures
        self._adc_failure_start_time = None
        # Track last published I2C failure state to prevent topic spam (only publish on state changes)
        self._last_published_i2c_failure_state = None  # None = never published, True/False = last published state
        # Sustained recovery tracking - require multiple consecutive successful reads before declaring recovery
        self._consecutive_successful_reads = 0  # Count of consecutive successful reads
        self._recovery_required_reads = 10  # Require 10 consecutive successful reads (at 1Hz = 10 seconds)
        self._recovery_start_time = None  # Time when recovery sequence started
        
        # I2C failure periodic republishing (using existing sensor read callbacks)
        self._last_i2c_failure_republish_time = 0.0
        self._i2c_failure_republish_interval = 5.0  # Republish every 5 seconds (both True and False states)
        self._initial_republish_done = False  # Track if we've done initial republish for late-joining subscribers

        # Timing and change detection for optimized publishing
        self._startup_time = time.monotonic()
        self._last_publish_time = 0.0
        self._last_log_time = 0.0
        # Previous sensor values for 5% change detection
        self._prev_battery_voltage = None
        self._prev_saltwater_voltage = None
        self._prev_sail_current = None
        self._prev_temperature = None
        self._prev_humidity = None
        self._prev_battery_remaining_pct = None
        self._prev_charging_status = None
        self._prev_ac_power_present = None

        # I2C preferences
        self.use_smbus2 = (smbus2 is not None)
        if self.use_smbus2:
            self.get_logger().info('Using smbus2 for I2C transactions')
        else:
            self.get_logger().info('smbus2 not available; this node requires smbus2')

        # ADC (MAX11612)
        # WARNING: Address 0x34 conflicts with LP5814 broadcast address (0x34)
        # LP5814 broadcast address cannot be disabled and causes ADC to return zeros
        # Solution: Replace MAX11612 with MAX11614 (0x33) or MAX11616 (0x35)
        # See docs/I2C_ADDRESS_CONFLICT_LP5814_MAX11612.md for details
        self.adc_addr = 0x34
        self.vref = 4.096
        self.lsb_value = self.vref / 4096.0
        # Battery divider scaling (updated hardware: 27k/18k -> 2.5x)
        self.battery_divider_scale = 2.5

        # Threshold parameters
        # LiPo 2S ~20% remaining ≈ 3.6 V/cell -> 7.2 V pack (tweak if needed)
        self.batt_low_threshold_v = float(
            self.declare_parameter('battery_low_threshold_v', BATTERY_LOW_THRESHOLD_V).value)
        self.batt_critical_threshold_v = float(
            self.declare_parameter('battery_critical_threshold_v', BATTERY_CRITICAL_THRESHOLD_V).value)
        self.battery_sos_throttle_s = 60.0
        self._last_battery_sos_time = 0.0
        self._abeep_script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'scripts',
            'abeep.sh',
        )
        # Battery low hysteresis (Volts)
        self.batt_low_hysteresis_v = 0.05
        # Saltwater alert threshold (Volts)
        self.saltwater_alert_threshold_v = float(
            self.declare_parameter('saltwater_alert_threshold_v', 1.0).value)
        # Humidity alert threshold (%)
        self.humidity_alert_threshold_pct = float(
            self.declare_parameter('humidity_alert_threshold_pct', 75.0).value)
        # Debug mode - force publishing every cycle for testing
        self.debug_mode = self.declare_parameter('debug_mode', False).value
        # Battery SOC parameters (per-cell formula; defaults from RC LiPo curve)
        self.batt_series_cells = int(
            self.declare_parameter('battery_series_cells', 2).value)
        # Formula: soc% = S - S / (1 + (v / V0)^A)^B
        self.soc_S = float(self.declare_parameter('soc_S', 123.0).value)
        self.soc_V0 = float(self.declare_parameter('soc_V0', 3.7).value)
        self.soc_A = float(self.declare_parameter('soc_A', 80.0).value)
        self.soc_B = float(self.declare_parameter('soc_B', 0.165).value)

        # SHT45
        self.sht_addr = 0x44
        self.SHT45_HIGH_PRECISION_CMD = 0xFD
        self.SHT45_MEASUREMENT_DELAY = 0.01

        # MP2672 I2C status monitoring
        self.mp2672_available = False
        self.mp2672_last_error_time = 0.0
        self.mp2672_error_log_interval = 60.0  # Log errors max once per minute
        
        # Watchdog timer reset tracking (if watchdog is enabled instead of disabled)
        self.mp2672_watchdog_enabled = False  # Track if watchdog is enabled (vs disabled)
        self.mp2672_last_watchdog_reset_time = 0.0
        self.mp2672_watchdog_reset_interval = 30.0  # Reset watchdog every 30s if enabled (timer is 40s default)
        
        # GPIO setup for MP2672GD charger status monitoring
        self.gpio_available = False
        self.charging_gpio_line = None
        self.acok_gpio_line = None
        self.pg_gpio_line = None
        # Note: HOST_CTL_GPIO_LINE (229) is tied high via hardware pullup resistor to VCC
        # MP2672 checks CV pin on power-on only, so hardware pullup ensures host control mode
        # GPIO can be used later to drive CV low to test standalone mode (future feature)
        if _HAS_GPIO:
            try:
                # Initialize GPIO chip
                self.gpio_chip = gpiod.Chip("/dev/gpiochip0")
                # Request charging status GPIO line (PC12, line 76)
                # MP2672 pulls this line LOW when charging, so we need pullup to detect HIGH when not charging
                self.charging_gpio_line = self.gpio_chip.get_line(CHARGING_GPIO_LINE)
                self.charging_gpio_line.request(
                    consumer="battery_water_node", 
                    type=gpiod.LINE_REQ_DIR_IN,
                    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
                )
                # Request AC power status GPIO line (PH9, line 233)
                # MP2672 pulls this line LOW when USB power is present, so we need pullup to detect HIGH when not powered
                self.acok_gpio_line = self.gpio_chip.get_line(ACOK_GPIO_LINE)
                self.acok_gpio_line.request(
                    consumer="battery_water_node", 
                    type=gpiod.LINE_REQ_DIR_IN,
                    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
                )
                # Request USB PD negotiation status GPIO line (PI4, line 260, pin 38)
                # CH221K PG pin (Pin 3) is open-drain output, active low - REQUIRES external pull-up (3.3V) to work correctly
                # Indicates successful USB Power Delivery (PD) negotiation of requested voltage (e.g., 5V, 9V, 12V, 15V, 20V)
                # Active low: CH221K pulls LOW when PD negotiation successful, leaves floating (HIGH via pull-up) when no PD or failed
                # Note: With standard USB cables (non-PD), this will be HIGH - not an indication of power failure
                # May not work consistently on subsequent USB cable plug-ins
                self.pg_gpio_line = self.gpio_chip.get_line(PG_GPIO_LINE)
                self.pg_gpio_line.request(
                    consumer="battery_water_node", 
                    type=gpiod.LINE_REQ_DIR_IN,
                    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP  # REQUIRED for open-drain !PG pin
                )
                self.gpio_available = True
                self.get_logger().info('GPIO setup complete for MP2672GD charger monitoring')
            except Exception as e:
                self.get_logger().error(f'GPIO setup failed: {e}')
                self.gpio_available = False
        else:
            self.get_logger().warning('GPIO not available - MP2672GD charger status monitoring disabled')

        # Perform ADC setup (internal ref always on, AIN3 as analog input)
        try:
            setup_byte = self._build_setup(
                reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(self.adc_addr, setup_byte)
            self.get_logger().info('ADC setup complete (internal ref always on).')
        except Exception as e:
            self.get_logger().error(f'ADC setup failed: {e}')

        # ASCII visual debug (like imu.py/adc.py) when --debug is passed and not in test mode
        self._vis_ascii = self.debug and not self.test_adc
        self._vis_initialized = False
        # Note: ASCII visualization will be initialized after startup messages are complete

        # Dual timers for different sampling requirements
        if self.test_adc:
            # Pause for user confirmation
            try:
                input('Connect a pulldown (e.g., scope probe 1 MΩ to GND) to AIN3, then press Enter to start the ADC RC-decay capture...')
            except Exception:
                pass
            # Perform a single capture: 50 ms REFOUT charge, 3 s decay with 10 ms sampling, then save PNG
            try:
                png_path = self._adc_rc_capture(
                    duration_s=3.0, sample_dt=0.010)
                if png_path:
                    self.get_logger().info(
                        f'RC-decay capture saved: {png_path}')
                else:
                    self.get_logger().warn(
                        'RC-decay capture completed but matplotlib could not be imported to save a plot.')
            except Exception as e:
                self.get_logger().error(f'RC-decay capture failed: {e}')
            # Keep node idle; no periodic publishing in test mode
            self.get_logger().info(
                'Battery/Water node is idle after --test-adc capture. Press Ctrl+C to exit.')
        else:
            # High-frequency timer for sail current (control critical)
            self.sail_current_timer = self.create_timer(
                1.0 / SAIL_CURRENT_RATE_HZ, self.read_sail_current)
            
            # High-frequency timer for GPIO status (1 Hz to catch MP2672GD cycling)
            # GPIO status polling timer (10Hz) - high frequency needed to reliably detect 1-2 Hz blinking fault pattern
            # MP2672 fault indication: 1Hz blinking, but observed at ~2Hz in practice
            # Sampling at 10Hz ensures we catch transitions reliably (Nyquist: need >4Hz for 2Hz signal)
            self.gpio_status_timer = self.create_timer(
                0.1, self.read_gpio_status_only)
            
            # Low-frequency timer for battery safety sensors
            self.battery_safety_timer = self.create_timer(
                BATTERY_SAFETY_INTERVAL_S, self.read_battery_safety_sensors)
            
            self.get_logger().info(
                f'Battery/Water node initialized - Sail current: {SAIL_CURRENT_RATE_HZ}Hz, '
                f'GPIO status: 10Hz, Battery safety: {BATTERY_SAFETY_INTERVAL_S}s intervals')

            # Initial health status will be set after first sensor readings
            
            # Check MP2672 I2C availability during initialization (only if host control enabled)
            if MP2672_HOST_CONTROL:
                self._check_mp2672_availability()
            else:
                self.mp2672_available = False
                self.get_logger().info('MP2672 host control disabled - using standalone mode (GPIO only)')
            
            # Perform initial sensor readings for immediate status display
            self._perform_initial_readings()
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _perform_initial_readings(self):
        """Perform initial sensor readings for immediate status display"""
        self.get_logger().info("Performing initial sensor readings...")
        
        # Use existing sensor reading methods to avoid code duplication
        # Read sail current (high frequency sensor)
        self.read_sail_current()
        
        # Read battery safety sensors (includes ADC, SHT45, GPIO, and lifetime estimation)
        self.read_battery_safety_sensors()
        
        # Publish initial I2C failure state (False if I2C is working, which it should be after successful reads)
        # This ensures late-joining subscribers (like power control) receive the current state
        if self._last_published_i2c_failure_state is None:
            # Never published before - publish current state
            self._publish_i2c_failure(self._critical_i2c_failure, force=True)
            self.get_logger().info(f"Published initial I2C failure state: {self._critical_i2c_failure}")
        
        # Log initial status summary (handle NaN values)
        battery_str = f"{self._latest_battery_voltage:.3f}V" if not math.isnan(self._latest_battery_voltage) else "NaN (I2C failure)"
        saltwater_str = f"{self._latest_saltwater_voltage:.3f}V" if not math.isnan(self._latest_saltwater_voltage) else "NaN"
        self.get_logger().info(
            f"Initial readings complete - Battery={battery_str}, "
            f"Saltwater={saltwater_str}, Sail_current={self._latest_sail_current:.3f}A"
        )
        
        # Initialize service buffer with initial readings
        self._update_service_buffer()
        
        # Note: ASCII visualization will be initialized just before first periodic update

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully and save slopes"""
        self.get_logger().info('=== BATTERY_WATER_NODE SHUTDOWN START ===')
        self.get_logger().info(f"Received signal {signum}, saving slopes and shutting down...")
        
        # Set shutdown flags to stop both timer callbacks and main spin loop
        self._shutdown_requested = True
        self.shutdown_requested = True  # Flag checked by spin_once() loop
        
        # Cancel all timers immediately to prevent blocking I2C operations during shutdown
        try:
            if hasattr(self, 'sail_current_timer') and self.sail_current_timer is not None:
                self.sail_current_timer.cancel()
            if hasattr(self, 'battery_safety_timer') and self.battery_safety_timer is not None:
                self.battery_safety_timer.cancel()
            if hasattr(self, 'gpio_status_timer') and self.gpio_status_timer is not None:
                self.gpio_status_timer.cancel()
            if hasattr(self, 'retry_timer') and self.retry_timer is not None:
                self.retry_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Error cancelling timers during shutdown: {e}")
        
        # Save slopes
        try:
            self._save_battery_slopes()
        except Exception as e:
            self.get_logger().error(f"Error saving slopes during shutdown: {e}")
        
        # Shutdown ROS2 context - this will cause spin_once() loop to exit
        # The spin_once() loop checks both rclpy.ok() and shutdown_requested flag
        try:
            if rclpy.ok():
                rclpy.shutdown()
                self.get_logger().info('=== BATTERY_WATER_NODE SHUTDOWN COMPLETE ===')
        except Exception as e:
            # Ignore errors - context may already be shutting down
            print(f"Warning: Error during ROS2 shutdown: {e}")
    
    def _update_service_buffer(self):
        """Atomically update the service buffer with latest complete sensor values"""
        with self._buffer_lock:
            self._service_buffer.update({
                'battery_voltage': self._latest_battery_voltage,
                'saltwater_voltage': self._latest_saltwater_voltage,
                'sail_current': self._latest_sail_current,
                'temperature': self._latest_temperature,
                'humidity': self._latest_humidity,
                'battery_remaining_pct': self._latest_battery_remaining_pct,
                'battery_low_alert': self._latest_battery_low_alert,
                'saltwater_alert': self._latest_saltwater_alert,
                'humidity_alert': self._latest_humidity_alert,
                'charging_status': self._latest_charging_status,
                'ac_power_present': self._latest_ac_power_present,
                'usb_pd_negotiated': self._latest_usb_pd_negotiated,
                'charging_fault_detected': self._charging_fault_detected,
                'charging_fault_frequency': self._charging_fault_frequency,
                'time_to_full_hours': self._latest_time_to_full_hours,
                'time_to_empty_hours': self._latest_time_to_empty_hours,
                'voltage_slope_vph': self._latest_voltage_slope_vph,
                'charging_anomaly': self._latest_charging_anomaly,
                'anomaly_code': self._latest_anomaly_code,
                'anomaly_reason': self._latest_anomaly_reason,
                # MP2672 charger information
                'mp2672_available': self.mp2672_available,
                'mp2672_status': getattr(self, '_mp2672_status_info', None),
                'mp2672_faults': getattr(self, '_mp2672_fault_info', None),
                'mp2672_battery_missing_fault': getattr(self, '_mp2672_battery_missing_fault', False),
                # I2C failure and data staleness tracking
                'i2c_failure': self._critical_i2c_failure,
                'stale_data': self._data_stale,
                # AC power plug-in tracking for MP2672 CHG timer
                'charging_time_remaining_hours': self._calculate_charging_time_remaining(),
                'battery_water_health': self.health_status,  # Include health status in buffer
                'timestamp': time.monotonic()
            })

    def _update_battery_water_health(self, battery_voltage: float, saltwater_detected: bool, humidity: float):
        """Update health status based on specific criteria:
        - Battery voltage above low threshold
        - No saltwater intrusion detected
        - Humidity below 70%
        """
        if battery_voltage is None or math.isnan(battery_voltage):
            self.set_unhealthy("No battery voltage reading (I2C failure)")
            return
        
        if saltwater_detected is None:
            self.set_unhealthy("No saltwater detection reading")
            return
            
        if humidity is None:
            self.set_unhealthy("No humidity reading")
            return
        
        # Check battery voltage (above low threshold)
        if battery_voltage <= BATTERY_LOW_THRESHOLD_V:
            self.set_unhealthy(f"Battery voltage too low: {battery_voltage:.2f}V <= {BATTERY_LOW_THRESHOLD_V:.2f}V")
            return
        
        # Check saltwater intrusion
        if saltwater_detected:
            self.set_unhealthy("Saltwater intrusion detected")
            return
        
        # Check humidity (below threshold)
        if humidity >= HUMIDITY_ALERT_THRESHOLD_PCT:
            self.set_unhealthy(f"Humidity too high: {humidity:.1f}% >= {HUMIDITY_ALERT_THRESHOLD_PCT:.1f}%")
            return
        
        # All criteria met
        # Use generic details string to avoid logging on every update (actual values in combined sensor states log)
        # The base class only logs when health_status or details change, so using a static string prevents spam
        self.set_healthy("Battery OK, No saltwater, Humidity OK")

    def _cleanup_on_exit(self):
        """Battery/Water node specific cleanup on exit"""
        self.get_logger().info('=== BATTERY_WATER_NODE CLEANUP ===')
        # Cancel any remaining timers
        if hasattr(self, 'sail_current_timer'):
            try:
                self.sail_current_timer.cancel()
            except Exception:
                pass
        if hasattr(self, 'battery_safety_timer'):
            try:
                self.battery_safety_timer.cancel()
            except Exception:
                pass

    def _init_csv_logging(self):
        """Initialize CSV logging directory and file"""
        try:
            # Create log directory if it doesn't exist
            os.makedirs(self.csv_log_dir, exist_ok=True)

            # Create CSV filename with current date
            date_str = datetime.now().strftime('%Y%m%d')
            csv_filename = f"battery-monitor-{date_str}.csv"
            self.csv_file_path = os.path.join(self.csv_log_dir, csv_filename)

            # Initialize CSV file with headers if it doesn't exist
            if not os.path.exists(self.csv_file_path):
                with open(self.csv_file_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([
                        'timestamp', 'battery_voltage', 'battery_remaining_pct',
                        'saltwater_voltage', 'sail_current', 'pcb_temperature',
                        'relative_humidity', 'battery_low_alert', 'saltwater_alert',
                        'humidity_alert', 'battery_water_health', 'charging_status', 'ac_power_present'
                    ])
                self.get_logger().info(
                    f"CSV logging initialized: {self.csv_file_path}")

            self._csv_file_initialized = True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV logging: {e}")
            self._csv_file_initialized = False

    def _log_to_csv(self, battery_voltage, battery_remaining_pct, saltwater_voltage,
                    sail_current, temperature, humidity, battery_low_alert,
                    saltwater_alert, humidity_alert, health_status, charging_status,
                    ac_power_present, adc_stale=False):
        """Log current sensor data to CSV file
        
        Args:
            adc_stale: If True, marks this entry as having stale ADC data (I2C failure)
        """
        if not self._csv_file_initialized:
            return

        try:
            current_time = datetime.now()
            timestamp = current_time.strftime('%Y-%m-%d %H:%M:%S')

            # Convert boolean values to 0/1 for CSV
            battery_low_csv = 1 if battery_low_alert else 0
            saltwater_alert_csv = 1 if saltwater_alert else 0
            humidity_alert_csv = 1 if humidity_alert else 0
            health_csv = 1 if health_status else 0
            charging_csv = 1 if charging_status else 0
            ac_power_csv = 1 if ac_power_present else 0

            # Handle NaN and None values - mark as "FAILED" if ADC is stale and value is NaN/None/0
            if adc_stale and (math.isnan(battery_voltage) or battery_voltage == 0.0):
                battery_voltage_str = "FAILED"
            else:
                battery_voltage_str = battery_voltage if not math.isnan(battery_voltage) else "NaN"
                
            battery_remaining_pct = battery_remaining_pct if battery_remaining_pct is not None else ("FAILED" if adc_stale else "")
            temperature = temperature if temperature is not None else ""
            humidity = humidity if humidity is not None else ""
            
            # Mark saltwater as FAILED if ADC is stale and value is NaN/None/0
            if adc_stale and (math.isnan(saltwater_voltage) or saltwater_voltage == 0.0):
                saltwater_voltage_str = "FAILED"
            else:
                saltwater_voltage_str = saltwater_voltage if not math.isnan(saltwater_voltage) else "NaN"

            # Write CSV row
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    timestamp, battery_voltage_str, battery_remaining_pct,
                    saltwater_voltage_str, sail_current, temperature,
                    humidity, battery_low_csv, saltwater_alert_csv,
                    humidity_alert_csv, health_csv, charging_csv, ac_power_csv
                ])

        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {e}")

    def battery_status_callback(self, request, response):
        """Service callback to provide latest battery and sensor status as JSON string
        
        IMPORTANT: This callback is NON-BLOCKING - it only reads from the thread-safe buffer.
        It never performs I2C operations, so it will always respond quickly even during I2C failures.
        The buffer is updated asynchronously by timer callbacks running in separate threads.
        """
        try:
            # Get current timestamp
            now = self.get_clock().now()
            
            # Use thread-safe double buffer to ensure consistent state
            # Lock is held only for dictionary copy operation (microseconds) - never during I2C operations
            with self._buffer_lock:
                buffer_copy = self._service_buffer.copy()

            # Build battery data dictionary from consistent buffer (alphabetically ordered for readability)
            battery_data = {
                'ac_power_present': buffer_copy['ac_power_present'] if buffer_copy['ac_power_present'] is not None else False,
                'anomaly_code': buffer_copy.get('anomaly_code'),
                'anomaly_reason': buffer_copy.get('anomaly_reason'),
                'battery_low_alert': buffer_copy['battery_low_alert'],
                'battery_remaining_pct': buffer_copy['battery_remaining_pct'] if buffer_copy['battery_remaining_pct'] is not None else 0.0,
                'battery_storage_rundown': os.path.exists(STORAGE_RUNDOWN_FLAG_FILE),
                'battery_voltage': buffer_copy['battery_voltage'],
                'battery_water_health': buffer_copy.get('battery_water_health', 'UNKNOWN'),
                'charging_anomaly': buffer_copy.get('charging_anomaly', False),
                'charging_fault_detected': buffer_copy.get('charging_fault_detected', False),
                'charging_fault_frequency': buffer_copy.get('charging_fault_frequency'),  # None if no fault or not detected
                'charging_status': buffer_copy['charging_status'] if buffer_copy['charging_status'] is not None else False,
                'charging_time_remaining_hours': buffer_copy.get('charging_time_remaining_hours'),
                'humidity_alert': buffer_copy['humidity_alert'],
                'i2c_failure': buffer_copy.get('i2c_failure', False),
                'mp2672_available': buffer_copy.get('mp2672_available', False),
                'mp2672_battery_missing_fault': buffer_copy.get('mp2672_battery_missing_fault', False),
                'mp2672_faults': buffer_copy.get('mp2672_faults'),
                'mp2672_status': buffer_copy.get('mp2672_status'),
                'pcb_temperature': buffer_copy['temperature'] if buffer_copy['temperature'] is not None else 0.0,
                'relative_humidity': buffer_copy['humidity'] if buffer_copy['humidity'] is not None else 0.0,
                'sail_current': buffer_copy['sail_current'],
                'saltwater_alert': buffer_copy['saltwater_alert'],
                'saltwater_voltage': buffer_copy['saltwater_voltage'],
                'stale_data': buffer_copy.get('stale_data', False),
                'timestamp_nanosec': now.seconds_nanoseconds()[1],
                'timestamp_sec': now.seconds_nanoseconds()[0],
                'time_to_empty_hours': buffer_copy['time_to_empty_hours'],
                'time_to_full_hours': buffer_copy['time_to_full_hours'],
                'usb_pd_negotiated': buffer_copy.get('usb_pd_negotiated') if buffer_copy.get('usb_pd_negotiated') is not None else False,  # False if not available or standard USB (non-PD)
                'voltage_slope_vph': buffer_copy.get('voltage_slope_vph')
            }
            
            # Determine MP2672 fault condition summary for alerts
            # In standalone mode (no I2C), use GPIO-based fault detection
            # In host control mode (I2C available), use I2C fault register
            mp2672_fault_summary = None
            active_faults = []
            
            # GPIO-based fault detection (standalone mode - no I2C access)
            if battery_data.get('charging_fault_detected') is True:
                freq = battery_data.get('charging_fault_frequency')
                if freq is not None:
                    active_faults.append(f'{freq:.2f}Hz blinking (battery missing/OVP/timer/NTC fault)')
                else:
                    active_faults.append('Blinking pattern detected (battery missing/OVP/timer/NTC fault)')
            
            # I2C-based fault detection (host control mode - I2C available)
            if battery_data.get('mp2672_available') and battery_data.get('mp2672_faults'):
                faults_dict = battery_data['mp2672_faults']
                
                if faults_dict.get('wd_fault'):
                    active_faults.append('Watchdog timer expiration')
                if faults_dict.get('input_fault'):
                    active_faults.append('Input OVP fault')
                if faults_dict.get('therm_sd_fault'):
                    active_faults.append('Thermal shutdown')
                if faults_dict.get('timer_fault'):
                    active_faults.append('Safety timer expiration')
                if faults_dict.get('bat_fault'):
                    active_faults.append('Battery OVP fault')
                if faults_dict.get('ntc_fault_str') and faults_dict.get('ntc_fault_str') != 'Normal':
                    active_faults.append(f"NTC {faults_dict['ntc_fault_str']}")
                if battery_data.get('mp2672_battery_missing_fault'):
                    active_faults.append('Battery missing or balance cable fault')
            
            if active_faults:
                mp2672_fault_summary = ' | '.join(active_faults)
            else:
                mp2672_fault_summary = None
            
            # Insert mp2672_fault_summary in alphabetical position (after mp2672_faults, before mp2672_status)
            # Create new dict with all items in alphabetical order
            temp_dict = dict(battery_data)
            temp_dict['mp2672_fault_summary'] = mp2672_fault_summary
            # Rebuild dict in alphabetical order
            battery_data = dict(sorted(temp_dict.items()))

            # Format battery summary
            battery_summary = None
            voltage = battery_data['battery_voltage']
            percent = battery_data['battery_remaining_pct']

            # Handle NaN values in voltage (I2C failure)
            if voltage is not None and not math.isnan(voltage) and percent is not None:
                battery_summary = f"{voltage:.1f}V ({percent:.0f}%)"
            elif voltage is not None and not math.isnan(voltage):
                battery_summary = f"{voltage:.1f}V"
            elif percent is not None:
                battery_summary = f"{percent:.0f}%"
            elif voltage is not None and math.isnan(voltage):
                battery_summary = "NaN (I2C failure)"

            # Format critical alerts
            active_alerts = []
            alert_descriptions = {
                'battery_low_alert': '🔋 LOW BATTERY',
                'saltwater_alert': '💧 SALTWATER INTRUSION',
                'humidity_alert': '💦 HIGH HUMIDITY'
            }

            for alert_key, description in alert_descriptions.items():
                if battery_data.get(alert_key) is True:
                    active_alerts.append(description)

            # Charging anomaly alert (voltage falling while charging reported)
            if battery_data.get('charging_anomaly') is True:
                active_alerts.append('⚠️ CHARGER POWER FAULT')
            
            # GPIO-based charging fault detection (standalone mode - no I2C access)
            # MP2672 datasheet: 1Hz blinking on STAT pin indicates fault conditions
            if battery_data.get('charging_fault_detected') is True:
                freq = battery_data.get('charging_fault_frequency')
                if freq is not None:
                    active_alerts.append(f'🔴 CHARGING FAULT: {freq:.2f}Hz blinking detected (battery missing/OVP/timer/NTC fault)')
                else:
                    active_alerts.append('🔴 CHARGING FAULT: Blinking pattern detected (battery missing/OVP/timer/NTC fault)')
            
            # MP2672 fault alerts (from fault register - only when USB power present and I2C available)
            if battery_data.get('mp2672_fault_summary'):
                active_alerts.append(f'⚠️ MP2672: {battery_data["mp2672_fault_summary"]}')
            
            # MP2672 battery missing/balance cable fault (from status register - only when I2C available)
            if battery_data.get('mp2672_battery_missing_fault'):
                active_alerts.append('⚠️ BATTERY MISSING OR BALANCE CABLE FAULT')
            
            # I2C failure alert (critical - battery monitoring unavailable)
            if battery_data.get('i2c_failure'):
                active_alerts.append('🔴 CRITICAL I2C FAILURE - Battery monitoring unavailable')
            
            # Stale data warning (data may be outdated due to I2C failure)
            if battery_data.get('stale_data'):
                active_alerts.append('⚠️ STALE DATA - Using last known values (I2C failure)')

            critical_alerts = " | ".join(
                active_alerts) if active_alerts else None

            # Create final response data
            response_data = {
                'battery_summary': battery_summary,
                'critical_alerts': critical_alerts,
                'raw_data': battery_data
            }

            # Convert to JSON string and return in Trigger response
            # Use compact JSON (no indent) for faster serialization
            response.success = True
            response.message = json.dumps(response_data, separators=(',', ':'))

            return response

        except Exception as e:
            self.get_logger().error(f"Error in battery status service: {e}", exc_info=True)
            # Always return a response, even on error - don't let service hang
            response.success = False
            response.message = json.dumps({'error': str(e), 'i2c_failure': self._critical_i2c_failure, 'stale_data': self._data_stale})
            return response
    
    def _load_battery_slopes(self):
        """Load persistent per-regime battery slopes and AC power plug-in time from file.
        
        Supports legacy schema (charging_slope_v_per_s / discharging_slope_v_per_s) by
        mapping it onto the regime store. Values outside their regime sanity band are
        ignored in favor of defaults.
        """
        try:
            if not os.path.exists(self._slopes_file_path):
                return
            with open(self._slopes_file_path, 'r') as f:
                slopes_data = json.load(f)
            
            def accept(regime, vph, soc_pct_per_h=None):
                lo, hi = REGIME_SLOPE_BANDS_VPH[regime]
                if vph is not None and lo <= vph <= hi:
                    self._regime_slopes[regime]['vph'] = float(vph)
                    if soc_pct_per_h is not None:
                        self._regime_slopes[regime]['soc_pct_per_h'] = float(soc_pct_per_h)
            
            regimes = slopes_data.get('regimes')
            if isinstance(regimes, dict):
                for regime, entry in regimes.items():
                    if regime in self._regime_slopes and isinstance(entry, dict):
                        accept(regime, entry.get('vph'), entry.get('soc_pct_per_h'))
            else:
                # Legacy schema: map old single charge/discharge slopes (V/s)
                chg = slopes_data.get('charging_slope_v_per_s')
                dch = slopes_data.get('discharging_slope_v_per_s')
                if chg is not None:
                    accept(REGIME_CHARGING, chg * 3600.0)
                if dch is not None:
                    accept(REGIME_DISCHARGE_IDLE, dch * 3600.0)
                self.get_logger().info("Migrated legacy battery slopes file to regime schema")
            
            # Load AC power plug-in time (for CHG timer persistence across reboots)
            ac_plug_in_iso = slopes_data.get('ac_power_plug_in_time')
            if ac_plug_in_iso:
                try:
                    plug_in_dt = datetime.fromisoformat(ac_plug_in_iso)
                    self._ac_power_plug_in_time_persistent = plug_in_dt.timestamp()
                    self.get_logger().info(f"Loaded AC power plug-in time: {ac_plug_in_iso}")
                except Exception as e:
                    self.get_logger().warning(f"Failed to parse AC power plug-in time: {e}")
            
            self.get_logger().info(
                "Loaded battery regime slopes: " + self._format_regime_slopes())
        except Exception as e:
            self.get_logger().warning(f"Failed to load battery slopes: {e}")
    
    def _format_regime_slopes(self) -> str:
        """Human-readable one-line summary of regime slopes in V/h."""
        parts = []
        for regime, label in ((REGIME_DISCHARGE_IDLE, 'idle'),
                              (REGIME_DISCHARGE_SAILING, 'sailing'),
                              (REGIME_PLUGGED_NET, 'plugged'),
                              (REGIME_CHARGING, 'charging')):
            vph = self._regime_slopes[regime].get('vph')
            soc = self._regime_slopes[regime].get('soc_pct_per_h')
            if vph is not None:
                soc_str = f"/{soc:+.1f}%SOC/h" if soc is not None else ""
                parts.append(f"{label}={vph:+.3f}V/h{soc_str}")
            else:
                parts.append(f"{label}=n/a")
        return ", ".join(parts)
    
    def _save_battery_slopes(self):
        """Persist per-regime slopes and AC power plug-in time (schema 2).
        
        Writes only when a regime slope changed (_slopes_dirty), the AC plug-in
        time changed (required for MP2672 CHG timer tracking across reboots), or
        the file doesn't exist / uses the legacy schema.
        """
        try:
            existing = {}
            if os.path.exists(self._slopes_file_path):
                try:
                    with open(self._slopes_file_path, 'r') as f:
                        existing = json.load(f)
                except Exception:
                    existing = {}
            
            ac_plug_in_iso = None
            if self._ac_power_plug_in_time_persistent is not None:
                try:
                    ac_plug_in_iso = datetime.fromtimestamp(
                        self._ac_power_plug_in_time_persistent).isoformat()
                except Exception:
                    pass
            ac_changed = (ac_plug_in_iso or None) != (existing.get('ac_power_plug_in_time') or None)
            needs_migration = existing.get('schema') != 2
            
            if not self._slopes_dirty and not ac_changed and not needs_migration:
                return
            
            json_data = {
                "schema": 2,
                "regimes": {r: dict(e) for r, e in self._regime_slopes.items()},
                "timestamp": datetime.now().isoformat(),
                "sample_count": len(self._voltage_samples),
            }
            if ac_plug_in_iso:
                # Top-level key consumed by power_control/argo_power_control.py
                json_data["ac_power_plug_in_time"] = ac_plug_in_iso
            
            with open(self._slopes_file_path, 'w') as f:
                json.dump(json_data, f, indent=2)
            
            self._slopes_dirty = False
            self._last_slope_save_time = time.monotonic()
            self.get_logger().info(
                f"Saved battery regime slopes ({len(self._voltage_samples)} samples): "
                + self._format_regime_slopes())
        except Exception as e:
            self.get_logger().error(f"Failed to save battery slopes: {e}")
    
    def _update_regime_slope(self, regime: str, vph: float, soc_pct_per_h: float):
        """Update a persisted regime slope from a fresh in-band regression.
        
        Saves at most once per minute to limit flash writes (shutdown handler
        flushes any pending change).
        """
        entry = self._regime_slopes[regime]
        old_vph = entry.get('vph')
        # Skip churn: ignore changes within 2% relative or 0.002 V/h absolute
        if old_vph is not None and abs(vph - old_vph) <= max(0.002, abs(old_vph) * 0.02):
            return
        entry['vph'] = round(float(vph), 4)
        entry['soc_pct_per_h'] = round(float(soc_pct_per_h), 3)
        self._slopes_dirty = True
        if time.monotonic() - self._last_slope_save_time >= 60.0:
            self._save_battery_slopes()
    
    def _linear_least_squares(self, samples: deque) -> Optional[Tuple[float, float]]:
        """
        Compute linear least squares fit for (timestamp, value) samples.
        Used for both voltage (V) and SOC (%) series.
        
        Args:
            samples: deque of (timestamp, value) tuples
        
        Returns:
            Tuple of (slope, intercept) in value/s and value, or None if insufficient samples
        """
        if len(samples) < self.battery_lifetime_min_samples:
            return None
        
        try:
            # Filter out NaN values from samples (I2C failures)
            valid_samples = [(t, v) for t, v in samples if not math.isnan(v)]
            if len(valid_samples) < self.battery_lifetime_min_samples:
                return None  # Not enough valid samples
            
            # Extract timestamps and voltages from valid samples
            n = len(valid_samples)
            t0 = valid_samples[0][0]  # Reference time for numerical stability
            times = [t - t0 for t, v in valid_samples]
            voltages = [v for t, v in valid_samples]
            
            # Compute sums for least squares
            sum_t = sum(times)
            sum_v = sum(voltages)
            sum_tv = sum(t * v for t, v in zip(times, voltages))
            sum_t2 = sum(t * t for t in times)
            
            # Compute slope and intercept
            # slope = (n * sum_tv - sum_t * sum_v) / (n * sum_t2 - sum_t * sum_t)
            denominator = n * sum_t2 - sum_t * sum_t
            if abs(denominator) < 1e-9:
                return None  # Avoid division by zero
            
            slope = (n * sum_tv - sum_t * sum_v) / denominator
            intercept = (sum_v - slope * sum_t) / n
            
            return (slope, intercept)
        
        except Exception as e:
            self.get_logger().error(f"Linear least squares fit failed: {e}")
            return None

    def _discharge_target_voltage_v(self) -> float:
        """Voltage target for time-to-empty (critical unless storage rundown)."""
        if os.path.exists(STORAGE_RUNDOWN_FLAG_FILE):
            return STORAGE_VOLTAGE_V
        return BATTERY_CRITICAL_THRESHOLD_V

    def _soc_from_voltage(self, voltage: float) -> Optional[float]:
        """Battery state-of-charge (%) from pack voltage via per-cell LiPo curve."""
        try:
            if voltage is None or math.isnan(voltage) or voltage <= 0:
                return None
            cells = max(1, int(self.batt_series_cells))
            v_cell = voltage / float(cells)
            base = 1.0 + (max(0.0, v_cell) / max(1e-9, self.soc_V0)) ** self.soc_A
            soc = self.soc_S - (self.soc_S / (base ** self.soc_B))
            return float(max(0.0, min(100.0, soc)))
        except Exception:
            return None

    def _classify_activity(self) -> str:
        """'sailing' when the slow EMA of |sail current| shows sustained winch activity."""
        if (self._sail_current_ema is not None and
                self._sail_current_ema > SAILING_SAIL_CURRENT_THRESHOLD_A):
            return 'sailing'
        return 'idle'

    def _classify_power_state(self, ac_power_present: Optional[bool]) -> str:
        """Classify power state. The measured voltage trend is authoritative;
        charger GPIO/I2C status is only a hint.
        
        - AC absent (or unknown): discharging (safe assumption)
        - AC present + charger fault blinking: plugged_drain (charger not delivering)
        - AC present + sustained positive trend: plugged_charge
        - AC present otherwise: plugged_drain (charger insufficient for load)
        """
        if ac_power_present is not True:
            return POWER_STATE_DISCHARGING
        if self._charging_fault_detected:
            return POWER_STATE_PLUGGED_DRAIN
        if (self._latest_voltage_slope_vph is not None and
                len(self._voltage_samples) >= self.battery_lifetime_min_samples and
                self._latest_voltage_slope_vph >= PLUGGED_CHARGE_MIN_SLOPE_VPH):
            return POWER_STATE_PLUGGED_CHARGE
        return POWER_STATE_PLUGGED_DRAIN

    def _regime_for_state(self, power_state: str, activity: str) -> str:
        """Map (power state, activity) to a slope-learning regime."""
        if power_state == POWER_STATE_PLUGGED_CHARGE:
            return REGIME_CHARGING
        if power_state == POWER_STATE_PLUGGED_DRAIN:
            return REGIME_PLUGGED_NET
        return REGIME_DISCHARGE_SAILING if activity == 'sailing' else REGIME_DISCHARGE_IDLE

    def _compute_lifetime_estimates(self, voltage: float, soc: Optional[float]) -> Tuple[Optional[float], Optional[float]]:
        """
        SOC-based lifetime estimates for the current power state.
        
        Extrapolates SOC %/h to a target SOC (linearizes the flat LiPo mid-range
        that distorts V/h extrapolation). Fresh in-regime regression is preferred;
        persisted regime slopes provide estimates right after startup/transitions.
        
        Returns:
            (time_to_full_hours, time_to_empty_hours) - exactly one is set, or
            (None, None) when no meaningful estimate is possible.
        """
        try:
            if soc is None or math.isnan(voltage):
                return None, None
            
            state = self._latest_power_state
            activity = self._latest_activity_state or 'idle'
            regime = self._regime_for_state(state, activity)
            
            # Fresh regressions over the current (single-regime) buffer
            v_fit = self._linear_least_squares(self._voltage_samples)
            soc_fit = self._linear_least_squares(self._soc_samples)
            fresh_vph = v_fit[0] * 3600.0 if v_fit is not None else None
            fresh_soc_per_h = soc_fit[0] * 3600.0 if soc_fit is not None else None
            self._latest_soc_slope_pct_per_h = fresh_soc_per_h
            
            # Learn: persist fresh slope when it falls inside its regime sanity band
            if fresh_vph is not None and fresh_soc_per_h is not None:
                band_lo, band_hi = REGIME_SLOPE_BANDS_VPH[regime]
                if band_lo <= fresh_vph <= band_hi:
                    self._update_regime_slope(regime, fresh_vph, fresh_soc_per_h)
            
            # Choose slope for the estimate: fresh if meaningful and correct sign,
            # else persisted regime slope (also sign-checked)
            want_positive = (state == POWER_STATE_PLUGGED_CHARGE)
            slope_soc_per_h = None
            if (fresh_soc_per_h is not None and
                    abs(fresh_soc_per_h) >= MIN_MEANINGFUL_SOC_SLOPE_PCT_PER_H and
                    (fresh_soc_per_h > 0) == want_positive):
                slope_soc_per_h = fresh_soc_per_h
            else:
                persisted = self._regime_slopes[regime].get('soc_pct_per_h')
                if persisted is not None and (persisted > 0) == want_positive:
                    slope_soc_per_h = persisted
            
            if state == POWER_STATE_PLUGGED_CHARGE:
                if slope_soc_per_h is None or slope_soc_per_h <= 0:
                    return None, None
                target_soc = self._soc_from_voltage(BATTERY_FULLY_CHARGED_THRESHOLD_V) or 91.0
                if soc >= target_soc:
                    return 0.0, None
                ttf = (target_soc - soc) / slope_soc_per_h
                return (ttf if 0 <= ttf <= 1000 else None), None
            
            # Discharging (on battery, or plugged in but net drain)
            eff_slope = slope_soc_per_h
            if state == POWER_STATE_DISCHARGING and activity == 'sailing':
                # Conservative on-water floor: never assume slower than this while sailing
                floor = -SAILING_TTE_FLOOR_SOC_PCT_PER_H
                eff_slope = floor if eff_slope is None else min(eff_slope, floor)
            if eff_slope is None or eff_slope >= 0:
                return None, None
            target_soc = self._soc_from_voltage(self._discharge_target_voltage_v()) or 2.0
            if soc <= target_soc:
                return None, 0.0
            tte = (soc - target_soc) / abs(eff_slope)
            return None, (tte if 0 <= tte <= 1000 else None)
        
        except Exception as e:
            self.get_logger().error(f"Battery lifetime estimation failed: {e}")
            return None, None

    # ---------- I2C helpers ----------
    def _i2c_write(self, addr: int, byte_val: int) -> None:
        # Skip I2C operations during shutdown to prevent blocking
        if self._shutdown_requested:
            raise RuntimeError('I2C operation skipped - shutdown in progress')
        if self.use_smbus2 and i2c_msg is not None:
            msg = i2c_msg.write(addr, bytes([byte_val & 0xFF]))
            with smbus2.SMBus(I2C_BUS_NUMBER) as b:
                b.i2c_rdwr(msg)
        else:
            raise RuntimeError('smbus2 required for this node')

    def _i2c_read(self, addr: int, n: int) -> list:
        # Skip I2C operations during shutdown to prevent blocking
        if self._shutdown_requested:
            raise RuntimeError('I2C operation skipped - shutdown in progress')
        if self.use_smbus2 and i2c_msg is not None:
            r = i2c_msg.read(addr, n)
            with smbus2.SMBus(I2C_BUS_NUMBER) as b:
                b.i2c_rdwr(r)
            return list(r)
        else:
            raise RuntimeError('smbus2 required for this node')

    def _i2c_write_read(self, addr: int, write_byte: int, n: int) -> list:
        # Skip I2C operations during shutdown to prevent blocking
        if self._shutdown_requested:
            raise RuntimeError('I2C operation skipped - shutdown in progress')
        if self.use_smbus2 and i2c_msg is not None:
            w = i2c_msg.write(addr, bytes([write_byte & 0xFF]))
            r = i2c_msg.read(addr, n)
            with smbus2.SMBus(I2C_BUS_NUMBER) as b:
                b.i2c_rdwr(w, r)
            return list(r)
        else:
            raise RuntimeError('smbus2 required for this node')

    # ---------- MAX11612 helpers ----------
    def _build_setup(self, reg, sel, clk, bip_uni, rst, x):
        return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)

    def _build_config(self, reg, scan, cs, sgl_dif):
        return ((reg & 1) << 7) | ((scan & 0b11) << 5) | ((cs & 0b1111) << 1) | (sgl_dif & 1)

    def _read_adc_channel_avg(self, ch: int, repeats: int = 8, settle_delay_s: float = 0.010) -> int:
        # SCAN=01: convert selected input eight times
        # CRITICAL: Use separate write-then-read transactions (like SHT45) with explicit delay
        # This matches the SHT45 pattern which works reliably with LP5814 on the bus
        # Increased default delay from 1ms to 10ms to handle bus capacitance/timing issues
        # when LP5814 is on the bus - gives ADC more time to complete conversion
        cfg = self._build_config(reg=0, scan=0b01, cs=ch, sgl_dif=1)
        acc = 0
        for _ in range(repeats):
            # Write config byte (triggers conversion) - separate transaction
            self._i2c_write(self.adc_addr, cfg)
            # Explicit delay to allow conversion to complete (increased for LP5814 compatibility)
            # SHT45 uses 15ms delay - using 10ms as compromise between speed and reliability
            time.sleep(settle_delay_s)
            # Read result - separate transaction
            d = self._i2c_read(self.adc_addr, 2)
            code = ((d[0] & 0x0F) << 8) | d[1]
            acc += code
        return acc // repeats

    # ---------- SHT45 helpers ----------
    def _sht_crc(self, data):
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
        return crc & 0xFF

    def _read_sht45(self):
        # Skip I2C operations during shutdown to prevent blocking
        if self._shutdown_requested:
            raise RuntimeError('SHT45 read skipped - shutdown in progress')
        # pure write command
        w = i2c_msg.write(self.sht_addr, [self.SHT45_HIGH_PRECISION_CMD])
        with smbus2.SMBus(I2C_BUS_NUMBER) as b:
            b.i2c_rdwr(w)
        time.sleep(self.SHT45_MEASUREMENT_DELAY)
        # Check shutdown again before read (measurement delay may have allowed shutdown)
        if self._shutdown_requested:
            raise RuntimeError('SHT45 read skipped - shutdown in progress')
        # pure read of 6 bytes
        r = i2c_msg.read(self.sht_addr, 6)
        with smbus2.SMBus(I2C_BUS_NUMBER) as b:
            b.i2c_rdwr(r)
        data = list(r)
        temp_data = data[0:2]
        temp_crc = data[2]
        humid_data = data[3:5]
        humid_crc = data[5]
        if self._sht_crc(temp_data) != temp_crc or self._sht_crc(humid_data) != humid_crc:
            raise RuntimeError('SHT45 CRC mismatch')
        raw_temp = (temp_data[0] << 8) | temp_data[1]
        raw_hum = (humid_data[0] << 8) | humid_data[1]
        # Convert per datasheet
        temperature = -45.0 + 175.0 * (raw_temp / 65535.0)
        humidity = -6.0 + 125.0 * (raw_hum / 65535.0)
        humidity = max(0.0, min(100.0, humidity))
        return temperature, humidity

    def _read_sht45_robust(self):
        """
        Read SHT45 sensor with graceful failure handling.
        Returns (temperature, humidity) or (None, None) if sensor unavailable.
        """
        if not self._sht_sensor_available:
            return None, None
            
        try:
            temperature, humidity = self._read_sht45()
            # Reset failure count on successful read
            self._sht_failure_count = 0
            return temperature, humidity
            
        except Exception as e:
            self._sht_failure_count += 1
            current_time = time.monotonic()
            
            # Log error with throttling (max once per minute)
            if current_time - self._sht_last_error_time >= self._sht_error_log_interval:
                self.get_logger().warn(
                    f"SHT45 sensor error (failure {self._sht_failure_count}): {e}")
                self._sht_last_error_time = current_time
            
            # Mark sensor as unavailable after multiple failures
            if self._sht_failure_count >= 5:
                if self._sht_sensor_available:
                    self._sht_sensor_available = False
                    self.get_logger().warn(
                        "SHT45 sensor marked as unavailable after multiple failures. "
                        "Temperature and humidity readings disabled.")
            
            return None, None

    def _attempt_sht45_recovery(self):
        """
        Attempt to recover SHT45 sensor by testing communication.
        Returns True if sensor is recovered, False otherwise.
        """
        if self._sht_sensor_available:
            return True  # Already available
            
        try:
            # Try a simple read to test if sensor is back
            temperature, humidity = self._read_sht45()
            if temperature is not None and humidity is not None:
                self._sht_sensor_available = True
                self._sht_failure_count = 0
                self.get_logger().info("SHT45 sensor recovered - temperature and humidity readings restored")
                return True
        except Exception:
            pass  # Sensor still unavailable
            
        return False

    # ---------- MP2672 I2C helpers ----------
    def _read_mp2672_register(self, reg: int) -> Optional[int]:
        """
        Read a single register from MP2672 via I2C.
        
        Args:
            reg: Register address (0x00-0xFF)
        
        Returns:
            Register value (0-255) or None if read failed
        """
        if not MP2672_HOST_CONTROL:
            return None  # Host control disabled - I2C not available
        if not self.use_smbus2:
            return None
            
        try:
            # MP2672 uses single-byte read after register address write
            # Write register address, then read the register value
            result = self._i2c_write_read(MP2672_I2C_ADDR, reg, 1)
            if result and len(result) > 0:
                return result[0]
            return None
        except Exception as e:
            current_time = time.monotonic()
            if current_time - self.mp2672_last_error_time >= self.mp2672_error_log_interval:
                self.get_logger().debug(f"MP2672 register 0x{reg:02x} read failed: {e}")
                self.mp2672_last_error_time = current_time
            return None
    
    def _write_mp2672_register(self, reg: int, value: int) -> bool:
        """
        Write a single register to MP2672 via I2C.
        
        Args:
            reg: Register address (0x00-0xFF)
            value: Register value to write (0-255)
        
        Returns:
            True if write succeeded, False otherwise
        """
        if not MP2672_HOST_CONTROL:
            return False  # Host control disabled - I2C not available
        if not self.use_smbus2:
            return False
            
        try:
            # MP2672 write: first byte is register address, second byte is data
            # Use write command with 2 bytes: [reg_addr, data]
            with smbus2.SMBus(I2C_BUS_NUMBER) as b:
                b.write_byte_data(MP2672_I2C_ADDR, reg, value & 0xFF)
            return True
        except Exception as e:
            current_time = time.monotonic()
            if current_time - self.mp2672_last_error_time >= self.mp2672_error_log_interval:
                self.get_logger().warn(f"MP2672 register 0x{reg:02x} write failed: {e}")
                self.mp2672_last_error_time = current_time
            return False
    
    def _configure_mp2672_safe_settings(self) -> bool:
        """
        Configure MP2672 with safe settings for long-term USB charging.
        
        Critical settings:
        - Disable CHG timer (REG02H bits [2:1] = 00) to prevent charging timeout
        - Disable Watchdog timer (REG02H bits [5:4] = 00) to prevent watchdog expiration
        - Disable Suspend mode (REG02H bit [0] = 1) to keep boost enabled
        
        This prevents mysterious charging timeouts and watchdog expiration during development
        when Argo is left plugged into USB for extended periods.
        
        Returns:
            True if configuration succeeded, False otherwise
        """
        if not MP2672_HOST_CONTROL:
            return False  # Host control disabled - I2C configuration not available
        if not self.mp2672_available:
            return False
        
        try:
            # Read current REG02H value
            current_val = self._read_mp2672_register(MP2672_REG_02)
            if current_val is None:
                self.get_logger().warn("Failed to read MP2672 REG02H for configuration")
                return False
            
            # Configure bits:
            # Bit 7 (FSW): Preserve existing (switching frequency)
            # Bit 6 (I2C WD Timer Reset): Set to 0 (normal operation, not resetting)
            # Bits 5-4 (WD Timer [1:0]): Set to 00 (disable watchdog timer)
            # Bit 3 (Register Reset): Preserve existing (0=keep)
            # Bits 2-1 (CHG_TMR [1:0]): Set to 00 (disable CHG timer)
            # Bit 0 (EN_SUSP): Set to 1 (disable suspend mode, enable boost)
            
            # Clear bits [5:0] to set all config bits, then set bit [0] = 1
            # Mask 0xC0 = 11000000 preserves bits 7-6, clears bits 5-0
            # But we want to clear bits 5-4 (watchdog) and bits 2-1 (CHG timer), keep bit 6 as 0
            # So mask should be 0xC0 to preserve bits 7-6, then we'll set bit 0
            new_val = current_val & 0xC0  # Preserve bits 7-6 (FSW and I2C WD Timer Reset)
            new_val |= 0x01  # Set bit [0] = 1 (disable suspend, enable boost)
            # Bits [5:4] remain 0 (watchdog timer disabled)
            # Bits [2:1] remain 0 (CHG timer disabled)
            # Bit 6 is preserved (I2C WD Timer Reset, typically 0)
            
            # Only write if value changed
            if new_val != current_val:
                if self._write_mp2672_register(MP2672_REG_02, new_val):
                    # Verify write by reading back
                    verify_val = self._read_mp2672_register(MP2672_REG_02)
                    if verify_val == new_val:
                        # Parse and log the configuration for verification
                        wd_timer = (verify_val >> 4) & 0x03
                        chg_tmr = (verify_val >> 1) & 0x03
                        wd_timer_str = {0b00: "Disabled", 0b01: "40s", 0b10: "80s", 0b11: "160s"}.get(wd_timer, "Unknown")
                        chg_tmr_str = {0b00: "Disabled", 0b01: "8hrs", 0b10: "20hrs", 0b11: "12hrs"}.get(chg_tmr, "Unknown")
                        self.get_logger().info(
                            f"MP2672 configured: REG02H = 0x{new_val:02x} "
                            f"(WD timer: {wd_timer_str}, CHG timer: {chg_tmr_str}, Suspend disabled, Boost enabled)")
                        return True
                    else:
                        # Verification failed - check if critical bits (WD timer, CHG timer, suspend) are correct
                        # Some bits may be read-only or auto-set by the chip
                        verify_wd_timer = (verify_val >> 4) & 0x03
                        verify_chg_tmr = (verify_val >> 1) & 0x03
                        verify_en_susp = verify_val & 0x01
                        expected_wd_timer = 0b00  # Disabled
                        expected_chg_tmr = 0b00  # Disabled
                        expected_en_susp = 0b01  # Disabled (boost enabled)
                        
                        # Check critical bits: WD timer (must be disabled) and suspend (must be disabled)
                        # CHG timer bit 2 may be read-only, so we accept if WD timer and suspend are correct
                        if (verify_wd_timer == expected_wd_timer and 
                            verify_en_susp == expected_en_susp):
                            # Critical bits are correct, accept the configuration
                            wd_timer_str = {0b00: "Disabled", 0b01: "40s", 0b10: "80s", 0b11: "160s"}.get(verify_wd_timer, "Unknown")
                            chg_tmr_str = {0b00: "Disabled", 0b01: "8hrs", 0b10: "20hrs", 0b11: "12hrs"}.get(verify_chg_tmr, "Unknown")
                            self.get_logger().info(
                                f"MP2672 configured: REG02H = 0x{verify_val:02x} "
                                f"(WD timer: {wd_timer_str}, CHG timer: {chg_tmr_str}, Suspend disabled, Boost enabled) "
                                f"[Note: Some non-critical bits differ from written value]")
                            return True
                        else:
                            self.get_logger().warn(
                                f"MP2672 configuration verification failed: "
                                f"wrote 0x{new_val:02x}, read back 0x{verify_val:02x}. "
                                f"Critical bits: WD={verify_wd_timer} (expected {expected_wd_timer}), "
                                f"CHG={verify_chg_tmr} (expected {expected_chg_tmr}), "
                                f"SUSP={verify_en_susp} (expected {expected_en_susp})")
                            return False
                else:
                    self.get_logger().warn("Failed to write MP2672 REG02H configuration")
                    return False
            else:
                # Already configured correctly
                self.get_logger().debug(
                    f"MP2672 already configured correctly: REG02H = 0x{current_val:02x}")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Error configuring MP2672: {e}")
            return False
    
    def _read_mp2672_status(self) -> Optional[Tuple[bool, bool]]:
        """
        Read MP2672 status register (REG 03h) and parse charging status.
        
        Reference: MP2672 datasheet page 31, table for REG 03h
        
        Returns:
            Tuple of (charging_status, ac_power_present) or None if read failed
            charging_status: True if charging, False if not charging
            ac_power_present: True if AC/USB power present, False if not present
        """
        status_reg = self._read_mp2672_register(MP2672_REG_STATUS)
        if status_reg is None:
            return None
        
        # Parse status register bits according to datasheet page 31 (REG 03H)
        # 
        # Bit definitions (from datasheet):
        # Bits 5-4 (CHG_STAT [1:0]): 2-bit charging status
        #   00 = Not charge
        #   01 = Pre-charge
        #   10 = Constant-current/constant-voltage charge (charging)
        #   11 = Charge done
        #
        # Bit 3 (PPM_STAT): Power Path Management status
        #   0 = Not PPM
        #   1 = VINPPM (Input voltage PPM active - AC/USB power present)
        #
        # Bit 2 (BATTFLOAT_STAT): Battery status
        #   0 = Battery present
        #   1 = Battery missing
        #
        # Bit 1 (THERM_STAT): Thermal status
        #   0 = Normal
        #   1 = Thermal regulation
        #
        # Bit 0 (VSYS_STAT): VSYSMIN regulation status
        #   0 = Not in VSYSMIN regulation
        #   1 = In VSYSMIN regulation
        
        # Extract CHG_STAT [1:0] from bits 5-4
        chg_stat = (status_reg >> 4) & 0x03  # Extract bits 5-4 (2-bit value)
        
        # Determine charging status: charging if CHG_STAT is 01 (pre-charge) or 10 (CC/CV charge)
        charging_status = (chg_stat == 0b01) or (chg_stat == 0b10)
        
        # Extract PPM_STAT from bit 3 (indicates AC/USB power present and active)
        ppm_stat = (status_reg >> 3) & 0x01
        ac_power_present = bool(ppm_stat)  # 1 = VINPPM active (AC power present)
        
        # Extract BATTFLOAT_STAT from bit 2 (battery present/missing)
        # Critical: BATTFLOAT_STAT=1 indicates battery missing/balance cable fault
        battfloat_stat = (status_reg >> 2) & 0x01
        battery_missing = bool(battfloat_stat)  # 1 = Battery missing or balance cable fault
        
        # Log battery missing/balance cable fault (throttled)
        if battery_missing:
            current_time = time.monotonic()
            if current_time - self.mp2672_last_error_time >= self.mp2672_error_log_interval:
                self.get_logger().warning(
                    "MP2672 BATTFLOAT_STAT=1: Battery missing or balance cable fault detected! "
                    "Check balance cable connection.")
                self.mp2672_last_error_time = current_time
        
        return charging_status, ac_power_present
    
    def _read_mp2672_register_01(self) -> Optional[dict]:
        """
        Read MP2672 register 01H, fast charge current setting. Top 4 bits are OTP (not user programmable)
        Lowest 4 bits are ICC, Fast charge current setting.

        The fast charge current depends on RISet, which for argo is 6kOhms.
        Then bit 0 is 100mA, bit 1 is 200mA, bit 2 is 400mA, bit 3 is 800mA, i.e. binary code.
        Bits 1111 thus means 500mA + 1500mA = 2000mA.
        
        
        Returns:
            ICC bits, OTP bits are masked out. Return None if read failed.
        """
        reg_val = self._read_mp2672_register(MP2672_REG_01)
        if reg_val is None:
            return None
        
        # TODO: Parse bits according to datasheet when available
        return {
            'raw': reg_val,
            'hex': f'0x{reg_val:02x}',
            'binary': f'{reg_val:08b}'
        }
    
    def _read_mp2672_register_02(self) -> Optional[dict]:
        """
        Read MP2672 register 02H (Configuration register).
        
        Default: 0x95 (1001 0101)
        
        Bit definitions (from datasheet):
        - Bit 7 (FSW): Switching frequency (0=600kHz, 1=1200kHz, default 1=1200kHz)
        - Bit 6: I2C WD Timer Reset (0=normal, 1=reset timer, default 0)
        - Bits 5-4 (WD Timer [1:0]): Watchdog timer (00=disable, 01=40s, 10=80s, 11=160s. default 40s (01))
          Note: OTP programmable means default is set during manufacturing, but bits are writable via I2C
        - Bit 3: Register Reset (0=keep, 1=reset, after reset goes back to 0 automatically)
        - Bits 2-1 (CHG_TMR [1:0]): Charge timer (00=disable, 01=8hrs, 10=20hrs, 11=12hrs. default 10 (20 hours))
          Note: Bit 2 may be read-only or auto-set by chip (observed behavior)
        - Bit 0 (EN_SUSP): Suspend mode (0=enable suspend, 1=disable suspend/enable boost, default 1=disable suspend/enable boost)
        
        Returns:
            Dictionary with parsed configuration or None if read failed
        """
        reg_val = self._read_mp2672_register(MP2672_REG_02)
        if reg_val is None:
            return None
        
        # Parse bits according to datasheet
        fsw = (reg_val >> 7) & 0x01
        i2c_wd_reset = (reg_val >> 6) & 0x01
        wd_timer = (reg_val >> 4) & 0x03
        reg_reset = (reg_val >> 3) & 0x01
        chg_tmr = (reg_val >> 1) & 0x03
        en_susp = reg_val & 0x01
        
        # Convert to human-readable values
        fsw_freq = "1200kHz" if fsw else "600kHz"
        wd_timer_vals = {0b00: "Disabled", 0b01: "40s", 0b10: "80s", 0b11: "160s"}
        chg_tmr_vals = {0b00: "Disabled", 0b01: "8hrs", 0b10: "20hrs", 0b11: "12hrs"}
        
        return {
            'raw': reg_val,
            'hex': f'0x{reg_val:02x}',
            'fsw': fsw,
            'fsw_freq': fsw_freq,
            'i2c_wd_reset': bool(i2c_wd_reset),
            'wd_timer': wd_timer,
            'wd_timer_str': wd_timer_vals.get(wd_timer, "Unknown"),
            'reg_reset': bool(reg_reset),
            'chg_tmr': chg_tmr,
            'chg_tmr_str': chg_tmr_vals.get(chg_tmr, "Unknown"),
            'en_susp': bool(en_susp),
            'suspend_mode': "Disabled (boost enabled)" if en_susp else "Enabled (boost disabled)"
        }
    
    def _read_mp2672_register_04(self) -> Optional[dict]:
        """
        Read MP2672 register 04H (Fault register).
        
        Default: 0x00 (0000 0000) - All read-only fault flags
        
        Bit definitions (from datasheet):
        - Bit 7 (WD_FAULT): Watchdog timer expiration (0=normal, 1=fault)
        - Bit 6 (INPUT_FAULT): Input OVP fault (0=normal, 1=fault)
        - Bit 5 (THERM_SD_FAULT): Thermal shutdown (0=normal, 1=fault)
        - Bit 4 (TIMER_FAULT): Safety timer expiration (0=normal, 1=fault)
        - Bit 3 (BAT_FAULT): Battery OVP fault (0=normal, 1=fault)
        - Bits 2-0 (NTC_FAULT [2:0]): NTC fault status
          000=normal, 001=cold, 010=cool, 011=warm, 100=hot
        
        Returns:
            Dictionary with parsed fault status or None if read failed
        """
        reg_val = self._read_mp2672_register(MP2672_REG_04)
        if reg_val is None:
            return None
        
        # Parse bits according to datasheet
        wd_fault = (reg_val >> 7) & 0x01
        input_fault = (reg_val >> 6) & 0x01
        therm_sd_fault = (reg_val >> 5) & 0x01
        timer_fault = (reg_val >> 4) & 0x01
        bat_fault = (reg_val >> 3) & 0x01
        ntc_fault = reg_val & 0x07
        
        # Convert NTC fault to human-readable
        ntc_vals = {
            0b000: "Normal",
            0b001: "Cold",
            0b010: "Cool",
            0b011: "Warm",
            0b100: "Hot"
        }
        
        # Check if any fault is active
        any_fault = (wd_fault or input_fault or therm_sd_fault or 
                    timer_fault or bat_fault or ntc_fault != 0b000)
        
        return {
            'raw': reg_val,
            'hex': f'0x{reg_val:02x}',
            'any_fault': any_fault,
            'wd_fault': bool(wd_fault),
            'input_fault': bool(input_fault),
            'therm_sd_fault': bool(therm_sd_fault),
            'timer_fault': bool(timer_fault),
            'bat_fault': bool(bat_fault),
            'ntc_fault': ntc_fault,
            'ntc_fault_str': ntc_vals.get(ntc_fault, "Unknown"),
            'fault_summary': {
                'wd_fault': "Watchdog timer expiration" if wd_fault else None,
                'input_fault': "Input OVP fault" if input_fault else None,
                'therm_sd_fault': "Thermal shutdown" if therm_sd_fault else None,
                'timer_fault': "Safety timer expiration" if timer_fault else None,
                'bat_fault': "Battery OVP fault" if bat_fault else None,
                'ntc_fault': f"NTC {ntc_vals.get(ntc_fault, 'Unknown')}" if ntc_fault != 0b000 else None
            }
        }
    
    def _reset_mp2672_watchdog(self) -> bool:
        """
        Reset MP2672 watchdog timer by toggling I2C WD Timer Reset bit (REG02H bit 6).
        
        According to datasheet: In host control mode, the watchdog timer resets all
        registers to default values if not reset periodically. This method resets the
        watchdog timer to prevent register resets.
        
        Returns:
            True if reset succeeded, False otherwise
        """
        if not MP2672_HOST_CONTROL:
            return False  # Host control disabled - I2C not available
        if not self.mp2672_available:
            return False
        
        try:
            # Read current REG02H value
            current_val = self._read_mp2672_register(MP2672_REG_02)
            if current_val is None:
                return False
            
            # Set bit 6 (I2C WD Timer Reset) to 1 to reset the timer
            reset_val = current_val | 0x40  # Set bit 6 = 1 (0x40 = 01000000)
            
            if self._write_mp2672_register(MP2672_REG_02, reset_val):
                # Immediately clear bit 6 back to 0 (normal operation)
                # This completes the reset cycle
                normal_val = reset_val & 0xBF  # Clear bit 6 (0xBF = 10111111)
                return self._write_mp2672_register(MP2672_REG_02, normal_val)
            
            return False
        except Exception as e:
            self.get_logger().debug(f"Failed to reset MP2672 watchdog: {e}")
            return False
    
    def _check_mp2672_availability(self):
        """
        Check if MP2672 is available on I2C bus and configure safe settings.
        
        Note: MP2672 only appears on I2C bus when USB power is present (powers the chip).
        During normal robot operation (battery only), MP2672 is not accessible via I2C.
        When USB power is connected (e.g., at shore), we configure safe settings:
        - Disable CHG timer to prevent charging timeout
        - Disable Watchdog timer to prevent register resets (or reset it periodically if enabled)
        - Disable Suspend mode to keep boost enabled
        
        According to datasheet: In host control mode, the watchdog timer resets all registers
        to default values if not reset periodically. We disable it by default, but if enabled,
        we reset it periodically to prevent register resets.
        
        When MP2672_HOST_CONTROL = False, this function does nothing (standalone mode).
        """
        if not MP2672_HOST_CONTROL:
            # Standalone mode - MP2672 not accessible via I2C
            if self.mp2672_available:
                self.mp2672_available = False
            return False
        
        try:
            # Try reading status register to verify device is present
            status = self._read_mp2672_register(MP2672_REG_STATUS)
            if status is not None:
                if not self.mp2672_available:
                    self.mp2672_available = True
                    self.get_logger().info(f"MP2672 detected at I2C address 0x{MP2672_I2C_ADDR:02x} (USB power present)")
                    
                    # First, check for and clear any existing watchdog fault by resetting watchdog
                    fault_reg = self._read_mp2672_register(MP2672_REG_04)
                    if fault_reg is not None and (fault_reg & 0x80):  # Bit 7 = watchdog fault
                        self.get_logger().warn("MP2672 watchdog fault detected - resetting watchdog timer")
                        self._reset_mp2672_watchdog()
                    
                    # Configure safe settings for long-term USB charging
                    self._configure_mp2672_safe_settings()
                    
                    # Check if watchdog is enabled (vs disabled) for periodic reset tracking
                    reg02 = self._read_mp2672_register(MP2672_REG_02)
                    if reg02 is not None:
                        wd_timer = (reg02 >> 4) & 0x03
                        chg_tmr = (reg02 >> 1) & 0x03
                        self.mp2672_watchdog_enabled = (wd_timer != 0b00)  # Enabled if not 00
                        wd_timer_str = {0b00: "Disabled", 0b01: "40s", 0b10: "80s", 0b11: "160s"}.get(wd_timer, "Unknown")
                        chg_tmr_str = {0b00: "Disabled", 0b01: "8hrs", 0b10: "20hrs", 0b11: "12hrs"}.get(chg_tmr, "Unknown")
                        self.get_logger().info(f"MP2672 REG02H = 0x{reg02:02x}: WD timer = {wd_timer_str}, CHG timer = {chg_tmr_str}")
                        if self.mp2672_watchdog_enabled:
                            self.get_logger().info(f"MP2672 watchdog timer is enabled, will reset periodically")
                else:
                    # MP2672 already available - check if we need to reset watchdog
                    # Only reset if watchdog is enabled (not disabled)
                    if self.mp2672_watchdog_enabled:
                        current_time = time.monotonic()
                        if (current_time - self.mp2672_last_watchdog_reset_time) >= self.mp2672_watchdog_reset_interval:
                            if self._reset_mp2672_watchdog():
                                self.mp2672_last_watchdog_reset_time = current_time
                                self.get_logger().debug("MP2672 watchdog timer reset")
                return True
            else:
                if self.mp2672_available:
                    self.mp2672_available = False
                    self.get_logger().debug("MP2672 not accessible via I2C (normal when USB power not present)")
                return False
        except Exception:
            if self.mp2672_available:
                self.mp2672_available = False
                self.get_logger().debug("MP2672 I2C communication error (normal when USB power not present)")
            return False

    # ---------- Charging status reading (MP2672 I2C with GPIO fallback) ----------
    def _read_charging_status(self):
        """
        Read charging status from MP2672 I2C register (primary) or GPIO (fallback).
        
        Priority:
        1. MP2672 I2C register 0x03 (if available and MP2672_HOST_CONTROL = True)
        2. GPIO status with time-window filtering (fallback or standalone mode)
        
        Returns:
            Tuple of (charging_status, ac_power_present) or (None, None) if unavailable
        """
        # Try MP2672 I2C first (primary source) - only if host control enabled
        if MP2672_HOST_CONTROL and self.mp2672_available:
            status = self._read_mp2672_status()
            if status is not None:
                charging_status, ac_power_present = status
                # Update GPIO timestamps to match I2C reading for consistency
                current_time = time.monotonic()
                if charging_status:
                    self._last_charging_true_time = current_time
                if ac_power_present:
                    self._last_ac_power_true_time = current_time
                return charging_status, ac_power_present
        
        # Fallback to GPIO if I2C not available
        return self._read_gpio_status()
    
    def _read_gpio_status(self):
        """
        Get current GPIO status based on time-window filtering.
        
        Note: When battery is fully charged, the MP2672GD cycles between charging
        and supplementing modes. This is normal behavior. We report "charging" or
        "AC power present" if seen within the last time window to provide stable
        status reporting despite the rapid cycling.
        
        The actual GPIO polling happens at 10Hz in read_gpio_status_only() (needed to reliably detect 1-2 Hz blinking fault pattern).
        This method just returns the time-filtered status.
        """
        if not self.gpio_available:
            return None, None
            
        current_time = time.monotonic()
        
        # Determine status based on time windows
        # Report True if seen as True within the time window
        charging_status_recent = (current_time - self._last_charging_true_time) < self._charging_window_s
        ac_power_present = (current_time - self._last_ac_power_true_time) < self._ac_power_window_s

        # Fail-safe: charging cannot be true if AC power is absent
        if charging_status_recent and not ac_power_present:
            if (current_time - self._last_charging_conflict_log_time) >= self._charging_conflict_log_interval:
                self.get_logger().warning(
                    "Charging GPIO asserted but AC power is absent; suppressing charging status (check charger hardware)."
                )
                self._last_charging_conflict_log_time = current_time
            # Age out the stale charging timestamp so it clears immediately
            self._last_charging_true_time = current_time - self._charging_window_s
            charging_status_recent = False

        charging_status = charging_status_recent
        
        return charging_status, ac_power_present

    # ---------- AC Power Plug-in/Plug-out Tracking for MP2672 CHG Timer ----------
    def _track_ac_power_state_change(self, ac_power_present: Optional[bool]):
        """Track AC power plug-in/plug-out events for MP2672 CHG timer monitoring"""
        if ac_power_present is None:
            return
        
        current_time = time.monotonic()
        current_wall_time = time.time()
        
        # Detect state change
        prev_ac_power = self._prev_ac_power_present
        if prev_ac_power is None:
            # First reading - check if AC is present and we have stored plug-in time (reboot case)
            if ac_power_present and self._ac_power_plug_in_time_persistent is not None:
                # AC is present and we have a stored plug-in time - restore it
                # Calculate elapsed time since stored plug-in time
                elapsed_since_plug_in = current_wall_time - self._ac_power_plug_in_time_persistent
                # Also set monotonic time for consistency (though we use wall clock for calculations)
                self._ac_power_plug_in_time = current_time
                remaining = self._calculate_charging_time_remaining()
                if remaining is not None:
                    self.get_logger().info(
                        f"AC power already present on startup - restored plug-in time "
                        f"(elapsed: {elapsed_since_plug_in/3600:.2f}h, remaining: {remaining:.2f}h)")
                else:
                    self.get_logger().warning(
                        f"AC power present but CHG timer has expired "
                        f"(elapsed: {elapsed_since_plug_in/3600:.2f}h > {self._mp2672_chg_timer_hours:.1f}h)")
            elif ac_power_present:
                # AC is present but no stored time - record new plug-in
                self._ac_power_plug_in_time = current_time
                self._ac_power_plug_in_time_persistent = current_wall_time
                self._ac_power_plug_out_time_persistent = None
                self.get_logger().info("AC power plugged in - starting CHG timer")
                # Save immediately to persist across reboots
                self._save_battery_slopes()
            # If AC not present, no action needed
        elif prev_ac_power != ac_power_present:
            # State changed
            if ac_power_present:
                # AC power plugged in
                self._ac_power_plug_in_time = current_time
                self._ac_power_plug_in_time_persistent = current_wall_time
                self._ac_power_plug_out_time_persistent = None
                self.get_logger().info(
                    f"AC power plugged in - starting CHG timer ({self._mp2672_chg_timer_hours:.1f}h)")
                # Save immediately to persist across reboots
                self._save_battery_slopes()
            else:
                # AC power unplugged
                self._ac_power_plug_out_time_persistent = current_wall_time
                self._ac_power_plug_in_time = None
                self._ac_power_plug_in_time_persistent = None
                self.get_logger().info("AC power unplugged - CHG timer stopped")
                # Save to clear plug-in time
                self._save_battery_slopes()
    
    def _calculate_charging_time_remaining(self) -> Optional[float]:
        """Calculate remaining time until MP2672 CHG timer expires
        
        Uses wall clock time for accuracy across reboots.
        
        Returns:
            Remaining time in hours, or None if:
            - AC power not present
            - CHG timer is disabled
            - Plug-in time not available
        """
        if self._ac_power_plug_in_time_persistent is None:
            return None
        
        if self._mp2672_chg_timer_hours is None:
            # CHG timer is disabled
            return None
        
        # Use wall clock time for accuracy across reboots
        current_wall_time = time.time()
        elapsed_hours = (current_wall_time - self._ac_power_plug_in_time_persistent) / 3600.0
        remaining_hours = self._mp2672_chg_timer_hours - elapsed_hours
        
        # Return None if timer has already expired (negative remaining time)
        if remaining_hours < 0:
            return None
        
        return remaining_hours

    # ---------- I2C Recovery Methods (similar to imu.py) ----------
    def _handle_io_error(self, error, sensor_name="I2C"):
        """Handle I2C IOError with health tracking and throttled logging"""
        current_time = time.monotonic()
        self._consecutive_io_errors += 1

        # Suppress "Transient" messages during retry mode - re-initialization attempts will log instead
        # Only log transient errors in normal mode (when retry_timer doesn't exist or is None)
        in_retry_mode = hasattr(self, 'retry_timer') and self.retry_timer is not None
        
        # Log error with throttling (max once per 5 seconds) - but not in retry mode
        if not in_retry_mode and current_time - self._last_i2c_error_log_time >= 5.0:
            self.get_logger().warn(
                f"Transient {sensor_name} error (attempt {self._consecutive_io_errors}): {error}")
            self._last_i2c_error_log_time = current_time

        # Mark node as unhealthy after first IO error
        if self.node_healthy:
            self.node_healthy = False
            self.set_unhealthy(f"Sensor error: {sensor_name}")
            self.get_logger().warn(
                f"Node health set to UNHEALTHY due to {sensor_name} errors. Switching to low-frequency retry mode.")
            # Switch to low-frequency retry mode
            self._switch_to_retry_mode()
        
        # Check for critical I2C failure (ADC sensor failure = critical for battery monitoring)
        if sensor_name == "ADC sensors" or "ADC" in sensor_name:
            # Reset recovery tracking on any new failure (interrupts recovery sequence)
            if self._recovery_start_time is not None:
                self.get_logger().warn(
                    f"I2C recovery interrupted by new failure (had {self._consecutive_successful_reads} "
                    f"consecutive successful reads)")
                self._consecutive_successful_reads = 0
                self._recovery_start_time = None
            
            if self._adc_failure_start_time is None:
                self._adc_failure_start_time = current_time
                self.get_logger().warn("ADC failure detected - starting critical failure timer")
            
            # Check if ADC has been failing for too long (critical failure)
            failure_duration = current_time - self._adc_failure_start_time
            if failure_duration >= self._adc_failure_timeout and not self._critical_i2c_failure:
                self._critical_i2c_failure = True
                self._publish_i2c_failure(True)
                self._last_i2c_failure_republish_time = current_time  # Initialize republish timer
                self.get_logger().error(
                    f"🔴 CRITICAL I2C FAILURE: ADC sensor has been failing for {failure_duration:.1f}s. "
                    f"Battery monitoring unavailable - controller should switch to RTH mode.")

    def _switch_to_retry_mode(self):
        """Switch timers to low-frequency retry mode"""
        # Cancel existing timers
        if hasattr(self, 'sail_current_timer'):
            self.sail_current_timer.cancel()
        if hasattr(self, 'battery_safety_timer'):
            self.battery_safety_timer.cancel()
        # Keep GPIO timer running even in retry mode
        
        # Create single low-frequency retry timer (1 Hz)
        self.retry_timer = self.create_timer(1.0, self._retry_callback)
        self.get_logger().info("Switched to 1Hz retry mode for I2C recovery (GPIO monitoring continues)")
        
        # Reset consecutive error counter for recovery tracking
        self._consecutive_io_errors = 0

    def _switch_to_normal_mode(self):
        """Switch timers back to normal frequency"""
        # Cancel retry timer if it exists
        if hasattr(self, 'retry_timer'):
            self.retry_timer.cancel()
        
        # Recreate normal timers
        self.sail_current_timer = self.create_timer(
            1.0 / SAIL_CURRENT_RATE_HZ, self.read_sail_current)
        self.battery_safety_timer = self.create_timer(
            BATTERY_SAFETY_INTERVAL_S, self.read_battery_safety_sensors)
        # GPIO timer is continuous and doesn't need recreation
        
        self.get_logger().info("Switched back to normal mode - I2C communication recovered")

    def _reinitialize_sensors(self):
        """Re-initialize ADC and SHT45 sensors after I2C recovery"""
        try:
            # Re-initialize ADC
            setup_byte = self._build_setup(
                reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(self.adc_addr, setup_byte)
            time.sleep(0.05)
            
            # SHT45 doesn't need re-init, but we can verify communication
            # by attempting a read (the actual read will be done in normal operation)
            
            return True
            
        except Exception as e:
            # Store error for caller to log - don't log here to avoid duplicate messages
            self._last_reinit_error = str(e)
            return False

    def _check_io_recovery(self):
        """Check if I2C communication has recovered and switch back to normal mode"""
        current_time = time.monotonic()
        time_since_last_success = current_time - self._last_successful_read_time
        time_since_last_recovery_attempt = current_time - self._last_recovery_attempt_time

        # Try recovery if we've been in retry mode for a while
        if (time_since_last_success > 5.0 and  # Been in retry mode for at least 5 seconds
                time_since_last_recovery_attempt > 3.0):  # Wait at least 3 seconds between attempts

            self._recovery_attempt_count += 1
            self._last_recovery_attempt_time = current_time

            # Try re-initialization (no debug message - only log on failure)
            if self._reinitialize_sensors():
                self.node_healthy = True
                self.set_healthy("Sensors recovered")
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0  # Reset counter on success
                self.get_logger().info(
                    f"✅ Sensor re-initialization successful after {self._consecutive_io_errors} I2C errors")
            else:
                # Re-initialization failed - log single informative error with attempt count and error details
                error_msg = getattr(self, '_last_reinit_error', 'I2C device not available')
                self.get_logger().warn(
                    f"⚠️  Sensor re-initialization failed (attempt {self._recovery_attempt_count}): {error_msg} - "
                    f"staying in retry mode. Check I2C bus connections and argo_battery_water.service status")

    def _retry_callback(self):
        """Retry callback for low-frequency I2C recovery attempts"""
        if self._shutdown_requested:
            return
        
        # Check for recovery
        self._check_io_recovery()
        
        # Try to read sensors (will mark as healthy if successful)
        try:
            # Try a simple ADC read to test I2C
            # Use longer delay for reliability with LP5814 on bus
            raw0 = self._read_adc_channel_avg(0, repeats=4, settle_delay_s=0.010)
            if raw0 > 0:  # Successful read
                self._last_successful_read_time = time.monotonic()
                self._consecutive_io_errors = 0
                
                # Track sustained recovery - require multiple consecutive successful reads before clearing critical failure
                if self._critical_i2c_failure:
                    # We're in critical failure state - track consecutive successful reads
                    if self._recovery_start_time is None:
                        self._recovery_start_time = time.monotonic()
                        self._consecutive_successful_reads = 0
                        self.get_logger().info("I2C recovery sequence started (retry mode) - monitoring for sustained recovery...")
                    
                    self._consecutive_successful_reads += 1
                    
                    # Check if we have sustained recovery (enough consecutive successful reads)
                    if self._consecutive_successful_reads >= self._recovery_required_reads:
                        # Sustained recovery achieved - clear critical failure
                        recovery_duration = time.monotonic() - self._recovery_start_time
                        self._adc_failure_start_time = None
                        self._critical_i2c_failure = False
                        self._consecutive_successful_reads = 0
                        self._recovery_start_time = None
                        self._publish_i2c_failure(False)
                        self._last_i2c_failure_republish_time = 0.0  # Reset republish timer
                        self.get_logger().info(
                            f"✅ I2C recovery: Sustained recovery achieved after {self._recovery_required_reads} "
                            f"consecutive successful reads ({recovery_duration:.1f}s) - critical failure cleared")
                    else:
                        # Still in recovery sequence - log progress periodically
                        if self._consecutive_successful_reads % 5 == 0:
                            self.get_logger().info(
                                f"I2C recovery progress (retry mode): {self._consecutive_successful_reads}/{self._recovery_required_reads} "
                                f"consecutive successful reads")
                else:
                    # Not in critical failure - reset recovery tracking
                    if self._adc_failure_start_time is not None:
                        self._adc_failure_start_time = None
                    self._consecutive_successful_reads = 0
                    self._recovery_start_time = None
                
                # Automatic recovery on successful read
                if not self.node_healthy:
                    self.node_healthy = True
                    self.set_healthy("Automatic recovery successful")
                    self._switch_to_normal_mode()
                    self._recovery_attempt_count = 0
                    self.get_logger().info("Automatic recovery: successful reads restored, switching back to normal mode")
        except Exception as e:
            # CRITICAL: Call _handle_io_error to check for critical failure timeout
            # This ensures the 30-second critical failure check happens even in retry mode
            self._handle_io_error(e, "ADC sensors")
            # Continue in retry mode
            pass
        
        # Attempt SHT45 sensor recovery (non-blocking)
        self._attempt_sht45_recovery()
        
        # Periodically republish I2C failure state if in failure (uses existing timer)
        self._maybe_republish_i2c_failure()

    # ---------- Test: RC decay of REFOUT on AIN3 with sampling & PNG ----------
    def _set_ain3_mode(self, sel_bits: int):
        setup_byte = self._build_setup(
            reg=1, sel=sel_bits, clk=0, bip_uni=0, rst=1, x=0)
        self._i2c_write(self.adc_addr, setup_byte)

    def _adc_rc_capture(self, duration_s: float = 3.0, sample_dt: float = 0.010):
        # Charge AIN3 via REFOUT for 50 ms
        self._set_ain3_mode(sel_bits=0b111)  # REFOUT on AIN3
        time.sleep(0.05)
        # Switch AIN3 to analog input and sample decay
        self._set_ain3_mode(sel_bits=0b101)
        t0 = time.monotonic()
        times = []
        volts = []
        # Use minimal repeats to reduce loading; settle quickly
        while True:
            t = time.monotonic() - t0
            if t >= duration_s:
                break
            # One-shot sample on CH3
            try:
                raw = self._read_adc_channel_avg(
                    3, repeats=1, settle_delay_s=0.005)  # Increased for LP5814 compatibility
            except Exception:
                raw = 0
            v = raw * self.lsb_value
            times.append(t)
            volts.append(v)
            # Sleep until next sample time
            rem = sample_dt - (time.monotonic() - t0 - t)
            if rem > 0:
                time.sleep(rem)
        # Save PNG if matplotlib is available (import only when needed)
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from datetime import datetime

            plt.figure(figsize=(8, 4))
            plt.plot(times, volts, '-', linewidth=1.5)
            plt.title('AIN3 RC Decay (REFOUT -> Analog Input)')
            plt.xlabel('Time (s)')
            plt.ylabel('Voltage (V)')
            plt.grid(True, alpha=0.3)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = f'ain3_decay_{ts}.png'
            try:
                plt.savefig(path, dpi=120, bbox_inches='tight')
            finally:
                plt.close()
            return path
        except Exception:
            return None

    def _scale_pct(self, value, vmin, vmax):
        # Handle NaN values (I2C failure)
        if math.isnan(value):
            return 0  # Return 0% for NaN (invalid reading)
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, bat_v, sw_v, cur_a, temp_c, humid_pct):
        if not self._vis_ascii:
            return
        
        # Initialize ASCII visualization on first call (delayed until first periodic update)
        if not self._vis_initialized:
            self._init_ascii_vis()
        
        # Nominal ranges
        bat_max = 12.0
        sw_max = 4.2
        cur_max = 4.2
        t_min, t_max = -20.0, 60.0
        h_min, h_max = 0.0, 100.0
        try:
            sys.stdout.write('\x1b[H')  # home
            # Handle NaN values in formatting
            bat_v_str = f"{bat_v:7.3f}" if not math.isnan(bat_v) else "   NaN"
            sw_v_str = f"{sw_v:7.3f}" if not math.isnan(sw_v) else "   NaN"
            lines = [
                f"Battery {bat_v_str} V  " + self._bar(bat_v, bat_max),
                f"Salt   {sw_v_str} V  " + self._bar(sw_v, sw_max),
                f"Sail I {cur_a:7.3f} A  " + self._bar(cur_a, cur_max),
            ]
            if temp_c is not None:
                # Map temp to 0..range for bar
                temp_span = max(1e-6, t_max - t_min)
                temp_norm = (max(t_min, min(t_max, temp_c)) -
                             t_min) / temp_span * 100.0
                lines.append(f"PCB   {temp_c:7.2f} C  " +
                             self._bar(temp_norm, 100.0))
            if humid_pct is not None:
                lines.append(f"Humid {humid_pct:7.2f} %  " +
                             self._bar(humid_pct, 100.0))
            
            # Add battery lifetime and slope information
            if self._latest_battery_remaining_pct is not None:
                lines.append(f"Batt% {self._latest_battery_remaining_pct:7.1f}%  " +
                             self._bar(self._latest_battery_remaining_pct, 100.0))
            
            # Add lifetime estimates
            if self._latest_time_to_full_hours is not None:
                lines.append(f"T2Full {self._latest_time_to_full_hours:7.1f}h")
            elif self._latest_time_to_empty_hours is not None:
                lines.append(f"T2Empty {self._latest_time_to_empty_hours:7.1f}h")
            else:
                lines.append("T2Full/Empty: N/A")
            
            # Power state and per-regime slope information
            if self._latest_power_state is not None:
                state_line = f"Power: {self._latest_power_state}"
                if self._latest_activity_state:
                    state_line += f" ({self._latest_activity_state})"
                lines.append(state_line)
            lines.append(f"Slopes: {self._format_regime_slopes()}")
            
            # Add charging status
            if self._latest_charging_status is not None and self._latest_ac_power_present is not None:
                charging_icon = "🔌" if self._latest_charging_status else "🔋"
                ac_icon = "⚡" if self._latest_ac_power_present else "🔌"
                lines.append(f"Status: {charging_icon}Chg={self._latest_charging_status}, {ac_icon}AC={self._latest_ac_power_present}")
            
            lines.append("Ctrl-C to exit")
            for ln in lines:
                sys.stdout.write(ln + '\n')
            sys.stdout.flush()
        except Exception:
            pass

    def _init_ascii_vis(self):
        if self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[?25l')  # hide cursor
            sys.stdout.write('\x1b[2J')    # clear
            sys.stdout.write('\x1b[H')     # home
            sys.stdout.flush()
            self._vis_initialized = True
        except Exception:
            self._vis_initialized = False

    def _teardown_ascii_vis(self):
        if not self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')
            sys.stdout.write('\x1b[2J')
            sys.stdout.write('\x1b[H')
            sys.stdout.write('\x1b[?25h')
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False

    def _bar(self, value: float, limit: float, width: int = 50) -> str:
        width = max(10, width)
        # Handle NaN values (I2C failure) - show empty bar
        if math.isnan(value):
            return '[' + ('-' * width) + ']'  # Empty bar for NaN
        v = max(0.0, min(limit, float(value)))
        fill = int(round((v / limit) * width)) if limit > 0 else 0
        if fill > width:
            fill = width
        return '[' + ('#' * fill) + ('-' * (width - fill)) + ']'

    def _publish_i2c_failure(self, failed: bool, force: bool = False):
        """Publish I2C failure status to critical failure topic
        
        IMPORTANT: This method is THROTTLED - it only publishes when the state changes.
        However, if force=True, it will republish even if state hasn't changed (for subscribers
        that may have missed the initial message).
        
        Publishing occurs:
        - When critical failure is detected (after 30s of ADC failures): publishes True
        - When I2C recovers (sustained successful reads): publishes False
        - Periodically when force=True (allows subscribers to catch up)
        """
        # Throttle: Only publish when state actually changes OR if forced
        if not force and self._last_published_i2c_failure_state == failed:
            # State hasn't changed and not forced - don't spam ROS topic
            return
        
        try:
            # Check if publisher is ready (node must be fully initialized)
            if not rclpy.ok() or self.pub_i2c_failure is None:
                self.get_logger().warning("Cannot publish I2C failure status - publisher not ready")
                return
            
            msg = Bool(data=failed)
            self.pub_i2c_failure.publish(msg)
            self._i2c_failure_state = failed
            self._last_published_i2c_failure_state = failed  # Track published state
            
            if failed:
                if force:
                    self.get_logger().debug("Republished CRITICAL I2C failure (periodic update)")
                else:
                    self.get_logger().error("Published CRITICAL I2C failure - controller should switch to RTH")
            else:
                if force:
                    self.get_logger().debug("Republished I2C failure state: False (periodic update)")
                else:
                    self.get_logger().info("Published I2C recovery - critical failure cleared")
        except Exception as e:
            self.get_logger().error(f"Error publishing I2C failure status: {e}")
    
    def _maybe_republish_i2c_failure(self):
        """Periodically republish I2C failure state so subscribers can catch up if they missed initial message.
        Called from existing sensor read callbacks - no separate timer needed.
        
        Republishes both True and False states periodically so late-joining subscribers (like topic echo)
        can always see the current state."""
        current_time = time.monotonic()
        
        # Always do initial republish once (for late-joining subscribers like power control)
        if not self._initial_republish_done and self._last_published_i2c_failure_state is not None:
            self._publish_i2c_failure(self._critical_i2c_failure, force=True)
            self._last_i2c_failure_republish_time = current_time
            self._initial_republish_done = True
            self.get_logger().debug(f"Initial I2C failure state republished: {self._critical_i2c_failure}")
            return
        
        # Periodic republish of current state (both True and False) so late-joining subscribers can catch up
        if current_time - self._last_i2c_failure_republish_time >= self._i2c_failure_republish_interval:
            # Republish current state (force=True to bypass throttling)
            self._publish_i2c_failure(self._critical_i2c_failure, force=True)
            self._last_i2c_failure_republish_time = current_time
    
    def _request_shutdown(self):
        """Request node shutdown due to critical sensor failure"""
        self._shutdown_requested = True
        self.get_logger().fatal("Node shutting down due to critical sensor failure")
        # Set health status as failed
        self.set_unhealthy("Critical sensor failure - shutting down")
        # Publish critical I2C failure before shutdown
        self._publish_i2c_failure(True)
        # Cancel timers to stop further execution
        if hasattr(self, 'sail_current_timer'):
            self.sail_current_timer.cancel()
        if hasattr(self, 'battery_safety_timer'):
            self.battery_safety_timer.cancel()

    def _has_significant_change(self, current_value, previous_value, threshold_pct=THRESHOLD_CHANGE_PCT):
        """Check if current value has changed by more than threshold_pct from previous value"""
        if previous_value is None or current_value is None:
            return True  # Always publish if we don't have previous data

        if previous_value == 0.0:
            # Avoid division by zero; consider any non-zero change significant
            return current_value != 0.0

        change_pct = abs((current_value - previous_value) /
                         previous_value) * 100.0
        return change_pct >= threshold_pct

    # ---------- High-frequency GPIO status reading (10Hz) ----------
    def read_gpio_status_only(self):
        """High-frequency reading of GPIO status only (10Hz) to track MP2672GD cycling and detect fault patterns
        
        Polling at 10Hz is necessary to reliably detect the 1-2 Hz blinking fault pattern.
        MP2672 datasheet specifies 1Hz blinking for faults, but we observe ~2Hz in practice.
        Nyquist sampling theorem requires >4Hz sampling rate for 2Hz signal.
        """
        if self._shutdown_requested or not self.gpio_available:
            return
            
        current_time = time.monotonic()
        
        try:
            # Read !ACOK first - needed to gate fault detection (only report fault when charger present)
            ac_power_raw = False
            if self.acok_gpio_line is not None:
                acok_gpio_value = self.acok_gpio_line.get_value()
                ac_power_raw = not acok_gpio_value  # Invert: !ACOK=0 means AC power present=True
                if ac_power_raw:
                    self._last_ac_power_true_time = current_time
            
            # Read !CHARGING GPIO (PC12, line 76) - invert logic for charging status
            # Note: Code uses !CHARGING (PC12) for transition tracking; MP2672 datasheet describes 1Hz blink on STAT.
            # When no charger is present, !CHARGING may toggle (floating/different IC state) and must not be reported as fault.
            if self.charging_gpio_line is not None:
                charging_gpio_value = self.charging_gpio_line.get_value()
                charging_raw = not charging_gpio_value  # Invert: !CHARGING=0 means charging=True
                
                # Track GPIO transitions for fault detection (1-2 Hz blinking pattern)
                # MP2672 datasheet: 1Hz blinking on STAT pin indicates fault conditions
                if self._last_charging_gpio_value is not None and charging_gpio_value != self._last_charging_gpio_value:
                    # GPIO state changed - record transition time
                    if self._last_charging_gpio_change_time is not None:
                        period = current_time - self._last_charging_gpio_change_time
                        self._charging_gpio_transitions.append(period)
                    
                    self._last_charging_gpio_change_time = current_time
                
                self._last_charging_gpio_value = charging_gpio_value
                
                # Only treat blinking as charging fault when AC power is present (charger plugged in).
                # When no charger is present, !CHARGING/STAT may toggle or float -> false positive "battery missing" fault.
                ac_power_changed = (
                    self._gpio_ac_power_raw_prev is not None and
                    ac_power_raw != self._gpio_ac_power_raw_prev
                )
                if ac_power_changed or not ac_power_raw:
                    # Discard transitions from floating/unpowered STAT line or from before plug-in.
                    self._charging_gpio_transitions.clear()
                    self._last_charging_gpio_change_time = None
                    self._charging_fault_detected = False
                    self._charging_fault_frequency = None
                elif len(self._charging_gpio_transitions) >= 4:
                    # Require recent edges: a steady STAT line must not keep a stale buffer latched.
                    time_since_last_edge = (
                        current_time - self._last_charging_gpio_change_time
                        if self._last_charging_gpio_change_time is not None else float('inf')
                    )
                    if time_since_last_edge > self._charging_fault_stale_edge_s:
                        self._charging_gpio_transitions.clear()
                        self._charging_fault_detected = False
                        self._charging_fault_frequency = None
                    else:
                        # AC present: analyze transitions for fault pattern (1-2 Hz edge rate)
                        recent_periods = list(self._charging_gpio_transitions)[-8:]  # Use last 8 transitions
                        avg_period = sum(recent_periods) / len(recent_periods)
                        frequency = 1.0 / avg_period if avg_period > 0 else 0.0

                        # MP2672 fault pattern: 1Hz blink ~= 2Hz edge rate (datasheet 1Hz full cycle)
                        if 0.8 <= frequency <= 2.5:
                            self._charging_fault_detected = True
                            self._charging_fault_frequency = frequency
                        else:
                            self._charging_fault_detected = False
                            self._charging_fault_frequency = None
                else:
                    self._charging_fault_detected = False
                    self._charging_fault_frequency = None

                self._gpio_ac_power_raw_prev = ac_power_raw
                
                # Update last-seen time when charging is active
                if charging_raw:
                    self._last_charging_true_time = current_time
            
            # Read !PG GPIO (PI4, line 260, pin 38) from CH221K USB-C voltage controller
            # CH221K PG pin (Pin 3) indicates successful USB Power Delivery (PD) negotiation
            # Active low: Hardware LOW = PD negotiation successful, Hardware HIGH (via pull-up) = no PD (standard USB) or failed
            # Note: With standard USB cables (non-PD), this will be HIGH - not an indication of power failure
            # May not work consistently on subsequent USB cable plug-ins
            # GPIO reading behavior: Hardware LOW reads as GPIO HIGH (1), hardware HIGH reads as GPIO LOW (0)
            # This inversion is likely due to GPIO driver or hardware configuration
            if self.pg_gpio_line is not None:
                pg_gpio_value = self.pg_gpio_line.get_value()
                # Note: GPIO reading appears inverted from hardware pin state
                # When hardware pin is LOW (PD negotiation successful), GPIO reads HIGH (1)
                # When hardware pin is HIGH (no PD or negotiation failed), GPIO reads LOW (0)
                usb_pd_negotiated = (pg_gpio_value == 1)  # GPIO HIGH = hardware LOW = USB PD negotiation successful
                
                # Store USB PD negotiation status (used in battery status service)
                self._latest_usb_pd_negotiated = usb_pd_negotiated
                    
        except Exception as e:
            # Silently handle errors in high-frequency polling
            pass
    
    # ---------- High-frequency sail current reading (10Hz) ----------
    def read_sail_current(self):
        """High-frequency reading of sail current for control purposes (10Hz)"""
        if self._shutdown_requested:
            return
            
        try:
            # Read only sail current channel (AIN2) - minimal I2C overhead
            # Increased delay for LP5814 compatibility (was 0.5ms, now 5ms - still fast enough for 10Hz)
            raw2 = self._read_adc_channel_avg(2, repeats=4, settle_delay_s=0.005)
            sail_current = raw2 * self.lsb_value
            
            # Update successful read time
            self._last_successful_read_time = time.monotonic()
            
            # Track sustained recovery - require multiple consecutive successful reads before clearing critical failure
            if self._critical_i2c_failure:
                # We're in critical failure state - track consecutive successful reads
                if self._recovery_start_time is None:
                    self._recovery_start_time = time.monotonic()
                    self._consecutive_successful_reads = 0
                    self.get_logger().info("I2C recovery sequence started - monitoring for sustained recovery...")
                
                self._consecutive_successful_reads += 1
                
                # Check if we have sustained recovery (enough consecutive successful reads)
                if self._consecutive_successful_reads >= self._recovery_required_reads:
                    # Sustained recovery achieved - clear critical failure
                    recovery_duration = time.monotonic() - self._recovery_start_time
                    self._adc_failure_start_time = None
                    self._critical_i2c_failure = False
                    self._consecutive_successful_reads = 0
                    self._recovery_start_time = None
                    self._publish_i2c_failure(False)
                    self._last_i2c_failure_republish_time = 0.0  # Reset republish timer
                    self.get_logger().info(
                        f"✅ I2C recovery: Sustained recovery achieved after {self._recovery_required_reads} "
                        f"consecutive successful reads ({recovery_duration:.1f}s) - critical failure cleared")
                else:
                    # Still in recovery sequence - log progress periodically
                    if self._consecutive_successful_reads % 5 == 0:
                        self.get_logger().info(
                            f"I2C recovery progress: {self._consecutive_successful_reads}/{self._recovery_required_reads} "
                            f"consecutive successful reads")
            else:
                # Not in critical failure - reset recovery tracking
                if self._adc_failure_start_time is not None:
                    self._adc_failure_start_time = None
                self._consecutive_successful_reads = 0
                self._recovery_start_time = None
            
            # If we were unhealthy but now have successful reads, recover to normal mode
            if not self.node_healthy:
                self.node_healthy = True
                self.set_healthy("Automatic recovery successful")
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0
                self.get_logger().info("Automatic recovery: successful reads restored, switching back to normal mode")
            
            self._latest_sail_current = sail_current
            
            # Slow EMA of |sail current| for idle/sailing activity classification
            now_mono = time.monotonic()
            if self._sail_ema_last_time is None or self._sail_current_ema is None:
                self._sail_current_ema = abs(sail_current)
            else:
                dt = max(1e-3, now_mono - self._sail_ema_last_time)
                alpha = min(1.0, dt / SAIL_ACTIVITY_EMA_TAU_S)
                self._sail_current_ema += alpha * (abs(sail_current) - self._sail_current_ema)
            self._sail_ema_last_time = now_mono
            
            if rclpy.ok():
                try:
                    self.pub_sail_current.publish(Float32(data=sail_current))
                except Exception:
                    pass  # Ignore publish errors during shutdown
                    
            self._prev_sail_current = sail_current
            
            # Update service buffer with latest sail current
            self._update_service_buffer()
            
            # Periodically republish I2C failure state if in failure (uses existing timer)
            self._maybe_republish_i2c_failure()
            
        except Exception as e:
            self._handle_io_error(e, "ADC (sail current)")

    # ---------- Low-frequency battery safety sensors (5s) ----------
    def read_battery_safety_sensors(self):
        """Low-frequency reading of battery, saltwater, temperature, humidity (5s interval)"""
        if self._shutdown_requested:
            return

        current_time = time.monotonic()
        
        # Read critical ADC sensors (battery and saltwater) - these are essential
        # CRITICAL: Continue with last known values on failure to allow power control to detect stale data
        adc_read_successful = False
        try:
            # CRITICAL: Re-write setup register before reading if we're getting zeros
            # This may be needed when LP5814 is on the bus (ADC might reset or lose setup)
            # Check if last reading was zero, and if so, re-initialize setup register
            if (self._latest_battery_voltage == 0.0 or 
                (not math.isnan(self._latest_battery_voltage) and self._latest_battery_voltage < 1.0)):
                try:
                    setup_byte = self._build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
                    self._i2c_write(self.adc_addr, setup_byte)
                    time.sleep(0.01)  # Allow ADC to stabilize after setup
                    self.get_logger().debug("Re-initialized ADC setup register (previous reading was zero/low)")
                except Exception as e:
                    self.get_logger().warn(f"Failed to re-initialize ADC setup register: {e}")
            
            # ADC averages for battery and saltwater
            raw0 = self._read_adc_channel_avg(0)
            raw1 = self._read_adc_channel_avg(1)

            battery_voltage = raw0 * self.lsb_value * self.battery_divider_scale
            saltwater_voltage = raw1 * self.lsb_value
            
            # CRITICAL: Detect when ADC returns zeros or suspiciously low values
            # This can happen with power supply issues or ADC reference problems
            # (not I2C failures - those would throw exceptions)
            if raw0 == 0 or (raw0 < 100 and battery_voltage < 1.0):
                # Log raw ADC code for diagnostics
                self.get_logger().warn(
                    f"⚠️ ADC returned suspiciously low value: raw_code={raw0} (0x{raw0:04x}), "
                    f"calculated_voltage={battery_voltage:.3f}V. "
                    f"This may indicate power supply droop, ADC reference issue, or input problem. "
                    f"Raw ADC bytes may be corrupted by bus capacitance.")
                # Mark as stale/invalid to trigger NaN handling downstream
                battery_voltage = float('nan')
                saltwater_voltage = float('nan') if raw1 == 0 else saltwater_voltage
                adc_read_successful = False  # Don't mark as successful
                self._data_stale = True
            else:
                # Valid ADC reading
                # Update successful read time for critical sensors
                self._last_successful_read_time = time.monotonic()
                
                # Publish initial I2C failure state if never published before (for late-joining subscribers)
                if self._last_published_i2c_failure_state is None:
                    self._publish_i2c_failure(self._critical_i2c_failure, force=True)
                    self.get_logger().info(f"Published initial I2C failure state (from sensor read): {self._critical_i2c_failure}")
                
                # Reset ADC failure timer on successful read
                if self._adc_failure_start_time is not None:
                    self._adc_failure_start_time = None
                    if self._critical_i2c_failure:
                        self._critical_i2c_failure = False
                        self._publish_i2c_failure(False)
                        self._last_i2c_failure_republish_time = 0.0  # Reset republish timer
                        self.get_logger().info("✅ I2C recovery: ADC sensor communication restored - critical failure cleared")
                
                # If we were unhealthy but now have successful reads, recover to normal mode
                if not self.node_healthy:
                    self.node_healthy = True
                    self.set_healthy("Automatic recovery successful")
                    self._switch_to_normal_mode()
                    self._recovery_attempt_count = 0
                    self.get_logger().info("Automatic recovery: successful reads restored, switching back to normal mode")

                # Update latest critical sensor values
                self._latest_battery_voltage = battery_voltage
                self._latest_saltwater_voltage = saltwater_voltage
                adc_read_successful = True
                self._data_stale = False  # Data is fresh
            
        except Exception as e:
            self._handle_io_error(e, "ADC sensors")
            # CRITICAL FIX: Continue with last known values instead of returning early
            # This allows power control to detect stale data and make safety decisions
            # Use NaN to indicate I2C failure (more informative than 0.0)
            # Only use last known value if it's valid (not NaN), otherwise use NaN
            if not math.isnan(self._latest_battery_voltage) and self._latest_battery_voltage > 0:
                battery_voltage = self._latest_battery_voltage
            else:
                battery_voltage = float('nan')
            if not math.isnan(self._latest_saltwater_voltage) and self._latest_saltwater_voltage >= 0:
                saltwater_voltage = self._latest_saltwater_voltage
            else:
                saltwater_voltage = float('nan')
            self._data_stale = True  # Mark data as stale
            if not math.isnan(battery_voltage):
                self.get_logger().warn(
                    f"ADC read failed, using last known values: battery={battery_voltage:.3f}V, "
                    f"saltwater={saltwater_voltage:.3f}V (data may be stale)")
            else:
                self.get_logger().warn(
                    f"ADC read failed, no valid previous reading: battery=NaN, saltwater=NaN (I2C failure)")
        
        # Read SHT45 sensor separately - this is non-critical and can fail gracefully
        temperature, humidity = self._read_sht45_robust()
        self._latest_temperature = temperature
        self._latest_humidity = humidity
        
        # Attempt SHT45 recovery if sensor was previously unavailable
        if not self._sht_sensor_available:
            self._attempt_sht45_recovery()

        # Calculate battery remaining percentage (only if we have valid voltage reading)
        battery_remaining_pct = None
        if adc_read_successful:
            battery_remaining_pct = self._soc_from_voltage(battery_voltage)
        # Use last known percentage if ADC read failed
        elif self._latest_battery_remaining_pct is not None:
            battery_remaining_pct = self._latest_battery_remaining_pct
        self._latest_battery_remaining_pct = battery_remaining_pct

        # Check MP2672 availability and read charging status (I2C primary if host control enabled, GPIO fallback)
        if MP2672_HOST_CONTROL:
            self._check_mp2672_availability()
            # Read MP2672 CHG timer configuration if available
            if self.mp2672_available:
                reg02_info = self._read_mp2672_register_02()
                if reg02_info is not None:
                    chg_tmr = reg02_info.get('chg_tmr')
                    # Map CHG timer bits to hours: 00=disable, 01=8hrs, 10=20hrs, 11=12hrs
                    chg_timer_map = {0b00: None, 0b01: 8.0, 0b10: 20.0, 0b11: 12.0}
                    self._mp2672_chg_timer_hours = chg_timer_map.get(chg_tmr, MP2672_CHG_TIMER_HOURS)
        
        charging_status, ac_power_present = self._read_charging_status()
        
        # Track AC power plug-in/plug-out events for MP2672 CHG timer monitoring
        self._track_ac_power_state_change(ac_power_present)
        
        # Calculate remaining charging time (for service response)
        charging_time_remaining = self._calculate_charging_time_remaining()
        if charging_time_remaining is not None:
            # Warn if timer is getting low (< 2 hours remaining)
            if charging_time_remaining < 2.0:
                self.get_logger().warning(
                    f"⚠️ MP2672 CHG timer low: {charging_time_remaining:.2f} hours remaining "
                    f"(timer will expire and disable charging after {self._mp2672_chg_timer_hours:.1f}h)")
            elif charging_time_remaining < 4.0:
                self.get_logger().info(
                    f"MP2672 CHG timer: {charging_time_remaining:.2f} hours remaining")
        
        self._latest_charging_status = charging_status
        self._latest_ac_power_present = ac_power_present
        self._latest_charging_fault_detected = self._charging_fault_detected
        self._latest_charging_fault_frequency = self._charging_fault_frequency
        
        # Read MP2672 register information for service response (when USB power present and host control enabled)
        mp2672_status_info = None
        mp2672_fault_info = None
        battery_missing_fault = False
        
        if MP2672_HOST_CONTROL and self.mp2672_available:
            try:
                # Read status register (REG 03H) for detailed charging state
                status_reg = self._read_mp2672_register(MP2672_REG_STATUS)
                if status_reg is not None:
                    # Parse status register
                    chg_stat = (status_reg >> 4) & 0x03
                    ppm_stat = (status_reg >> 3) & 0x01
                    battfloat_stat = (status_reg >> 2) & 0x01
                    therm_stat = (status_reg >> 1) & 0x01
                    vsys_stat = status_reg & 0x01
                    
                    chg_stat_names = {
                        0b00: 'Not charge',
                        0b01: 'Pre-charge',
                        0b10: 'Constant-current/constant-voltage charge',
                        0b11: 'Charge done'
                    }
                    
                    battery_missing_fault = bool(battfloat_stat)
                    
                    mp2672_status_info = {
                        'raw': status_reg,
                        'hex': f'0x{status_reg:02x}',
                        'chg_stat': chg_stat,
                        'chg_stat_str': chg_stat_names.get(chg_stat, 'Unknown'),
                        'ppm_stat': bool(ppm_stat),
                        'ppm_stat_str': 'VINPPM active (AC/USB power present)' if ppm_stat else 'No AC/USB power',
                        'battfloat_stat': bool(battfloat_stat),
                        'battfloat_stat_str': 'Battery missing or balance cable fault' if battfloat_stat else 'Battery present',
                        'therm_stat': bool(therm_stat),
                        'therm_stat_str': 'Thermal regulation' if therm_stat else 'Normal',
                        'vsys_stat': bool(vsys_stat),
                        'vsys_stat_str': 'In VSYSMIN regulation' if vsys_stat else 'Not in VSYSMIN regulation'
                    }
                
                # Read fault register (REG 04H) for fault conditions
                fault_info = self._read_mp2672_register_04()
                if fault_info is not None:
                    mp2672_fault_info = fault_info
                    
            except Exception as e:
                # Fail gracefully - MP2672 info not critical for basic operation
                self.get_logger().debug(f"Failed to read MP2672 register info: {e}")
        
        # Store MP2672 information for service buffer
        self._mp2672_status_info = mp2672_status_info
        self._mp2672_fault_info = mp2672_fault_info
        self._mp2672_battery_missing_fault = battery_missing_fault

        # Publish all battery safety sensors
        if rclpy.ok():
            try:
                self.pub_battery_voltage.publish(Float32(data=battery_voltage))
                self.pub_saltwater_voltage.publish(Float32(data=saltwater_voltage))
                if battery_remaining_pct is not None:
                    self.pub_battery_remaining_pct.publish(Float32(data=battery_remaining_pct))
                if temperature is not None:
                    self.pub_temperature.publish(Float32(data=temperature))
                if humidity is not None:
                    self.pub_humidity.publish(Float32(data=humidity))
                
                # Update health status based on sensor readings
                saltwater_detected = saltwater_voltage >= SALTWATER_ALERT_THRESHOLD_V
                self._update_battery_water_health(battery_voltage, saltwater_detected, humidity)
            except Exception:
                pass

        # Log charging fault if detected (throttled logging)
        current_time = time.monotonic()
        if self._charging_fault_detected:
            if current_time - self._last_charging_fault_log_time >= self._charging_fault_log_interval:
                freq_str = f" ({self._charging_fault_frequency:.2f} Hz)" if self._charging_fault_frequency is not None else ""
                self.get_logger().warning(
                    f"🔴 CHARGING FAULT DETECTED{freq_str}: MP2672 STAT pin blinking pattern indicates fault "
                    f"(battery missing/OVP/timer/NTC fault). Check charger hardware and battery connections.")
                self._last_charging_fault_log_time = current_time
        
        # Publish GPIO status (always publish for web dashboard, but only log on actual changes)
        if rclpy.ok():
            try:
                # Always publish charging status (web dashboard needs frequent updates)
                if charging_status is not None:
                    self.pub_charging_status.publish(Bool(data=charging_status))
                    # Only log when status actually changes
                    if self._prev_charging_status is not None and charging_status != self._prev_charging_status:
                        self.get_logger().info(f"Charging status changed: {self._prev_charging_status} -> {charging_status}")
                # Always publish AC power status (web dashboard needs frequent updates)
                if ac_power_present is not None:
                    self.pub_ac_power_present.publish(Bool(data=ac_power_present))
                    # Only log when status actually changes
                    if self._prev_ac_power_present is not None and ac_power_present != self._prev_ac_power_present:
                        self.get_logger().info(f"AC power status changed: {self._prev_ac_power_present} -> {ac_power_present}")
            except Exception:
                pass

        # Periodically republish I2C failure state if in failure (uses existing timer)
        self._maybe_republish_i2c_failure()

        # Reset regression buffers on AC presence change: the IR step at plug-in/out
        # makes any window spanning the transition meaningless, and keeping the
        # buffer single-regime is what makes slope learning trustworthy.
        if (self._prev_ac_power_present is not None and ac_power_present is not None and
                ac_power_present != self._prev_ac_power_present):
            self._voltage_samples.clear()
            self._soc_samples.clear()
            self.get_logger().info(
                f"AC power transition ({self._prev_ac_power_present} -> {ac_power_present}): "
                f"cleared slope regression buffers")
        
        # Same reset on idle<->sailing transitions (different load regime)
        activity = self._classify_activity()
        if self._latest_activity_state is not None and activity != self._latest_activity_state:
            self._voltage_samples.clear()
            self._soc_samples.clear()
            self.get_logger().info(
                f"Activity transition ({self._latest_activity_state} -> {activity}): "
                f"cleared slope regression buffers")
        self._latest_activity_state = activity

        # Update previous values
        self._prev_battery_voltage = battery_voltage
        self._prev_saltwater_voltage = saltwater_voltage
        self._prev_temperature = temperature
        self._prev_humidity = humidity
        self._prev_battery_remaining_pct = battery_remaining_pct
        self._prev_charging_status = charging_status
        self._prev_ac_power_present = ac_power_present

        # Add battery voltage + SOC samples for lifetime estimation
        self._voltage_samples.append((current_time, battery_voltage))
        self._soc_samples.append((
            current_time,
            battery_remaining_pct if battery_remaining_pct is not None else float('nan')))

        # Voltage trend over the current (single-regime) buffer
        self._latest_voltage_slope_vph = None
        try:
            fit = self._linear_least_squares(self._voltage_samples)
            if fit is not None:
                self._latest_voltage_slope_vph = fit[0] * 3600.0
        except Exception:
            self._latest_voltage_slope_vph = None

        # Classify power state (voltage trend authoritative; charger GPIO only a hint)
        self._latest_power_state = self._classify_power_state(ac_power_present)
        if self._latest_power_state != self._prev_power_state:
            if self._prev_power_state is not None:
                trend_str = (f", trend={self._latest_voltage_slope_vph:+.3f} V/h"
                             if self._latest_voltage_slope_vph is not None else "")
                self.get_logger().info(
                    f"Power state: {self._prev_power_state} -> {self._latest_power_state} "
                    f"(activity={activity}{trend_str})")
            self._prev_power_state = self._latest_power_state

        # SOC-based lifetime estimates for the current state
        ttf, tte = self._compute_lifetime_estimates(battery_voltage, battery_remaining_pct)
        self._latest_time_to_full_hours = ttf
        self._latest_time_to_empty_hours = tte
        self._latest_battery_lifetime_hours = ttf if ttf is not None else tte
        if self._latest_battery_lifetime_hours is not None and rclpy.ok():
            try:
                self.pub_battery_lifetime_hours.publish(
                    Float32(data=self._latest_battery_lifetime_hours))
            except Exception:
                pass

        # CHARGING ANOMALY: AC present (and/or charger claims charging) but the battery
        # is net-draining. Causes: insufficient USB supply, charger fault (check MP2672
        # fault register / STAT blinking), or load exceeding charger capability.
        self._latest_charging_anomaly = False
        self._latest_anomaly_code = None
        self._latest_anomaly_reason = None
        if (self._latest_power_state == POWER_STATE_PLUGGED_DRAIN and
                self._latest_voltage_slope_vph is not None and
                len(self._voltage_samples) >= self.battery_lifetime_min_samples and
                self._latest_voltage_slope_vph <= MIN_DISCHARGING_SLOPE_FOR_ANOMALY_VPH):
            self._latest_charging_anomaly = True
            self._latest_anomaly_code = "charger_power_fault"
            self._latest_anomaly_reason = (
                f"AC power present but battery draining (trend {self._latest_voltage_slope_vph:.2f} V/h). "
                f"Possible causes: insufficient USB power, charger fault, or excessive load. "
                f"{'Check MP2672 fault register for details.' if self.mp2672_available else 'MP2672 not accessible (USB power not present).'}"
            )

        self._process_alerts(battery_voltage, saltwater_voltage, humidity)
        
        self._log_sensor_states(battery_voltage, battery_remaining_pct, saltwater_voltage, self._latest_sail_current, temperature, humidity, charging_status, ac_power_present)

        self._update_bars(battery_voltage, saltwater_voltage, self._latest_sail_current, temperature, humidity)

        if current_time - self._last_csv_log_time >= self.csv_log_interval:
            # Log to CSV even when ADC fails (mark as stale/unreliable)
            self._log_to_csv(
                battery_voltage, battery_remaining_pct, saltwater_voltage,
                self._latest_sail_current, temperature, humidity,
                self._latest_battery_low_alert, self._latest_saltwater_alert,
                self._latest_humidity_alert, self.health_status,
                charging_status, ac_power_present,
                adc_stale=not adc_read_successful  # Mark as stale if ADC read failed
            )
            self._last_csv_log_time = current_time
        
        # Atomically update service buffer with complete sensor readings
        self._update_service_buffer()

    def _process_alerts(self, battery_voltage, saltwater_voltage, humidity):
        """Process and publish alert states"""
        try:
            # Handle NaN values (I2C failure) - no alerts when voltage is invalid
            if math.isnan(battery_voltage):
                batt_low = False  # Don't trigger alerts on NaN
            else:
                lower = self.batt_low_threshold_v
                upper = lower + self.batt_low_hysteresis_v
                if self._batt_low_prev:
                    batt_low = not (battery_voltage >= upper)
                else:
                    batt_low = (battery_voltage <= lower)
            
            salt_alert = saltwater_voltage >= self.saltwater_alert_threshold_v
            humid_alert = (humidity is not None) and (humidity >= self.humidity_alert_threshold_pct)

            if rclpy.ok():
                try:
                    self.pub_battery_low_alert.publish(Bool(data=bool(batt_low)))
                    self.pub_saltwater_alert.publish(Bool(data=bool(salt_alert)))
                    self.pub_humidity_alert.publish(Bool(data=bool(humid_alert)))
                except Exception:
                    pass

            if batt_low and not self._batt_low_prev:
                if not math.isnan(battery_voltage):
                    lower = self.batt_low_threshold_v
                    self.get_logger().warning(f"Battery low alert: {battery_voltage:.2f} V <= threshold {lower:.2f} V")
            if (not batt_low) and self._batt_low_prev and not math.isnan(battery_voltage):
                lower = self.batt_low_threshold_v
                upper = lower + self.batt_low_hysteresis_v
                if battery_voltage >= upper:
                    self.get_logger().info(f"Battery voltage OK: {battery_voltage:.2f} V >= release {upper:.2f} V")
            if salt_alert and not self._salt_alert_prev:
                self.get_logger().warning(f"Saltwater alert: {saltwater_voltage:.3f} V >= threshold {self.saltwater_alert_threshold_v:.3f} V")
            if humid_alert and not self._humid_alert_prev:
                self.get_logger().warning(f"Humidity alert: {humidity:.1f}% >= threshold {self.humidity_alert_threshold_pct:.1f}%")

            self._batt_low_prev = bool(batt_low)
            self._salt_alert_prev = bool(salt_alert)
            self._humid_alert_prev = bool(humid_alert)

            self._latest_battery_low_alert = bool(batt_low)
            self._latest_saltwater_alert = bool(salt_alert)
            self._latest_humidity_alert = bool(humid_alert)
            self._maybe_emit_battery_sos(bool(batt_low), battery_voltage)
        except Exception:
            pass

    def _maybe_emit_battery_sos(self, batt_low: bool, battery_voltage: float):
        """Play throttled SOS buzzer pattern for low/critical battery state."""
        if not batt_low:
            return
        if math.isnan(battery_voltage):
            return
        now = time.monotonic()
        if (now - self._last_battery_sos_time) < self.battery_sos_throttle_s:
            return
        if not os.path.isfile(self._abeep_script):
            self.get_logger().warning(f"abeep script not found: {self._abeep_script}")
            return
        try:
            subprocess.Popen(
                ['bash', self._abeep_script, '--sos'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self._last_battery_sos_time = now
            is_critical = battery_voltage <= self.batt_critical_threshold_v
            if is_critical:
                self.get_logger().warning(
                    f"Battery CRITICAL ({battery_voltage:.2f} V): triggered SOS buzzer (throttled {self.battery_sos_throttle_s:.0f}s)"
                )
            else:
                self.get_logger().warning(
                    f"Battery LOW ({battery_voltage:.2f} V): triggered SOS buzzer (throttled {self.battery_sos_throttle_s:.0f}s)"
                )
        except Exception as e:
            self.get_logger().warning(f"Failed to trigger battery SOS buzzer: {e}")

    def _log_sensor_states(self, battery_voltage, battery_remaining_pct, saltwater_voltage,
                          sail_current, temperature, humidity, charging_status, ac_power_present):
        """Log sensor states and health status combined (throttled to once per minute)"""
        current_time = time.monotonic()
        time_since_last_log = current_time - self._last_log_time
        
        if time_since_last_log >= 60.0:  # Throttle to once per minute
            battery_pct_str = f"{battery_remaining_pct:.1f}%" if battery_remaining_pct is not None else "N/A"
            temp_humid_str = f"PCB_Temp={temperature:.2f}C, Humidity={humidity:.1f}%" if temperature is not None and humidity is not None else "PCB_Temp/Humidity unavailable"
            
            # Power state + measured trend (state machine output, not GPIO claims)
            state = self._latest_power_state or 'unknown'
            power_str = f", Power={state}"
            if state == POWER_STATE_DISCHARGING and self._latest_activity_state:
                power_str += f"/{self._latest_activity_state}"

            slope_str = ""
            if self._latest_voltage_slope_vph is not None:
                soc_part = (f" ({self._latest_soc_slope_pct_per_h:+.1f} %SOC/h)"
                            if self._latest_soc_slope_pct_per_h is not None else "")
                slope_str = f", Trend={self._latest_voltage_slope_vph:+.3f} V/h{soc_part}"

            # Lifetime: TTF only in true net-charge state, otherwise TTE
            lifetime_str = ""
            if self._latest_time_to_full_hours is not None:
                lifetime_str = f", TTF={self._latest_time_to_full_hours:.2f}h"
            elif self._latest_time_to_empty_hours is not None:
                lifetime_str = f", TTE={self._latest_time_to_empty_hours:.2f}h"

            charging_str = ""
            if charging_status is not None and ac_power_present is not None:
                charging_icon = "🔌" if charging_status else "🔋"
                ac_icon = "⚡" if ac_power_present else "🔌"
                charging_str = f", Charging={charging_icon}{charging_status}, AC={ac_icon}{ac_power_present}"
            elif charging_status is not None:
                charging_icon = "🔌" if charging_status else "🔋"
                charging_str = f", Charging={charging_icon}{charging_status}"
            elif ac_power_present is not None:
                ac_icon = "⚡" if ac_power_present else "🔌"
                charging_str = f", AC={ac_icon}{ac_power_present}"

            # Include health status in combined log
            health_status_str = ""
            if hasattr(self, 'health_status') and hasattr(self, 'health_details'):
                health_icon = "🟢" if self.health_status else "🔴"
                health_status_str = f" | Health: {health_icon} {self.health_details}"
            if os.path.exists(STORAGE_RUNDOWN_FLAG_FILE):
                health_status_str += " | Battery storage rundown to 7.6V"
            
            # Handle NaN values in logging
            battery_voltage_str = f"{battery_voltage:.3f}V" if not math.isnan(battery_voltage) else "NaN (I2C failure)"
            saltwater_voltage_str = f"{saltwater_voltage:.3f}V" if not math.isnan(saltwater_voltage) else "NaN"
            self.get_logger().info(
                f"Status: Battery={battery_voltage_str} ({battery_pct_str}), "
                f"Saltwater={saltwater_voltage_str}, Sail_current={sail_current:.3f}A, "
                f"{temp_humid_str}{charging_str}{power_str}{slope_str}{lifetime_str}{health_status_str}"
            )
            self._last_log_time = current_time

def main(args=None):
    parser = ArgoBaseNode.create_standard_parser(
        'Battery/Water Monitoring Node for Argo',
        epilog="""
This ROS2 node monitors various sensors on the Argo autonomous sailboat:
- MAX11612 ADC: Battery voltage (via voltage divider), saltwater probe, sail winch current
- SHT45: Temperature and humidity sensor
- Calculates battery state-of-charge using LiPo discharge curve
- Publishes alerts for low battery, saltwater detection, and high humidity
- Estimates time to full charge or depletion using linear regression

TOPICS:
  Publishes:
    /battery_voltage: Float32 - Battery voltage in volts
    /saltwater_voltage: Float32 - Saltwater probe voltage in volts  
    /sail_current: Float32 - Sail winch current in amperes
    /temperature_pcb: Float32 - PCB temperature in Celsius (from SHT45 sensor)
    /relative_humidity: Float32 - Relative humidity percentage
    /battery_remaining_pct: Float32 - Battery state-of-charge percentage
    /battery_lifetime_hours: Float32 - Estimated hours to full/empty
    /battery_low_alert: Bool - Battery low voltage alert
    /saltwater_alert: Bool - Saltwater detection alert
    /humidity_alert: Bool - High humidity alert
    /battery_water_health: Bool - Node health status (ArgoBaseNode)

SERVICES:
  /battery_water_node/health: Trigger - Health status service endpoint

HARDWARE:
  - I2C Bus: Uses I2C bus 0 (Orange Pi Zero 2W default)
  - MAX11612 ADC at I2C address 0x34
  - SHT45 sensor at I2C address 0x44
  - GPIO Pins: PC12 (!CHARGING), PH9 (!ACOK)
  - Battery voltage divider: 27k/18k (2.5x scaling)
        """
    )
    parser.add_argument('--test-adc', action='store_true',
                        help='Perform RC decay test on AIN3 and save plot (requires matplotlib)')
    
    # Use custom spin loop instead of ArgoBaseNode.run_node() for better shutdown responsiveness
    node = None
    try:
        # Parse arguments
        parsed_args, unknown_args = parser.parse_known_args(args)
        
        # Initialize ROS2
        rclpy.init(args=unknown_args)
        
        # Create node
        node = BatteryWaterNode()
        
        # Custom spin loop with frequent shutdown checks
        # This allows signal handler to break the loop quickly instead of waiting for rclpy.spin()
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt, shutting down gracefully...")
    except rclpy.executors.ExternalShutdownException:
        if node:
            node.get_logger().info("External shutdown received, exiting gracefully...")
    except Exception as e:
        print(f"CRITICAL: Failed to initialize Battery/Water node: {e}")
        print("CRITICAL: Check I2C permissions and hardware connections.")
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    finally:
        # Clean shutdown
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
