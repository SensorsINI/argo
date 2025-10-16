#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
# Battery/Water ROS2 node
# - Reads MAX11612 ADC: AIN0=battery via 27k/18k divider, AIN1=saltwater probe, AIN2=sail winch shunt
# - Reads SHT45 temperature/humidity sensor
# - Monitors MP2672GD charger status via GPIO: PC12 (!CHARGING) and PH9 (!ACOK)
# Publishes (Float32):
# - battery_voltage (V), saltwater_voltage (V), sail_current (A), pcb_temperature (C), relative_humidity (%)
# - battery_remaining_pct (%) using per‑cell LiPo formula: soc% = S − S/(1 + (v/V0)^A)^B
# Publishes (Bool):
# - charging_status (true=charging, false=not charging) - inverted from !CHARGING GPIO
# - ac_power_present (true=AC/USB power present, false=not present) - inverted from !ACOK GPIO
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
# - GPIO Pins: PC12 (pin 36, line 76) !CHARGING, PH9 (pin 26, line 233) !ACOK from MP2672GD

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
import time
import sys
import argparse
import argcomplete
import os
import csv
import json
from datetime import datetime
from rclpy.executors import ExternalShutdownException
from collections import deque
from typing import Optional, Tuple

# Using standard Trigger service - no custom imports needed

# Hardware configuration constants
I2C_BUS_NUMBER = 0  # Orange Pi Zero 2W default I2C bus

# GPIO configuration for MP2672GD charger status
CHARGING_GPIO_LINE = 76   # PC12 (pin 36) - !CHARGING from MP2672GD
ACOK_GPIO_LINE = 233      # PH9 (pin 26) - !ACOK from MP2672GD

# Sample rate configuration - dual timers for different sensor requirements
SAIL_CURRENT_RATE_HZ = 5.0  # Hz for sail current (control critical)
BATTERY_SAFETY_INTERVAL_S = 10.0  # seconds for battery/saltwater/humidity (safety critical)

# only publish if the change is greater than this percentage
THRESHOLD_CHANGE_PCT = 0.1  # Reduced from 1.0 to 0.1% for more frequent publishing

# Battery monitoring thresholds
BATTERY_LOW_THRESHOLD_V = 7.5       # V, ~20% for 2S LiPo, triggers warning
BATTERY_CRITICAL_THRESHOLD_V = 6.8  # V, ~10% for 2S LiPo, may trigger shutdown
BATTERY_FULLY_CHARGED_THRESHOLD_V = 8.2  # V, fully charged, by observation with USB charging pluugged in under full laod (sans servos)

# Battery lifetime estimation configuration
BATTERY_LIFETIME_SAMPLE_WINDOW = 300  # Number of samples for linear regression (default: 300 samples)
BATTERY_LIFETIME_MIN_SAMPLES = 30     # Minimum samples required for estimation (default: 30 samples)
BATTERY_SLOPES_FILE = "battery_slopes.json"  # Persistent storage for charge/discharge slopes

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


class BatteryWaterNode(Node):
    def __init__(self):
        super().__init__('battery_water_node')
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
            Float32, 'pcb_temperature', 10)
        self.pub_humidity = self.create_publisher(
            Float32, 'relative_humidity', 10)
        # Alert publishers for safety-critical alerts
        self.pub_battery_low_alert = self.create_publisher(
            Bool, 'battery_low_alert', 10)
        self.pub_saltwater_alert = self.create_publisher(
            Bool, 'saltwater_alert', 10)
        self.pub_humidity_alert = self.create_publisher(
            Bool, 'humidity_alert', 10)

        # Health status publisher
        self.pub_health = self.create_publisher(
            Bool, 'battery_water_health', 10)
        self.health_status = False  # Track current health status
        # Battery remaining percentage publisher
        self.pub_battery_remaining_pct = self.create_publisher(
            Float32, 'battery_remaining_pct', 10)
        
        # GPIO status publishers for MP2672GD charger monitoring
        self.pub_charging_status = self.create_publisher(
            Bool, 'charging_status', 10)
        self.pub_ac_power_present = self.create_publisher(
            Bool, 'ac_power_present', 10)
        
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

        # Latest sensor values for service response
        self._latest_battery_voltage = 0.0
        self._latest_saltwater_voltage = 0.0
        self._latest_sail_current = 0.0
        self._latest_temperature = None
        self._latest_humidity = None
        self._latest_battery_remaining_pct = None
        self._latest_battery_low_alert = False
        self._latest_saltwater_alert = False
        self._latest_humidity_alert = False
        self._latest_charging_status = None
        self._latest_ac_power_present = None
        self._latest_timestamp = None
        
        # Battery lifetime estimation
        self.battery_lifetime_sample_window = int(
            self.declare_parameter('battery_lifetime_sample_window', BATTERY_LIFETIME_SAMPLE_WINDOW).value)
        self.battery_lifetime_min_samples = int(
            self.declare_parameter('battery_lifetime_min_samples', BATTERY_LIFETIME_MIN_SAMPLES).value)
        
        # Voltage sample history (timestamp, voltage) for lifetime estimation
        self._voltage_samples = deque(maxlen=self.battery_lifetime_sample_window)
        
        # Persistent charging/discharging slopes (V/s) for early estimates
        self._charging_slope = None  # V/s when charging
        self._discharging_slope = None  # V/s when discharging
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
        
        # I2C failure logging throttling
        self._last_i2c_error_log_time = 0.0
        self._i2c_error_log_interval = 30.0  # Log I2C errors max once every 30 seconds
        self._max_failures = 3
        self._shutdown_requested = False
        
        # I2C recovery tracking (similar to imu.py)
        self.node_healthy = True
        self._consecutive_io_errors = 0
        self._last_successful_read_time = time.monotonic()
        self._last_recovery_attempt_time = 0.0
        self._recovery_attempt_count = 0

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

        # GPIO setup for MP2672GD charger status monitoring
        self.gpio_available = False
        self.charging_gpio_line = None
        self.acok_gpio_line = None
        if _HAS_GPIO:
            try:
                # Initialize GPIO chip
                self.gpio_chip = gpiod.Chip("/dev/gpiochip0")
                # Request charging status GPIO line (PC12, line 76)
                self.charging_gpio_line = self.gpio_chip.get_line(CHARGING_GPIO_LINE)
                self.charging_gpio_line.request(consumer="battery_water_node", type=gpiod.LINE_REQ_DIR_IN)
                # Request AC power status GPIO line (PH9, line 233)
                self.acok_gpio_line = self.gpio_chip.get_line(ACOK_GPIO_LINE)
                self.acok_gpio_line.request(consumer="battery_water_node", type=gpiod.LINE_REQ_DIR_IN)
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
        if self._vis_ascii:
            self._init_ascii_vis()

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
            
            # Low-frequency timer for battery safety sensors
            self.battery_safety_timer = self.create_timer(
                BATTERY_SAFETY_INTERVAL_S, self.read_battery_safety_sensors)
            
            self.get_logger().info(
                f'Battery/Water node initialized - Sail current: {SAIL_CURRENT_RATE_HZ}Hz, '
                f'Battery safety: {BATTERY_SAFETY_INTERVAL_S}s intervals')

            # Publish initial health status as healthy
            self._publish_health_status(True)

    def _publish_health_status(self, is_healthy: bool):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            if is_healthy:
                self.get_logger().info("Battery/Water health status: HEALTHY")
            else:
                self.get_logger().warn("Battery/Water health status: FAILED")

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
                    ac_power_present):
        """Log current sensor data to CSV file"""
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

            # Handle None values
            battery_remaining_pct = battery_remaining_pct if battery_remaining_pct is not None else ""
            temperature = temperature if temperature is not None else ""
            humidity = humidity if humidity is not None else ""

            # Write CSV row
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    timestamp, battery_voltage, battery_remaining_pct,
                    saltwater_voltage, sail_current, temperature,
                    humidity, battery_low_csv, saltwater_alert_csv,
                    humidity_alert_csv, health_csv, charging_csv, ac_power_csv
                ])

        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {e}")

    def battery_status_callback(self, request, response):
        """Service callback to provide latest battery and sensor status as JSON string"""
        try:
            # Get current timestamp
            now = self.get_clock().now()

            # Build battery data dictionary
            battery_data = {
                'battery_voltage': self._latest_battery_voltage,
                'saltwater_voltage': self._latest_saltwater_voltage,
                'sail_current': self._latest_sail_current,
                'pcb_temperature': self._latest_temperature if self._latest_temperature is not None else 0.0,
                'relative_humidity': self._latest_humidity if self._latest_humidity is not None else 0.0,
                'battery_remaining_pct': self._latest_battery_remaining_pct if self._latest_battery_remaining_pct is not None else 0.0,
                'battery_low_alert': self._latest_battery_low_alert,
                'saltwater_alert': self._latest_saltwater_alert,
                'humidity_alert': self._latest_humidity_alert,
                'charging_status': self._latest_charging_status if self._latest_charging_status is not None else False,
                'ac_power_present': self._latest_ac_power_present if self._latest_ac_power_present is not None else False,
                'battery_water_health': self.health_status,
                'timestamp_sec': now.seconds_nanoseconds()[0],
                'timestamp_nanosec': now.seconds_nanoseconds()[1],
                'time_to_full_hours': self._latest_time_to_full_hours,
                'time_to_empty_hours': self._latest_time_to_empty_hours
            }

            # Format battery summary
            battery_summary = None
            voltage = battery_data['battery_voltage']
            percent = battery_data['battery_remaining_pct']

            if voltage is not None and percent is not None:
                battery_summary = f"{voltage:.1f}V ({percent:.0f}%)"
            elif voltage is not None:
                battery_summary = f"{voltage:.1f}V"
            elif percent is not None:
                battery_summary = f"{percent:.0f}%"

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

            critical_alerts = " | ".join(
                active_alerts) if active_alerts else None

            # Create final response data
            response_data = {
                'battery_summary': battery_summary,
                'critical_alerts': critical_alerts,
                'raw_data': battery_data
            }

            # Convert to JSON string and return in Trigger response
            response.success = True
            response.message = json.dumps(response_data, indent=2)

            self.get_logger().debug(f"Battery status service called - returning formatted data")
            return response

        except Exception as e:
            self.get_logger().error(f"Error in battery status service: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def _load_battery_slopes(self):
        """Load persistent battery charging/discharging slopes from file"""
        try:
            if os.path.exists(self._slopes_file_path):
                with open(self._slopes_file_path, 'r') as f:
                    slopes_data = json.load(f)
                    self._charging_slope = slopes_data.get('charging_slope')
                    self._discharging_slope = slopes_data.get('discharging_slope')
                    self.get_logger().info(
                        f"Loaded battery slopes: charging={self._charging_slope:.6f} V/s, "
                        f"discharging={self._discharging_slope:.6f} V/s" if self._charging_slope and self._discharging_slope else
                        "Loaded partial battery slopes from persistent storage"
                    )
        except Exception as e:
            self.get_logger().warning(f"Failed to load battery slopes: {e}")
    
    def _save_battery_slopes(self):
        """Save current battery charging/discharging slopes to persistent storage"""
        try:
            if self._charging_slope is not None or self._discharging_slope is not None:
                slopes_data = {
                    'charging_slope': self._charging_slope,
                    'discharging_slope': self._discharging_slope,
                    'timestamp': datetime.now().isoformat()
                }
                with open(self._slopes_file_path, 'w') as f:
                    json.dump(slopes_data, f, indent=2)
                self.get_logger().info(
                    f"Saved battery slopes: charging={self._charging_slope:.6f} V/s, "
                    f"discharging={self._discharging_slope:.6f} V/s" if self._charging_slope and self._discharging_slope else
                    "Saved partial battery slopes to persistent storage"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to save battery slopes: {e}")
    
    def _linear_least_squares(self, samples: deque) -> Optional[Tuple[float, float]]:
        """
        Compute linear least squares fit for voltage samples.
        
        Args:
            samples: deque of (timestamp, voltage) tuples
        
        Returns:
            Tuple of (slope, intercept) in V/s and V, or None if insufficient samples
        """
        if len(samples) < self.battery_lifetime_min_samples:
            return None
        
        try:
            # Extract timestamps and voltages
            n = len(samples)
            t0 = samples[0][0]  # Reference time for numerical stability
            times = [t - t0 for t, v in samples]
            voltages = [v for t, v in samples]
            
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
    
    def _estimate_battery_lifetime(self, voltage: float, charging: bool) -> Optional[float]:
        """
        Estimate time to full charge or depletion in hours.
        
        Args:
            voltage: Current battery voltage
            charging: True if charging, False if discharging
        
        Returns:
            Time in hours, or None if estimation not possible
        """
        try:
            # Use linear regression on recent samples
            fit_result = self._linear_least_squares(self._voltage_samples)
            
            if fit_result is not None:
                slope, intercept = fit_result
                
                # Update persistent slopes if we have good data
                if charging and slope > 1e-6:  # Positive slope for charging
                    self._charging_slope = slope
                elif not charging and slope < -1e-6:  # Negative slope for discharging
                    self._discharging_slope = slope
            else:
                # Use persistent slopes if available
                if charging:
                    slope = self._charging_slope
                else:
                    slope = self._discharging_slope
                
                if slope is None:
                    return None
            
            # Compute time to target voltage
            if charging:
                # Time to BATTERY_FULLY_CHARGED_THRESHOLD_V
                target_voltage = BATTERY_FULLY_CHARGED_THRESHOLD_V
                if slope <= 1e-6:  # Not charging or charging too slowly
                    return None
                if voltage >= target_voltage:
                    return 0.0  # Already at target
                time_seconds = (target_voltage - voltage) / slope
            else:
                # Time to 0% (approximate as 6.0V for 2S LiPo, empty)
                target_voltage = 6.0  # Conservative empty voltage
                if slope >= -1e-6:  # Not discharging or discharging too slowly
                    return None
                if voltage <= target_voltage:
                    return 0.0  # Already depleted
                time_seconds = (target_voltage - voltage) / slope
            
            # Convert to hours and clamp to reasonable range
            time_hours = time_seconds / 3600.0
            if time_hours < 0 or time_hours > 1000:  # Sanity check
                return None
            
            return time_hours
        
        except Exception as e:
            self.get_logger().error(f"Battery lifetime estimation failed: {e}")
            return None

    # ---------- I2C helpers ----------
    def _i2c_write(self, addr: int, byte_val: int) -> None:
        if self.use_smbus2 and i2c_msg is not None:
            msg = i2c_msg.write(addr, bytes([byte_val & 0xFF]))
            with smbus2.SMBus(I2C_BUS_NUMBER) as b:
                b.i2c_rdwr(msg)
        else:
            raise RuntimeError('smbus2 required for this node')

    def _i2c_read(self, addr: int, n: int) -> list:
        if self.use_smbus2 and i2c_msg is not None:
            r = i2c_msg.read(addr, n)
            with smbus2.SMBus(I2C_BUS_NUMBER) as b:
                b.i2c_rdwr(r)
            return list(r)
        else:
            raise RuntimeError('smbus2 required for this node')

    def _i2c_write_read(self, addr: int, write_byte: int, n: int) -> list:
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

    def _read_adc_channel_avg(self, ch: int, repeats: int = 8, settle_delay_s: float = 0.001) -> int:
        # SCAN=01: convert selected input eight times
        cfg = self._build_config(reg=0, scan=0b01, cs=ch, sgl_dif=1)
        acc = 0
        self._i2c_write(self.adc_addr, cfg)
        time.sleep(settle_delay_s)
        for _ in range(repeats):
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
        # pure write command
        w = i2c_msg.write(self.sht_addr, [self.SHT45_HIGH_PRECISION_CMD])
        with smbus2.SMBus(I2C_BUS_NUMBER) as b:
            b.i2c_rdwr(w)
        time.sleep(self.SHT45_MEASUREMENT_DELAY)
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

    # ---------- GPIO reading helpers ----------
    def _read_gpio_status(self):
        """Read GPIO status from MP2672GD charger"""
        charging_status = None
        ac_power_present = None
        
        if not self.gpio_available:
            return charging_status, ac_power_present
            
        try:
            # Read !CHARGING GPIO (PC12, line 76) - invert logic for charging status
            if self.charging_gpio_line is not None:
                charging_gpio_value = self.charging_gpio_line.get_value()
                charging_status = not charging_gpio_value  # Invert: !CHARGING=0 means charging=True
                
            # Read !ACOK GPIO (PH9, line 233) - invert logic for AC power present
            if self.acok_gpio_line is not None:
                acok_gpio_value = self.acok_gpio_line.get_value()
                ac_power_present = not acok_gpio_value  # Invert: !ACOK=0 means AC power present=True
                
        except Exception as e:
            self.get_logger().error(f'GPIO read failed: {e}')
            
        return charging_status, ac_power_present

    # ---------- I2C Recovery Methods (similar to imu.py) ----------
    def _handle_io_error(self, error, sensor_name="I2C"):
        """Handle I2C IOError with health tracking and throttled logging"""
        current_time = time.monotonic()
        self._consecutive_io_errors += 1

        # Log error with throttling (max once per 5 seconds)
        if current_time - self._last_i2c_error_log_time >= 5.0:
            self.get_logger().warn(
                f"Transient {sensor_name} error (attempt {self._consecutive_io_errors}): {error}")
            self._last_i2c_error_log_time = current_time

        # Mark node as unhealthy after first IO error
        if self.node_healthy:
            self.node_healthy = False
            self._publish_health_status(False)
            self.get_logger().warn(
                f"Node health set to UNHEALTHY due to {sensor_name} errors. Switching to low-frequency retry mode.")
            # Switch to low-frequency retry mode
            self._switch_to_retry_mode()

    def _switch_to_retry_mode(self):
        """Switch timers to low-frequency retry mode"""
        # Cancel existing timers
        if hasattr(self, 'sail_current_timer'):
            self.sail_current_timer.cancel()
        if hasattr(self, 'battery_safety_timer'):
            self.battery_safety_timer.cancel()
        
        # Create single low-frequency retry timer (1 Hz)
        self.retry_timer = self.create_timer(1.0, self._retry_callback)
        self.get_logger().info("Switched to 1Hz retry mode for I2C recovery")
        
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
        
        self.get_logger().info("Switched back to normal mode - I2C communication recovered")

    def _reinitialize_sensors(self):
        """Re-initialize ADC and SHT45 sensors after I2C recovery"""
        self.get_logger().info("Re-initializing ADC and SHT45 sensors...")
        
        try:
            # Re-initialize ADC
            setup_byte = self._build_setup(
                reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(self.adc_addr, setup_byte)
            time.sleep(0.05)
            
            # SHT45 doesn't need re-init, but we can verify communication
            # by attempting a read (the actual read will be done in normal operation)
            
            self.get_logger().info("Sensor re-initialization successful")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Sensor re-initialization failed: {e}")
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

            self.get_logger().info(
                f"Attempting sensor re-initialization (attempt {self._recovery_attempt_count})...")
            
            if self._reinitialize_sensors():
                self.node_healthy = True
                self._publish_health_status(True)
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0  # Reset counter on success
                self.get_logger().info(
                    f"Sensor re-initialization successful after {self._consecutive_io_errors} I2C errors")
            else:
                self.get_logger().error(
                    f"Sensor re-initialization failed (attempt {self._recovery_attempt_count}), staying in retry mode")

    def _retry_callback(self):
        """Retry callback for low-frequency I2C recovery attempts"""
        if self._shutdown_requested:
            return
        
        # Check for recovery
        self._check_io_recovery()
        
        # Try to read sensors (will mark as healthy if successful)
        try:
            # Try a simple ADC read to test I2C
            raw0 = self._read_adc_channel_avg(0, repeats=4, settle_delay_s=0.001)
            if raw0 > 0:  # Successful read
                self._last_successful_read_time = time.monotonic()
                self._consecutive_io_errors = 0
                
                # Automatic recovery on successful read
                if not self.node_healthy:
                    self.node_healthy = True
                    self._publish_health_status(True)
                    self._switch_to_normal_mode()
                    self._recovery_attempt_count = 0
                    self.get_logger().info("Automatic recovery: successful reads restored, switching back to normal mode")
        except Exception as e:
            # Continue in retry mode
            pass

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
                    3, repeats=1, settle_delay_s=0.0005)
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
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, bat_v, sw_v, cur_a, temp_c, humid_pct):
        if not self._vis_ascii:
            return
        # Nominal ranges
        bat_max = 12.0
        sw_max = 4.2
        cur_max = 4.2
        t_min, t_max = -20.0, 60.0
        h_min, h_max = 0.0, 100.0
        try:
            sys.stdout.write('\x1b[H')  # home
            lines = [
                f"Battery {bat_v:7.3f} V  " + self._bar(bat_v, bat_max),
                f"Salt   {sw_v:7.3f} V  " + self._bar(sw_v, sw_max),
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
        v = max(0.0, min(limit, float(value)))
        fill = int(round((v / limit) * width)) if limit > 0 else 0
        if fill > width:
            fill = width
        return '[' + ('#' * fill) + ('-' * (width - fill)) + ']'

    def _request_shutdown(self):
        """Request node shutdown due to critical sensor failure"""
        self._shutdown_requested = True
        self.get_logger().fatal("Node shutting down due to critical sensor failure")
        # Publish health status as failed
        self._publish_health_status(False)
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

    # ---------- High-frequency sail current reading (10Hz) ----------
    def read_sail_current(self):
        """High-frequency reading of sail current for control purposes (10Hz)"""
        if self._shutdown_requested:
            return
            
        try:
            # Read only sail current channel (AIN2) - minimal I2C overhead
            raw2 = self._read_adc_channel_avg(2, repeats=4, settle_delay_s=0.0005)  # Faster for 10Hz
            sail_current = raw2 * self.lsb_value
            
            # Update successful read time
            self._last_successful_read_time = time.monotonic()
            
            # If we were unhealthy but now have successful reads, recover to normal mode
            if not self.node_healthy:
                self.node_healthy = True
                self._publish_health_status(True)
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0
                self.get_logger().info("Automatic recovery: successful reads restored, switching back to normal mode")
            
            self._latest_sail_current = sail_current
            
            if rclpy.ok():
                try:
                    self.pub_sail_current.publish(Float32(data=sail_current))
                except Exception:
                    pass  # Ignore publish errors during shutdown
                    
            self._prev_sail_current = sail_current
            
        except Exception as e:
            self._handle_io_error(e, "ADC (sail current)")

    # ---------- Low-frequency battery safety sensors (30s) ----------
    def read_battery_safety_sensors(self):
        """Low-frequency reading of battery, saltwater, temperature, humidity (30s interval)"""
        if self._shutdown_requested:
            return

        current_time = time.monotonic()
        
        try:
            # ADC averages for battery and saltwater
            raw0 = self._read_adc_channel_avg(0)
            raw1 = self._read_adc_channel_avg(1)

            battery_voltage = raw0 * self.lsb_value * self.battery_divider_scale
            saltwater_voltage = raw1 * self.lsb_value

            # SHT45
            temperature, humidity = self._read_sht45()
            
            # Update successful read time
            self._last_successful_read_time = time.monotonic()
            
            # If we were unhealthy but now have successful reads, recover to normal mode
            if not self.node_healthy:
                self.node_healthy = True
                self._publish_health_status(True)
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0
                self.get_logger().info("Automatic recovery: successful reads restored, switching back to normal mode")

            # Update latest sensor values
            self._latest_battery_voltage = battery_voltage
            self._latest_saltwater_voltage = saltwater_voltage
            self._latest_temperature = temperature
            self._latest_humidity = humidity
            
        except Exception as e:
            self._handle_io_error(e, "I2C sensors")
            return  # Skip publishing on error

        # Calculate battery remaining percentage
        battery_remaining_pct = None
        try:
            cells = max(1, int(self.batt_series_cells))
            v_cell = battery_voltage / float(cells) if cells > 0 else battery_voltage
            base = 1.0 + (max(0.0, v_cell) / max(1e-9, self.soc_V0)) ** self.soc_A
            soc = self.soc_S - (self.soc_S / (base ** self.soc_B))
            battery_remaining_pct = float(max(0.0, min(100.0, soc)))
        except Exception:
            pass
        self._latest_battery_remaining_pct = battery_remaining_pct

        # Read GPIO status
        charging_status, ac_power_present = self._read_gpio_status()
        self._latest_charging_status = charging_status
        self._latest_ac_power_present = ac_power_present

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
                self._publish_health_status(True)
            except Exception:
                pass

        # Publish GPIO status on change
        if rclpy.ok():
            try:
                if charging_status is not None and charging_status != self._prev_charging_status:
                    self.pub_charging_status.publish(Bool(data=charging_status))
                    if self._prev_charging_status is not None:
                        self.get_logger().info(f"Charging status changed: {charging_status}")
                if ac_power_present is not None and ac_power_present != self._prev_ac_power_present:
                    self.pub_ac_power_present.publish(Bool(data=ac_power_present))
                    if self._prev_ac_power_present is not None:
                        self.get_logger().info(f"AC power status changed: {ac_power_present}")
            except Exception:
                pass

        # Update previous values
        self._prev_battery_voltage = battery_voltage
        self._prev_saltwater_voltage = saltwater_voltage
        self._prev_temperature = temperature
        self._prev_humidity = humidity
        self._prev_battery_remaining_pct = battery_remaining_pct
        self._prev_charging_status = charging_status
        self._prev_ac_power_present = ac_power_present

        # Add battery voltage sample for lifetime estimation
        self._voltage_samples.append((current_time, battery_voltage))
        
        # Compute battery lifetime estimates
        if charging_status is not None:
            if charging_status:
                # Charging: estimate time to full
                self._latest_time_to_full_hours = self._estimate_battery_lifetime(
                    battery_voltage, charging=True)
                self._latest_time_to_empty_hours = None
                self._latest_battery_lifetime_hours = self._latest_time_to_full_hours
            else:
                # Discharging: estimate time to empty
                self._latest_time_to_full_hours = None
                self._latest_time_to_empty_hours = self._estimate_battery_lifetime(
                    battery_voltage, charging=False)
                self._latest_battery_lifetime_hours = self._latest_time_to_empty_hours
            
            # Publish battery lifetime hours topic
            if self._latest_battery_lifetime_hours is not None and rclpy.ok():
                try:
                    self.pub_battery_lifetime_hours.publish(
                        Float32(data=self._latest_battery_lifetime_hours))
                except Exception:
                    pass

        self._process_alerts(battery_voltage, saltwater_voltage, humidity)
        
        self._log_sensor_states(battery_voltage, battery_remaining_pct, saltwater_voltage, self._latest_sail_current, temperature, humidity, charging_status, ac_power_present)

        self._update_bars(battery_voltage, saltwater_voltage, self._latest_sail_current, temperature, humidity)

        if current_time - self._last_csv_log_time >= self.csv_log_interval:
            self._log_to_csv(
                battery_voltage, battery_remaining_pct, saltwater_voltage,
                self._latest_sail_current, temperature, humidity,
                self._latest_battery_low_alert, self._latest_saltwater_alert,
                self._latest_humidity_alert, self.health_status,
                charging_status, ac_power_present
            )
            self._last_csv_log_time = current_time

    def _process_alerts(self, battery_voltage, saltwater_voltage, humidity):
        """Process and publish alert states"""
        try:
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
                self.get_logger().warning(f"Battery low alert: {battery_voltage:.2f} V <= threshold {lower:.2f} V")
            if (not batt_low) and self._batt_low_prev and (battery_voltage >= upper):
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
        except Exception:
            pass

    def _log_sensor_states(self, battery_voltage, battery_remaining_pct, saltwater_voltage,
                          sail_current, temperature, humidity, charging_status, ac_power_present):
        """Log sensor states for debugging"""
        current_time = time.monotonic()
        time_since_last_log = current_time - self._last_log_time
        
        if time_since_last_log >= 30.0:
            battery_pct_str = f"{battery_remaining_pct:.1f}%" if battery_remaining_pct is not None else "N/A"
            temp_humid_str = f"PCB_Temp={temperature:.2f}C, Humidity={humidity:.1f}%" if temperature is not None and humidity is not None else "PCB_Temp/Humidity unavailable"
            
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

            self.get_logger().info(
                f"Sensor states: Battery={battery_voltage:.3f}V ({battery_pct_str}), "
                f"Saltwater={saltwater_voltage:.3f}V, Sail_current={sail_current:.3f}A, "
                f"{temp_humid_str}{charging_str}"
            )
            self._last_log_time = current_time

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Battery/Water ROS2 Node - Monitors battery, saltwater, and environmental sensors',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node monitors various sensors on the Argo autonomous sailboat:
- MAX11612 ADC: Battery voltage (via voltage divider), saltwater probe, sail winch current
- SHT45: Temperature and humidity sensor
- Calculates battery state-of-charge using LiPo discharge curve
- Publishes alerts for low battery, saltwater detection, and high humidity
- Estimates time to full charge or depletion using linear regression

Topics:
  Publishes:
    /battery_voltage: Float32 - Battery voltage in volts
    /saltwater_voltage: Float32 - Saltwater probe voltage in volts  
    /sail_current: Float32 - Sail winch current in amperes
    /pcb_temperature: Float32 - PCB temperature in Celsius (from SHT45 sensor)
    /relative_humidity: Float32 - Relative humidity percentage
    /battery_remaining_pct: Float32 - Battery state-of-charge percentage
    /battery_lifetime_hours: Float32 - Estimated hours to full/empty
    /battery_low_alert: Bool - Battery low voltage alert
    /saltwater_alert: Bool - Saltwater detection alert
    /humidity_alert: Bool - High humidity alert

Parameters:
  battery_low_threshold_v: Low battery threshold in volts (default: 7.2)
  saltwater_alert_threshold_v: Saltwater detection threshold in volts (default: 1.0)
  humidity_alert_threshold_pct: High humidity threshold percentage (default: 75.0)
  battery_series_cells: Number of battery cells in series (default: 2)
  soc_S, soc_V0, soc_A, soc_B: LiPo state-of-charge curve parameters
  battery_lifetime_sample_window: Sample window size for lifetime estimation (default: 300)
  battery_lifetime_min_samples: Minimum samples for estimation (default: 30)

Options:
  --debug: Enable ASCII terminal visualization of sensor values
  --test-adc: Perform RC decay test on AIN3 and save plot (requires matplotlib)

Hardware:
  MAX11612 ADC at I2C address 0x34
  SHT45 temperature/humidity sensor at I2C address 0x44
  Battery voltage divider: 27k/18k (2.5x scaling)
        """
    )
    parser.add_argument('--debug', action='store_true',
                        help='Enable ASCII terminal visualization of sensor values')
    parser.add_argument('--test-adc', action='store_true',
                        help='Perform RC decay test on AIN3 and save plot (requires matplotlib)')

    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args(args)

    # Initialize ROS2 with remaining arguments
    rclpy.init(args=unknown_args)
    node = None
    try:
        node = BatteryWaterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if node:
            try:
                # Save battery slopes on shutdown
                if hasattr(node, '_save_battery_slopes'):
                    node._save_battery_slopes()
            except Exception as e:
                print(f"Error saving battery slopes: {e}")
            try:
                if hasattr(node, '_teardown_ascii_vis'):
                    node._teardown_ascii_vis()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
