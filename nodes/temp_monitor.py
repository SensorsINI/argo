#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
# System Temperature ROS2 node
# - Reads CPU temperature from thermal zones
# - Monitors system temperature and provides alerts
# Publishes (Float32):
# - cpu_temperature (C) - CPU temperature in Celsius
# - system_temperature (C) - Overall system temperature
# Alerts (Bool):
# - temperature_high_alert - When temperature exceeds high threshold
# - temperature_critical_alert - When temperature exceeds critical threshold
# Health (Bool):
# - temp_monitor_health (true=healthy, false=failed)
# Publishing optimization (saves rosbag space):
# - Samples temperature data at 1 sample per minute (60s intervals)
# - Initial sample occurs immediately after startup
# - Publishes temperature data only when values change >5% OR every 60s (whichever is shorter)
# - Alert topics always publish at 1Hz regardless of optimization for safety
# Debug:
# - ASCII terminal bars when --debug is used
# Key parameters:
# - temperature_high_threshold_c (default 70.0 C), temperature_critical_threshold_c (default 85.0 C)
# - temperature_hysteresis_c (default 2.0 C)

import sys
import os
# Import the shared pause service
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
# Import ArgoBaseNode for standardized functionality
from argo_base_node import ArgoBaseNode
# Removed toggle_pause_service import - temp_monitor node doesn't need pause functionality
import rclpy
# Removed QoS imports - using default QoS only
from std_msgs.msg import Float32, Bool
import time
import sys
import argparse
import argcomplete
import os
import glob
from rclpy.executors import ExternalShutdownException


# constants for high and critical temperatures
HIGH_TEMPERATURE_THRESHOLD_C = 85.0  # makes node unhealthy
CRITICAL_TEMPERATURE_THRESHOLD_C = 100.0
TEMPERATURE_HYSTERESIS_C = 2.0

# Sampling rate constant - 1 sample per minute
SAMPLE_PERIOD_SECONDS = 60.0


class TempMonitorNode(ArgoBaseNode):
    def __init__(self, debug_mode=False):
        super().__init__('temp_monitor_node')

        # Initialize pause service with namespaced name
        # Removed pause service - temp_monitor node doesn't need pause functionality

        self.get_logger().info('Initializing Temperature Monitor node...')

        # Debug flag
        self.debug = ('--debug' in sys.argv)

        # Using default QoS for all publishers

        # Publishers
        self.pub_cpu_temperature = self.create_publisher(
            Float32, 'temperature_cpu', 10)
        # Alert publishers with default QoS
        self.pub_temperature_high_alert = self.create_publisher(
            Bool, 'temperature_high_alert', 10)
        self.pub_temperature_critical_alert = self.create_publisher(
            Bool, 'temperature_critical_alert', 10)
        # Alert previous-state flags for edge-triggered logging
        self._temp_high_prev = False
        self._temp_critical_prev = False

        # Timing and change detection for optimized publishing
        self._startup_time = time.monotonic()
        self._last_publish_time = 0.0
        self._last_log_time = 0.0
        # Previous temperature values for 5% change detection
        self._prev_cpu_temperature = None

        # Temperature thresholds
        self.temp_high_threshold_c = float(self.declare_parameter(
            'temperature_high_threshold_c', HIGH_TEMPERATURE_THRESHOLD_C).value)
        self.temp_critical_threshold_c = float(self.declare_parameter(
            'temperature_critical_threshold_c', CRITICAL_TEMPERATURE_THRESHOLD_C).value)
        self.temp_hysteresis_c = float(self.declare_parameter(
            'temperature_hysteresis_c', TEMPERATURE_HYSTERESIS_C).value)

        # Find thermal zones
        self.thermal_zones = self._find_thermal_zones()
        if not self.thermal_zones:
            self.get_logger().error('No thermal zones found! Temperature monitoring will not work.')
        else:
            self.get_logger().info(
                f'Found thermal zones: {self.thermal_zones}')

        # ASCII visual debug when --debug is passed
        self._vis_ascii = self.debug
        self._vis_initialized = False
        if self._vis_ascii:
            self._init_ascii_vis()

        # Timer: 1 sample per minute
        self.timer = self.create_timer(SAMPLE_PERIOD_SECONDS, self.read_and_publish)
        self.get_logger().info(f'Temperature Monitor node initialized and reading at {1.0/SAMPLE_PERIOD_SECONDS:.3f} Hz (1 sample per {SAMPLE_PERIOD_SECONDS:.0f} seconds).')
        
        # Set initial health status as unhealthy (no temperature reading yet)
        self.set_unhealthy("No temperature reading yet")
        
        # Take initial sample immediately after startup
        self.get_logger().info('Taking initial temperature sample...')
        self.read_and_publish()

    def _find_thermal_zones(self):
        """Find available thermal zones for temperature monitoring"""
        thermal_zones = []
        try:
            # Look for thermal zones in /sys/class/thermal/
            thermal_path = '/sys/class/thermal'
            if os.path.exists(thermal_path):
                for zone_dir in glob.glob(os.path.join(thermal_path, 'thermal_zone*')):
                    if os.path.isdir(zone_dir):
                        # Check if this zone has a type file
                        type_file = os.path.join(zone_dir, 'type')
                        if os.path.exists(type_file):
                            try:
                                with open(type_file, 'r') as f:
                                    zone_type = f.read().strip()
                                # Look for CPU-related thermal zones
                                if 'cpu' in zone_type.lower() or 'soc' in zone_type.lower():
                                    thermal_zones.append(zone_dir)
                                    self.get_logger().info(
                                        f'Found thermal zone: {zone_type} at {zone_dir}')
                            except Exception as e:
                                self.get_logger().debug(
                                    f'Could not read type from {type_file}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error finding thermal zones: {e}')

        return thermal_zones

    def _read_thermal_zone_temp(self, zone_path):
        """Read temperature from a thermal zone"""
        try:
            temp_file = os.path.join(zone_path, 'temp')
            if os.path.exists(temp_file):
                with open(temp_file, 'r') as f:
                    temp_millicelsius = int(f.read().strip())
                return temp_millicelsius / 1000.0  # Convert to Celsius
        except Exception as e:
            self.get_logger().debug(
                f'Error reading temperature from {zone_path}: {e}')
        return None

    def _read_cpu_temperature(self):
        """Read CPU temperature from thermal zones"""
        if not self.thermal_zones:
            return None

        temperatures = []
        for zone in self.thermal_zones:
            temp = self._read_thermal_zone_temp(zone)
            if temp is not None:
                temperatures.append(temp)

        if temperatures:
            # Return the maximum temperature found (hottest CPU core)
            return max(temperatures)
        return None


    def _scale_pct(self, value, vmin, vmax):
        """Scale value to percentage for visualization"""
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, cpu_temp_c):
        """Update ASCII visualization bars"""
        if not self._vis_ascii:
            return

        # Temperature ranges for visualization
        temp_min, temp_max = 20.0, 100.0

        try:
            sys.stdout.write('\x1b[H')  # home
            lines = []

            if cpu_temp_c is not None:
                # Map temp to 0..range for bar
                temp_span = max(1e-6, temp_max - temp_min)
                temp_norm = (max(temp_min, min(temp_max, cpu_temp_c)
                                 ) - temp_min) / temp_span * 100.0
                lines.append(
                    f"CPU    {cpu_temp_c:7.2f} C  " + self._bar(temp_norm, 100.0))

            lines.append("Ctrl-C to exit")
            for ln in lines:
                sys.stdout.write(ln + '\n')
            sys.stdout.flush()
        except Exception:
            pass

    def _init_ascii_vis(self):
        """Initialize ASCII visualization"""
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
        """Clean up ASCII visualization"""
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
        """Create ASCII bar visualization"""
        width = max(10, width)
        v = max(0.0, min(limit, float(value)))
        fill = int(round((v / limit) * width)) if limit > 0 else 0
        if fill > width:
            fill = width
        return '[' + ('#' * fill) + ('-' * (width - fill)) + ']'

    def _has_significant_change(self, current_value, previous_value, threshold_pct=5.0):
        """Check if current value has changed by more than threshold_pct from previous value"""
        if previous_value is None or current_value is None:
            return True  # Always publish if we don't have previous data

        if previous_value == 0.0:
            # Avoid division by zero; consider any non-zero change significant
            return current_value != 0.0

        change_pct = abs((current_value - previous_value) /
                         previous_value) * 100.0
        return change_pct >= threshold_pct

    def read_and_publish(self):
        """Main read and publish function"""
        # Removed pause functionality - temp_monitor runs continuously

        current_time = time.monotonic()
        time_since_startup = current_time - self._startup_time
        time_since_last_publish = current_time - self._last_publish_time
        time_since_last_log = current_time - self._last_log_time

        # Read temperatures
        cpu_temperature = self._read_cpu_temperature()
        
        # Update health status based on temperature safety
        self._update_temp_health_status(cpu_temperature)

        # Determine if we should publish values
        should_publish = False

        # First 30 seconds: publish every cycle (1Hz) and log every 5s
        if time_since_startup <=10.0:
            should_publish = True
            if time_since_last_log >= 5.0:
                # Build temperature state message
                cpu_str = f"CPU={cpu_temperature:.2f}C" if cpu_temperature is not None else "CPU=N/A"

                self.get_logger().info(
                    f"Temperature states: {cpu_str}")
                self._last_log_time = current_time
        else:
            # After 30s: publish only if significant change (>5%) or 60s elapsed
            significant_change = (
                self._has_significant_change(cpu_temperature, self._prev_cpu_temperature)
            )

            if significant_change or time_since_last_publish >= 60.0:
                should_publish = True

        # Publish values if conditions are met
        if should_publish:
            if cpu_temperature is not None:
                self.pub_cpu_temperature.publish(Float32(data=cpu_temperature))

            # Update previous values and timestamp
            self._prev_cpu_temperature = cpu_temperature
            self._last_publish_time = current_time

        # Alerts (always published regardless of temperature data publishing logic)
        try:
            # Temperature alerts with hysteresis
            if cpu_temperature is not None:
                # High temperature alert with hysteresis
                high_lower = self.temp_high_threshold_c
                high_upper = high_lower + self.temp_hysteresis_c
                if self._temp_high_prev:
                    temp_high = not (cpu_temperature <= high_upper)
                else:
                    temp_high = (cpu_temperature >= high_lower)

                # Critical temperature alert with hysteresis
                critical_lower = self.temp_critical_threshold_c
                critical_upper = critical_lower + self.temp_hysteresis_c
                if self._temp_critical_prev:
                    temp_critical = not (cpu_temperature <= critical_upper)
                else:
                    temp_critical = (cpu_temperature >= critical_lower)
            else:
                temp_high = False
                temp_critical = False

            self.pub_temperature_high_alert.publish(Bool(data=bool(temp_high)))
            self.pub_temperature_critical_alert.publish(
                Bool(data=bool(temp_critical)))

            # Edge-triggered warnings
            if temp_high and not self._temp_high_prev:
                self.get_logger().warning(
                    f"High temperature alert: {cpu_temperature:.2f} C >= threshold {self.temp_high_threshold_c:.2f} C"
                )
            if (not temp_high) and self._temp_high_prev and (cpu_temperature <= high_upper):
                self.get_logger().info(
                    f"Temperature normal: {cpu_temperature:.2f} C <= release {high_upper:.2f} C"
                )
            if temp_critical and not self._temp_critical_prev:
                self.get_logger().error(
                    f"CRITICAL temperature alert: {cpu_temperature:.2f} C >= threshold {self.temp_critical_threshold_c:.2f} C"
                )
            if (not temp_critical) and self._temp_critical_prev and (cpu_temperature <= critical_upper):
                self.get_logger().info(
                    f"Temperature below critical: {cpu_temperature:.2f} C <= release {critical_upper:.2f} C"
                )

            # Update previous states
            self._temp_high_prev = bool(temp_high)
            self._temp_critical_prev = bool(temp_critical)
        except Exception as e:
            self.get_logger().error(f"Error in read_and_publish: {e}")

        # Update ASCII bars if enabled
        self._update_bars(cpu_temperature)

    def _update_temp_health_status(self, cpu_temperature):
        """Update health status based on temperature safety"""
        if cpu_temperature is None:
            self.set_unhealthy("No temperature reading available")
            return
        
        # Check if temperature is in safe range (below high threshold)
        if cpu_temperature >= HIGH_TEMPERATURE_THRESHOLD_C:
            self.set_unhealthy(f"Temperature too high: {cpu_temperature:.1f}°C >= {HIGH_TEMPERATURE_THRESHOLD_C:.1f}°C")
        else:
            self.set_healthy(f"Temperature in safe range: {cpu_temperature:.1f}°C")


def main(args=None):
    """Main function using ArgoBaseNode standardized approach"""
    parser = ArgoBaseNode.create_standard_parser(
        'System Temperature ROS2 Node - Monitors CPU temperature',
        epilog="""
This ROS2 node monitors system temperature on the Argo autonomous sailboat:
- Reads CPU temperature from thermal zones at 1 sample per minute
- Takes initial sample immediately after startup
- Monitors system temperature and provides alerts
- Calculates temperature alerts with hysteresis
- Publishes alerts for high and critical temperatures

Topics:
  Publishes:
    /temperature_cpu: Float32 - CPU temperature in Celsius
    /temperature_high_alert: Bool - High temperature alert
    /temperature_critical_alert: Bool - Critical temperature alert
    /temp_monitor_health: Bool - Node health status (ArgoBaseNode)

Services:
  /temp_monitor_node/health: Trigger - Health status service endpoint

Parameters:
  temperature_high_threshold_c: High temperature threshold in Celsius (default: 70.0)
  temperature_critical_threshold_c: Critical temperature threshold in Celsius (default: 85.0)
  temperature_hysteresis_c: Temperature hysteresis in Celsius (default: 2.0)

Options:
  --debug: Enable ASCII terminal visualization of temperature values

Hardware:
  Reads from /sys/class/thermal/thermal_zone* for CPU temperature
        """
    )
    
    try:
        ArgoBaseNode.run_node(TempMonitorNode, args, parser)
    except Exception as e:
        # Handle temperature monitor specific errors
        print(f"CRITICAL: Failed to initialize Temperature Monitor node: {e}")
        print("CRITICAL: Check thermal zone access and permissions.")
        sys.exit(1)


if __name__ == '__main__':
    main()
