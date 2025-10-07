#!/usr/bin/env python3
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
# - First 30s: publishes all temperature data at 1Hz, logs temperature states every 5s via ROS info
# - After 30s: publishes temperature data only when values change >5% OR every 60s (whichever is shorter)
# - Alert topics always publish at 1Hz regardless of optimization for safety
# Debug:
# - ASCII terminal bars when --debug is used
# Key parameters:
# - temperature_high_threshold_c (default 70.0 C), temperature_critical_threshold_c (default 85.0 C)
# - temperature_hysteresis_c (default 2.0 C)

from toggle_pause_service import TogglePauseService
import rclpy
from rclpy.node import Node
# Removed QoS imports - using default QoS only
from std_msgs.msg import Float32, Bool
import time
import sys
import argparse
import os
import glob
from rclpy.executors import ExternalShutdownException

# Import the shared pause service
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))

# constants for high and critical temperatures
HIGH_TEMPERATURE_THRESHOLD_C = 85.0
CRITICAL_TEMPERATURE_THRESHOLD_C = 100.0
TEMPERATURE_HYSTERESIS_C = 2.0


class TempMonitorNode(Node):
    def __init__(self):
        super().__init__('temp_monitor_node')

        # Initialize pause service with namespaced name
        self.pause_service = TogglePauseService(
            self, f'{self.get_name()}/toggle_pause')

        self.get_logger().info('Initializing Temperature Monitor node...')

        # Debug flag
        self.debug = ('--debug' in sys.argv)

        # Using default QoS for all publishers

        # Publishers
        self.pub_cpu_temperature = self.create_publisher(
            Float32, 'cpu_temperature', 10)
        self.pub_system_temperature = self.create_publisher(
            Float32, 'system_temperature', 10)
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
        self._prev_system_temperature = None

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

        # Timer: 1 Hz
        self.timer = self.create_timer(1.0, self.read_and_publish)
        self.get_logger().info('Temperature Monitor node initialized and reading at 1 Hz.')

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

    def _read_system_temperature(self):
        """Read overall system temperature (same as CPU for now)"""
        return self._read_cpu_temperature()

    def _scale_pct(self, value, vmin, vmax):
        """Scale value to percentage for visualization"""
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, cpu_temp_c, sys_temp_c):
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

            if sys_temp_c is not None:
                temp_span = max(1e-6, temp_max - temp_min)
                temp_norm = (max(temp_min, min(temp_max, sys_temp_c)
                                 ) - temp_min) / temp_span * 100.0
                lines.append(
                    f"System {sys_temp_c:7.2f} C  " + self._bar(temp_norm, 100.0))

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
        # Check if node is paused
        if self.pause_service.is_paused():
            return  # Skip processing when paused

        current_time = time.monotonic()
        time_since_startup = current_time - self._startup_time
        time_since_last_publish = current_time - self._last_publish_time
        time_since_last_log = current_time - self._last_log_time

        # Read temperatures
        cpu_temperature = self._read_cpu_temperature()
        system_temperature = self._read_system_temperature()

        # Determine if we should publish values
        should_publish = False

        # First 30 seconds: publish every cycle (1Hz) and log every 5s
        if time_since_startup <= 30.0:
            should_publish = True
            if time_since_last_log >= 5.0:
                # Build temperature state message
                cpu_str = f"CPU={cpu_temperature:.2f}C" if cpu_temperature is not None else "CPU=N/A"
                sys_str = f"System={system_temperature:.2f}C" if system_temperature is not None else "System=N/A"

                self.get_logger().info(
                    f"Temperature states: {cpu_str}, {sys_str}")
                self._last_log_time = current_time
        else:
            # After 30s: publish only if significant change (>5%) or 60s elapsed
            significant_change = (
                self._has_significant_change(cpu_temperature, self._prev_cpu_temperature) or
                self._has_significant_change(
                    system_temperature, self._prev_system_temperature)
            )

            if significant_change or time_since_last_publish >= 60.0:
                should_publish = True

        # Publish values if conditions are met
        if should_publish:
            if cpu_temperature is not None:
                self.pub_cpu_temperature.publish(Float32(data=cpu_temperature))
            if system_temperature is not None:
                self.pub_system_temperature.publish(
                    Float32(data=system_temperature))

            # Update previous values and timestamp
            self._prev_cpu_temperature = cpu_temperature
            self._prev_system_temperature = system_temperature
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
        self._update_bars(cpu_temperature, system_temperature)


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='System Temperature ROS2 Node - Monitors CPU and system temperature',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node monitors system temperature on the Argo autonomous sailboat:
- Reads CPU temperature from thermal zones
- Monitors system temperature and provides alerts
- Calculates temperature alerts with hysteresis
- Publishes alerts for high and critical temperatures

Topics:
  Publishes:
    /cpu_temperature: Float32 - CPU temperature in Celsius
    /system_temperature: Float32 - System temperature in Celsius
    /temperature_high_alert: Bool - High temperature alert
    /temperature_critical_alert: Bool - Critical temperature alert

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
    parser.add_argument('--debug', action='store_true',
                        help='Enable ASCII terminal visualization of temperature values')

    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args(args)

    # Initialize ROS2 with remaining arguments
    rclpy.init(args=unknown_args)
    node = TempMonitorNode()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            # Quiet shutdown to avoid traceback
            try:
                if hasattr(node, '_teardown_ascii_vis'):
                    node._teardown_ascii_vis()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
        except ExternalShutdownException:
            try:
                if hasattr(node, '_teardown_ascii_vis'):
                    node._teardown_ascii_vis()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
        else:
            # Normal shutdown path
            try:
                if hasattr(node, '_teardown_ascii_vis'):
                    node._teardown_ascii_vis()
            except Exception:
                pass
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
