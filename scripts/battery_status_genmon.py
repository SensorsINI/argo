#!/usr/bin/env python3
"""
Battery Status Script for XFCE4 Generic Monitor Plugin
Outputs battery status in XML format for the genmon plugin
"""

import sys
import json
import math
from typing import Optional, Dict, Any

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BatteryStatusGenmon(Node):
    def __init__(self):
        super().__init__('battery_status_genmon')
        
        # Create service client for battery status
        self.battery_service_client = self.create_client(Trigger, '/battery_status')
        
        # Wait for service to be available (short timeout)
        if not self.battery_service_client.wait_for_service(timeout_sec=2.0):
            self._output_unavailable()
            return
        
        # Get battery status
        self.get_battery_status()
    
    def _output_unavailable(self):
        """Output service unavailable status - always show ??? so genmon does not keep old value"""
        print('<icon>battery-missing</icon>')
        print('<txt>???</txt>')
        print('<tool>Battery service unavailable</tool>')
    
    def _output_error(self, error_msg: str):
        """Output error status - show ??? so genmon does not keep old value"""
        print('<icon>battery-missing</icon>')
        print('<txt>???</txt>')
        print(f'<tool>Battery service error: {error_msg}</tool>')
    
    def get_battery_status(self):
        """Get and display battery status in genmon XML format"""
        try:
            # Create service request
            request = Trigger.Request()
            
            # Call service with timeout
            future = self.battery_service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    # Parse JSON response
                    battery_data = json.loads(response.message)
                    self._format_genmon_output(battery_data)
                else:
                    self._output_error(response.message)
            else:
                self._output_error("Service call timeout")
                
        except Exception as e:
            self._output_error(str(e))
    
    def _output_invalid(self, reason: str):
        """Output invalid/stale battery data - show ??? so genmon does not show cached or invalid values"""
        print('<icon>battery-missing</icon>')
        print('<txt>???</txt>')
        print(f'<tool>{reason}</tool>')
    
    def _format_genmon_output(self, battery_data: Dict[str, Any]):
        """Format battery data for genmon XML output"""
        try:
            raw_data = battery_data.get('raw_data', {})
            battery_summary = battery_data.get('battery_summary', 'N/A')
            critical_alerts = battery_data.get('critical_alerts')
            
            # Treat I2C failure or stale data as invalid - do not show possibly cached voltage
            if raw_data.get('i2c_failure'):
                self._output_invalid("I2C failure – battery monitoring unavailable")
                return
            if raw_data.get('stale_data'):
                self._output_invalid("Stale data – using last known values (I2C failure)")
                return
            # Invalid or missing battery voltage
            v = raw_data.get('battery_voltage')
            if v is None or (isinstance(v, float) and math.isnan(v)):
                self._output_invalid("Invalid battery voltage (sensor or I2C failure)")
                return
            
            # Determine icon based on status
            if 'LOW BATTERY' in (critical_alerts or ''):
                icon = "battery-low"
                color = "red"
            elif 'SALTWATER' in (critical_alerts or ''):
                icon = "weather-showers"
                color = "blue"
            elif 'HUMIDITY' in (critical_alerts or ''):
                icon = "weather-showers-scattered"
                color = "blue"
            else:
                # Check charging status for icon
                charging_status = raw_data.get('charging_status')
                if charging_status is True:
                    icon = "battery-charging"
                    color = "green"
                else:
                    icon = "battery-good"
                    color = "black"
            
            # Build tooltip with detailed information
            tooltip_lines = []
            tooltip_lines.append(f"Battery: {battery_summary}")
            
            # Add charging status
            charging_status = raw_data.get('charging_status')
            ac_power = raw_data.get('ac_power_present')
            if charging_status is not None:
                charging_text = "Charging" if charging_status else "Not charging"
                tooltip_lines.append(f"Status: {charging_text}")
            if ac_power is not None:
                ac_text = "AC power present" if ac_power else "Battery power"
                tooltip_lines.append(f"Power: {ac_text}")
            
            # Add other sensor data (safe float for NaN/None from service during I2C failure)
            def _f3(x):
                try:
                    v = float(x) if x is not None else 0.0
                    return f"{v:.3f}" if not math.isnan(v) else "—"
                except (TypeError, ValueError):
                    return "—"
            saltwater_v = raw_data.get('saltwater_voltage')
            sail_a = raw_data.get('sail_current')
            tooltip_lines.append(f"Saltwater: {_f3(saltwater_v)}V")
            tooltip_lines.append(f"Sail current: {_f3(sail_a)}A")
            temperature = raw_data.get('pcb_temperature')
            humidity = raw_data.get('relative_humidity')
            if temperature is not None and not math.isnan(temperature):
                tooltip_lines.append(f"Temperature: {temperature:.1f}°C")
            if humidity is not None and not math.isnan(humidity):
                tooltip_lines.append(f"Humidity: {humidity:.1f}%")
            
            # Add critical alerts
            if critical_alerts:
                tooltip_lines.append("")
                tooltip_lines.append("⚠️ ALERTS:")
                tooltip_lines.append(critical_alerts)
            
            # Output genmon XML
            print(f'<icon>{icon}</icon>')
            print(f'<txt><span color="{color}">{battery_summary}</span></txt>')
            
            # Join tooltip lines with newlines
            tooltip_text = '\n'.join(tooltip_lines)
            print(f'<tool>{tooltip_text}</tool>')
            
        except Exception as e:
            self._output_error(f"Format error: {e}")

def main():
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run battery status genmon
        genmon = BatteryStatusGenmon()
        
    except Exception:
        print('<icon>battery-missing</icon>')
        print('<txt>???</txt>')
        print('<tool>Fatal error in battery status script</tool>')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
