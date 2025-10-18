#!/usr/bin/env python3
"""
Battery Status Script for XFCE4 Generic Monitor Plugin
Outputs battery status in XML format for the genmon plugin
"""

import sys
import json
import time
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
        """Output service unavailable status"""
        print('<icon>battery-missing</icon>')
        print('<txt>N/A</txt>')
        print('<tool>Battery service unavailable</tool>')
    
    def _output_error(self, error_msg: str):
        """Output error status"""
        print('<icon>battery-missing</icon>')
        print('<txt>Error</txt>')
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
    
    def _format_genmon_output(self, battery_data: Dict[str, Any]):
        """Format battery data for genmon XML output"""
        try:
            raw_data = battery_data.get('raw_data', {})
            battery_summary = battery_data.get('battery_summary', 'N/A')
            critical_alerts = battery_data.get('critical_alerts')
            
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
            
            # Add other sensor data
            saltwater_voltage = raw_data.get('saltwater_voltage', 0)
            sail_current = raw_data.get('sail_current', 0)
            temperature = raw_data.get('pcb_temperature')
            humidity = raw_data.get('relative_humidity')
            
            tooltip_lines.append(f"Saltwater: {saltwater_voltage:.3f}V")
            tooltip_lines.append(f"Sail current: {sail_current:.3f}A")
            
            if temperature is not None:
                tooltip_lines.append(f"Temperature: {temperature:.1f}°C")
            if humidity is not None:
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
        print('<txt>Error</txt>')
        print('<tool>Fatal error in battery status script</tool>')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
