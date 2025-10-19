#!/usr/bin/env python3
"""
Simple Battery Status Display for XFCE4 Panel
One-shot battery status display that can be used with XFCE4 Generic Monitor plugin
"""

import sys
import json
import time
from typing import Optional, Dict, Any

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class SimpleBatteryStatus(Node):
    def __init__(self):
        super().__init__('simple_battery_status')
        
        # Create service client for battery status
        self.battery_service_client = self.create_client(Trigger, '/battery_status')
        
        # Wait for service to be available (short timeout)
        if not self.battery_service_client.wait_for_service(timeout_sec=2.0):
            print("🔋 Service unavailable", flush=True)
            return
        
        # Get battery status
        self.get_battery_status()
    
    def get_battery_status(self):
        """Get and display battery status"""
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
                    display_text = self._format_battery_display(battery_data)
                    print(display_text, flush=True)
                else:
                    print("🔋 Service error", flush=True)
            else:
                print("🔋 Timeout", flush=True)
                
        except Exception as e:
            print("🔋 Error", flush=True)
    
    def _format_battery_display(self, battery_data: Dict[str, Any]) -> str:
        """Format battery data for display"""
        try:
            raw_data = battery_data.get('raw_data', {})
            battery_summary = battery_data.get('battery_summary', 'N/A')
            critical_alerts = battery_data.get('critical_alerts')
            
            # Base battery icon and summary
            if 'LOW BATTERY' in (critical_alerts or ''):
                icon = "🔴"
            elif 'SALTWATER' in (critical_alerts or ''):
                icon = "💧"
            elif 'HUMIDITY' in (critical_alerts or ''):
                icon = "💦"
            else:
                icon = "🔋"
            
            # Build display text
            display_text = f"{icon} {battery_summary}"
            
            # Add charging indicator if available
            charging_status = raw_data.get('charging_status')
            if charging_status is True:
                display_text += " 🔌"
            elif charging_status is False:
                display_text += " 🔋"
            
            # Add AC power indicator
            ac_power = raw_data.get('ac_power_present')
            if ac_power is True:
                display_text += " ⚡"
            
            # Add battery lifetime information if available
            charging_status = raw_data.get('charging_status')
            time_to_full = raw_data.get('time_to_full_hours')
            time_to_empty = raw_data.get('time_to_empty_hours')
            
            if charging_status is True and time_to_full is not None:
                if time_to_full > 0:
                    if time_to_full < 1:
                        display_text += f" +{time_to_full*60:.0f}m"  # Show minutes if less than 1 hour
                    else:
                        display_text += f" +{time_to_full:.1f}h"  # Show hours with 1 decimal
                else:
                    display_text += " ✓"  # Fully charged
            elif charging_status is False and time_to_empty is not None:
                if time_to_empty > 0:
                    if time_to_empty < 1:
                        display_text += f" -{time_to_empty*60:.0f}m"  # Show minutes if less than 1 hour
                    else:
                        display_text += f" -{time_to_empty:.1f}h"  # Show hours with 1 decimal
                else:
                    display_text += " ⚠"  # Empty
            
            return display_text
            
        except Exception:
            return "🔋 Error"

def main():
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run simple battery status
        status = SimpleBatteryStatus()
        
    except Exception:
        print("🔋 Error", flush=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

