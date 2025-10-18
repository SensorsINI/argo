#!/usr/bin/env python3
"""
Battery Status Panel for XFCE4
Displays battery status from battery_water.py ROS2 node in the XFCE4 panel
"""

import sys
import time
import json
import signal
import argparse
from datetime import datetime
from typing import Optional, Dict, Any

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BatteryStatusPanel(Node):
    def __init__(self, update_interval: float = 5.0):
        super().__init__('battery_status_panel')
        
        self.update_interval = update_interval
        self.last_update_time = 0.0
        self.last_error_log_time = 0.0
        self.error_log_interval = 30.0  # Log errors max once every 30 seconds
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # Battery status cache
        self.last_battery_data = None
        self.last_display_text = "🔋 N/A"
        
        # Create service client for battery status
        self.battery_service_client = self.create_client(Trigger, '/battery_status')
        
        # Wait for service to be available
        self.get_logger().info("Waiting for battery_water node service...")
        if not self.battery_service_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Battery service not available - will retry periodically")
        
        # Create timer for periodic updates
        self.timer = self.create_timer(self.update_interval, self.update_battery_status)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info(f"Battery status panel initialized (update interval: {self.update_interval}s)")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        if rclpy.ok():
            rclpy.shutdown()
    
    def _throttled_error_log(self, message: str):
        """Log error with throttling to prevent log spam"""
        current_time = time.monotonic()
        if current_time - self.last_error_log_time >= self.error_log_interval:
            self.get_logger().error(message)
            self.last_error_log_time = current_time
    
    def _get_battery_status(self) -> Optional[Dict[str, Any]]:
        """Get battery status from battery_water node service"""
        try:
            if not self.battery_service_client.service_is_ready():
                return None
            
            # Create service request
            request = Trigger.Request()
            
            # Call service with timeout
            future = self.battery_service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    # Parse JSON response
                    battery_data = json.loads(response.message)
                    self.consecutive_errors = 0  # Reset error counter on success
                    return battery_data
                else:
                    self._throttled_error_log(f"Battery service returned error: {response.message}")
                    return None
            else:
                self._throttled_error_log("Battery service call timed out")
                return None
                
        except Exception as e:
            self.consecutive_errors += 1
            if self.consecutive_errors <= self.max_consecutive_errors:
                self._throttled_error_log(f"Error calling battery service: {e}")
            return None
    
    def _format_battery_display(self, battery_data: Dict[str, Any]) -> str:
        """Format battery data for display in panel"""
        try:
            raw_data = battery_data.get('raw_data', {})
            battery_summary = battery_data.get('battery_summary', 'N/A')
            critical_alerts = battery_data.get('critical_alerts')
            
            # Base battery icon and summary
            if 'LOW BATTERY' in (critical_alerts or ''):
                icon = "🔴"  # Red for low battery
            elif 'SALTWATER' in (critical_alerts or ''):
                icon = "💧"  # Water drop for saltwater
            elif 'HUMIDITY' in (critical_alerts or ''):
                icon = "💦"  # Droplets for humidity
            else:
                icon = "🔋"  # Normal battery
            
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
            
            return display_text
            
        except Exception as e:
            self._throttled_error_log(f"Error formatting battery display: {e}")
            return "🔋 Error"
    
    def update_battery_status(self):
        """Update battery status and display"""
        current_time = time.monotonic()
        
        # Get battery status
        battery_data = self._get_battery_status()
        
        if battery_data is not None:
            # Format and display battery status
            display_text = self._format_battery_display(battery_data)
            
            # Only update if display text changed
            if display_text != self.last_display_text:
                self.last_display_text = display_text
                print(display_text)  # Output to stdout for panel consumption
                sys.stdout.flush()
            
            self.last_battery_data = battery_data
            self.last_update_time = current_time
            
        else:
            # Service not available - show fallback
            if self.consecutive_errors == 0:
                # First failure - show service unavailable
                fallback_text = "🔋 Service unavailable"
                if fallback_text != self.last_display_text:
                    self.last_display_text = fallback_text
                    print(fallback_text)
                    sys.stdout.flush()
            elif self.consecutive_errors >= self.max_consecutive_errors:
                # Too many failures - show error state
                error_text = "🔋 Error"
                if error_text != self.last_display_text:
                    self.last_display_text = error_text
                    print(error_text)
                    sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(
        description='Battery Status Panel for XFCE4',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This script displays battery status from the battery_water.py ROS2 node
in the XFCE4 panel. It connects to the /battery_status service and
formats the data for display.

The script outputs formatted battery status to stdout, which can be
consumed by XFCE4 panel plugins or other desktop components.

Features:
- Real-time battery voltage and percentage
- Charging status indicators
- Critical alert indicators (low battery, saltwater, humidity)
- Graceful error handling with throttled logging
- Automatic recovery from service failures

Usage:
  python3 battery_status_panel.py [--update-interval SECONDS]
  
Options:
  --update-interval: Update interval in seconds (default: 5.0)
        """
    )
    parser.add_argument('--update-interval', type=float, default=5.0,
                        help='Update interval in seconds (default: 5.0)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run battery status panel
        panel = BatteryStatusPanel(update_interval=args.update_interval)
        rclpy.spin(panel)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

