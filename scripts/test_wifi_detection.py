#!/usr/bin/env python3

import sys
import os
sys.path.append('/home/orangepi/argo/power_control')

from argo_power_control import PowerController

def test_wifi_detection():
    """Test WiFi detection with current network"""
    print("Testing WiFi detection...")
    
    # Create power controller instance
    controller = PowerController(test_mode=True)
    
    # Test WiFi connectivity
    wifi_status = controller.check_wifi_connectivity()
    
    print(f"WiFi connectivity status: {wifi_status}")
    
    # Get current connection info
    import subprocess
    try:
        result = subprocess.run(['nmcli', '-t', '-f', 'NAME', 'connection', 'show', '--active'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            active_connection = result.stdout.strip()
            print(f"Active connection: {active_connection}")
        else:
            print("Could not determine active connection")
    except Exception as e:
        print(f"Error getting connection info: {e}")
    
    # Test DNS resolution
    try:
        dns_result = subprocess.run(['nslookup', 'google.com'], 
                                  capture_output=True, text=True, timeout=5)
        if dns_result.returncode == 0:
            print("DNS resolution: OK")
        else:
            print("DNS resolution: FAILED")
    except Exception as e:
        print(f"DNS test error: {e}")
    
    # Test internet connectivity
    try:
        ping_result = subprocess.run(['ping', '-c', '1', '-W', '3', '8.8.8.8'], 
                                   capture_output=True, text=True, timeout=5)
        if ping_result.returncode == 0:
            print("Internet connectivity: OK")
        else:
            print("Internet connectivity: FAILED")
    except Exception as e:
        print(f"Internet test error: {e}")

if __name__ == "__main__":
    test_wifi_detection()
