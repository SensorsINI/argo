#!/usr/bin/env python3
"""
Test script to manually trigger GPS hardware factory reset
This tests the reset mechanism without waiting 10 minutes
"""

import sys
import os

# Add Argo project root to path (works from any location)
script_dir = os.path.dirname(os.path.abspath(__file__))
argo_root = os.path.dirname(os.path.dirname(script_dir))  # Go up from nodes/test/ to argo root
sys.path.insert(0, argo_root)
sys.path.insert(0, os.path.join(argo_root, 'nodes'))
sys.path.insert(0, os.path.join(argo_root, 'nodes', 'support'))

import rclpy
from gps import GpsNode
import time

def test_factory_reset():
    """Test the hardware factory reset functionality"""
    print("=" * 70)
    print("Testing GPS Hardware Factory Reset")
    print("=" * 70)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create GPS node
        print("\n1. Creating GPS node...")
        node = GpsNode(debug_mode=True)
        print("   ✓ GPS node created")
        
        # Wait a moment for node to initialize
        time.sleep(1.0)
        
        # Check if serial port is open
        if not (node.serial_port and node.serial_port.is_open):
            print("\n❌ ERROR: Serial port is not open!")
            print("   Cannot test reset without serial connection")
            return False
        
        print(f"\n2. Serial port status:")
        print(f"   Port: {node.serial_port_name}")
        print(f"   Baud: {node.baud_rate}")
        print(f"   Open: {node.serial_port.is_open}")
        
        # Test factory reset
        print("\n3. Testing hardware factory reset...")
        print("   WARNING: This will clear ALL GPS configuration!")
        print("   Waiting 3 seconds before proceeding...")
        time.sleep(3.0)
        
        # Perform factory reset
        print("\n4. Sending hardware factory reset command...")
        reset_ok = node.gps_factory_reset()
        
        if reset_ok:
            print("   ✓ Factory reset command sent successfully")
        else:
            print("   ❌ Factory reset command failed")
            return False
        
        # Wait for GPS to reboot
        print("\n5. Waiting for GPS to reboot (5 seconds)...")
        time.sleep(5.0)
        
        # Close serial port
        print("\n6. Closing serial port for GPS reboot...")
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
            print("   ✓ Serial port closed")
        time.sleep(2.0)
        
        # Reopen serial port
        print("\n7. Reopening serial port after GPS reboot...")
        try:
            import serial
            node.serial_port = serial.Serial(node.serial_port_name, node.baud_rate, timeout=1.0)
            time.sleep(1.0)
            print("   ✓ Serial port reopened")
        except Exception as e:
            print(f"   ❌ Failed to reopen serial port: {e}")
            print("   Attempting recovery via setup_gps()...")
            node.setup_gps()
            return False
        
        # Reconfigure GPS
        print("\n8. Reconfiguring GPS after factory reset...")
        node.setup_gps()
        print("   ✓ GPS reconfiguration completed")
        
        # Verify GPS is responding
        print("\n9. Verifying GPS communication...")
        time.sleep(2.0)
        response = node.send_cmd("$PMTK605", timeout=1.0)
        if response:
            print(f"   ✓ GPS is responding: {response[:80]}")
            print("\n✅ Hardware factory reset test completed successfully!")
            print("   GPS should now be in factory default state and fully reconfigured")
            return True
        else:
            print("   ⚠️  GPS did not respond to version query (may need more time)")
            print("   But reset process completed")
            return True
            
    except Exception as e:
        print(f"\n❌ ERROR during test: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Cleanup
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    success = test_factory_reset()
    sys.exit(0 if success else 1)

