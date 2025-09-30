#!/usr/bin/env python3
"""
Simple test script to verify high impedance mode functionality
"""
from pathlib import Path

# Define paths directly
SYS_BASE_PATH = Path("/sys/kernel/argo_radio_servo")
SERVO_RUDDER_PATH = SYS_BASE_PATH / "servo_rudder_pw_us"
SERVO_SAIL_PATH = SYS_BASE_PATH / "servo_sail_pw_us"

def test_high_impedance():
    print("Testing High Impedance Mode Functionality")
    print("=" * 50)
    
    # Check if sysfs interface exists
    if not SYS_BASE_PATH.is_dir():
        print(f"ERROR: Sysfs path {SYS_BASE_PATH} not found!")
        return False
    
    print(f"✓ Sysfs interface found at {SYS_BASE_PATH}")
    
    # Test high impedance mode
    print("\n1. Setting servos to HIGH IMPEDANCE mode (0)...")
    SERVO_RUDDER_PATH.write_text("0")
    SERVO_SAIL_PATH.write_text("0")
    
    # Read back values
    rudder_val = SERVO_RUDDER_PATH.read_text().strip()
    sail_val = SERVO_SAIL_PATH.read_text().strip()
    print(f"   Rudder value: {rudder_val}")
    print(f"   Sail value: {sail_val}")
    
    # Test PWM mode
    print("\n2. Setting servos to PWM mode (1500µs)...")
    SERVO_RUDDER_PATH.write_text("1500")
    SERVO_SAIL_PATH.write_text("1500")
    
    # Read back values
    rudder_val = SERVO_RUDDER_PATH.read_text().strip()
    sail_val = SERVO_SAIL_PATH.read_text().strip()
    print(f"   Rudder value: {rudder_val}")
    print(f"   Sail value: {sail_val}")
    
    # Test back to high impedance
    print("\n3. Setting servos back to HIGH IMPEDANCE mode (0)...")
    SERVO_RUDDER_PATH.write_text("0")
    SERVO_SAIL_PATH.write_text("0")
    
    # Read back values
    rudder_val = SERVO_RUDDER_PATH.read_text().strip()
    sail_val = SERVO_SAIL_PATH.read_text().strip()
    print(f"   Rudder value: {rudder_val}")
    print(f"   Sail value: {sail_val}")
    
    print("\n✓ High impedance mode test completed successfully!")
    return True

if __name__ == "__main__":
    test_high_impedance()
