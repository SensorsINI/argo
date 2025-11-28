#!/usr/bin/env python3
"""
GPS Firmware Recovery Script

Attempts to recover GPS from bootloader mode by sending firmware update commands.
This script uses UBX protocol to communicate with the GPS in bootloader mode.

WARNING: This is experimental. Firmware flashing typically requires u-center or
ubxfwupdate tools. This script attempts basic recovery procedures.

Usage:
    python3 scripts/gps_firmware_recovery.py [--port PORT] [--firmware-file FILE]
    
Options:
    --port PORT           Serial port (default: /dev/ttyS5)
    --firmware-file FILE  Path to firmware file (if available)
    --baud RATE          Baud rate (default: 9600)
"""

import serial
import time
import sys
import argparse

def ubx_checksum(payload):
    """Calculate UBX checksum for payload."""
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx(ser, ubx_class, ubx_id, payload, expect_ack=False, timeout=1.0):
    """Send a UBX message."""
    length = len(payload)
    header = bytes([0xB5, 0x62, ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF])
    ck = ubx_checksum(bytes([ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF]) + payload)
    frame = header + payload + ck
    
    ser.write(frame)
    if not expect_ack:
        return True
    
    # Wait for ACK
    original_timeout = ser.timeout
    ser.timeout = timeout
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if ser.in_waiting >= 10:
            resp = ser.read(ser.in_waiting)
            # Look for ACK
            for i in range(max(0, len(resp) - 10)):
                if i + 10 <= len(resp) and resp[i:i+2] == b"\xB5\x62" and resp[i+2] == 0x05:
                    if resp[i+3] == 0x01:  # ACK-ACK
                        ack_class = resp[i+4]
                        ack_id = resp[i+5]
                        if ack_class == ubx_class and ack_id == ubx_id:
                            ser.timeout = original_timeout
                            return True
        time.sleep(0.1)
    
    ser.timeout = original_timeout
    return False

def check_bootloader_mode(ser):
    """Check if GPS is in bootloader mode by querying version."""
    print("Checking GPS version to detect bootloader mode...")
    
    # Send MON-VER query
    ver_resp = send_ubx(ser, 0x0A, 0x04, b'', expect_ack=False, timeout=1.0)
    time.sleep(0.5)
    
    if ser.in_waiting > 0:
        resp = ser.read(ser.in_waiting)
        if len(resp) >= 8 and resp[0:2] == b'\xB5\x62' and resp[2] == 0x0A and resp[3] == 0x04:
            # Parse software version
            length = resp[4] + (resp[5] << 8)
            if length >= 30 and len(resp) >= 6 + 30:
                sw_version_end = resp.find(b'\x00', 6, 6 + 30)
                if sw_version_end > 6:
                    sw_version = resp[6:sw_version_end].decode('ascii', errors='ignore')
                    print(f"  Software Version: {sw_version}")
                    
                    if "ROM BOOT" in sw_version.upper() or "BOOT" in sw_version.upper():
                        print("  ✗ GPS is in BOOTLOADER mode")
                        return True
                    else:
                        print("  ✓ GPS appears to be in normal firmware mode")
                        return False
    
    print("  ⚠ Could not determine GPS mode")
    return None

def attempt_firmware_recovery(ser):
    """Attempt to recover GPS from bootloader mode."""
    print("\n" + "=" * 60)
    print("Firmware Recovery Attempt")
    print("=" * 60)
    
    print("\nNOTE: Full firmware recovery typically requires:")
    print("  1. u-blox u-center software (Windows)")
    print("  2. ubxfwupdate command-line tool (Windows)")
    print("  3. Correct firmware file for NEO-M9N")
    print("\nThis script will attempt basic recovery procedures...")
    
    # Method 1: Try to exit bootloader mode with a reset
    print("\n[1/3] Attempting controlled reset to exit bootloader...")
    # UBX-CFG-RST: Hardware reset
    reset_payload = bytes([
        0x00, 0x04,  # navBbrMask: cold start
        0x09,        # resetMode: hardware reset immediately
        0x00         # reserved
    ])
    reset_ok = send_ubx(ser, 0x06, 0x04, reset_payload, expect_ack=False, timeout=0.1)
    if reset_ok:
        print("  ✓ Reset command sent")
        print("  Waiting 5 seconds for GPS to reboot...")
        ser.close()
        time.sleep(5.0)
        
        # Reopen serial port
        try:
            ser = serial.Serial(ser.port, ser.baudrate, timeout=2.0)
            time.sleep(2.0)
            print("  Serial port reopened")
            
            # Check if still in bootloader
            if check_bootloader_mode(ser):
                print("  ✗ GPS still in bootloader mode after reset")
            else:
                print("  ✓ GPS appears to have exited bootloader mode!")
                return True
        except Exception as e:
            print(f"  ✗ Error reopening serial port: {e}")
            return False
    else:
        print("  ✗ Reset command failed")
    
    # Method 2: Try CFG-CFG to clear all configuration
    print("\n[2/3] Attempting to clear all configuration...")
    device_mask = bytes([0xFF, 0xFF, 0xFF, 0xFF])  # All devices
    clear_mask = bytes([0xFF, 0xFF, 0xFF, 0xFF])    # Clear all
    save_mask = bytes([0x00, 0x00, 0x00, 0x00])     # Don't save
    load_mask = bytes([0x00, 0x00, 0x00, 0x00])     # Don't load
    
    cfg_payload = device_mask + clear_mask + save_mask + load_mask
    cfg_ok = send_ubx(ser, 0x06, 0x09, cfg_payload, expect_ack=True, timeout=2.0)
    if cfg_ok:
        print("  ✓ Configuration clear command accepted")
        time.sleep(2.0)
        if check_bootloader_mode(ser):
            print("  ✗ GPS still in bootloader mode")
        else:
            print("  ✓ GPS appears to have exited bootloader mode!")
            return True
    else:
        print("  ✗ Configuration clear command rejected")
    
    # Method 3: Check if we can query bootloader status
    print("\n[3/3] Querying bootloader status...")
    # Some bootloaders support status queries
    # This is device-specific and may not work
    
    print("\n" + "=" * 60)
    print("Recovery Attempt Complete")
    print("=" * 60)
    print("\nIf GPS is still in bootloader mode, you will need to:")
    print("  1. Use u-blox u-center (Windows) or ubxfwupdate tool")
    print("  2. Download correct firmware file for NEO-M9N")
    print("  3. Flash firmware using official tools")
    print("\nAlternative: Check if SAFEBOOT pin is stuck LOW")
    print("  - SAFEBOOT should be HIGH for normal operation")
    print("  - If stuck LOW, GPS will always boot into bootloader mode")
    
    return False

def main():
    parser = argparse.ArgumentParser(
        description='Attempt GPS firmware recovery from bootloader mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Check if GPS is in bootloader mode
  python3 scripts/gps_firmware_recovery.py
  
  # Attempt recovery
  python3 scripts/gps_firmware_recovery.py --port /dev/ttyS5 --baud 9600
        """
    )
    parser.add_argument('--port', default='/dev/ttyS5',
                       help='Serial port (default: /dev/ttyS5)')
    parser.add_argument('--baud', type=int, default=9600,
                       help='Baud rate (default: 9600)')
    parser.add_argument('--firmware-file', default=None,
                       help='Path to firmware file (not yet implemented)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("GPS Firmware Recovery Script")
    print("=" * 60)
    print(f"\nPort: {args.port}")
    print(f"Baud: {args.baud}")
    
    if args.firmware_file:
        print(f"Firmware file: {args.firmware_file}")
        print("\n⚠ WARNING: Firmware file flashing not yet implemented")
        print("  This script only attempts basic recovery procedures")
    
    print("\nThis script will attempt to recover GPS from bootloader mode.")
    print("Press Ctrl+C to cancel, or wait 3 seconds to continue...")
    
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        print("\n\nCancelled by user.")
        sys.exit(0)
    
    try:
        # Open serial port
        print(f"\nOpening serial port at {args.baud} baud...")
        ser = serial.Serial(
            args.port,
            baudrate=args.baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2.0
        )
        time.sleep(0.5)
        print("  ✓ Serial port opened")
        
        # Flush any pending data
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        # Check if in bootloader mode
        is_bootloader = check_bootloader_mode(ser)
        
        if is_bootloader:
            print("\n" + "=" * 60)
            print("GPS is in BOOTLOADER mode - attempting recovery...")
            print("=" * 60)
            success = attempt_firmware_recovery(ser)
            
            if success:
                print("\n✓ Recovery successful! GPS should now work normally.")
            else:
                print("\n✗ Recovery failed. GPS is still in bootloader mode.")
                print("\nNext steps:")
                print("  1. Check SAFEBOOT pin - should be HIGH (not stuck LOW)")
                print("  2. Use u-blox u-center or ubxfwupdate to flash firmware")
                print("  3. Contact u-blox support if hardware fault suspected")
        elif is_bootloader is False:
            print("\n✓ GPS is NOT in bootloader mode - firmware appears normal")
            print("  If GPS is not working, the issue may be hardware or configuration")
        else:
            print("\n⚠ Could not determine GPS mode")
            print("  Attempting recovery procedures anyway...")
            attempt_firmware_recovery(ser)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"\nERROR: Serial port error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        if 'ser' in locals():
            try:
                ser.close()
            except:
                pass
        sys.exit(1)

if __name__ == '__main__':
    main()
