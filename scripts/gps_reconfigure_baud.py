#!/usr/bin/env python3
"""
GPS Baud Rate Reconfiguration Script

Reconfigures GPS module to 38400 baud using UBX commands.
Use this when GPS is responding at a different baud rate (e.g., 9600).

Usage:
    python3 scripts/gps_reconfigure_baud.py [--port PORT] [--current-baud CURRENT] [--target-baud TARGET]
    
Options:
    --port PORT           Serial port (default: /dev/ttyS5)
    --current-baud RATE   Current baud rate GPS is using (default: 9600)
    --target-baud RATE    Target baud rate to configure (default: 38400)
"""

import serial
import time
import sys
import argparse

# Try to import gpiod for GPIO control
try:
    import gpiod
    _HAS_GPIOD = True
except ImportError:
    gpiod = None
    _HAS_GPIOD = False

def ubx_checksum(payload):
    """Calculate UBX checksum for payload."""
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx(ser, ubx_class, ubx_id, payload, expect_ack=False, timeout=1.0):
    """Send a UBX message and optionally wait for ACK."""
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
            # Look for ACK (0xB5 0x62 0x05 0x01 ...)
            for i in range(max(0, len(resp) - 10)):
                if i + 10 <= len(resp) and resp[i:i+2] == b"\xB5\x62" and resp[i+2] == 0x05:
                    if resp[i+3] == 0x01:  # ACK-ACK
                        ack_class = resp[i+4]
                        ack_id = resp[i+5]
                        if ack_class == ubx_class and ack_id == ubx_id:
                            ser.timeout = original_timeout
                            return True
                    elif resp[i+3] == 0x00:  # ACK-NAK
                        ack_class = resp[i+4]
                        ack_id = resp[i+5]
                        if ack_class == ubx_class and ack_id == ubx_id:
                            ser.timeout = original_timeout
                            return False
        time.sleep(0.1)
    
    ser.timeout = original_timeout
    return False

def configure_port_baud(ser, baud_rate):
    """Configure UART1 port baud rate using UBX CFG-PRT command."""
    print(f"Configuring UART1 to {baud_rate} baud...")
    
    # CFG-PRT: Configure UART1 port
    # Mode format: [charLen(4 bits)|reserved(4 bits), parity(2 bits)|nStopBits(2 bits)|reserved(4 bits), 
    #               baudRate(16 bits, little-endian)]
    # For 8N1: charLen=8 (0x08), parity=none (0x0), nStopBits=1 (0x0)
    
    # Convert baud rate to little-endian 16-bit
    baud_low = baud_rate & 0xFF
    baud_high = (baud_rate >> 8) & 0xFF
    
    mode_bytes = bytes([0x08, 0x00, baud_low, baud_high])  # 8N1, baud rate
    
    payload = bytes([
        0x01,        # Port ID: 1 (UART1)
        0x00,        # Reserved
        0x00, 0x00,  # TX Ready: disabled
    ]) + mode_bytes + bytes([
        0x00, 0x00,  # Reserved
        0x07, 0x00,  # In Protocol Mask: UBX(1) + NMEA(2) + RTCM3(4) = 0x07
        0x07, 0x00,  # Out Protocol Mask: UBX(1) + NMEA(2) + RTCM3(4) = 0x07
        0x00, 0x00,  # Flags
        0x00, 0x00, 0x00, 0x00  # Reserved
    ])
    
    ok = send_ubx(ser, 0x06, 0x00, payload, expect_ack=True, timeout=2.0)
    if ok:
        print(f"  ✓ Port configured to {baud_rate} baud (ACK received)")
    else:
        print(f"  ⚠ Port configuration sent (no ACK received - may still work)")
    
    # Save configuration to flash memory so it persists
    print("Saving configuration to flash memory...")
    time.sleep(0.3)  # Brief delay before save command
    
    # UBX-CFG-CFG: Save port configuration to flash
    # Device mask: All devices (0xFFFFFFFF) to ensure we save everything
    # Save mask: IOPORT (0x00000001) = port configuration including baud rate
    # This saves the port configuration to flash memory so it persists after power cycles
    device_mask = bytes([0xFF, 0xFF, 0xFF, 0xFF])  # All devices
    clear_mask = bytes([0x00, 0x00, 0x00, 0x00])    # Don't clear anything
    save_mask = bytes([0x01, 0x00, 0x00, 0x00])     # Save IOPORT (port config) to flash
    load_mask = bytes([0x00, 0x00, 0x00, 0x00])      # Don't load from flash
    
    save_payload = device_mask + clear_mask + save_mask + load_mask
    save_ok = send_ubx(ser, 0x06, 0x09, save_payload, expect_ack=True, timeout=3.0)
    
    if save_ok:
        print("  ✓ Configuration saved to flash (ACK received)")
        print("  ✓ GPS will retain 38400 baud after power cycles")
    else:
        print("  ⚠ Save command sent (no ACK received)")
        print("  ⚠ Configuration may not persist - try hardware reset after reconfiguration")
    
    return ok

def reset_gps_hardware(rst_gpio_num):
    """Perform hardware reset via GPIO RST pin."""
    if not _HAS_GPIOD:
        print("ERROR: gpiod library not available. Cannot control RST pin.")
        return False
    
    try:
        print(f"\nPerforming hardware reset via GPIO {rst_gpio_num}...")
        chip = gpiod.Chip('gpiochip0')
        line = chip.get_line(rst_gpio_num)
        line.request(consumer='gps_reconfigure', type=gpiod.LINE_REQ_DIR_OUT)
        
        print("  Pulling RST LOW (reset)...")
        line.set_value(0)  # Active LOW reset
        time.sleep(0.1)    # Hold reset for 100ms
        
        print("  Releasing RST HIGH...")
        line.set_value(1)  # Release reset
        time.sleep(2.0)    # Wait for GPS to initialize
        
        line.release()
        chip.close()
        
        print("  ✓ Hardware reset complete")
        return True
        
    except Exception as e:
        print(f"  ✗ ERROR: Hardware reset failed: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Reconfigure GPS baud rate using UBX commands',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Reconfigure from 9600 to 38400 baud (most common case)
  python3 scripts/gps_reconfigure_baud.py
  
  # Reconfigure from specific baud rate
  python3 scripts/gps_reconfigure_baud.py --current-baud 115200 --target-baud 38400
  
  # Reconfigure to different baud rate
  python3 scripts/gps_reconfigure_baud.py --target-baud 57600
        """
    )
    parser.add_argument('--port', default='/dev/ttyS5',
                       help='Serial port (default: /dev/ttyS5)')
    parser.add_argument('--current-baud', type=int, default=9600,
                       help='Current baud rate GPS is using (default: 9600)')
    parser.add_argument('--target-baud', type=int, default=38400,
                       help='Target baud rate to configure (default: 38400)')
    parser.add_argument('--rst-gpio', type=int, default=None,
                       help='GPIO number for RST pin (e.g., 264). If specified, will reset GPS after reconfiguration')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("GPS Baud Rate Reconfiguration")
    print("=" * 60)
    print(f"\nPort: {args.port}")
    print(f"Current baud: {args.current_baud}")
    print(f"Target baud: {args.target_baud}")
    if args.rst_gpio:
        print(f"RST GPIO: {args.rst_gpio} (will reset GPS after reconfiguration)")
    
    if args.current_baud == args.target_baud:
        print("\nCurrent and target baud rates are the same. Nothing to do.")
        sys.exit(0)
    
    print("\nThis will reconfigure the GPS baud rate using UBX commands.")
    print("Press Ctrl+C to cancel, or wait 3 seconds to continue...")
    
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        print("\n\nCancelled by user.")
        sys.exit(0)
    
    try:
        # Open serial port at current baud rate
        print(f"\nOpening serial port at {args.current_baud} baud...")
        ser = serial.Serial(
            args.port,
            baudrate=args.current_baud,
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
        
        # Configure port to target baud rate
        print(f"\nSending UBX CFG-PRT command to set baud rate to {args.target_baud}...")
        success = configure_port_baud(ser, args.target_baud)
        
        # Close port at old baud rate
        ser.close()
        time.sleep(0.5)
        
        # Optionally perform hardware reset to apply configuration
        if args.rst_gpio:
            reset_gps_hardware(args.rst_gpio)
            time.sleep(1.0)  # Additional wait after reset
        
        # Reopen at new baud rate
        print(f"\nReopening serial port at {args.target_baud} baud...")
        try:
            ser = serial.Serial(
                args.port,
                baudrate=args.target_baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=2.0
            )
            time.sleep(1.0)
            print("  ✓ Serial port reopened at new baud rate")
            
            # Verify communication by sending a UBX MON-VER query
            print("\nVerifying communication at new baud rate...")
            time.sleep(0.5)
            
            # Send UBX MON-VER (version query)
            ver_response = send_ubx(ser, 0x0A, 0x04, b'', expect_ack=False, timeout=1.0)
            time.sleep(0.5)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                if len(response) >= 8 and response[0:2] == b'\xB5\x62':
                    print("  ✓ UBX response received - communication verified!")
                    print(f"  ✓ GPS is now configured for {args.target_baud} baud")
                    print("\n" + "=" * 60)
                    print("SUCCESS: GPS baud rate reconfigured")
                    print("=" * 60)
                    print(f"\nThe GPS is now configured for {args.target_baud} baud.")
                    print("You can now use gps.py with the standard 38400 baud setting.")
                else:
                    print("  ⚠ Response received but doesn't look like UBX")
                    print("  GPS may still be configured correctly - try gps.py")
            else:
                print("  ⚠ No immediate response (GPS may need time to initialize)")
                print("  GPS should still be configured correctly - try gps.py")
            
            ser.close()
            
        except serial.SerialException as e:
            print(f"  ✗ ERROR: Failed to open at {args.target_baud} baud: {e}")
            print("\nPossible causes:")
            print("  1. GPS didn't accept the baud rate change")
            print("  2. GPS needs more time to apply the change")
            print("  3. Try running the test script again to verify current baud rate")
            sys.exit(1)
        
    except serial.SerialException as e:
        print(f"\nERROR: Serial port error: {e}")
        print("\nTroubleshooting:")
        print(f"  1. Verify port exists: ls -l {args.port}")
        print(f"  2. Check permissions: sudo chmod 666 {args.port}")
        print(f"  3. Verify current baud rate with: python3 scripts/test_gps_baud_rates.py")
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
