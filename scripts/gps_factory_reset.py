#!/usr/bin/env python3
"""
GPS Factory Reset Recovery Script

This script performs a complete factory reset of the GPS module to recover from
bad configuration states. It will:
1. Connect to the GPS
2. Perform a hardware factory reset (clears ALL configuration)
3. Wait for GPS to reboot
4. Reconfigure the GPS with all necessary settings

WARNING: This will clear all GPS configuration and restore factory defaults.
The GPS will then be fully reconfigured.

Usage:
    python3 scripts/gps_factory_reset.py [--port PORT] [--baud BAUDRATE]
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

def send_ubx(ser, ubx_class, ubx_id, payload, expect_ack=False, timeout=0.2):
    """Send a UBX message."""
    length = len(payload)
    header = bytes([0xB5, 0x62, ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF])
    ck = ubx_checksum(bytes([ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF]) + payload)
    frame = header + payload + ck
    
    ser.write(frame)
    if not expect_ack:
        return True
    
    # Simple ACK wait
    original_timeout = ser.timeout
    ser.timeout = timeout
    resp = ser.read(32)
    ser.timeout = original_timeout
    
    # Look for ACK
    for i in range(max(0, len(resp) - 10)):
        if i + 10 <= len(resp) and resp[i:i+2] == b"\xB5\x62" and resp[i+2] == 0x05:
            if resp[i+3] in (0x01, 0x00):  # ACK-ACK or ACK-NAK
                if i + 10 <= len(resp):
                    ack_class = resp[i+4]
                    ack_id = resp[i+5]
                    if ack_class == ubx_class and ack_id == ubx_id:
                        return resp[i+3] == 0x01
    return False

def factory_reset(ser):
    """Perform hardware factory reset."""
    print("Sending hardware factory reset command...")
    # UBX-CFG-RST: hardware reset immediately with cold start
    payload = bytes([
        0x00, 0x04,  # navBbrMask: cold start (clear all)
        0x09,        # resetMode: hardware reset immediately
        0x00         # reserved
    ])
    ok = send_ubx(ser, 0x06, 0x04, payload, expect_ack=False, timeout=0.1)
    print(f"  Reset command sent: {ok}")
    return ok

def configure_port(ser):
    """Configure UART1 port for NMEA output."""
    print("Configuring UART1 port...")
    # CFG-PRT: 38400 baud 8N1, NMEA output enabled
    mode_bytes = bytes([0x08, 0x00, 0x00, 0x96])  # 8N1, 38400 baud
    payload = bytes([
        0x01,        # Port ID: 1 (UART1)
        0x00,        # Reserved
        0x00, 0x00,  # TX Ready: disabled
    ]) + mode_bytes + bytes([
        0x00, 0x00,  # Reserved
        0x07, 0x00,  # In Protocol Mask: UBX + NMEA + RTCM3
        0x07, 0x00,  # Out Protocol Mask: UBX + NMEA + RTCM3
        0x00, 0x00,  # Flags
        0x00, 0x00, 0x00, 0x00  # Reserved
    ])
    ok = send_ubx(ser, 0x06, 0x00, payload, expect_ack=True, timeout=1.0)
    print(f"  Port configured: {ok}")
    return ok

def enable_nmea_sentences(ser):
    """Enable GGA, RMC, and VTG NMEA sentences."""
    print("Enabling NMEA sentences...")
    
    sentences = [
        (0xF0, 0x00, "GGA"),
        (0xF0, 0x04, "RMC"),
        (0xF0, 0x05, "VTG")
    ]
    
    for msg_class, msg_id, name in sentences:
        payload = bytes([
            msg_class, msg_id,
            0x00, 0x01, 0x00, 0x01, 0x00, 0x00  # Rates: [DDC, UART1, UART2, USB, SPI, Reserved]
        ])
        ok = send_ubx(ser, 0x06, 0x01, payload, expect_ack=True, timeout=1.0)
        print(f"  {name} enabled: {ok}")
        time.sleep(0.1)
    
    return True

def configure_hot_start(ser):
    """Configure GPS for hot start capability."""
    print("Configuring hot start settings...")
    
    # CFG-BAT (backup battery)
    bat_payload = bytes([0x01, 0x00, 0x00, 0x00])
    ok = send_ubx(ser, 0x06, 0x09, bat_payload, expect_ack=True, timeout=1.0)
    print(f"  CFG-BAT: {ok}")
    time.sleep(0.1)
    
    # CFG-NAV5 (navigation engine settings)
    nav5_payload = bytes([
        0xFF, 0xFF,  # Mask: apply all
        0x06,        # Dynamic model: automotive
        0x03,        # Fix mode: auto 2D/3D
        0x00, 0x00,  # Fixed altitude
        0x10, 0x27,  # Fixed altitude variance
        0x05,        # Min elevation: 5 degrees
        0x00,        # Reserved
        0xFA, 0x00,  # Position DOP mask
        0xFA, 0x00,  # Time DOP mask
        0x64, 0x00,  # Position accuracy mask
        0x2C, 0x01,  # Time accuracy mask
        0x00,        # Static hold threshold
        0x00,        # DGNSS timeout
        0x00,        # CNO threshold
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # Reserved
    ])
    ok = send_ubx(ser, 0x06, 0x24, nav5_payload, expect_ack=True, timeout=1.0)
    print(f"  CFG-NAV5: {ok}")
    time.sleep(0.1)
    
    # CFG-PMS (power management)
    pms_payload = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    ok = send_ubx(ser, 0x06, 0x86, pms_payload, expect_ack=True, timeout=1.0)
    print(f"  CFG-PMS: {ok}")
    time.sleep(0.1)
    
    return True

def main():
    parser = argparse.ArgumentParser(description='GPS Factory Reset Recovery')
    parser.add_argument('--port', default='/dev/ttyS5', help='Serial port (default: /dev/ttyS5)')
    parser.add_argument('--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("GPS Factory Reset Recovery Script")
    print("=" * 60)
    print(f"\nPort: {args.port}, Baud: {args.baud}")
    print("\nWARNING: This will perform a hardware factory reset!")
    print("All GPS configuration will be cleared and restored to defaults.")
    print("\nPress Ctrl+C to cancel, or wait 5 seconds to continue...")
    
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print("\n\nCancelled by user.")
        sys.exit(0)
    
    try:
        # Open serial port
        print(f"\nOpening serial port {args.port}...")
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        time.sleep(1.0)
        print("  Serial port opened")
        
        # Step 1: Factory reset
        print("\n" + "=" * 60)
        print("STEP 1: Performing Factory Reset")
        print("=" * 60)
        factory_reset(ser)
        
        # Step 2: Wait for GPS to reboot
        print("\nWaiting for GPS to reboot after factory reset...")
        ser.close()
        time.sleep(5.0)  # Give GPS time to reboot
        
        # Step 3: Reopen serial port (may need different baud rate initially)
        print("\nReopening serial port...")
        # Try factory default baud rate (usually 9600)
        for baud_rate in [38400, 9600, 115200]:
            try:
                ser = serial.Serial(args.port, baud_rate, timeout=2.0)
                print(f"  Opened at {baud_rate} baud")
                time.sleep(2.0)
                
                # Try to read something to verify communication
                if ser.in_waiting > 0:
                    ser.read(ser.in_waiting)  # Flush buffer
                
                # If we opened at non-38400, reconfigure to 38400
                if baud_rate != 38400:
                    print(f"  Reconfiguring to 38400 baud...")
                    # Configure port with 38400 baud
                    configure_port(ser)
                    ser.close()
                    time.sleep(1.0)
                    ser = serial.Serial(args.port, 38400, timeout=2.0)
                    print("  Reopened at 38400 baud")
                
                break
            except (serial.SerialException, OSError):
                if 'ser' in locals():
                    try:
                        ser.close()
                    except:
                        pass
                continue
        else:
            print("ERROR: Could not open serial port after reset!")
            sys.exit(1)
        
        # Step 4: Configure GPS
        print("\n" + "=" * 60)
        print("STEP 2: Reconfiguring GPS")
        print("=" * 60)
        
        time.sleep(2.0)  # Give GPS time to stabilize
        
        # Configure port (ensure NMEA is enabled)
        if not configure_port(ser):
            print("WARNING: Port configuration may have failed")
        
        time.sleep(0.5)
        
        # Enable NMEA sentences
        enable_nmea_sentences(ser)
        
        time.sleep(0.5)
        
        # Flush any pending ACKs
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        # Configure hot start
        configure_hot_start(ser)
        
        time.sleep(0.5)
        
        # Final flush
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        # Step 5: Verify NMEA output
        print("\n" + "=" * 60)
        print("STEP 3: Verifying NMEA Output")
        print("=" * 60)
        print("Listening for NMEA sentences (10 seconds)...")
        
        start_time = time.time()
        nmea_count = 0
        while time.time() - start_time < 10.0:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline()
                    data_str = line.decode('ascii', errors='ignore').strip()
                    if data_str.startswith('$'):
                        nmea_count += 1
                        if nmea_count <= 5:
                            print(f"  [{nmea_count}] {data_str[:80]}")
                except:
                    pass
            time.sleep(0.1)
        
        print(f"\nNMEA sentences received: {nmea_count}")
        if nmea_count > 0:
            print("✓ GPS factory reset and reconfiguration successful!")
            print("  NMEA output is working. GPS should now acquire satellites normally.")
        else:
            print("⚠ WARNING: No NMEA sentences received!")
            print("  GPS may need more time, or there may be a hardware issue.")
        
        ser.close()
        print("\n" + "=" * 60)
        print("Recovery complete!")
        print("=" * 60)
        
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

