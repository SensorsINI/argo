#!/usr/bin/env python3
"""
GPS Cold Start Script

Triggers a cold start reset on the GPS to clear stale almanac/ephemeris data.
This is useful when the GPS sees many satellites but can't acquire a fix,
which often indicates stale satellite data.

Usage:
    python3 scripts/gps_cold_start.py
"""

import serial
import time
import sys

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

def clear_navigation_data(ser):
    """Clear navigation BBR data (ephemeris, almanac) without doing a full reset.
    
    This is safer than cold start as it doesn't stop NMEA output.
    """
    print("Sending clear navigation data command...")
    # UBX-CFG-CFG: Clear navigation BBR only
    device_mask = bytes([0xFF, 0xFF, 0xFF, 0xFF])  # All devices
    clear_mask = bytes([0x01, 0x00, 0x00, 0x00])   # Clear navBBR only
    save_mask = bytes([0x00, 0x00, 0x00, 0x00])    # Don't save to flash
    load_mask = bytes([0x00, 0x00, 0x00, 0x00])    # Don't load from flash
    
    payload = device_mask + clear_mask + save_mask + load_mask
    ok = send_ubx(ser, 0x06, 0x09, payload, expect_ack=True, timeout=2.0)
    print(f"  Clear navigation data command sent: {ok}")
    return ok

def cold_start(ser):
    """Perform cold start reset (clears all satellite data).
    
    WARNING: This may temporarily stop NMEA output. Restart gps.py after this.
    """
    print("Sending cold start reset command...")
    # UBX-CFG-RST: controlled software reset with cold start mask
    # This clears ephemeris, almanac, position, and time
    payload = bytes([
        0x00, 0x04,  # navBbrMask: cold start (clear all data)
        0x08,        # resetMode: controlled reset (GPS continues after reset)
        0x00         # reserved
    ])
    ok = send_ubx(ser, 0x06, 0x04, payload, expect_ack=False, timeout=0.1)
    print(f"  Cold start command sent: {ok}")
    return ok

def main():
    port = '/dev/ttyS5'
    baud = 38400
    
    print("GPS Navigation Data Clear")
    print(f"Port: {port}, Baud: {baud}")
    print("\nThis will clear GPS navigation data (ephemeris, almanac) to fix stale data issues.")
    print("This method is safer than cold start - it preserves NMEA output.")
    print("The GPS will need to reacquire satellites after clearing.")
    print("\nPress Ctrl+C to cancel, or wait 3 seconds to continue...")
    
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        print("\n\nCancelled by user.")
        sys.exit(0)
    
    try:
        print(f"\nOpening serial port {port}...")
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(1.0)
        print("  Serial port opened")
        
        # Flush any pending data
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        # Clear navigation data (safer than cold start)
        print("\nClearing navigation data...")
        if clear_navigation_data(ser):
            print("\nNavigation data cleared successfully!")
            print("GPS is now clearing stale ephemeris and almanac data.")
            print("The GPS will need 30-60 seconds to reacquire satellites.")
            print("\nNOTE: NMEA output is preserved - gps.py should continue working normally.")
        else:
            print("\nWARNING: Clear navigation data command may have failed.")
            print("You may need to restart gps.py to recover.")
        
        ser.close()
        print("\nDone!")
        
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
