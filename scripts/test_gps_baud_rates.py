#!/usr/bin/env python3
"""
GPS Baud Rate Test Script

Tests GPS communication at various baud rates to diagnose firmware/configuration issues.
Optionally controls RST pin via GPIO for hardware reset.

Usage:
    python3 scripts/test_gps_baud_rates.py [--port PORT] [--rst-gpio GPIO_NUM] [--no-reset]
    
Options:
    --port PORT        Serial port (default: /dev/ttyS5)
    --rst-gpio NUM     GPIO number for RST pin control (default: 264 for PI8/pin 3)
                       If specified, will perform hardware reset before testing
                       Recommended: GPIO 264 (PI8, pin 3) - currently free
    --no-reset         Skip hardware reset even if RST GPIO is specified
    --verbose          Show detailed debug output
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

# Common baud rates to test (u-blox defaults and common values)
BAUD_RATES = [38400, 9600, 115200, 57600, 19200, 4800]

def checksum(msg):
    """Calculate NMEA checksum."""
    calc_cksum = 0
    for char in msg:
        calc_cksum ^= ord(char)
    return calc_cksum

def send_nmea_cmd(ser, cmd, timeout=1.0):
    """Send NMEA command and read response."""
    if not '*' in cmd:
        cs = checksum(cmd)
        cmd = f"{cmd}*{cs:02X}"
    
    if not cmd.endswith('\r\n'):
        cmd = cmd + '\r\n'
    
    try:
        original_timeout = ser.timeout
        ser.timeout = timeout
        ser.reset_input_buffer()
        ser.write(cmd.encode('ascii'))
        
        # Try to read response
        reply_bytes = ser.readline()
        reply = reply_bytes.decode('ascii', errors='ignore').strip()
        
        ser.timeout = original_timeout
        return reply
    except Exception as e:
        return None

def send_ubx_cmd(ser, ubx_class, ubx_id, payload, timeout=1.0):
    """Send UBX command and read response."""
    def ubx_checksum(payload_bytes):
        ck_a = 0
        ck_b = 0
        for byte in payload_bytes:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return bytes([ck_a, ck_b])
    
    length = len(payload)
    header = bytes([0xB5, 0x62, ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF])
    ck = ubx_checksum(bytes([ubx_class & 0xFF, ubx_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF]) + payload)
    frame = header + payload + ck
    
    try:
        original_timeout = ser.timeout
        ser.timeout = timeout
        ser.reset_input_buffer()
        ser.write(frame)
        
        # Try to read response (UBX messages start with 0xB5 0x62)
        time.sleep(0.1)  # Give device time to respond
        response = ser.read(64)  # Read up to 64 bytes
        
        ser.timeout = original_timeout
        
        # Check if we got a UBX response
        if len(response) >= 8 and response[0:2] == b'\xB5\x62':
            return response
        return None
    except Exception as e:
        return None

def test_baud_rate(port, baud, verbose=False):
    """Test GPS communication at a specific baud rate."""
    print(f"\n{'='*60}")
    print(f"Testing baud rate: {baud}")
    print(f"{'='*60}")
    
    try:
        # Open serial port
        ser = serial.Serial(
            port,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1.0
        )
        
        # Wait a bit for port to stabilize
        time.sleep(0.5)
        
        # Flush any pending data
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        results = {
            'baud': baud,
            'port_open': True,
            'nmea_version': None,
            'nmea_pubx': None,
            'nmea_pmtk414': None,
            'ubx_version': None,
            'automatic_output': None
        }
        
        # Test 1: Check for automatic NMEA output
        print("  [1/5] Checking for automatic NMEA output...")
        time.sleep(1.0)  # Wait for automatic output
        if ser.in_waiting > 0:
            try:
                auto_data = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
                if auto_data.strip():
                    # Check if it looks like NMEA
                    if '$' in auto_data or 'GGA' in auto_data or 'RMC' in auto_data:
                        results['automatic_output'] = auto_data[:100]  # First 100 chars
                        print(f"      ✓ Automatic output detected: {auto_data[:80]}...")
                    else:
                        print(f"      ⚠ Data received but doesn't look like NMEA: {auto_data[:80]}")
                else:
                    print("      ✗ No automatic output")
            except Exception as e:
                print(f"      ✗ Error reading automatic output: {e}")
        else:
            print("      ✗ No automatic output")
        
        # Flush buffer before command tests
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        # Test 2: NMEA version query ($PMTK605)
        print("  [2/5] Testing NMEA version query ($PMTK605)...")
        response = send_nmea_cmd(ser, "$PMTK605", timeout=1.5)
        if response and len(response) > 0:
            results['nmea_version'] = response
            print(f"      ✓ Response: {response[:80]}")
        else:
            print("      ✗ No response")
        
        # Test 3: u-blox PUBX position query ($PUBX,00)
        print("  [3/5] Testing u-blox PUBX query ($PUBX,00)...")
        response = send_nmea_cmd(ser, "$PUBX,00", timeout=1.5)
        if response and len(response) > 0:
            results['nmea_pubx'] = response
            print(f"      ✓ Response: {response[:80]}")
        else:
            print("      ✗ No response")
        
        # Test 4: NMEA output rate query ($PMTK414)
        print("  [4/5] Testing NMEA rate query ($PMTK414)...")
        response = send_nmea_cmd(ser, "$PMTK414", timeout=1.5)
        if response and len(response) > 0:
            results['nmea_pmtk414'] = response
            print(f"      ✓ Response: {response[:80]}")
        else:
            print("      ✗ No response")
        
        # Test 5: UBX version query (MON-VER)
        print("  [5/5] Testing UBX version query (MON-VER)...")
        # UBX-MON-VER: no payload
        response = send_ubx_cmd(ser, 0x0A, 0x04, b'', timeout=1.5)
        if response and len(response) >= 8:
            results['ubx_version'] = True
            # Parse UBX response (simplified - just check for valid UBX frame)
            print(f"      ✓ UBX response received ({len(response)} bytes)")
            if verbose:
                print(f"         Raw: {response.hex()[:80]}...")
        else:
            print("      ✗ No UBX response")
        
        ser.close()
        
        # Summary
        success_count = sum([
            results['automatic_output'] is not None,
            results['nmea_version'] is not None,
            results['nmea_pubx'] is not None,
            results['nmea_pmtk414'] is not None,
            results['ubx_version'] is not None
        ])
        
        if success_count > 0:
            print(f"\n  ✓ SUCCESS: {success_count}/5 tests passed at {baud} baud")
            return results
        else:
            print(f"\n  ✗ FAILED: 0/5 tests passed at {baud} baud")
            return results
        
    except serial.SerialException as e:
        print(f"  ✗ ERROR: Failed to open serial port: {e}")
        return {'baud': baud, 'port_open': False, 'error': str(e)}
    except Exception as e:
        print(f"  ✗ ERROR: Unexpected error: {e}")
        return {'baud': baud, 'port_open': True, 'error': str(e)}

def reset_gps_hardware(rst_gpio_num, verbose=False):
    """Perform hardware reset via GPIO RST pin."""
    if not _HAS_GPIOD:
        print("ERROR: gpiod library not available. Cannot control RST pin.")
        print("       Install with: sudo apt-get install python3-gpiod")
        return False
    
    try:
        print(f"\n{'='*60}")
        print(f"Performing hardware reset via GPIO {rst_gpio_num}")
        print(f"{'='*60}")
        
        # Open GPIO chip (usually chip0 on Orange Pi)
        chip = gpiod.Chip('gpiochip0')
        
        # Get line for RST pin
        line = chip.get_line(rst_gpio_num)
        
        # Configure as output
        line.request(consumer='gps_test', type=gpiod.LINE_REQ_DIR_OUT)
        
        # RST is typically active LOW, so:
        # - Pull LOW to reset
        # - Pull HIGH to release reset
        
        print("  Pulling RST LOW (reset)...")
        line.set_value(0)  # Active LOW reset
        time.sleep(0.1)    # Hold reset for 100ms
        
        print("  Releasing RST HIGH...")
        line.set_value(1)  # Release reset
        time.sleep(0.5)    # Wait for GPS to initialize
        
        # Release line
        line.release()
        chip.close()
        
        print("  ✓ Hardware reset complete")
        print("  Waiting 2 seconds for GPS to stabilize...")
        time.sleep(2.0)
        
        return True
        
    except Exception as e:
        print(f"  ✗ ERROR: Hardware reset failed: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Test GPS communication at various baud rates',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test all baud rates without reset
  python3 scripts/test_gps_baud_rates.py
  
  # Test with hardware reset via GPIO 264 (PI8, pin 3)
  python3 scripts/test_gps_baud_rates.py --rst-gpio 264
  
  # Test without hardware reset
  python3 scripts/test_gps_baud_rates.py --no-reset
  
  # Test specific port with verbose output
  python3 scripts/test_gps_baud_rates.py --port /dev/ttyS5 --rst-gpio 264 --verbose
        """
    )
    parser.add_argument('--port', default='/dev/ttyS5', 
                       help='Serial port (default: /dev/ttyS5)')
    parser.add_argument('--rst-gpio', type=int, default=None,
                       help='GPIO number for RST pin control (recommended: 264 for PI8/pin 3)')
    parser.add_argument('--no-reset', action='store_true',
                       help='Skip hardware reset even if RST GPIO is specified')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed debug output')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("GPS Baud Rate Test Script")
    print("=" * 60)
    print(f"\nPort: {args.port}")
    print(f"Baud rates to test: {', '.join(map(str, BAUD_RATES))}")
    
    if args.rst_gpio and not args.no_reset:
        print(f"RST GPIO: {args.rst_gpio} (PI8, pin 3) - will perform hardware reset")
    elif args.rst_gpio:
        print(f"RST GPIO: {args.rst_gpio} (PI8, pin 3) - reset disabled by --no-reset")
    else:
        print("RST GPIO: Not specified (no hardware reset)")
    
    print("\nThis script will test GPS communication at multiple baud rates.")
    print("Press Ctrl+C to cancel, or wait 3 seconds to continue...")
    
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        print("\n\nCancelled by user.")
        sys.exit(0)
    
    # Perform hardware reset if requested
    if args.rst_gpio and not args.no_reset:
        if not reset_gps_hardware(args.rst_gpio, verbose=args.verbose):
            print("\nWARNING: Hardware reset failed, but continuing with tests...")
    
    # Test each baud rate
    all_results = []
    for baud in BAUD_RATES:
        result = test_baud_rate(args.port, baud, verbose=args.verbose)
        all_results.append(result)
        time.sleep(0.5)  # Brief pause between tests
    
    # Print summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    successful_bauds = []
    for result in all_results:
        if not result.get('port_open', False):
            print(f"  {result['baud']:6d} baud: ✗ Port open failed")
            continue
        
        success_count = sum([
            result.get('automatic_output') is not None,
            result.get('nmea_version') is not None,
            result.get('nmea_pubx') is not None,
            result.get('nmea_pmtk414') is not None,
            result.get('ubx_version') is not None
        ])
        
        if success_count > 0:
            successful_bauds.append(result['baud'])
            print(f"  {result['baud']:6d} baud: ✓ {success_count}/5 tests passed")
            
            # Show what worked
            if result.get('automatic_output'):
                print(f"           - Automatic NMEA output")
            if result.get('nmea_version'):
                print(f"           - NMEA version query: {result['nmea_version'][:60]}")
            if result.get('nmea_pubx'):
                print(f"           - PUBX query: {result['nmea_pubx'][:60]}")
            if result.get('nmea_pmtk414'):
                print(f"           - PMTK414 query: {result['nmea_pmtk414'][:60]}")
            if result.get('ubx_version'):
                print(f"           - UBX version query")
        else:
            print(f"  {result['baud']:6d} baud: ✗ No responses")
    
    print("\n" + "=" * 60)
    if successful_bauds:
        print(f"✓ GPS communication successful at: {', '.join(map(str, successful_bauds))} baud")
        print(f"\nRecommended: Use {successful_bauds[0]} baud in gps.py")
        print("\nIf GPS responded at a different baud rate than 38400:")
        print("  1. The GPS may have been reconfigured to a different baud rate")
        print("  2. Run: python3 scripts/gps_factory_reset.py to restore defaults")
        print("  3. Or manually reconfigure the GPS to 38400 baud")
    else:
        print("✗ GPS did not respond at any baud rate")
        print("\nTroubleshooting:")
        print("  1. Check hardware connections (TX, RX, GND)")
        print("  2. Verify GPS module is powered")
        print("  3. Try hardware reset: python3 scripts/test_gps_baud_rates.py --rst-gpio <GPIO_NUM>")
        print("  4. Check if GPS is in SAFEBOOT mode (may require special recovery)")
        print("  5. Verify serial port: ls -l /dev/ttyS5")
    print("=" * 60)

if __name__ == '__main__':
    main()
