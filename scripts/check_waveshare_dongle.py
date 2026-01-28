#!/usr/bin/env python3
"""
Check and configure Waveshare USB-TO-LoRa dongle

This script helps verify and configure the Waveshare dongle settings
to match Argo's LoRa configuration.

Required settings:
- NETID=18 (0x12) - Must match Argo sync word
- MODE=1 (Stream mode)
- SF=7 (Spreading factor)
- BW=0 (125 kHz bandwidth)
- CR=1 (Code rate 4/5)
- TXCH=23 (433 MHz for LF version)
- RXCH=23 (433 MHz for LF version)
- KEY=0 (No encryption)
"""

import serial
import time
import sys
import argparse

def enter_at_mode(ser):
    """Enter AT command mode"""
    ser.write(b"+++\r\n")
    time.sleep(1.0)  # Wait for response
    response = ser.read(ser.in_waiting)
    if b"OK" in response or b"OK\r\n" in response:
        return True
    return False

def exit_at_mode(ser):
    """Exit AT command mode"""
    ser.write(b"AT+EXIT\r\n")
    time.sleep(0.5)
    ser.read(ser.in_waiting)

def send_at_command(ser, command):
    """Send AT command and return response"""
    ser.write((command + "\r\n").encode())
    time.sleep(0.5)
    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    return response.strip()

def check_configuration(ser):
    """Check current dongle configuration"""
    print("\n=== Checking Waveshare Dongle Configuration ===\n")
    
    config = {}
    
    # Check all settings
    commands = {
        'VER': 'AT+VER?',
        'MODE': 'AT+MODE?',
        'SF': 'AT+SF?',
        'BW': 'AT+BW?',
        'CR': 'AT+CR?',
        'NETID': 'AT+NETID?',
        'TXCH': 'AT+TXCH?',
        'RXCH': 'AT+RXCH?',
        'KEY': 'AT+KEY?',
    }
    
    for key, cmd in commands.items():
        response = send_at_command(ser, cmd)
        config[key] = response
        print(f"{key:8s}: {response}")
    
    return config

def configure_dongle(ser, force=False):
    """Configure dongle with required settings"""
    print("\n=== Configuring Waveshare Dongle ===\n")
    
    required_settings = {
        'NETID': '18',
        'MODE': '1',  # Stream mode
        'SF': '7',    # Spreading factor 7
        'BW': '0',    # 125 kHz
        'CR': '1',    # Code rate 4/5
        'TXCH': '23', # 433 MHz (LF version)
        'RXCH': '23', # 433 MHz (LF version)
        'KEY': '0',   # No encryption
    }
    
    for param, value in required_settings.items():
        cmd = f"AT+{param}={value}"
        response = send_at_command(ser, cmd)
        print(f"Set {param}={value}: {response}")
        if "OK" not in response and "ERROR" not in response:
            print(f"  ⚠️  Warning: Unexpected response")
    
    print("\n✅ Configuration complete!")

def main():
    parser = argparse.ArgumentParser(
        description='Check and configure Waveshare USB-TO-LoRa dongle',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Check current configuration
  python3 check_waveshare_dongle.py --port /dev/ttyACM0

  # Configure dongle with required settings
  python3 check_waveshare_dongle.py --port /dev/ttyACM0 --configure

  # Force reconfiguration (even if already correct)
  python3 check_waveshare_dongle.py --port /dev/ttyACM0 --configure --force
        """
    )
    parser.add_argument('--port', '-p', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--configure', '-c', action='store_true',
                       help='Configure dongle with required settings')
    parser.add_argument('--force', '-f', action='store_true',
                       help='Force reconfiguration even if settings match')
    
    args = parser.parse_args()
    
    try:
        print(f"Connecting to {args.port} at {args.baud} baud...")
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(0.5)  # Wait for connection
        
        # Enter AT mode
        print("Entering AT command mode...")
        if not enter_at_mode(ser):
            print("⚠️  Warning: Could not confirm AT mode entry")
            print("  (This is OK if dongle is already in AT mode)")
        
        # Check configuration
        config = check_configuration(ser)
        
        # Check if configuration matches requirements
        needs_config = False
        if 'NETID' in config and '18' not in config['NETID']:
            needs_config = True
            print("\n⚠️  NETID is not 18 - needs configuration")
        if 'MODE' in config and '1' not in config['MODE']:
            needs_config = True
            print("⚠️  MODE is not 1 (stream) - needs configuration")
        
        # Configure if requested or needed
        if args.configure or (needs_config and not args.force):
            configure_dongle(ser, force=args.force)
            # Re-check after configuration
            print("\n=== Verifying Configuration ===\n")
            check_configuration(ser)
        elif needs_config:
            print("\n⚠️  Configuration does not match requirements!")
            print("   Run with --configure to fix")
        
        # Exit AT mode
        print("\nExiting AT command mode...")
        exit_at_mode(ser)
        
        ser.close()
        print("\n✅ Done!")
        
    except serial.SerialException as e:
        print(f"\n❌ Error: Could not open serial port: {e}")
        print(f"\nTroubleshooting:")
        print(f"  1. Check if dongle is connected: ls -l {args.port}")
        print(f"  2. Check permissions: groups | grep dialout")
        print(f"  3. If not in dialout group: sudo usermod -a -G dialout $USER")
        print(f"     (then log out and back in)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        if 'ser' in locals():
            try:
                exit_at_mode(ser)
                ser.close()
            except:
                pass
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
