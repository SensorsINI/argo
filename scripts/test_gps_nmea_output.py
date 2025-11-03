#!/usr/bin/env python3
"""
Temporary test script to diagnose GPS NMEA output regression.

This script tests different GPS configurations to identify what caused
the regression in NMEA sentence output.

Test scenarios:
1. No configuration (just listen for default NMEA)
2. Minimal config (only version query to verify communication)
3. Previous working config (only CFG-MSG, no CFG-PRT)
4. Current config (CFG-PRT + CFG-MSG)
5. Test each UBX command individually

Usage:
    python3 scripts/test_gps_nmea_output.py [scenario]
    
    scenario: 1-5 (default: all)
"""

import serial
import time
import sys
import argparse
from functools import reduce

def ubx_checksum(payload):
    """Calculate UBX checksum for payload."""
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx_cfg_msg(ser, msg_class, msg_id, rates):
    """Send UBX-CFG-MSG to enable/disable a message.
    
    rates: [DDC, UART1, UART2, USB, SPI, Reserved]
    """
    # Format: sync chars, class, id, length, payload, checksum
    payload = bytes([
        rates[0], rates[1], rates[2], rates[3], rates[4], rates[5]
    ])
    msg = bytes([0xB5, 0x62,  # Sync chars
                0x06, 0x01,  # Class: CFG, ID: MSG
                0x08, 0x00,  # Length: 8 bytes
                msg_class, msg_id]) + payload
    
    # Calculate checksum on everything after sync chars
    msg += ubx_checksum(msg[2:])
    ser.write(msg)
    return msg

def send_ubx_cfg_prt(ser, port_id, mode_bytes, in_proto_mask, out_proto_mask):
    """Send UBX-CFG-PRT to configure UART port."""
    payload = bytes([
        0x06, 0x00,  # Class: CFG, ID: PRT
        0x14, 0x00,  # Length: 20 bytes
        port_id,
        0x00,        # Reserved
        0x00, 0x00,  # TX Ready
    ]) + mode_bytes + bytes([
        0x00, 0x00,  # Reserved
        in_proto_mask & 0xFF, (in_proto_mask >> 8) & 0xFF,  # In Protocol Mask
        out_proto_mask & 0xFF, (out_proto_mask >> 8) & 0xFF,  # Out Protocol Mask
        0x00, 0x00,  # Flags
        0x00, 0x00, 0x00, 0x00  # Reserved
    ])
    msg = bytes([0xB5, 0x62]) + payload + ubx_checksum(payload[2:])
    ser.write(msg)
    return msg

def test_scenario(ser, name, setup_func):
    """Run a test scenario."""
    print(f"\n{'='*60}")
    print(f"TEST: {name}")
    print(f"{'='*60}")
    
    # Clear input buffer
    ser.reset_input_buffer()
    time.sleep(0.5)
    
    # Run setup
    if setup_func:
        print("Running setup...")
        setup_func(ser)
        time.sleep(0.5)
    
    # Listen for NMEA sentences
    print("\nListening for NMEA sentences (30 seconds)...")
    print("Press Ctrl+C to stop early\n")
    
    start_time = time.time()
    nmea_count = 0
    sentence_types = set()
    binary_data = []
    
    try:
        while time.time() - start_time < 30.0:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline()
                    data_str = line.decode('ascii', errors='ignore').strip()
                    
                    if data_str:
                        if data_str.startswith('$'):
                            nmea_count += 1
                            sentence_type = data_str.split(',')[0]
                            sentence_types.add(sentence_type)
                            print(f"[{nmea_count:3d}] {data_str[:80]}")
                        else:
                            # Non-NMEA data
                            binary_data.append(repr(data_str[:50]))
                            if len(binary_data) <= 5:  # Only show first few
                                print(f"[BINARY] {repr(data_str[:50])}")
                except Exception as e:
                    print(f"[ERROR reading] {e}")
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped early")
    
    # Results
    print(f"\n{'='*60}")
    print(f"RESULTS for {name}:")
    print(f"  NMEA sentences received: {nmea_count}")
    if sentence_types:
        print(f"  Sentence types: {', '.join(sorted(sentence_types))}")
    else:
        print(f"  Sentence types: NONE")
    if binary_data:
        print(f"  Non-NMEA data samples: {len(binary_data)}")
        print(f"    (First 3: {binary_data[:3]})")
    print(f"{'='*60}\n")
    
    return nmea_count > 0

def scenario_1_no_config(ser):
    """Scenario 1: No configuration, just listen for default NMEA."""
    pass

def scenario_2_version_query_only(ser):
    """Scenario 2: Only version query to verify communication."""
    print("Sending version query...")
    version_cmd = "$PMTK605*31\r\n".encode('ascii')
    ser.write(version_cmd)
    time.sleep(0.5)
    # Read response
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
        print(f"Version response: {response[:100]}")

def scenario_3_cfg_msg_only(ser):
    """Scenario 3: Previous working config - only CFG-MSG, no CFG-PRT."""
    print("Sending CFG-MSG for GGA, RMC, VTG (previous working config)...")
    
    # GGA
    send_ubx_cfg_msg(ser, 0xF0, 0x00, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])
    time.sleep(0.1)
    
    # RMC
    send_ubx_cfg_msg(ser, 0xF0, 0x04, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])
    time.sleep(0.1)
    
    # VTG
    send_ubx_cfg_msg(ser, 0xF0, 0x05, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])
    time.sleep(0.1)
    
    print("CFG-MSG commands sent")

def scenario_4_cfg_prt_and_msg(ser):
    """Scenario 4: Current config - CFG-PRT + CFG-MSG."""
    print("Sending CFG-PRT to enable NMEA protocol...")
    
    # CFG-PRT: 38400 baud 8N1, NMEA output enabled
    mode_bytes = bytes([0x08, 0x00, 0x00, 0x96])  # 8N1, 38400 baud
    send_ubx_cfg_prt(ser, 0x01, mode_bytes, 0x07, 0x07)  # All protocols
    time.sleep(0.2)
    
    print("Sending CFG-MSG for GGA, RMC, VTG...")
    
    # Then CFG-MSG
    send_ubx_cfg_msg(ser, 0xF0, 0x00, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # GGA
    time.sleep(0.1)
    send_ubx_cfg_msg(ser, 0xF0, 0x04, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # RMC
    time.sleep(0.1)
    send_ubx_cfg_msg(ser, 0xF0, 0x05, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # VTG
    time.sleep(0.1)
    
    print("CFG-PRT + CFG-MSG commands sent")

def scenario_6_after_power_on(ser):
    """Scenario 6: Test immediately after power-on (GPS defaults)."""
    print("Testing GPS default configuration after power-on...")
    print("(Assuming GPS was just powered on with factory defaults)")
    pass

def scenario_7_full_init_sequence(ser):
    """Scenario 7: Full initialization sequence (CFG-PRT + CFG-MSG + configure_hot_start)."""
    print("Sending CFG-PRT to enable NMEA protocol...")
    
    # CFG-PRT: 38400 baud 8N1, NMEA output enabled
    mode_bytes = bytes([0x08, 0x00, 0x00, 0x96])  # 8N1, 38400 baud
    send_ubx_cfg_prt(ser, 0x01, mode_bytes, 0x07, 0x07)  # All protocols
    time.sleep(0.2)
    
    print("Sending CFG-MSG for GGA, RMC, VTG...")
    
    # CFG-MSG commands
    send_ubx_cfg_msg(ser, 0xF0, 0x00, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # GGA
    time.sleep(0.1)
    send_ubx_cfg_msg(ser, 0xF0, 0x04, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # RMC
    time.sleep(0.1)
    send_ubx_cfg_msg(ser, 0xF0, 0x05, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # VTG
    time.sleep(0.5)
    
    # Flush ACKs
    if ser.in_waiting > 0:
        ser.read(ser.in_waiting)
    
    print("Sending configure_hot_start() commands...")
    
    # CFG-BAT (backup battery)
    bat_payload = bytes([0x01, 0x00, 0x00, 0x00])
    bat_msg = bytes([0xB5, 0x62, 0x06, 0x09, 0x04, 0x00]) + bat_payload
    bat_msg += ubx_checksum(bat_msg[2:])
    ser.write(bat_msg)
    time.sleep(0.1)
    print("  CFG-BAT sent")
    
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
    nav5_msg = bytes([0xB5, 0x62, 0x06, 0x24, 0x24, 0x00]) + nav5_payload
    nav5_msg += ubx_checksum(nav5_msg[2:])
    ser.write(nav5_msg)
    time.sleep(0.1)
    print("  CFG-NAV5 sent")
    
    # CFG-PMS (power management)
    pms_payload = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    pms_msg = bytes([0xB5, 0x62, 0x06, 0x86, 0x08, 0x00]) + pms_payload
    pms_msg += ubx_checksum(pms_msg[2:])
    ser.write(pms_msg)
    time.sleep(0.1)
    print("  CFG-PMS sent")
    
    # Flush ACKs after hot start config
    time.sleep(0.5)
    if ser.in_waiting > 0:
        ser.read(ser.in_waiting)
        print("  Flushed UBX ACKs")
    
    print("Full initialization sequence complete")

def scenario_5_individual_commands(ser):
    """Scenario 5: Test each command individually to find the culprit."""
    print("\nTesting individual commands...")
    
    # Test CFG-PRT with different protocol masks
    print("\n5a: CFG-PRT with NMEA disabled (UBX only)")
    mode_bytes = bytes([0x08, 0x00, 0x00, 0x96])
    send_ubx_cfg_prt(ser, 0x01, mode_bytes, 0x01, 0x01)  # UBX only
    time.sleep(0.5)
    test_scenario(ser, "CFG-PRT UBX only", None)
    
    print("\n5b: CFG-PRT with NMEA enabled")
    send_ubx_cfg_prt(ser, 0x01, mode_bytes, 0x07, 0x07)  # All protocols
    time.sleep(0.5)
    test_scenario(ser, "CFG-PRT NMEA enabled", None)
    
    print("\n5c: CFG-MSG after NMEA enabled")
    send_ubx_cfg_msg(ser, 0xF0, 0x00, [0x00, 0x01, 0x00, 0x01, 0x00, 0x00])  # GGA
    time.sleep(0.5)
    test_scenario(ser, "CFG-MSG GGA", None)

def main():
    parser = argparse.ArgumentParser(description='Test GPS NMEA output configurations')
    parser.add_argument('scenario', type=int, nargs='?', default=0,
                       help='Scenario to test (1-6, or 0 for all)')
    parser.add_argument('--port', default='/dev/ttyS5',
                       help='Serial port (default: /dev/ttyS5)')
    parser.add_argument('--baud', type=int, default=38400,
                       help='Baud rate (default: 38400)')
    
    args = parser.parse_args()
    
    print("GPS NMEA Output Diagnostic Test")
    print(f"Port: {args.port}, Baud: {args.baud}")
    print("\nThis script will test different GPS configurations")
    print("to identify what caused the NMEA output regression.")
    print("\nWARNING: This will modify GPS configuration!")
    print("Power cycle the GPS after testing to restore defaults.")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        print(f"\nSerial port opened: {args.port}")
        time.sleep(1.0)  # Give GPS time to initialize
        
        scenarios = {
            1: ("No configuration", scenario_1_no_config),
            2: ("Version query only", scenario_2_version_query_only),
            3: ("CFG-MSG only (previous working)", scenario_3_cfg_msg_only),
            4: ("CFG-PRT + CFG-MSG (current)", scenario_4_cfg_prt_and_msg),
            5: ("Individual commands", scenario_5_individual_commands),
            6: ("After power-on (defaults)", scenario_6_after_power_on),
            7: ("Full init sequence (includes configure_hot_start)", scenario_7_full_init_sequence),
        }
        
        if args.scenario == 0:
            # Run all scenarios
            for num, (name, func) in scenarios.items():
                if not test_scenario(ser, name, func):
                    print(f"\n⚠️  {name} produced NO NMEA output!")
                time.sleep(2)  # Pause between tests
        else:
            if args.scenario in scenarios:
                name, func = scenarios[args.scenario]
                test_scenario(ser, name, func)
            else:
                print(f"Invalid scenario: {args.scenario}")
                print("Valid scenarios: 1-7, or 0 for all")
                sys.exit(1)
        
        ser.close()
        print("\n" + "="*60)
        print("Testing complete!")
        print("="*60)
        print("\nRECOMMENDATION: Power cycle the GPS to restore defaults.")
        print("The GPS configuration may have been modified during testing.")
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        if 'ser' in locals():
            ser.close()
        sys.exit(1)

if __name__ == '__main__':
    main()
