#!/usr/bin/env python3
"""
Enable NMEA output on u-blox M9 GPS using CFG-VALSET.

This script uses the newer CFG-VALSET method to configure NMEA output,
which may work better than CFG-PRT after firmware updates.

Usage:
    python3 scripts/gps_enable_nmea.py [port] [baud_rate]
"""

import serial
import sys
import time

def calculate_ubx_checksum(data):
    """Calculate UBX checksum (CK_A, CK_B) for message."""
    ck_a = 0
    ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx_command(port, msg_class, msg_id, payload, timeout=1.0):
    """Send UBX command and wait for ACK."""
    # Build message
    payload_len = len(payload)
    msg = bytes([0xB5, 0x62,  # Sync chars
                msg_class, msg_id,  # Class, ID
                payload_len & 0xFF, (payload_len >> 8) & 0xFF])  # Length
    msg += payload
    msg += calculate_ubx_checksum(msg[2:])
    
    # Flush input
    port.reset_input_buffer()
    
    # Send message
    port.write(msg)
    port.flush()
    
    # Wait for ACK
    start_time = time.time()
    response = bytearray()
    while time.time() - start_time < timeout:
        if port.in_waiting > 0:
            chunk = port.read(port.in_waiting)
            response.extend(chunk)
        time.sleep(0.05)
    
    # Look for ACK-ACK (0x05, 0x01) or ACK-NAK (0x05, 0x00)
    resp_bytes = bytes(response)
    for i in range(len(resp_bytes) - 9):
        if (resp_bytes[i:i+2] == b'\xB5\x62' and 
            resp_bytes[i+2] == 0x05 and 
            resp_bytes[i+3] in (0x01, 0x00) and
            resp_bytes[i+4] == 0x02 and resp_bytes[i+5] == 0x00 and
            resp_bytes[i+6] == msg_class and resp_bytes[i+7] == msg_id):
            is_ack = resp_bytes[i+3] == 0x01
            return is_ack
    
    return False

def enable_nmea_output_cfg_valset(port):
    """Enable NMEA output using CFG-VALSET (newer method)."""
    print("Configuring NMEA output using CFG-VALSET...")
    
    # CFG-VALSET format:
    # Version (1 byte): 0x00 = transactionless, 0x01 = transaction, 0x02 = transaction with commit
    # Layers (1 byte): 0x01 = RAM, 0x02 = BBR, 0x04 = Flash
    # Reserved (2 bytes): 0x0000
    # Key-Value pairs: Key (4 bytes, little-endian) + Value (variable length)
    
    # CFG-UART1OUTPROT-NMEA: Enable NMEA on UART1 output
    # Key: 0x10730002 (CFG-UART1OUTPROT-NMEA)
    # Value: 1 byte, 0x01 = enabled
    
    # CFG-UART1OUTPROT-UBX: Keep UBX enabled (optional, for compatibility)
    # Key: 0x10730001 (CFG-UART1OUTPROT-UBX)
    # Value: 1 byte, 0x01 = enabled
    
    # Build CFG-VALSET payload
    version = 0x01  # Transaction mode (safer)
    layers = 0x07   # RAM + BBR + Flash (save to all layers)
    reserved = 0x0000
    
    # Enable NMEA on UART1 output
    nmea_key = 0x10730002  # CFG-UART1OUTPROT-NMEA
    nmea_value = 0x01      # Enabled
    
    # Enable UBX on UART1 output (keep both enabled)
    ubx_key = 0x10730001   # CFG-UART1OUTPROT-UBX
    ubx_value = 0x01      # Enabled
    
    payload = bytes([
        version,
        layers,
        reserved & 0xFF, (reserved >> 8) & 0xFF,
        # NMEA key (little-endian)
        nmea_key & 0xFF, (nmea_key >> 8) & 0xFF, (nmea_key >> 16) & 0xFF, (nmea_key >> 24) & 0xFF,
        # NMEA value
        nmea_value,
        # UBX key (little-endian)
        ubx_key & 0xFF, (ubx_key >> 8) & 0xFF, (ubx_key >> 16) & 0xFF, (ubx_key >> 24) & 0xFF,
        # UBX value
        ubx_value
    ])
    
    print("  Sending CFG-VALSET to enable NMEA output...")
    result = send_ubx_command(port, 0x06, 0x8A, payload, timeout=2.0)
    
    if result:
        print("  ✓ CFG-VALSET ACK received - NMEA output enabled")
        return True
    else:
        print("  ⚠ CFG-VALSET not acknowledged")
        return False

def enable_nmea_sentences_cfg_msg(port):
    """Enable specific NMEA sentences using CFG-MSG."""
    print("\nEnabling NMEA sentences (GGA, RMC, VTG)...")
    
    sentences = [
        (0xF0, 0x00, "GGA"),  # GGA
        (0xF0, 0x04, "RMC"),  # RMC
        (0xF0, 0x05, "VTG"),  # VTG
    ]
    
    success_count = 0
    for msg_class, msg_id, name in sentences:
        # CFG-MSG: Enable message on UART1 (rate=1 = every measurement cycle)
        payload = bytes([
            msg_class, msg_id,  # Message class and ID
            0x00,  # Rate on DDC (I2C)
            0x01,  # Rate on UART1 (our connection)
            0x00,  # Rate on UART2
            0x01,  # Rate on USB
            0x00,  # Rate on SPI
            0x00   # Reserved
        ])
        
        print(f"  Enabling {name}...")
        result = send_ubx_command(port, 0x06, 0x01, payload, timeout=1.0)
        if result:
            print(f"    ✓ {name} enabled")
            success_count += 1
        else:
            print(f"    ⚠ {name} not acknowledged")
        time.sleep(0.1)
    
    return success_count == len(sentences)

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS5'
    baud_rate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    
    print(f"Opening {port_name} at {baud_rate} baud...")
    
    try:
        port = serial.Serial(port_name, baud_rate, timeout=1.0)
        time.sleep(0.5)
        
        # Flush existing data
        print("Flushing existing data...")
        for _ in range(3):
            if port.in_waiting > 0:
                port.read(port.in_waiting)
            time.sleep(0.2)
        port.reset_input_buffer()
        port.reset_output_buffer()
        
        # Try CFG-VALSET method first (newer, may work better)
        print("\n=== Method 1: CFG-VALSET (newer configuration method) ===")
        if enable_nmea_output_cfg_valset(port):
            print("\n✓ NMEA output protocol enabled via CFG-VALSET")
        else:
            print("\n⚠ CFG-VALSET failed, trying CFG-MSG for sentences...")
        
        # Enable specific NMEA sentences
        if enable_nmea_sentences_cfg_msg(port):
            print("\n✓ NMEA sentences enabled")
        else:
            print("\n⚠ Some NMEA sentences may not be enabled")
        
        print("\n=== Testing NMEA output ===")
        print("Waiting 3 seconds to see if NMEA sentences appear...")
        time.sleep(3.0)
        
        if port.in_waiting > 0:
            data = port.read(port.in_waiting).decode('ascii', errors='ignore')
            nmea_lines = [line for line in data.split('\n') if line.strip().startswith('$')]
            if nmea_lines:
                print(f"\n✓ NMEA sentences detected! ({len(nmea_lines)} sentences)")
                for line in nmea_lines[:5]:  # Show first 5
                    print(f"  {line.strip()}")
            else:
                print("\n⚠ No NMEA sentences detected (still seeing binary data?)")
                print(f"  Raw data sample: {repr(data[:100])}")
        else:
            print("\n⚠ No data received from GPS")
        
        port.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted")
        if port.is_open:
            port.close()
        sys.exit(1)

if __name__ == '__main__':
    main()

