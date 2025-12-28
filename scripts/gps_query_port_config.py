#!/usr/bin/env python3
"""
Query GPS port configuration to see what protocols are enabled.

This script queries the current UART port configuration to see if NMEA output is enabled.
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

def send_ubx_query(port, msg_class, msg_id):
    """Send UBX query and return response."""
    # Build query message
    payload_len = 0
    msg = bytes([0xB5, 0x62,  # Sync chars
                msg_class, msg_id,  # Class, ID
                payload_len & 0xFF, (payload_len >> 8) & 0xFF])  # Length (little-endian)
    
    # Add checksum
    msg += calculate_ubx_checksum(msg[2:])
    
    # Send message
    port.write(msg)
    port.flush()
    
    # Read response
    time.sleep(0.1)
    if port.in_waiting > 0:
        response = port.read(port.in_waiting)
        return response
    return None

def parse_cfg_prt_response(data):
    """Parse CFG-PRT response to extract port configuration."""
    if len(data) < 20:
        return None
    
    # Find UBX message start
    start_idx = data.find(b'\xB5\x62')
    if start_idx == -1:
        return None
    
    # Check if it's CFG-PRT response (class=0x06, id=0x00)
    if len(data) < start_idx + 6:
        return None
    
    if data[start_idx + 2] != 0x06 or data[start_idx + 3] != 0x00:
        return None
    
    # Get length
    length = data[start_idx + 4] + (data[start_idx + 5] << 8)
    if len(data) < start_idx + 8 + length:
        return None
    
    payload = data[start_idx + 6:start_idx + 6 + length]
    
    if len(payload) < 20:
        return None
    
    port_id = payload[0]
    reserved1 = payload[1]
    tx_ready = payload[2] + (payload[3] << 8)
    mode = payload[4:8]
    reserved2 = payload[8:10]
    in_proto_mask = payload[10] + (payload[11] << 8)
    out_proto_mask = payload[12] + (payload[13] << 8)
    flags = payload[14] + (payload[15] << 8)
    reserved3 = payload[16:20]
    
    # Parse mode
    char_len = mode[0] & 0x0F
    parity = (mode[1] >> 0) & 0x03
    n_stop_bits = (mode[1] >> 2) & 0x03
    baud_rate = mode[2] + (mode[3] << 8)
    
    return {
        'port_id': port_id,
        'tx_ready': tx_ready,
        'char_len': char_len,
        'parity': parity,
        'n_stop_bits': n_stop_bits,
        'baud_rate': baud_rate,
        'in_proto_mask': in_proto_mask,
        'out_proto_mask': out_proto_mask,
        'flags': flags
    }

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS5'
    baud_rate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    
    print(f"Opening {port_name} at {baud_rate} baud...")
    
    try:
        port = serial.Serial(port_name, baud_rate, timeout=1.0)
        time.sleep(0.5)
        
        # Flush any existing data - GPS might be outputting continuously
        print("Flushing existing data...")
        for _ in range(3):
            if port.in_waiting > 0:
                discarded = port.read(port.in_waiting)
                print(f"  Discarded {len(discarded)} bytes of existing data")
            time.sleep(0.2)
        
        port.reset_input_buffer()
        port.reset_output_buffer()
        
        print("\nQuerying CFG-PRT (Port Configuration) for UART1 (port ID 1)...")
        print("Sending UBX-CFG-PRT query...")
        
        # Query CFG-PRT for UART1 (port ID 1)
        # CFG-PRT GET requires port ID in payload (1 byte)
        payload = bytes([0x01])  # Port ID 1 (UART1)
        msg_class = 0x06
        msg_id = 0x00
        payload_len = len(payload)
        
        msg = bytes([0xB5, 0x62,  # Sync chars
                    msg_class, msg_id,  # Class, ID
                    payload_len & 0xFF, (payload_len >> 8) & 0xFF])  # Length
        msg += payload
        msg += calculate_ubx_checksum(msg[2:])
        
        print(f"Sending: {msg.hex()}")
        port.write(msg)
        port.flush()
        
        print("Waiting for response (reading for 2 seconds)...")
        # Read for longer to catch the response (GPS might be slow or outputting other data)
        response = bytearray()
        start_time = time.time()
        while time.time() - start_time < 2.0:
            if port.in_waiting > 0:
                chunk = port.read(port.in_waiting)
                response.extend(chunk)
            time.sleep(0.1)
        
        if len(response) > 0:
            print(f"Response ({len(response)} bytes): {response.hex()[:200]}...")
            
            config = parse_cfg_prt_response(bytes(response))
            if config:
                print("\n=== Port Configuration ===")
                print(f"Port ID: {config['port_id']} (1=UART1)")
                print(f"Baud Rate: {config['baud_rate']}")
                print(f"Character Length: {config['char_len']} bits")
                print(f"Parity: {config['parity']} (0=None, 1=Even, 2=Odd)")
                print(f"Stop Bits: {config['n_stop_bits']} (0=1, 1=1.5, 2=2)")
                print(f"\nInput Protocol Mask: 0x{config['in_proto_mask']:04X}")
                print("  Bit 0 (UBX):", "✓ Enabled" if (config['in_proto_mask'] & 0x01) else "✗ Disabled")
                print("  Bit 1 (NMEA):", "✓ Enabled" if (config['in_proto_mask'] & 0x02) else "✗ Disabled")
                print("  Bit 2 (RTCM3):", "✓ Enabled" if (config['in_proto_mask'] & 0x04) else "✗ Disabled")
                print(f"\nOutput Protocol Mask: 0x{config['out_proto_mask']:04X}")
                print("  Bit 0 (UBX):", "✓ Enabled" if (config['out_proto_mask'] & 0x01) else "✗ Disabled")
                print("  Bit 1 (NMEA):", "✓ Enabled" if (config['out_proto_mask'] & 0x02) else "✗ Disabled")
                print("  Bit 2 (RTCM3):", "✓ Enabled" if (config['out_proto_mask'] & 0x04) else "✗ Disabled")
                
                if not (config['out_proto_mask'] & 0x02):
                    print("\n⚠ WARNING: NMEA output is DISABLED!")
                    print("  This is why you're only seeing UBX binary data.")
                    print("  The GPS needs to be configured to enable NMEA output.")
                else:
                    print("\n✓ NMEA output is enabled")
            else:
                print("Could not parse CFG-PRT response")
                print("\nTrying alternative: Query MON-VER to verify GPS is responding...")
                # Try MON-VER query as a test
                mon_ver_payload = b''
                mon_ver_msg = bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00])
                mon_ver_msg += calculate_ubx_checksum(mon_ver_msg[2:])
                
                port.reset_input_buffer()
                port.write(mon_ver_msg)
                port.flush()
                time.sleep(0.5)
                
                if port.in_waiting > 0:
                    mon_ver_resp = port.read(port.in_waiting)
                    print(f"MON-VER response ({len(mon_ver_resp)} bytes): {mon_ver_resp.hex()[:100]}...")
                    if b'\xB5\x62\x0A\x04' in mon_ver_resp:
                        print("✓ GPS is responding to UBX queries")
                    else:
                        print("⚠ GPS response doesn't look like MON-VER")
                else:
                    print("⚠ No response to MON-VER query")
        else:
            print("No response received")
            print("\nTrying MON-VER query to verify GPS communication...")
            # Try MON-VER as a simpler test
            mon_ver_msg = bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00])
            mon_ver_msg += calculate_ubx_checksum(mon_ver_msg[2:])
            
            port.reset_input_buffer()
            port.write(mon_ver_msg)
            port.flush()
            time.sleep(0.5)
            
            if port.in_waiting > 0:
                mon_ver_resp = port.read(port.in_waiting)
                print(f"MON-VER response ({len(mon_ver_resp)} bytes): {mon_ver_resp.hex()[:100]}...")
                if b'\xB5\x62\x0A\x04' in mon_ver_resp:
                    print("✓ GPS is responding to UBX queries")
                    print("  CFG-PRT query might have failed, but GPS is communicating")
                else:
                    print("⚠ GPS response doesn't look like MON-VER")
            else:
                print("⚠ No response to MON-VER query - GPS may not be responding")
        
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
