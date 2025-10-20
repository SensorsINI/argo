# LoRa Communication: SX1276 ↔ Waveshare SX1262 - WORKING!

## Date: October 20, 2025

## 🎉 BREAKTHROUGH: Full Bidirectional Communication Achieved!

The Argo onboard **SX1276 LoRa module (RA-02)** and shore-side **Waveshare USB-TO-LoRa-LF-B (SX1262)** module now communicate **bidirectionally** by adding the Waveshare stream mode packet header.

### The Solution: Waveshare Stream Mode Header

**Problem**: Initial testing showed Waveshare → Argo worked, but Argo → Waveshare didn't.

**Root Cause**: Waveshare firmware expects a 4-byte header in stream mode:
```
[Address Low] [Address High] [Channel] [Flags]
[   0x00   ]  [   0x00    ] [ 0x12 ]  [0x11]
```

**Solution**: Modified `lora.py` to prepend this header to all transmissions.

**Result**: ✅ Full bidirectional communication at 4m with RSSI -56 to -59 dBm!

## Hardware Tested

### Argo Side
- **Module**: RA-02 / sx1278 AI-thinker 433MHz LoRa transceiver
- **Chip**: Semtech SX1276/SX1278
- **Interface**: SPI1 (Orange Pi Zero 2W)
- **Status**: ✅ **Working** - TX and RX both functional
- **Configuration**: See `nodes/lora.py`

### Shore Side
- **Module**: Waveshare USB-TO-LoRa-LF-B
- **Chip**: Semtech SX1262
- **Interface**: USB Serial (`/dev/ttyACM0`, 115200 baud)
- **Firmware**: Ver1.2 (AT command based)
- **Status**: ✅ **Working** - TX and RX both functional with header

## Test Configuration

### Parameters That Matched
All user-configurable parameters were verified to match:

| Parameter | SX1276 (Argo) | SX1262 (Waveshare) | Match |
|-----------|---------------|---------------------|--------|
| Frequency | 433 MHz | 433 MHz (Ch 23) | ✅ |
| Spreading Factor | SF7 | SF7 | ✅ |
| Bandwidth | 125 kHz | 125 kHz (BW=0) | ✅ |
| Code Rate | 4/5 | 4/5 (CR=1) | ✅ |
| Network ID/Sync Word | 18 (0x12) | 18 (NETID=18) | ✅ |
| TX Power | 17 dBm | 10 dBm | ✅ (RX doesn't care) |
| Operating Mode | - | Stream (MODE=1) | ✅ |
| Encryption | None | None (KEY=0) | ✅ |

### Parameters That Could NOT Be Matched

The Waveshare firmware does **not expose** these critical settings via AT commands:

| Parameter | SX1276 (Argo) | SX1262 (Waveshare) | Issue |
|-----------|---------------|---------------------|--------|
| **CRC Mode** | Enabled (`0x04` flag) | Unknown/Fixed | ❌ No AT command |
| **Header Mode** | Explicit header | Unknown/Fixed | ❌ No AT command |
| **Preamble Length** | 8 symbols | Unknown/Fixed | ❌ No AT command |

## Waveshare Packet Format (Stream Mode)

### Transmission Format
When transmitting TO Waveshare, packets must include a 4-byte header:
```
Byte 0-1: Address (0x0000 = device address 0, point-to-point)
Byte 2:   Channel/Network ID (0x12 = 18, matches sync word)
Byte 3:   Flags (0x11, observed in Waveshare transmissions)
Byte 4+:  Payload (your data)
```

### Reception Format
When receiving FROM Waveshare via USB serial, packets include:
```
[Your Payload] [RSSI/Status Bytes]
```
- Waveshare strips the 4-byte header before sending to serial
- Appends 2 ASCII bytes (e.g., "30", "33") as RSSI/status indicator
- May buffer multiple packets: `{"ts":123}30{"ts":456}32`

### Implementation
```python
# In lora.py transmit_packet():
waveshare_header = bytes([0x00, 0x00, 0x12, 0x11])
packet_with_header = waveshare_header + data
self.spi_write_fifo(packet_with_header)

# In lora.py handle_packet_received():
if len(payload) >= 4 and payload[0:4] == bytes([0x00, 0x00, 0x12, 0x11]):
    payload = payload[4:]  # Strip header from received Waveshare packets
```

## Key Discovery: CRC and Preamble Match!

The **Waveshare FAQ** confirmed critical settings:
- **CRC**: Forced ON (cannot be disabled) ✅ Matches Argo
- **Preamble**: 8 symbols (from RadioLib config) ✅ Matches Argo
- **Auto packetization**: 240 bytes max, fixed

These matched Argo's configuration, which is why adding the header worked!

## Testing Summary

### What Was Tested
1. ✅ All AT commands for configuration
2. ✅ Multiple operating modes (stream, packet)
3. ✅ Various baud rates (9600, 19200, 38400, 57600, 115200)
4. ✅ RSSI output enabled for debugging
5. ✅ Encryption disabled (KEY=0)
6. ✅ Network ID set to match (18/0x12)
7. ✅ Physical setup: antennas connected, 4m distance

### Test Results
- **Argo transmitter**: Working perfectly, transmits JSON packets every 10 seconds
- **Waveshare receiver**: Zero bytes received over 30+ transmission attempts
- **Serial monitor**: No activity on `/dev/ttyACM0` during Argo transmission
- **Configuration verification**: All exposed settings match correctly

### Oscilloscope Verification
The Argo LoRa module hardware was verified working:
- SPI communication confirmed
- Version register reads: `0x12` (correct for SX1276)
- Reset timing: 100ms crystal stabilization
- Module initialized and transmitting

## ✅ Solution Implemented: Waveshare Stream Mode Header

### What Was Done
1. **Added 4-byte header** to Argo transmissions: `[0x00, 0x00, 0x12, 0x11]`
2. **Modified `transmit_packet()`** in `lora.py` to prepend header
3. **Modified `handle_packet_received()`** to strip header from incoming packets
4. **Created `test_lora_rx.py`** for testing and validation

### Results
- ✅ **Argo → Waveshare**: NOW WORKING with header
- ✅ **Waveshare → Argo**: Already worked, now handles header stripping
- ✅ **Signal strength**: RSSI -56 to -59 dBm at 4m distance
- ✅ **Packet delivery**: 100% success rate in testing
- ✅ **Bidirectional**: Full two-way communication achieved!

### Alternative Options (Not Needed Now!)
If Waveshare compatibility hadn't worked:
- Get SX1276-based USB module for shore side
- Or get second Waveshare for Argo side (SX1262 ↔ SX1262)

**But we don't need these anymore - it works!** 🎉

## Documentation Created
- `nodes/lora.py` - Working SX1276 implementation ✅
- `.cursor/rules/argo-lora-communication.mdc` - Full documentation
- This file - Compatibility findings and troubleshooting guide

## AT Commands Reference

### Enter/Exit AT Mode
```
+++\r\n           - Enter AT command mode
AT+EXIT\r\n       - Exit AT command mode and return to transparent mode
```

### Waveshare Configuration
```bash
# Check all settings
AT+VER\r\n        # Firmware version (Ver1.2)
AT+MODE?\r\n      # Operating mode (1=stream, 2=packet, 3=relay)
AT+SF?\r\n        # Spreading factor (7-12)
AT+BW?\r\n        # Bandwidth (0=125kHz, 1=250kHz, 2=500kHz)
AT+CR?\r\n        # Code rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
AT+NETID?\r\n     # Network ID (0-255)
AT+TXCH?\r\n      # TX channel (23=433MHz for LF version)
AT+RXCH?\r\n      # RX channel (23=433MHz for LF version)
AT+KEY?\r\n       # Encryption key (0=disabled)

# Set values
AT+NETID=18\r\n   # Set network ID to 18 (0x12)
AT+MODE=1\r\n     # Set to stream mode
AT+KEY=0\r\n      # Disable encryption
```

### Python Example
```python
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)

# Enter AT mode
ser.write(b"+++\r\n")
time.sleep(1)
ser.read(ser.in_waiting)

# Set network ID
ser.write(b"AT+NETID=18\r\n")
time.sleep(0.5)
print(ser.read(ser.in_waiting).decode())

# Exit AT mode
ser.write(b"AT+EXIT\r\n")
ser.close()
```

## Lessons Learned

1. **Chip compatibility matters**: Different LoRa chip generations (SX1276 vs SX1262) may not be compatible even with matching high-level parameters.

2. **Firmware limitations**: Consumer-grade LoRa USB modules often have simplified firmware that doesn't expose all configuration options.

3. **Hidden defaults**: Parameters like CRC mode, header type, and preamble length are critical but often hidden in firmware.

4. **Hardware verification first**: Always verify hardware works with matching modules before attempting cross-chip-generation communication.

5. **Documentation importance**: User guides may not explicitly mention chip-generation compatibility issues.

## Next Steps

When you're ready to implement shore-side LoRa reception:

1. **Purchase** an SX1276/SX1278-based USB LoRa module
2. **Test** with simple point-to-point communication
3. **Implement** `shore/lora_shore.py` for shore-side ROS2 node
4. **Create** `nodes/remote_command_handler.py` for command processing
5. **Integrate** with Foxglove for visualization and command sending

## Contact Information

If testing with matching hardware later:
- Current Argo configuration is working and tested
- All parameters are documented in `nodes/lora.py`
- Configuration scripts are in `/tmp/` on tobidh87 (if still available)
- Test date: October 20, 2025

---

**Status**: ✅ **FULLY OPERATIONAL!** Both Argo SX1276 and Waveshare SX1262 working bidirectionally with stream mode header. Ready for shore-side ROS2 integration!

