# LoRa Hardware Diagnostic Tools

This directory contains hardware diagnostic scripts for testing and debugging the SX1276 LoRa module.

## Tools

### test_lora_spi.py

Comprehensive SPI and GPIO diagnostic tool for the LoRa module.

**Hardware Tested:**
- SPI communication (bus 1, device 0)
- GPIO control (CS, RST, IRQ pins)
- SX1276 register access
- Module reset sequence

**Test Modes:**

```bash
# Basic SPI communication test (default)
python3 test_lora_spi.py basic

# GPIO pin toggling for oscilloscope verification
python3 test_lora_spi.py pins

# Reset sequence testing
python3 test_lora_spi.py reset

# Repeated version register reads
python3 test_lora_spi.py version

# Multiple register dump
python3 test_lora_spi.py regs

# Run all tests
python3 test_lora_spi.py all
```

**Expected Results:**
- Version register (0x42) should read **0x12** for SX1276
- OpMode register (0x01) should have valid mode bits
- MISO should show ~3.3V amplitude on oscilloscope
- All GPIO pins should toggle cleanly

**Common Issues:**

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Version = 0x00 or 0xFF | No SPI communication | Check wiring, power, module orientation |
| Version = 0x3F | Wrong chip or mode | Check if module is in correct state, verify power |
| Low MISO amplitude | Wiring issue | Check header adapter, verify pin connections |
| No GPIO activity | GPIO not configured | Check device tree overlays, udev rules |

**Hardware Configuration:**

```
Orange Pi Pin -> LoRa Module
PI10 (GPIO 266) -> CS   (Chip Select)
PI0  (GPIO 256) -> RST  (Reset)
PI15 (GPIO 271) -> DIO0 (Interrupt)
SPI1_MOSI       -> MOSI
SPI1_MISO       -> MISO
SPI1_CLK        -> SCK
3.3V            -> VCC
GND             -> GND
```

## Usage Tips

### For Hardware Debugging
1. Start with `basic` test to verify communication
2. Use `pins` test with oscilloscope to verify signal integrity
3. Use `version` test to check for intermittent issues
4. Use `regs` test to see full module state

### For Oscilloscope Verification
```bash
# Run pin toggle test - watch CS and RST on scope
python3 test_lora_spi.py pins

# Look for:
# - Clean 3.3V logic levels
# - No ringing or overshoot
# - Proper timing between transitions
```

### If Module Not Responding
1. Check power: should be 3.3V ±0.1V
2. Check antenna: must be connected (even for local testing)
3. Check wiring: MISO/MOSI not swapped, correct pin adapter
4. Reset module: power cycle if necessary
5. Verify SPI enabled: check `/dev/spidev1.0` exists

## Integration with Main LoRa Node

The main LoRa node is at `../lora.py`. These test scripts use the same:
- SPI configuration (bus 1, device 0, mode 0, 1 MHz)
- GPIO pins (266, 256, 271)
- Register addresses (0x42 version, 0x01 opmode, etc.)

If these test scripts work but `lora.py` doesn't, the issue is in the higher-level logic (modes, packet handling, etc.).

## Troubleshooting

### Version Register Shows 0x3F

This is what you're currently seeing. Possible causes:

1. **Module in wrong mode**
   - Module might be in continuous TX/RX mode
   - Try resetting with `reset` test mode
   - Power cycle the Orange Pi

2. **Wrong chip type**
   - SX1276 should return 0x12
   - SX1262 has different register map
   - Verify module label and datasheet

3. **Chip not responding properly**
   - Check if register 0x01 (OpMode) is readable
   - Try writing and reading back OpMode
   - May need power cycle

4. **Wiring issue**
   - MISO might not be connected properly
   - Check with oscilloscope in `pins` mode
   - Verify continuity with multimeter

### Commands to Try

```bash
# Quick diagnostic
cd ~/argo/nodes/test
python3 test_lora_spi.py basic

# If basic test shows 0x3F, try reset
python3 test_lora_spi.py reset

# Check if value is stable
python3 test_lora_spi.py version

# Full register dump to see module state
python3 test_lora_spi.py regs
```

## See Also

- `../lora.py` - Main LoRa ROS2 node
- `../.cursor/rules/argo-lora-communication.mdc` - Full LoRa system documentation
- `../README-LORA-TO-WAVESHARE.md` - Waveshare compatibility notes

