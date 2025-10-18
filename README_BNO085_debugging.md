# BNO085 Debugging Guide

This document provides a comprehensive guide for debugging the BNO085 IMU sensor integration with the Argo sailboat system.

## Overview

The BNO085 (Adafruit 9-DOF Orientation IMU Fusion Breakout) is a 9-axis sensor that combines accelerometer, gyroscope, and magnetometer with an ARM Cortex M0+ processor for sensor fusion. It communicates via I2C using the SHTP (Sensor Hub Transport Protocol).

## Hardware Information

### I2C Addresses
- **Primary Address**: 0x4A (SA0 pin tied low)
- **Secondary Address**: 0x4B (SA0 pin tied high)
- **Detection**: The sensor is detected during I2C scan but may be unresponsive to communication

### Pin Configuration
- **VCC**: 3.3V power supply
- **GND**: Ground
- **SDA**: I2C data line
- **SCL**: I2C clock line
- **SA0**: Address select pin (determines I2C address)

## Current Status

### ✅ Completed
- [x] Dual sensor detection (ICM-20948 and BNO085)
- [x] SHTP protocol implementation based on datasheet
- [x] Comprehensive debugging output
- [x] Version checking via Product ID request
- [x] Sensor report enabling
- [x] Centralized screen clearing control
- [x] Standalone test script

### ❌ Current Issues
- **I2C Communication Failure**: BNO085 responds to I2C scan but fails all communication attempts
- **Error**: `[Errno 6] No such device or address` on all I2C operations
- **Status**: Hardware connectivity issue or sensor initialization problem

## Files

### Core Implementation
- **`nodes/imu.py`**: Main IMU node with dual sensor support
- **`test_bno085.py`**: Standalone BNO085 test script
- **`README_BNO085_debugging.md`**: This debugging guide

### Key Classes
- **`BNO085`**: BNO085 sensor class with SHTP protocol implementation
- **`ImuNode`**: Main ROS2 node supporting both ICM-20948 and BNO085
- **`BNO085Test`**: Standalone test class for debugging

## Testing

### Quick Test
```bash
# Test BNO085 communication
python3 test_bno085.py --debug

# Test with different address
python3 test_bno085.py --address 0x4b --debug
```

### Full IMU Node Test
```bash
# Test with debug output (screen clearing disabled)
python3 nodes/imu.py --debug
```

### I2C Bus Scan
```bash
# Check for I2C devices
i2cdetect -y 0
```

## SHTP Protocol Implementation

### Header Format
```
Byte 0: Length LSB (little-endian)
Byte 1: Length MSB
Byte 2: Channel
Byte 3: Sequence Number
```

### Channels
- **0x00**: SHTP command channel
- **0x01**: Executable channel (reset, on, sleep)
- **0x02**: Sensor hub control channel (SH-2)
- **0x03**: Input sensor reports

### Key Commands
- **Product ID Request**: Report ID 0xF9 on channel 0x02
- **Set Feature Command**: Report ID 0xFD on channel 0x02
- **Reset Command**: Command 0x01 on channel 0x01

## Debugging Steps

### 1. Hardware Check
- [ ] Verify BNO085 is properly connected
- [ ] Check power supply (3.3V)
- [ ] Verify I2C pull-up resistors
- [ ] Check SA0 pin configuration

### 2. I2C Communication
- [ ] Run `i2cdetect -y 0` to confirm device presence
- [ ] Test basic I2C read/write operations
- [ ] Check for I2C bus conflicts

### 3. SHTP Protocol
- [ ] Test Product ID request/response
- [ ] Verify header parsing
- [ ] Test sensor report enabling
- [ ] Check packet sequence numbers

### 4. Power Management
- [ ] Try power cycling the system
- [ ] Test wake-up sequences
- [ ] Check for sleep mode issues

## Known Issues

### I2C Communication Failure
**Symptoms**: Device detected in I2C scan but all communication fails
**Error**: `[Errno 6] No such device or address`
**Possible Causes**:
- Sensor in sleep/power-down mode
- I2C bus timing issues
- Hardware connection problems
- Clock stretching not supported

### Solutions to Try
1. **Power Cycle**: Complete system power cycle
2. **Wake-up Sequence**: Send simple I2C write to wake device
3. **Timing Adjustments**: Add delays between operations
4. **Hardware Check**: Verify connections and power

## Datasheet References

### Key Sections
- **1.3.2.1**: I2C Operation
- **1.3.2.2**: I2C Protocol
- **1.4.1**: SHTP Protocol
- **1.4.2**: Report Structure
- **1.4.5**: Sensor Reports

### Important Notes
- BNO080 does NOT support repeated start conditions
- Clock stretching is supported and may be used
- Maximum packet size: 32766 bytes minus header
- Little-endian format for multi-byte values

## Next Steps

### Immediate Actions
1. **Power Cycle Test**: Test after system power cycle
2. **Hardware Verification**: Check physical connections
3. **Alternative Address**: Test with SA0 pin high (0x4B address)

### Development Options
1. **Focus on ICM-20948**: Ensure existing sensor works with dual support
2. **BNO085 Library**: Consider using existing BNO085 library
3. **Hardware Debugging**: Use oscilloscope to check I2C signals

### Long-term Goals
1. **Complete BNO085 Integration**: Full sensor fusion capabilities
2. **Calibration Support**: Magnetometer calibration for BNO085
3. **Performance Optimization**: Compare with ICM-20948 performance

## Troubleshooting Commands

```bash
# Check I2C devices
i2cdetect -y 0

# Test BNO085 communication
python3 test_bno085.py --debug

# Test full IMU node
python3 nodes/imu.py --debug

# Check system logs
journalctl -u argo-launch.service -f

# Monitor I2C bus
i2cdump -y 0 0x4a
```

## Contact Information

For questions or issues related to BNO085 integration:
- Check this README first
- Review the test script output
- Examine the datasheet sections referenced above
- Test on different hardware if available

## Version History

- **v1.0**: Initial BNO085 integration attempt
- **v1.1**: Added SHTP protocol implementation
- **v1.2**: Added comprehensive debugging
- **v1.3**: Created standalone test script
- **v1.4**: Fixed I2C protocol based on datasheet

---

*Last updated: After power cycle test preparation*
