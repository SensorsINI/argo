# SparkFun NEO-M9N GPS Firmware Update Guide

This directory contains firmware files and tools for updating the u-blox NEO-M9N GPS module firmware on the Argo sailboat.

## Contents

- **u-blox-m9n-v4.04-firmware.bin** - Firmware binary file (v4.04)
- **ubxfwupdate.exe** - Windows firmware update tool (runs via Wine on Linux)
- **flash.xml** - Flash Information Structure file (required for firmware updates)
- **flash.txt** - Flash Information Structure text file (reference)

## When to Flash Firmware

Flash firmware when:

1. **GPS is in bootloader mode** - GPS outputs "ROM BOOT" version instead of firmware version
   - Symptoms: No NMEA output, no PPS pulses, GPS not acquiring satellites
   - Check with: `python3 scripts/test_gps_baud_rates.py` or `python3 nodes/gps.py --debug`
   - Look for: "ROM BOOT 1.02" in version output

2. **Firmware update available** - New firmware version with bug fixes or features
   - Check u-blox website for latest firmware: https://www.u-blox.com/en/product/neo-m9n-module

3. **GPS malfunctioning** - GPS not responding or behaving erratically
   - After trying other troubleshooting steps (reset, cold start, etc.)

## Prerequisites

### On Linux (Orange Pi / Development Machine)

1. **Install Wine** (if not already installed):
   ```bash
   sudo apt-get install wine
   ```

2. **Connect GPS** - Ensure GPS is connected and accessible:
   - On Orange Pi: `/dev/ttyS5` (UART5)
   - On development machine: `/dev/ttyACM0` (USB) or `/dev/ttyS*` (serial)

3. **Check GPS connection**:
   ```bash
   # Test communication
   python3 scripts/test_gps_baud_rates.py --port /dev/ttyS5
   ```

## Flashing Firmware

### Quick Start

From the Argo project root directory:

```bash
# Flash firmware (Orange Pi - UART5)
./scripts/gps_firmware_flash_wine.sh \
    firmware/sparkfun_neo_n9m/u-blox-m9n-v4.04-firmware.bin \
    /dev/ttyS5 \
    firmware/sparkfun_neo_n9m/ubxfwupdate.exe \
    firmware/sparkfun_neo_n9m/flash.xml

# Flash firmware (Development machine - USB)
./scripts/gps_firmware_flash_wine.sh \
    firmware/sparkfun_neo_n9m/u-blox-m9n-v4.04-firmware.bin \
    /dev/ttyACM0 \
    firmware/sparkfun_neo_n9m/ubxfwupdate.exe \
    firmware/sparkfun_neo_n9m/flash.xml
```

### Detailed Steps

1. **Navigate to project root**:
   ```bash
   cd ~/argo
   ```

2. **Run the flash script**:
   ```bash
   ./scripts/gps_firmware_flash_wine.sh \
       firmware/sparkfun_neo_n9m/u-blox-m9n-v4.04-firmware.bin \
       <port> \
       firmware/sparkfun_neo_n9m/ubxfwupdate.exe \
       firmware/sparkfun_neo_n9m/flash.xml
   ```

   Replace `<port>` with:
   - `/dev/ttyS5` - Orange Pi UART5 (default)
   - `/dev/ttyACM0` - USB connection
   - `/dev/ttyS*` - Other serial ports

3. **Wait for completion**:
   - Script will show progress
   - Look for "Firmware Update SUCCESS" message
   - GPS will reboot automatically

4. **Verify firmware version**:
   ```bash
   # Wait 10 seconds for GPS to reboot
   sleep 10
   
   # Test GPS communication
   python3 nodes/gps.py --debug
   ```
   
   Should see firmware version (not "ROM BOOT") and NMEA sentences.

## Post-Flash Configuration

After flashing firmware, the GPS may need NMEA output re-enabled:

### Automatic (via gps.py)

The `gps.py` node automatically configures NMEA output using CFG-VALSET:
- Default baud rate: **38400** (factory default after firmware update)
- Automatically enables NMEA sentences (GGA, RMC, VTG)
- Uses CFG-VALSET method (more reliable than CFG-PRT after firmware updates)

### Manual Configuration

If automatic configuration fails, manually enable NMEA:

```bash
# Enable NMEA output at 38400 baud
python3 scripts/gps_enable_nmea.py /dev/ttyS5 38400
```

## Troubleshooting

### "No response received" or "GPS not responding"

1. **Check baud rate** - GPS may be at different baud rate after firmware update:
   ```bash
   # Try different baud rates
   python3 scripts/test_gps_baud_rates.py --port /dev/ttyS5
   ```

2. **Check connection**:
   ```bash
   # Verify port exists and permissions
   ls -l /dev/ttyS5
   # May need to add user to dialout group
   sudo usermod -aG dialout $USER
   ```

3. **Try different port** - If using USB, try `/dev/ttyACM0` or `/dev/ttyUSB0`

### "ERROR: Could not open the FIS file flash.xml"

- Ensure `flash.xml` is in the same directory or specify path:
  ```bash
  ./scripts/gps_firmware_flash_wine.sh \
      firmware/sparkfun_neo_n9m/u-blox-m9n-v4.04-firmware.bin \
      /dev/ttyS5 \
      firmware/sparkfun_neo_n9m/ubxfwupdate.exe \
      firmware/sparkfun_neo_n9m/flash.xml
  ```

### "Firmware flash failed" or GPS still in bootloader mode

1. **Check error messages** - Look for specific error in output
2. **Try again** - Sometimes retry works
3. **Check GPS power** - Ensure stable power supply during flash
4. **Check connection** - Ensure serial connection is stable

### GPS outputs binary data instead of NMEA after flash

This is normal - GPS resets to factory defaults (38400 baud, UBX output only).

**Solution**: The `gps.py` node automatically configures NMEA output. If it doesn't work:

1. **Check baud rate** - GPS is at 38400 baud (not 9600):
   ```bash
   # gps.py defaults to 38400, but verify
   python3 nodes/gps.py --debug
   ```

2. **Manually enable NMEA**:
   ```bash
   python3 scripts/gps_enable_nmea.py /dev/ttyS5 38400
   ```

## Getting New Firmware

1. **Check u-blox website**:
   - https://www.u-blox.com/en/product/neo-m9n-module
   - Click "Documentation & resources" tab
   - Look for "Firmware Update" section

2. **Use helper script**:
   ```bash
   ./scripts/gps_firmware_info.sh
   ```

3. **Download firmware file**:
   - Usually named: `NEO-M9N-XX.XX.hex` or `NEO-M9N-firmware.ubx`
   - Place in this directory: `firmware/sparkfun_neo_n9m/`

4. **Update flash script** - Use new firmware file in flash command

## Important Notes

- **Do not interrupt** firmware flashing - can brick the GPS
- **Stable power** required during flash - use battery or stable power supply
- **Serial connection** must be stable - avoid USB hubs if possible
- **Backup current firmware** - Not easily possible, but note current version before flashing
- **Baud rate changes** - GPS resets to 38400 baud after firmware update (factory default)
- **NMEA configuration** - GPS resets to UBX-only output after firmware update (needs re-configuration)

## Related Scripts

- `scripts/gps_firmware_flash_wine.sh` - Main firmware flashing script
- `scripts/gps_firmware_info.sh` - Firmware download information
- `scripts/gps_enable_nmea.py` - Enable NMEA output after flash
- `scripts/test_gps_baud_rates.py` - Test GPS communication at various baud rates
- `scripts/gps_query_port_config.py` - Query GPS port configuration

## References

- **u-blox M9 Interface Description**: https://content.u-blox.com/sites/default/files/documents/u-blox-M9-MDR-2.16_InterfaceDescription_UBX-22037308.pdf
- **Protocol Version**: 35.16 (M9-MDR-2.16)
- **Module**: SparkFun NEO-M9N (u-blox NEO-M9N)
- **Current Firmware**: v4.04 (as of last update)

## Version History

- **v4.04** - Current firmware in this directory
  - Flashed: November 2024
  - Fixed bootloader mode issue
  - Restored NMEA output functionality

