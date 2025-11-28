#!/bin/bash
# GPS Firmware Flash using Wine (Windows ubxfwupdate.exe on Linux)
#
# This script helps set up Wine and run u-blox ubxfwupdate.exe on Linux
#
# Prerequisites:
#   1. Install Wine: sudo apt-get install wine
#   2. Download u-center from u-blox website (extract ubxfwupdate.exe)
#   3. Download firmware file for NEO-M9N
#
# Usage:
#   ./scripts/gps_firmware_flash_wine.sh [firmware_file.ubx] [port]
#

set -e

FIRMWARE_FILE="${1:-}"
PORT="${2:-/dev/ttyS5}"
UBXFWUPDATE="${3:-./ubxfwupdate.exe}"

echo "============================================================"
echo "GPS Firmware Flash using Wine"
echo "============================================================"

# Check if Wine is installed
if ! command -v wine &> /dev/null; then
    echo "ERROR: Wine is not installed"
    echo "Install with: sudo apt-get install wine"
    exit 1
fi

# Check if ubxfwupdate.exe exists
if [ ! -f "$UBXFWUPDATE" ]; then
    echo "ERROR: ubxfwupdate.exe not found at: $UBXFWUPDATE"
    echo ""
    echo "To get ubxfwupdate.exe:"
    echo "  1. Download u-center from: https://www.u-blox.com/en/product/u-center"
    echo "  2. Extract ubxfwupdate.exe from the installation"
    echo "  3. Place it in the current directory or specify path"
    exit 1
fi

# Check if firmware file is provided
if [ -z "$FIRMWARE_FILE" ]; then
    echo "ERROR: Firmware file not specified"
    echo ""
    echo "Usage: $0 <firmware_file.ubx> [port] [ubxfwupdate.exe]"
    echo ""
    echo "Example:"
    echo "  $0 NEO-M9N-02.01.hex /dev/ttyS5 ./ubxfwupdate.exe"
    exit 1
fi

# Check if firmware file exists
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "ERROR: Firmware file not found: $FIRMWARE_FILE"
    exit 1
fi

# ubxfwupdate.exe supports Linux device names directly (e.g., /dev/ttyACM0, /dev/ttyS5)
# No need to convert to COM port format
# According to ubxfwupdate help: "/dev/ttySy - serial (RS232) port y (Linux)"

echo ""
echo "Configuration:"
echo "  Port: $PORT"
echo "  Firmware: $FIRMWARE_FILE"
echo "  Tool: $UBXFWUPDATE"
echo ""
echo "WARNING: Firmware flashing can brick the device if interrupted!"
echo "Press Ctrl+C to cancel, or wait 5 seconds to continue..."
sleep 5

echo ""
echo "Running ubxfwupdate via Wine..."
echo "Command: wine $UBXFWUPDATE -p $PORT -f $FIRMWARE_FILE -b 9600:9600:115200"
echo ""

# Run ubxfwupdate via Wine
# -p: port (Linux device name works directly)
# -f: firmware file
# -b: baud rates (current:safeboot:update) - 9600:9600:115200 is default but explicit is better
wine "$UBXFWUPDATE" -p "$PORT" -f "$FIRMWARE_FILE" -b 9600:9600:115200

echo ""
echo "============================================================"
if [ $? -eq 0 ]; then
    echo "✓ Firmware flash completed successfully"
    echo "  GPS should reboot with new firmware"
    echo "  Wait 10 seconds, then test with: python3 nodes/gps.py --debug"
else
    echo "✗ Firmware flash failed"
    echo "  Check error messages above"
    echo "  GPS may still be in bootloader mode"
fi
echo "============================================================"
