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

# Convert Linux serial port to Wine COM port format
# Wine typically maps /dev/ttyS* to COM ports
# /dev/ttyS0 -> COM1, /dev/ttyS1 -> COM2, etc.
PORT_NUM=$(echo "$PORT" | sed 's/.*ttyS\([0-9]*\)/\1/')
COM_PORT="COM$((PORT_NUM + 1))"

echo ""
echo "Configuration:"
echo "  Port: $PORT (Wine: $COM_PORT)"
echo "  Firmware: $FIRMWARE_FILE"
echo "  Tool: $UBXFWUPDATE"
echo ""
echo "WARNING: Firmware flashing can brick the device if interrupted!"
echo "Press Ctrl+C to cancel, or wait 5 seconds to continue..."
sleep 5

echo ""
echo "Running ubxfwupdate via Wine..."
echo "Command: wine $UBXFWUPDATE -p $COM_PORT -f $FIRMWARE_FILE"
echo ""

# Run ubxfwupdate via Wine
wine "$UBXFWUPDATE" -p "$COM_PORT" -f "$FIRMWARE_FILE"

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
