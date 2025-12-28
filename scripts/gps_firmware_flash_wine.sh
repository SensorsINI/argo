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
#   ./scripts/gps_firmware_flash_wine.sh <firmware_file> [port] [ubxfwupdate.exe] [flash.xml]
#
# Examples:
#   ./scripts/gps_firmware_flash_wine.sh firmware.bin /dev/ttyACM0
#   ./scripts/gps_firmware_flash_wine.sh firmware.bin /dev/ttyACM0 ./ubxfwupdate.exe flash.xml
#

set -e

FIRMWARE_FILE="${1:-}"
PORT="${2:-/dev/ttyS5}"
UBXFWUPDATE="${3:-./ubxfwupdate.exe}"
FLASH_XML="${4:-}"

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
    echo "Usage: $0 <firmware_file> [port] [ubxfwupdate.exe] [flash.xml]"
    echo ""
    echo "Examples:"
    echo "  $0 firmware.bin /dev/ttyACM0"
    echo "  $0 firmware.bin /dev/ttyACM0 ./ubxfwupdate.exe flash.xml"
    exit 1
fi

# Check if firmware file exists
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "ERROR: Firmware file not found: $FIRMWARE_FILE"
    exit 1
fi

# Look for flash.xml file (Flash Information Structure)
# It's needed for firmware updates unless --no-fis is used
# flash.xml is typically included in the u-center installer
if [ -z "$FLASH_XML" ]; then
    # Check common locations
    if [ -f "flash.xml" ]; then
        FLASH_XML="flash.xml"
    elif [ -f "firmware/flash.xml" ]; then
        FLASH_XML="firmware/flash.xml"
    elif [ -f "./flash.xml" ]; then
        FLASH_XML="./flash.xml"
    # Check if ubxfwupdate.exe is in same directory (might have flash.xml there)
    elif [ -f "$(dirname "$UBXFWUPDATE")/flash.xml" ]; then
        FLASH_XML="$(dirname "$UBXFWUPDATE")/flash.xml"
    # Check Wine u-center installation directory
    elif [ -f "$HOME/.wine/drive_c/Program Files/u-blox/u-center/flash.xml" ]; then
        FLASH_XML="$HOME/.wine/drive_c/Program Files/u-blox/u-center/flash.xml"
    else
        echo "WARNING: flash.xml not found"
        echo "  ubxfwupdate needs flash.xml for proper firmware updates"
        echo "  flash.xml should be in the u-center installer (extract it)"
        echo ""
        echo "Options:"
        echo "  1. Extract flash.xml from u-center installer and place in current directory"
        echo "  2. Specify path: $0 $FIRMWARE_FILE $PORT $UBXFWUPDATE <path/to/flash.xml>"
        echo "  3. Continue with --no-fis (may work but not recommended)"
        echo ""
        read -p "Continue with --no-fis? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
        FLASH_XML=""  # Will use --no-fis
    fi
fi

# Check if flash.xml exists if specified
if [ -n "$FLASH_XML" ] && [ ! -f "$FLASH_XML" ]; then
    echo "WARNING: Specified flash.xml not found: $FLASH_XML"
    echo "  Continuing without explicit FIS file (will use default or fail)"
    FLASH_XML=""
fi

# ubxfwupdate.exe supports Linux device names directly (e.g., /dev/ttyACM0, /dev/ttyS5)
# No need to convert to COM port format
# According to ubxfwupdate help: "/dev/ttySy - serial (RS232) port y (Linux)"

echo ""
echo "Configuration:"
echo "  Port: $PORT"
echo "  Firmware: $FIRMWARE_FILE"
echo "  Flash XML: ${FLASH_XML:-<default or --no-fis>}"
echo "  Tool: $UBXFWUPDATE"
echo ""
echo "WARNING: Firmware flashing can brick the device if interrupted!"
echo "Press Ctrl+C to cancel, or wait 5 seconds to continue..."
sleep 5

echo ""
echo "Running ubxfwupdate via Wine..."

# Build command with optional flash.xml
if [ -n "$FLASH_XML" ]; then
    echo "Command: wine $UBXFWUPDATE -p $PORT -b 9600:9600:115200 -s 1 -v 1 -F $FLASH_XML $FIRMWARE_FILE"
    echo ""
    # -F: specify flash.xml file
    wine "$UBXFWUPDATE" -p "$PORT" -b 9600:9600:115200 -s 1 -v 1 -F "$FLASH_XML" "$FIRMWARE_FILE"
else
    echo "Command: wine $UBXFWUPDATE -p $PORT -b 9600:9600:115200 -s 1 -v 1 --no-fis 1 $FIRMWARE_FILE"
    echo ""
    echo "NOTE: Using --no-fis 1 (no Flash Information Structure)"
    echo "  This may work, but FIS merging is recommended for proper updates"
    # --no-fis 1: don't merge FIS (use if flash.xml not available)
    wine "$UBXFWUPDATE" -p "$PORT" -b 9600:9600:115200 -s 1 -v 1 --no-fis 1 "$FIRMWARE_FILE"
fi

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
