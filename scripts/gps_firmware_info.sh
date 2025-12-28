#!/bin/bash
# GPS Firmware Information and Download Helper
#
# This script provides information about where to get NEO-M9N firmware
# and checks if firmware files are already available locally

echo "============================================================"
echo "NEO-M9N Firmware Information"
echo "============================================================"

echo ""
echo "Firmware Download Locations:"
echo "  1. Official u-blox website:"
echo "     https://www.u-blox.com/en/product/neo-m9n-module"
echo "     → Click 'Documentation & resources' tab"
echo "     → Look for 'Firmware Update' section"
echo ""
echo "  2. Direct firmware files are typically named:"
echo "     - NEO-M9N-XX.XX.hex (where XX.XX is version number)"
echo "     - NEO-M9N-firmware.ubx"
echo ""
echo "  3. Check u-center installation directory (if installed):"
echo "     - Windows: C:\\Program Files\\u-blox\\u-center\\"
echo "     - Look for firmware files or firmware download cache"
echo ""

# Check for local firmware files
echo "Checking for local firmware files..."
FOUND=0

# Check current directory
shopt -s nullglob
for file in *.hex *.ubx NEO-M9N*.hex NEO-M9N*.ubx; do
    if [ -f "$file" ]; then
        echo "  ✓ Found: $file ($(du -h "$file" | cut -f1))"
        FOUND=1
    fi
done
shopt -u nullglob

# Check common locations
if [ -d "$HOME/.wine/drive_c/Program Files/u-blox" ]; then
    echo ""
    echo "Checking Wine u-center installation..."
    find "$HOME/.wine/drive_c/Program Files/u-blox" -name "*.hex" -o -name "*.ubx" 2>/dev/null | while read file; do
        if [ -f "$file" ]; then
            echo "  ✓ Found in Wine: $file"
            FOUND=1
        fi
    done
fi

if [ $FOUND -eq 0 ]; then
    echo "  ✗ No firmware files found locally"
    echo ""
    echo "You need to download the firmware file from u-blox website"
fi

echo ""
echo "============================================================"
echo "Firmware File Requirements:"
echo "============================================================"
echo ""
echo "For NEO-M9N, you need firmware file compatible with:"
echo "  - Module: NEO-M9N"
echo "  - Protocol version: 35.16 (M9-MDR-2.16)"
echo ""
echo "File format: Usually .hex or .ubx"
echo ""
echo "Once you have the firmware file, you can flash it using:"
echo "  ./scripts/gps_firmware_flash_wine.sh <firmware_file> /dev/ttyS5"
echo ""
