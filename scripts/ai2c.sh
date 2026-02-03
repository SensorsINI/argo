#!/bin/bash
# I2C bus scanner for Argo sensors
# Scans I2C bus 0 and reports which sensors are functional and which are missing
# Times out after 1 second if I2C bus is malfunctioning

set -euo pipefail

I2C_BUS=0
TIMEOUT=1

# Expected I2C addresses for Argo devices
# Note: MP2672 (0x4B) excluded - using standalone mode (GPIO only), not I2C
# Mast LED: PCA9632 shows as 0x62 (individual) and 0x70 (All Call); both expected for one device
declare -A EXPECTED_DEVICES=(
    ["0x21"]="Wind sensor 1 (SDP3x)"
    ["0x22"]="Wind sensor 2 (SDP3x)"
    ["0x23"]="Wind sensor 3 (SDP3x)"
    ["0x62"]="Mast LED (PCA9632 individual)"
    ["0x70"]="Mast LED (PCA9632 All Call)"
    ["0x34"]="ADC (MAX11612) - Battery/Water"
    ["0x44"]="Temperature/Humidity (SHT45)"
    ["0x4a"]="IMU (BNO085)"
)

echo "Scanning I2C bus ${I2C_BUS}..."
echo ""

# Run i2cdetect with timeout and measure execution time
START_TIME=$(date +%s.%N)
if ! I2C_OUTPUT=$(timeout ${TIMEOUT}s i2cdetect -y ${I2C_BUS} 2>&1); then
    EXIT_CODE=$?
    END_TIME=$(date +%s.%N)
    ELAPSED=$(echo "$END_TIME - $START_TIME" | bc)
    
    if [ $EXIT_CODE -eq 124 ]; then
        echo "❌ I2C bus ${I2C_BUS} scan TIMED OUT after ${TIMEOUT}s"
        echo "   This indicates I2C bus is malfunctioning or locked"
        echo ""
        echo "⚠️  CRITICAL: No I2C devices detected - battery monitoring unavailable!"
        echo "   Check I2C bus wiring and dtoverlay configuration (pi-i2c0)"
        echo "   Try: sudo $ARGO_DIR/scripts/reset_i2c_bus.sh"
        exit 1
    else
        echo "❌ I2C bus ${I2C_BUS} scan FAILED (exit code: $EXIT_CODE)"
        echo "   Error output: $I2C_OUTPUT"
        exit 1
    fi
fi

END_TIME=$(date +%s.%N)
ELAPSED=$(echo "$END_TIME - $START_TIME" | bc)
ELAPSED_MS=$(echo "$ELAPSED * 1000" | bc | sed 's/\.0*$//')

echo "Scan completed in ${ELAPSED_MS}ms"
echo ""

# Parse i2cdetect output to extract detected addresses
# i2cdetect outputs addresses in format: "34" or "--" for empty
DETECTED_ADDRESSES=()
while IFS= read -r line; do
    # Skip header lines (containing "00:" or just whitespace)
    if [[ "$line" =~ ^[[:space:]]*[0-9a-fA-F]{2}:[[:space:]] ]]; then
        # Extract addresses from this line (skip first column which is row number)
        # Format: "00: -- -- -- -- 34 -- -- -- -- 44 -- -- -- -- --"
        for addr in $line; do
            # Skip row numbers (format "00:")
            if [[ "$addr" =~ ^[0-9a-fA-F]{2}:$ ]]; then
                continue
            fi
            # Skip "--" (empty addresses)
            if [[ "$addr" == "--" ]]; then
                continue
            fi
            # Valid address found (hex digits)
            if [[ "$addr" =~ ^[0-9a-fA-F]{2}$ ]]; then
                # Convert to lowercase and add 0x prefix
                addr_lower=$(echo "$addr" | tr '[:upper:]' '[:lower:]')
                addr_hex="0x${addr_lower}"
                DETECTED_ADDRESSES+=("$addr_hex")
            fi
        done
    fi
done <<< "$I2C_OUTPUT"

# Check if any devices were detected
if [ ${#DETECTED_ADDRESSES[@]} -eq 0 ]; then
    echo "⚠️  CRITICAL: No I2C devices detected on bus ${I2C_BUS}!"
    echo "   This prevents battery voltage monitoring (ADC at 0x34)"
    echo "   Critical battery halt protection is DISABLED without battery monitoring!"
    echo ""
    echo "   Check I2C bus wiring and dtoverlay configuration (pi-i2c0)"
    echo "   Try: sudo $ARGO_DIR/scripts/reset_i2c_bus.sh"
    exit 1
fi

# Report detected addresses
echo "Detected I2C addresses: ${DETECTED_ADDRESSES[*]}"
echo ""

# Check each expected device
FOUND_COUNT=0
MISSING_COUNT=0

for addr in "${!EXPECTED_DEVICES[@]}"; do
    device_name="${EXPECTED_DEVICES[$addr]}"
    
    # Check if address is in detected list (case-insensitive)
    found=false
    for detected in "${DETECTED_ADDRESSES[@]}"; do
        if [[ "${addr,,}" == "${detected,,}" ]]; then
            found=true
            break
        fi
    done
    
    if [ "$found" = true ]; then
        echo "🟢 ${addr}: ${device_name}"
        FOUND_COUNT=$((FOUND_COUNT + 1))
    else
        echo "🔴 ${addr}: ${device_name} - MISSING"
        MISSING_COUNT=$((MISSING_COUNT + 1))
    fi
done

echo ""

# Summary
TOTAL_EXPECTED=${#EXPECTED_DEVICES[@]}
echo "Summary: ${FOUND_COUNT}/${TOTAL_EXPECTED} expected devices found"

if [ $MISSING_COUNT -gt 0 ]; then
    echo "⚠️  ${MISSING_COUNT} device(s) missing - check connections and power"
    exit 1
else
    echo "✅ All expected devices present"
    exit 0
fi
