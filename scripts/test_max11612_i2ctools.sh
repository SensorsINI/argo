#!/bin/bash
# Diagnostic script for MAX11612 ADC using i2c-tools
# Tests I2C communication, setup, and reading

I2C_BUS=0
ADC_ADDR=0x34

echo "============================================================"
echo "MAX11612 ADC Diagnostic Tool (using i2c-tools)"
echo "I2C Bus: $I2C_BUS, Address: $ADC_ADDR"
echo "============================================================"

# Test 1: Device detection
echo ""
echo "Test 1: Device Detection"
echo "------------------------------------------------------------"
echo "Reading from device (any register)..."
RESULT=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 0x00 2>&1)
if [ $? -eq 0 ]; then
    echo "✅ Device responded: $RESULT"
else
    echo "❌ Device did not respond: $RESULT"
    exit 1
fi

# Test 2: Setup register write
echo ""
echo "Test 2: Setup Register Write"
echo "------------------------------------------------------------"
# Setup byte: reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0
# Binary: 1 101 0 0 1 0 = 0xD2
SETUP_BYTE=0xD2
echo "Setup byte: 0x$(printf '%02x' $SETUP_BYTE) (reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)"
echo "Writing setup byte..."
sudo i2cset -y $I2C_BUS $ADC_ADDR $SETUP_BYTE
if [ $? -eq 0 ]; then
    echo "✅ Setup register write successful"
    sleep 0.01
else
    echo "❌ Setup register write failed"
    exit 1
fi

# Test 3: Configuration and read
echo ""
echo "Test 3: Configuration and Read (Channel 0)"
echo "------------------------------------------------------------"
# Config byte: reg=0, scan=0b01, cs=0, sgl_dif=1
# Binary: 0 01 0000 1 = 0x21
CONFIG_BYTE=0x21
echo "Config byte: 0x$(printf '%02x' $CONFIG_BYTE) (reg=0, scan=0b01, cs=0, sgl_dif=1)"
echo "Writing config byte..."
sudo i2cset -y $I2C_BUS $ADC_ADDR $CONFIG_BYTE
sleep 0.001

echo "Reading 2 bytes (ADC result)..."
# Read first byte
BYTE1=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 2>&1 | head -1)
if [ $? -ne 0 ]; then
    echo "❌ Failed to read first byte: $BYTE1"
    exit 1
fi

# Read second byte
BYTE2=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 2>&1 | head -1)
if [ $? -ne 0 ]; then
    echo "❌ Failed to read second byte: $BYTE2"
    exit 1
fi

# Convert hex to decimal
BYTE1_DEC=$((BYTE1))
BYTE2_DEC=$((BYTE2))

# Decode ADC result: ((byte1 & 0x0F) << 8) | byte2
CODE=$((( (BYTE1_DEC & 0x0F) << 8 ) | BYTE2_DEC))
echo "✅ Read successful:"
echo "   Byte 1: $BYTE1 (decimal: $BYTE1_DEC)"
echo "   Byte 2: $BYTE2 (decimal: $BYTE2_DEC)"
echo "   Raw ADC code: $CODE (0x$(printf '%04x' $CODE))"

# Test 4: Multiple channels
echo ""
echo "Test 4: Multiple Channel Reads"
echo "------------------------------------------------------------"
for CH in 0 1 2 3; do
    # Config byte: reg=0, scan=0b01, cs=CH, sgl_dif=1
    CONFIG_BYTE=$((0x20 | (CH << 1) | 1))
    echo ""
    echo "Channel $CH:"
    echo "  Config byte: 0x$(printf '%02x' $CONFIG_BYTE)"
    sudo i2cset -y $I2C_BUS $ADC_ADDR $CONFIG_BYTE
    sleep 0.001
    
    BYTE1=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 2>&1 | head -1)
    BYTE2=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 2>&1 | head -1)
    
    if [ $? -eq 0 ] && [[ "$BYTE1" =~ ^0x ]] && [[ "$BYTE2" =~ ^0x ]]; then
        BYTE1_DEC=$((BYTE1))
        BYTE2_DEC=$((BYTE2))
        CODE=$((( (BYTE1_DEC & 0x0F) << 8 ) | BYTE2_DEC))
        echo "  ✅ Channel $CH: $CODE (0x$(printf '%04x' $CODE))"
    else
        echo "  ❌ Channel $CH: Failed to read"
    fi
done

# Test 5: Scan mode (multiple reads)
echo ""
echo "Test 5: Scan Mode (Multiple Conversions)"
echo "------------------------------------------------------------"
CONFIG_BYTE=0x21  # Channel 0, scan mode
echo "Config byte: 0x$(printf '%02x' $CONFIG_BYTE) (scan mode)"
sudo i2cset -y $I2C_BUS $ADC_ADDR $CONFIG_BYTE
sleep 0.001

echo "Reading 8 conversions..."
CODES=()
for i in {1..8}; do
    BYTE1=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 2>&1 | head -1)
    BYTE2=$(sudo i2cget -y $I2C_BUS $ADC_ADDR 2>&1 | head -1)
    
    if [ $? -eq 0 ] && [[ "$BYTE1" =~ ^0x ]] && [[ "$BYTE2" =~ ^0x ]]; then
        BYTE1_DEC=$((BYTE1))
        BYTE2_DEC=$((BYTE2))
        CODE=$((( (BYTE1_DEC & 0x0F) << 8 ) | BYTE2_DEC))
        CODES+=($CODE)
        echo "  Conversion $i: $CODE (0x$(printf '%04x' $CODE))"
    else
        echo "  Conversion $i: Failed"
    fi
done

if [ ${#CODES[@]} -gt 0 ]; then
    SUM=0
    for code in "${CODES[@]}"; do
        SUM=$((SUM + code))
    done
    AVG=$((SUM / ${#CODES[@]}))
    MIN=${CODES[0]}
    MAX=${CODES[0]}
    for code in "${CODES[@]}"; do
        [ $code -lt $MIN ] && MIN=$code
        [ $code -gt $MAX ] && MAX=$code
    done
    echo ""
    echo "✅ Scan mode successful:"
    echo "   Average: $AVG (0x$(printf '%04x' $AVG))"
    echo "   Range: $MIN - $MAX"
else
    echo "❌ Scan mode failed: No successful reads"
fi

echo ""
echo "============================================================"
echo "Diagnostic complete"
echo "============================================================"
