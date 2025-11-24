#!/bin/bash
# Argo MAC Address Freeze Script
# Freezes WiFi MAC address to a specified value for persistence across hardware changes
#
# Usage:
#   ./freeze_mac_address.sh                    # Freeze to current MAC address
#   ./freeze_mac_address.sh c8:26:e2:6c:58:ba   # Freeze to specific MAC address
#   ./freeze_mac_address.sh --from-logs          # Find and freeze to previous MAC from logs

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default WiFi connections to configure
WIFI_CONNECTIONS=("tobi-wlan" "tobi-matebook" "tobi-s24" "uzh-iot")

# Function to get MAC address from ARGO_MAC_ID.txt (preferred)
get_mac_from_file() {
    local mac_file=""
    
    # Try multiple possible locations
    if [ -f "network/ARGO_MAC_ID.txt" ]; then
        mac_file="network/ARGO_MAC_ID.txt"
    elif [ -f "../network/ARGO_MAC_ID.txt" ]; then
        mac_file="../network/ARGO_MAC_ID.txt"
    elif [ -f "/home/orangepi/argo/network/ARGO_MAC_ID.txt" ]; then
        mac_file="/home/orangepi/argo/network/ARGO_MAC_ID.txt"
    fi
    
    if [ -n "$mac_file" ]; then
        # Extract MAC address from file (last line with MAC format)
        local mac=$(grep -E "^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$" "$mac_file" | tail -1 | tr '[:upper:]' '[:lower:]')
        if [ -n "$mac" ]; then
            echo -e "${GREEN}✅ Found MAC address in $mac_file: $mac${NC}"
            echo "$mac"
            return 0
        fi
    fi
    
    return 1
}

# Function to find previous MAC address from logs
find_previous_mac_from_logs() {
    echo -e "${BLUE}🔍 Searching logs for previous MAC address...${NC}"
    
    # First, try to get MAC from ARGO_MAC_ID.txt (preferred method)
    MAC_FROM_FILE=$(get_mac_from_file)
    if [ -n "$MAC_FROM_FILE" ]; then
        echo "$MAC_FROM_FILE"
        return 0
    fi
    
    # Fallback to searching logs
    CURRENT_MAC=$(ip link show wlan0 2>/dev/null | grep -oP '(?<=link/ether )[^ ]+' || echo "")
    
    # Find all MAC addresses from logs, sorted by frequency (most used first)
    # This helps find the MAC that was used consistently before hardware changes
    MACS_FROM_LOGS=$(grep -r "sprdwl:mac_addr" /var/log.hdd/persistent 2>/dev/null | \
        grep -oE "([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}" | \
        sort | uniq -c | sort -rn | awk '{print $2}' | tr '[:upper:]' '[:lower:]')
    
    if [ -z "$MACS_FROM_LOGS" ]; then
        echo -e "${RED}❌ No MAC addresses found in logs${NC}"
        return 1
    fi
    
    # Find the most frequently used MAC that's not the current one
    PREVIOUS_MAC=""
    for mac in $MACS_FROM_LOGS; do
        if [ "$mac" != "$CURRENT_MAC" ]; then
            PREVIOUS_MAC="$mac"
            echo -e "${GREEN}✅ Found previous MAC from logs: $PREVIOUS_MAC${NC}"
            break
        fi
    done
    
    if [ -z "$PREVIOUS_MAC" ]; then
        echo -e "${YELLOW}⚠️  Could not find previous MAC address (different from current)${NC}"
        return 1
    fi
    
    echo "$PREVIOUS_MAC"
}

# Parse arguments
MAC_ADDRESS=""
if [ "$1" == "--from-logs" ]; then
    MAC_ADDRESS=$(find_previous_mac_from_logs)
    if [ -z "$MAC_ADDRESS" ]; then
        echo -e "${RED}❌ Failed to find previous MAC address from logs${NC}"
        exit 1
    fi
elif [ -n "$1" ]; then
    # Validate MAC address format
    if [[ ! "$1" =~ ^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$ ]]; then
        echo -e "${RED}❌ Invalid MAC address format: $1${NC}"
        echo "   Expected format: XX:XX:XX:XX:XX:XX"
        exit 1
    fi
    MAC_ADDRESS="$1"
else
    # Use current MAC address
    MAC_ADDRESS=$(ip link show wlan0 2>/dev/null | grep -oP '(?<=link/ether )[^ ]+' || echo "")
    if [ -z "$MAC_ADDRESS" ]; then
        echo -e "${RED}❌ Could not detect current MAC address for wlan0${NC}"
        exit 1
    fi
    echo -e "${BLUE}📋 Using current MAC address: $MAC_ADDRESS${NC}"
fi

# Normalize MAC address to uppercase
MAC_ADDRESS=$(echo "$MAC_ADDRESS" | tr '[:lower:]' '[:upper:]')

echo ""
echo -e "${BLUE}🔒 Freezing MAC address to: $MAC_ADDRESS${NC}"
echo ""

# Set cloned MAC for each connection
SUCCESS_COUNT=0
FAIL_COUNT=0

for conn in "${WIFI_CONNECTIONS[@]}"; do
    if nmcli connection show "$conn" &>/dev/null; then
        echo -e "${BLUE}📡 Setting MAC for: $conn${NC}"
        if nmcli connection modify "$conn" 802-11-wireless.cloned-mac-address "$MAC_ADDRESS" 2>/dev/null; then
            echo -e "   ${GREEN}✅ Updated${NC}"
            ((SUCCESS_COUNT++))
        else
            echo -e "   ${RED}❌ Failed${NC}"
            ((FAIL_COUNT++))
        fi
    else
        echo -e "${YELLOW}⚠️  Connection '$conn' not found, skipping${NC}"
    fi
done

echo ""
if [ $FAIL_COUNT -eq 0 ] && [ $SUCCESS_COUNT -gt 0 ]; then
    echo -e "${GREEN}✅ MAC address frozen for $SUCCESS_COUNT WiFi connection(s)${NC}"
    echo ""
    echo -e "${BLUE}💡 To apply changes, reconnect WiFi:${NC}"
    echo "   nmcli connection down <connection-name>"
    echo "   nmcli connection up <connection-name>"
    echo ""
    echo -e "${BLUE}   Or restart NetworkManager:${NC}"
    echo "   sudo systemctl restart NetworkManager"
    echo ""
    echo -e "${BLUE}   Verify MAC address:${NC}"
    echo "   ip link show wlan0 | grep -oP '(?<=link/ether )[^ ]+'"
else
    echo -e "${RED}❌ Failed to update $FAIL_COUNT connection(s)${NC}"
    exit 1
fi
