#!/bin/bash
# Argo WiFi Network Setup Script
# Configures WiFi networks with proper priority order for Argo sailboat

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Network configuration
declare -A NETWORKS=(
    ["tobi-matebook"]="20:tdtgtdtg"
    ["tobi-s24"]="15:tdtgtdtg"
    ["tobi-wlan"]="5:"
    ["uzh-iot"]="0:e85e2d541bffeb9d6415adc6"
)

echo -e "${BLUE}🚢 Argo WiFi Network Setup${NC}"
echo "=================================="
echo ""

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    echo -e "${RED}❌ This script should not be run as root${NC}"
    echo "   Run as regular user (sudo will be used when needed)"
    exit 1
fi

# Check if NetworkManager is available
if ! command -v nmcli &> /dev/null; then
    echo -e "${RED}❌ NetworkManager (nmcli) not found${NC}"
    echo "   Install with: sudo apt install network-manager"
    exit 1
fi

echo -e "${YELLOW}🔍 Current WiFi connections:${NC}"
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show | grep -E "(NAME|wifi)" || echo "   No WiFi connections found"
echo ""

# Function to add or update WiFi connection
setup_wifi_connection() {
    local ssid="$1"
    local priority="$2"
    local password="$3"
    
    echo -e "${BLUE}📡 Setting up: ${ssid} (Priority: ${priority})${NC}"
    
    # Check if connection already exists
    if nmcli connection show "$ssid" &> /dev/null; then
        echo "   Connection exists, updating..."
        
        # Update autoconnect and priority
        nmcli connection modify "$ssid" connection.autoconnect yes
        nmcli connection modify "$ssid" connection.autoconnect-priority "$priority"
        
        # Update password if provided
        if [[ -n "$password" ]]; then
            nmcli connection modify "$ssid" wifi-sec.key-mgmt wpa-psk
            nmcli connection modify "$ssid" wifi-sec.psk "$password"
        fi
        
        echo -e "   ${GREEN}✅ Updated ${ssid}${NC}"
    else
        echo "   Creating new connection..."
        
        # Create new connection
        if [[ -n "$password" ]]; then
            nmcli connection add type wifi con-name "$ssid" ifname wlan0 ssid "$ssid" \
                wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$password" \
                connection.autoconnect yes connection.autoconnect-priority "$priority"
        else
            nmcli connection add type wifi con-name "$ssid" ifname wlan0 ssid "$ssid" \
                connection.autoconnect yes connection.autoconnect-priority "$priority"
        fi
        
        echo -e "   ${GREEN}✅ Created ${ssid}${NC}"
    fi
}

# Remove any existing wired ethernet connections (disabled via overlay)
echo -e "${YELLOW}🔧 Cleaning up wired connections...${NC}"
if nmcli connection show "Wired connection 1" &> /dev/null; then
    nmcli connection delete "Wired connection 1"
    echo -e "   ${GREEN}✅ Removed wired ethernet connection${NC}"
else
    echo "   No wired connections to remove"
fi
echo ""

# Set up each WiFi network
echo -e "${YELLOW}📡 Configuring WiFi networks...${NC}"
for ssid in "${!NETWORKS[@]}"; do
    IFS=':' read -r priority password <<< "${NETWORKS[$ssid]}"
    setup_wifi_connection "$ssid" "$priority" "$password"
done

echo ""
echo -e "${YELLOW}🔍 Final WiFi configuration:${NC}"
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show | sort -k3 -nr

echo ""
echo -e "${GREEN}✅ WiFi network setup complete!${NC}"
echo ""
echo -e "${BLUE}📋 Network Priority Order:${NC}"
echo "   1. tobi-matebook (Priority: 20) - Windows laptop hotspot"
echo "   2. tobi-s24 (Priority: 15) - Android phone hotspot"
echo "   3. tobi-wlan (Priority: 5) - Original router"
echo "   4. uzh-iot (Priority: 0) - Fallback network"
echo ""
echo -e "${BLUE}💡 Usage Notes:${NC}"
echo "   • Argo will automatically connect to the highest priority available network"
echo "   • SSH sessions will drop when switching networks (use tmux to maintain work)"
echo "   • All networks have autoconnect enabled for seamless switching"
echo ""
echo -e "${YELLOW}🔧 To test network switching:${NC}"
echo "   1. Turn off tobi-wlan router"
echo "   2. Argo should connect to tobi-matebook or tobi-s24"
echo "   3. SSH from laptop/phone using the hotspot IP range"
echo "   4. Use 'tmux attach' to resume previous sessions"
