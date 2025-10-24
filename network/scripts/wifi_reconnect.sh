#!/bin/bash

# Argo WiFi Reconnection Script
# Forces connection to preferred networks when they become available
# Priority: tobi-s24 (15) > tobi-wlan (10) > uzh-iot (5)

# Configuration
PREFERRED_NETWORKS=("tobi-s24" "tobi-wlan")
LOG_FILE="/var/log.hdd/persistent/wifi-reconnect.log"
MAX_LOG_SIZE=1048576  # 1MB

# Function to log messages with timestamp
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Function to rotate log if it gets too large
rotate_log() {
    if [ -f "$LOG_FILE" ] && [ $(stat -f%z "$LOG_FILE" 2>/dev/null || stat -c%s "$LOG_FILE" 2>/dev/null) -gt $MAX_LOG_SIZE ]; then
        mv "$LOG_FILE" "${LOG_FILE}.old"
        log_message "Log rotated due to size limit"
    fi
}

# Function to get current connection name
get_current_connection() {
    nmcli -t -f NAME connection show --active | grep -v "ztw4lntpwi" | head -1
}

# Function to check if a network is available
is_network_available() {
    local network_name="$1"
    nmcli dev wifi list | grep -q "$network_name"
}

# Function to connect to a network
connect_to_network() {
    local network_name="$1"
    log_message "Attempting to connect to preferred network: $network_name"
    
    if nmcli connection up "$network_name" 2>/dev/null; then
        log_message "Successfully connected to $network_name"
        return 0
    else
        log_message "Failed to connect to $network_name"
        return 1
    fi
}

# Main logic
main() {
    rotate_log
    
    # Get current connection
    CURRENT_CONNECTION=$(get_current_connection)
    
    if [ -z "$CURRENT_CONNECTION" ]; then
        log_message "No active WiFi connection found"
        return 1
    fi
    
    log_message "Current connection: $CURRENT_CONNECTION"
    
    # Check if current connection is already a preferred network
    for preferred in "${PREFERRED_NETWORKS[@]}"; do
        if [ "$CURRENT_CONNECTION" = "$preferred" ]; then
            log_message "Already connected to preferred network: $preferred"
            return 0
        fi
    done
    
    # Current connection is not preferred, check for preferred networks
    log_message "Current connection ($CURRENT_CONNECTION) is not preferred, checking for better options"
    
    for preferred in "${PREFERRED_NETWORKS[@]}"; do
        if is_network_available "$preferred"; then
            log_message "Preferred network $preferred is available, attempting connection"
            if connect_to_network "$preferred"; then
                return 0
            fi
        else
            log_message "Preferred network $preferred is not available"
        fi
    done
    
    log_message "No preferred networks available, staying on current connection: $CURRENT_CONNECTION"
}

# Run main function
main "$@"
