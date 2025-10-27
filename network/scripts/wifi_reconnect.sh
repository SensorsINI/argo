#!/bin/bash

# Argo WiFi Reconnection Script
# Forces connection to preferred networks when they become available
# Priority: tobi-s24 (15) > tobi-wlan (10) > uzh-iot (5)
# 
# SAFETY FEATURES:
# - Only runs when called by systemd timer (every 2 minutes)
# - Prevents connection changes if already on preferred network
# - Includes connection stability checks
# - Logs all actions for debugging

# Configuration
PREFERRED_NETWORKS=("tobi-s24" "tobi-wlan")
LOG_FILE="/var/log.hdd/persistent/wifi-reconnect-$(date +%Y%m%d).log"
MAX_LOG_SIZE=1048576  # 1MB
LOCK_FILE="/tmp/wifi_reconnect.lock"
MIN_CONNECTION_TIME=30  # Minimum seconds to stay on a connection before switching

# Function to ensure log file has correct permissions
ensure_log_permissions() {
    # Create log directory if it doesn't exist
    mkdir -p "$(dirname "$LOG_FILE")"
    
    # Create log file if it doesn't exist with correct permissions
    if [ ! -f "$LOG_FILE" ]; then
        touch "$LOG_FILE"
        chmod 644 "$LOG_FILE"
        # Service runs as root, log file is owned by root
        # This is fine - the service needs root to handle log rotation
    else
        # Ensure file is readable by others (644 = rw-r--r--)
        local current_perms=$(stat -c '%a' "$LOG_FILE" 2>/dev/null || echo "644")
        if [ "$current_perms" != "644" ]; then
            chmod 644 "$LOG_FILE" 2>/dev/null || true
        fi
    fi
}

# Function to log messages with timestamp
log_message() {
    local message="$(date '+%Y-%m-%d %H:%M:%S') - $1"
    
    # Ensure log file has correct permissions before writing
    ensure_log_permissions
    
    # Append to log file (script runs as root, so permissions are handled)
    echo "$message" | tee -a "$LOG_FILE" >/dev/null 2>&1
    echo "$message"
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

# Function to check if connection is stable (not recently changed)
is_connection_stable() {
    local connection_name="$1"
    local timestamp_file="/tmp/wifi_connection_${connection_name}_timestamp"
    
    if [ -f "$timestamp_file" ]; then
        local last_change=$(cat "$timestamp_file")
        local current_time=$(date +%s)
        local time_diff=$((current_time - last_change))
        
        if [ $time_diff -lt $MIN_CONNECTION_TIME ]; then
            log_message "Connection $connection_name is not stable yet (${time_diff}s < ${MIN_CONNECTION_TIME}s), skipping switch"
            return 1
        fi
    fi
    
    return 0
}

# Function to record connection change timestamp
record_connection_change() {
    local connection_name="$1"
    local timestamp_file="/tmp/wifi_connection_${connection_name}_timestamp"
    echo "$(date +%s)" > "$timestamp_file"
}

# Main logic
main() {
    # Check for lock file to prevent multiple instances
    if [ -f "$LOCK_FILE" ]; then
        log_message "Another instance is running, exiting"
        exit 1
    fi
    
    # Create lock file
    echo $$ > "$LOCK_FILE"
    
    # Ensure lock file is removed on exit
    trap 'rm -f "$LOCK_FILE"' EXIT
    
    rotate_log
    log_message "WiFi reconnection check started"
    
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
    
    # Check if current connection is stable before switching
    if ! is_connection_stable "$CURRENT_CONNECTION"; then
        return 0
    fi
    
    for preferred in "${PREFERRED_NETWORKS[@]}"; do
        if is_network_available "$preferred"; then
            log_message "Preferred network $preferred is available, attempting connection"
            if connect_to_network "$preferred"; then
                record_connection_change "$preferred"
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
