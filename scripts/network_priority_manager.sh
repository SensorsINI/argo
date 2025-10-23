#!/bin/bash

# Network Priority Manager for Argo
# Manages WiFi connection priorities and automatic switching

NETWORK_LOG="/tmp/network_priority_$(date +%Y%m%d_%H%M%S).log"

# Function to log with timestamp
log_with_time() {
    echo "[$(date '+%H:%M:%S')] $1" | tee -a $NETWORK_LOG
}

# Function to run command with timeout
run_with_timeout() {
    local timeout=$1
    shift
    timeout $timeout "$@" 2>&1 | tee -a $NETWORK_LOG
    return ${PIPESTATUS[0]}
}

log_with_time "=== Network Priority Manager - $(date) ==="

# Set network priorities
log_with_time "Setting network priorities..."
nmcli connection modify tobi-wlan connection.autoconnect-priority 10
nmcli connection modify uzh-iot connection.autoconnect-priority 5

# Ensure both networks are set to auto-connect
nmcli connection modify tobi-wlan connection.autoconnect yes
nmcli connection modify uzh-iot connection.autoconnect yes

log_with_time "Network priorities set:"
log_with_time "  tobi-wlan: priority 10 (highest)"
log_with_time "  uzh-iot: priority 5 (lower)"

# Check current status
log_with_time "Current network status:"
nmcli device status >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

log_with_time "Current connections:"
nmcli connection show >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

# Function to force connection to highest priority network
force_priority_connection() {
    log_with_time "Forcing connection to highest priority network..."
    
    # Disconnect from current network
    current_connection=$(nmcli -t -f NAME connection show --active)
    if [ -n "$current_connection" ]; then
        log_with_time "Disconnecting from: $current_connection"
        nmcli connection down "$current_connection"
        sleep 2
    fi
    
    # Connect to highest priority network
    log_with_time "Connecting to tobi-wlan (highest priority)..."
    if run_with_timeout 30 nmcli connection up tobi-wlan; then
        log_with_time "Successfully connected to tobi-wlan"
        return 0
    else
        log_with_time "Failed to connect to tobi-wlan, trying uzh-iot..."
        if run_with_timeout 30 nmcli connection up uzh-iot; then
            log_with_time "Successfully connected to uzh-iot"
            return 0
        else
            log_with_time "Failed to connect to any network"
            return 1
        fi
    fi
}

# Function to check and restore network connectivity
check_and_restore() {
    log_with_time "Checking network connectivity..."
    
    # Check if we have a valid connection
    if ! nmcli device status | grep -q "wlan0.*connected"; then
        log_with_time "No WiFi connection detected, attempting to restore..."
        force_priority_connection
    else
        current_connection=$(nmcli -t -f NAME connection show --active)
        log_with_time "Current connection: $current_connection"
        
        # If connected to uzh-iot, try to switch to tobi-wlan if available
        if [ "$current_connection" = "uzh-iot" ]; then
            log_with_time "Currently on uzh-iot, checking if tobi-wlan is available..."
            if nmcli connection show tobi-wlan | grep -q "connection.available"; then
                log_with_time "tobi-wlan is available, switching to higher priority network..."
                force_priority_connection
            else
                log_with_time "tobi-wlan not available, staying on uzh-iot"
            fi
        fi
    fi
}

# Main execution
case "${1:-check}" in
    "force-priority")
        force_priority_connection
        ;;
    "check")
        check_and_restore
        ;;
    "status")
        log_with_time "Network status:"
        nmcli device status
        echo ""
        log_with_time "Active connection:"
        nmcli -t -f NAME connection show --active
        ;;
    "help")
        echo "Usage: $0 [command]"
        echo "Commands:"
        echo "  check          - Check and restore network connectivity (default)"
        echo "  force-priority - Force connection to highest priority network"
        echo "  status         - Show current network status"
        echo "  help           - Show this help message"
        ;;
    *)
        echo "Unknown command: $1"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac

log_with_time "Network priority management completed. Log: $NETWORK_LOG"
