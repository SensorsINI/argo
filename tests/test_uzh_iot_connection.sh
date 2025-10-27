#!/bin/bash

# Safe network test script with timeouts
# Tests uzh-iot connection and restores tobi-wlan

NETWORK_LOG="/tmp/uzh_iot_test_$(date +%Y%m%d_%H%M%S).log"
TIMEOUT=60  # 30 second timeout for network operations

echo "=== UZH-IoT Network Test - $(date) ===" > $NETWORK_LOG
echo "Log file: $NETWORK_LOG"

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

log_with_time "Starting network test..."

# Record initial state
log_with_time "Initial network state:"
nmcli device status >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

log_with_time "Initial connections:"
nmcli connection show >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

log_with_time "Initial IP addresses:"
ip addr show >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

# Test uzh-iot connection with timeout
log_with_time "Attempting to connect to uzh-iot (timeout: ${TIMEOUT}s)..."
if run_with_timeout $TIMEOUT nmcli connection up uzh-iot; then
    log_with_time "Successfully connected to uzh-iot"
    
    # Test connectivity with timeout
    log_with_time "Testing internet connectivity (timeout: 10s)..."
    if run_with_timeout 10 ping -c 3 8.8.8.8; then
        log_with_time "Internet connectivity: OK"
    else
        log_with_time "Internet connectivity: FAILED"
    fi
    
    # Test DNS with timeout
    log_with_time "Testing DNS resolution (timeout: 10s)..."
    if run_with_timeout 10 nslookup google.com; then
        log_with_time "DNS resolution: OK"
    else
        log_with_time "DNS resolution: FAILED"
    fi
    
    # Record uzh-iot state
    log_with_time "Network state on uzh-iot:"
    nmcli device status >> $NETWORK_LOG
    echo "" >> $NETWORK_LOG
    
    log_with_time "IP addresses on uzh-iot:"
    ip addr show >> $NETWORK_LOG
    echo "" >> $NETWORK_LOG
    
    log_with_time "Routing table on uzh-iot:"
    ip route show >> $NETWORK_LOG
    echo "" >> $NETWORK_LOG
    
else
    log_with_time "Failed to connect to uzh-iot"
fi

# Restore tobi-wlan connection
log_with_time "Restoring connection to tobi-wlan (timeout: ${TIMEOUT}s)..."
if run_with_timeout $TIMEOUT nmcli connection up tobi-wlan; then
    log_with_time "Successfully restored tobi-wlan connection"
else
    log_with_time "Failed to restore tobi-wlan connection"
fi

# Final state check
log_with_time "Final network state:"
nmcli device status >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

log_with_time "Final IP addresses:"
ip addr show >> $NETWORK_LOG
echo "" >> $NETWORK_LOG

log_with_time "Network test completed. Log saved to: $NETWORK_LOG"

# Display summary
echo ""
echo "=== Test Summary ==="
echo "Log file: $NETWORK_LOG"
echo "Current connection: $(nmcli -t -f NAME connection show --active)"
echo "Current IP: $(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+' || echo 'unknown')"
