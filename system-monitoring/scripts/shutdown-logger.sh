#!/bin/bash
# Shutdown logger - captures shutdown events and service stop times
# Runs during system shutdown to log which services take time to stop

PERSIST_DIR="/var/log.hdd/persistent"
SHUTDOWN_LOG="$PERSIST_DIR/shutdown-$(date +%Y%m%d-%H%M%S).log"

# Create persistent directory if it doesn't exist
mkdir -p "$PERSIST_DIR"

# Log function
log_msg() {
    local msg="$1"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$timestamp] $msg" >> "$SHUTDOWN_LOG" 2>/dev/null || true
    # Also log to console/kmsg for visibility
    echo "[$timestamp] SHUTDOWN-LOGGER: $msg" > /dev/console 2>/dev/null || true
    echo "[$timestamp] SHUTDOWN-LOGGER: $msg" > /dev/kmsg 2>/dev/null || true
}

# Detect shutdown reason
log_msg "=== SHUTDOWN LOGGER STARTED ==="
log_msg "Shutdown initiated at $(date '+%Y-%m-%d %H:%M:%S')"

# Check if systemd is still available to query
if command -v systemctl >/dev/null 2>&1; then
    # Get systemd target
    if [ -f /etc/systemd/system/default.target ]; then
        DEFAULT_TARGET=$(readlink -f /etc/systemd/system/default.target | xargs basename)
        log_msg "Default target: $DEFAULT_TARGET"
    fi
    
    # Check for active jobs (services being stopped)
    log_msg "=== ACTIVE SYSTEMD JOBS ==="
    systemctl list-jobs --no-pager >> "$SHUTDOWN_LOG" 2>/dev/null || true
    
    # List Argo services and their stop timeout configuration
    log_msg "=== ARGO SERVICES STOP TIMEOUTS ==="
    for service in argo_launch_standard argo_power_control argo_battery_water argo_bno085 \
                   argo_health_monitor argo_thermal_monitor argo_storage_monitor \
                   argo_radio_servo_module argo_wifi_reconnect; do
        if systemctl list-unit-files "$service.service" >/dev/null 2>&1; then
            TIMEOUT=$(systemctl show -p TimeoutStopUSec --value "$service.service" 2>/dev/null || echo "infinity")
            STATE=$(systemctl is-active "$service.service" 2>/dev/null || echo "inactive")
            # Convert microseconds to seconds for readability
            if [ "$TIMEOUT" != "infinity" ] && [ -n "$TIMEOUT" ]; then
                TIMEOUT_SEC=$((TIMEOUT / 1000000))
                log_msg "  $service.service: timeout=${TIMEOUT_SEC}s, state=$STATE"
            else
                log_msg "  $service.service: timeout=$TIMEOUT, state=$STATE"
            fi
        fi
    done
    
    # List all active services with stop timeouts > 10s
    log_msg "=== SERVICES WITH LONG STOP TIMEOUTS (>10s) ==="
    systemctl list-units --type=service --state=running --no-pager 2>/dev/null | \
        grep "\.service" | \
        awk '{print $1}' | \
        while read -r unit; do
            TIMEOUT=$(systemctl show -p TimeoutStopUSec --value "$unit" 2>/dev/null || echo "infinity")
            if [ "$TIMEOUT" != "infinity" ] && [ -n "$TIMEOUT" ]; then
                # Convert microseconds to seconds
                TIMEOUT_SEC=$((TIMEOUT / 1000000))
                if [ "$TIMEOUT_SEC" -gt 10 ]; then
                    STATE=$(systemctl is-active "$unit" 2>/dev/null || echo "unknown")
                    log_msg "  $unit: timeout=${TIMEOUT_SEC}s, state=$STATE"
                fi
            fi
        done
    
    # Get systemd shutdown target info
    log_msg "=== SYSTEMD SHUTDOWN TARGET INFO ==="
    if systemctl list-jobs | grep -q "shutdown.target"; then
        log_msg "shutdown.target job found"
        systemctl list-jobs shutdown.target --no-pager >> "$SHUTDOWN_LOG" 2>/dev/null || true
    fi
else
    log_msg "WARNING: systemctl not available (already in late shutdown phase)"
fi

# Check if critical battery flag exists
if [ -f /tmp/argo_critical_battery ]; then
    log_msg "=== CRITICAL BATTERY FLAG DETECTED ==="
    log_msg "System shutting down due to critical battery - power relay will remain energized"
fi

# Log filesystem sync status
log_msg "=== FILESYSTEM SYNC ==="
if command -v sync >/dev/null 2>&1; then
    log_msg "Syncing filesystems..."
    sync >> "$SHUTDOWN_LOG" 2>&1 || true
    log_msg "Filesystem sync completed"
else
    log_msg "WARNING: sync command not available"
fi

# Final timestamp
log_msg "=== SHUTDOWN LOGGER ENDING ==="
log_msg "Shutdown logger completed at $(date '+%Y-%m-%d %H:%M:%S')"

exit 0




