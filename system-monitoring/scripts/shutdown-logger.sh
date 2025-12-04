#!/bin/bash
# Shutdown timing logger - captures actual service stop times during shutdown
# Runs during shutdown and logs real timing to persistent storage
#
# This script monitors systemd during shutdown and records when each service
# actually stops, including whether it was force-killed.

PERSIST_DIR="/var/log.hdd/persistent"
SHUTDOWN_LOG="$PERSIST_DIR/shutdown-$(date +%Y%m%d-%H%M%S).log"

# Create persistent directory if it doesn't exist
mkdir -p "$PERSIST_DIR"

# Log function with forced flush
log_msg() {
    local msg="$1"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$timestamp] $msg" >> "$SHUTDOWN_LOG" 2>/dev/null
    sync "$SHUTDOWN_LOG" 2>/dev/null || true
}

# Start logging
log_msg "=== SHUTDOWN TIMING LOGGER STARTED ==="
log_msg "Shutdown initiated at $(date '+%Y-%m-%d %H:%M:%S')"

# Record initial state of Argo services
log_msg ""
log_msg "=== INITIAL ARGO SERVICE STATES ==="
for service in argo_launch_standard argo_power_control argo_battery_water argo_bno085 \
               argo_health_monitor argo_thermal_monitor argo_storage_monitor \
               argo_radio_servo_module; do
    if systemctl list-unit-files "$service.service" >/dev/null 2>&1; then
        STATE=$(systemctl is-active "$service.service" 2>/dev/null || echo "inactive")
        TIMEOUT=$(systemctl show -p TimeoutStopUSec --value "$service.service" 2>/dev/null | sed 's/us$//' | awk '{printf "%.1fs", $1/1000000}')
        KILLMODE=$(systemctl show -p KillMode --value "$service.service" 2>/dev/null)
        log_msg "  $service: state=$STATE, timeout=$TIMEOUT, killmode=$KILLMODE"
    fi
done

# Monitor shutdown in real-time by tailing journalctl and watching for stop events
log_msg ""
log_msg "=== MONITORING SHUTDOWN PROGRESS ==="
log_msg "Watching for service stop events..."

# Create a background process to monitor journal
(
    # Use journalctl to follow shutdown events
    journalctl -f --no-pager --since "now" 2>/dev/null | \
    grep --line-buffered -E "Stopping|Stopped|Deactivated|Killing.*signal" | \
    while IFS= read -r line; do
        # Extract timestamp and message
        ts=$(echo "$line" | awk '{print $1, $2, $3}')
        msg=$(echo "$line" | sed 's/^[^ ]* [^ ]* [^ ]* //')
        
        # Check if it's an Argo service
        if echo "$msg" | grep -qE "argo_(launch|power|battery|bno|health|thermal|storage|radio)"; then
            log_msg "  $ts: $msg"
        fi
    done
) &
MONITOR_PID=$!

# Wait a reasonable time for shutdown to progress
# Services should stop within their TimeoutStopSec values
# We'll wait up to 60 seconds total
for i in {1..60}; do
    sleep 1
    
    # Check if any Argo services are still running
    RUNNING_COUNT=0
    for service in argo_launch_standard argo_power_control argo_battery_water argo_bno085 \
                   argo_health_monitor; do
        if systemctl is-active --quiet "$service.service" 2>/dev/null; then
            ((RUNNING_COUNT++))
        fi
    done
    
    # If all critical services have stopped, we're done monitoring
    if [ $RUNNING_COUNT -eq 0 ]; then
        log_msg "All monitored services have stopped after ${i} seconds"
        break
    fi
    
    # Log progress every 5 seconds
    if [ $((i % 5)) -eq 0 ]; then
        log_msg "Still monitoring... ($i seconds elapsed, $RUNNING_COUNT services still running)"
    fi
done

# Stop the monitoring process
kill $MONITOR_PID 2>/dev/null || true
wait $MONITOR_PID 2>/dev/null || true

# Final service state check
log_msg ""
log_msg "=== FINAL ARGO SERVICE STATES ==="
for service in argo_launch_standard argo_power_control argo_battery_water argo_bno085 \
               argo_health_monitor argo_thermal_monitor argo_storage_monitor \
               argo_radio_servo_module; do
    if systemctl list-unit-files "$service.service" >/dev/null 2>&1; then
        STATE=$(systemctl is-active "$service.service" 2>/dev/null || echo "inactive")
        log_msg "  $service: $STATE"
    fi
done

# Check for critical battery flag
if [ -f /tmp/argo_critical_battery ]; then
    log_msg ""
    log_msg "=== CRITICAL BATTERY FLAG DETECTED ==="
    log_msg "Shutdown may have been triggered by critical battery condition"
fi

# Sync filesystem one final time
log_msg ""
log_msg "=== FINAL FILESYSTEM SYNC ==="
log_msg "Syncing filesystems..."
sync 2>/dev/null || true
log_msg "Sync completed"

# Final timestamp
log_msg ""
log_msg "=== SHUTDOWN TIMING LOGGER COMPLETE ==="
log_msg "Logger completed at $(date '+%Y-%m-%d %H:%M:%S')"

# Force final sync of the log file
sync "$SHUTDOWN_LOG" 2>/dev/null || true

exit 0
