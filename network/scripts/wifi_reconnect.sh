#!/bin/bash

# Argo WiFi Reconnection Script
# Forces connection to preferred networks when they become available
# Priority: tobi-matebook (laptop hotspot) > tobi-s24 (phone) > tobi-wlan (travel router) > CapoCacciaWorkshop2026 (workshop LAN)
# 
# SAFETY FEATURES:
# - Only runs when called by systemd timer (every 1 minute)
# - Checks for higher-priority preferred networks even when already on preferred network
# - Includes connection stability checks
# - Logs all actions for debugging

# Configuration
PREFERRED_NETWORKS=("tobi-matebook" "tobi-s24" "tobi-wlan" "CapoCacciaWorkshop2026")
# Extra preferred networks (added via dashboard) are listed here, one SSID per line:
EXTRA_PREFERRED_NETWORKS_FILE="/home/orangepi/argo/network/config/extra_preferred_networks.txt"
# Passwords are stored in NetworkManager system connections (e.g. /etc/NetworkManager/system-connections).
LOG_FILE="/var/log.hdd/persistent/wifi-reconnect-$(date +%Y%m%d).log"
MAX_LOG_SIZE=1048576  # 1MB
LOCK_FILE="/tmp/wifi_reconnect.lock"
MIN_CONNECTION_TIME=30  # Minimum seconds to stay on a connection before switching

# When ZeroTier is unhealthy on a given SSID, we want enough info in logs to debug later.
# Keep snapshots short and rate-limited so the 1-minute timer can't spam/hang.
ZT_HEALTH_LOG_THROTTLE_SECONDS=300  # 5 minutes
ZT_HEALTH_LASTLOG_FILE="/tmp/zt_health_lastlog_timestamp"

# Load extra preferred networks from file (appended after core list)
load_extra_preferred_networks() {
    if [ ! -f "$EXTRA_PREFERRED_NETWORKS_FILE" ]; then
        return 0
    fi

    while IFS= read -r line; do
        line="$(echo "$line" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
        if [ -z "$line" ] || [[ "$line" == \#* ]]; then
            continue
        fi

        # Extras are appended *after* core list and must never override core priority.
        # If an "extra" duplicates a core SSID, skip it (core order wins).
        local already=false
        for n in "${PREFERRED_NETWORKS[@]}"; do
            if [ "$n" = "$line" ]; then
                already=true
                break
            fi
        done

        if [ "$already" = false ]; then
            PREFERRED_NETWORKS+=("$line")
        fi
    done < "$EXTRA_PREFERRED_NETWORKS_FILE"
}

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

run_cmd_short() {
    local label="$1"
    shift

    # Ensure we never hang the timer. `timeout` exit 124 is fine; we log it.
    local out
    out="$(timeout 4 "$@" 2>&1)"
    local rc=$?

    if [ $rc -eq 0 ]; then
        log_message "[health] $label: ${out//$'\n'/ | }"
    else
        # Truncate very long outputs (e.g. help text) to keep logs readable.
        out="$(echo "$out" | head -n 6 | tr '\n' '|' | sed 's/|$//')"
        log_message "[health] $label: (rc=$rc) $out"
    fi
}

log_network_health_snapshot() {
    local context="$1"

    log_message "[health] ===== snapshot begin ($context) ====="
    run_cmd_short "nm-active" nmcli -t -f ACTIVE,NAME,DEVICE,TYPE connection show --active
    run_cmd_short "ip-addr" ip -br addr
    run_cmd_short "ip-route-default" bash -c "ip route show default || true"
    run_cmd_short "dns-resolvectl" bash -c "command -v resolvectl >/dev/null 2>&1 && resolvectl status | rg -n \"Current DNS Server|DNS Servers|Link\" | head -n 40 || echo \"resolvectl not found\""

    # ZeroTier health
    if command -v zerotier-cli >/dev/null 2>&1; then
        run_cmd_short "zt-info" zerotier-cli info
        run_cmd_short "zt-listnetworks" zerotier-cli listnetworks
        run_cmd_short "zt-listpeers" bash -c "zerotier-cli listpeers | head -n 30"
    else
        log_message "[health] zerotier-cli not found on PATH"
    fi

    log_message "[health] ===== snapshot end ($context) ====="
}

zt_health_is_unhealthy() {
    # Heuristic: if ZeroTier isn't ONLINE or no networks are OK, treat as unhealthy.
    if ! command -v zerotier-cli >/dev/null 2>&1; then
        return 0
    fi

    local info
    info="$(timeout 3 zerotier-cli info 2>/dev/null || true)"
    echo "$info" | grep -q " ONLINE" || return 0

    local nets
    nets="$(timeout 3 zerotier-cli listnetworks 2>/dev/null || true)"
    echo "$nets" | grep -q " OK " || return 0

    return 1
}

maybe_log_zt_health_throttled() {
    local reason="$1"

    local now
    now="$(date +%s)"

    local last=0
    if [ -f "$ZT_HEALTH_LASTLOG_FILE" ]; then
        last="$(cat "$ZT_HEALTH_LASTLOG_FILE" 2>/dev/null || echo 0)"
    fi

    local diff=$((now - last))
    if [ $diff -lt $ZT_HEALTH_LOG_THROTTLE_SECONDS ]; then
        return 0
    fi

    echo "$now" > "$ZT_HEALTH_LASTLOG_FILE"
    log_network_health_snapshot "$reason"
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

    # Extend preferred list with user-added networks (appended at end)
    load_extra_preferred_networks
    log_message "Preferred network priority order: ${PREFERRED_NETWORKS[*]}"
    
    # Get current connection
    CURRENT_CONNECTION=$(get_current_connection)
    
    if [ -z "$CURRENT_CONNECTION" ]; then
        # Exit 0 so systemd does not mark the oneshot failed; NM may still be
        # associating right after boot. The timer will run again shortly.
        log_message "No active WiFi connection yet; skipping (will retry on next timer run)"
        return 0
    fi
    
    log_message "Current connection: $CURRENT_CONNECTION"
    
    # Check if current connection is stable before switching
    if ! is_connection_stable "$CURRENT_CONNECTION"; then
        return 0
    fi

    # If we're on a potentially problematic network and ZeroTier looks unhealthy,
    # capture enough info to debug remotely later.
    if [ "$CURRENT_CONNECTION" = "CapoCacciaWorkshop2026" ]; then
        if zt_health_is_unhealthy; then
            maybe_log_zt_health_throttled "zt-unhealthy-on-$CURRENT_CONNECTION"
        fi
    fi
    
    # Check if we're already on the highest-priority preferred network
    # The array is ordered by priority (lower index = higher priority)
    IS_ON_PREFERRED=false
    CURRENT_PRIORITY_INDEX=-1
    
    for i in "${!PREFERRED_NETWORKS[@]}"; do
        if [ "$CURRENT_CONNECTION" = "${PREFERRED_NETWORKS[$i]}" ]; then
            IS_ON_PREFERRED=true
            CURRENT_PRIORITY_INDEX=$i
            log_message "Current connection is preferred network: ${PREFERRED_NETWORKS[$i]} (priority index $i)"
            break
        fi
    done
    
    # If on a preferred network, only check for higher-priority ones
    if [ "$IS_ON_PREFERRED" = true ]; then
        # Check if there's a higher-priority preferred network available
        for i in "${!PREFERRED_NETWORKS[@]}"; do
            if [ $i -lt $CURRENT_PRIORITY_INDEX ]; then
                # This is a higher-priority network
                if is_network_available "${PREFERRED_NETWORKS[$i]}"; then
                    log_message "Higher-priority preferred network ${PREFERRED_NETWORKS[$i]} is available, switching"
                    if connect_to_network "${PREFERRED_NETWORKS[$i]}"; then
                        record_connection_change "${PREFERRED_NETWORKS[$i]}"
                        log_network_health_snapshot "after-switch-to-${PREFERRED_NETWORKS[$i]}"
                        return 0
                    fi
                fi
            fi
        done
        log_message "Already on highest-priority preferred network, staying on: $CURRENT_CONNECTION"
        return 0
    fi
    
    # Current connection is not preferred, check for any preferred networks
    log_message "Current connection ($CURRENT_CONNECTION) is not preferred, checking for better options"
    
    for preferred in "${PREFERRED_NETWORKS[@]}"; do
        if is_network_available "$preferred"; then
            log_message "Preferred network $preferred is available, attempting connection"
            if connect_to_network "$preferred"; then
                record_connection_change "$preferred"
                log_network_health_snapshot "after-switch-to-$preferred"
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
