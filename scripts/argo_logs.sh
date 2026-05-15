#!/bin/bash
#
# Argo Multi-Service Log Viewer
# Tails logs from argo-launch, argo_battery_water, argo_power_control, IMU, health monitor,
# and argo_wifi_reconnect (journal) with color-coded output for easy identification
#
# Usage: argo_logs.sh [OPTIONS] [PATTERN]
#   -n N    Show last N lines (default: 20, ignored when PATTERN provided)
#   -f      Follow mode (tail -f behavior, default)
#   -e      Error check mode: search all logs for ERROR/WARN/FATAL [with optional PATTERN]
#           When combined with -f, shows recent errors (default: last 1 minute) then follows
#   -t TIME Time window for recent logs (e.g., "1m", "5m", "1h", "30s")
#           Default: 1 minute when using -ef combination
#   -a      Show all logs for last 2 boots with optional PATTERN filter
#   -h      Show this help
#   PATTERN Optional grep pattern to filter logs (extended regex supported)
#           Examples: "controller", "anem_node", "rudder", "(controller|dashboard)"

# Default options
# Note: journalctl merges all -u units into one stream; a small -n mostly shows
# the noisiest unit (often launch). Use a larger default so quieter units (IMU)
# still appear in the initial batch.
LINES=200
FOLLOW=true
GREP_PATTERN=""
ERROR_CHECK=false
ALL_LOGS=false
TIME_WINDOW=""

# Color codes for each service
COLOR_ARGO_LAUNCH='\e[0;96m'      # Cyan for argo-launch
COLOR_BATTERY='\e[0;93m'           # Yellow for argo_battery_water
COLOR_POWER='\e[0;92m'             # Green for power_control
COLOR_IMU='\e[0;95m'               # Magenta for BNO085 IMU
COLOR_WIFI='\e[0;94m'              # Bright blue for WiFi reconnect
COLOR_TIMESTAMP='\e[0;90m'         # Gray for timestamps
RESET='\e[0m'

# Priority color codes (unconditional highlighting)
COLOR_ERROR='\e[1;91m'             # Bold bright red for ERROR
COLOR_WARN='\e[1;33m'              # Bold dark yellow for WARN

# Service names
SERVICE_ARGO="argo_launch_standard.service"
SERVICE_BATTERY="argo_battery_water.service"
SERVICE_POWER="argo_power_control.service"
SERVICE_IMU="argo_bno085.service"
SERVICE_HEALTH="argo_health_monitor.service"
SERVICE_WIFI="argo_wifi_reconnect.service"

# Parse command line options
while getopts "n:fet:ha" opt; do
    case $opt in
        n)
            LINES=$OPTARG
            ;;
        f)
            FOLLOW=true
            ;;
        e)
            ERROR_CHECK=true
            # Don't disable FOLLOW when -e is used - allow -ef combination
            ;;
        t)
            TIME_WINDOW="$OPTARG"
            ;;
        a)
            ALL_LOGS=true
            FOLLOW=false
            ;;
        h)
            echo "Argo Multi-Service Log Viewer"
            echo ""
            echo "Usage: argo_logs.sh [OPTIONS] [PATTERN]"
            echo ""
            echo "Options:"
            echo "  -n N    Show last N lines per service (default: 20)"
            echo "          Note: Ignored when PATTERN provided (scans all logs)"
            echo "  -f      Follow mode (default, live tail)"
            echo "  -e      Error check mode: scan ALL logs for ERROR/WARN/FATAL"
            echo "          Optional PATTERN filters by node/service name"
            echo "          When combined with -f, shows recent errors then follows"
            echo "  -t TIME Time window for recent logs (e.g., \"1m\", \"5m\", \"1h\", \"30s\")"
            echo "          Default: 1 minute when using -ef combination"
            echo "  -a      Show all logs since last 2 boots with optional PATTERN filter"
            echo "  -h      Show this help"
            echo ""
            echo "Arguments:"
            echo "  PATTERN Optional grep pattern to filter logs (extended regex supported)"
            echo "          Examples: 'controller', '(controller|dashboard)', 'anem.*node'"
            echo ""
            echo "Services monitored:"
            echo "  - ${SERVICE_ARGO} (cyan)"
            echo "  - ${SERVICE_BATTERY} (yellow)"
            echo "  - ${SERVICE_POWER} (green)"
            echo "  - ${SERVICE_IMU} (magenta)"
            echo "  - ${SERVICE_HEALTH} (cyan)"
            echo "  - ${SERVICE_WIFI} (bright blue)"
            echo ""
            echo "Priority highlighting:"
            echo "  - ERROR lines              (bold bright red)"
            echo "  - WARN lines               (bold dark yellow)"
            echo ""
            echo "Examples:"
            echo "  argo_logs.sh                    # Follow all logs (live)"
            echo "  argo_logs.sh -n 50              # Show last 50 lines"
            echo "  argo_logs.sh controller         # Scan ALL logs for 'controller'"
            echo "  argo_logs.sh -e                 # Check for any errors/warnings (ALL logs)"
            echo "  argo_logs.sh -e anem            # Check for errors in anem node (ALL logs)"
            echo "  argo_logs.sh -ef controller     # Show recent errors matching 'controller', then follow"
            echo "  argo_logs.sh -ef '(controller|dashboard)'  # Show errors matching either, then follow"
            echo "  argo_logs.sh -ef -t 5m          # Show last 5 minutes of errors, then follow"
            echo "  argo_logs.sh -a                 # Show all logs (no filtering)"
            echo "  argo_logs.sh -a controller      # Show all logs filtered by 'controller'"
            exit 0
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done

# Get positional argument (grep pattern) after options
shift $((OPTIND-1))
GREP_PATTERN="$1"

# Set default time window for -ef combination
if [ "$ERROR_CHECK" = true ] && [ "$FOLLOW" = true ] && [ -z "$TIME_WINDOW" ]; then
    TIME_WINDOW="1m"
fi

# When filtering by PATTERN, default to a recent window so we don't spam with
# historical restart-loop chatter (especially during IMU failures).
if [ -n "$GREP_PATTERN" ] && [ "$ALL_LOGS" = false ] && [ -z "$TIME_WINDOW" ]; then
    TIME_WINDOW="5m"
fi

# --- Main Execution ---

echo "📋 Argo Multi-Service Logs"
if [ "$ERROR_CHECK" = true ] && [ "$FOLLOW" = true ]; then
    echo "🚨 Error Follow Mode: Recent errors (last ${TIME_WINDOW}) + follow"
    if [ -n "$GREP_PATTERN" ]; then
        echo "🔍 Filter: $GREP_PATTERN"
    fi
elif [ "$ERROR_CHECK" = true ]; then
    echo "🚨 Error Check Mode: Scanning ALL logs for ERROR/WARN/FATAL"
    if [ -n "$GREP_PATTERN" ]; then
        echo "🔍 Filter: $GREP_PATTERN"
    fi
elif [ "$ALL_LOGS" = true ]; then
    echo "📜 All Logs Mode: Showing all logs from last 2 boots"
    if [ -n "$GREP_PATTERN" ]; then
        echo "🔍 Filter: $GREP_PATTERN"
    fi
elif [ -n "$GREP_PATTERN" ]; then
    echo "🔍 Filter: $GREP_PATTERN (default: last ${TIME_WINDOW:-?} then follow; use -t/-a for more)"
elif [ "$FOLLOW" = true ]; then
    echo "👁️  Follow Mode: Live tail (last $LINES lines + new)"
else
    echo "📄 Static Mode: Last $LINES lines per service"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${COLOR_ARGO_LAUNCH}●${RESET} ${SERVICE_ARGO} (cyan)"
echo -e "${COLOR_BATTERY}●${RESET} ${SERVICE_BATTERY} (yellow)"
echo -e "${COLOR_POWER}●${RESET} ${SERVICE_POWER} (green)"
echo -e "${COLOR_IMU}●${RESET} ${SERVICE_IMU} (magenta)"
echo -e "${COLOR_ARGO_LAUNCH}●${RESET} ${SERVICE_HEALTH} (cyan)"
echo -e "${COLOR_WIFI}●${RESET} ${SERVICE_WIFI} (bright blue)"
echo -e "${COLOR_ERROR}●${RESET} ERROR lines (bold bright red)"
echo -e "${COLOR_WARN}●${RESET} WARN lines (bold dark yellow)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Function to colorize log lines
colorize_logs() {
    while IFS= read -r line; do
        # Priority highlighting for errors and warnings
        if echo "$line" | grep -qi "ERROR"; then
            echo -e "${COLOR_ERROR}${line}${RESET}"
        elif echo "$line" | grep -qi "WARN"; then
            echo -e "${COLOR_WARN}${line}${RESET}"
        # Service-specific coloring
        elif echo "$line" | grep -q "argo_lifecycle_manager\|argo-launch\|argo_health_monitor"; then
            echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
        elif echo "$line" | grep -q "argo_battery_water_node\|argo_battery_water"; then
            echo -e "${COLOR_BATTERY}${line}${RESET}"
        elif echo "$line" | grep -q "argo_power_control"; then
            echo -e "${COLOR_POWER}${line}${RESET}"
        elif echo "$line" | grep -qiE "bno08x|bno085|bno08x_ros|imu_health|/imu"; then
            echo -e "${COLOR_IMU}${line}${RESET}"
        elif echo "$line" | grep -qE "argo-wifi-reconnect|argo_wifi_reconnect"; then
            echo -e "${COLOR_WIFI}${line}${RESET}"
        else
            echo "$line" # No color
        fi
    done
}

# Construct the journalctl command
journalctl_cmd="journalctl --no-pager"
journalctl_cmd+=" -u $SERVICE_ARGO -u $SERVICE_BATTERY -u $SERVICE_POWER -u $SERVICE_IMU -u $SERVICE_HEALTH -u $SERVICE_WIFI"

# Add options based on flags
if [ "$ALL_LOGS" = true ]; then
    journalctl_cmd+=" -b -1 -b 0"
else
    if [ -n "$TIME_WINDOW" ]; then
        journalctl_cmd+=" --since \"${TIME_WINDOW} ago\""
    fi

    # EFFECTIVE_LINES: journalctl merges all units into one stream.
    # In pattern mode we primarily rely on --since (TIME_WINDOW) to bound history.
    EFFECTIVE_LINES=$LINES

    if [ "$ERROR_CHECK" = true ] && [ "$FOLLOW" = true ]; then
        journalctl_cmd+=" -f"
    elif [ "$FOLLOW" = true ]; then
        if [ -n "$TIME_WINDOW" ]; then
            journalctl_cmd+=" -f"
        else
            journalctl_cmd+=" -n $EFFECTIVE_LINES -f"
        fi
    elif [ -z "$GREP_PATTERN" ] && [ "$ERROR_CHECK" = false ] && [ -z "$TIME_WINDOW" ]; then
        journalctl_cmd+=" -n $EFFECTIVE_LINES"
    fi
fi

journalctl_cmd+=" --output=short-iso-precise"

# Build the full command pipeline (do not hide journalctl errors — empty output was misleading)
full_cmd="$journalctl_cmd"

# Filter out foxglove bridge spam (Advertising/Removing channel messages)
full_cmd+=" | grep --line-buffered -v -E 'foxglove_bridge.*(Advertising|Removing).*channel'"

# Apply filters: when -ef is used with pattern, we need both error keywords AND pattern
if [ "$ERROR_CHECK" = true ]; then
    # First filter for errors
    full_cmd+=" | grep --line-buffered -iE 'ERROR|WARN|FATAL'"
    # Then filter for pattern if provided
    if [ -n "$GREP_PATTERN" ]; then
        full_cmd+=" | grep --line-buffered -iE '$GREP_PATTERN'"
    fi
elif [ -n "$GREP_PATTERN" ]; then
    # Just pattern filtering
    full_cmd+=" | grep --line-buffered -iE '$GREP_PATTERN'"
fi

full_cmd+=" | colorize_logs"

# Execute the command
eval "$full_cmd"
