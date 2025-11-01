#!/bin/bash
#
# Argo Multi-Service Log Viewer
# Tails logs from argo-launch, argo_battery_water, and argo_power_control services
# with color-coded output for easy identification
#
# Usage: argo_logs.sh [OPTIONS] [PATTERN]
#   -n N    Show last N lines (default: 20, ignored when PATTERN provided)
#   -f      Follow mode (tail -f behavior, default)
#   -e      Error check mode: search all logs for ERROR/WARN/FATAL [with optional PATTERN]
#   -a      Show all logs for last 2 boots with optional PATTERN filter
#   -h      Show this help
#   PATTERN Optional grep pattern to filter logs (searches ALL logs, not just last N)
#           Examples: "controller", "anem_node", "rudder"

# Default options
LINES=20
FOLLOW=true
GREP_PATTERN=""
ERROR_CHECK=false
ALL_LOGS=false

# Color codes for each service
COLOR_ARGO_LAUNCH='\e[0;96m'      # Cyan for argo-launch
COLOR_BATTERY='\e[0;93m'           # Yellow for argo_battery_water
COLOR_POWER='\e[0;92m'             # Green for power_control
COLOR_IMU='\e[0;95m'               # Magenta for BNO085 IMU
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

# Parse command line options
while getopts "n:feh:a" opt; do
    case $opt in
        n)
            LINES=$OPTARG
            ;;
        f)
            FOLLOW=true
            ;;
        e)
            ERROR_CHECK=true
            FOLLOW=false
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
            echo "  -a      Show all logs since last 2 boots with optional PATTERN filter"
            echo "  -h      Show this help"
            echo ""
            echo "Arguments:"
            echo "  PATTERN Optional grep pattern to filter logs (scans ALL logs)"
            echo ""
            echo "Services monitored:"
            echo "  - argo_launch.service      (cyan)"
            echo "  - argo_battery_water.service (yellow)"
            echo "  - argo_power_control.service (green)"
            echo "  - argo_bno085.service      (magenta)"
            echo ""
            echo "Priority highlighting:"
            echo "  - ERROR lines              (bold bright red)"
            echo "  - WARN lines               (bold dark yellow)"
            echo ""
            echo "Examples:"
            echo "  argo_logs.sh                    # Follow all logs (live)"
            echo "  argo_logs.sh -n 50              # Show last 50 lines"
            echo "  argo_logs.sh controller         # Scan ALL logs for 'controller'"
            echo "  argo_logs.sh -e                 # Check for any errors/warnings"
            echo "  argo_logs.sh -e anem            # Check for errors in anem node"
            echo "  argo_logs.sh -e controller      # Check for controller errors"
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

# Check if services exist
check_service() {
    local service=$1
    if ! systemctl list-unit-files | grep -q "^${service}"; then
        echo "Warning: ${service} not found" >&2
        return 1
    fi
    return 0
}

# Apply ERROR and WARN highlighting to a line
highlight_priority() {
    local line="$1"
    
    # Check for ERROR first (higher priority)
    if echo "$line" | grep -qi "ERROR"; then
        echo -e "${COLOR_ERROR}${line}${RESET}"
    # Then check for WARN
    elif echo "$line" | grep -qi "WARN"; then
        echo -e "${COLOR_WARN}${line}${RESET}"
    else
        echo "$line"
    fi
}

echo "📋 Argo Multi-Service Logs"
if [ "$ERROR_CHECK" = true ]; then
    echo "🚨 Error Check Mode: Scanning ALL logs for ERROR/WARN/FATAL"
    if [ -n "$GREP_PATTERN" ]; then
        echo "🔍 Filter: $GREP_PATTERN"
    fi
elif [ "$ALL_LOGS" = true ]; then
    echo "📜 All Logs Mode: Showing all logs"
    if [ -n "$GREP_PATTERN" ]; then
        echo "🔍 Filter: $GREP_PATTERN"
    fi
elif [ -n "$GREP_PATTERN" ]; then
    echo "🔍 Filter: $GREP_PATTERN (scanning ALL logs)"
elif [ "$FOLLOW" = true ]; then
    echo "👁️  Follow Mode: Live tail (last $LINES lines + new)"
else
    echo "📄 Static Mode: Last $LINES lines per service"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${COLOR_ARGO_LAUNCH}●${RESET} ${SERVICE_ARGO:-argo_launch service} (cyan)"
echo -e "${COLOR_BATTERY}●${RESET} argo_battery_water.service (yellow)"
echo -e "${COLOR_POWER}●${RESET} argo_power_control.service (green)"
echo -e "${COLOR_IMU}●${RESET} argo_bno085.service (magenta)"
echo -e "${COLOR_ERROR}●${RESET} ERROR lines (bold bright red)"
echo -e "${COLOR_WARN}●${RESET} WARN lines (bold dark yellow)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [ "$ERROR_CHECK" = true ]; then
    # Error check mode - scan ALL logs for ERROR/WARN/FATAL with optional pattern filter
    echo -e "${COLOR_ERROR}=== Scanning for errors/warnings ===${RESET}"
    
    # Build grep pattern for error messages
    ERROR_PATTERN="ERROR\|WARN\|FATAL\|error\|warn\|fatal"
    
    # Combine with user pattern if provided
    if [ -n "$GREP_PATTERN" ]; then
        # Use grep pipeline to match both error keywords AND user pattern
        journalctl --no-pager \
            -u "$SERVICE_ARGO" \
            -u "$SERVICE_BATTERY" \
            -u "$SERVICE_POWER" \
            -u "$SERVICE_IMU" \
            --output=short-iso-precise \
            2>/dev/null | \
        grep -i "$ERROR_PATTERN" | \
        grep "$GREP_PATTERN" | \
        awk '
        {
            # Extract the core message (remove timestamps, process info, and variable numbers)
            core_msg = $0
            gsub(/^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]+[+-][0-9]{4} [^:]+: /, "", core_msg)
            gsub(/\[[0-9]+\.[0-9]+\]/, "", core_msg)
            gsub(/\[[0-9]+\]/, "", core_msg)
            # Remove variable timeout numbers (e.g., "for 46015s" -> "for XXXs")
            gsub(/for [0-9]+s/, "for XXXs", core_msg)
            # Remove variable channel numbers (e.g., "channels 0.0us" -> "channels X.Xus")
            gsub(/channels [0-9]+\.[0-9]+us/, "channels X.Xus", core_msg)
            # Remove any remaining numbers that might vary
            gsub(/[0-9]+/, "X", core_msg)
            
            # Count occurrences of each core message
            count[core_msg]++
            if (count[core_msg] == 1) {
                # Store the first occurrence for display
                first_occurrence[core_msg] = $0
            }
        }
        END {
            # Display throttled results
            for (msg in count) {
                if (count[msg] == 1) {
                    print first_occurrence[msg]
                } else {
                    # Show count for repeated messages
                    print first_occurrence[msg] " " "\033[0;90m(+" count[msg]-1 " more)\033[0m"
                }
            }
        }' | \
        while IFS= read -r line; do
            # Apply color based on error level and service
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            elif echo "$line" | grep -q "argo_lifecycle_manager\|argo-launch\|argo_health_monitor"; then
                echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
            elif echo "$line" | grep -q "argo_battery_water_node\|argo_battery_water"; then
                echo -e "${COLOR_BATTERY}${line}${RESET}"
            elif echo "$line" | grep -q "argo_power_control"; then
                echo -e "${COLOR_POWER}${line}${RESET}"
            elif echo "$line" | grep -q "bno08x_driver\|argo_bno085"; then
                echo -e "${COLOR_IMU}${line}${RESET}"
            else
                echo "$line"
            fi
        done
    else
        # Just error keywords, no additional filtering
        journalctl --no-pager \
            -u "$SERVICE_ARGO" \
            -u "$SERVICE_BATTERY" \
            -u "$SERVICE_POWER" \
            -u "$SERVICE_IMU" \
            --output=short-iso-precise \
            2>/dev/null | \
        grep -i "$ERROR_PATTERN" | \
        awk '
        {
            # Extract the core message (remove timestamps, process info, and variable numbers)
            core_msg = $0
            gsub(/^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]+[+-][0-9]{4} [^:]+: /, "", core_msg)
            gsub(/\[[0-9]+\.[0-9]+\]/, "", core_msg)
            gsub(/\[[0-9]+\]/, "", core_msg)
            # Remove variable timeout numbers (e.g., "for 46015s" -> "for XXXs")
            gsub(/for [0-9]+s/, "for XXXs", core_msg)
            # Remove variable channel numbers (e.g., "channels 0.0us" -> "channels X.Xus")
            gsub(/channels [0-9]+\.[0-9]+us/, "channels X.Xus", core_msg)
            # Remove any remaining numbers that might vary
            gsub(/[0-9]+/, "X", core_msg)
            
            # Count occurrences of each core message
            count[core_msg]++
            if (count[core_msg] == 1) {
                # Store the first occurrence for display
                first_occurrence[core_msg] = $0
            }
        }
        END {
            # Display throttled results
            for (msg in count) {
                if (count[msg] == 1) {
                    print first_occurrence[msg]
                } else {
                    # Show count for repeated messages
                    print first_occurrence[msg] " " "\033[0;90m(+" count[msg]-1 " more)\033[0m"
                }
            }
        }' | \
        while IFS= read -r line; do
            # Apply color based on error level and service
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            elif echo "$line" | grep -q "argo_lifecycle_manager\|argo-launch\|argo_health_monitor"; then
                echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
            elif echo "$line" | grep -q "argo_battery_water_node\|argo_battery_water"; then
                echo -e "${COLOR_BATTERY}${line}${RESET}"
            elif echo "$line" | grep -q "argo_power_control"; then
                echo -e "${COLOR_POWER}${line}${RESET}"
            elif echo "$line" | grep -q "bno08x_driver\|argo_bno085"; then
                echo -e "${COLOR_IMU}${line}${RESET}"
            else
                echo "$line"
            fi
        done
    fi
elif [ "$ALL_LOGS" = true ]; then
    # All logs mode - show logs from current and previous boot for shutdown/boot debugging
    echo -e "${COLOR_ARGO_LAUNCH}=== ${SERVICE_ARGO:-argo_launch service} ===${RESET}"
    if [ -n "$GREP_PATTERN" ]; then
        journalctl -u "$SERVICE_ARGO" --boot=-1..0 --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_ARGO" --boot=-1..0 --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
                fi
            done
    fi
    echo ""
    
    echo -e "${COLOR_BATTERY}=== argo_battery_water.service ===${RESET}"
    if [ -n "$GREP_PATTERN" ]; then
        journalctl -u "$SERVICE_BATTERY" --boot=-1..0 --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_BATTERY}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_BATTERY" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_BATTERY}${line}${RESET}"
                fi
            done
    fi
    echo ""
    
    echo -e "${COLOR_POWER}=== argo_power_control.service ===${RESET}"
    if [ -n "$GREP_PATTERN" ]; then
        journalctl -u "$SERVICE_POWER" --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_POWER}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_POWER" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_POWER}${line}${RESET}"
                fi
            done
    fi
    echo ""
    
    echo -e "${COLOR_IMU}=== argo_bno085.service ===${RESET}"
    if [ -n "$GREP_PATTERN" ]; then
        journalctl -u "$SERVICE_IMU" --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_IMU}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_IMU" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_IMU}${line}${RESET}"
                fi
            done
    fi
elif [ "$FOLLOW" = true ]; then
    # Follow mode - use journalctl with multiple units and color them
    # Use a single journalctl process but add service prefixes for identification
    journalctl -n "$LINES" -f --no-pager \
        -u "$SERVICE_ARGO" \
        -u "$SERVICE_BATTERY" \
        -u "$SERVICE_POWER" \
        -u "$SERVICE_IMU" \
        --output=short-iso-precise \
        2>/dev/null | \
    (if [ -n "$GREP_PATTERN" ]; then grep --line-buffered "$GREP_PATTERN"; else cat; fi) | \
    while IFS= read -r line; do
        # First check for ERROR/WARN (highest priority)
        if echo "$line" | grep -qi "ERROR"; then
            echo -e "${COLOR_ERROR}${line}${RESET}"
        elif echo "$line" | grep -qi "WARN"; then
            echo -e "${COLOR_WARN}${line}${RESET}"
        # Then check for service names in the log line (they appear in different formats)
        elif echo "$line" | grep -q "argo_lifecycle_manager\|argo-launch\|argo_health_monitor"; then
            # Color the entire line cyan for argo-launch
            echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
        elif echo "$line" | grep -q "argo_battery_water_node\|argo_battery_water"; then
            # Color the entire line yellow for argo_battery_water
            echo -e "${COLOR_BATTERY}${line}${RESET}"
        elif echo "$line" | grep -q "argo_power_control"; then
            # Color the entire line green for power_control
            echo -e "${COLOR_POWER}${line}${RESET}"
        elif echo "$line" | grep -q "bno08x_driver\|argo_bno085"; then
            # Color the entire line magenta for BNO085 IMU
            echo -e "${COLOR_IMU}${line}${RESET}"
        else
            # Default: no color (fallback)
            echo "$line"
        fi
    done
elif [ -n "$GREP_PATTERN" ]; then
    # Pattern search mode - scan ALL logs (no line limit) for the pattern
    echo -e "${COLOR_ARGO_LAUNCH}=== ${SERVICE_ARGO:-argo_launch service} ===${RESET}"
    journalctl -u "$SERVICE_ARGO" --no-pager 2>/dev/null | \
        grep "$GREP_PATTERN" | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
            fi
        done
    echo ""
    
    echo -e "${COLOR_BATTERY}=== argo_battery_water.service ===${RESET}"
    journalctl -u "$SERVICE_BATTERY" --no-pager 2>/dev/null | \
        grep "$GREP_PATTERN" | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_BATTERY}${line}${RESET}"
            fi
        done
    echo ""
    
    echo -e "${COLOR_POWER}=== argo_power_control.service ===${RESET}"
    journalctl -u "$SERVICE_POWER" --no-pager 2>/dev/null | \
        grep "$GREP_PATTERN" | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_POWER}${line}${RESET}"
            fi
        done
    echo ""
    
    echo -e "${COLOR_IMU}=== argo_bno085.service ===${RESET}"
    journalctl -u "$SERVICE_IMU" --no-pager 2>/dev/null | \
        grep "$GREP_PATTERN" | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_IMU}${line}${RESET}"
            fi
        done
else
    # Non-follow, non-pattern mode - just show last N lines from each service
    echo -e "${COLOR_ARGO_LAUNCH}=== ${SERVICE_ARGO:-argo_launch service} ===${RESET}"
    journalctl -u "$SERVICE_ARGO" -n "$LINES" --no-pager 2>/dev/null | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
            fi
        done
    echo ""
    
    echo -e "${COLOR_BATTERY}=== argo_battery_water.service ===${RESET}"
    journalctl -u "$SERVICE_BATTERY" -n "$LINES" --no-pager 2>/dev/null | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_BATTERY}${line}${RESET}"
            fi
        done
    echo ""
    
    echo -e "${COLOR_POWER}=== argo_power_control.service ===${RESET}"
    journalctl -u "$SERVICE_POWER" -n "$LINES" --no-pager 2>/dev/null | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_POWER}${line}${RESET}"
            fi
        done
    echo ""
    
    echo -e "${COLOR_IMU}=== argo_bno085.service ===${RESET}"
    journalctl -u "$SERVICE_IMU" -n "$LINES" --no-pager 2>/dev/null | \
        while IFS= read -r line; do
            if echo "$line" | grep -qi "ERROR"; then
                echo -e "${COLOR_ERROR}${line}${RESET}"
            elif echo "$line" | grep -qi "WARN"; then
                echo -e "${COLOR_WARN}${line}${RESET}"
            else
                echo -e "${COLOR_IMU}${line}${RESET}"
            fi
        done
fi
