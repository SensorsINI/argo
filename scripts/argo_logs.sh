#!/bin/bash
#
# Argo Multi-Service Log Viewer
# Tails logs from argo-launch, battery_water, and argo_power_control services
# with color-coded output for easy identification
#
# Usage: argo_logs.sh [OPTIONS] [PATTERN]
#   -n N    Show last N lines (default: 20)
#   -f      Follow mode (tail -f behavior, default)
#   -h      Show this help
#   PATTERN Optional grep pattern to filter logs (e.g., "controller", "ERROR", "anem_node")

# Default options
LINES=20
FOLLOW=true
GREP_PATTERN=""

# Color codes for each service
COLOR_ARGO_LAUNCH='\e[0;96m'      # Cyan for argo-launch
COLOR_BATTERY='\e[0;93m'           # Yellow for battery_water
COLOR_POWER='\e[0;92m'             # Green for power_control
COLOR_IMU='\e[0;95m'               # Magenta for BNO085 IMU
COLOR_TIMESTAMP='\e[0;90m'         # Gray for timestamps
RESET='\e[0m'

# Priority color codes (unconditional highlighting)
COLOR_ERROR='\e[1;91m'             # Bold bright red for ERROR
COLOR_WARN='\e[1;33m'              # Bold dark yellow for WARN

# Service names
SERVICE_ARGO="argo-launch.service"
SERVICE_BATTERY="battery_water.service"
SERVICE_POWER="argo_power_control.service"
SERVICE_IMU="argo_bno085.service"

# Parse command line options
while getopts "n:fh" opt; do
    case $opt in
        n)
            LINES=$OPTARG
            ;;
        f)
            FOLLOW=true
            ;;
        h)
            echo "Argo Multi-Service Log Viewer"
            echo ""
            echo "Usage: argo_logs.sh [OPTIONS] [PATTERN]"
            echo ""
            echo "Options:"
            echo "  -n N    Show last N lines per service (default: 20)"
            echo "  -f      Follow mode (default)"
            echo "  -h      Show this help"
            echo ""
            echo "Arguments:"
            echo "  PATTERN Optional grep pattern to filter logs"
            echo ""
            echo "Services monitored:"
            echo "  - argo-launch.service      (cyan)"
            echo "  - battery_water.service    (yellow)"
            echo "  - argo_power_control.service (green)"
            echo "  - argo_bno085.service      (magenta)"
            echo ""
            echo "Priority highlighting:"
            echo "  - ERROR lines              (bold bright red)"
            echo "  - WARN lines               (bold dark yellow)"
            echo ""
            echo "Examples:"
            echo "  argo_logs.sh                    # Show all logs"
            echo "  argo_logs.sh controller         # Filter for 'controller'"
            echo "  argo_logs.sh -n 50 ERROR        # Last 50 lines, filter for 'ERROR'"
            echo "  argo_logs.sh 'anem_node'        # Filter for anem_node"
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
if [ -n "$GREP_PATTERN" ]; then
    echo "🔍 Filter: $GREP_PATTERN"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${COLOR_ARGO_LAUNCH}●${RESET} argo-launch.service (cyan)"
echo -e "${COLOR_BATTERY}●${RESET} battery_water.service (yellow)"
echo -e "${COLOR_POWER}●${RESET} argo_power_control.service (green)"
echo -e "${COLOR_IMU}●${RESET} argo_bno085.service (magenta)"
echo -e "${COLOR_ERROR}●${RESET} ERROR lines (bold bright red)"
echo -e "${COLOR_WARN}●${RESET} WARN lines (bold dark yellow)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [ "$FOLLOW" = true ]; then
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
        elif echo "$line" | grep -q "argo_lifecycle_manager\|argo-launch"; then
            # Color the entire line cyan for argo-launch
            echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
        elif echo "$line" | grep -q "battery_water_node\|battery_water"; then
            # Color the entire line yellow for battery_water
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
else
    # Non-follow mode - just show last N lines from each service
    echo -e "${COLOR_ARGO_LAUNCH}=== argo-launch.service ===${RESET}"
    if [ -n "$GREP_PATTERN" ]; then
        journalctl -u "$SERVICE_ARGO" -n "$LINES" --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_ARGO_LAUNCH}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_ARGO" -n "$LINES" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
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
    
    echo -e "${COLOR_BATTERY}=== battery_water.service ===${RESET}"
    if [ -n "$GREP_PATTERN" ]; then
        journalctl -u "$SERVICE_BATTERY" -n "$LINES" --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_BATTERY}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_BATTERY" -n "$LINES" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
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
        journalctl -u "$SERVICE_POWER" -n "$LINES" --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_POWER}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_POWER" -n "$LINES" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
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
        journalctl -u "$SERVICE_IMU" -n "$LINES" --no-pager 2>/dev/null | \
            grep "$GREP_PATTERN" | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_IMU}${line}${RESET}"
                fi
            done
    else
        journalctl -u "$SERVICE_IMU" -n "$LINES" --no-pager 2>/dev/null | \
            while IFS= read -r line; do
                # Apply ERROR/WARN highlighting first, then service color if no priority
                if echo "$line" | grep -qi "ERROR"; then
                    echo -e "${COLOR_ERROR}${line}${RESET}"
                elif echo "$line" | grep -qi "WARN"; then
                    echo -e "${COLOR_WARN}${line}${RESET}"
                else
                    echo -e "${COLOR_IMU}${line}${RESET}"
                fi
            done
    fi
fi

