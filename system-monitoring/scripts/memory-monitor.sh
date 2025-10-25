#!/bin/bash
# Comprehensive memory monitoring script
LOG_DIR="/var/log.hdd/persistent"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Function to get memory usage percentage
get_memory_usage() {
    local meminfo=$(cat /proc/meminfo)
    local memtotal=$(echo "$meminfo" | grep MemTotal | awk '{print $2}')
    local memavailable=$(echo "$meminfo" | grep MemAvailable | awk '{print $2}')
    local memused=$((memtotal - memavailable))
    local mempercent=$((memused * 100 / memtotal))
    echo "$mempercent"
}

# Function to get swap usage percentage
get_swap_usage() {
    local meminfo=$(cat /proc/meminfo)
    local swaptotal=$(echo "$meminfo" | grep SwapTotal | awk '{print $2}')
    local swapfree=$(echo "$meminfo" | grep SwapFree | awk '{print $2}')
    if [ "$swaptotal" -gt 0 ]; then
        local swapused=$((swaptotal - swapfree))
        local swappercent=$((swapused * 100 / swaptotal))
        echo "$swappercent"
    else
        echo "0"
    fi
}

# Monitor memory every 30 seconds
while true; do
    # Re-evaluate date and log files on each iteration (handles log rotation at midnight)
    MEM_LOG="$LOG_DIR/memory-$(date +%Y%m%d).log"
    PROC_LOG="$LOG_DIR/processes-$(date +%Y%m%d).log"
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Get memory statistics
    MEMPERCENT=$(get_memory_usage)
    SWAPPERCENT=$(get_swap_usage)
    
    # Get detailed memory info
    MEMINFO=$(cat /proc/meminfo | grep -E "MemTotal|MemFree|MemAvailable|Buffers|Cached|SwapCached|SwapTotal|SwapFree" | tr '\n' ' ' | sed 's/ kB//g' | sed 's/: /=/g')
    
    # Log memory state
    echo "$TIMESTAMP: Memory:${MEMPERCENT}% Swap:${SWAPPERCENT}% $MEMINFO" >> "$MEM_LOG"
    
    # Log process list (every 30 seconds)
    echo "$TIMESTAMP: Top memory consumers:" >> "$PROC_LOG"
    ps aux --sort=-%mem | head -11 >> "$PROC_LOG"
    echo "" >> "$PROC_LOG"
    
    # Check for high memory usage
    if [ "$MEMPERCENT" -gt 85 ]; then
        echo "$TIMESTAMP: HIGH MEMORY USAGE: ${MEMPERCENT}%" >> "$MEM_LOG"
        logger -p daemon.warning "High memory usage: ${MEMPERCENT}%"
    fi
    
    # Check for high swap usage
    if [ "$SWAPPERCENT" -gt 60 ]; then
        logger -p daemon.warning "High swap usage detected: ${SWAPPERCENT}%"
    fi
    
    sleep 30
done
