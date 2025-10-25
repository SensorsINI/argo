#!/bin/bash
# Cursor process monitoring and logging
LOG_DIR="/var/log.hdd/persistent"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Monitor Cursor processes every 60 seconds
while true; do
    # Re-evaluate date and log file on each iteration (handles log rotation at midnight)
    CURSOR_LOG="$LOG_DIR/cursor-processes-$(date +%Y%m%d).log"
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Check for Cursor agent processes
    CURSOR_PROCS=$(ps aux | grep -E "cursor.*server|cursor.*agent|cursor.*extension" | grep -v grep)
    
    if [ -n "$CURSOR_PROCS" ]; then
        # Calculate total memory usage
        TOTAL_MEM=$(echo "$CURSOR_PROCS" | awk '{sum+=$6} END {print sum/1024}')
        PROC_COUNT=$(echo "$CURSOR_PROCS" | wc -l)
        
        echo "$TIMESTAMP: Cursor processes: $PROC_COUNT, Total Memory: ${TOTAL_MEM}MB" >> "$CURSOR_LOG"
        echo "$CURSOR_PROCS" >> "$CURSOR_LOG"
        echo "" >> "$CURSOR_LOG"
        
        # Alert if using too much memory (>1GB)
        if [ $(echo "$TOTAL_MEM > 1000" | bc 2>/dev/null || echo 0) -eq 1 ]; then
            logger -p daemon.warning "Cursor processes using ${TOTAL_MEM}MB memory"
        fi
    else
        echo "$TIMESTAMP: No Cursor processes running" >> "$CURSOR_LOG"
    fi
    
    sleep 60
done
