#!/bin/bash
# Thermal monitoring script
LOG_DIR="/var/log.hdd/persistent"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Log thermal data every 30 seconds
while true; do
    # Re-evaluate date and log file on each iteration (handles log rotation at midnight)
    LOG_FILE="$LOG_DIR/thermal-$(date +%Y%m%d).log"
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    GPU_TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "N/A")
    VE_TEMP=$(cat /sys/class/thermal/thermal_zone1/temp 2>/dev/null || echo "N/A")
    CPU_TEMP=$(cat /sys/class/thermal/thermal_zone2/temp 2>/dev/null || echo "N/A")
    DDR_TEMP=$(cat /sys/class/thermal/thermal_zone3/temp 2>/dev/null || echo "N/A")
    
    # Convert to Celsius
    GPU_C=$((GPU_TEMP/1000))
    VE_C=$((VE_TEMP/1000))
    CPU_C=$((CPU_TEMP/1000))
    DDR_C=$((DDR_TEMP/1000))
    
    echo "$TIMESTAMP: GPU:${GPU_C}°C VE:${VE_C}°C CPU:${CPU_C}°C DDR:${DDR_C}°C" >> "$LOG_FILE"
    
    # Check for critical temperatures (>80°C)
    if [ "$CPU_C" -gt 80 ] || [ "$GPU_C" -gt 80 ]; then
        echo "$TIMESTAMP: WARNING - High temperature detected! CPU:${CPU_C}°C GPU:${GPU_C}°C" >> "$LOG_FILE"
        logger -p daemon.warning "High temperature detected: CPU:${CPU_C}°C GPU:${GPU_C}°C"
    fi
    
    sleep 30
done
