#!/bin/bash

# Temperature logging script for Orange Pi Zero 2W
# Logs thermal zone temperatures to /var/log/temperature.log

LOG_FILE="/var/log/temperature.log"
DATE_FORMAT="%Y-%m-%d %H:%M:%S"

# Create log file if it doesn't exist
touch "$LOG_FILE"

# Function to get temperature in Celsius
get_temp_celsius() {
    local temp_millicelsius=$1
    echo "scale=1; $temp_millicelsius / 1000" | bc -l
}

# Get current timestamp
timestamp=$(date "+$DATE_FORMAT")

# Read temperatures from all thermal zones
gpu_temp=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")
ve_temp=$(cat /sys/class/thermal/thermal_zone1/temp 2>/dev/null || echo "0")
cpu_temp=$(cat /sys/class/thermal/thermal_zone2/temp 2>/dev/null || echo "0")
ddr_temp=$(cat /sys/class/thermal/thermal_zone3/temp 2>/dev/null || echo "0")

# Convert to Celsius
gpu_celsius=$(get_temp_celsius $gpu_temp)
ve_celsius=$(get_temp_celsius $ve_temp)
cpu_celsius=$(get_temp_celsius $cpu_temp)
ddr_celsius=$(get_temp_celsius $ddr_temp)

# Log the temperatures
echo "$timestamp,GPU:$gpu_celsius°C,VE:$ve_celsius°C,CPU:$cpu_celsius°C,DDR:$ddr_celsius°C" >> "$LOG_FILE"

# Optional: Log to syslog as well
logger -t temp-logger "Temps: GPU:$gpu_celsius°C VE:$ve_celsius°C CPU:$cpu_celsius°C DDR:$ddr_celsius°C"

# Check for high temperatures and log warnings
if (( $(echo "$cpu_celsius > 80" | bc -l) )); then
    logger -t temp-logger -p user.warning "HIGH CPU TEMPERATURE: $cpu_celsius°C"
fi

if (( $(echo "$gpu_celsius > 80" | bc -l) )); then
    logger -t temp-logger -p user.warning "HIGH GPU TEMPERATURE: $gpu_celsius°C"
fi

if (( $(echo "$ddr_celsius > 85" | bc -l) )); then
    logger -t temp-logger -p user.warning "HIGH DDR TEMPERATURE: $ddr_celsius°C"
fi