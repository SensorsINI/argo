#!/bin/bash
#
# test_wifi_loss.sh - Temporarily disable WiFi to test WiFi loss detection
#
# This script will:
# 1. Disable WiFi interface (wlan0)
# 2. Wait for specified duration (default 60 seconds)
# 3. Re-enable WiFi interface
#
# WARNING: This will disconnect your SSH session if connected via WiFi!
# The script runs in the background with nohup to ensure WiFi is restored.
#
# Usage:
#   ./test_wifi_loss.sh [duration_seconds]
#   
# Example:
#   ./test_wifi_loss.sh 60    # Disable WiFi for 60 seconds
#   ./test_wifi_loss.sh 120   # Disable WiFi for 120 seconds

# Default duration: 60 seconds
DURATION=${1:-60}

# Log file for tracking the test
LOG_FILE="/tmp/wifi_loss_test.log"

echo "========================================" | tee -a "$LOG_FILE"
echo "WiFi Loss Test - $(date)" | tee -a "$LOG_FILE"
echo "Duration: ${DURATION} seconds" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Check if wlan0 exists
if ! ip link show wlan0 &>/dev/null; then
    echo "ERROR: WiFi interface wlan0 not found!" | tee -a "$LOG_FILE"
    exit 1
fi

# Get current WiFi status
echo "Current WiFi status:" | tee -a "$LOG_FILE"
ip addr show wlan0 | grep "inet " | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Disable WiFi
echo "$(date): Disabling WiFi interface wlan0..." | tee -a "$LOG_FILE"
sudo ip link set wlan0 down

if [ $? -eq 0 ]; then
    echo "$(date): WiFi disabled successfully" | tee -a "$LOG_FILE"
else
    echo "$(date): ERROR: Failed to disable WiFi" | tee -a "$LOG_FILE"
    exit 1
fi

echo "$(date): Waiting ${DURATION} seconds before re-enabling WiFi..." | tee -a "$LOG_FILE"
echo "$(date): SSH connection will be lost if connected via WiFi!" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Wait for the specified duration
sleep "$DURATION"

# Re-enable WiFi
echo "$(date): Re-enabling WiFi interface wlan0..." | tee -a "$LOG_FILE"
sudo ip link set wlan0 up

if [ $? -eq 0 ]; then
    echo "$(date): WiFi re-enabled successfully" | tee -a "$LOG_FILE"
else
    echo "$(date): ERROR: Failed to re-enable WiFi" | tee -a "$LOG_FILE"
    exit 1
fi

# Wait a moment for WiFi to reconnect
echo "$(date): Waiting 10 seconds for WiFi to reconnect..." | tee -a "$LOG_FILE"
sleep 10

# Check WiFi status after re-enabling
echo "" | tee -a "$LOG_FILE"
echo "$(date): WiFi status after re-enabling:" | tee -a "$LOG_FILE"
ip addr show wlan0 | grep "inet " | tee -a "$LOG_FILE"

echo "" | tee -a "$LOG_FILE"
echo "$(date): WiFi loss test completed!" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Show the log file location
echo "Full test log saved to: $LOG_FILE"





