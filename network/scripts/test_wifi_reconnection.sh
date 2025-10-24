#!/bin/bash

# Argo WiFi Reconnection Test Script
# Tests network switching behavior and power control LED indicators
# Runs for ~3 minutes with automatic tobi-wlan restoration

set -e  # Exit on any error

# Configuration
TEST_DURATION=180  # 3 minutes in seconds
LOG_FILE="/var/log.hdd/persistent/wifi-reconnection-test.log"
POWER_CONTROL_LOG="/var/log.hdd/persistent/argo-power-control.log"
TEST_START_TIME=$(date '+%Y-%m-%d %H:%M:%S')

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging function
log() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${BLUE}[$timestamp]${NC} $1" | tee -a "$LOG_FILE"
}

log_success() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${GREEN}[$timestamp] ✓${NC} $1" | tee -a "$LOG_FILE"
}

log_warning() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${YELLOW}[$timestamp] ⚠${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${RED}[$timestamp] ✗${NC} $1" | tee -a "$LOG_FILE"
}

log_test() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${CYAN}[$timestamp] 🧪${NC} $1" | tee -a "$LOG_FILE"
}

# Cleanup function - ALWAYS restore tobi-wlan
cleanup() {
    log "🧹 CLEANUP: Restoring tobi-wlan connection..."
    
    # Force restore tobi-wlan regardless of any errors
    if nmcli connection up tobi-wlan 2>/dev/null; then
        log_success "tobi-wlan connection restored successfully"
    else
        log_error "Failed to restore tobi-wlan - attempting multiple times..."
        
        # Try multiple times with delays
        for i in {1..5}; do
            sleep 2
            if nmcli connection up tobi-wlan 2>/dev/null; then
                log_success "tobi-wlan restored on attempt $i"
                break
            else
                log_warning "Attempt $i failed, retrying..."
            fi
        done
    fi
    
    # Final status check
    if nmcli connection show --active | grep -q "tobi-wlan"; then
        log_success "✅ tobi-wlan is active - remote access restored"
    else
        log_error "❌ tobi-wlan still not active - manual intervention may be required"
    fi
    
    log "🏁 Test completed at $(date '+%Y-%m-%d %H:%M:%S')"
}

# Set trap to ensure cleanup runs even on script termination
trap cleanup EXIT INT TERM

# Function to get current connection
get_current_connection() {
    nmcli -t -f NAME connection show --active | grep -v "ztw4lntpwi" | head -1
}

# Function to monitor power control logs
monitor_power_control() {
    local duration=$1
    local start_time=$(date +%s)
    
    log_test "Monitoring power control service for $duration seconds..."
    
    # Start monitoring in background
    journalctl -u argo_power_control.service -f --no-pager > "$POWER_CONTROL_LOG" 2>&1 &
    local monitor_pid=$!
    
    # Monitor for the specified duration
    while [ $(($(date +%s) - start_time)) -lt $duration ]; do
        sleep 1
    done
    
    # Stop monitoring
    kill $monitor_pid 2>/dev/null || true
    
    log_test "Power control monitoring completed"
}

# Function to check LED status from power control logs
check_led_status() {
    local expected_pattern="$1"
    local timeout=10
    local start_time=$(date +%s)
    
    while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
        if tail -20 "$POWER_CONTROL_LOG" 2>/dev/null | grep -q "$expected_pattern"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# Function to wait for network change
wait_for_network_change() {
    local expected_network="$1"
    local timeout=30
    local start_time=$(date +%s)
    
    log_test "Waiting for connection to switch to $expected_network (timeout: ${timeout}s)..."
    
    while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
        current=$(get_current_connection)
        if [ "$current" = "$expected_network" ]; then
            log_success "Successfully connected to $expected_network"
            return 0
        fi
        sleep 2
    done
    
    log_error "Timeout waiting for connection to $expected_network"
    return 1
}

# Function to simulate network disconnection
simulate_disconnection() {
    local network="$1"
    log_test "Simulating disconnection from $network..."
    
    if nmcli connection down "$network" 2>/dev/null; then
        log_success "Disconnected from $network"
        return 0
    else
        log_error "Failed to disconnect from $network"
        return 1
    fi
}

# Main test function
run_test() {
    log "🚀 Starting WiFi Reconnection Test"
    log "Test Duration: $TEST_DURATION seconds (~3 minutes)"
    log "Start Time: $TEST_START_TIME"
    log "Log File: $LOG_FILE"
    log "Power Control Log: $POWER_CONTROL_LOG"
    
    # Initial status check
    log "📊 Initial Network Status:"
    nmcli connection show --active | tee -a "$LOG_FILE"
    
    current_connection=$(get_current_connection)
    log "Current connection: $current_connection"
    
    # Test 1: Monitor current state (30 seconds)
    log_test "=== TEST 1: Baseline Monitoring (30s) ==="
    monitor_power_control 30
    
    # Test 2: Simulate tobi-wlan disconnection
    log_test "=== TEST 2: Simulate tobi-wlan Disconnection ==="
    if [ "$current_connection" = "tobi-wlan" ]; then
        simulate_disconnection "tobi-wlan"
        
        # Check for LED status change (WiFi loss indicator)
        log_test "Checking for WiFi loss LED pattern..."
        if check_led_status "WiFi connectivity lost"; then
            log_success "✅ Power control detected WiFi loss - LED pattern activated"
        else
            log_warning "⚠️ Power control may not have detected WiFi loss"
        fi
        
        # Wait for fallback to uzh-iot
        wait_for_network_change "uzh-iot"
        
        # Monitor fallback behavior (60 seconds)
        log_test "Monitoring fallback behavior (60s)..."
        monitor_power_control 60
        
    else
        log_warning "Not connected to tobi-wlan, skipping disconnection test"
    fi
    
    # Test 3: Simulate tobi-wlan restoration
    log_test "=== TEST 3: Simulate tobi-wlan Restoration ==="
    log_test "Attempting to restore tobi-wlan connection..."
    
    if nmcli connection up tobi-wlan 2>/dev/null; then
        log_success "tobi-wlan connection restored"
        
        # Check for LED status change (WiFi restored)
        log_test "Checking for WiFi restoration LED pattern..."
        if check_led_status "WiFi connectivity restored\|LED pattern.*normal"; then
            log_success "✅ Power control detected WiFi restoration"
        else
            log_warning "⚠️ Power control may not have detected WiFi restoration"
        fi
        
        # Monitor restoration behavior (30 seconds)
        log_test "Monitoring restoration behavior (30s)..."
        monitor_power_control 30
        
    else
        log_error "Failed to restore tobi-wlan connection"
    fi
    
    # Test 4: Final monitoring
    log_test "=== TEST 4: Final System Monitoring (30s) ==="
    monitor_power_control 30
    
    # Final status
    log "📊 Final Network Status:"
    nmcli connection show --active | tee -a "$LOG_FILE"
    
    log_success "🎉 WiFi Reconnection Test Completed Successfully!"
}

# Function to show test results
show_results() {
    echo
    log "📋 TEST RESULTS SUMMARY"
    log "======================"
    
    if [ -f "$LOG_FILE" ]; then
        log "Test Log: $LOG_FILE"
        log "Recent test events:"
        tail -10 "$LOG_FILE" | sed 's/^/  /'
    fi
    
    if [ -f "$POWER_CONTROL_LOG" ]; then
        log "Power Control Log: $POWER_CONTROL_LOG"
        log "Power control events during test:"
        grep -E "(WiFi|LED|connectivity)" "$POWER_CONTROL_LOG" 2>/dev/null | tail -5 | sed 's/^/  /' || log "No power control events found"
    fi
    
    echo
    log "🔍 Manual Verification Commands:"
    log "  Current connection: nmcli connection show --active"
    log "  WiFi reconnection log: tail -f /var/log.hdd/persistent/wifi-reconnect.log"
    log "  Power control status: systemctl status argo_power_control.service"
    echo
}

# Check if running as root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

# Main execution
main() {
    check_root
    
    # Create log directory if it doesn't exist
    mkdir -p "$(dirname "$LOG_FILE")"
    mkdir -p "$(dirname "$POWER_CONTROL_LOG")"
    
    # Clear previous logs
    > "$LOG_FILE"
    > "$POWER_CONTROL_LOG"
    
    # Run the test
    run_test
    
    # Show results
    show_results
}

# Run main function
main "$@"

