#!/bin/bash

# WiFi Reconnection Test Launcher
# Runs the test in background and provides easy access to results

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_SCRIPT="$SCRIPT_DIR/test_wifi_reconnection.sh"
LOG_FILE="/var/log.hdd/persistent/wifi-reconnection-test.log"
PID_FILE="/tmp/wifi-test.pid"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() {
    echo -e "${BLUE}[$(date '+%H:%M:%S')]${NC} $1"
}

show_help() {
    echo "WiFi Reconnection Test Launcher"
    echo "==============================="
    echo ""
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  start     - Start the test in background"
    echo "  status    - Show test status and recent logs"
    echo "  logs      - Show live test logs"
    echo "  stop      - Stop running test"
    echo "  results   - Show test results summary"
    echo "  help      - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 start    # Start test in background"
    echo "  $0 status   # Check if test is running"
    echo "  $0 logs     # Watch live test output"
    echo "  $0 results  # View test results"
}

start_test() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        log "Test is already running (PID: $(cat "$PID_FILE"))"
        return 1
    fi
    
    log "Starting WiFi reconnection test in background..."
    
    # Run test in background
    nohup sudo "$TEST_SCRIPT" > /dev/null 2>&1 &
    local test_pid=$!
    
    # Save PID
    echo $test_pid > "$PID_FILE"
    
    log "Test started with PID: $test_pid"
    log "Use '$0 status' to check progress"
    log "Use '$0 logs' to watch live output"
    
    return 0
}

show_status() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        local pid=$(cat "$PID_FILE")
        log "✅ Test is running (PID: $pid)"
        
        # Show recent logs
        if [ -f "$LOG_FILE" ]; then
            echo ""
            log "Recent test activity:"
            tail -5 "$LOG_FILE" | sed 's/^/  /'
        fi
    else
        log "❌ Test is not running"
        
        # Clean up stale PID file
        [ -f "$PID_FILE" ] && rm -f "$PID_FILE"
        
        # Show last test results if available
        if [ -f "$LOG_FILE" ]; then
            echo ""
            log "Last test results:"
            tail -10 "$LOG_FILE" | sed 's/^/  /'
        fi
    fi
}

show_logs() {
    if [ -f "$LOG_FILE" ]; then
        log "Showing live test logs (Ctrl+C to exit):"
        echo ""
        tail -f "$LOG_FILE"
    else
        log "No test log file found. Start a test first with '$0 start'"
    fi
}

stop_test() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        local pid=$(cat "$PID_FILE")
        log "Stopping test (PID: $pid)..."
        
        # Send SIGTERM to allow cleanup
        kill -TERM "$pid" 2>/dev/null
        
        # Wait a moment for graceful shutdown
        sleep 2
        
        # Force kill if still running
        if kill -0 "$pid" 2>/dev/null; then
            log "Force stopping test..."
            kill -KILL "$pid" 2>/dev/null
        fi
        
        rm -f "$PID_FILE"
        log "Test stopped"
    else
        log "No test is currently running"
        [ -f "$PID_FILE" ] && rm -f "$PID_FILE"
    fi
}

show_results() {
    if [ -f "$LOG_FILE" ]; then
        log "WiFi Reconnection Test Results"
        log "=============================="
        echo ""
        
        # Show test summary
        if grep -q "WiFi Reconnection Test Completed" "$LOG_FILE"; then
            echo -e "${GREEN}✅ Test completed successfully${NC}"
        elif grep -q "Starting WiFi Reconnection Test" "$LOG_FILE"; then
            echo -e "${YELLOW}⚠️ Test may still be running or was interrupted${NC}"
        else
            echo -e "${YELLOW}⚠️ Test status unclear${NC}"
        fi
        
        echo ""
        log "Full test log:"
        cat "$LOG_FILE"
        
        echo ""
        log "Power control events:"
        if [ -f "/var/log.hdd/persistent/argo-power-control.log" ]; then
            grep -E "(WiFi|LED|connectivity)" "/var/log.hdd/persistent/argo-power-control.log" 2>/dev/null | tail -10 | sed 's/^/  /' || log "No power control events found"
        else
            log "No power control log found"
        fi
        
    else
        log "No test results found. Run a test first with '$0 start'"
    fi
}

# Main command handling
case "${1:-help}" in
    start)
        start_test
        ;;
    status)
        show_status
        ;;
    logs)
        show_logs
        ;;
    stop)
        stop_test
        ;;
    results)
        show_results
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        log "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
