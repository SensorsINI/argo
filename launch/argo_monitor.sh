#!/bin/bash
#
# Argo Monitor Script
# Continuously displays Argo status with proper buffering
#
PAUSE_TIME=3

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_DIR="$(dirname "$SCRIPT_DIR")"

# Colors for better output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}👁️  Starting Argo Monitor...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop monitoring${NC}"
echo

# Function to run status and capture output
run_status() {
    python3 "$ARGO_DIR/launch/argo_lifecycle_manager.py" status
}

# Main monitoring loop
while true; do
    # Capture the status output to a buffer
    status_output=$(run_status 2>&1)
    exit_code=$?
    
    # Clear screen and display the buffered output
    clear
    echo "$status_output"
    echo -e "${YELLOW}Press Ctrl+C to stop monitoring${NC}. Next update in $PAUSE_TIME seconds."
    
    # Wait 10 seconds before next update
    sleep $PAUSE_TIME
done
