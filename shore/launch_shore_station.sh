#!/bin/bash
################################################################################
# Argo Shore Station Launcher
################################################################################
# Launches LoRa shore receiver and web dashboard

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS2 environment (try both Humble and Foxy)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "ERROR: ROS2 not found! Run setup_shore_station.sh first."
    exit 1
fi

# Detect serial port
LORA_PORT=""
for port in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
    if [ -e "$port" ]; then
        LORA_PORT="$port"
        break
    fi
done

if [ -z "$LORA_PORT" ]; then
    echo "WARNING: No USB serial device found (/dev/ttyUSB* or /dev/ttyACM*)"
    echo "Please connect your LoRa USB device and try again."
    echo ""
    echo "If using WSL2, attach device with:"
    echo "  usbipd list"
    echo "  usbipd attach --wsl --busid YOUR_BUSID"
    exit 1
fi

echo "========================================"
echo "Argo Shore Station"
echo "========================================"
echo ""
echo "Starting services..."
echo "  LoRa port: $LORA_PORT"
echo "  Web dashboard: http://localhost:8081"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch LoRa receiver in background
cd "$ARGO_ROOT"
python3 shore/lora_shore.py --port "$LORA_PORT" &
LORA_PID=$!

# Wait for LoRa to initialize
sleep 2

# Launch web dashboard
python3 nodes/argo_web_dashboard.py &
DASHBOARD_PID=$!

# Wait a moment for dashboard to start
sleep 3

echo ""
echo "========================================"
echo "Shore Station Running!"
echo "========================================"
echo ""
echo "Web Dashboard: http://localhost:8081"
echo ""
echo "To find your IP for remote access:"
echo "  hostname -I"
echo "  Then access from phone: http://YOUR_IP:8081"
echo ""
echo "Press Ctrl+C to stop both services"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping shore station..."
    kill $LORA_PID 2>/dev/null
    kill $DASHBOARD_PID 2>/dev/null
    wait $LORA_PID 2>/dev/null
    wait $DASHBOARD_PID 2>/dev/null
    echo "Stopped."
    exit 0
}

# Set trap for Ctrl+C
trap cleanup SIGINT SIGTERM

# Wait for both processes
wait
