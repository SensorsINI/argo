#!/bin/bash
# Launch Argo in local simulation mode

echo "🚢 Starting Argo in LOCAL simulation mode..."
echo "This will launch only the necessary nodes for simulation:"
echo "  - argo_unified_simulator_bridge.py (local mode)"
echo "  - controller.py (autonomous navigation)"
echo "  - argo_battery_water.py (hardware monitoring)"
echo "  - temp_monitor.py (hardware monitoring)"
echo ""
echo "Hardware nodes excluded (conflict with simulator):"
echo "  - gps.py, imu.py, anem.py, rudder_sail_radio.py"
echo ""

# Change to argo directory
cd "$(dirname "$0")/.."

# Launch the unified simulator bridge in local mode
echo "Starting unified simulator bridge in local mode..."
python3 nodes/argo_unified_simulator_bridge.py --mode local &
SIMULATOR_PID=$!

# Wait a moment for simulator to start
sleep 3

# Launch other simulation nodes
echo "Starting controller..."
python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml &
CONTROLLER_PID=$!

echo "Starting battery monitoring..."
python3 nodes/argo_battery_water.py &
BATTERY_PID=$!

echo "Starting temperature monitoring..."
python3 nodes/temp_monitor.py &
TEMP_PID=$!

echo ""
echo "✅ Argo simulation mode started!"
echo "Simulated sensor data available on:"
echo "  - /pose (compass heading)"
echo "  - /gps_cog, /gps_sog, /gps_velocity (GPS navigation)"
echo "  - /anem_speed_angle_temp (wind data)"
echo "  - /rudder_sail_radio (mock human input)"
echo ""
echo "Control commands sent to simulator via /rudder_sail_servo"
echo ""
echo "Press Ctrl+C to stop all simulation nodes"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Stopping simulation nodes..."
    kill $SIMULATOR_PID $CONTROLLER_PID $BATTERY_PID $TEMP_PID 2>/dev/null
    wait
    echo "✅ All simulation nodes stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for all background processes
wait




