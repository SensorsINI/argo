#!/bin/bash
# Launch Argo in remote simulation mode

echo "🚢 Starting Argo in REMOTE simulation mode..."
echo "This will launch only the necessary nodes for remote simulation:"
echo "  - argo_unified_simulator_bridge.py (remote mode)"
echo "  - controller.py (autonomous navigation)"
echo "  - battery_water.py (hardware monitoring)"
echo "  - temp_monitor.py (hardware monitoring)"
echo ""
echo "Hardware nodes excluded (conflict with simulator):"
echo "  - gps.py, imu.py, anem.py, rudder_sail_radio.py"
echo ""
echo "Prerequisites:"
echo "  1. SSH tunnel running: ./scripts/remote_simulator_tunnel.sh"
echo "  2. Remote simulator running: python3 scripts/remote_simulator_launch.py"
echo ""

# Change to argo directory
cd "$(dirname "$0")/.."

# Launch the unified simulator bridge in remote mode
echo "Starting unified simulator bridge in remote mode..."
python3 nodes/argo_unified_simulator_bridge.py --mode remote &
SIMULATOR_PID=$!

# Wait a moment for simulator to start
sleep 3

# Launch other simulation nodes
echo "Starting controller..."
python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml &
CONTROLLER_PID=$!

echo "Starting battery monitoring..."
python3 nodes/battery_water.py &
BATTERY_PID=$!

echo "Starting temperature monitoring..."
python3 nodes/temp_monitor.py &
TEMP_PID=$!

echo ""
echo "✅ Argo remote simulation mode started!"
echo "Waiting for connection to remote simulator..."
echo ""
echo "If connection fails, check:"
echo "  1. SSH tunnel is running: ./scripts/remote_simulator_tunnel.sh"
echo "  2. Remote simulator is running: python3 scripts/remote_simulator_launch.py"
echo "  3. Network connectivity to remote host"
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




