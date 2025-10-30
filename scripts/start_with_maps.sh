#!/bin/bash
# Start Argo with sailing area map visualization

# Determine Argo repository directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ARGO_DIR="$(dirname "$SCRIPT_DIR")"

echo "Starting Argo with sailing area map visualization..."

# Convert KMZ files to GeoJSON if needed
echo "Converting KMZ files to GeoJSON..."
python3 "$ARGO_DIR/scripts/kmz_to_geojson.py"

# Start rosbridge for Foxglove connection
echo "Starting rosbridge server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Wait a moment for rosbridge to start
sleep 3

# Start sailing area publisher
echo "Starting sailing area publisher..."
python3 "$ARGO_DIR/nodes/sailing_area_publisher.py" &
SAILING_PUB_PID=$!

# Wait a moment for publisher to start
sleep 2

# Start Argo system
echo "Starting Argo system..."
sudo python3 "$ARGO_DIR/launch/argo_lifecycle_manager.py" start &
ARGO_PID=$!

echo ""
echo "Argo with map visualization started!"
echo ""
echo "To connect Foxglove:"
echo "1. Open https://studio.foxglove.dev/"
echo "2. Connect to: ws://$(hostname -I | awk '{print $1}'):9090"
echo "3. Import layout: $ARGO_DIR/foxglove/argo_maps_final.json"
echo ""
echo "To test map visualization:"
echo "python3 $ARGO_DIR/scripts/test_map_visualization.py"
echo ""
echo "Press Ctrl+C to stop all services"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Stopping services..."
    kill $ROSBRIDGE_PID $SAILING_PUB_PID $ARGO_PID 2>/dev/null
    echo "Services stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for user interrupt
wait
