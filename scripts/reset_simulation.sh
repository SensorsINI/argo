#!/bin/bash
# Reset simulation to initial state (home waypoint)
# Usage: ./reset_simulation.sh

source /opt/ros/humble/setup.bash

echo "Resetting simulation to initial state..."
ros2 service call /simulator/reset std_srvs/srv/Trigger

if [ $? -eq 0 ]; then
    echo "✅ Simulation reset successful"
else
    echo "❌ Simulation reset failed - is the simulator bridge running?"
    exit 1
fi

