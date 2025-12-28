#!/bin/bash

if systemctl is-active --quiet argo_battery_water.service; then
    source /opt/ros/humble/setup.bash && \
    timeout 30 ros2 service call /battery_status std_srvs/srv/Trigger "{}" 2>&1 | \
    ${ARGO_DIR:-$HOME/argo}/scripts/format_service_response.sh
else
    echo "❌ Error: argo_battery_water.service is not running"
    echo "   Start it first with: sudo systemctl start argo_battery_water.service"
fi
