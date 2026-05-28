#!/bin/bash
# Argo Stop Script - Stop Argo nodes via standard ROS2 launch system

echo "🛑 Stopping Argo ROS2 nodes..."

# Determine service name (support both old and new)
SERVICE_NAME="argo_launch_standard.service"

# Try to stop the service
if sudo systemctl stop "$SERVICE_NAME" 2>/dev/null; then
    echo "✅ Argo launch service stopped successfully"
else
    echo "⚠️  Failed to stop $SERVICE_NAME via systemctl"
fi

# Always run akill to catch orphan node processes started outside the service
# (e.g. via 'ars <node>' which uses start_new_session=True and escapes the cgroup).
# akill.sh preserves critical services (battery, power, health, bno085).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AKILL="$SCRIPT_DIR/../scripts/akill.sh"
if [[ -x "$AKILL" ]]; then
    echo "🧹 Cleaning up any orphan Argo processes..."
    "$AKILL" || true
else
    echo "⚠️  akill.sh not found at $AKILL — orphan processes may survive"
fi

# Verify the service is stopped
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "⚠️  Service still appears to be running"
    echo "📋 Checking service status..."
    sudo systemctl status "$SERVICE_NAME" --no-pager -l
else
    echo "✅ Argo launch service confirmed stopped"
fi

