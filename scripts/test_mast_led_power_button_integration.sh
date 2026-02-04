#!/bin/bash
# Test mast LED / power button integration: publish RGB to /argo/power_button/rgb
# so mastleds_node mirrors the same colors on the mast head (PCA9632).
# Requires: ROS2 sourced, mastleds_node and (optionally) power_control running.

set -euo pipefail

ARGO_DIR="${ARGO_DIR:-/home/orangepi/argo}"
SOURCE_ROS="${ARGO_DIR}/scripts/load_config.py"  # or source directly

if ! command -v ros2 &>/dev/null; then
  echo "Sourcing ROS2..."
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash 2>/dev/null || true
fi
if ! command -v ros2 &>/dev/null; then
  echo "Error: ros2 not found. Source ROS2 first: source /opt/ros/humble/setup.bash"
  exit 1
fi

echo "Mast LED / power button integration test"
echo "Publishing to /argo/power_button/rgb (mast head should mirror)."
echo ""

for color in "red" "green" "blue" "off"; do
  case "$color" in
    red)   x=1.0 y=0.0 z=0.0 ;;
    green) x=0.0 y=1.0 z=0.0 ;;
    blue)  x=0.0 y=0.0 z=1.0 ;;
    off)   x=0.0 y=0.0 z=0.0 ;;
  esac
  echo "  -> $color (x=$x y=$y z=$z)"
  ros2 topic pub --once /argo/power_button/rgb geometry_msgs/Vector3 "{x: $x, y: $y, z: $z}" >/dev/null 2>&1
  sleep 2
done

echo ""
echo "Done. If mast head LEDs cycled red -> green -> blue -> off, integration is OK."
echo "(White is not tested: power button is RGB only; mast is RGBW.)"
