#!/usr/bin/env bash
set -euo pipefail

# Launch a fullscreen terminal on the local HDMI desktop (Xorg :0)
# and run anem.py in visual debug mode.
# Screen is fully cleared on the first successful redraw after sensor/I2C recovery (see anem.py).

# This script is for debugging the anemometer node when we don't have keyboard/mouse plugged in to OPi but we can plug
# in an HDMI monitor.

# to launch
# DISPLAY=:0 XAUTHORITY=/home/orangepi/.Xauthority nohup /home/orangepi/argo/scripts/anem_debug_visually_terminal.sh >/tmp/anem_debug_terminal.log 2>&1 &

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/orangepi/.Xauthority}"

exec xfce4-terminal \
  --fullscreen \
  --title="anem.py --debug_visually" \
  --command "bash -lc 'cd /home/orangepi/argo && exec python3 /home/orangepi/argo/nodes/anem.py --debug_visually'"
