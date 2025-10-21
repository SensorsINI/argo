#!/bin/bash

# Start VNC server with Xfce4 desktop
# Usage: ./start_vnc_xfce.sh [display_number] [geometry]

DISPLAY_NUM=${1:-2}
GEOMETRY=${2:-1920x1080}

echo "Starting VNC server on display :$DISPLAY_NUM with geometry $GEOMETRY"

# Kill any existing VNC session on this display
vncserver -kill :$DISPLAY_NUM 2>/dev/null

# Start VNC server with Xfce4
vncserver :$DISPLAY_NUM -geometry $GEOMETRY -depth 24

echo "VNC server started on display :$DISPLAY_NUM"
echo "Connect with: vncviewer <server-ip>:$((5900 + DISPLAY_NUM))"
echo "Or from localhost: vncviewer localhost:$((5900 + DISPLAY_NUM))"
