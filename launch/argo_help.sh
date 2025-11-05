#!/bin/bash

echo "🚢 Argo Robot Control Commands"
echo "=================================="

echo "Direct Commands:"
echo "  python3 launch/argo_lifecycle_manager.py status   # Show status"
echo "  python3 launch/argo_lifecycle_manager.py monitor  # Monitor mode"
echo "  python3 launch/argo_lifecycle_manager.py simulate_local   # Start local simulation"
echo "  python3 launch/argo_lifecycle_manager.py simulate_remote  # Start remote simulation"

echo "Help:"
echo "  argo_help - Show this help"
echo "  make help - Show full Makefile commands"

echo "Service Control:"
echo "  al   - Launch argo service (start all nodes)"
echo "  aq   - Quit argo service (stop all nodes)"
echo "  ars  - Restart argo service"
echo "  as   - Show service status"
echo "  am   - Monitor mode (watch for failures)"
echo "  alog - Follow argo launch logs"
echo "  adevcheck - Check for expected hardware devices"

echo "Simulation:"
echo "  asim      - Start argo in LOCAL simulation mode"
echo "  asimr     - Start argo in REMOTE simulation mode"
echo "  asimreset - Reset simulation to origin/home location"
echo "  asimkb    - Start keyboard control for simulation"

echo "Recording Control:"
echo "  ar   - Start data recording"
echo "  ac   - Stop data recording"
echo "  apb  - Play back latest recording with visualization"

echo "GUI Control:"
echo "  ag   - Launch Argo CLI GUI (interactive control)"

echo "Examples:"
echo "  al && as    # Start service and check status"
echo "  ar && ac    # Start and stop recording"
echo "  am          # Monitor for node failures"
