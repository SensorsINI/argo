#!/bin/bash

echo "🚢 Argo Robot Control Commands"
echo "=================================="

echo "Service Control:"
echo "  al   - Launch argo service (start all nodes)"
echo "  aq   - Quit argo service (stop all nodes)"
echo "  ars  - Restart argo service"
echo "  as   - Show service status"
echo "  am   - Monitor mode (watch for failures)"
echo "  alog - Follow argo launch logs"

echo "Recording Control:"
echo "  ar   - Start data recording"
echo "  ac   - Stop data recording"

echo "GUI Control:"
echo "  ag   - Launch Argo CLI GUI (interactive control)"

echo "Direct Commands:"
echo "  python3 launch/argo_lifecycle_manager.py run      # Start and keep running"
echo "  python3 launch/argo_lifecycle_manager.py stop     # Stop all nodes"
echo "  python3 launch/argo_lifecycle_manager.py restart  # Restart all nodes"
echo "  python3 launch/argo_lifecycle_manager.py status   # Show status"
echo "  python3 launch/argo_lifecycle_manager.py monitor  # Monitor mode"

echo "Help:"
echo "  argo_help - Show this help"
echo "  make help - Show full Makefile commands"

echo "Examples:"
echo "  al && as    # Start service and check status"
echo "  ar && ac    # Start and stop recording"
echo "  am          # Monitor for node failures"
