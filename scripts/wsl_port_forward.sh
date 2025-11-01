#!/bin/bash
# WSL Port Forwarding Helper Script
# This script helps set up port forwarding from WSL to Windows host
#
# In Windows 11, WSL2 ports are usually automatically forwarded, but you may need
# to run the PowerShell script on Windows to ensure port 8765 is accessible.

set -e

WSL_IP=$(hostname -I | awk '{print $1}')

echo "WSL IP Address: $WSL_IP"
echo ""
echo "To forward port 8765 from WSL to Windows host, run this in PowerShell (as Administrator):"
echo ""
echo "  netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$WSL_IP"
echo ""
echo "Or use the PowerShell script:"
echo "  .\\scripts\\wsl_port_forward.ps1"
echo ""
echo "After setting up port forwarding, connect Foxglove Studio to:"
echo "  ws://localhost:8765"
echo ""
echo "Alternative: Connect directly to WSL IP (if Windows firewall allows):"
echo "  ws://$WSL_IP:8765"
echo ""
echo "To check if port forwarding is working, test from Windows PowerShell:"
echo "  Test-NetConnection -ComputerName localhost -Port 8765"

