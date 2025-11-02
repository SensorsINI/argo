# WSL Port Forwarding for Foxglove Bridge

This guide explains how to forward port 8765 from WSL2 to Windows so that Foxglove Studio (running on Windows) can connect to the Foxglove Bridge running inside WSL.

## Quick Start

### Method 1: Using Cursor Tasks (Recommended) ✅

1. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
2. Type "Tasks: Run Task"
3. Select "Forward WSL Port 8765 (Foxglove)"
4. You may need to enter your Windows admin password

**Note**: The task automatically bypasses PowerShell execution policy.

### Method 2: Using Batch File (Easy)

1. Navigate to `scripts\` folder in Windows Explorer
2. Right-click `wsl_port_forward.bat`
3. Select "Run as Administrator"
4. Follow the prompts

### Method 3: Manual Commands

From PowerShell (as Administrator):

```powershell
# Get WSL IP
$wslIP = wsl hostname -I | ForEach-Object { $_.Trim() -split '\s+' | Select-Object -First 1 }
Write-Host "WSL IP: $wslIP"

# Remove existing rule
netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0

# Add port forwarding
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wslIP

# Add firewall rule
New-NetFirewallRule -DisplayName "WSL Foxglove Bridge 8765" -Direction Inbound -LocalPort 8765 -Protocol TCP -Action Allow
```

## Verify Port Forwarding

### From Cursor
1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select "Check WSL Port Forwarding"

### From PowerShell
```powershell
# View all port proxy rules
netsh interface portproxy show all

# Test connection
Test-NetConnection -ComputerName localhost -Port 8765
```

### From WSL
```bash
# Check if port is listening
netstat -tuln | grep 8765

# Test connection (if netcat is installed)
nc -zv localhost 8765
```

## Connect Foxglove Studio

After port forwarding is set up:

1. Start the Foxglove Bridge in WSL:
   ```bash
   ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
   ```

2. Open Foxglove Studio: https://studio.foxglove.dev/

3. Connect using:
   - **WebSocket URL**: `ws://localhost:8765`
   - Or directly: `ws://<WSL_IP>:8765` (if firewall allows)

## Remove Port Forwarding

### From Cursor
1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select "Remove WSL Port Forwarding"

### From PowerShell
```powershell
netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0
Remove-NetFirewallRule -DisplayName "WSL Foxglove Bridge 8765"
```

## Troubleshooting

### "Execution Policy" Error
- Use Method 1 (Cursor Tasks) or Method 2 (Batch file) - both bypass execution policy
- Or run PowerShell as Administrator with: `powershell -ExecutionPolicy Bypass -File script.ps1`

### "Access Denied" Error
- All methods require **Administrator privileges**
- Make sure you're running as Administrator

### Port Already in Use
- The script automatically removes existing rules before adding new ones
- If issues persist, manually remove: `netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0`

### Cannot Connect from Foxglove Studio
1. Verify port forwarding: Run "Check WSL Port Forwarding" task
2. Verify Foxglove Bridge is running: `ros2 node list | grep foxglove`
3. Check Windows Firewall: The script should add a rule automatically
4. Try connecting to WSL IP directly: `ws://<WSL_IP>:8765`

### WSL IP Changed
- WSL2 IP addresses can change after reboot
- Just run the port forwarding script again to update it

## Available Cursor Tasks

- **Forward WSL Port 8765 (Foxglove)** - Sets up port forwarding
- **Check WSL Port Forwarding** - Verifies current configuration
- **Remove WSL Port Forwarding** - Removes port forwarding rules

Access via: `Ctrl+Shift+P` → "Tasks: Run Task"

