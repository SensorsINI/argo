# WSL Port Forwarding Script for Windows
# Run this script in PowerShell as Administrator to forward port 8765 from WSL to Windows host
#
# Usage:
#   1. Open PowerShell as Administrator
#   2. Run: .\scripts\wsl_port_forward.ps1
#   3. Or copy the commands below and run manually

Write-Host "Setting up WSL port forwarding for Foxglove Bridge (port 8765)..." -ForegroundColor Cyan

# Get WSL IP address
$wslIP = wsl hostname -I | ForEach-Object { $_.Trim() -split '\s+' | Select-Object -First 1 }
Write-Host "WSL IP Address: $wslIP" -ForegroundColor Green

# Remove existing port proxy rule if it exists
netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0 2>$null
Write-Host "Cleaned up existing port proxy rules" -ForegroundColor Yellow

# Add port proxy rule
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wslIP
Write-Host "Added port proxy: 0.0.0.0:8765 -> $wslIP:8765" -ForegroundColor Green

# Add Windows Firewall rule
$firewallRule = Get-NetFirewallRule -DisplayName "WSL Foxglove Bridge 8765" -ErrorAction SilentlyContinue
if (-not $firewallRule) {
    New-NetFirewallRule -DisplayName "WSL Foxglove Bridge 8765" -Direction Inbound -LocalPort 8765 -Protocol TCP -Action Allow | Out-Null
    Write-Host "Added Windows Firewall rule for port 8765" -ForegroundColor Green
} else {
    Write-Host "Windows Firewall rule already exists" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Port forwarding configured successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "Connect Foxglove Studio to:" -ForegroundColor Cyan
Write-Host "  ws://localhost:8765" -ForegroundColor White
Write-Host "  or" -ForegroundColor Gray
Write-Host "  ws://$wslIP:8765" -ForegroundColor White
Write-Host ""
Write-Host "To view current port proxy rules:" -ForegroundColor Yellow
Write-Host "  netsh interface portproxy show all" -ForegroundColor Gray
Write-Host ""
Write-Host "To remove port forwarding:" -ForegroundColor Yellow
Write-Host "  netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0" -ForegroundColor Gray

