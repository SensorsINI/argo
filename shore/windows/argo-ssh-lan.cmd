@echo off
setlocal
REM Resolves the Argo host IP via Get-NetNeighbor (same data as: Get-NetNeighbor -AddressFamily IPv4
REM | Select IPAddress, LinkLayerAddress, State) and runs SSH. WiFi MAC: see ..\README.md and argo-ssh-lan.ps1.
REM Usage:  argo-ssh-lan.cmd
REM     or:  argo-ssh-lan.ps1 -ArgoMac "C8-26-E2-6C-58-BA" -SshUser orangepi

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0argo-ssh-lan.ps1" %*
endlocal
exit /b %ERRORLEVEL%
