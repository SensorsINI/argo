@echo off
REM ============================================================================
REM Create Desktop Shortcut for Argo Shore Station
REM ============================================================================
REM 
REM This script creates a desktop shortcut to the Argo shore station launcher.
REM Run this once after completing the WSL2 setup.
REM ============================================================================

echo.
echo Creating desktop shortcut for Argo Shore Station...
echo.

REM Get the directory where this batch file is located
set SCRIPT_DIR=%~dp0

REM Get the desktop path
set DESKTOP=%USERPROFILE%\Desktop

REM Create a shortcut using PowerShell
powershell -Command "$WS = New-Object -ComObject WScript.Shell; $SC = $WS.CreateShortcut('%DESKTOP%\Argo Shore Station.lnk'); $SC.TargetPath = '%SCRIPT_DIR%launch_argo_shore.bat'; $SC.WorkingDirectory = '%SCRIPT_DIR%'; $SC.Description = 'Launch Argo Shore Station (LoRa + Web Dashboard)'; $SC.Save()"

if %errorlevel% equ 0 (
    echo ✓ Shortcut created successfully!
    echo.
    echo Location: %DESKTOP%\Argo Shore Station.lnk
    echo.
    echo You can now double-click this shortcut to launch the shore station.
) else (
    echo ✗ Failed to create shortcut.
    echo.
    echo You can manually create a shortcut to:
    echo   %SCRIPT_DIR%launch_argo_shore.bat
)

echo.
pause


