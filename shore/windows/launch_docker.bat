@echo off
REM ============================================================================
REM Argo Shore Station Docker Launcher for Windows
REM ============================================================================
REM 
REM This batch file launches the Argo shore station in a Docker container.
REM 
REM REQUIREMENTS:
REM   - Docker Desktop for Windows installed and running
REM   - Argo project directory accessible
REM 
REM See README_WINDOWS_SETUP.md for complete setup instructions.
REM ============================================================================

echo.
echo ========================================
echo Argo Shore Station Docker Launcher
echo ========================================
echo.

REM Check if Docker is installed and running
docker --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not installed or not running!
    echo.
    echo Please install Docker Desktop for Windows:
    echo   https://www.docker.com/products/docker-desktop/
    echo.
    pause
    exit /b 1
)

echo [1/3] Docker detected: 
docker --version
echo.

REM Get the directory where this batch file is located
set SCRIPT_DIR=%~dp0
set ARGO_DIR=%SCRIPT_DIR%..\..

echo [2/3] Building Docker image (first time only, may take 5-10 minutes)...
echo.

REM Build Docker image
docker build -t argo-shore "%SCRIPT_DIR%"
if %errorlevel% neq 0 (
    echo ERROR: Docker build failed!
    pause
    exit /b 1
)

echo.
echo [3/3] Starting Argo shore station container...
echo.
echo Mounting Argo directory from: %ARGO_DIR%
echo.

REM Run Docker container with:
REM   - Privileged mode for USB access
REM   - Port 8081 exposed for web dashboard
REM   - Argo directory mounted
REM   - Device access for USB serial

docker run --rm -it ^
    --privileged ^
    -p 8081:8081 ^
    -v "%ARGO_DIR%":/argo ^
    --device-cgroup-rule='c *:* rmw' ^
    argo-shore

REM Note: USB device passthrough on Windows requires additional configuration
REM See: https://docs.docker.com/desktop/windows/wsl/#gpu-support

echo.
echo Shore station stopped.
pause


