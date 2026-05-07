#!/bin/bash
# I2C Bus Reset Script
# Resets I2C controller 5002000.i2c by unbinding and rebinding the driver
# This is a software-level reset that only resets driver state, not hardware electrical state
#
# Usage: sudo ./reset_i2c_bus.sh [--force]
#   --force: Skip confirmation prompt

set -e

I2C_CONTROLLER="5002000.i2c"
I2C_DRIVER="mv64xxx_i2c"
I2C_BUS="0"

# Services that use I2C and should be stopped
I2C_SERVICES=(
    "argo_battery_water.service"
    "argo_bno085.service"
    "argo_launch_standard.service"
)

FORCE_MODE=false
if [ "$1" = "--force" ]; then
    FORCE_MODE=true
fi

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

echo "=== I2C Bus Reset Script ==="
echo "Controller: $I2C_CONTROLLER"
echo "Driver: $I2C_DRIVER"
echo "Bus: $I2C_BUS"
echo ""

# Check which I2C services are running
echo "Checking I2C services..."
RUNNING_SERVICES=()
for service in "${I2C_SERVICES[@]}"; do
    if systemctl is-active --quiet "$service" 2>/dev/null; then
        RUNNING_SERVICES+=("$service")
        echo "  ✓ $service is running"
    else
        echo "  - $service is not running"
    fi
done

if [ ${#RUNNING_SERVICES[@]} -eq 0 ]; then
    echo "No I2C services are currently running."
else
    echo ""
    echo "Services that will be stopped:"
    for service in "${RUNNING_SERVICES[@]}"; do
        echo "  - $service"
    done
fi

# Confirmation prompt (unless --force)
if [ "$FORCE_MODE" = false ]; then
    echo ""
    read -p "Continue with I2C bus reset? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
    fi
fi

# Stop I2C services
echo ""
echo "Stopping I2C services..."
for service in "${RUNNING_SERVICES[@]}"; do
    echo "  Stopping $service..."
    systemctl stop "$service" || {
        echo "  ⚠️  Warning: Failed to stop $service (continuing anyway)"
    }
done

# Wait a moment for services to fully stop
sleep 1

# Unbind controller
echo ""
echo "Unbinding I2C controller $I2C_CONTROLLER..."
if [ -f "/sys/bus/platform/drivers/$I2C_DRIVER/unbind" ]; then
    echo "$I2C_CONTROLLER" > "/sys/bus/platform/drivers/$I2C_DRIVER/unbind" || {
        echo "  ⚠️  Warning: Failed to unbind controller (may already be unbound)"
    }
    sleep 0.5
else
    echo "  ❌ Error: Unbind file not found: /sys/bus/platform/drivers/$I2C_DRIVER/unbind"
    echo "  Attempting to restart services anyway..."
fi

# Rebind controller
echo ""
echo "Rebinding I2C controller $I2C_CONTROLLER..."
if [ -f "/sys/bus/platform/drivers/$I2C_DRIVER/bind" ]; then
    echo "$I2C_CONTROLLER" > "/sys/bus/platform/drivers/$I2C_DRIVER/bind" || {
        echo "  ❌ Error: Failed to rebind controller"
        echo "  Attempting to restart services anyway..."
    }
    sleep 1
else
    echo "  ❌ Error: Bind file not found: /sys/bus/platform/drivers/$I2C_DRIVER/bind"
    echo "  Attempting to restart services anyway..."
fi

# Restart services
echo ""
echo "Restarting I2C services..."
for service in "${RUNNING_SERVICES[@]}"; do
    echo "  Starting $service..."
    systemctl start "$service" || {
        echo "  ⚠️  Warning: Failed to start $service"
    }
done

# Wait for services to initialize
sleep 2

# Test bus with i2cdetect
echo ""
echo "Testing I2C bus $I2C_BUS..."
if command -v i2cdetect >/dev/null 2>&1; then
    if i2cdetect -y "$I2C_BUS" >/dev/null 2>&1; then
        echo "  ✓ I2C bus $I2C_BUS is accessible"
        echo ""
        echo "Detected devices on bus $I2C_BUS:"
        i2cdetect -y "$I2C_BUS" | grep -v "^$" | tail -n +2 || true
    else
        echo "  ⚠️  Warning: i2cdetect failed (bus may still be initializing)"
    fi
else
    echo "  ⚠️  Warning: i2cdetect not found (skipping bus test)"
fi

echo ""
echo "=== I2C Bus Reset Complete ==="
echo ""
echo "Note: This is a software-level reset. It only resets driver state,"
echo "      not hardware electrical state. It will NOT recover from bus lock."

exit 0
