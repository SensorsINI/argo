#!/bin/bash
# Safe Hardware Watchdog Installation for Argo
# Integrates with Makefile installation process

set -e

echo "=================================================================="
echo "Argo Hardware Watchdog Installation"
echo "=================================================================="
echo ""
echo "⚠️  CRITICAL: This will enable hardware watchdog protection."
echo ""
echo "Configuration:"
echo "  • Runtime timeout: 10 seconds (system hang detection)"
echo "  • Reboot timeout: 2 minutes (shutdown hang detection)"
echo "  • Hardware timeout: 16 seconds (H618 fixed limit)"
echo ""
echo "Benefits:"
echo "  ✓ Auto-recovery from I2C bus hangs"
echo "  ✓ Auto-recovery from complete system freezes"
echo "  ✓ Auto-recovery from hung shutdowns"
echo ""
echo "Requirements:"
echo "  ⚠️  HDMI monitor + keyboard connected (for emergency access)"
echo "  ⚠️  SSH session must remain open during testing"
echo ""

# Check if we're on Orange Pi
if [ ! -f /proc/device-tree/compatible ] || ! grep -q "orangepi" /proc/device-tree/compatible 2>/dev/null; then
    echo "❌ Error: This script is only for Orange Pi hardware"
    exit 1
fi

# Check for watchdog device
if [ ! -e /dev/watchdog ]; then
    echo "❌ Error: /dev/watchdog device not found"
    echo "   Hardware watchdog may not be available on this system"
    exit 1
fi

# Verify watchdog hardware in dmesg
if ! dmesg | grep -q "sunxi-wdt"; then
    echo "❌ Error: sunxi-wdt driver not detected in kernel"
    echo "   Hardware watchdog may not be properly initialized"
    exit 1
fi

echo ""
read -p "Do you have HDMI monitor + keyboard connected? (yes/no): " has_console

if [ "$has_console" != "yes" ]; then
    echo ""
    echo "❌ Aborted: Console access is REQUIRED for safe testing"
    echo ""
    echo "Please connect HDMI monitor and keyboard, then run again."
    exit 1
fi

echo ""
read -p "Continue with watchdog installation? (yes/no): " confirm

if [ "$confirm" != "yes" ]; then
    echo "Aborted."
    exit 0
fi

# Create backup directory
BACKUP_DIR="/home/orangepi/argo_watchdog_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
echo ""
echo "Creating backups in: $BACKUP_DIR"

# Backup systemd configuration
if [ -f /etc/systemd/system.conf ]; then
    sudo cp /etc/systemd/system.conf "$BACKUP_DIR/system.conf.backup"
    echo "  ✓ Backed up /etc/systemd/system.conf"
fi

# Backup watchdog configuration
if [ -f /etc/watchdog.conf ]; then
    sudo cp /etc/watchdog.conf "$BACKUP_DIR/watchdog.conf.backup"
    echo "  ✓ Backed up /etc/watchdog.conf"
fi

# Ensure watchdog daemon is disabled and masked
echo ""
echo "Disabling watchdog daemon (we use systemd watchdog instead)..."

if systemctl is-active --quiet watchdog.service 2>/dev/null; then
    sudo systemctl stop watchdog.service
    echo "  ✓ Stopped watchdog.service"
fi

if systemctl is-enabled --quiet watchdog.service 2>/dev/null; then
    sudo systemctl disable watchdog.service
    echo "  ✓ Disabled watchdog.service"
fi

if ! systemctl is-masked --quiet watchdog.service 2>/dev/null; then
    sudo systemctl mask watchdog.service
    echo "  ✓ Masked watchdog.service"
fi

# Configure systemd watchdog
echo ""
echo "Configuring systemd watchdog..."

# Backup original if not already backed up
if [ ! -f /etc/systemd/system.conf.pre-watchdog ]; then
    sudo cp /etc/systemd/system.conf /etc/systemd/system.conf.pre-watchdog
fi

# Update RuntimeWatchdogSec
if grep -q "^RuntimeWatchdogSec=" /etc/systemd/system.conf; then
    sudo sed -i 's/^RuntimeWatchdogSec=.*/RuntimeWatchdogSec=10s/' /etc/systemd/system.conf
else
    sudo sed -i 's/^#RuntimeWatchdogSec=.*/RuntimeWatchdogSec=10s/' /etc/systemd/system.conf
fi

# Update RebootWatchdogSec
if grep -q "^RebootWatchdogSec=" /etc/systemd/system.conf; then
    sudo sed -i 's/^RebootWatchdogSec=.*/RebootWatchdogSec=2min/' /etc/systemd/system.conf
else
    sudo sed -i 's/^#RebootWatchdogSec=.*/RebootWatchdogSec=2min/' /etc/systemd/system.conf
fi

# Update WatchdogDevice
if grep -q "^WatchdogDevice=" /etc/systemd/system.conf; then
    sudo sed -i 's|^WatchdogDevice=.*|WatchdogDevice=/dev/watchdog|' /etc/systemd/system.conf
else
    sudo sed -i 's|^#WatchdogDevice=.*|WatchdogDevice=/dev/watchdog|' /etc/systemd/system.conf
fi

echo "  ✓ Configured systemd watchdog"

# Show current configuration
echo ""
echo "Current watchdog configuration:"
grep "^RuntimeWatchdogSec" /etc/systemd/system.conf || echo "  RuntimeWatchdogSec: (not set)"
grep "^RebootWatchdogSec" /etc/systemd/system.conf || echo "  RebootWatchdogSec: (not set)"
grep "^WatchdogDevice" /etc/systemd/system.conf || echo "  WatchdogDevice: (not set)"

echo ""
echo "=================================================================="
echo "Configuration Complete - TESTING REQUIRED"
echo "=================================================================="
echo ""
echo "⚠️  CRITICAL: Watchdog is configured but NOT YET ACTIVE"
echo ""
echo "Testing procedure:"
echo ""
echo "1. ✅ Keep this SSH session open"
echo ""
echo "2. 🔌 Ensure HDMI monitor is showing console"
echo ""
echo "3. 🚀 Activate watchdog:"
echo "   sudo systemctl daemon-reexec"
echo ""
echo "4. ⏱️  Monitor for 5 minutes"
echo "   - If system reboots → Configuration failed, needs adjustment"
echo "   - If system stable → Watchdog working correctly!"
echo ""
echo "5. ✅ If stable, watchdog is protecting your system"
echo ""
echo "=================================================================="
echo "Emergency Recovery (if reboot loop occurs):"
echo "=================================================================="
echo ""
echo "From console (HDMI monitor):"
echo "  1. Log in at console"
echo "  2. Run: sudo nano /etc/systemd/system.conf"
echo "  3. Comment out (add # at start):"
echo "     #RuntimeWatchdogSec=10s"
echo "     #RebootWatchdogSec=2min"
echo "     #WatchdogDevice=/dev/watchdog"
echo "  4. Save and run: sudo systemctl daemon-reexec"
echo ""
echo "Or restore from backup:"
echo "  sudo cp $BACKUP_DIR/system.conf.backup /etc/systemd/system.conf"
echo "  sudo systemctl daemon-reexec"
echo ""
echo "=================================================================="
echo "Backups saved to: $BACKUP_DIR"
echo "=================================================================="
echo ""

