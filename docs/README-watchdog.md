# Hardware Watchdog Setup for Argo

## Status: ✅ TESTED AND WORKING

**Last tested:** December 5, 2025  
**Test result:** Kernel panic test successful - system auto-rebooted in ~20 seconds  
**Current status:** Watchdog enabled and protecting system 24/7

## Overview

The Orange Pi H618 has a hardware watchdog (`sunxi-wdt`) that automatically reboots the system if it hangs. This system has been successfully installed, tested, and verified on the Argo robot.

**Real-world problem solved:** System completely froze during I2C bus testing (WiFi dead, power button unresponsive, required battery unplug). With watchdog enabled, the system now auto-recovers from such hangs within 20 seconds.

## When You Need Watchdog

Watchdog protects against:
- ✅ **I2C bus hangs** (like the freeze that required battery unplug)
- ✅ **Complete system freezes** (no SSH, no WiFi, no power button)
- ✅ **Kernel deadlocks**
- ✅ **Hung shutdown processes**

Watchdog does NOT protect against:
- ❌ Individual service crashes (systemd handles this)
- ❌ Network disconnections (not a system hang)
- ❌ High CPU load (not a hang, just slow)
- ❌ Application-level bugs (process still runs)

## Hardware Specifications

- **Device**: `/dev/watchdog` (sunxi-wdt driver)
- **Hardware timeout**: 16 seconds (FIXED, cannot be changed)
- **Platform**: Allwinner H618 SoC

## Configuration

### Systemd Watchdog Settings

Location: `/etc/systemd/system.conf`

```ini
[Manager]
RuntimeWatchdogSec=10s      # System hang detection (MUST be < 16s)
RebootWatchdogSec=2min      # Shutdown hang detection
WatchdogDevice=/dev/watchdog
```

### How It Works

**During Normal Operation:**
1. Systemd "pets" the watchdog every 5 seconds (half of 10s)
2. Hardware watchdog resets its 16-second countdown
3. System continues running normally

**During a Hang:**
1. System freezes (I2C lockup, kernel panic, etc.)
2. Systemd can't pet the watchdog
3. Hardware watchdog times out after 16 seconds
4. **Hardware forces immediate reboot**
5. System boots fresh and recovers

**During Shutdown:**
1. Normal runtime watchdog is disabled
2. Reboot watchdog (2-minute timer) activates
3. If shutdown hangs > 2 minutes, hardware forces reboot
4. Prevents stuck "shutting down" state

## Installation

### Prerequisites

**⚠️ CRITICAL: HDMI Monitor + Keyboard Required**

You MUST have console access before installing watchdog. If configuration fails, you'll need console to recover from reboot loops.

### Step 1: Check Current Status

```bash
make status-watchdog
```

This shows:
- Hardware watchdog device status
- Kernel driver status
- Current systemd configuration
- Watchdog daemon status

### Step 2: Install Watchdog

```bash
make install-watchdog
```

This script will:
1. ✅ Verify hardware watchdog is present
2. ✅ Check you have console access
3. ✅ Backup current configuration
4. ✅ Disable conflicting watchdog daemon
5. ✅ Configure systemd watchdog
6. ✅ Provide testing instructions

**Important**: The script configures but does NOT activate the watchdog. You must test it first.

### Step 3: Test Configuration

**Keep your SSH session open during testing!**

In a **second SSH session** or at console:

```bash
sudo systemctl daemon-reexec
```

**Monitor for 5 minutes:**
- ✅ If system stable → Watchdog working correctly!
- ❌ If reboot loops → Follow emergency recovery below

### Step 4: Verify Watchdog is Active

```bash
make status-watchdog
```

Should show:
```
✅ Watchdog ENABLED
   RuntimeWatchdogSec=10s
   RebootWatchdogSec=2min
   WatchdogDevice=/dev/watchdog
```

Also check kernel logs:
```bash
dmesg | grep -i watchdog
```

Should show:
```
sunxi-wdt 30090a0.watchdog: Watchdog enabled (timeout=16 sec, nowayout=0)
```

## Testing

### Safe Test (Verify Configuration)

```bash
make status-watchdog
```

Shows current configuration without triggering any action.

### Reboot Test (Verify Functionality)

**⚠️ WARNING: This WILL reboot your system!**

Only run if:
- Watchdog is installed and active
- You have console access (HDMI monitor)
- You can afford a system reboot

```bash
make test-watchdog
```

**Test Results (December 5, 2025):**
- ✅ Triggered kernel panic using `/proc/sysrq-trigger`
- ✅ System auto-rebooted in ~20 seconds (hardware watchdog timeout)
- ✅ System came back up normally with watchdog still enabled
- ✅ Configuration persisted across reboot
- ✅ **CONFIRMED WORKING**

This verifies:
- Hardware watchdog detects complete system freeze
- Automatic reboot occurs within expected timeframe
- No manual intervention required
- Configuration survives reboot

## Emergency Recovery

### If Reboot Loop Occurs

**Option 1: From Console (HDMI Monitor)**

1. System will briefly show login prompt during boot
2. Log in quickly: `orangepi` / password
3. Disable watchdog:
   ```bash
   make disable-watchdog
   ```

**Option 2: Manual Fix at Console**

1. Log in at console
2. Edit config:
   ```bash
   sudo nano /etc/systemd/system.conf
   ```
3. Comment out watchdog lines (add `#`):
   ```ini
   #RuntimeWatchdogSec=10s
   #RebootWatchdogSec=2min
   #WatchdogDevice=/dev/watchdog
   ```
4. Save and reload:
   ```bash
   sudo systemctl daemon-reexec
   ```

**Option 3: SD Card Recovery**

1. Power off and remove SD card
2. Mount SD card on another Linux system
3. Edit `etc/systemd/system.conf` on mounted partition
4. Comment out watchdog lines
5. Safely unmount and reinstall SD card
6. Boot normally

### If You Can't Access Console

1. Remove SD card
2. Mount on another computer
3. Navigate to mounted partition
4. Restore backup:
   ```bash
   cp home/orangepi/argo_watchdog_backup_*/system.conf.backup etc/systemd/system.conf
   ```
5. Safely unmount and reinstall

## Disabling Watchdog

To permanently disable:

```bash
make disable-watchdog
```

This will:
1. Backup current configuration
2. Comment out watchdog settings
3. Reload systemd
4. Show new status

## Troubleshooting

### Previous Installation Failure (December 5, 2025 - Morning)

**What happened:**
- Initial attempt used 60-second RuntimeWatchdogSec timeout
- Hardware watchdog has FIXED 16-second timeout (cannot be changed)
- Systemd waited ~30s between watchdog pings (half of 60s)
- Hardware timed out after 16s → immediate reboot loop
- Required SD card removal and manual recovery

**Root cause:**
- Mismatch between configured timeout (60s) and hardware limit (16s)
- Systemd couldn't refresh watchdog fast enough

**Fix applied:**
- Changed RuntimeWatchdogSec from 60s to 10s
- Systemd now pings every ~5s (well under 16s limit)
- **Result: System stable, test successful**

### Watchdog Causing Unexpected Reboots

**Symptom**: System reboots every 10-20 seconds

**Cause**: Systemd can't pet the watchdog fast enough

**Solution**:
1. Disable immediately: `make disable-watchdog`
2. Check system load: `uptime` (extremely high load may delay systemd)
3. Review logs: `journalctl -b -1` (check last boot)
4. Current 10s timeout has been tested and works correctly

### Watchdog Not Triggering During Hangs

**Symptom**: System hangs but doesn't reboot

**Cause**: Watchdog not actually enabled

**Solution**:
1. Check status: `make status-watchdog`
2. Verify settings are **uncommented** in `/etc/systemd/system.conf`
3. Reload systemd: `sudo systemctl daemon-reexec`
4. Test with: `make test-watchdog` (verified working on this system)

### Hardware Watchdog Not Available

**Symptom**: `/dev/watchdog` doesn't exist

**Note**: This is NOT an issue on Argo - hardware watchdog confirmed present and working.

**If you encounter this:**
1. Check kernel messages: `dmesg | grep -i watchdog`
2. Verify device tree: `ls /proc/device-tree/watchdog*/`
3. Orange Pi H618 has hardware watchdog - may need kernel update if missing

## Integration with Argo System

### What Gets Protected

When watchdog is enabled, these hangs will auto-recover:

**Hardware Level:**
- I2C bus deadlocks (SDA/SCL stuck)
- SPI communication freezes
- GPIO interrupt storms
- DMA failures

**Kernel Level:**
- Kernel panics
- Deadlocks in kernel space
- Memory allocation failures
- Interrupt handler hangs

**System Level:**
- Systemd hangs or crashes
- Complete system freeze
- Network stack deadlock
- Filesystem hangs

**Shutdown Level:**
- Service won't stop during shutdown
- Filesystem won't unmount
- Network won't disconnect
- Init process stuck

### What Is NOT Protected

**Application Level:**
- Individual ROS2 node crashes → Use lifecycle manager
- Python script exceptions → Use try/except
- Service failures → Use systemd restart policies

**Network Level:**
- WiFi disconnection → Use network reconnection service
- SSH session drops → Use systemd journal logging
- Slow network performance → Use timeout handling

## Files Modified

### System Files
- `/etc/systemd/system.conf` - Watchdog configuration (persists across reboots)
- Backup: `/etc/systemd/system.conf.pre-watchdog`

### Argo Files
- `scripts/install_watchdog.sh` - Installation script
- `docs/README-watchdog.md` - This documentation
- `.cursor/rules/argo-watchdog-configuration.mdc` - Configuration rules
- `Makefile` - Added watchdog targets (install/status/disable/test)
- Backups: `/home/orangepi/argo_watchdog_backup_*` (created during installation)

### Services
- `watchdog.service` - Masked (conflicts with systemd watchdog, don't use)
- Systemd itself - Configured to use hardware watchdog directly

### Verification
```bash
# Check configuration file
grep "RuntimeWatchdogSec" /etc/systemd/system.conf
# Should show: RuntimeWatchdogSec=10s

# Check current status
make status-watchdog
# Should show: ✅ Watchdog ENABLED

# Check kernel logs
dmesg | grep watchdog
# Should show: sunxi-wdt ... Watchdog enabled (timeout=16 sec)
```

## Fresh SD Card Installation

To include watchdog in fresh SD card setup, add to installation procedure:

```bash
# After base system installation
cd /home/orangepi/argo

# Install Argo system
make install-all
make -C launch install
make -C power_control install

# Install watchdog (requires console access)
make install-watchdog
# Then activate: sudo systemctl daemon-reexec

# Test and verify
make status-watchdog
```

Add to `install-all` target for automatic installation:

```makefile
install-all: install-python-deps
ifeq ($(IS_ORANGEPI),1)
	@$(MAKE) install-hardware install-cpu-tuning fix-orangepi-ramlog install-network-improvements
	# Note: watchdog requires manual console testing, not auto-installed
	@echo "⚠️  To enable watchdog: make install-watchdog (requires HDMI console)"
endif
```

## Makefile Commands

```bash
make install-watchdog    # Install watchdog (requires console)
make status-watchdog     # Show watchdog status
make disable-watchdog    # Disable watchdog
make test-watchdog       # Test watchdog (causes reboot!)
```

## Best Practices

### DO:
- ✅ Have HDMI console connected before installation (REQUIRED)
- ✅ Test in safe environment first (verified working on Argo)
- ✅ Keep SSH session open during initial activation
- ✅ Monitor system for 5+ minutes after activation
- ✅ Run `make test-watchdog` to verify functionality (causes reboot)
- ✅ Check status regularly: `make status-watchdog`

### DON'T:
- ❌ Install without console access (learned from experience!)
- ❌ Set RuntimeWatchdogSec > 15s (hardware limit is 16s, we use 10s)
- ❌ Enable on untested systems without console backup
- ❌ Use watchdog daemon service (conflicts with systemd watchdog, keep it masked)
- ❌ Modify timeout without understanding hardware limitations

### Lessons Learned (December 5, 2025)

**First attempt (morning):**
- Used 60-second timeout → immediate reboot loops
- Required SD card removal to recover
- Issue: Timeout longer than 16s hardware limit

**Second attempt (afternoon):**
- Used 10-second timeout (correct)
- Had console access ready
- Test successful, system stable
- **Configuration now working 24/7**

## References

- **Hardware**: [Orange Pi Zero 2W Watchdog Documentation](http://linux-sunxi.org/Watchdog)
- **Driver**: `sunxi-wdt` in Linux kernel (confirmed working on Argo H618)
- **Systemd**: [systemd-system.conf(5)](https://www.freedesktop.org/software/systemd/man/systemd-system.conf.html)
- **Configuration Rule**: `.cursor/rules/argo-watchdog-configuration.mdc`

## Current Status Summary

**Installation Date:** December 5, 2025  
**Hardware:** Orange Pi Zero 2W (Allwinner H618)  
**Watchdog Device:** `/dev/watchdog` (sunxi-wdt)  
**Configuration:** 10-second runtime timeout, 2-minute reboot timeout  
**Status:** ✅ Active and tested  
**Test Results:** Successfully triggered and recovered from kernel panic  

**System Protection:**
- ✅ I2C bus hangs (verified need from real incident)
- ✅ Kernel panics (tested and confirmed)
- ✅ Complete system freezes
- ✅ Hung shutdown processes

**Verification Commands:**
```bash
make status-watchdog     # Check current status
dmesg | grep watchdog    # Check kernel messages
uptime                   # Verify no unexpected reboots
```

## Support

If you experience issues:

1. Check status: `make status-watchdog`
2. Review logs: `journalctl -b | grep -i watchdog`
3. Check kernel: `dmesg | grep -i watchdog`
4. Disable if needed: `make disable-watchdog`
5. Review configuration: `.cursor/rules/argo-watchdog-configuration.mdc`

For emergency recovery, see **Emergency Recovery** section above.

---

**Note:** This watchdog system has been tested and verified working on the Argo robot. The configuration persists across reboots and provides automatic recovery from hardware-level system hangs.
