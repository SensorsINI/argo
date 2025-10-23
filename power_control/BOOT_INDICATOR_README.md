# Argo Boot Indicator Service

## Overview

The Argo Boot Indicator Service provides immediate visual feedback during system boot by briefly flashing the GREEN LED in the power button. This addresses the ~30-second delay before the main power control service starts its heartbeat pattern.

## Purpose

- **Immediate Boot Feedback**: Flashes the GREEN LED 3 times during early boot to indicate the system is booting
- **Early Boot Execution**: Runs as a one-shot service very early in the boot sequence (`sysinit.target`)
- **Non-Blocking**: Briefly claims the GPIO, flashes the LED, then releases it for the main power control service
- **User Confidence**: Provides visual confirmation that boot is proceeding normally

## Components

### Files

1. **`argo_boot_indicator.py`** - Python script that flashes the GREEN LED
2. **`argo_boot_indicator.service`** - Systemd one-shot service configuration
3. **`Makefile`** - Installation target: `install_boot_indicator`

### Hardware

- **Green LED**: PH4 (Pin 18, GPIO 228)
- **Polarity**: Active LOW (GPIO LOW = LED ON)
- **Flash Pattern**: 3 quick flashes (0.15s on, 0.15s off)

## Installation

The boot indicator service is automatically installed with the power control system:

```bash
cd /home/orangepi/argo/power_control
make install
```

To install only the boot indicator service:

```bash
make install_boot_indicator
```

This will:
1. Make the Python script executable
2. Install the systemd service to `/etc/systemd/system/`
3. Enable the service for automatic startup
4. Reload systemd daemon

## Testing

### Manual Test

Test the boot indicator LED flash pattern without rebooting:

```bash
sudo python3 /home/orangepi/argo/power_control/argo_boot_indicator.py
```

You should see the GREEN LED flash 3 times quickly.

### Service Test

Test the systemd service:

```bash
sudo systemctl start argo_boot_indicator.service
```

Check service status:

```bash
sudo systemctl status argo_boot_indicator.service
```

View service logs:

```bash
sudo journalctl -u argo_boot_indicator.service
```

### Boot Test

Reboot the system and observe the GREEN LED during boot:

```bash
sudo reboot
```

**Expected behavior:**
1. Power button pressed → System powers on
2. **GREEN LED flashes 3 times** (boot indicator - ~1-5 seconds after power on)
3. 25-30 seconds of no LED activity (system booting)
4. GREEN LED heartbeat starts (main power control service started)

## Timing Details

### Boot Sequence

```
Time 0s:     Power button pressed, system powers on
Time 1-5s:   Boot indicator flashes GREEN LED (3 flashes)
Time 5-30s:  System continues booting (no LED activity)
Time 30s+:   Main power control service starts heartbeat
```

### Flash Pattern

- **Flash count**: 3 flashes
- **Flash on time**: 0.15 seconds
- **Flash off time**: 0.15 seconds
- **Total duration**: ~1 second
- **Final delay**: 0.5 seconds before GPIO release

## Service Configuration

### Systemd Service Properties

```ini
[Unit]
Description=Argo Boot Indicator - Early boot LED flash
DefaultDependencies=no
Before=sysinit.target shutdown.target
Conflicts=shutdown.target

[Service]
Type=oneshot
User=root
ExecStart=/usr/bin/python3 /home/orangepi/argo/power_control/argo_boot_indicator.py
TimeoutStartSec=5
Restart=no

[Install]
WantedBy=sysinit.target
```

### Key Features

- **`Type=oneshot`**: Runs once and exits
- **`DefaultDependencies=no`**: Minimal dependencies for early boot
- **`Before=sysinit.target`**: Runs very early in boot sequence
- **`User=root`**: Required for GPIO access during early boot
- **`TimeoutStartSec=5`**: Fast timeout (should complete in ~1 second)
- **`WantedBy=sysinit.target`**: Installed in early boot target

## GPIO Safety

### GPIO Claiming and Release

The boot indicator service:
1. Claims the GREEN LED GPIO (line 228)
2. Flashes the LED 3 times
3. **Releases the GPIO** before exiting

This ensures the main power control service can claim the same GPIO when it starts later.

### No Conflicts

- **Early boot**: Boot indicator service runs first (sysinit.target)
- **Main service**: Power control service runs later (multi-user.target)
- **GPIO release**: Boot indicator releases GPIO before main service starts
- **No overlap**: Services never run simultaneously

## Troubleshooting

### LED doesn't flash during boot

1. Check service is enabled:
   ```bash
   sudo systemctl is-enabled argo_boot_indicator.service
   ```

2. Check service status after boot:
   ```bash
   sudo systemctl status argo_boot_indicator.service
   ```

3. View service logs:
   ```bash
   sudo journalctl -u argo_boot_indicator.service
   ```

### Permission errors

The service must run as root for GPIO access during early boot. Check the service file:

```bash
grep "^User=" /etc/systemd/system/argo_boot_indicator.service
```

Should show: `User=root`

### GPIO conflicts

If the boot indicator and main service conflict:

1. Check both services are not running simultaneously:
   ```bash
   sudo systemctl status argo_boot_indicator.service argo_power_control.service
   ```

2. Verify boot indicator is oneshot (exits after flashing):
   ```bash
   systemctl show argo_boot_indicator.service | grep Type
   ```

## Uninstallation

To remove the boot indicator service:

```bash
cd /home/orangepi/argo/power_control
make clean_power_control
```

Or manually:

```bash
sudo systemctl stop argo_boot_indicator.service
sudo systemctl disable argo_boot_indicator.service
sudo rm /etc/systemd/system/argo_boot_indicator.service
sudo systemctl daemon-reload
```

## Customization

### Changing Flash Pattern

Edit `argo_boot_indicator.py` and modify these constants:

```python
FLASH_COUNT = 3           # Number of flashes
FLASH_ON_TIME = 0.15      # LED on time (seconds)
FLASH_OFF_TIME = 0.15     # LED off time (seconds)
FINAL_DELAY = 0.5         # Delay before GPIO release (seconds)
```

After changes:

```bash
cd /home/orangepi/argo/power_control
make install_boot_indicator
```

### Changing Boot Timing

To run the boot indicator earlier or later, edit `argo_boot_indicator.service`:

- **Earlier**: Change `WantedBy=sysinit.target` to `WantedBy=local-fs.target`
- **Later**: Change to `WantedBy=multi-user.target`

## Integration with Main Power Control

The boot indicator service is designed to work seamlessly with the main power control service:

1. **Boot indicator** (early boot):
   - Flashes GREEN LED 3 times
   - Releases GPIO
   - Exits

2. **System continues booting** (25-30 seconds)

3. **Main power control** (after boot):
   - Claims GREEN LED GPIO
   - Starts heartbeat pattern
   - Continues running until shutdown

## Benefits

- **Immediate feedback**: User knows system is booting within 1-5 seconds
- **No waiting**: Don't wait 30 seconds to confirm boot started
- **Diagnostic aid**: Helps identify if system is booting vs. stuck before boot
- **Professional appearance**: System appears more responsive
- **Safety**: GPIO properly released before main service starts

## Technical Notes

### Why One-Shot Service?

The boot indicator uses a one-shot service instead of a persistent service because:
- Only needs to flash LED once during boot
- Must release GPIO for main service
- Reduces resource usage
- Simpler lifecycle management

### Why Run as Root?

Early boot services require root access because:
- GPIO group permissions not yet established
- Udev rules not yet applied
- Service runs before user sessions
- Minimal system initialization at sysinit.target

### Boot Timeline

```
Kernel boot → initrd → sysinit.target → multi-user.target
                           ↑                    ↑
                     Boot indicator      Power control
                     (1 second)          (starts ~30s later)
```

## See Also

- [argo_power_control.py](argo_power_control.py) - Main power control service
- [argo_power_control.service](argo_power_control.service) - Main service configuration
- [Makefile](Makefile) - Installation and management targets
- [README.md](README.md) - Main power control documentation



