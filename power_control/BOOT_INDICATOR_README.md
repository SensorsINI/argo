# Argo Boot Indicator Service

## Overview

The Argo Boot Indicator Service provides continuous visual feedback during system boot by showing an early heartbeat pattern on the GREEN LED. This addresses the ~45-second delay before the main power control service can start its heartbeat pattern.

## Purpose

- **Immediate Boot Feedback**: Initial 3-flash sequence starts at ~15 seconds after power-on
- **Continuous Heartbeat**: Continues with 1Hz heartbeat pattern until main power control takes over
- **Seamless Handoff**: Systemd coordinates clean GPIO handoff when main service starts
- **User Confidence**: Provides visual confirmation that boot is proceeding normally throughout the entire boot sequence

## How It Works

1. **Boot starts** → ~15 seconds of system initialization
2. **Boot indicator service starts** → 3 quick flashes, then continuous 1Hz heartbeat
3. **Main power control service starts** (~30 seconds later) → systemd stops boot indicator
4. **GPIO handoff** → Boot indicator releases GPIO cleanly via SIGTERM handler
5. **Main heartbeat continues** → Power control service takes over LED control

This ensures the GREEN LED is always showing activity from ~15 seconds until full system ready.

## Components

### Files

1. **`argo_boot_indicator.py`** - Python script with continuous heartbeat until terminated
2. **`argo_boot_indicator.service`** - Systemd simple service that conflicts with main power control
3. **`Makefile`** - Installation target: `install_boot_indicator`

### Hardware

- **Green LED**: PH4 (Pin 18, GPIO 228)
- **Polarity**: Active LOW (GPIO LOW = LED ON)
- **Initial Pattern**: 3 quick flashes (0.15s on, 0.15s off)
- **Heartbeat Pattern**: 1Hz continuous (50% duty cycle) until terminated

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

Test the boot indicator heartbeat pattern without rebooting:

```bash
# Stop power control service first to release GPIO
sudo systemctl stop argo_power_control.service

# Run boot indicator script (runs continuously until stopped)
sudo python3 /home/orangepi/argo/power_control/argo_boot_indicator.py
```

Expected behavior:
- GREEN LED flashes 3 times quickly (initial boot indication)
- GREEN LED continues with 1Hz heartbeat pattern
- Script runs continuously until you press Ctrl+C
- GPIO is released cleanly on exit with proper cleanup messages

### Service Test

Test the systemd service and automatic handoff:

```bash
# Stop main power control service
sudo systemctl stop argo_power_control.service

# Start boot indicator service
sudo systemctl start argo_boot_indicator.service

# Verify it's running with heartbeat visible on LED
sudo systemctl status argo_boot_indicator.service
```

You should see the GREEN LED in continuous heartbeat mode.

Now test the automatic handoff:

```bash
# Start main power control (should stop boot indicator automatically via Conflicts)
sudo systemctl start argo_power_control.service

# Verify handoff occurred
sudo systemctl status argo_boot_indicator.service  # Should show "inactive (dead)"
sudo systemctl status argo_power_control.service   # Should show "active (running)"
```

View service logs:

```bash
# Boot indicator logs
sudo journalctl -u argo_boot_indicator.service -n 50

# Look for "Received signal 15" (SIGTERM) and "GPIO line released"
```

### Boot Test

Reboot the system and observe the GREEN LED during boot:

```bash
sudo reboot
```

**Expected behavior:**
1. Power button pressed → System powers on
2. **~15 seconds**: GREEN LED flashes 3 times (boot indicator starts)
3. **GREEN LED continues heartbeat** (boot indicator 1Hz heartbeat)
4. **~45 seconds total**: Main power control takes over, heartbeat continues seamlessly
5. Heartbeat frequency changes based on Argo service state (1Hz/2Hz/recording pattern)

## Timing Details

### Boot Sequence with Continuous Heartbeat

```
Time 0s:      Power button pressed, system powers on
Time 0-15s:   System initialization (no LED activity)
Time 15s:     Boot indicator service starts
              - 3 quick flashes to indicate boot progress
              - Continuous 1Hz heartbeat begins
Time 15-45s:  Boot indicator heartbeat continues
              - ROS2 and other services starting in background
              - Visual confirmation that boot is proceeding
Time 45s:     Main power control service starts
              - Systemd sends SIGTERM to boot indicator
              - Boot indicator releases GPIO cleanly
              - Main power control takes over LED immediately
Time 45s+:    Main power control heartbeat
              - Frequency varies with system state
              - No gap in LED activity
```

### LED Patterns

#### Initial Flash Pattern
- **Flash count**: 3 flashes
- **Flash on time**: 0.15 seconds
- **Flash off time**: 0.15 seconds
- **Total duration**: ~1 second

#### Continuous Heartbeat Pattern
- **Frequency**: 1Hz (matching normal power control)
- **Duty cycle**: 50% (0.5s on, 0.5s off)
- **Duration**: Until main power control starts (~30 seconds)

## Service Configuration

### Systemd Service Properties

```ini
[Unit]
Description=Argo Boot Indicator - Early boot LED heartbeat
DefaultDependencies=no
Before=sysinit.target shutdown.target argo_power_control.service
Conflicts=shutdown.target argo_power_control.service

[Service]
Type=simple
User=orangepi
Group=orangepi
ExecStart=/usr/bin/python3 /home/orangepi/argo/power_control/argo_boot_indicator.py
TimeoutStopSec=5
KillMode=mixed
KillSignal=SIGTERM
Restart=no

[Install]
WantedBy=sysinit.target
```

### Key Features

- **`Type=simple`**: Runs continuously until terminated
- **`DefaultDependencies=no`**: Minimal dependencies for early boot
- **`Before=argo_power_control.service`**: Ensures boot indicator starts first
- **`Conflicts=argo_power_control.service`**: Systemd stops boot indicator when main service starts
- **`User=orangepi`**: Same user as main power control for consistency
- **`KillSignal=SIGTERM`**: Graceful termination for proper GPIO cleanup
- **`TimeoutStopSec=5`**: Allow time for GPIO release on termination
- **`WantedBy=sysinit.target`**: Installed in early boot target

### Service Coordination

The handoff between boot indicator and main power control is coordinated by systemd:

1. Boot indicator starts early at `sysinit.target`
2. When main power control starts, systemd sees the `Conflicts` directive
3. Systemd sends SIGTERM to boot indicator process
4. Boot indicator's signal handler catches SIGTERM
5. Boot indicator releases GPIO cleanly and exits
6. Main power control can now claim GPIO without conflict

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



