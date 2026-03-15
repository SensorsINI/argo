# Argo Power Control System

Intelligent power management system for Orange Pi Zero 2W with battery monitoring, safe shutdown, and emergency power preservation for manual sailing.

## Architecture Overview

The power control system consists of three main components working together:

### 1. **argo_power_control.py** - Main Control Daemon
ROS2 service that runs continuously and handles:
- **Hardware interrupt-based button monitoring** (double tap, triple tap, long press)
- **LED indicators** (heartbeat, charging status, battery warnings)
- **Battery voltage monitoring** (low battery SOS, critical battery halt)
- **Argo service control** (start/stop via double tap)
- **Recording control** (start/stop via triple tap)
- **Safe shutdown initiation** (long button press)
- **Critical battery management** (automatic halt with user confirmation)

### 2. **argo_poweroff.shutdown** - Late Shutdown Hook
systemd shutdown hook that runs during final shutdown phase:
- **Normal shutdown**: Pulses GPIO 259 HIGH to reset latching relay → cuts power
- **Critical battery halt**: Detects `/tmp/argo_critical_battery` flag → preserves power relay
- **Reboot protection**: Skips power cut during reboot operations
- **Runs after all services stop** for safe power control

### 3. **argo_power_control.service** - systemd Service
Manages the power control daemon lifecycle:
- Starts at boot with proper dependencies
- Runs as root for GPIO and shutdown control
- Restarts automatically on failure
- Sources ROS2 environment for service calls

### 4. **argo_boot_indicator.py** - Early Boot LED Indicator (Optional)
One-shot systemd service that provides early visual feedback during boot:
- **Runs at sysinit.target**: Very early in boot sequence (~15 seconds after power-on)
- **Brief GREEN LED flashing**: 5 quick flashes to indicate boot is proceeding
- **Releases GPIO immediately**: Ensures power control service can claim GPIO cleanly
- **Purpose**: Provides visual feedback during the ~30-45 second boot delay before main heartbeat starts

## Boot Timing Observations

On Orange Pi Zero 2W, the boot sequence timing is:
1. **~15 seconds**: System initialization, boot indicator flashes GREEN LED (if enabled)
2. **~30-45 seconds additional delay**: ROS2 services and dependencies starting
3. **~45 seconds total**: Main power control heartbeat becomes visible

The boot indicator service provides early feedback during this long boot sequence, reassuring users that the system is booting properly even though the main heartbeat takes nearly a minute to appear.

## Hang diagnosis (no logs, LEDs not updating)

If the service shows as running but there is no ongoing log output and power-button LEDs do not update:

1. **Run the diagnostic script** (uses `/proc` to sample syscalls; no strace required):
   ```bash
   bash power_control/diagnose_power_control_hang.sh
   ```
   With sudo you get full syscall info; without sudo you still get tee state and instructions.

2. **Trace the process** (recommended to see where it blocks):
   ```bash
   sudo apt install -y strace
   PID=$(pgrep -f "python3.*argo_power_control\.py" | head -1)
   # Use readlink to get the Python PID, not bash: for p in $(pgrep -f "argo_power_control\.py"); do readlink -f /proc/$p/exe 2>/dev/null | grep -q python && PID=$p && break; done
   sudo strace -p $PID -f 2>&1
   ```
   Run for 10–15 seconds then Ctrl+C. Look for threads stuck in the same syscall (e.g. `futex`, `epoll_pwait`, or `read`/`write` on a fd).

3. **Interpretation**: Tee in `read` means the pipe is empty—Python is not writing. So the process is stuck before producing log output (e.g. in a callback or in `_check_network_status` / WiFi check). If you temporarily remove `|& tee -a ...` from the service `ExecStart` and log only to the journal, you can confirm whether the problem is the pipeline or the Python process.

## Hardware Configuration (Rev3 PCB)

### GPIO Pin Assignments
- **PI3 (pin 40, GPIO 259)**: `POW_OFF` - Output pulse to relay RESET coil (active HIGH)
- **PH0 (pin 8, GPIO 224)**: `POW_BUT` - Power button input (active HIGH when pressed)
- **PH4 (pin 18, GPIO 228)**: Green LED in power button (ACTIVE_HIGH - NFET control: HIGH = ON, LOW = OFF)
- **PI1 (pin 12, GPIO 257)**: Blue LED in power button (active LOW - cathode control)
- **PI16 (pin 37, GPIO 272)**: Red LED for SOS warnings (active LOW - cathode control)
- **PC12 (pin 36, GPIO 76)**: `!CHARGING` from MP2672GD (LOW = charging active)
- **PH9 (pin 26, GPIO 233)**: `!ACOK` from MP2672GD (LOW = AC power present)

## Power Circuit Design

### Latching Relay System (PRL3-5V-DC-1A)
- **SET coil**: Activated by power button press → latches relay ON
- **RESET coil**: Activated by GPIO 259 HIGH pulse → unlatches relay OFF
- Magnetic latching maintains state without continuous power

### Power Distribution
1. **Battery** (2S LiPo, ~8.4V fully charged) → **7A Fuse** → **Latching Relay**
2. **Relay Output (MAIN)** powers:
   - **5V Traco switching regulator** → powers OrangePi Zero 2W and sensors
   - **Servos and radio** (via servo cables) for manual control
3. **OrangePi 3.3V regulator** powers:
   - SHT5-ADB1B humidity sensor
   - RA-01 SX1278 LoRa radio

### Power States
- **Relay ON**: Full system operation (computer + sensors + servos)
- **Relay OFF**: Minimal battery draw (~40µA from MP2672GD charger only)
- **Computer halted, Relay ON**: Radio control active for manual sailing

## Critical Battery Management

The system implements two-tier battery protection:

### 1. Low Battery Warning (< 7.6V)
- **Red LED SOS pattern** starts flashing
- **Desktop notification** warns user
- **System continues operating** but alerts are active
- User can return to shore or plug in charger

### 2. Critical Battery Halt (< 7.2V)

#### Production Mode (Default: `CRITICAL_BATTERY_USE_SHUTDOWN = False`)
**Purpose**: Preserve battery for manual sailing when computer can't operate

**Sequence**:
1. Battery monitoring detects voltage < 7.2V
2. Shows **30-second confirmation dialog** (timeout = automatic halt)
3. Pauses all sensor nodes to minimize power consumption
4. **Creates `/tmp/argo_critical_battery` flag file**
5. Executes `sudo halt` command
6. **Shutdown hook detects flag** → **PRESERVES power relay**
7. Computer halts, but servos remain powered for **manual radio control**

**Result**: Radio control active, PWM pins in high-impedance mode, ~50mA power draw vs ~500mA

#### Development Mode (`CRITICAL_BATTERY_USE_SHUTDOWN = True`)
**Purpose**: Normal shutdown for testing when you want full power cut

**Sequence**:
1. Battery monitoring detects voltage < 7.2V
2. Shows **30-second confirmation dialog**
3. Pauses all sensor nodes
4. **Does NOT create flag file**
5. Executes `shutdown -h now` command
6. **Shutdown hook runs normally** → **CUTS power relay**
7. Complete system shutdown with minimal battery drain

**Configuration**: Edit line ~206 in `argo_power_control.py`:
```python
CRITICAL_BATTERY_USE_SHUTDOWN = False  # Production: preserve power
CRITICAL_BATTERY_USE_SHUTDOWN = True   # Development: cut power
```

### Critical Battery Flag Mechanism
- **Flag file**: `/tmp/argo_critical_battery`
- **Created by**: `argo_power_control.py` when critical battery detected (production mode only)
- **Read by**: `argo_poweroff.shutdown` during shutdown sequence
- **Effect**: If flag exists → skip power cut, preserve relay for manual sailing 

## User Interface

### Power Button Behavior (Active HIGH)
- **Double tap** (2 quick presses within 3s): Toggle Argo service (start/stop all ROS2 nodes)
- **Triple tap** (3 quick presses within 3s): Toggle recording (start/stop rosbag recording)
- **Long press** (≥5 seconds): Initiate graceful system shutdown with power cut
- **Warning threshold** (3 seconds): Desktop notification warns to release button to cancel

### LED Indicators (Active LOW - Common Anode RGB)

#### Green LED (System Heartbeat)
- **1Hz pulse**: Normal operation, Argo service stopped
- **2Hz pulse**: Argo service running
- **3-flash pattern**: Recording active (3 quick flashes, then pause)
- **Paused**: Low or critical battery warning active

#### Blue LED (Charging Status)
- **ON**: Battery charging (charger connected and active)
- **OFF**: Not charging or AC power absent

#### Red LED (Battery Warnings)
- **Slow SOS pattern**: Low battery warning (< 7.6V)
- **Continuous during button press**: Combined with green for visual feedback

#### Both LEDs During Button Press
- **Gradual frequency increase**: 2Hz → 20Hz over 5-second threshold
- **Shows progress** toward shutdown threshold

#### Both LEDs During Shutdown
- **1Hz, 5% duty cycle**: Short blink pattern indicating shutdown in progress

## Main Files

### argo_power_control.py (~2800 lines)
**Primary control daemon with:**
- ROS2 service client for Argo control (`/argo/recording/*` services)
- Battery monitoring thread (30-second polling interval)
- Hardware interrupt-based button monitoring (efficient, low CPU)
- LED control threads (heartbeat, SOS patterns, shutdown indicators)
- Critical battery confirmation dialogs with timeout
- Desktop notification system for user feedback

**Key configuration constants** (lines 190-210):
```python
LOW_BATTERY_THRESHOLD_V = 7.6           # SOS warning threshold
CRITICAL_BATTERY_THRESHOLD_V = 7.2      # Halt threshold
CRITICAL_BATTERY_USE_SHUTDOWN = False   # Development flag
```

### argo_poweroff.shutdown (~134 lines)
**Late systemd shutdown hook:**
- Invoked during `system-shutdown` phase (after all services stopped)
- Checks `/tmp/argo_critical_battery` flag
- Mode detection: poweroff/halt vs reboot
- GPIO 259 pulse using Python gpiod or gpioset fallback
- Console and kernel log messages for visibility

### argo_power_control.service
**systemd service configuration:**
```ini
[Unit]
Description=Argo Power Control Service
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 ~/argo/power_control/argo_power_control.py'
Restart=always
User=root

[Install]
WantedBy=multi-user.target
```

### Makefile
**Installation and management targets:**
- `make install_service`: Install systemd service
- `make install_shutdown_hook`: Install late shutdown hook
- `make check_deps`: Verify python3-libgpiod installed
- `make clean_power_control`: Remove all components

## Important: Why not gpio-poweroff overlay on Zero 2W?
- The Orange Pi Zero 2W uses an AXP PMIC that registers `pm_power_off` first during boot.
- When `pm_power_off` is already registered, the kernel `gpio-poweroff` driver refuses to bind and logs:
  - `poweroff-gpio: gpio_poweroff_probe: pm_power_off function already registered`
- Result: any `gpio-poweroff` DT node (including overlays) will not take effect.
- Solution: a systemd late shutdown hook claims PI3 and pulses it HIGH just before poweroff. This avoids conflicts and is reliably last.
- The shutdown hook intelligently differentiates between shutdown modes:
  - **Poweroff/Halt**: Triggers power cut sequence (PI3 HIGH pulse)
  - **Reboot**: Skips power cut to allow normal restart

## Prerequisites
```bash
# Install required packages
sudo apt-get update
sudo apt-get install python3-libgpiod libgpiod-tools python3-rclpy

# Add user to gpio group (if not running as root)
sudo usermod -a -G gpio $USER
# Then log out and back in
```

## Installation

### Full Installation (Recommended)
```bash
cd ~/argo/power_control

# Check dependencies
make check_deps

# Install and enable systemd service
make install_service enable_service

# Install shutdown hook
make install_shutdown_hook

# Verify installation
sudo systemctl status argo_power_control.service
```

### Manual Installation Steps
```bash
# 1. Install service
sudo cp argo_power_control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable argo_power_control.service
sudo systemctl start argo_power_control.service

# 2. Install shutdown hook
sudo cp argo_poweroff.shutdown /lib/systemd/system-shutdown/argo_poweroff
sudo chmod +x /lib/systemd/system-shutdown/argo_poweroff
```

## Testing

### Test Shutdown Hook (Safe - No Actual Shutdown)
```bash
# Test poweroff mode (should show power cut sequence)
sudo bash -x /lib/systemd/system-shutdown/argo_poweroff poweroff

# Test reboot mode (should skip power cut)
sudo bash -x /lib/systemd/system-shutdown/argo_poweroff reboot

# Test critical battery mode (should preserve power)
echo "TEST" | sudo tee /tmp/argo_critical_battery
sudo bash -x /lib/systemd/system-shutdown/argo_poweroff poweroff
sudo rm /tmp/argo_critical_battery
```

### Test Power Control Daemon
```bash
# Test mode (safe - simulates actions without executing)
sudo python3 argo_power_control.py --test-mode

# Test wall message (safe)
sudo python3 argo_power_control.py --test-wall-message

# Test desktop notifications (safe)
python3 argo_power_control.py --test-notification

# Simulate double tap (Argo service toggle)
sudo python3 argo_power_control.py --simulate-double-tap

# Simulate triple tap (recording toggle)
sudo python3 argo_power_control.py --simulate-triple-tap

# Simulate critical battery (shows confirmation dialog)
sudo python3 argo_power_control.py --simulate-critical-battery

# Simulate low battery (SOS LED pattern)
sudo python3 argo_power_control.py --simulate-low-battery
```

### Test Actual Shutdown (Requires Hardware)
```bash
# Normal shutdown (should cut power after ~3 seconds)
sync
sudo systemctl poweroff
# Watch GPIO 259 - should go HIGH briefly before power cuts

# Reboot test (should NOT cut power)
sync
sudo systemctl reboot
# GPIO 259 should remain LOW, system should reboot normally
```

## Usage

### Command Line Options
```bash
# Show help
python3 argo_power_control.py --help

# Normal operation (production)
sudo python3 argo_power_control.py

# Test mode (safe for development - no actual shutdown)
sudo python3 argo_power_control.py --test-mode

# Custom shutdown threshold
sudo python3 argo_power_control.py --threshold 3.0

# All options combined
sudo python3 argo_power_control.py --test-mode --threshold 2.0
```

### Service Management
```bash
# Check service status
sudo systemctl status argo_power_control.service

# View service logs
sudo journalctl -u argo_power_control.service -f

# Restart service
sudo systemctl restart argo_power_control.service

# Stop service
sudo systemctl stop argo_power_control.service

# Disable service
sudo systemctl disable argo_power_control.service
```

### Development Mode Configuration
To use normal shutdown (cut power) instead of halt (preserve power) on critical battery:

```bash
# Edit argo_power_control.py around line 206
CRITICAL_BATTERY_USE_SHUTDOWN = True   # Development: cuts power
CRITICAL_BATTERY_USE_SHUTDOWN = False  # Production: preserves power (default)

# Then restart service
sudo systemctl restart argo_power_control.service
```

## Troubleshooting

### Service Won't Start
```bash
# Check for errors
sudo journalctl -u argo_power_control.service -n 50

# Common issues:
# 1. ROS2 not sourced - check service ExecStart
# 2. GPIO permissions - check user is root or in gpio group
# 3. python3-libgpiod not installed - run make check_deps
```

### Button Presses Not Detected
```bash
# Check GPIO line status
gpioinfo | grep -E "(259|224|228|257|272)"

# Test button manually
gpioget gpiochip0 224  # Should be 0 when released, 1 when pressed

# Check service is running
sudo systemctl status argo_power_control.service
```

### LEDs Not Working
```bash
# Test LEDs manually
# Green LED (line 228, active LOW - 0=ON, 1=OFF)
gpioset gpiochip0 228=0  # Turn ON
gpioset gpiochip0 228=1  # Turn OFF

# Blue LED (line 257, active LOW)
gpioset gpiochip0 257=0  # Turn ON
gpioset gpiochip0 257=1  # Turn OFF

# Red LED (line 272, active LOW)
gpioset gpiochip0 272=0  # Turn ON
gpioset gpiochip0 272=1  # Turn OFF
```

### Power Not Cutting on Shutdown
```bash
# Check shutdown hook is installed
ls -la /lib/systemd/system-shutdown/argo_poweroff

# Check hook is executable
sudo chmod +x /lib/systemd/system-shutdown/argo_poweroff

# Test hook manually (safe - won't shutdown)
sudo bash -x /lib/systemd/system-shutdown/argo_poweroff poweroff

# Check GPIO 259 can be controlled
gpioset gpiochip0 259=1  # Should pulse relay reset coil
gpioset gpiochip0 259=0
```

### Critical Battery Flag Not Working
```bash
# Check flag file location
ls -la /tmp/argo_critical_battery

# Manually test flag mechanism
echo "TEST" | sudo tee /tmp/argo_critical_battery
sudo bash -x /lib/systemd/system-shutdown/argo_poweroff poweroff
# Should see message about preserving power

# Clean up test
sudo rm /tmp/argo_critical_battery
```

### ROS2 Service Calls Failing
```bash
# Check if Argo service is running
sudo systemctl status argo_launch_standard.service

# Test recording services manually
ros2 service list | grep recording
ros2 service call /argo/recording/start std_srvs/srv/Trigger

# Check ROS2 environment in power control service
sudo journalctl -u argo_power_control.service | grep ROS
```

## GPIO Quick Reference (H616)

| Function | Pin | GPIO Line | Direction | Active |
|----------|-----|-----------|-----------|--------|
| POW_OFF (relay reset) | 40 | 259 (PI3) | Output | HIGH |
| POW_BUT (power button) | 8 | 224 (PH0) | Input | HIGH |
| Green LED | 18 | 228 (PH4) | Output | LOW |
| Blue LED | 12 | 257 (PI1) | Output | LOW |
| Red LED | 37 | 272 (PI16) | Output | LOW |
| !CHARGING | 36 | 76 (PC12) | Input | LOW |
| !ACOK | 26 | 233 (PH9) | Input | LOW |

## Uninstall

### Full Uninstall
```bash
cd ~/argo/power_control

# Use Makefile target
make clean_power_control

# Or manually:
sudo systemctl stop argo_power_control.service
sudo systemctl disable argo_power_control.service
sudo rm /etc/systemd/system/argo_power_control.service
sudo rm /lib/systemd/system-shutdown/argo_poweroff
sudo systemctl daemon-reload
```

### Temporary Disable (Keep Files)
```bash
# Just stop and disable the service
sudo systemctl stop argo_power_control.service
sudo systemctl disable argo_power_control.service

# Re-enable later
sudo systemctl enable argo_power_control.service
sudo systemctl start argo_power_control.service
```

## References

- **ARGO_POWER_CONTROL_README.md**: Detailed implementation documentation
- **CRITICAL_BATTERY_PAUSE.md**: Critical battery pause mechanism details
- **Makefile**: All installation/management targets
