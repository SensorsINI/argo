# Argo Radio Servo PWM Capture Module

A Linux kernel module for the Orange Pi Zero 2W that captures PWM signals from radio control receivers and controls servo outputs for autonomous boat navigation.

## Overview

This kernel module provides a hardware interface for autonomous boat control by:
- **Capturing PWM signals** from radio control receivers (rudder and sail inputs)
- **Controlling servo outputs** for rudder and sail actuators
- **Providing sysfs interface** for userspace applications
- **Supporting automatic/manual control switching** based on radio input

## Hardware Requirements

- **Orange Pi Zero 2W** (Allwinner H618 SoC)
- **PWM Input Pins:**
  - PI11 (Radio Rudder Input)
  - PI13 (Radio Sail Input)
- **PWM Output Pins:**
  - PI12 (Servo Rudder Output) - PWM2
  - PI14 (Servo Sail Output) - PWM4

## Features

- **Real-time PWM pulse width measurement** using GPIO interrupts
- **Hardware PWM output generation** via kernel PWM subsystem
- **Automatic human/computer control switching** based on radio input activity
- **High-resolution timing** using kernel hrtimers
- **Sysfs interface** for easy userspace integration
- **Built-in safety features** with pulse width validation

## Sysfs Interface

The module creates the following entries in `/sys/kernel/argo_radio_servo/`:

### Read-Only (Input Measurements)
- `radio_rudder_pw_us` - Current radio rudder pulse width (microseconds)
- `radio_sail_pw_us` - Current radio sail pulse width (microseconds)

### Read-Write (Servo Commands)
- `servo_rudder_pw_us` - Set/read rudder servo pulse width (1000-2000μs)
- `servo_sail_pw_us` - Set/read sail servo pulse width (1000-2000μs)

### Usage Example
```bash
# Read current radio inputs
cat /sys/kernel/argo_radio_servo/radio_rudder_pw_us
# Output: 1520

# Set servo outputs
echo 1600 | sudo tee /sys/kernel/argo_radio_servo/servo_rudder_pw_us
echo 1400 | sudo tee /sys/kernel/argo_radio_servo/servo_sail_pw_us
```

## Installation

### Prerequisites

1. **Orange Pi Zero 2W** running Armbian with kernel 6.1.31-sun50iw9
2. **Kernel headers** installed:
   ```bash
   sudo apt update
   sudo apt install linux-headers-$(uname -r)
   ```
3. **Build tools**:
   ```bash
   sudo apt install build-essential device-tree-compiler
   ```

### Step 1: Enable Required Overlays

Edit `/boot/orangepiEnv.txt` and ensure these overlays are enabled:
```
overlays=pi-i2c0 disable-uart0 ph-uart5 pi-pwm2 pi-pwm4 
user_overlays=argo_radio_servo_overlay 
```

### Step 2: Build and Install the Module

```bash
# Navigate to the module directory
cd /path/to/argo/scripts/pwm_capture_module

# Build the kernel module
make

# Build the device tree overlay
make overlay

# Install the overlay
sudo cp argo_radio_servo_overlay.dtbo /boot/overlay-user/

# Install the kernel module
sudo cp argo_radio_servo_module.ko /lib/modules/$(uname -r)/kernel/drivers/misc/argo/
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/misc/argo/
sudo depmod -a

# Install udev rules for permissions
sudo cp 99-argo-radio-servo.rules /etc/udev/rules.d/

# Install systemd service for post-init setup
sudo cp argo-radio-servo-perms.service /etc/systemd/system/
sudo systemctl enable argo-radio-servo-perms.service
```

### Step 3: Load the Module

```bash
# Load the module manually
sudo modprobe argo_radio_servo_module

# Or add to automatic loading
echo "argo_radio_servo_module" | sudo tee -a /etc/modules
```

### Step 4: Reboot and Verify

```bash
sudo reboot

# After reboot, check that the module is loaded
lsmod | grep argo

# Verify sysfs entries exist
ls -la /sys/kernel/argo_radio_servo/

# Check kernel messages
dmesg | grep argo
```

## How It Works

### Device Tree Integration

The module uses a device tree overlay (`argo_radio_servo_overlay.dts`) that:
- Creates a platform device with compatible string `"argo,radio-servo-gpio"`
- Maps GPIO pins PI11 and PI13 for radio input capture
- References PWM channels 2 and 4 for servo output control
- Disables Ethernet GMAC to free up pins

### Kernel Module Architecture

1. **Platform Driver**: Registers with the kernel using the device tree compatible string
2. **GPIO Interrupt Handling**: Captures rising/falling edges on input pins to measure pulse widths
3. **PWM Output Control**: Uses kernel PWM subsystem to generate precise servo control signals
4. **High-Resolution Timer**: Provides periodic status updates and measurements
5. **Sysfs Interface**: Exposes measurements and controls to userspace

### Signal Processing

- **Input Capture**: GPIO interrupts measure time between rising and falling edges
- **Debouncing**: Invalid pulse widths (outside 500-2500μs range) are filtered
- **Output Generation**: PWM subsystem generates 50Hz signals with precise duty cycles
- **Safety**: Default 1500μs (neutral) position on startup

## Troubleshooting

### Module Won't Load
```bash
# Check kernel messages
dmesg | grep argo

# Common issues:
# - Missing device tree overlay
# - PWM overlays not enabled
# - Pin conflicts with other drivers
```

### No Sysfs Entries
```bash
# Verify platform device exists
find /sys -name "*argo*"

# Check driver binding
ls -la /sys/bus/platform/drivers/argo_radio_servo/

# Verify overlays are applied
cat /boot/orangepiEnv.txt
```

### Permission Denied
```bash
# Check udev rules
cat /etc/udev/rules.d/99-argo-radio-servo.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Pin Conflicts
```bash
# Check for pin conflicts
dmesg | grep -i "pin.*already requested"

# Ensure no conflicting overlays are enabled
# Remove any pinctrl configurations for PI11/PI13
```

## Development

### Building from Source
```bash
# Clean build
make clean
make

# Rebuild overlay
make overlay

# Install and test
sudo rmmod argo_radio_servo_module 2>/dev/null || true
sudo cp argo_radio_servo_overlay.dtbo /boot/overlay-user/
sudo insmod argo_radio_servo_module.ko
```

### Debugging
```bash
# Enable debug messages
echo 8 | sudo tee /proc/sys/kernel/printk

# Monitor real-time messages
sudo dmesg -w | grep argo
```

## Integration with ROS2

The module is designed to work with the ROS2 PWM node (`pwm.py`) which:
- Reads radio inputs from sysfs
- Publishes radio control values as ROS2 topics
- Receives autonomous control commands
- Automatically switches between manual/autonomous control

## Version History

- **v0.4** - Fixed device tree overlay structure and PWM references
- **v0.3** - Added proper GPIO handling and interrupt-based capture
- **v0.2** - Initial PWM output support
- **v0.1** - Basic platform driver framework

## License

GPL v2 - See kernel module source for full license text.

## Author

Tobi Delbruck - Autonomous boat navigation system for Orange Pi Zero 2W
