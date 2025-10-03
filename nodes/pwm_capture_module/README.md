# Argo Radio Servo PWM Capture Module

A Linux kernel module for the Orange Pi Zero 2W that captures PWM signals from radio control receivers and controls servo outputs for autonomous boat navigation with high impedance safety mode.

## Overview

This kernel module provides a hardware interface for autonomous boat control by:
- **Capturing PWM signals** from radio control receivers (rudder and sail inputs)
- **Controlling servo outputs** for rudder and sail actuators with high impedance safety mode
- **Providing sysfs interface** for userspace applications
- **Supporting automatic/manual control switching** based on radio input
- **High impedance mode** allows radio control to pass through directly to servos when PWM is disabled

## Hardware Requirements

- **Orange Pi Zero 2W** (Allwinner H618 SoC)
- **PWM Input Pins:**
  - PI11 (Pin 7) - Radio Rudder Input - GPIO input with interrupt capability
  - PI13 (Pin 26) - Radio Sail Input - GPIO input with interrupt capability
- **PWM Output Pins:**
  - PI12 (Pin 33) - Servo Rudder Output - PWM2 channel, high impedance when disabled
  - PI14 (Pin 16) - Servo Sail Output - PWM4 channel, high impedance when disabled

## Pin Definitions

### Orange Pi Zero 2W 40-Pin Header Layout
```
Physical Pin  | GPIO Line | GPIO Name | Function              | Module Usage
-------------|-----------|-----------|----------------------|------------------
Pin 7        | 267       | PI11      | GPIO Input           | Radio Rudder Input
Pin 16       | 270       | PI14      | PWM4 Output          | Servo Sail Output  
Pin 26       | 269       | PI13      | GPIO Input           | Radio Sail Input
Pin 33       | 268       | PI12      | PWM2 Output          | Servo Rudder Output
```

### GPIO Chip Information
The Orange Pi Zero 2W uses `gpiochip0` with 288 GPIO lines. The module uses:
- **Line 267** (PI11): Radio Rudder Input - named "radio_rudder" in kernel module
- **Line 268** (PI12): Servo Rudder Output - PWM2 channel
- **Line 269** (PI13): Radio Sail Input - named "radio_sail" in kernel module  
- **Line 270** (PI14): Servo Sail Output - PWM4 channel

**Note:** Pin assignments are based on the official Orange Pi Zero 2W schematic showing:
- **Rudder Servo Output (PI12)** on Pin 33
- **Sail Servo Output (PI14)** on Pin 16  
- **Rudder Radio Input (PI11)** on Pin 7
- **Sail Radio Input (PI13)** on Pin 26

### 40-Pin Header Physical Layout
```
    3.3V  [1]  [2]  5V
    SDA   [3]  [4]  5V  
    SCL   [5]  [6]  GND 
    RADIO [7]  [8]  TXD0 ← PI11 (Radio Rudder Input)
    GND   [9]  [10] RXD0
    TXD5  [11] [12] PI01
    RXD5  [13] [14] GND 
    TXD2  [15] [16] SERVO ← PI14 (Sail Servo Output)
    3.3V  [17] [18] PH04
    MOSI  [19] [20] GND 
    MISO  [21] [22] RXD2
    SCLK  [23] [24] CE0  
    GND   [25] [26] RADIO ← PI13 (Radio Sail Input)
    SDA   [27] [28] SCL2
    PI00  [29] [30] GND 
    PI15  [31] [32] PWM1
    SERVO [33] [34] GND  ← PI12 (Rudder Servo Output)
    PI02  [35] [36] PC12
    PI16  [37] [38] PI04
    GND   [39] [40] PI03
```

### Verification Commands
```bash
# Check GPIO line usage
gpioinfo | grep -E "(radio_rudder|radio_sail|267|268|269|270)"

# Check physical pin layout
gpio readall

# Verify kernel module is using the pins
dmesg | grep -E "(radio_rudder|radio_sail|PWM)"

# Check sysfs interface
ls -la /sys/kernel/argo_radio_servo/
```

### Safety Circuit Design
The servo output pins are connected to the radio control outputs through resistors, creating a fail-safe design:
- **When PWM is active**: Software controls the servos
- **When PWM is disabled (high impedance)**: Radio control passes through resistors directly to servos
- **Default state**: PWM disabled on module load for maximum safety

## Features

- **Real-time PWM pulse width measurement** using GPIO interrupts
- **Hardware PWM output generation** via kernel PWM subsystem with high impedance safety mode
- **Automatic human/computer control switching** based on radio input activity
- **High-resolution timing** using kernel hrtimers
- **Sysfs interface** for easy userspace integration
- **Built-in safety features** with pulse width validation
- **High impedance mode**: Writing 0 to servo control files disables PWM output, allowing radio control to pass through
- **Fail-safe design**: Default state is PWM disabled (high impedance) for maximum safety

## Sysfs Interface

The module creates the following entries in `/sys/kernel/argo_radio_servo/`:

### Read-Only (Input Measurements)
- `radio_rudder_pw_us` - Current radio rudder pulse width (microseconds)
- `radio_sail_pw_us` - Current radio sail pulse width (microseconds)

### Read-Write (Servo Commands)
- `servo_rudder_pw_us` - Set/read rudder servo pulse width (0 for high impedance, 900-2100μs for PWM)
- `servo_sail_pw_us` - Set/read sail servo pulse width (0 for high impedance, 900-2100μs for PWM)

### Usage Example
```bash
# Read current radio inputs
cat /sys/kernel/argo_radio_servo/radio_rudder_pw_us
# Output: 1520

# Set servo outputs to PWM mode
echo 1600 | sudo tee /sys/kernel/argo_radio_servo/servo_rudder_pw_us
echo 1400 | sudo tee /sys/kernel/argo_radio_servo/servo_sail_pw_us

# Set servo outputs to high impedance mode (radio control active)
echo 0 | sudo tee /sys/kernel/argo_radio_servo/servo_rudder_pw_us
echo 0 | sudo tee /sys/kernel/argo_radio_servo/servo_sail_pw_us

# Check current servo states
cat /sys/kernel/argo_radio_servo/servo_rudder_pw_us
# Output: 0 (high impedance) or pulse width value (PWM active)
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
The overlays listed in `orangepiEnv.txt` are required for the following reasons:

- `pi-i2c0`: Enables I2C bus 0 on the Orange Pi Zero 2W, which is used for sensor communication (e.g., ADC, IMU, wind sensors).
- `disable-uart0`: Disables the default UART0 to free up pins or avoid conflicts, as UART0 is not used by the Argo system.
- `ph-uart5`: Enables UART5 on the PH2/PH3 pins, required for the GPS module (u-blox NEO-M9N).
- `pi-pwm2` and `pi-pwm4`: Enable hardware PWM channels 2 and 4, which are used for generating servo control signals.
- `user_overlays=argo_radio_servo_overlay`: Loads the custom device tree overlay for the Argo radio/servo kernel module, configuring the necessary pins and hardware resources for PWM capture and servo output.

Each overlay ensures the correct hardware interfaces are available and properly configured for the Argo sailboat's radio control and sensor systems.

### Step 2: Build and Install the Module

```bash
# Navigate to the module directory
cd /path/to/argo/nodes/pwm_capture_module

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
- **High Impedance Mode**: Writing 0 disables PWM output, putting pins in high impedance state
- **Safety**: Default state is PWM disabled (high impedance) for maximum safety, allowing radio control to pass through

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

### GPIO Pin Verification
```bash
# Verify the kernel module is using the correct GPIO lines
gpioinfo | grep -E "(267|268|269|270)"

# Expected output should show:
# line 267: "radio_rudder" input active-high [used]
# line 268: kernel input active-high [used]  
# line 269: "radio_sail" input active-high [used]
# line 270: kernel input active-high [used]

# Check if pins are in correct mode
gpio readall | grep -E "(PI11|PI12|PI13|PI14)"

# Physical pin verification (from gpio readall output)
# Pin 7: Should show PI11 (radio rudder input)
# Pin 16: Should show PI14 (servo sail output) 
# Pin 26: Should show PI13 (radio sail input)
# Pin 33: Should show PI12 (servo rudder output)
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

The module is designed to work with the ROS2 PWM node (`rudder_sail_radio.py`) which:
- Reads radio inputs from sysfs
- Publishes radio control values as ROS2 topics
- Receives autonomous control commands
- Automatically switches between manual/autonomous control
- Supports high impedance mode for fail-safe operation
- Sets servos to high impedance mode when not actively controlling them

## Version History

- **v0.5** - Added high impedance mode support for servo outputs, fail-safe design with default PWM disabled
- **v0.4** - Fixed device tree overlay structure and PWM references
- **v0.3** - Added proper GPIO handling and interrupt-based capture
- **v0.2** - Initial PWM output support
- **v0.1** - Basic platform driver framework

## License

GPL v2 - See kernel module source for full license text.

## Author

Tobi Delbruck - Autonomous boat navigation system for Orange Pi Zero 2W

