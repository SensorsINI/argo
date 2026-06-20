# Argo I2C Bus Configuration and Recovery

## Table of Contents
- [I2C Bus Overview](#i2c-bus-overview)
- [Hardware Configuration](#hardware-configuration)
- [I2C Bus Lock Issue](#i2c-bus-lock-issue)
- [Recovery Solutions](#recovery-solutions)
- [Device Tree Overlay Solution](#device-tree-overlay-solution)
- [Current Reset Scripts](#current-reset-scripts)
- [References](#references)

## I2C Bus Overview

The Argo sailboat uses multiple I2C buses on the Orange Pi Zero 2W:
- **I2C bus 0** for battery/water and IMU sensors
- **I2C bus 2** (pins 27/28) for mast/wind sensors

### I2C Bus Mapping

| Controller | I2C Bus | Physical Pins | Device Tree | Devices | Usage |
|------------|---------|---------------|-------------|---------|-------|
| `5002000.i2c` | **bus 0** | PI5 (pin 15), PI6 (pin 22) | Enabled via `pi-i2c0` overlay | Sensors (0x34, 0x44, 0x4a) | **Battery/IMU sensors** |
| `i2c-2 (v4 header bus)` | **bus 2** | Pin 27 (SDA.2), pin 28 (SCL.2) | Enabled on v4 pin mapping | Sensors (0x21-0x23, 0x62, 0x70) | **Wind + mast LED sensors** |
| `7081400.i2c` | bus 1 | Internal | Built-in | PMIC (0x36) | **⚠️ CRITICAL - Power Management** |

### Critical Warning

**NEVER unbind controller `7081400.i2c`** - It hosts the AXP313a PMIC which controls system voltage regulation, power button, and power sequencing. Unbinding it will cause immediate system crash and watchdog reboot.

## Hardware Configuration

### I2C Bus 0 Pin Assignment

From device tree analysis:
- **SDA (Data)**: GPIO 261 (PI5) - Physical pin 15
- **SCL (Clock)**: GPIO 262 (PI6) - Physical pin 22
- **Function**: I2C0 when `pi-i2c0` overlay is enabled
- **Alternative**: Can be muxed to GPIO mode for bus recovery

### Current Boot Configuration

From `/boot/orangepiEnv.txt`:
```
overlays=pi-i2c0 ph-uart5 pi-pwm2 pi-pwm4 spi1-cs0-spidev
user_overlays=argo_radio_servo_overlay argo-ph4-led-overlay
```

The `pi-i2c0` overlay enables I2C bus 0 on pins PI5/PI6.

### I2C Bus 2 Pin Assignment (v4 mast/wind bus)

- **SDA (Data)**: GPIO 266 (SDA.2) - Physical pin 27
- **SCL (Clock)**: GPIO 265 (SCL.2) - Physical pin 28
- **Function**: Dedicated mast/wind bus on v4 pin mapping

### Connected Devices by I2C Bus

| Bus | Address | Device | Purpose | Node |
|-----|---------|--------|---------|------|
| 2 | 0x21 | Sensirion SDP3x | Wind sensor (CCW) | `anem.py` |
| 2 | 0x22 | Sensirion SDP3x | Wind sensor (Center) | `anem.py` |
| 2 | 0x23 | Sensirion SDP3x | Wind sensor (CW) | `anem.py` |
| 2 | 0x62 | NXP PCA9632 | Mast RGBW LED controller (individual addr) | `argo_mast_leds.py` |
| 2 | 0x70 | NXP PCA9632 | LED All Call (broadcast; active at power-up) | — |
| 0 | 0x34 | MAX11612 ADC | Battery/water monitoring | `argo_battery_water.py` |
| 0 | 0x44 | SHT45 | Temperature/humidity | `argo_battery_water.py` |
| 0 | 0x4a | Adafruit BNO085 | 9-DOF IMU | `bno085.py` (via C++ driver) |

**Power and Logic Levels**:
- **Orange Pi I2C**: 3.3V logic levels (PI5/PI6 pins)
- **Most sensors**: 5V power supply (SDP3x, MAX11612, BNO085) but I2C compatible with 3.3V logic
- **SHT45**: 3.3V power supply
- **Pull-up resistors**: Provided by the two TXS0102 level-shifter sections on both sides of the bus path (3.3V OPi side and 5V sensor side)
- **Level shifter**: TXS0102DCT used in the active I2C path
- **Bus speed**: 100kHz (standard I2C) from the `pi-i2c0` overlay by default
- **Optional 400kHz (Fast mode)** on bus 0: build/install the Argo overlay with  
  `make -C nodes bno085-i2c0-fast-install` (from the `nodes` Makefile: compiles `nodes/bno085_i2c0_fast/`, copies to `/boot/overlay-user`, appends `argo-bno085-i2c0-fast` to `user_overlays` in `/boot/orangepiEnv.txt`). **Reboot required.** All devices sharing **i2c-0** must support Fast mode.

### I2C Services

These systemd services actively use I2C:
- `argo_bno085.service` - IMU sensor fusion (**starts first** on bus 0)
- `argo_battery_water.service` - Battery/water monitoring (starts after IMU init)
- `argo_launch_standard.service` - Main ROS2 launch (includes `anem.py` on bus 2)

**Boot ordering (bus 0):** Both services share i2c-0. The BNO085 needs uninterrupted
SHTP traffic during init (`sh2_getProdIds`); concurrent MAX11612/SHT45 polls from
battery/water cause init failures. At boot:

1. `argo_bno085.service` starts (`Before=argo_battery_water.service`)
2. `argo_battery_water.service` waits in `ExecStartPre` until `bno08x_driver` has been
   running stably for 3 seconds (`scripts/wait_for_bno085_ready.sh`, 45 s timeout)
3. After IMU init, short ADC/humidity reads can interleave with the driver's streaming

This does **not** fix an already electrically locked bus (see [I2C Bus Lock Issue](#i2c-bus-lock-issue))
or IMU restarts that race with battery/water mid-init. Reinstall services after changing
unit files: `make -C nodes battery-water-install` and `make -C nodes bno085-service-install`.

## I2C Bus Lock Issue

### Problem Description

The I2C bus can become **electrically locked** when a device holds the SDA (data) or SCL (clock) line low, typically due to:
- **Cable disconnection** during an active transaction
- **Device crash** mid-communication
- **Electrical noise** causing transaction corruption
- **Power glitch** to a sensor

### Symptoms

```bash
$ i2cget -y 0 0x44 0x00
Error: Read failed

$ dmesg | tail
[  958.388832] i2c i2c-0: mv64xxx: I2C bus locked, block: 1, time_left: 0
```

The driver detects the lock but **cannot recover automatically** because:
1. The `mv64xxx_i2c` driver lacks **pinctrl** support for bus recovery
2. Error message: `can't get pinctrl, bus recovery not supported`
3. Without pinctrl, the driver cannot switch pins to GPIO mode to perform bit-banging recovery

### Why Manual Cable Replugging Works

When you physically replug the I2C cable:
1. **Electrical connection is broken** - stuck devices lose power/connection
2. **Capacitance drains** - SDA/SCL lines float high via pull-up resistors  
3. **Devices reset** - internal state machines restart
4. **Lines released** - when reconnected, bus is no longer locked

The recovery requires **specific timing** - plugging in during an idle period allows devices to properly initialize.

### Why Software Reset Doesn't Work

#### Simple Unbind/Rebind (Doesn't Work)

```bash
# Stops services, unbinds and rebinds controller
echo "5002000.i2c" | sudo tee /sys/bus/platform/drivers/mv64xxx_i2c/unbind
echo "5002000.i2c" | sudo tee /sys/bus/platform/drivers/mv64xxx_i2c/bind
```

**Result**: Resets the driver software state but **NOT the hardware electrical state**. The physical SDA/SCL lines remain stuck.

#### GPIO Bit-Banging Attempt (Doesn't Work)

Attempted in `scripts/i2c_gpio_recovery.sh`:
1. Unbind I2C controller
2. Export GPIO 261/262
3. Bit-bang 9 clock pulses + STOP condition
4. Rebind controller

**Result**: Failed because:
- Pins could not be exported as GPIOs while I2C driver had them claimed
- Even after unbinding, pins remained in I2C hardware mode
- Bash `sleep 0.000005` cannot achieve 5µs timing (sleeps much longer)

## Recovery Solutions

### Current Workaround: Manual Cable Replug

**Status**: ✅ Works but requires physical access

**Procedure**:
1. Stop power control to prevent auto-shutdown: `sudo systemctl stop argo_power_control.service`
2. Unplug I2C cable
3. Wait 1-2 seconds
4. Replug cable
5. Sometimes requires multiple attempts with different timing
6. Restart power control: `sudo systemctl start argo_power_control.service`

### Failed Approach: Software-Only Reset

**Status**: ❌ Does not recover locked bus

**Script**: `scripts/reset_i2c_bus.sh`
- Stops I2C services
- Unbinds/rebinds controller `5002000.i2c`
- Restarts services

**Why it fails**: Only resets software driver state, not hardware electrical state.

### Failed Approach: GPIO Bit-Bang Recovery

**Status**: ❌ Cannot switch pins to GPIO mode

**Script**: `scripts/i2c_gpio_recovery.sh`
- Attempts to export PI5/PI6 as GPIOs
- Tries to bit-bang recovery sequence

**Why it fails**: 
- Pins remain locked to I2C hardware function
- Cannot export as GPIOs even after unbinding driver
- Bash timing too imprecise for I2C protocol

## Device Tree Overlay Solution

### The Proper Fix: Enable Kernel I2C Recovery

**Status**: ⚠️ Not yet implemented - requires device tree modification and reboot

The Linux kernel `mv64xxx_i2c` driver **supports automatic I2C bus recovery** if properly configured in the device tree. The driver can:
1. Switch pins to GPIO mode (via pinctrl)
2. Bit-bang 9 clock pulses to unstick devices
3. Send STOP condition
4. Switch pins back to I2C mode

### Required Device Tree Overlay

Create `/boot/dtb/allwinner/overlay/sun50i-h616-pi-i2c0-recovery.dtbo` from this source:

```dts
/dts-v1/;
/plugin/;

/ {
    compatible = "allwinner,sun50i-h616";

    fragment@0 {
        target = <&i2c0>;
        __overlay__ {
            pinctrl-names = "default", "gpio";
            pinctrl-0 = <&i2c0_pi_pins>;
            pinctrl-1 = <&i2c0_gpio_recovery_pins>;
            scl-gpios = <&pio 8 6 GPIO_ACTIVE_HIGH>;  /* PI6 - SCL */
            sda-gpios = <&pio 8 5 GPIO_ACTIVE_HIGH>;  /* PI5 - SDA */
            status = "okay";
        };
    };

    fragment@1 {
        target = <&pio>;
        __overlay__ {
            i2c0_gpio_recovery_pins: i2c0-gpio-recovery-pins {
                pins = "PI5", "PI6";
                function = "gpio_in";
            };
        };
    };
};
```

**Key elements**:
- `pinctrl-names`: Defines "default" (I2C mode) and "gpio" (recovery mode) states
- `pinctrl-0`: Normal I2C pin configuration (already exists in `pi-i2c0` overlay)
- `pinctrl-1`: GPIO pin configuration for recovery
- `scl-gpios`, `sda-gpios`: Tell driver which GPIOs to use for recovery

### Installation Procedure

```bash
# 1. Create the DTS file
cd /tmp
cat > sun50i-h616-pi-i2c0-recovery.dts << 'EOF'
/dts-v1/;
/plugin/;

/ {
    compatible = "allwinner,sun50i-h616";

    fragment@0 {
        target = <&i2c0>;
        __overlay__ {
            pinctrl-names = "default", "gpio";
            pinctrl-0 = <&i2c0_pi_pins>;
            pinctrl-1 = <&i2c0_gpio_recovery_pins>;
            scl-gpios = <&pio 8 6 GPIO_ACTIVE_HIGH>;
            sda-gpios = <&pio 8 5 GPIO_ACTIVE_HIGH>;
            status = "okay";
        };
    };

    fragment@1 {
        target = <&pio>;
        __overlay__ {
            i2c0_gpio_recovery_pins: i2c0-gpio-recovery-pins {
                pins = "PI5", "PI6";
                function = "gpio_in";
            };
        };
    };
};
EOF

# 2. Compile the overlay
dtc -I dts -O dtb -o sun50i-h616-pi-i2c0-recovery.dtbo sun50i-h616-pi-i2c0-recovery.dts

# 3. Install to boot partition
sudo cp sun50i-h616-pi-i2c0-recovery.dtbo /boot/dtb/allwinner/overlay/

# 4. Update boot configuration
sudo nano /boot/orangepiEnv.txt
# Change this line:
#   overlays=pi-i2c0 ph-uart5 pi-pwm2 pi-pwm4 spi1-cs0-spidev
# To:
#   overlays=pi-i2c0-recovery ph-uart5 pi-pwm2 pi-pwm4 spi1-cs0-spidev

# 5. Reboot
sudo reboot

# 6. Verify recovery support is enabled
dmesg | grep -i "i2c.*recovery"
# Should show recovery enabled message
```

### Expected Behavior After Fix

Once the device tree overlay is installed:
- The `mv64xxx_i2c` driver will automatically detect bus locks
- It will perform GPIO-based recovery (9 clock pulses + STOP)
- Recovery happens transparently without manual intervention
- The error message will change to show recovery attempts

### Alternative: Kernel Driver Patch

If device tree overlay doesn't work, a kernel driver patch could add recovery support directly. This requires:
1. Modify `drivers/i2c/busses/i2c-mv64xxx.c`
2. Add `i2c_bus_recovery_info` structure
3. Implement GPIO control functions
4. Recompile custom kernel

**Not recommended** due to complexity of maintaining custom kernel builds.

## Current Reset Scripts

### scripts/reset_i2c_bus.sh

**Purpose**: Software-level I2C controller reset
**Status**: ✅ Safe but ❌ Ineffective for bus lock recovery

```bash
# Usage
sudo /home/orangepi/argo/scripts/reset_i2c_bus.sh

# Or with alias
ai2creset
```

**What it does**:
1. Checks running I2C services
2. Asks for confirmation to stop them
3. Stops `argo_battery_water.service`, `argo_bno085.service`, `argo_launch_standard.service`
4. Unbinds controller `5002000.i2c`
5. Rebinds controller `5002000.i2c`
6. Restarts services
7. Tests bus with `i2cdetect`

**Limitations**: Only resets driver state, not hardware electrical state. Will not recover from bus lock.

**Safety Features**:
- Uses correct controller (`5002000.i2c`, not `7081400.i2c`)
- Stops services before unbind to prevent kernel hangs
- Automatic cleanup on errors
- Force mode with `--force` flag (dangerous, may crash nodes)

**Sudoers Setup**:
```bash
sudo /home/orangepi/argo/scripts/install_i2c_reset_sudoers.sh
```

This allows `ai2creset` alias to work without password.

### scripts/i2c_gpio_recovery.sh

**Purpose**: GPIO-based hardware recovery attempt
**Status**: ❌ Does not work - pins cannot be switched to GPIO mode

```bash
# Usage
sudo /home/orangepi/argo/scripts/i2c_gpio_recovery.sh
```

**What it attempts**:
1. Stops I2C services
2. Unbinds controller
3. Exports GPIO 261 (PI5-SDA) and 262 (PI6-SCL)
4. Configures as outputs
5. Bit-bangs 9 clock pulses
6. Sends STOP condition
7. Unexports GPIOs
8. Rebinds controller
9. Restarts services

**Why it fails**: Even after unbinding the I2C driver, the pins remain in I2C hardware mode and cannot be exported as GPIOs without proper pinctrl device tree configuration.

## References

### Documentation Created During Investigation

- `docs/I2C-BUS-WARNING.md` - Critical warning about controller mapping
- `docs/README-i2c.md` - This file

### Related Files

- `nodes/README.md` - I2C sensor configuration and addresses
- `nodes/anem.py` - Wind sensor node (I2C bus 2: 0x21, 0x22, 0x23)
- `nodes/argo_battery_water.py` - ADC and humidity (I2C 0x34, 0x44)
- `nodes/bno085.py` - IMU node (I2C 0x4a via C++ driver)

### Kernel and Device Tree

- Driver: `mv64xxx_i2c` (Marvell I2C controller, used by Allwinner)
- Current overlay: `/boot/dtb/allwinner/overlay/sun50i-h616-pi-i2c0.dtbo`
- Boot config: `/boot/orangepiEnv.txt`
- Kernel: `6.1.31-sun50iw9` (custom build)

### Research Sources

- [Armbian I2C3 Forum Discussion](https://forum.armbian.com/topic/54456-i2c3/)
- [Linux Kernel I2C Documentation](https://www.kernel.org/doc/html/v6.6/driver-api/i2c.html)
- [Orange Pi I2C Setup Guide](https://www.9a5dlz.eu/embedded/opi_i2c.php)
- [Armbian Device Tree Overlay Documentation](https://docs.armbian.com/User-Guide_Armbian_overlays/)
- [I2C Bus Recovery Techniques](https://bits4device.wordpress.com/2017/07/28/i2c-bus-recovery/)

## Next Steps

1. **Create and test device tree overlay** for I2C0 recovery support
2. **Validate automatic recovery** works when bus locks
3. **Update this documentation** with test results
4. **Remove failed recovery scripts** if overlay works
5. **Document recovery behavior** in system monitoring

## Summary

- **Current State**: I2C bus lock issues are primarily observed on bus 0; bus 2 hosts wind/mast devices on v4.
- **Root Cause**: Missing pinctrl device tree configuration for GPIO-based recovery
- **Proper Solution**: Install device tree overlay to enable kernel automatic recovery
- **Workaround**: Physical cable replug (works but impractical for remote operation)
- **Failed Attempts**: Software-only reset and GPIO bit-banging don't work
- **Critical Safety**: Never unbind controller `7081400.i2c` (PMIC - will crash system)
