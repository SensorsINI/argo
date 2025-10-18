# DVFS Userspace Access on Orange Pi Zero 2W

## Summary
**YES**, both CPU frequency AND voltage are accessible from userspace on the Orange Pi Zero 2W with Armbian.

## Current System Configuration
- **SoC**: Allwinner H618 (sun50iw9)
- **Kernel**: Linux 6.1.31-sun50iw9
- **PMIC**: AXP313A (I2C address 0x36 on bus 1)
- **CPU Voltage Rail**: axp313a-dcdc2 (regulator.5)
- **Speed Bin**: 0 (premium grade - lowest voltages)

## Userspace Access Methods

### 1. CPU Frequency (Read/Write)
```bash
# Read current frequency
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq  # in kHz

# Read available frequencies
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
# Output: 480000 600000 792000 1008000 1200000 1344000 1416000 1512000

# Read/Write governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo "performance" | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Force specific frequency (requires userspace governor)
echo "userspace" | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 1008000 | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed
```

### 2. CPU Voltage (Read-Only)
```bash
# Read current CPU voltage (from DCDC2 regulator)
cat /sys/class/regulator/regulator.5/microvolts  # in microvolts
# or by name:
cat /sys/class/regulator/regulator.5/name        # axp313a-dcdc2
cat /sys/class/regulator/regulator.5/microvolts  # current voltage

# Read voltage limits
cat /sys/class/regulator/regulator.5/min_microvolts  # 500000 uV
cat /sys/class/regulator/regulator.5/max_microvolts  # 1200000 uV

# Verify it's the CPU regulator
ls -la /sys/class/regulator/regulator.5/consumer:cpu:cpu0
```

### 3. All PMIC Voltages (Read-Only)
```bash
# List all AXP313A voltage rails
for reg in /sys/class/regulator/regulator.*; do
    name=$(cat $reg/name 2>/dev/null)
    if [[ $name == axp313a* ]]; then
        volt=$(cat $reg/microvolts 2>/dev/null)
        printf "%-20s: %7d uV = %.3f V\n" "$name" "$volt" \
               $(echo "scale=3; $volt/1000000" | bc)
    fi
done
```

**Output:**
```
axp313a-dcdc1       :  900000 uV = 0.900 V  (GPU or system rail)
axp313a-dcdc2       : 1050000 uV = 1.050 V  (CPU core - DVFS active)
axp313a-dcdc3       : 1100000 uV = 1.100 V  (DRAM - fixed voltage)
axp313a-aldo1       : 1800000 uV = 1.800 V  (Analog LDO - I/O)
axp313a-dldo1       : 3300000 uV = 3.300 V  (Digital LDO - peripherals)
```

## CPU OPP Table (Speed Bin 0 - This System)
The kernel uses this frequency/voltage table for DVFS:

| Frequency | Voltage (Speed Bin 0) |
|-----------|----------------------|
| 480 MHz   | 0.900 V             |
| 600 MHz   | 0.900 V             |
| 792 MHz   | 0.900 V             |
| 1008 MHz  | 0.900 V             |
| 1200 MHz  | 0.960 V             |
| 1344 MHz  | 1.050 V             |
| 1416 MHz  | 1.100 V             |
| 1512 MHz  | 1.100 V             |

**Note**: The H618 supports 3 speed bins (0, 1, 2). Speed bin 0 is the premium grade requiring the lowest voltages. Your chip is running speed bin 0.

## DVFS Verification Test
```bash
# Watch voltage change with frequency
watch -n 0.5 'echo "Freq: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) kHz, \
Voltage: $(cat /sys/class/regulator/regulator.5/microvolts) uV"'

# Stress test to force max frequency
stress-ng --cpu 4 --timeout 10s &
# Watch voltage increase to 1.100V at max frequency
```

## Real-Time Monitoring Script
A monitoring script is available at `/tmp/dvfs_monitor.sh`:

```bash
/tmp/dvfs_monitor.sh
# Output: [HH:MM:SS] CPU: 1344 MHz | Voltage: 1.050 V | Governor: ondemand | Load: 25.3%
```

## Important Notes

### Voltage is Read-Only
- **Userspace CANNOT write voltages** - this is controlled by the kernel's regulator framework
- The kernel automatically adjusts voltage based on the OPP table when frequency changes
- This is a safety feature to prevent voltage/frequency mismatches that could damage hardware

### Why You Can Read Voltages
The AXP313A PMIC provides voltage readback via I2C registers, which the kernel driver exposes through sysfs.

### Device Tree Location
The OPP table is defined in the device tree:
```bash
# CPU OPP table with speed bins
ls /sys/firmware/devicetree/base/opp-table-cpu/

# PMIC configuration
ls /sys/firmware/devicetree/base/soc/i2c@7081400/pmic@36/
```

## Kernel Driver Details
- **PMIC Driver**: `axp20x-i2c` + `axp20x-regulator`
- **Source Files** (in kernel sources):
  - `drivers/mfd/axp20x-i2c.c` - I2C interface
  - `drivers/mfd/axp20x.c` - Core MFD driver
  - `drivers/regulator/axp20x-regulator.c` - Voltage regulator
  - `include/linux/mfd/axp20x.h` - Register definitions
- **PMIC Location**: I2C bus 1, address 0x36
- **Kernel Detection**: Shows as `UU` in `i2cdetect -y 1` (kernel driver active)

## Comparison with Frequency-Only Systems
Unlike some embedded systems that only expose frequency scaling, the Orange Pi Zero 2W provides:
1. ✅ **Frequency**: Read/write via cpufreq sysfs
2. ✅ **Voltage**: Read-only via regulator sysfs  
3. ✅ **Automatic DVFS**: Kernel manages voltage changes when frequency changes
4. ✅ **OPP Table**: Voltage/frequency pairs defined in device tree

This makes it possible to fully monitor power consumption, thermal behavior, and DVFS operation from userspace.

## Example Use Cases

### Power Monitoring
Monitor voltage × current to calculate CPU power consumption in real-time.

### Thermal Analysis
Correlate CPU temperature with voltage/frequency to optimize cooling.

### Performance Profiling
Track DVFS transitions during workload execution for optimization.

### Battery Life Estimation
Calculate power draw at different OPP points for battery life predictions.

---

**Last Updated**: Based on system running Linux 6.1.31-sun50iw9 with AXP313A PMIC on Orange Pi Zero 2W

