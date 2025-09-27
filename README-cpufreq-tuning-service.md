# CPU Frequency Tuning Service

## Overview

This service configures the Orange Pi Zero 2W CPU frequency scaling to use a **conservative governor** with thermal-friendly parameters to prevent system hangs during heavy workloads (such as Cursor agent interactions).

## Problem Addressed

The original system was configured with an **ondemand governor** with aggressive parameters:
- `up_threshold: 25%` - CPU scaled to maximum frequency at only 25% usage
- This caused sustained high CPU frequencies (1.51 GHz) during memory-intensive operations
- Contributed to thermal stress and potential system instability

## Solution

The `cpufreq-tuning.service` applies a **conservative governor** with optimized parameters:

### Governor: Conservative
- More gradual frequency scaling
- Better thermal management
- Reduced power consumption
- More stable under sustained loads

### Conservative Governor Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `up_threshold` | 80% | CPU must reach 80% usage before scaling up |
| `down_threshold` | 20% | CPU drops below 20% usage before scaling down |
| `sampling_rate` | 100,000 μs | 100ms sampling interval (faster response) |
| `sampling_down_factor` | 1 | Equal scaling speed up and down |
| `ignore_nice_load` | 0 | Include nice processes in load calculation |
| `freq_step` | 0 | Use default frequency step size |

## Files

### Service Configuration
- **Service File**: `/etc/systemd/system/cpufreq-tuning.service`
- **Script**: `/usr/local/bin/cpufreq-tuning.sh`
- **Log File**: `/var/log/cpufreq-tuning.log`

### Dependencies
- **Requires**: `cpufrequtils.service`
- **After**: System boots and cpufrequtils is active

## Usage

### Service Management
```bash
# Check service status
sudo systemctl status cpufreq-tuning.service

# Start service manually
sudo systemctl start cpufreq-tuning.service

# Stop service
sudo systemctl stop cpufreq-tuning.service

# Disable service (revert to default)
sudo systemctl disable cpufreq-tuning.service
```

### Verify Configuration
```bash
# Check current governor
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Check conservative parameters
cat /sys/devices/system/cpu/cpufreq/conservative/up_threshold
cat /sys/devices/system/cpu/cpufreq/conservative/down_threshold

# Check current frequencies
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
```

### Monitor Performance
```bash
# View service log
cat /var/log/cpufreq-tuning.log

# Monitor CPU frequencies in real-time
watch -n 1 'cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq'
```

## Expected Benefits

1. **Reduced Thermal Stress**: Less aggressive scaling reduces heat generation
2. **Better Stability**: Conservative approach prevents frequency thrashing
3. **Lower Power Consumption**: Reduced time at maximum frequency
4. **Improved Hang Prevention**: More stable CPU behavior during memory pressure

## Monitoring Integration

The service works alongside the existing monitoring stack:
- **Memory monitoring**: Tracks memory usage patterns
- **Thermal monitoring**: Monitors temperature changes
- **Orange Pi Monitor**: Logs CPU frequency and performance metrics

## Troubleshooting

### Service Not Starting
```bash
# Check service status
sudo systemctl status cpufreq-tuning.service

# View system logs
sudo journalctl -u cpufreq-tuning.service

# Reload systemd configuration
sudo systemctl daemon-reload
```

### Governor Not Applied
```bash
# Check if cpufrequtils is running
sudo systemctl status cpufrequtils

# Manually run the tuning script
sudo /usr/local/bin/cpufreq-tuning.sh

# Verify parameters
cat /sys/devices/system/cpu/cpufreq/conservative/up_threshold
```

### Reverting to Default
```bash
# Disable the service
sudo systemctl disable cpufreq-tuning.service

# Set back to ondemand
echo ondemand | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Restart cpufrequtils
sudo systemctl restart cpufrequtils
```

## Performance Impact

- **Positive**: Reduced thermal stress, better stability
- **Trade-off**: Slightly slower response to CPU load changes
- **Overall**: Better for sustained workloads and system stability

## Created

- **Date**: September 25, 2025
- **Purpose**: Prevent system hangs during heavy Cursor agent interactions
- **System**: Orange Pi Zero 2W (Allwinner H616)
