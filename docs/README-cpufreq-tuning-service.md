# CPU Frequency Tuning Service

## Overview

Argo’s Orange Pi Zero 2W (Allwinner H618) uses **two independent DVFS layers**:

| Layer | Mechanism | Trigger | Purpose |
|-------|-----------|---------|---------|
| **Argo (this service)** | `conservative` cpufreq governor | CPU load | Less aggressive scaling than stock `ondemand`; reduces sustained max-frequency time |
| **Kernel thermal** | `cpufreq-cpu0` cooling device | CPU temp ≥ 85°C | Caps `scaling_max_freq` when the passive thermal trip is crossed |

This document covers the **Argo cpufreq-tuning service** and how it interacts with kernel thermal throttling. For sysfs OPP/voltage details, see [dvfs_userspace_access.md](dvfs_userspace_access.md). For temperature logging and alerts, see [README-thermal-monitoring.md](README-thermal-monitoring.md).

**Last verified on hardware:** 2026-06-17 (kernel 6.1.31-sun50iw9).

---

## Problem Addressed

The original Armbian image used an **ondemand** governor with aggressive parameters:

- `up_threshold: 25%` — CPU scaled to maximum frequency at only 25% usage
- Caused sustained 1512 MHz during memory-intensive work (e.g. Cursor agent sessions)
- Contributed to thermal stress and occasional system instability

## Solution

`cpufreq-tuning.service` runs once at boot and applies a **conservative** governor with higher scale-up thresholds.

### Governor: Conservative

- More gradual frequency scaling under moderate load
- Still reaches 1512 MHz when all cores are busy
- Does **not** temperature-limit the CPU — that is handled separately by kernel thermal cooling

---

## Available CPU Frequencies (OPP)

Eight discrete levels (kHz values from `scaling_available_frequencies`):

| MHz | kHz | Notes |
|-----|-----|-------|
| 480 | 480000 | Minimum |
| 600 | 600000 | |
| 792 | 792000 | |
| 1008 | 1008000 | |
| 1200 | 1200000 | Typical kernel thermal cap (cooling state 3/7) |
| 1344 | 1344000 | |
| 1416 | 1416000 | |
| 1512 | 1512000 | Maximum |

Voltage per OPP (speed bin 0): see [dvfs_userspace_access.md](dvfs_userspace_access.md).

---

## Conservative Governor Parameters

### Values set by `/usr/local/bin/cpufreq-tuning.sh`

| Parameter | Script writes | Description |
|-----------|---------------|-------------|
| `scaling_governor` | `conservative` | All CPU cores |
| `up_threshold` | 80 | Scale up only when average load ≥ 80% |
| `down_threshold` | 20 | Scale down when load < 20% |
| `sampling_rate` | 100000 μs | 100 ms sample interval |
| `sampling_down_factor` | 1 | Same speed scaling up and down |
| `ignore_nice_load` | 0 | Include nice processes in load |
| `freq_step` | 0 | Kernel default step (observed as 5 at runtime) |

### Runtime values observed (2026-06-17)

After boot, verify live values — some may differ from what the script writes (e.g. if `cpufrequtils` or the kernel adjusts them later):

```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpufreq/conservative/{up_threshold,down_threshold,sampling_rate,freq_step}
```

Observed at last check: governor `conservative`, `up_threshold=80`, `down_threshold=20`, `sampling_rate=10000`, `freq_step=5`.

---

## Kernel Thermal Throttling (separate from this service)

The cpufreq-tuning service does **not** configure thermal limits. The kernel manages those via `thermal_zone2` (`cpu-thermal`):

| Setting | Value |
|---------|-------|
| Policy | `fair_share` |
| Passive trip | **85°C** (`trip_point_0_temp`) |
| Hysteresis | **2°C** (throttle above 85°C, release below ~83°C) |
| Critical trip | 100°C |
| Cooling device | `cpufreq-cpu0`, states 0–7 |

### Cooling state → max frequency cap

When the passive trip is crossed, the kernel sets `scaling_max_freq` according to cooling state:

| State | Max frequency |
|-------|---------------|
| 0/7 | 1512 MHz (no thermal cap) |
| 1/7 | 1416 MHz |
| 2/7 | 1344 MHz |
| 3/7 | **1200 MHz** |
| 4/7 | 1008 MHz |
| 5/7 | 792 MHz |
| 6/7 | 600 MHz |
| 7/7 | 480 MHz |

Under sustained 4-core stress (2026-06-17, `stress-ng --cpu 4`), behavior was:

- CPU temp **81.9 – 87.1°C** (mean 84.6°C)
- **~52%** of 1 Hz samples showed cooling state **3/7** at **1200 MHz**
- **~35%** at **1512 MHz** (cooling state 0/7)
- Deeper states (4–7) were **not** observed — small overshoot above 85°C plus 2°C hysteresis causes a **~2 s bang-bang oscillation** between 1512 and 1200 MHz rather than sustained deep throttling

The 30-second `argo_thermal_monitor` logs round to integer °C and do not record frequency or cooling state, so they under-report this oscillation.

### Quick thermal + frequency check

```bash
# One-shot
echo "temp=$(($(cat /sys/class/thermal/thermal_zone2/temp)/1000))°C \
  cooling=$(cat /sys/class/thermal/cooling_device0/cur_state)/7 \
  freq=$(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)/1000))MHz \
  max=$(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq)/1000))MHz"

# 1 Hz for 15 s (good for seeing bang-bang throttling)
for i in $(seq 15); do
  printf '%s  %5.1f°C  %s/7  %4s MHz\n' "$(date +%H:%M:%S)" \
    "$(echo "scale=1; $(cat /sys/class/thermal/thermal_zone2/temp)/1000" | bc)" \
    "$(cat /sys/class/thermal/cooling_device0/cur_state)" \
    "$(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)/1000))"
  sleep 1
done
```

Persistent thermal logs: `/var/log.hdd/persistent/thermal-YYYYMMDD.log` (see [README-persistent-logging.md](README-persistent-logging.md)).

---

## Files

**Note:** The service and script are installed on the robot filesystem only; they are **not** tracked in the argo git repo.

| Item | Path |
|------|------|
| Systemd unit | `/etc/systemd/system/cpufreq-tuning.service` |
| Boot script | `/usr/local/bin/cpufreq-tuning.sh` |
| Service log | `/var/log/cpufreq-tuning.log` |

### Boot script (reference)

```bash
#!/bin/bash
for cpu in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
    echo conservative > "$cpu" 2>/dev/null || true
done
echo 80  > /sys/devices/system/cpu/cpufreq/conservative/up_threshold
echo 20  > /sys/devices/system/cpu/cpufreq/conservative/down_threshold
echo 100000 > /sys/devices/system/cpu/cpufreq/conservative/sampling_rate
echo 1  > /sys/devices/system/cpu/cpufreq/conservative/sampling_down_factor
echo 0  > /sys/devices/system/cpu/cpufreq/conservative/ignore_nice_load
echo 0  > /sys/devices/system/cpu/cpufreq/conservative/freq_step
```

### Dependencies

- **After:** `cpufrequtils.service`
- **Type:** oneshot, `RemainAfterExit=yes`
- **Install:** `WantedBy=multi-user.target` (enabled on current robot)

---

## Usage

### Service management

```bash
sudo systemctl status cpufreq-tuning.service
sudo systemctl start cpufreq-tuning.service   # re-apply settings
sudo systemctl disable cpufreq-tuning.service # stop applying at boot
```

### Verify configuration

```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
cat /sys/class/thermal/cooling_device0/{cur_state,max_state}
```

### Monitor under load

```bash
# Optional 1-minute stress + sampling (requires stress-ng)
stress-ng --cpu 4 --timeout 60s &
for i in $(seq 60); do
  printf '%s  %5.1f°C  %s/7  %4s MHz\n' "$(date +%H:%M:%S)" \
    "$(echo "scale=1; $(cat /sys/class/thermal/thermal_zone2/temp)/1000" | bc)" \
    "$(cat /sys/class/thermal/cooling_device0/cur_state)" \
    "$(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)/1000))"
  sleep 1
done
wait
```

---

## Monitoring integration

| Service | Role |
|---------|------|
| `cpufreq-tuning.service` | Boot-time governor policy |
| `argo_thermal_monitor.service` | Logs GPU/VE/CPU/DDR temps every 30 s; warns at >80°C |
| `orangepi-monitor.service` (optional) | Logs MHz, load, temp, cooling state (`C.St.`) every ~5 s |

---

## Troubleshooting

### Governor not conservative after boot

```bash
sudo systemctl status cpufreq-tuning.service cpufrequtils.service
sudo /usr/local/bin/cpufreq-tuning.sh
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### High temps but frequency stays at 1512 MHz

Expected between throttle bursts: thermal hysteresis releases the cap quickly. Sample at **1 Hz** and check `cooling_device0/cur_state`, not just the thermal log.

### Reverting to stock ondemand

```bash
sudo systemctl disable cpufreq-tuning.service
echo ondemand | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
sudo systemctl restart cpufrequtils
```

---

## Performance impact

- **Positive:** Less time at max frequency under moderate load; better stability during sustained desktop/agent work
- **Trade-off:** Slower response when load ramps gradually (80% threshold)
- **Under full CPU stress:** Still reaches 1512 MHz whenever thermal cooling is at state 0/7
- **Thermal ceiling:** Kernel may cap at 1200 MHz intermittently above 85°C regardless of governor

---

## Related documentation

- [dvfs_userspace_access.md](dvfs_userspace_access.md) — OPP table, voltage rails, sysfs read/write
- [README-thermal-monitoring.md](README-thermal-monitoring.md) — `argo_thermal_monitor` logging
- [README-persistent-logging.md](README-persistent-logging.md) — `/var/log.hdd/persistent/` log layout

---

## History

| Date | Change |
|------|--------|
| 2025-09-25 | Initial `cpufreq-tuning.service` — conservative governor to reduce hangs during Cursor agent work |
| 2026-06-17 | Documented kernel thermal layer, OPP table, cooling-state mapping, and observed bang-bang throttling behavior |
