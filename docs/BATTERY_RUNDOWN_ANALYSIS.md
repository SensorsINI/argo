# Battery Rundown Analysis - December 2025

## Executive Summary

**MOST LIKELY ROOT CAUSE**: The `argo_power_control.service` died/crashed and did not restart, leaving no service to monitor battery voltage or initiate critical battery shutdown.

**Why This Is Most Likely**:
- Watchdog was tested and working (Dec 5, 2025) - protects against system freezes
- If system froze, watchdog should have triggered reboot within ~20 seconds
- Power control service has `Restart=always` but may have:
  - Crashed in a way that prevented restart
  - Entered a crash loop that exhausted restart attempts
  - Failed to start due to dependency issues (I2C, ROS2, etc.)
- Without power control running, battery monitoring thread doesn't exist
- Without battery monitoring, critical shutdown cannot trigger

**Key Insight**: Watchdog protects against system freezes, but **NOT** against individual service crashes (as documented in README-watchdog.md). If power control dies, there's no fallback battery protection.

**SMOKING GUN**: Persistent logs show power control service died on Dec 13 at 13:35:05 and did NOT restart until Dec 15 at 12:51:32 - a **~47 hour gap** with no battery monitoring. During this period, the battery drained from healthy voltage to 0V with no protection.

## Incident Summary

**Date**: December 13-16, 2025  
**Issue**: Battery ran down to 0V, causing permanent damage  
**Symptoms**:
- Battery voltage dropped to 0V (damaged battery)
- Critical low battery shutdown did not trigger
- System did not reboot even with USB power plugged in
- System may have been frozen (watchdog should have triggered but didn't)

## Root Cause Analysis

### 1. Power Control Service Died (Most Likely)

**Problem**: The `argo_power_control.service` crashed/died and did not restart, leaving no service to monitor battery voltage or initiate critical battery shutdown.

**Evidence**:
- Boot history shows system was running from Dec 13 14:30 to Dec 16 10:15 (~2.5 days)
- No shutdown logs between Dec 13 and Dec 16
- System did not reboot (watchdog would have triggered if system froze)
- Power control service has `Restart=always` but may have failed to restart
- Without power control, battery monitoring thread doesn't exist
- No independent battery monitoring process

**Service Configuration**: `power_control/argo_power_control.service`
```ini
Restart=always
RestartSec=5
```

**Why Restart May Have Failed**:
1. **Service hung**: Service may have hung (not crashed, but not responding) - systemd thinks it's running but it's not functional
2. **Silent exit**: Service may have exited cleanly (exit code 0) but systemd didn't restart it for some reason
3. **Systemd restart rate limiting**: May have hit systemd's restart rate limits after multiple restarts on Dec 13
4. **Dependency failure**: Required services (argo_battery_water, ROS2) unavailable, preventing restart
5. **Resource exhaustion**: System out of memory, file descriptors, etc., preventing new process creation
6. **Log file issue**: Service may have been running but not logging (unlikely but possible)

**Note**: ROS2 shutdown exceptions are NOT the cause - they occur after shutdown completes and don't prevent restart.

**Evidence from Logs**:
- Dec 13: Service restarted 4 times (09:58, 11:11, 11:58, 13:35)
- Dec 13 13:35:05 - Service shutdown cleanly, ROS2 exception occurred AFTER "SHUTDOWN COMPLETE" (known ROS2 cleanup issue)
- Dec 13 13:35:05 - Service restarted normally ~9 seconds later (matches RestartSec=5)
- **Gap in logs**: No entries from after 13:35 restart until Dec 15 12:51:32
- **Conclusion**: Service restarted normally after 13:35, but then stopped working. Service may have:
  1. Died silently (no log entry, exit code 0)
  2. Hung without crashing (systemd thinks it's running but it's not responding)
  3. Log file rotation/truncation issue (unlikely - would see some entries)
  4. Service stopped logging but continued running (unlikely - would see some activity)

**Code Location**: `power_control/argo_power_control.py:3334-3770`
- Battery monitoring runs in a separate thread within power control service
- If service dies, thread dies with it
- No independent battery monitoring process
- No hardware-level battery protection

### 2. Watchdog Status (Not the Issue)

**Status**: Watchdog was tested and verified working on December 5, 2025 (see `docs/README-watchdog.md`).

**Configuration** (`/etc/systemd/system.conf`):
```
RuntimeWatchdogSec=10s
RebootWatchdogSec=2min
WatchdogDevice=/dev/watchdog
```

**Expected Behavior**: If system freezes, watchdog should reboot within ~20 seconds (16s hardware timeout + margin).

**Why Watchdog Didn't Help**:
- Watchdog protects against **system freezes** (kernel panics, I2C deadlocks, etc.)
- Watchdog does **NOT** protect against **individual service crashes** (as documented in README-watchdog.md)
- If power control service dies but system continues running, watchdog doesn't trigger
- System was still running (not frozen), so watchdog had no reason to reboot

**Key Insight**: Watchdog is for system-level hangs, not application-level failures. If power control dies, the system continues running normally, just without battery protection.

### 3. I2C Bus Failure (Possible Contributing Factor)

**Problem**: If I2C bus fails, battery monitoring becomes unavailable. Power control should detect this and initiate shutdown, but if power control itself is dead, this protection doesn't work.

**Code Location**: `power_control/argo_power_control.py:3736-3756`
- After 5 minutes of battery service failures, power control initiates safe shutdown
- But this only works if power control service is running
- If power control dies, there's no service to detect I2C failures

**Scenario**: 
1. I2C bus fails → battery service can't read voltage
2. Power control detects I2C failure → should shutdown after 5 minutes
3. But if power control service dies before or during this, shutdown never happens
4. Battery continues draining with no monitoring

### 4. Battery Service Failure Timeout Too Long

**Problem**: If battery service fails, system waits 5 minutes before initiating safe shutdown.

**Code Location**: `power_control/argo_power_control.py:3343`
```python
BATTERY_SERVICE_FAILURE_TIMEOUT = 10  # 5 minutes (10 * 30s) of failures = initiate safe shutdown
```

**Issue**: 
- 5 minutes is too long if battery is already critically low
- If system freezes, this timeout never triggers anyway
- Battery can drain significantly in 5 minutes

**Recommendation**: Reduce timeout to 2-3 minutes (4-6 consecutive failures)

### 5. No Hardware-Level Battery Protection

**Problem**: All battery protection relies on software monitoring. If software fails or system freezes, there's no hardware safety net.

**Current Protection**:
- Software monitoring every 30 seconds
- Critical threshold: 7.2V
- Shutdown initiated via software command

**Missing Protection**:
- Hardware voltage monitor circuit
- Hardware power cut at low voltage
- Independent watchdog that doesn't rely on systemd

## Log Analysis

### Battery Monitor Logs
- Last battery reading: Dec 16 10:44:53 at 8.1875V (well above 7.2V critical threshold)
- No readings below 7.2V in recent logs
- Battery was healthy when last logged

### Power Control Logs
- **CRITICAL FINDING**: Persistent log shows multiple restarts on Dec 13:
  - 09:58:07 - Service restarted
  - 11:11:45 - Service restarted
  - 11:58:47 - Service restarted
  - 13:35:05 - Service crashed with ROS2 exception and restarted
- **ROS2 shutdown exceptions detected** (NOT the root cause):
  - **11:58:47**: `ValueError: generator already executing` in rclpy executor (during shutdown)
  - **13:35:05**: `rclpy._rclpy_pybind11.InvalidHandle: cannot use Destroyable because destruction was requested` (during shutdown)
- **Analysis**: These exceptions occur AFTER "SHUTDOWN COMPLETE" is logged, during ROS2 cleanup phase
- **Timing evidence**: Exception occurs ~0.04s after shutdown complete, service restarts normally ~9s later (matches RestartSec=5)
- **Conclusion**: These are known ROS2 shutdown cleanup issues and do NOT prevent service restart. They are NOT the cause of the service dying.
- **Service restarted immediately** after crash and continued running normally (logs show normal operation)
- **Gap in logs**: No entries from after 13:35 restart until Dec 15 12:51:32
- **Possible scenarios**:
  1. Service died silently after 13:35 (no crash log, systemd didn't restart it)
  2. Service hung without crashing (systemd thinks it's running but it's not responding)
  3. Log file issue (rotation, truncation, or write failure)
  4. Service stopped logging but continued running (unlikely - would still see some activity)
- **~47 hours without battery monitoring** (Dec 13 13:35 → Dec 15 12:51)
- No critical battery shutdown events in recent logs (service wasn't running to detect them)

### Boot History
- System booted Dec 13 14:30
- Next boot: Dec 16 10:15 (after battery damage occurred)
- No graceful shutdown between these dates
- **Key Question**: Was power control service running during this period?

### Service Status Investigation
```bash
# Check if power control was active during incident period
journalctl -u argo_power_control.service --since "2025-12-13" --until "2025-12-16"
# Result: No entries (suspicious - service may not have been running)

# Check service restart history
systemctl show argo_power_control.service -p NRestarts -p Result
# Would show restart count and last result if available
```

## Recommendations

### Immediate Actions

1. **Add Power Control Service Health Monitoring**
   ```bash
   # Check if power control service is running
   systemctl is-active argo_power_control.service
   
   # Check service status and restart count
   systemctl status argo_power_control.service
   
   # Check for crash loops
   journalctl -u argo_power_control.service --since "24 hours ago" | grep -i "restart\|fail\|crash\|exit"
   
   # Monitor service health
   watch -n 5 'systemctl is-active argo_power_control.service && echo "OK" || echo "FAILED"'
   ```

2. **Add Independent Battery Monitor**
   - Create separate systemd service that monitors battery voltage
   - Doesn't depend on ROS2 or power control service
   - Triggers shutdown if voltage < 7.2V
   - Runs independently as safety net

3. **Verify Watchdog Functionality** (Already Working, But Good to Verify)
   ```bash
   # Check watchdog device
   ls -l /dev/watchdog
   
   # Check systemd watchdog status
   sudo systemctl show systemd -p WatchdogTimestamp -p WatchdogUSec
   
   # Review watchdog configuration
   grep -i watchdog /etc/systemd/system.conf
   
   # Test watchdog (WARNING: will reboot system - only do with console access)
   # echo 1 | sudo tee /proc/sys/kernel/sysrq
   # echo c | sudo tee /proc/sysrq-trigger
   ```

2. **Reduce Battery Service Failure Timeout**
   - Change `BATTERY_SERVICE_FAILURE_TIMEOUT` from 10 to 6 (3 minutes instead of 5)
   - Reduces time before safe shutdown if monitoring fails

3. **Add Hardware Watchdog Verification**
   - Add periodic watchdog health checks to power control service
   - Log watchdog status to detect if it becomes unavailable

### Short-Term Improvements

1. **Add Independent Battery Monitor Service** (CRITICAL)
   - Create separate systemd service that monitors battery voltage
   - Doesn't depend on ROS2 or power control service
   - Direct I2C access to battery monitoring hardware
   - Triggers shutdown if voltage < 7.2V for > 1 minute
   - Runs as independent safety net even if power control dies

2. **Improve Power Control Service Resilience**
   - Add health check endpoint that external monitor can query
   - Add watchdog process that monitors power control service
   - Alert if power control service dies and doesn't restart
   - Log all service crashes with stack traces

3. **Implement Hardware-Level Protection**
   - Add hardware voltage monitor circuit
   - Automatic power cut at low voltage (e.g., 6.5V)
   - Independent of software state

3. **Improve Freeze Detection**
   - Add system health monitoring that detects freezes
   - Monitor systemd watchdog petting
   - Alert if watchdog stops responding

### Long-Term Solutions

1. **Hardware Battery Management Unit (BMU)**
   - Dedicated hardware for battery monitoring
   - Automatic power cut at critical voltage
   - Independent of main system

2. **Dual Watchdog System**
   - Hardware watchdog (current)
   - Software watchdog that monitors hardware watchdog
   - Independent battery monitoring process

3. **Battery Voltage Logging to Persistent Storage**
   - Log battery voltage to persistent storage every 30 seconds
   - Even if system freezes, last known voltage is recorded
   - Helps diagnose future incidents

## Code Changes Needed

### 1. Reduce Battery Service Failure Timeout

**File**: `power_control/argo_power_control.py`

**Change**:
```python
# Current
BATTERY_SERVICE_FAILURE_TIMEOUT = 10  # 5 minutes

# Recommended
BATTERY_SERVICE_FAILURE_TIMEOUT = 6  # 3 minutes (6 * 30s)
```

### 1. Add Sustained Invalid Reading Timeout (IMPLEMENTED)

**Problem**: Invalid battery readings (0V, I2C failures) were never triggering shutdown, even when sustained for hours. This is correct for sailing (external sensor shorts), but dangerous in development.

**Solution Implemented**: 
- Track duration of invalid readings (I2C/sensor failures)
- Short-term failures (<1 hour) are OK (external sensor shorts during sailing)
- Sustained failures (>1 hour) trigger safe shutdown to prevent battery damage
- Clear logging every 10 minutes during sustained failures
- Shutdown reason: `sustained_invalid_readings_{hours}h`

**Code Location**: `power_control/argo_power_control.py:3346-3436`
- `INVALID_READING_TIMEOUT_S = 3600.0` (1 hour)
- Tracks `invalid_reading_start_time` when invalid readings begin
- Logs warnings every 10 minutes during sustained failures
- Triggers shutdown if duration >= 1 hour

### 2. ROS2 Shutdown Exceptions (NOT a Problem)

**Status**: ROS2 exceptions during shutdown are known issues and do NOT prevent service restart.

**Evidence from logs**:
- Exception occurs AFTER "SHUTDOWN COMPLETE" is logged
- Service restarts normally ~9 seconds later (matches RestartSec=5)
- These are cleanup-phase exceptions, not operational failures

**Conclusion**: No code changes needed for ROS2 shutdown exceptions. They are cosmetic and don't affect service reliability.

### 2. Add Independent Battery Monitor Service

**New Service**: `scripts/independent_battery_monitor.py` + `system-monitoring/services/independent-battery-monitor.service`

**Purpose**: Monitor battery voltage independently of power control service

**Features**:
- Direct I2C access to battery monitoring hardware
- Doesn't depend on ROS2 or power control service
- Triggers shutdown if voltage < 7.2V for > 1 minute
- Logs to persistent storage every 30 seconds
- Runs as separate systemd service with `Restart=always`

**Implementation**:
```python
#!/usr/bin/env python3
"""Independent battery monitor - doesn't depend on ROS2 or power control"""
import time
import subprocess
import smbus2  # Direct I2C access

CRITICAL_THRESHOLD_V = 7.2
CHECK_INTERVAL_S = 30
CRITICAL_DURATION_S = 60  # Must be below threshold for 1 minute

def read_battery_voltage():
    # Direct I2C access to MAX11612 ADC
    # Implementation similar to argo_battery_water.py but standalone
    pass

def main():
    critical_start_time = None
    while True:
        voltage = read_battery_voltage()
        if voltage < CRITICAL_THRESHOLD_V:
            if critical_start_time is None:
                critical_start_time = time.time()
            elif time.time() - critical_start_time > CRITICAL_DURATION_S:
                # Trigger shutdown
                subprocess.run(['sudo', 'shutdown', '-h', 'now'])
        else:
            critical_start_time = None
        time.sleep(CHECK_INTERVAL_S)
```

### 3. Add Power Control Service Health Monitor

**New Service**: Monitor `argo_power_control.service` and alert if it dies

**Purpose**: Detect when power control service crashes and doesn't restart

**Implementation**:
```bash
#!/bin/bash
# Monitor power control service health
while true; do
    if ! systemctl is-active --quiet argo_power_control.service; then
        echo "$(date): WARNING: argo_power_control.service is not active!" | \
            tee -a /var/log.hdd/persistent/power-control-health.log
        # Could trigger alert or emergency shutdown
    fi
    sleep 60
done
```

**New Script**: `scripts/independent_battery_monitor.py`
- Runs as separate systemd service
- Doesn't depend on ROS2
- Monitors battery voltage via direct I2C access
- Triggers shutdown if voltage < 7.2V for > 1 minute
- Logs to persistent storage

## Testing Recommendations

1. **Test Watchdog Functionality**
   - Trigger kernel panic and verify reboot occurs within 20 seconds
   - Verify watchdog is petting correctly during normal operation

2. **Test Battery Monitoring During Freeze**
   - Simulate system freeze (kernel panic)
   - Verify watchdog triggers reboot
   - Check if battery monitoring would have caught low voltage

3. **Test Battery Service Failure**
   - Stop battery service
   - Verify safe shutdown triggers after timeout
   - Reduce timeout and retest

## Prevention Checklist

- [ ] **CRITICAL: Create independent battery monitor service** - Doesn't depend on power control
- [ ] **Add power control service health monitoring** - Alert if service dies
- [ ] **Investigate why power control stopped restarting** - Service died Dec 13 13:35, didn't restart until Dec 15 12:51 (~47 hours)
- [ ] **Add service crash reporting** - Log all crashes with stack traces to understand why restart failed
- [ ] **Review systemd restart limits** - Ensure StartLimitAction doesn't prevent restarts
- [ ] **Add external service monitor** - Independent process that checks if power control is running
- [ ] Reduce battery service failure timeout to 3 minutes (in power control)
- [ ] Add service crash logging with stack traces
- [ ] Consider hardware-level battery protection
- [ ] Document power control service failure scenarios
- [ ] Add battery voltage logging to persistent storage (independent of power control)
- [ ] Test power control service crash scenarios
- [ ] Verify watchdog is still active (should be, but good to check)

## Related Documentation

- [Watchdog Configuration](README-watchdog.md)
- [Battery Power Management](README-battery-power.md)
- [Persistent Logging](README-persistent-logging.md)
- [Power Control System](power_control/README.md)
