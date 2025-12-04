# Shutdown Timing Analysis

## Overview

This document analyzes shutdown behavior and timing issues discovered through the shutdown logger service.

## Key Findings from Shutdown 2025-12-04 18:41:46

### Timeline

```
18:41:46 - Shutdown initiated
18:41:46 - Initial state: 2 Argo services active (battery_water, bno085)
18:41:52 - 5 seconds: 2 services still running
18:41:58 - 10 seconds: 2 services still running (past their 10s timeout!)
18:42:01 - 15 seconds: argo_health_monitor FORCE-KILLED with SIGKILL
           - Process 5976 (python3) killed
           - Process 6062 (python3) killed
18:42:04 - Logger still monitoring, 2 services still running
~18:42:04+ - Shutdown logger killed (log ends abruptly)
Total: >60 seconds for full shutdown
```

### Services That Hung

#### argo_health_monitor (CONFIRMED FORCE-KILLED)
- **Timeout**: 15s
- **Behavior**: Force-killed with SIGKILL at exactly 15 seconds
- **Processes killed**: 2 Python processes (PIDs 5976, 6062)
- **Status**: This is EXPECTED behavior - service hit its timeout

#### argo_battery_water (SUSPECTED HANGING)
- **Timeout**: 10s
- **Behavior**: Still running at 15+ seconds (past its 10s timeout!)
- **Status**: This service is NOT stopping gracefully within its timeout
- **Action needed**: Investigate why graceful shutdown is failing

#### argo_bno085 (SUSPECTED HANGING)
- **Timeout**: 10s
- **Behavior**: Still running at 15+ seconds (past its 10s timeout!)
- **Status**: This service is NOT stopping gracefully within its timeout
- **Action needed**: Investigate why graceful shutdown is failing

### Why argo_health_monitor Was Force-Killed

The health monitor service has:
- `TimeoutStopSec=15` in its service file
- `KillMode=control-group` (correct setting)
- Two Python processes running (main + subprocess?)

When it didn't stop within 15 seconds, systemd correctly sent SIGKILL to both processes.

**This is EXPECTED behavior** - the service is working as configured.

### Why battery_water and bno085 Are Hanging

These services have `TimeoutStopSec=10`, but were still running at 15+ seconds.

**Possible causes:**
1. **Signal handler not working properly**
   - The services may not be responding to SIGTERM
   - Check that `rclpy.spin_once()` loop is checking `shutdown_requested` flag

2. **Blocking operations during shutdown**
   - I2C operations might be blocking
   - File I/O might be slow
   - ROS2 cleanup might be hanging

3. **Child processes not terminating**
   - Check if services spawn subprocesses that don't die with parent
   - Verify `KillMode=control-group` is set correctly

## Recommendations

### 1. Fix battery_water and bno085 Graceful Shutdown

Verify these services use the same shutdown pattern as we fixed earlier:

```python
# In signal handler
def _signal_handler(self, sig, frame):
    self._shutdown_requested = True
    self.shutdown_requested = True
    rclpy.shutdown()

# In main loop
while rclpy.ok() and not node.shutdown_requested:
    rclpy.spin_once(node, timeout_sec=0.1)
```

### 2. Increase Timeouts if Necessary

If services genuinely need more time to shut down gracefully:
- Consider increasing `TimeoutStopSec` from 10s to 15s or 20s
- But first, investigate WHY they're taking so long

### 3. Investigate health_monitor Force-Kill

The health monitor is being force-killed every time. Options:
1. **Increase timeout** if 15s isn't enough for graceful shutdown
2. **Fix shutdown logic** if it should be stopping faster
3. **Accept force-kill** if it's not causing problems (data loss, etc.)

### 4. Improve Shutdown Logger

The logger itself was killed before capturing the full shutdown. Improvements:
- Increase logger's own timeout beyond 90s
- Move logger to run LATER in shutdown sequence
- Use a more persistent logging mechanism (direct file writes, no journald)

## Service Shutdown Configuration Reference

| Service | Timeout | KillMode | Graceful Shutdown Status |
|---------|---------|----------|-------------------------|
| argo_launch_standard | 15s | control-group | Unknown |
| argo_power_control | 30s | control-group | Fixed (graceful) |
| argo_battery_water | 10s | control-group | **FAILING** (hangs >15s) |
| argo_bno085 | 10s | control-group | **FAILING** (hangs >15s) |
| argo_health_monitor | 15s | control-group | Force-killed at timeout |
| argo_radio_servo_module | 30s | control-group | Unknown |

## Next Steps

1. **Test battery_water shutdown** in isolation:
   ```bash
   sudo systemctl stop argo_battery_water.service
   # Check how long it actually takes
   time sudo systemctl stop argo_battery_water.service
   ```

2. **Test bno085 shutdown** in isolation:
   ```bash
   time sudo systemctl stop argo_bno085.service
   ```

3. **Check if these services have the spin_once() fix** from the previous graceful shutdown work

4. **Monitor next shutdown** with improved logger to see if fixes work

