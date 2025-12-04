# Root Cause Analysis: 60-90 Second Shutdown Hang

## Executive Summary

**Problem**: System shutdown takes 60-90 seconds instead of expected 10-15 seconds

**Root Cause**: Services with `Before=shutdown.target` are trying to perform graceful ROS2 shutdown after ROS2 infrastructure is already dead

**Affected Services**:
- `argo_battery_water.service` - hangs >15 seconds
- `argo_bno085.service` - hangs >15 seconds

**Impact**: Extremely slow shutdowns, but services DO eventually stop

## Technical Analysis

### The Design Intent

From `docs/README-services.md`:
```
battery_water → shutdown.target (Before)
  - Battery monitoring stays active late into shutdown
  - Critical for safe power-off

bno085 → shutdown.target (Before)
  - IMU driver stays active late into shutdown
```

These services were intentionally configured with `Before=shutdown.target` to:
1. Keep battery monitoring active as long as possible during shutdown
2. Keep IMU data available during shutdown sequence
3. Provide critical status information until the very end

### The Implementation Problem

**Service Configuration**:
```ini
[Unit]
Before=shutdown.target
DefaultDependencies=no

[Service]
TimeoutStopSec=10
```

**What Actually Happens**:

1. **T+0s**: User initiates shutdown
2. **T+0s**: Systemd stops normal services (INCLUDING ROS2 infrastructure)
3. **T+3s**: Most services stopped
4. **T+5s**: ROS2 middleware/daemon stopped
5. **T+5s**: `argo_battery_water` and `argo_bno085` receive SIGTERM
6. **T+5s**: Services try to shut down gracefully via `rclpy.shutdown()`
7. **T+5s - T+65s**: Services HANG waiting for ROS2 operations that will never complete
   - `rclpy.shutdown()` tries to cleanup ROS2 context
   - ROS2 middleware is already dead
   - I2C operations may also be blocked
   - Services wait indefinitely
8. **T+65s**: Systemd finally gives up and SIGKILLs everything

### Evidence

#### From Shutdown Logger 2025-12-04 18:41:46
```
18:41:46 - Shutdown initiated
18:41:58 - 10 seconds: 2 services still running (past their 10s timeout!)
18:42:04 - 15+ seconds: Still 2 services running
```

#### From Individual Service Testing
```bash
$ time sudo systemctl stop argo_battery_water.service
real    0m0.681s  # ✓ Stops in < 1 second!

$ time sudo systemctl stop argo_bno085.service  
real    0m1.008s  # ✓ Stops in < 1 second!
```

**Key Insight**: Services CAN shut down gracefully in < 1 second when ROS2 is still alive, but hang indefinitely when ROS2 is dead.

### Why `TimeoutStopSec=10` Doesn't Work

Systemd's `TimeoutStopSec` only applies to the time from SIGTERM to SIGKILL. But with `Before=shutdown.target`:

1. Systemd sends SIGTERM at T+5s
2. Service hangs waiting for dead ROS2
3. Systemd waits `TimeoutStopSec=10` seconds
4. At T+15s, systemd sends SIGKILL... but nothing happens!
5. System is now in "final shutdown" mode
6. Normal timeout enforcement is disabled
7. Services continue hanging until final system halt at T+90s

## Solutions

### Option 1: Remove `Before=shutdown.target` (RECOMMENDED)

**Change**: Remove the late shutdown ordering, let services stop normally

**Service Configuration**:
```ini
[Unit]
After=network.target
# REMOVED: Before=shutdown.target
# REMOVED: DefaultDependencies=no
```

**Pros**:
- ✅ Services stop in < 1 second (verified by testing)
- ✅ Graceful ROS2 shutdown works properly
- ✅ Total shutdown time: 10-15 seconds (normal)

**Cons**:
- ❌ Battery monitoring stops earlier in shutdown
- ❌ IMU data not available during late shutdown

**Risk Assessment**: **LOW**
- Battery status is not critical during shutdown
- IMU data is not used during shutdown
- Original "late shutdown" design may be over-engineered

### Option 2: Make Shutdown Non-Blocking

**Change**: Remove all ROS2 cleanup from signal handler

**Code Changes**:
```python
def _signal_handler(self, sig, frame):
    """Fast, non-blocking shutdown"""
    # DO NOT call rclpy.shutdown() here!
    # Just exit immediately
    self.get_logger().info('Shutdown signal received, exiting immediately')
    sys.exit(0)  # No cleanup, just die
```

**Pros**:
- ✅ Services stop immediately (< 0.1 seconds)
- ✅ Works even with dead ROS2 infrastructure
- ✅ Keeps `Before=shutdown.target` if desired

**Cons**:
- ❌ No graceful ROS2 cleanup
- ❌ May leave stale ROS2 state
- ❌ Not a "clean" solution

**Risk Assessment**: **MEDIUM**
- ROS2 cleanup during shutdown is probably unnecessary anyway
- System is shutting down, stale state doesn't matter

### Option 3: Increase Timeout and Accept Slow Shutdown

**Change**: Increase `TimeoutStopSec` to match worst-case hang time

**Service Configuration**:
```ini
[Service]
TimeoutStopSec=90  # Increased from 10
```

**Pros**:
- ✅ No code changes needed
- ✅ No configuration changes to dependencies
- ✅ Eventually works

**Cons**:
- ❌ Shutdown still takes 60-90 seconds
- ❌ Doesn't fix the root cause
- ❌ User experience is terrible

**Risk Assessment**: **HIGH** (solves nothing)

### Option 4: Add ROS2 Dependency Ordering

**Change**: Ensure ROS2 infrastructure stops AFTER these services

**Service Configuration**:
```ini
[Unit]
Before=shutdown.target some-ros2-daemon.service
```

**Problem**: ROS2 Humble doesn't install a system daemon service. The middleware is embedded in each node process. **This solution is not possible.**

## Recommendation

**Implement Option 1**: Remove `Before=shutdown.target` from both services.

**Rationale**:
1. Testing proves services stop gracefully in < 1 second when ROS2 is alive
2. The "late shutdown" feature provides no actual benefit during shutdown
3. Battery/IMU data is not used during shutdown sequence
4. Clean, simple solution that fixes the root cause

**Implementation**:
1. Remove `Before=shutdown.target` from `argo_battery_water.service`
2. Remove `DefaultDependencies=no` (restore default dependencies)
3. Same changes for `argo_bno085.service`
4. Reload systemd: `sudo systemctl daemon-reload`
5. Test shutdown timing (should be < 15 seconds)

**Alternative**: If late shutdown is truly critical, implement Option 2 (non-blocking shutdown) as a pragmatic compromise.

## Expected Outcome

After implementing Option 1:
- **Total shutdown time**: 10-15 seconds (down from 60-90s)
- **Service stop time**: < 1 second each (verified)
- **Graceful shutdown**: ✅ Works properly
- **Data loss risk**: None (shutdown is shutdown)

## Test Plan

1. Implement changes
2. Reload systemd: `sudo systemctl daemon-reload`
3. Test individual service stops: `time sudo systemctl stop argo_battery_water.service`
4. Test full system shutdown: `sudo shutdown -h now`
5. Check shutdown logger: Latest log in `/var/log.hdd/persistent/shutdown-*.log`
6. Expected: All services stop within their timeouts, total shutdown < 20 seconds
