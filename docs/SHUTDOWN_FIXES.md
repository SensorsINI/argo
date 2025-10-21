# ROS2 Shutdown Handling Fixes

## Problems Identified

From the terminal output, we saw several shutdown-related issues:

1. **"Failed to publish log message to rosout: publisher's context is invalid"** - Nodes trying to log after ROS2 context shutdown
2. **"rcl_shutdown already called on the given context"** - Multiple calls to `rclpy.shutdown()`
3. **Uncoordinated shutdown** - Signal handler calling `stop()` while continuous loop also handles shutdown

## Root Causes

### 1. Double rclpy.shutdown() Calls
The lifecycle manager's signal handler called `_cleanup_ros2()` which called `rclpy.shutdown()`, but then individual nodes also called `rclpy.shutdown()` in their own cleanup, causing the "rcl_shutdown already called" error.

### 2. Logging After Context Shutdown
When `rclpy.shutdown()` is called, it invalidates the ROS2 context. Nodes that try to publish log messages after this point get the "publisher's context is invalid" error.

### 3. Race Conditions
The signal handler immediately called `stop()` and `sys.exit()`, racing with the continuous monitoring loop's own shutdown handling.

## Fixes Applied

### 1. Added Shutdown Coordination Flag

```python
def __init__(self, quiet: bool = True):
    # ... existing code ...
    self.shutdown_requested = False  # Flag to coordinate shutdown
```

This flag allows all parts of the system to check if shutdown has been requested and avoid duplicate cleanup.

### 2. Improved Signal Handler

**Before:**
```python
def _signal_handler(self, signum, frame):
    print(f"\n🛑 argo_lifecycle_manager: Received signal {signum}, shutting down...")
    self._cleanup_ros2()  # BAD: Immediate cleanup
    self.stop()           # BAD: Races with main loop
    sys.exit(0)           # BAD: Abrupt exit
```

**After:**
```python
def _signal_handler(self, signum, frame):
    if self.shutdown_requested:
        return  # Already shutting down
    
    self.shutdown_requested = True
    print(f"\n🛑 argo_lifecycle_manager: Received signal {signum}, shutting down...")
    
    # Don't call stop() here - let the main loop handle it
    # Just set the flag and let continuous() exit cleanly
```

The signal handler now just sets a flag and lets the main loop handle the shutdown gracefully.

### 3. Protected ROS2 Cleanup

```python
def _cleanup_ros2(self):
    """Clean up ROS2 resources - only call this once at final shutdown"""
    if self.ros2_node:
        try:
            # Destroy node first
            self.ros2_node.destroy_node()
        except Exception:
            # Silently fail - context may already be shutdown
            pass
        self.ros2_node = None
    
    # Only shutdown context if it's still valid
    if ROS2_AVAILABLE and rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            # Silently fail - may already be shutdown
            pass
```

Now checks if `rclpy.ok()` before calling `shutdown()` to avoid double-shutdown errors.

### 4. Graceful Main Loop Exit

**Before:**
```python
while rclpy.ok() if self.ros2_node else True:
    rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
    # ... monitoring code ...

except KeyboardInterrupt:
    self.stop()
    return True
```

**After:**
```python
while not self.shutdown_requested and (rclpy.ok() if self.ros2_node else True):
    if self.ros2_node:
        try:
            rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
        except Exception:
            # Context may be shutting down
            if self.shutdown_requested:
                break
    # ... monitoring code ...

except KeyboardInterrupt:
    self.shutdown_requested = True
finally:
    # Publish final status before cleanup
    self._publish_status_update("Argo lifecycle manager stopping")
    
    # Stop all nodes first
    self.stop()
    
    # Clean up ROS2 last
    self._cleanup_ros2()
    
    return True  # Always return True for clean exit
```

Key improvements:
- Check `shutdown_requested` flag in loop condition
- Wrap `spin_once()` in try/except to handle context shutdown
- Use `finally` block to ensure cleanup always happens
- Stop nodes before cleaning up ROS2 context

### 5. Selective Cleanup in main()

```python
try:
    if args.command == 'run':
        # continuous() handles its own cleanup
        success = manager.continuous()
        sys.exit(0 if success else 1)
    elif args.command == 'stop':
        success = manager.stop()
        # Clean up ROS2 for non-continuous commands
        manager._cleanup_ros2()
        sys.exit(0 if success else 1)
    # ... other commands also call cleanup ...
except Exception as e:
    print(f"❌ Unexpected error: {e}")
    manager._cleanup_ros2()
    sys.exit(1)
```

Only `continuous()` and `simulate()` methods handle their own cleanup. Other commands call `_cleanup_ros2()` explicitly.

## Expected Behavior After Fixes

### Clean Shutdown Sequence

1. **Signal received** (SIGINT/SIGTERM)
2. **Flag set** (`shutdown_requested = True`)
3. **Main loop exits** (checks flag, breaks out of loop)
4. **Status published** ("Argo lifecycle manager stopping")
5. **Nodes stopped** (`self.stop()` - terminates processes)
6. **ROS2 cleaned up** (`self._cleanup_ros2()` - destroys node, shuts down context)
7. **Exit** (returns True, exits cleanly)

### No More Errors

- ✅ No "rcl_shutdown already called" errors
- ✅ No "publisher's context is invalid" errors
- ✅ No race conditions between signal handler and main loop
- ✅ Clean, orderly shutdown of all nodes

## Testing the Fixes

### Test 1: Normal Shutdown (Ctrl+C)
```bash
python3 launch/argo_lifecycle_manager.py run
# Wait for nodes to start
# Press Ctrl+C
# Should see clean shutdown without errors
```

### Test 2: Signal-Based Shutdown
```bash
python3 launch/argo_lifecycle_manager.py run &
PID=$!
sleep 10
kill -TERM $PID  # Send SIGTERM
# Should see clean shutdown
```

### Test 3: Service-Based Stop
```bash
# Terminal 1:
python3 launch/argo_lifecycle_manager.py run

# Terminal 2:
ros2 service call /argo/lifecycle/stop std_srvs/srv/Trigger
# Should stop cleanly
```

## Node Shutdown Issues - FIXED

Individual nodes (like `argo_transform_publisher.py`, `sailing_area_publisher.py`, `argo_boat_visualization.py`) have been updated to follow the proper shutdown pattern and should no longer show "rcl_shutdown already called" errors.

### Recommended Node Pattern

Nodes should check if context is still valid before calling shutdown:

```python
def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass  # Context already shutdown
    finally:
        node.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass  # Already shutdown
```

This pattern has been applied to the following nodes:
- ✅ `argo_transform_publisher.py` - Updated with proper shutdown handling
- ✅ `sailing_area_publisher.py` - Updated with proper shutdown handling  
- ✅ `argo_boat_visualization.py` - Updated with proper shutdown handling

Other ROS2 nodes in the system should follow this same pattern if they show shutdown errors.

## Benefits

1. **Clean Logs** - No more confusing error messages during shutdown
2. **Reliable** - Shutdown always completes successfully
3. **Coordinated** - All parts of system agree on shutdown state
4. **Debuggable** - Clear, predictable shutdown sequence
5. **Robust** - Handles edge cases (double signals, quick Ctrl+C, etc.)

