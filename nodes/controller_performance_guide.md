# Controller Performance Optimization Guide

## CPU Usage Optimizations Applied

### 1. Control Loop Frequency Reduction
- **Changed**: 10 Hz → 5 Hz (200ms period)
- **Rationale**: 5 Hz is sufficient for sailboat control while reducing CPU load by ~50%
- **Trade-off**: Slightly slower response time (100ms → 200ms)

### 2. Conditional Debug Logging
- **Added**: Check for debug level before string formatting
- **Benefit**: Eliminates expensive operations when debug logging disabled
- **Code**: `if self.get_logger().get_effective_level() <= 10:`

### 3. Parameter File Monitoring
- **Changed**: 3s → 10s check interval + optional disable
- **New Parameter**: `enable_param_reload: false` to disable completely
- **Benefit**: Reduces file I/O overhead by 70%

### 4. QoS Buffer Optimization
- **Changed**: Standard QoS depth 10 → 5
- **Benefit**: 50% reduction in message buffer memory
- **Trade-off**: Slightly less message history buffering

### 5. CPU Monitoring
- **Added**: Built-in performance monitoring every 30 seconds
- **Output**: CPU % and memory usage in logs
- **Benefit**: Early detection of performance issues

## Configuration for Maximum Performance

### Minimal CPU Usage Configuration
```yaml
controller_node:
  ros__parameters:
    enable_param_reload: false  # Disable parameter file monitoring
    controller_type: proportional  # Use simplest controller
    data_collection_enabled: false  # Disable data collection
```

### Balanced Performance Configuration (Default)
```yaml
controller_node:
  ros__parameters:
    enable_param_reload: true   # Enable with 10s interval
    controller_type: proportional
    data_collection_enabled: false
```

## Monitoring Commands

### Check Controller CPU Usage
```bash
# View controller process CPU usage
ps aux | grep controller.py

# Monitor real-time CPU usage
top -p $(pgrep -f controller.py)

# Check systemd service CPU consumption
systemctl status argo-launch.service | grep "CPU time"
```

### View Controller Performance Logs
```bash
# Monitor controller CPU reports
journalctl -u argo-launch.service -f | grep "Controller CPU"

# Check for performance warnings
journalctl -u argo-launch.service | grep -E "(CPU|Memory|performance)"
```

## Performance Troubleshooting

### High CPU Usage Indicators
1. **Control loop running too fast**: Check if timer period is set correctly
2. **Excessive logging**: Verify debug level settings
3. **Parameter file issues**: Check if param file exists and is readable
4. **QoS buffer overflow**: Monitor subscription queue depths

### Optimization Checklist
- [ ] Set `enable_param_reload: false` for production
- [ ] Use `controller_type: proportional` for lowest CPU
- [ ] Disable debug logging in production
- [ ] Monitor CPU usage logs every 30 seconds
- [ ] Check for memory leaks in long-running sessions

## Expected Performance Metrics

### Before Optimization
- Control loop: 10 Hz (100ms)
- Parameter checks: Every 3 seconds
- Debug logging: Always formatted
- QoS depth: 10 messages
- **Typical CPU**: 15-25%

### After Optimization
- Control loop: 5 Hz (200ms)
- Parameter checks: Every 10 seconds (or disabled)
- Debug logging: Conditional
- QoS depth: 5 messages
- **Expected CPU**: 5-10%

## Further Optimizations (If Needed)

### 1. Reduce Control Loop to 2 Hz
```python
self.control_loop_period = 0.5  # 2 Hz
```

### 2. Disable Data Collection Completely
```python
# Remove data collector initialization
# self.data_collector = DataCollector(training_data_dir)
```

### 3. Use Best Effort QoS for Non-Critical Topics
```python
# For topics that don't need reliability
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)
```

### 4. Implement Message Filtering
```python
# Only process messages when control mode changes
if self.boat_state.human_controlled != self.last_control_mode:
    # Process control logic
```

## Performance Testing

### Load Testing
```bash
# Start controller and monitor CPU
sudo systemctl start argo-launch.service
watch -n 1 'ps aux | grep controller.py | grep -v grep'
```

### Memory Leak Detection
```bash
# Monitor memory usage over time
while true; do
  ps aux | grep controller.py | grep -v grep
  sleep 60
done
```

### Stress Testing
```bash
# Simulate high message rates
ros2 topic pub /pose geometry_msgs/msg/Vector3 "x: 0.0, y: 0.0, z: 0.0" --rate 100
```
