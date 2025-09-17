# Argo Power Control - Threading Architecture

## Overview
The argo_power_control ROS2 node uses a multi-threaded architecture to separate time-critical operations from the main ROS spin loop, allowing for optimized performance and responsive LED control.

## Threading Structure

### 1. Main ROS Spin Loop (1Hz)
- **Frequency**: 1Hz (reduced from default high-frequency spinning)
- **Purpose**: Handles ROS2 message processing, service calls, and node health monitoring
- **Timeout**: 1.0 second per spin cycle
- **Benefits**: 
  - Lower CPU usage
  - Reduced system load
  - Still responsive enough for ROS2 operations

### 2. Button Monitoring Thread (1Hz)
- **Frequency**: 1Hz polling
- **Purpose**: Monitors power button state and detects press/release events
- **Independence**: Runs completely separate from ROS spin loop
- **Benefits**:
  - Responsive button detection regardless of ROS2 message load
  - Consistent timing for button press detection
  - No interference with ROS2 operations

### 3. Green LED Heartbeat Thread (Variable Frequency)
- **Normal Operation**: 1Hz heartbeat
- **Bagfile Recording**: 2Hz heartbeat (dynamic switching)
- **Purpose**: Provides visual system status indication
- **Independence**: Runs completely separate from ROS spin loop
- **Benefits**:
  - Smooth LED transitions
  - Real-time frequency changes based on recording status
  - No delays from ROS2 message processing

### 4. SOS Pattern Thread (On-Demand)
- **Frequency**: Morse code timing (··· --- ···)
- **Purpose**: SOS pattern when argo nodes are missing
- **Independence**: Runs when triggered, separate from other threads
- **Benefits**:
  - Immediate response to node health issues
  - Clear visual fault indication
  - No interference with other LED patterns

## Thread Independence Benefits

### 1. Performance Optimization
- **Main Spin Loop**: Reduced to 1Hz since critical operations are in separate threads
- **CPU Usage**: Lower overall CPU consumption
- **Responsiveness**: Button and LED operations remain highly responsive

### 2. Fault Isolation
- **Button Monitoring**: Continues even if ROS2 has issues
- **LED Control**: Maintains visual feedback regardless of ROS2 state
- **System Reliability**: Critical functions isolated from ROS2 message processing

### 3. Real-time Requirements
- **LED Timing**: Precise LED control without ROS2 interference
- **Button Detection**: Consistent button press detection timing
- **Visual Feedback**: Immediate response to system state changes

## Code Implementation

### Thread Creation
```python
# Button monitoring thread
button_thread = threading.Thread(target=self.monitor_power_button_polling, daemon=True)
button_thread.start()

# LED heartbeat thread  
heartbeat_thread = threading.Thread(target=self.green_led_heartbeat, daemon=True)
heartbeat_thread.start()
```

### Main Spin Loop
```python
# Main ROS2 spin loop - reduced frequency since button/LED processing is in separate threads
while self.running and rclpy.ok():
    rclpy.spin_once(self, timeout_sec=1.0)  # 1Hz spin frequency
```

### Dynamic LED Frequency
```python
# Adjust frequency based on bagfile recording status
if self.bagfile_recording:
    heartbeat_frequency = 2.0  # 2Hz during bagfile recording
else:
    heartbeat_frequency = LED_HEARTBEAT_HZ  # 1Hz normal operation
```

## Timing Characteristics

| Operation | Frequency | Thread | Independence |
|-----------|-----------|--------|--------------|
| ROS2 Spin Loop | 1Hz | Main | Dependent on ROS2 |
| Button Monitoring | 1Hz | Separate | Independent |
| Green LED (Normal) | 1Hz | Separate | Independent |
| Green LED (Recording) | 2Hz | Separate | Independent |
| Node Health Check | 0.2Hz (5s) | ROS2 Timer | Dependent on ROS2 |
| SOS Pattern | Morse timing | On-demand | Independent |

## Benefits of This Architecture

1. **Optimized Performance**: Main ROS loop runs at 1Hz instead of high frequency
2. **Responsive Control**: Button and LED operations remain highly responsive
3. **Fault Tolerance**: Critical functions isolated from ROS2 issues
4. **Real-time Requirements**: LED timing and button detection meet real-time needs
5. **Scalability**: Can handle additional ROS2 operations without affecting core functionality
6. **Debugging**: Easier to isolate issues between ROS2 and hardware control

## Future Enhancements

- Could add more ROS2 timers for different monitoring frequencies
- Could implement priority-based thread scheduling
- Could add thread health monitoring
- Could implement graceful thread shutdown coordination

