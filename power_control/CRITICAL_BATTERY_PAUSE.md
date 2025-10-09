# Critical Battery Power Conservation via Pause

## Overview

When critical battery voltage is detected (< 7.2V), the power control system now automatically pauses the Argo launch system **before** halting the computer. This allows all sensor nodes to put their hardware into low-power shutdown states, conserving remaining battery power for potential manual sailing or recovery operations.

## Implementation in argo_power_control.py

### New Method: `_pause_argo_system_for_power_conservation()`

This method calls the `/toggle_pause` ROS2 service to pause all pausable nodes:

```python
def _pause_argo_system_for_power_conservation(self):
    """Pause all sensor nodes to put hardware in low-power shutdown state
    
    This allows sensors (IMU, anemometer, LoRa) to enter hardware shutdown
    mode before system halt to conserve remaining battery power.
    
    Critical nodes (battery_water, temp_monitor) are never paused.
    """
```

### Critical Battery Halt Sequence

The `initiate_critical_battery_halt()` method now follows this sequence:

1. **Show confirmation dialog** (30-second timeout)
2. **Broadcast notifications** (wall message, desktop notification)
3. **User decision window** (cancel or proceed)
4. **PAUSE ARGO SYSTEM** ← NEW STEP
5. **Wait 2 seconds** (for sensor hardware shutdown)
6. **Wait 3 seconds** (for final notifications)
7. **Execute halt command**

## Required Sensor Node Changes

Each sensor node must implement proper hardware shutdown when paused. The pause mechanism is already integrated via `TogglePauseService`, but nodes need to handle hardware power-down.

### Nodes That Need Hardware Shutdown Implementation

#### 1. IMU Node (`nodes/imu.py`)

**What needs to be done:**
- When `is_paused()` returns True, put IMU hardware into shutdown/sleep mode
- Use appropriate I2C commands to disable IMU sensors
- Restore normal operation when unpaused

**Example implementation:**
```python
def timer_callback(self):
    # Check pause state
    if self.pause_service.is_paused():
        # Put IMU in shutdown mode if not already done
        if not self._hardware_shutdown:
            self._shutdown_imu_hardware()
            self._hardware_shutdown = True
        return  # Skip processing when paused
    
    # If we were shutdown, restore hardware
    if self._hardware_shutdown:
        self._restore_imu_hardware()
        self._hardware_shutdown = False
    
    # Normal IMU processing here...

def _shutdown_imu_hardware(self):
    """Put IMU hardware into low-power shutdown mode"""
    try:
        # Send I2C commands to disable IMU sensors
        # Example for ICM-20948 or similar:
        # - Disable gyroscope
        # - Disable accelerometer  
        # - Disable magnetometer
        # - Enter sleep mode
        self.get_logger().info("IMU hardware entering shutdown mode for power conservation")
    except Exception as e:
        self.get_logger().error(f"Error shutting down IMU hardware: {e}")

def _restore_imu_hardware(self):
    """Restore IMU hardware from shutdown mode"""
    try:
        # Send I2C commands to re-enable IMU sensors
        # Restore normal operating mode
        self.get_logger().info("IMU hardware restored from shutdown mode")
    except Exception as e:
        self.get_logger().error(f"Error restoring IMU hardware: {e}")
```

#### 2. Anemometer Node (`nodes/anem.py`)

**What needs to be done:**
- Put anemometer into low-power/shutdown mode when paused
- May involve I2C sleep commands or disabling heating elements
- Restore when unpaused

**Example implementation:**
```python
def timer_callback(self):
    if self.pause_service.is_paused():
        if not self._hardware_shutdown:
            self._shutdown_anemometer_hardware()
            self._hardware_shutdown = True
        return
    
    if self._hardware_shutdown:
        self._restore_anemometer_hardware()
        self._hardware_shutdown = False
    
    # Normal anemometer processing...

def _shutdown_anemometer_hardware(self):
    """Put anemometer into low-power shutdown mode"""
    try:
        # Send commands to disable sensor
        # May include:
        # - Disable heating element (major power draw)
        # - Put sensor in sleep mode
        # - Disable measurement circuits
        self.get_logger().info("Anemometer hardware entering shutdown mode for power conservation")
    except Exception as e:
        self.get_logger().error(f"Error shutting down anemometer hardware: {e}")
```

#### 3. LoRa Radio Node (`nodes/lora.py` if present)

**What needs to be done:**
- Put LoRa radio module into sleep mode when paused
- Disable radio transmission/reception
- Restore when unpaused

**Example implementation:**
```python
def _shutdown_lora_hardware(self):
    """Put LoRa radio into sleep mode"""
    try:
        # Send commands to LoRa module:
        # - Enter sleep mode
        # - Disable radio circuits
        # - Minimize power consumption
        self.get_logger().info("LoRa radio entering sleep mode for power conservation")
    except Exception as e:
        self.get_logger().error(f"Error putting LoRa radio to sleep: {e}")
```

#### 4. GPS Node (`nodes/gps.py`)

**What needs to be done:**
- Put GPS module into standby/backup mode when paused
- Some GPS modules have low-power modes that maintain RTC and ephemeris data
- Restore when unpaused

**Example implementation:**
```python
def _shutdown_gps_hardware(self):
    """Put GPS into backup/standby mode"""
    try:
        # Send NMEA or UBX commands to enter backup mode
        # GPS may maintain hot start capability while drawing minimal power
        self.get_logger().info("GPS entering backup mode for power conservation")
    except Exception as e:
        self.get_logger().error(f"Error putting GPS to backup mode: {e}")
```

### Nodes That Should NOT Be Paused

These nodes are already excluded from pause operations (defined in `launch/argo_lifecycle_manager.py`):

- **`battery_water.py`** - Critical for battery monitoring
- **`temp_monitor.py`** - Critical for system temperature monitoring

## Testing

### Test Critical Battery Pause Sequence

```bash
# Simulate critical battery condition
sudo python3 power_control/argo_power_control.py --simulate-critical-battery

# Or test just the pause functionality
ros2 service call /toggle_pause std_srvs/srv/Trigger
```

### Verify Sensor Shutdown

After implementing hardware shutdown in sensor nodes:

1. Start Argo system: `al`
2. Pause system: `ros2 service call /toggle_pause std_srvs/srv/Trigger`
3. Check sensor logs for "hardware entering shutdown mode" messages
4. Measure power consumption (should drop significantly)
5. Unpause system: `ros2 service call /toggle_pause std_srvs/srv/Trigger`
6. Verify sensors restore normal operation

## Benefits

1. **Extended Battery Life**: Sensors in shutdown mode use minimal power
2. **Manual Sailing Capability**: More battery available for critical systems
3. **Recovery Time**: More time to reach shore or swap batteries
4. **Graceful Degradation**: System can still monitor battery/temperature while sensors are off

## Power Savings Estimates

Typical power consumption when paused (approximate):
- **IMU shutdown**: ~10mA → 1mA (90% reduction)
- **Anemometer (with heater)**: ~100mA → 1mA (99% reduction)
- **GPS**: ~50mA → 5mA (90% reduction in backup mode)
- **LoRa**: ~30mA → 1mA (97% reduction in sleep mode)

**Total estimated savings**: 150-200mA when all sensors are properly shutdown

At 7.2V critical threshold, this could extend usable power by 30-60 minutes.

## Implementation Priority

1. **High Priority**: Anemometer (due to heating element power draw)
2. **High Priority**: LoRa radio (if present)
3. **Medium Priority**: IMU
4. **Medium Priority**: GPS

## Notes

- Hardware shutdown should be **reversible** - sensors must restore when unpaused
- Use sensor datasheets to find proper low-power/shutdown commands
- Test unpausing thoroughly - ensure sensors recover properly
- Log all hardware state changes for debugging
- Consider I2C bus errors during shutdown/restore operations

