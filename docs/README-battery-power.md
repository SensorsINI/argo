# Battery Power Management and Monitoring

Comprehensive documentation for battery monitoring, charging status, LED indicators, and power management on the Argo autonomous sailboat system.

## Overview

The Argo system implements a sophisticated multi-layer battery monitoring and power management system:

1. **Battery Monitoring Node** (`argo_battery_water.py`) - Continuous sensor readings and battery state estimation
2. **Power Control Service** (`argo_power_control.py`) - Critical battery protection and LED status indicators
3. **Health Monitor** (`argo_health_monitor.py`) - System-wide health tracking including battery node status
4. **Data Visualization** (`plot-battery-water.py`) - Historical data analysis and plotting

## Battery Monitoring Node (`argo_battery_water.py`)

### Core Functionality

The battery monitoring node provides continuous monitoring of:
- **Battery voltage** (via MAX11612 ADC with 27k/18k voltage divider)
- **Battery state-of-charge** (using per-cell LiPo formula)
- **Saltwater intrusion detection** (voltage probe)
- **Sail winch current** (shunt measurement)
- **PCB temperature and humidity** (SHT45 sensor)
- **Charging status** (MP2672GD charger via GPIO and optional I2C)
- **AC power presence** (MP2672GD charger status)

### Battery State-of-Charge Calculation

The node calculates battery percentage using a per-cell LiPo discharge curve formula:

```python
soc% = S - S / (1 + (v / V0)^A)^B
```

**Default Parameters** (for 2S LiPo):
- `S = 123.0` - Scaling factor
- `V0 = 3.7V` - Reference voltage per cell
- `A = 80.0` - Curve shape parameter
- `B = 0.165` - Curve steepness parameter
- `battery_series_cells = 2` - Number of cells in series

**Voltage Range**:
- **Fully charged**: ~8.4V (4.2V per cell)
- **Nominal**: ~7.4V (3.7V per cell)
- **Low battery**: 7.5V threshold (warning)
- **Critical**: 6.8V threshold (system halt)
- **Fully discharged**: ~6.0V (3.0V per cell)

### Battery Lifetime Estimation

The node estimates time to full charge or depletion using linear regression on recent voltage samples:

**Key Features**:
- **Persistent slopes**: Charging/discharging slopes saved to `battery_slopes.json`
- **Early estimates**: Uses saved slopes immediately after startup (no waiting for 5 samples)
- **Dynamic updates**: Updates slopes when sufficient data is available
- **Slope validation**: Only saves meaningful slopes (>0.3 V/h for charging, <-0.3 V/h for discharging)

**Estimation Logic**:
```python
# Priority 1: Use persistent slopes (immediate estimates)
if charging:
    slope = self._charging_slope_v_per_s
else:
    slope = self._discharging_slope_v_per_s

# Priority 2: Update from linear regression on recent samples
fit_result = self._linear_least_squares(self._voltage_samples)
if fit_result and sufficient_samples:
    regression_slope = fit_result[0]
    # Update persistent slope if valid
    if charging and regression_slope > MIN_CHARGING_SLOPE:
        self._charging_slope_v_per_s = regression_slope
```

**Configuration**:
- `battery_lifetime_sample_window = 60` - Number of samples for regression
- `battery_lifetime_min_samples = 5` - Minimum samples required for estimation
- `BATTERY_FULLY_CHARGED_THRESHOLD_V = 8.2V` - Target voltage for "full"

### Charging Status Detection

**MP2672GD Charger Integration**:

The node supports two operating modes:

1. **Standalone Mode** (`MP2672_HOST_CONTROL = False`):
   - Uses GPIO only for charging status
   - PC12 (line 76): !CHARGING signal
   - PH9 (line 233): !ACOK signal
   - Time-window filtering to handle MP2672GD cycling behavior

2. **Host Control Mode** (`MP2672_HOST_CONTROL = True`):
   - Primary: MP2672 I2C register 0x03 (Status register)
   - Fallback: GPIO status with time-window filtering
   - Automatic configuration on USB connection:
     - Disables CHG timer (prevents 20-hour timeout)
     - Disables watchdog timer (prevents register resets)
     - Disables suspend mode (keeps boost enabled)

**Charging Status Priority**:
```python
# 1. Try MP2672 I2C register (if host control enabled and USB present)
if MP2672_HOST_CONTROL and self.mp2672_available:
    status = self._read_mp2672_status()  # Returns (charging, ac_power)
    
# 2. Fallback to GPIO with time-window filtering
else:
    status = self._read_gpio_status()  # Time-filtered GPIO reading
```

**Time-Window Filtering**:
- **Charging window**: 30 seconds - reports "charging" if seen within last 30s
- **AC power window**: 5 seconds - reports "AC present" if seen within last 5s
- Handles MP2672GD cycling behavior when battery is fully charged

### Charging Anomaly Detection

The node detects when charger reports "charging" but voltage is actually decreasing:

**Detection Criteria**:
- Charging status = True (from GPIO or I2C)
- Voltage slope < -0.05 V/h (falling faster than threshold)
- Sufficient samples for slope calculation (≥5 samples)

**Anomaly Codes**:
- `charger_power_fault` - Charger not delivering power despite reporting charging

**Possible Causes**:
- Insufficient USB power supply
- Poor USB connection
- System load exceeds charger capability
- Charger hardware fault (check MP2672 fault register when USB present)

### I2C Failure Detection

**Critical I2C Failure**:
- Detected when ADC sensor fails for >30 seconds
- Publishes to `/argo/critical/i2c_failure` topic (Bool)
- Triggers SOS LED pattern in power control service
- System can switch to RTH mode automatically

**Recovery**:
- Requires 10 consecutive successful reads (10 seconds at 1Hz)
- Sustained recovery tracking prevents false recoveries
- Publishes recovery status when cleared

### Published Topics

**Sensor Data** (Float32):
- `/battery_voltage` - Battery voltage in volts
- `/battery_remaining_pct` - State-of-charge percentage
- `/saltwater_voltage` - Saltwater probe voltage
- `/sail_current` - Sail winch current (5Hz for control)
- `/temperature_pcb` - PCB temperature (°C)
- `/relative_humidity` - Relative humidity (%)
- `/battery_lifetime_hours` - Estimated hours to full/empty

**Status Topics** (Bool):
- `/charging_status` - True if charging, False if not
- `/ac_power_present` - True if AC/USB power present
- `/battery_low_alert` - Battery voltage below threshold
- `/saltwater_alert` - Saltwater intrusion detected
- `/humidity_alert` - High humidity detected
- `/battery_water_node_health` - Node health status

**Critical Topics**:
- `/argo/critical/i2c_failure` - Critical I2C failure (triggers SOS pattern)

### Services

**`/battery_status`** (Trigger service):
Returns comprehensive battery status as JSON:

```json
{
  "battery_summary": "7.8V (45%)",
  "critical_alerts": "🔋 LOW BATTERY | ⚠️ CHARGER POWER FAULT",
  "raw_data": {
    "battery_voltage": 7.8,
    "battery_remaining_pct": 45.2,
    "charging_status": true,
    "ac_power_present": true,
    "time_to_full_hours": 2.5,
    "voltage_slope_vph": 0.15,
    "charging_anomaly": false,
    "mp2672_available": true,
    "mp2672_status": {...},
    "mp2672_faults": {...},
    "i2c_failure": false,
    "stale_data": false
  }
}
```

### CSV Logging

**Log File**: `/var/log.hdd/persistent/battery-monitor-YYYYMMDD.csv`

**Logging Interval**: Every 30 seconds

**Columns**:
- `timestamp` - ISO format timestamp
- `battery_voltage` - Battery voltage (V)
- `battery_remaining_pct` - State-of-charge (%)
- `saltwater_voltage` - Saltwater probe (V)
- `sail_current` - Sail winch current (A)
- `pcb_temperature` - PCB temperature (°C)
- `relative_humidity` - Humidity (%)
- `battery_low_alert` - Low battery alert (0/1)
- `saltwater_alert` - Saltwater alert (0/1)
- `humidity_alert` - Humidity alert (0/1)
- `battery_water_health` - Node health (0/1)
- `charging_status` - Charging status (0/1)
- `ac_power_present` - AC power present (0/1)

**Stale Data Marking**: When I2C failure occurs, CSV entries are marked with "FAILED" for voltage readings.

## Power Control Service (`argo_power_control.py`)

### Critical Battery Protection

The power control service implements two-tier battery protection:

#### 1. Low Battery Warning (< 7.6V)

**Threshold**: `LOW_BATTERY_THRESHOLD_V = 7.6V`

**Behavior**:
- **SOS LED pattern** starts flashing (red LED)
- **Desktop notification** warns user
- **System continues operating** normally
- Pattern only shows when **AC power is NOT present**

**LED Pattern**: Standard SOS pattern (···---···)

#### 2. Critical Battery Halt (< 7.2V)

**Threshold**: `CRITICAL_BATTERY_THRESHOLD_V = 7.2V`

**Production Mode** (`CRITICAL_BATTERY_USE_SHUTDOWN = False`):
1. Shows **30-second confirmation dialog** (auto-halt on timeout)
2. **Pauses all sensor nodes** (hardware shutdown for power conservation)
3. Creates `/tmp/argo_critical_battery` flag file
4. Executes `sudo halt` command
5. **Shutdown hook preserves power relay** (radio control remains active)
6. Computer halts, servos powered for manual sailing (~50mA vs ~500mA)

**Development Mode** (`CRITICAL_BATTERY_USE_SHUTDOWN = True`):
1. Shows confirmation dialog
2. Pauses sensor nodes
3. Executes `shutdown -h now` (normal shutdown)
4. **Power relay cuts** (complete shutdown)

### Charging State LED Pattern

**LED**: Green LED (configurable via `CHARGE_STATE_LED`)

**Pattern Behavior**:
- **Active only when AC power is present**
- **Duty cycle represents charge percentage**:
  - 0% = LED always off
  - 50% = LED on 50% of period
  - 100% = LED steady ON (fully charged)

**Configuration**:
- `CHARGE_STATE_PERIOD_S = 3.0` - Total period (seconds)
- `CHARGE_STATE_DUTY_CYCLE = 0.2` - Base duty cycle (20% on)
- Pattern dynamically adjusts based on battery percentage

**Example**:
- Battery at 60% charge → LED on for 60% of 3s period = 1.8s on, 1.2s off
- Battery fully charged → LED steady ON

### I2C Failure SOS Pattern

**Trigger**: `/argo/critical/i2c_failure` topic (True)

**Pattern**: Modified SOS pattern indicating I2C failure

**Behavior**:
- Takes priority over charge state pattern
- Pauses normal heartbeat
- Desktop notification sent
- Clears when I2C recovery detected

### Battery Monitoring Thread

**Interval**: Every 30 seconds (`BATTERY_MONITORING_INTERVAL_S`)

**Functionality**:
- Calls `/battery_status` service
- Extracts `i2c_failure` and `stale_data` flags to distinguish I2C failures from actual low voltage
- Validates voltage readings (only 0.0V indicates invalid reading)
- Logs voltage changes >50mV (`BATTERY_LOG_THRESHOLD_V`)
- Handles service failures gracefully
- Initiates safe shutdown after 5 minutes of service failures

**I2C Failure Detection**:
- **Primary check**: If `i2c_failure=True` or `stale_data=True` from battery service, treat as I2C failure
- **I2C failures**: Tracked for 1-hour timeout before shutdown (prevents false shutdowns from temporary sensor issues)
- **Never triggers immediate shutdown** on I2C failures - only after sustained 1-hour failure
- **Skips critical battery check** when I2C is failing (prevents treating 0.0V as critical low voltage)

**Invalid Reading Detection**:
- **Only 0.0V** is treated as invalid reading (indicates I2C failure, not actual battery voltage)
- **Sustained invalid readings** (>1 hour) trigger safety shutdown
- **Duration validation**: Prevents false shutdowns from timestamp corruption or system clock adjustments
- **Reading count validation**: Ensures duration matches expected number of readings before shutdown

**Safety Features**:
- **I2C failure protection**: Checks `i2c_failure` and `stale_data` flags before processing voltage
- **Never halts on invalid readings immediately** - only after 1 hour of sustained failure
- **Startup grace period** (60s) for battery service availability
- **Consecutive failure tracking** (3 failures = warning, 10 failures = shutdown)

### LED Status Summary

| Condition | LED Pattern | LED Color | Priority |
|-----------|-------------|-----------|----------|
| **Normal operation** | Heartbeat (slow blink) | Green | Low |
| **Low battery** (< 7.6V, no AC) | SOS pattern | Red | Medium |
| **Charging** (AC present) | Duty cycle flash | Green | Medium |
| **Fully charged** (AC present) | Steady ON | Green | Medium |
| **I2C failure** | Modified SOS | Red | High |
| **Critical battery** (< 7.2V) | Shutdown sequence | N/A | Highest |

## Health Monitor Integration (`argo_health_monitor.py`)

### Battery Node Health Tracking

The health monitor subscribes to `/battery_water_node_health` topic to track battery node status.

**Health Status**:
- **Healthy**: Battery node operational, sensors reading correctly
- **Unhealthy**: Battery node has issues (I2C failures, sensor errors, low battery, saltwater, high humidity)

**Integration**:
- Health monitor polls battery node health via topic subscription
- Battery node publishes health status using ArgoBaseNode health system
- Health monitor aggregates all node health for system status

**Service**: `/argo/health/status` returns health status including battery node:

```json
{
  "nodes": {
    "argo_battery_water.py": {
      "healthy": true,
      "required": true,
      "critical": true,
      "description": "Battery and water monitoring",
      "last_seen": 1234567890.0
    }
  }
}
```

## Data Visualization (`plot-battery-water.py`)

### Plot Types

1. **Battery Voltage Decay Plot**:
   - Battery voltage over time
   - Battery state-of-charge percentage
   - Perfect for analyzing discharge patterns

2. **Sensor Trends Plot**:
   - 3×2 grid of all sensor readings
   - Battery voltage, percentage, saltwater, current, temperature, humidity

3. **Alert Patterns Plot**:
   - Timeline of battery, saltwater, and humidity alerts
   - System health status over time

### Usage

```bash
# Auto-detect latest CSV file and generate all plots
python3 plot-battery-water.py

# Specify a particular CSV file
python3 plot-battery-water.py /var/log.hdd/persistent/battery-monitor-20251005.csv

# Generate plots in a specific directory
python3 plot-battery-water.py --output-dir /tmp/plots

# Only show data summary (no plots)
python3 plot-battery-water.py --no-plots
```

### Dependencies

```bash
pip3 install pandas matplotlib
```

## Key Code References

### Battery Slope Estimation

```1151:1251:nodes/argo_battery_water.py
    def _estimate_battery_lifetime(self, voltage: float, charging: bool) -> Optional[float]:
        """
        Estimate time to full charge or depletion in hours.
        
        Args:
            voltage: Current battery voltage
            charging: True if charging, False if discharging
        
        Returns:
            Time in hours, or None if estimation not possible
        """
        try:
            # Priority 1: Use persistent slopes immediately if available (no need to wait for 5 samples)
            # This allows estimates to be provided right after startup if we have saved slopes
            slope_v_per_s = None
            if charging:
                slope_v_per_s = self._charging_slope_v_per_s
            else:
                slope_v_per_s = self._discharging_slope_v_per_s
            
            # Priority 2: Try linear regression on recent samples to update persistent slopes
            # This provides more accurate estimates when we have sufficient recent data
            fit_result = self._linear_least_squares(self._voltage_samples)
            
            if fit_result is not None:
                regression_slope_v_per_s, intercept = fit_result
                
                # Minimum slope thresholds to avoid saving near-zero slopes when battery is stable
                # 0.3 V/h = 8.33e-5 V/s minimum for meaningful slope updates
                MIN_CHARGING_SLOPE_V_PER_S = 8.33e-5  # ~0.3 V/h
                MIN_DISCHARGING_SLOPE_V_PER_S = -8.33e-5  # -0.3 V/h (magnitude)
                
                # Also check that battery is not already fully charged to avoid capturing voltage float near full
                is_fully_charged = voltage >= (BATTERY_FULLY_CHARGED_THRESHOLD_V - 0.1)  # Within 0.1V of full
                is_near_empty = voltage <= 6.5  # Within 0.5V of empty
                
                # Update persistent slopes if we have good data, sufficient samples, and proper charging state
                if (charging and 
                    regression_slope_v_per_s > MIN_CHARGING_SLOPE_V_PER_S and  # Significant positive slope
                    len(self._voltage_samples) >= self.battery_lifetime_min_samples and 
                    self._is_proper_charging_state(self._latest_charging_status, self._voltage_samples) and
                    not is_fully_charged):  # Don't update slope when already fully charged
                    self._charging_slope_v_per_s = regression_slope_v_per_s
                    self.get_logger().debug(f"Updated charging slope: {regression_slope_v_per_s:.6f} V/s from {len(self._voltage_samples)} samples (voltage: {voltage:.2f}V)")
                    # Use the updated slope for this estimation
                    slope_v_per_s = regression_slope_v_per_s
                elif (not charging and 
                      regression_slope_v_per_s < MIN_DISCHARGING_SLOPE_V_PER_S and  # Significant negative slope
                      len(self._voltage_samples) >= self.battery_lifetime_min_samples and 
                      self._is_proper_charging_state(self._latest_charging_status, self._voltage_samples) and
                      not is_near_empty):  # Don't update slope when already near empty
                    self._discharging_slope_v_per_s = regression_slope_v_per_s
                    self.get_logger().debug(f"Updated discharging slope: {regression_slope_v_per_s:.6f} V/s from {len(self._voltage_samples)} samples (voltage: {voltage:.2f}V)")
                    # Use the updated slope for this estimation
                    slope_v_per_s = regression_slope_v_per_s
```

### Charging Status Detection

```1883:1945:nodes/argo_battery_water.py
    def _read_charging_status(self):
        """
        Read charging status from MP2672 I2C register (primary) or GPIO (fallback).
        
        Priority:
        1. MP2672 I2C register 0x03 (if available and MP2672_HOST_CONTROL = True)
        2. GPIO status with time-window filtering (fallback or standalone mode)
        
        Returns:
            Tuple of (charging_status, ac_power_present) or (None, None) if unavailable
        """
        # Try MP2672 I2C first (primary source) - only if host control enabled
        if MP2672_HOST_CONTROL and self.mp2672_available:
            status = self._read_mp2672_status()
            if status is not None:
                charging_status, ac_power_present = status
                # Update GPIO timestamps to match I2C reading for consistency
                current_time = time.monotonic()
                if charging_status:
                    self._last_charging_true_time = current_time
                if ac_power_present:
                    self._last_ac_power_true_time = current_time
                return charging_status, ac_power_present
        
        # Fallback to GPIO if I2C not available
        return self._read_gpio_status()
    
    def _read_gpio_status(self):
        """
        Get current GPIO status based on time-window filtering.
        
        Note: When battery is fully charged, the MP2672GD cycles between charging
        and supplementing modes. This is normal behavior. We report "charging" or
        "AC power present" if seen within the last time window to provide stable
        status reporting despite the rapid cycling.
        
        The actual GPIO polling happens at 1Hz in read_gpio_status_only().
        This method just returns the time-filtered status.
        """
        if not self.gpio_available:
            return None, None
            
        current_time = time.monotonic()
        
        # Determine status based on time windows
        # Report True if seen as True within the time window
        charging_status_recent = (current_time - self._last_charging_true_time) < self._charging_window_s
        ac_power_present = (current_time - self._last_ac_power_true_time) < self._ac_power_window_s

        # Fail-safe: charging cannot be true if AC power is absent
        if charging_status_recent and not ac_power_present:
            if (current_time - self._last_charging_conflict_log_time) >= self._charging_conflict_log_interval:
                self.get_logger().warning(
                    "Charging GPIO asserted but AC power is absent; suppressing charging status (check charger hardware)."
                )
                self._last_charging_conflict_log_time = current_time
            # Age out the stale charging timestamp so it clears immediately
            self._last_charging_true_time = current_time - self._charging_window_s
            charging_status_recent = False

        charging_status = charging_status_recent
        
        return charging_status, ac_power_present
```

### Critical Battery Monitoring

**Key Logic Flow**:

1. **I2C Failure Check** (first priority):
   - Extract `i2c_failure` and `stale_data` flags from battery service response
   - If either flag is `True` → Treat as I2C failure (not critical battery)
   - Track for 1-hour timeout before shutdown
   - Skip critical battery check entirely (prevents false shutdowns)

2. **Invalid Reading Check** (if I2C flags are False):
   - Only 0.0V is treated as invalid reading
   - Track for 1-hour timeout with duration and reading count validation
   - Prevents false shutdowns from timestamp corruption or system clock adjustments

3. **Critical Battery Check** (only for valid readings):
   - If voltage < 7.2V and I2C is working → Immediate shutdown (actual low battery)
   - If AC power present and charging → Don't shutdown (battery will recover)

**Code Reference**:

```3407:3540:power_control/argo_power_control.py
                    battery_voltage = battery_data.get('battery_voltage', 0)
                    charging_status = battery_data.get('charging_status', None)
                    ac_power_present = battery_data.get('ac_power_present', None)
                    # CRITICAL: Extract I2C failure and stale data flags to distinguish I2C failures from actual low voltage
                    i2c_failure = battery_data.get('i2c_failure', False)
                    stale_data = battery_data.get('stale_data', False)
                    
                    # CRITICAL SAFETY CHECK: If I2C is failing or data is stale, treat as I2C failure (not critical battery)
                    # I2C failures should only trigger shutdown after 1 hour of sustained failure, not immediately
                    if i2c_failure or stale_data:
                        # I2C is failing - treat any voltage reading (including 0.0V) as invalid due to I2C failure
                        # Do NOT treat as critical low voltage - only track for 1-hour invalid reading timeout
                        # ... (tracks for 1-hour timeout, skips critical battery check)
                        continue  # Skip critical battery check - this is I2C failure, not low voltage

                    # CRITICAL SAFETY CHECK: Only 0.0V indicates invalid reading (I2C failure)
                    # 0.0V reading indicates sensor/communication error, not actual battery voltage
                    if battery_voltage <= 0.0:
                        # ... (tracks for 1-hour timeout with validation)
                        continue

                    # Valid reading - check for critical battery
                    if battery_voltage < CRITICAL_BATTERY_THRESHOLD_V:
                        # Immediate shutdown for actual low battery (only if I2C is working)
                        # ...
```

## Troubleshooting

### Battery Service Not Available

```bash
# Check if battery service is running
sudo systemctl status argo_battery_water.service

# Check battery node logs
journalctl -u argo_battery_water.service -f

# Verify ROS2 node is running
ros2 node list | grep battery_water
```

### Invalid Battery Readings and I2C Failures

**Symptoms**: Voltage readings are 0.0V (invalid reading) or I2C failure flags are set

**Detection Logic**:
1. **I2C Failure Check** (first priority):
   - If `i2c_failure=True` or `stale_data=True` from battery service → Treat as I2C failure
   - Track for 1-hour timeout before shutdown
   - Skip critical battery check (prevents false shutdowns)

2. **Invalid Reading Check** (if I2C flags are False):
   - Only 0.0V is treated as invalid reading
   - Track for 1-hour timeout before shutdown
   - Duration and reading count validation prevents false shutdowns

**Possible Causes**:
- I2C bus failure (sensor short, bus conflict)
- ADC sensor disconnected
- Voltage divider circuit issue
- Service crash
- System clock adjustment (causes duration validation to reset timer)

**Solution**: 
- System will NOT halt immediately on invalid readings or I2C failures
- Only triggers shutdown after **1 hour of sustained failure**
- Check hardware connections and I2C bus status
- Review logs for "time since last valid reading" to diagnose actual failure duration

### Charging Status Not Updating

**Symptoms**: Charging status stuck or incorrect

**Possible Causes**:
- GPIO conflicts
- MP2672GD charger cycling (normal when fully charged)
- I2C communication issues (if host control enabled)

**Solution**: Check GPIO status and MP2672GD charger hardware connections.

### Battery Slope Estimation Not Working

**Symptoms**: Time-to-full/empty estimates are None

**Possible Causes**:
- Insufficient voltage samples (<5 samples)
- Battery voltage stable (no significant slope)
- Charging status unknown

**Solution**: Wait for more samples or check charging status detection.

## Related Documentation

- [Power Control System](../power_control/README.md)
- [Battery Monitoring Installation](../scripts/README-plot-battery-water.md)
- [Persistent Logging](README-persistent-logging.md)
