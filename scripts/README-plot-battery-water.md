# Battery Water Data Plotting

The `plot-battery-water.py` script provides comprehensive visualization of battery and sensor data collected by the `argo_battery_water.py` ROS2 node.

## Features

### Data Visualization
- **Battery Voltage Decay**: Shows battery voltage and state-of-charge over time
- **Sensor Trends**: Comprehensive view of all sensors (voltage, current, temperature, humidity)
- **Alert Patterns**: Timeline of battery, saltwater, and humidity alerts
- **Data Summary**: Statistical summary of all sensor readings

### Plot Types
1. **Battery Voltage Decay Plot**: 
   - Battery voltage over time
   - Battery state-of-charge percentage
   - Perfect for analyzing battery discharge patterns

2. **Sensor Trends Plot**:
   - All 6 sensor readings in a 3x2 grid
   - Battery voltage, percentage, saltwater, current, temperature, humidity
   - Comprehensive overview of system health

3. **Alert Patterns Plot**:
   - Alert status over time (battery low, saltwater, humidity)
   - System health status
   - Critical for identifying problem periods

## Usage

### Basic Usage
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

### Command Line Options
- `csv_file`: Path to CSV file (optional, auto-detects latest if not provided)
- `--output-dir`: Directory for output plots (default: `/var/log.hdd/persistent`)
- `--no-plots`: Only print data summary, don't generate plots

## Data Requirements

The script expects CSV files with the following columns:
- `timestamp`: ISO format timestamp
- `battery_voltage`: Battery voltage in volts
- `battery_remaining_pct`: Battery state-of-charge percentage
- `saltwater_voltage`: Saltwater probe voltage in volts
- `sail_current`: Sail winch current in amperes
- `pcb_temperature`: PCB temperature in Celsius
- `relative_humidity`: Relative humidity percentage
- `battery_low_alert`: Battery low alert (0/1)
- `saltwater_alert`: Saltwater intrusion alert (0/1)
- `humidity_alert`: High humidity alert (0/1)
- `argo_battery_water_health`: Overall health status (0/1)

## Output Files

The script generates timestamped PNG files:
- `battery_voltage_decay_YYYYMMDD_HHMMSS.png`
- `sensor_trends_YYYYMMDD_HHMMSS.png`
- `alert_patterns_YYYYMMDD_HHMMSS.png`

## Dependencies

- `pandas`: For CSV data handling
- `matplotlib`: For plot generation
- `numpy`: For numerical operations (usually comes with matplotlib)

Install dependencies:
```bash
pip3 install pandas matplotlib
```

## Examples

### Analyze Battery Decay Over a Day
```bash
# Generate plots for today's data
python3 plot-battery-water.py

# View the battery voltage decay plot
# Look for gradual voltage decline indicating battery discharge
```

### Check for Alert Patterns
```bash
# Generate all plots and check alert patterns
python3 plot-battery-water.py

# Look for:
# - Battery low alerts during high current draw
# - Saltwater alerts indicating water intrusion
# - Humidity alerts during weather changes
```

### Monitor System Health
```bash
# Quick data summary without plots
python3 plot-battery-water.py --no-plots

# Check for:
# - Current battery voltage and percentage
# - Any active alerts
# - System health status
```

## Integration with Battery Water Node

The plotting script works seamlessly with the `argo_battery_water.py` node:
- CSV files are automatically created every 30 seconds
- Files are stored in `/var/log.hdd/persistent/`
- Script auto-detects the latest file
- No manual data export needed

## Troubleshooting

### No CSV Files Found
```bash
# Check if argo_battery_water.py is running
ros2 node list | grep argo_battery_water_node

# Check if CSV files exist
ls -la /var/log.hdd/persistent/battery-monitor-*.csv
```

### Missing Dependencies
```bash
# Install required packages
pip3 install pandas matplotlib

# Or install system packages
sudo apt install python3-pandas python3-matplotlib
```

### Permission Issues
```bash
# Ensure script is executable
chmod +x plot-battery-water.py

# Check output directory permissions
ls -la /var/log.hdd/persistent/
```
