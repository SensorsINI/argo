# Requirements Files Migration

## Summary

Merged `requirements.txt` and `requirements-runtime.txt` into a single comprehensive `requirements.txt` file. This simplifies dependency management and makes it easier to maintain the project.

## Changes Made

### 1. Created New Merged `requirements.txt`
- **Location**: `/home/orangepi/argo/requirements.txt`
- **Structure**: Organized by category with clear comments
- **Content**: Includes all dependencies from both previous files plus additional packages discovered in the codebase

### 2. Deleted Old File
- **Removed**: `requirements-runtime.txt` (merged into `requirements.txt`)

### 3. Updated References
The following files were updated to use the new `requirements.txt`:

- `Makefile` - Updated `install-python-deps` target
  - Changed from: `pip3 install -r requirements-runtime.txt`
  - Changed to: `pip3 install -r requirements.txt`
  - Added more comprehensive package listing in success message

- `README.md` - Updated installation instructions
  - Changed comment from: `# installs from requirements-runtime.txt`
  - Changed to: `# installs from requirements.txt`

- `scripts/debug_remote_ros2.sh` - Updated remote debugging script
  - Changed from: `cat $REMOTE_ARGO_DIR/requirements-runtime.txt`
  - Changed to: `cat $REMOTE_ARGO_DIR/requirements.txt`

## Package Categories

### Core Runtime Packages (Required)
- **ROS2**: rclpy, std-msgs, geometry-msgs, diagnostic-msgs, sensor-msgs, std-srvs, visualization-msgs
- **Hardware**: smbus2, pyserial, pynmea2, gpiod, spidev, OPi.GPIO
- **Scientific**: numpy
- **Config**: PyYAML
- **System**: psutil

### Optional Packages
- **Visualization**: matplotlib, pandas
- **Diagnostics**: diagnostic-updater

### Development/Analysis (Commented Out)
- **ROS1**: rospy, bagpy (host-side bag file interpretation)
- **Legacy**: pigpio, smbus, libnmea_navsat_driver
- **Analysis**: pypref, easygui, gmplot

## Installation

```bash
# Install all runtime dependencies
make install-python-deps

# Or directly:
pip3 install -r requirements.txt
```

## Notes

1. **Removed packages**:
   - `scipy` - Not used in the codebase
   - `pathlib` - Built-in module in Python 3.4+, not installable via pip
   - `functools` - Built-in module, not installable via pip

2. **Added packages** (discovered from code analysis):
   - `sensor-msgs` - Used for GPS NavSatFix messages
   - `std-srvs` - Used for ROS2 service types (Trigger, Empty)
   - `visualization-msgs` - Used for Marker and MarkerArray
   - `gpiod` - Used in power_control for modern GPIO control
   - `psutil` - Used in lifecycle manager and controller for process monitoring
   - `pandas` - Used in plotting scripts

3. **Legacy packages** (now commented out):
   - Kept as comments for reference
   - Includes ROS1 packages (rospy, libnmea_navsat_driver)
   - Includes old hardware libraries (pigpio, smbus)
   - Includes development tools (bagpy, pypref, easygui, gmplot)

## Verification

To verify all dependencies are installed correctly:

```bash
# Check ROS2 packages
python3 -c "import rclpy; print('rclpy OK')"
python3 -c "import std_msgs; print('std_msgs OK')"

# Check hardware packages
python3 -c "import smbus2; print('smbus2 OK')"
python3 -c "import serial; print('pyserial OK')"
python3 -c "import pynmea2; print('pynmea2 OK')"
python3 -c "import gpiod; print('gpiod OK')"

# Check scientific/utility packages
python3 -c "import numpy; print('numpy OK')"
python3 -c "import yaml; print('PyYAML OK')"
python3 -c "import psutil; print('psutil OK')"
```

## Migration Complete

All references to `requirements-runtime.txt` have been removed and replaced with `requirements.txt`. The new file is properly integrated with `Makefile`, `setup.py`, and all documentation.

