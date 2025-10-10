# Compass Calibration Guide

## Overview

The IMU node now supports advanced magnetometer calibration with two methods:
1. **Min-max calibration** - Simple diagonal soft-iron approximation
2. **Ellipsoid fitting** - Advanced 3D ellipsoid fit with full rotation correction

## Features Implemented

### 1. Persistent Calibration Data Storage
- Calibration samples are now saved to `nodes/imu_calib_samples_YYYYMMDD_HHMMSS.json`
- Data persists across reboots (not lost in `/tmp`)
- Enables offline analysis and reprocessing of calibration data

### 2. Ellipsoid Fitting Method
- Pure NumPy implementation (no scipy dependency issues)
- Fits full 3D ellipsoid to magnetometer data
- Computes:
  - Hard iron offset (bias)
  - Soft iron scale factors
  - 3D rotation matrix for axis-aligned correction
  - Semi-axes radii of the ellipsoid

### 3. 3D Visualization
- Shows uncalibrated data (RED points) vs calibrated data (GREEN points)
- Side-by-side comparison in 3D space
- Helps visualize calibration quality
- Saved as `imu_calib_3d_YYYYMMDD_HHMMSS.png`

## Usage

### Running Calibration

```bash
cd /home/orangepi/argo/nodes
python3 imu.py --calib_compass
```

**Interactive Steps:**
1. Rotate the sensor slowly through all orientations (figure-8 pattern)
2. Press `Ctrl+C` when done collecting samples
3. Choose calibration method:
   - Enter `1` for min-max (default)
   - Enter `2` for ellipsoid fitting
4. Review calibration parameters
5. Confirm save (Y/n)

### Viewing Calibration Data

```bash
python3 imu.py --plot_calib
```

This will:
- Load the most recent calibration data (from `/tmp` or `nodes/`)
- Generate time-series plot
- Generate 3D visualization (red = uncalibrated, green = calibrated)
- Print calibration statistics

## Calibration Methods Comparison

### Min-Max Method
- **Pros:**
  - Simple and fast
  - Works well for basic hard iron correction
  - No external dependencies
  
- **Cons:**
  - Assumes ellipsoid is axis-aligned
  - Cannot correct for rotated soft iron effects
  - Less accurate for complex magnetic distortions

### Ellipsoid Fitting Method
- **Pros:**
  - Full 3D correction including rotation
  - More accurate for complex environments
  - Handles non-axis-aligned soft iron effects
  
- **Cons:**
  - Requires more computation
  - Needs good sample coverage of all orientations
  - More sensitive to poor sample distribution

## Calibration Data Format

### Min-Max Calibration
```json
{
  "method": "minmax",
  "bias_uT": [bx, by, bz],
  "scale_diag": [sx, sy, sz],
  "num_samples": 1234,
  "ranges_uT": {
    "x": [min_x, max_x],
    "y": [min_y, max_y],
    "z": [min_z, max_z]
  },
  "timestamp": "2025-10-09T12:34:56.789Z"
}
```

### Ellipsoid Calibration
```json
{
  "method": "ellipsoid",
  "bias_uT": [cx, cy, cz],
  "scale_diag": [sx, sy, sz],
  "rotation": [
    [r11, r12, r13],
    [r21, r22, r23],
    [r31, r32, r33]
  ],
  "radii": [rx, ry, rz],
  "num_samples": 1234,
  "ranges_uT": { ... },
  "timestamp": "2025-10-09T12:34:56.789Z"
}
```

## Files Generated

During calibration, the following files are created:

### In `/tmp/` (temporary)
- `imu_calib_plot_YYYYMMDD_HHMMSS.png` - Time-series plot
- `imu_calib_3d_YYYYMMDD_HHMMSS.png` - 3D visualization

### In `nodes/` (persistent)
- `invensense-20948-compass-calibration.json` - Active calibration parameters
- `imu_calib_samples_YYYYMMDD_HHMMSS.json` - Raw calibration data
- `imu_calib_backups/invensense-20948-compass-calibration_YYYYMMDD_HHMMSS.json` - Backup of previous calibration

## Tips for Good Calibration

1. **Sample Coverage:**
   - Rotate sensor through ALL orientations
   - Use smooth figure-8 patterns
   - Include upside-down orientations
   - Aim for 500+ samples

2. **Environment:**
   - Calibrate away from large metal objects
   - Avoid motors, batteries, and magnets
   - Outdoor calibration is ideal
   - Keep environment consistent with usage

3. **Quality Check:**
   - Look at 3D visualization
   - Uncalibrated data should form ellipsoid
   - Calibrated data should form sphere
   - Check for outliers or gaps

## NumPy/SciPy Compatibility

The ellipsoid fitting implementation uses **pure NumPy** to avoid scipy version conflicts. The numpy version warning when importing scipy can be safely ignored for this application, as we don't use scipy at all.

If you want to experiment with scipy-based fitting methods, you would need to:
```bash
# Check current numpy version
python3 -c "import numpy; print(numpy.__version__)"

# For scipy compatibility, may need numpy 1.17.3 to 1.24.x
# But current implementation doesn't require scipy
```

## Offline Analysis

Since calibration data is now saved persistently in `nodes/`, you can:

1. **Reprocess with different methods:**
   - Load JSON data
   - Apply different fitting algorithms
   - Compare results

2. **Batch processing:**
   - Analyze multiple calibration sessions
   - Track calibration drift over time
   - Statistical quality assessment

3. **Custom visualization:**
   - Import data into Python/MATLAB
   - Create custom plots
   - Analyze residual errors

## Example: Loading Calibration Data for Analysis

```python
import json
import numpy as np

# Load calibration data
with open('nodes/imu_calib_samples_20251009_123456.json', 'r') as f:
    data = json.load(f)

# Extract samples (time, mx, my, mz)
samples = np.array(data['samples'])
times = samples[:, 0]
mx = samples[:, 1]
my = samples[:, 2]
mz = samples[:, 3]

# Get calibration parameters
calib = data['calibration']
bias = np.array(calib['bias_uT'])
scale = np.array(calib['scale_diag'])

# Apply calibration
calibrated = (np.column_stack([mx, my, mz]) - bias) * scale

# Your analysis here...
```

## Troubleshooting

### "Only X samples collected"
- Need at least 100 samples for basic calibration
- Aim for 500+ samples for best results
- Collect for 30-60 seconds while rotating

### "Ellipsoid fit failed"
- Need minimum 9 samples (but really want 100+)
- Check for degenerate sample distribution
- Falls back to min-max method automatically

### "No calibration data found"
- Calibration was not saved (answered 'n' to save prompt)
- Check `nodes/` directory for `imu_calib_samples_*.json` files
- Re-run calibration with `--calib_compass`

### Poor Calibration Quality
- 3D plot shows calibrated data not spherical
- Need better sample coverage
- May have nearby magnetic interference
- Try recalibrating in different location


