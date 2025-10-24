# LoRa Testing Scripts

This directory contains testing and analysis scripts for LoRa communication validation.

**Location**: `~/argo/tests/lora/`

## Scripts

### `analyze_latest_lora.py`
**Purpose**: Comprehensive analysis of LoRa communication from ROS bag recordings
**Usage**: `python3 analyze_latest_lora.py [bag_path]`
**Features**:
- Analyzes LoRa RX data, signal strength, and GPS data
- Calculates communication rates and statistics
- Identifies JSON vs binary data packets
- Checks for lora/ prefixed topics (indicates successful shore-side parsing)
- Shows GPS fix samples

### `check_signal_strength.py`
**Purpose**: Detailed analysis of LoRa signal strength data
**Usage**: `python3 check_signal_strength.py [bag_path]`
**Features**:
- Extracts signal strength statistics (min, max, average)
- Categorizes signals by quality (good ≥-80 dBm, fair -90 to -80 dBm, poor <-90 dBm)
- Shows signal strength samples and distribution

### `plot_gps_lora_simple.py`
**Purpose**: Creates GPS track visualization with LoRa ping reception points
**Usage**: `python3 plot_gps_lora_simple.py [bag_path]`
**Features**:
- Plots GPS track as blue line
- Overlays LoRa ping points color-coded by signal strength
- Shows signal strength over time
- Calculates approximate distance traveled
- Saves plot as `/tmp/gps_lora_track.png`
- **Verbose progress tracking** - shows processing status every 5000 messages

## Dependencies

All scripts require:
- `rosbag2_py` (ROS2 bag reading)
- `matplotlib` (for plotting script)
- `numpy` (for plotting script)

Install with:
```bash
pip install matplotlib numpy
```

## Example Usage

```bash
# Navigate to the lora test directory
cd ~/argo/tests/lora

# Analyze the latest recording
python3 analyze_latest_lora.py ~/argo/bags/argo_20251024_122903

# Check signal strength statistics
python3 check_signal_strength.py ~/argo/bags/argo_20251024_122903

# Create GPS + LoRa visualization
python3 plot_gps_lora_simple.py ~/argo/bags/argo_20251024_122903
```

## Test Results Summary

### Latest Test (argo_20251024_122903)
- **Duration**: 338 seconds (5.6 minutes)
- **LoRa RX Rate**: 0.96 msg/sec (325 messages)
- **Signal Strength Rate**: 0.51 msg/sec (171 measurements)
- **GPS Fix Rate**: 0.39 msg/sec (133 valid fixes)
- **Data Quality**: All samples were valid JSON ping packets
- **Signal Quality**: 92.4% poor signals (<-90 dBm), but communication still worked
- **GPS Location**: Switzerland (Lat=47.399052, Lon=8.546594, Alt=~152m)

### Key Findings
- ✅ Bidirectional LoRa communication working
- ✅ Valid JSON packets being received
- ✅ GPS fix working and providing position data
- ⚠️ Most signals are poor quality but communication still functional
- ❌ No lora/ prefixed topics found (shore-side JSON parsing may not be publishing to ROS)

## Notes

- Scripts are designed to work with ROS2 bag files from Argo recordings
- Default bag path is `~/argo/bags/argo_20251024_122903` if not specified
- Plot output is saved to `/tmp/gps_lora_track.png`
- All scripts handle missing data gracefully
