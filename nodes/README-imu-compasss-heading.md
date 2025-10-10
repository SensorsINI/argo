### IMU Compass Heading with Madgwick Filter (ROS 2)

This doc explains how to run a fused compass heading using the prebuilt ROS 2 Madgwick filter package, how the nodes fit together with the current `imu.py`, and practical tips to get robust results.

The goal is zero C++/CMake work: only install the prebuilt package and use Python nodes/scripts.

---

### Current State
- `nodes/imu.py` publishes raw sensor topics:
  - `/accel` (geometry_msgs/Vector3) in g
  - `/gyro` (geometry_msgs/Vector3) in deg/s
  - `/magnetometer` (geometry_msgs/Vector3) in µT
  - `/compass` (std_msgs/Float64) = built-in tilt‑compensated heading

Madgwick filter expects standard messages:
- `sensor_msgs/Imu` (linear_acceleration in m/s², angular_velocity in rad/s)
- `sensor_msgs/MagneticField` (Tesla)

Because of this, a thin conversion step is needed between our topics and the filter.

---

### Install the Madgwick Filter (prebuilt)
```bash
sudo apt update && sudo apt install -y ros-humble-imu-filter-madgwick
```

---

### Node Graph Options

- Option A (recommended with no code changes to `imu.py`):
  1) `imu.py` → publishes `/accel`, `/gyro`, `/magnetometer`
  2) Converter node (Python) → publishes `imu/data_raw` (sensor_msgs/Imu) and `imu/mag` (sensor_msgs/MagneticField)
  3) `imu_filter_madgwick` → publishes fused `imu/data` (sensor_msgs/Imu with orientation)
  4) (Optional) Quaternion→heading node → publishes `/compass_madgwick` (std_msgs/Float64)

- Option B (later): update `imu.py` to also publish `sensor_msgs/Imu` and `sensor_msgs/MagneticField` directly, skipping the converter. The filter then runs without the extra node.

This README assumes Option A so you can test immediately without modifying `imu.py`.

---

### Run: Converter + Madgwick filter

1) Start the existing IMU node (raw data + built-in tilt heading):
```bash
python3 /home/orangepi/argo/nodes/imu.py --debug
```

2) Start a small Python converter that republishes in standard message types.

Until we integrate the converter into the repo, you can run an ad-hoc Python one-liner or drop a short script. The converter must do unit conversions:
- accel: g → m/s² (× 9.80665)
- gyro: deg/s → rad/s (× π/180)
- mag: µT → T (× 1e-6)

Example topic names (expected by Madgwick default params):
- Publish IMU as `imu/data_raw`
- Publish magnetometer as `imu/mag`

3) Run the Madgwick filter:
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=true \
  -p world_frame:=enu \
  -p publish_tf:=false
```

Outputs: `imu/data` (sensor_msgs/Imu) with orientation quaternion.

4) (Optional) Convert quaternion to a heading topic for side‑by‑side comparison:
```bash
# Example: view quaternion (orientation) directly
ros2 topic echo /imu/data

# Or run a small Python node that subscribes to /imu/data and publishes /compass_madgwick (Float64)
```

Now compare:
```bash
ros2 topic echo /compass              # built-in tilt-compensated heading from imu.py
ros2 topic echo /compass_madgwick     # fused heading (optional node)
```

---

### How They Work Together
- `imu.py` (hardware driver + tilt compensation):
  - Reads ICM-20948 accel/gyro and AK09916 magnetometer
  - Publishes raw vectors and a tilt‑compensated heading

- Converter node (Python):
  - Subscribes: `/accel`, `/gyro`, `/magnetometer`
  - Converts units and republishes as `sensor_msgs/Imu` (`imu/data_raw`) and `sensor_msgs/MagneticField` (`imu/mag`)

- `imu_filter_madgwick`:
  - Subscribes: `imu/data_raw`, `imu/mag`
  - Fuses accel+gyro+mag → publishes `imu/data` with orientation quaternion

- (Optional) Quaternion→Heading node:
  - Subscribes: `/imu/data` → computes yaw/heading (0–360°) → `/compass_madgwick`

---

### Practical Tips for Robust Heading (addresses sensitivity you observed)

- Magnetometer calibration quality is everything:
  - Perform a full hard‑iron and soft‑iron calibration (ellipsoid fit). The current `imu.py` supports a min‑max diagonal approximation; a full 3×3 fit is more robust in distorted fields.
  - Recalibrate if the sensor is moved, rotated, or nearby ferrous objects change.

- Mounting and environment:
  - Keep the IMU away from steel fasteners, motors, batteries, magnets, and high currents.
  - Secure rigid mounting; avoid flex or micro‑vibrations that change the local field seen by the sensor.

- Frame and axis consistency:
  - Ensure magnetometer axis mapping matches the accelerometer frame. In `imu.py`, mag Y is inverted to match accel frame. Any mismatch causes yaw to swing with small tilts.
  - Verify `world_frame` in Madgwick: use `enu` for standard ROS frames.

- Units and rates:
  - Confirm conversions: accel (m/s²), gyro (rad/s), mag (Tesla). Wrong units destabilize filters.
  - Provide consistent sample rates (e.g., 10 Hz). Madgwick expects stable dt.

- Filtering and rejection:
  - Low‑pass magnetometer modestly (already done in `imu.py`) to reduce noise.
  - Reject mag samples when field magnitude is far from expected Earth field (~25–65 µT) to avoid transient distortions.

- Parameter tuning (Madgwick):
  - If available, adjust the beta parameter (gain) for your noise level; too aggressive can overreact, too low can lag.

Symptoms you described (true north swings wildly with small orientation change) are classic for: mis‑aligned frames, poor calibration (soft‑iron distortions), or unit mismatches. Start by verifying axis alignment and performing a high‑quality calibration away from ferrous objects.

---

### Quick Verification Checklist
- [ ] `imu.py` publishes sane values (accel ~1g magnitude at rest, gyro near 0 deg/s at rest, mag 25–65 µT total)
- [ ] Converter outputs correct units/types
- [ ] `imu_filter_madgwick` receives both `imu/data_raw` and `imu/mag`
- [ ] Orientation from `/imu/data` is stable when device is held still
- [ ] Heading agrees with `/compass` within a reasonable margin when level

---

### Commands Summary
```bash
# Start IMU
python3 /home/orangepi/argo/nodes/imu.py --debug

# Start Madgwick (after converter is running)
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -p use_mag:=true -p world_frame:=enu -p publish_tf:=false

# Inspect outputs
ros2 topic echo /imu/data
ros2 topic echo /compass
# (optional)
ros2 topic echo /compass_madgwick
```

---

### TODO (for later integration)
- Integrate the converter and quaternion→heading nodes into the repository under `nodes/`.
- Add a launch option in `launch/argo_lifecycle_manager.py` to start the Madgwick pipeline automatically and expose `/compass_madgwick`.
- Consider enhancing calibration in `imu.py` to support a full 3×3 soft‑iron fit for improved robustness.

---

### Notes
- When running with `sudo`, source the ROS 2 environment explicitly:
```bash
sudo bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'
```


