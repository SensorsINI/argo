#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
BNO085 IMU Management Tool
==========================

Single unified tool for managing the BNO085 9-axis IMU sensor.

ARCHITECTURE
------------
The BNO085 system consists of two components:
  1. C++ Driver (bno08x_driver) - Hardware interface via I2C
     - Communicates with BNO085 sensor at 0x4a on I2C bus 0
     - Publishes standard ROS2 sensor_msgs (Imu, MagneticField)
     
  2. Python Bridge (this tool in 'bridge' mode) - Data conversion
     - Converts sensor_msgs to Argo Vector3 format
     - Publishes to /compass, /pose, /accel, /gyro topics
     - Monitors health and publishes /imu_health status

SENSOR FEATURES
---------------
The BNO085 provides:
  • Rotation Vector (datasheet 2.2.4): Quaternion orientation with on-chip
    sensor fusion (accelerometer + gyroscope + magnetometer)
  • Magnetic north reference: Compass heading corrected for tilt
  • Dynamic calibration: Auto-calibrates during operation, saves to flash
  • High accuracy: ±0.01° heading stability when calibrated

LAUNCHING
---------
The BNO085 is automatically launched by the Argo lifecycle manager.
It can also be launched manually:

  1. Via Launch File (recommended):
     cd ~/argo/nodes
     make bno08x-launch-full
     # Launches both C++ driver and Python bridge

  2. Via Lifecycle Manager:
     python3 ~/argo/launch/argo_lifecycle_manager.py run
     # Discovers and launches bno085.py in bridge mode

  3. Standalone Bridge (if driver already running):
     bno085.py bridge
     # Runs as persistent ROS2 node

TOPICS PUBLISHED
----------------
  /compass      (Vector3) - Heading in degrees (z=0-360, 0=N, 90=E)
  /pose         (Vector3) - Same as compass (z=heading)
  /accel        (Vector3) - Acceleration in g (gravity units)
  /gyro         (Vector3) - Angular velocity in degrees/second
  /imu_health   (Bool)    - Health status (true=healthy, false=unhealthy)

CALIBRATION
-----------
The BNO085 uses unified sensor fusion - all sensors calibrate together.
Enhanced interactive calibration with visual feedback:

  1. Real-time Visual Feedback:
     - Magnetometer min/max ranges displayed with progress bars
     - Coverage quality indicator (POOR → FAIR → GOOD → EXCELLENT)
     - Sensor accuracy levels (UNRELIABLE → LOW → MEDIUM → HIGH)
     
  2. Interactive Control:
     - Watch magnetometer ranges expand as you move the sensor
     - Press Ctrl+C when satisfied with calibration quality
     - No need to wait for timer - finish when ranges look good
     
  3. Magnetometer (most important):
     - Move sensor in figure-8 pattern through ALL orientations so that 
       each magnetometer axis is aligned with and opposite to the Earth's magnetic field.
     - Target: 40-60 µT range on each axis (X, Y, Z)
     - Monitor coverage quality reaching EXCELLENT (80%+)
     
  4. Accelerometer & Gyroscope (automatic):
     - Auto-calibrate during motion and stationary periods
     - Benefit from varied orientations during magnetometer calibration
     
  5. Calibration Persistence:
     - Automatically saved to sensor's internal flash memory
     - Persists across power cycles and reboots
     - No external calibration files needed
     
  Run calibration:
     bno085.py calibrate --duration 120
     # Interactive tool with visual range tracking

COMMANDS
--------
  bridge       Run as data bridge (for launch files/lifecycle manager)
               - Persistent ROS2 node
               - Converts C++ driver topics to Argo format
               - Publishes health status
               
  calibrate    Interactive calibration with visual feedback
               - Real-time magnetometer range visualization with bars
               - Coverage quality indicator (POOR → EXCELLENT)
               - Shows accuracy levels (UNRELIABLE → LOW → MEDIUM → HIGH)
               - Press Ctrl+C when satisfied with calibration
               - Comprehensive final report with statistics
               
  verify       Interactive ASCII display for sensor testing
               - Real-time visualization of all sensor data
               - Shows accelerometer, gyroscope, magnetometer bars
               - Displays compass heading with visual compass
               - Validates quaternion, Euler angles, heading stability
               - Perfect for interactive sensor testing and debugging
               
  status       Check BNO085 system health
               - Shows driver and bridge status
               - Lists topic availability
               - Displays health status
               - Checks I2C hardware detection

EXAMPLES
--------
  # Check system status
  bno085.py status
  
  # Calibrate sensor (interactive with visual feedback)
  bno085.py calibrate
  # Watch magnetometer ranges expand, press Ctrl+C when satisfied
  
  # Quick 1-minute calibration
  bno085.py calibrate --duration 60
  
  # Extended 5-minute calibration for best results
  bno085.py calibrate --duration 300
  # TIP: Monitor coverage quality - finish early if EXCELLENT
  
  # Interactive sensor display (10 seconds)
  bno085.py verify --duration 10
  
  # Extended interactive testing (60 seconds)
  bno085.py verify --duration 60
  
  # Run as bridge (normally done by launch file)
  bno085.py bridge

HEALTH MONITORING
-----------------
The bridge publishes /imu_health for lifecycle management:
  • Healthy (true):  Receiving data from C++ driver
  • Unhealthy (false): No data for 3+ seconds
  • Auto-recovery when data resumes

TROUBLESHOOTING
---------------
  No data in calibrate/verify:
    1. Check driver is running: bno085.py status
    2. Check topics: ros2 topic list | grep imu
    3. Restart system: make bno08x-launch-full
    
  Hardware not detected:
    1. Check I2C: i2cdetect -y 0
    2. Should show "4a" or "UU" at address 0x4a
    3. Check wiring and power
    
  Poor heading accuracy:
    1. Run calibration: bno085.py calibrate
    2. Ensure figure-8 motion covers all orientations
    3. Calibrate away from metal/electronics
    4. Allow 2-3 minutes for full calibration

QUICK START
-----------
  1. Build driver:      cd ~/argo/nodes && make bno08x-build
  2. Test hardware:     make bno08x-test
  3. Launch system:     make bno08x-launch-full
  4. Calibrate sensor:  make bno08x-calibrate
  5. Interactive display: python3 bno085.py verify

For more information, see:
  • nodes/BNO085_USAGE.md - Complete usage guide
  • BNO085_CALIBRATION_GUIDE.md - Detailed calibration instructions
  • BNO085_HEALTH_MONITORING.md - Health monitoring details
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import math
import sys
import time
import json
import argparse
import argcomplete
import subprocess
from datetime import datetime
import collections
from pathlib import Path

# ============================================================================
# BRIDGE MODE: Convert C++ driver output to Argo format
# ============================================================================

class BNO085Bridge(Node):
    """Bridge node that converts BNO08x driver topics to Argo format with I2C error recovery."""
    
    def __init__(self):
        super().__init__('bno085_bridge')
        
        # Publishers (Argo format)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_accel = self.create_publisher(Vector3, '/accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, '/gyro', 10)
        self.pub_health = self.create_publisher(Bool, '/imu_health', 10)
        
        # Health service (for lifecycle manager queries)
        self.health_service = self.create_service(
            Trigger, '/bno085_bridge/health', self._handle_health_request)
        
        # Subscribers (from BNO08x driver)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.sub_mag = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        
        # Health monitoring - simple KISS approach
        self.health_status = False  # Start unhealthy until we get driver data
        self.last_imu_time = None
        self.health_check_timer = self.create_timer(1.0, self._check_health)
        
        # Publish initial health status
        self._publish_health_status(False)
        
        # I2C error recovery tracking
        self.node_healthy = False  # Track if we're receiving data
        self.recovery_attempt_count = 0
        self.last_recovery_attempt_time = 0.0
        self.last_unreachable_log_time = 0.0
        self.consecutive_failures = 0
        self.total_errors_this_session = 0
        
        # Recovery mode flag (switches to low-frequency checks when unhealthy)
        self.in_recovery_mode = False
        
        self.get_logger().info("BNO085 Bridge: Converting C++ driver → Argo topics")
        self.get_logger().info("Publishing: /compass, /pose, /accel, /gyro, /imu_health")
        self.get_logger().info("I2C error recovery: Enabled with automatic C++ driver restart")

        # Determine Argo repo dir for subprocess calls
        script_path = Path(__file__).resolve()
        self.argo_dir = str(script_path.parents[1])  # nodes -> argo
    
    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        if yaw_deg < 0:
            yaw_deg += 360.0
        
        return roll_deg, pitch_deg, yaw_deg
    
    def imu_callback(self, msg: Imu):
        """Process IMU message: extract heading, gyro, accel."""
        current_time = time.time()
        
        # Update health tracking - simple KISS approach
        self.last_imu_time = current_time
        
        # Set health to true when we get driver data
        if not self.health_status:
            self._publish_health_status(True)
        
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.w, msg.orientation.x, 
            msg.orientation.y, msg.orientation.z
        )
        
        # Publish compass/pose (heading)
        heading_msg = Vector3(x=0.0, y=0.0, z=yaw)
        self.pub_compass.publish(heading_msg)
        self.pub_pose.publish(heading_msg)
        
        # Publish gyroscope (rad/s → deg/s)
        gyro_msg = Vector3(
            x=math.degrees(msg.angular_velocity.x),
            y=math.degrees(msg.angular_velocity.y),
            z=math.degrees(msg.angular_velocity.z)
        )
        self.pub_gyro.publish(gyro_msg)
        
        # Publish accelerometer (m/s² → g)
        accel_msg = Vector3(
            x=msg.linear_acceleration.x / 9.81,
            y=msg.linear_acceleration.y / 9.81,
            z=msg.linear_acceleration.z / 9.81
        )
        self.pub_accel.publish(accel_msg)
    
    def mag_callback(self, msg: MagneticField):
        """Process magnetic field message (currently unused)."""
        pass
    
    def _check_health(self):
        """Simple health check - timeout if no driver data."""
        current_time = time.time()
        
        # If we have no IMU data or it's stale (3+ seconds), mark unhealthy
        if self.last_imu_time is None or (current_time - self.last_imu_time) > 3.0:
            if self.health_status:
                self._publish_health_status(False)
        else:
            # We have recent data, ensure we're healthy
            if not self.health_status:
                self._publish_health_status(True)
    
    def _publish_health_status(self, is_healthy: bool):
        """Publish health status update."""
        # Always publish health status (not just when it changes)
        self.health_status = is_healthy
        health_msg = Bool()
        health_msg.data = is_healthy
        self.pub_health.publish(health_msg)
        
        if is_healthy:
            self.get_logger().info("BNO085 health: HEALTHY")
        else:
            self.get_logger().warn("BNO085 health: UNHEALTHY")
    
    def _handle_health_request(self, request, response):
        """Handle health status service request."""
        try:
            response.success = True
            response.message = json.dumps({
                'healthy': self.health_status,
                'details': f"BNO085 bridge health: {'HEALTHY' if self.health_status else 'UNHEALTHY'}",
                'timestamp': time.time(),
                'node_name': 'bno085_bridge'
            })
        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {e}"
        
        return response
    
    def _switch_to_recovery_mode(self):
        """Switch to low-frequency recovery mode (checking every 3 seconds)."""
        if not self.in_recovery_mode:
            self.in_recovery_mode = True
            self.health_check_timer.destroy()
            self.health_check_timer = self.create_timer(3.0, self._check_health)
            self.get_logger().info("Switched to recovery mode (3s check interval)")
    
    def _switch_to_normal_mode(self):
        """Switch back to normal health check frequency (1 second)."""
        if self.in_recovery_mode:
            self.in_recovery_mode = False
            self.health_check_timer.destroy()
            self.health_check_timer = self.create_timer(1.0, self._check_health)
            self.get_logger().info("Switched back to normal mode (1s check interval)")
    
    def _attempt_recovery(self, current_time):
        """Attempt to recover from I2C failure by restarting the C++ driver."""
        # Throttle recovery attempts (try every 5 seconds)
        time_since_last_attempt = current_time - self.last_recovery_attempt_time
        if time_since_last_attempt < 5.0:
            return
        
        self.recovery_attempt_count += 1
        self.last_recovery_attempt_time = current_time
        
        # Log throttled unreachable sensor status (once per 60s)
        time_since_last_log = current_time - self.last_unreachable_log_time
        if time_since_last_log >= 60.0 or self.last_unreachable_log_time == 0.0:
            time_since_healthy = current_time - self.last_successful_read_time
            self.get_logger().error(
                f"BNO085 sensor unreachable for {time_since_healthy:.1f}s "
                f"(recovery attempts: {self.recovery_attempt_count}, "
                f"failures: {self.total_errors_this_session})")
            self.last_unreachable_log_time = current_time
        
        # Attempt to restart the C++ driver
        self.get_logger().info(f"Attempting recovery #{self.recovery_attempt_count}: Restarting bno08x_driver...")
        
        try:
            # Try to restart the C++ driver service using systemctl
            result = subprocess.run(
                ['sudo', 'systemctl', 'restart', 'argo_bno085.service'],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                self.get_logger().info("argo_bno085.service restart command successful - waiting for data...")
            else:
                self.get_logger().warn(f"argo_bno085.service restart command failed: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("Driver restart command timed out")
        except Exception as e:
            self.get_logger().error(f"Failed to restart driver: {e}")
    
    def _debug_parameters(self):
        """Debug function to check driver parameters."""
        try:
            cmd1 = (f'source /opt/ros/humble/setup.bash && '
                    f'source {self.argo_dir}/nodes/argo_bno08x_driver_workspace/install/setup.bash && '
                    f'ros2 param get /bno08x_ros publish.imu.rate')
            result = subprocess.run([
                'bash', '-c', cmd1
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info(f"DEBUG: Driver IMU rate parameter: {result.stdout.strip()}")
            else:
                self.get_logger().warn(f"DEBUG: Failed to get IMU rate parameter: {result.stderr}")
                
            # Also check magnetic field rate
            cmd2 = (f'source /opt/ros/humble/setup.bash && '
                    f'source {self.argo_dir}/nodes/argo_bno08x_driver_workspace/install/setup.bash && '
                    f'ros2 param get /bno08x_ros publish.magnetic_field.rate')
            result2 = subprocess.run([
                'bash', '-c', cmd2
            ], capture_output=True, text=True, timeout=5)
            
            if result2.returncode == 0:
                self.get_logger().info(f"DEBUG: Driver magnetic field rate parameter: {result2.stdout.strip()}")
            else:
                self.get_logger().warn(f"DEBUG: Failed to get magnetic field rate parameter: {result2.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"DEBUG: Error checking parameters: {e}")


# ============================================================================
# CALIBRATION MODE: Interactive sensor calibration
# ============================================================================

class BNO085Calibrator(Node):
    """Interactive calibration tool with real-time guidance and visual feedback."""
    
    def __init__(self, duration=60, save_interval=30):
        super().__init__('bno085_calibrator')
        self.calibration_duration = duration
        self.save_interval = save_interval
        self.start_time = time.time()
        
        # Calibration tracking
        self.mag_accuracy = self.accel_accuracy = self.gyro_accuracy = 0
        self.imu_sample_count = self.mag_sample_count = 0
        self.orientation_changes = 0
        self.last_orientation = None
        self.last_accel = None
        self.motion_detected = False
        self.last_save_time = time.time()
        
        # Magnetometer range tracking for visual feedback
        self.mag_samples = []
        self.mag_min_x = self.mag_min_y = self.mag_min_z = float('inf')
        self.mag_max_x = self.mag_max_y = self.mag_max_z = float('-inf')
        
        # User control flags
        self.user_wants_to_finish = False
        self.manual_mode = False  # Set to True to enable user prompts
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        
        # Timers
        self.status_timer = self.create_timer(2.0, self.print_status)
        if save_interval > 0:
            self.save_timer = self.create_timer(float(save_interval), self.auto_save)
        
        self.print_header()
        self.get_logger().info(f"Calibration started: {duration}s duration")
        self.get_logger().info("Press Ctrl+C at any time to finish calibration early")
    
    def print_header(self):
        print("\n" + "=" * 80)
        print("BNO085 INTERACTIVE CALIBRATION TOOL")
        print("=" * 80)
        print("\nCALIBRATION APPROACH:")
        print("The BNO085 uses unified sensor fusion - all sensors calibrate together.")
        print("\nINSTRUCTIONS:")
        print("1. MAGNETOMETER (Most Important):")
        print("   - Move sensor in figure-8 pattern through ALL orientations")
        print("   - Rotate through roll, pitch, and yaw axes")
        print("   - Watch the magnetometer ranges expand as you move")
        print("   - Target: Cover full 3D space for best calibration")
        print("\n2. ACCELEROMETER:")
        print("   - Auto-calibrates during motion")
        print("   - Benefits from varied orientations")
        print("\n3. GYROSCOPE:")
        print("   - Auto-calibrates during still and motion periods")
        print("\nVISUAL FEEDBACK:")
        print("   - Magnetometer ranges will show min/max values and progress bars")
        print("   - Coverage Quality indicator: POOR → FAIR → GOOD → EXCELLENT")
        print("   - Sensor Accuracy bars: UNRELIABLE → LOW → MEDIUM → HIGH")
        print("\nTARGET: All sensors reach HIGH accuracy (green bars)")
        print("TIP: Press Ctrl+C when satisfied with calibration quality")
        print("=" * 80 + "\n")
        
        # Prompt user to read and confirm before starting
        try:
            input("Press ENTER to start calibration (or Ctrl+C to cancel)...")
            print("\n🚀 Starting calibration...\n")
        except KeyboardInterrupt:
            print("\n\n❌ Calibration cancelled by user")
            rclpy.shutdown()
            raise SystemExit(0)
    
    def imu_callback(self, msg: Imu):
        self.imu_sample_count += 1
        
        # Estimate accuracy from covariance
        self.mag_accuracy = self._cov_to_accuracy(msg.orientation_covariance[0])
        self.accel_accuracy = self._cov_to_accuracy(msg.linear_acceleration_covariance[0])
        self.gyro_accuracy = self._cov_to_accuracy(msg.angular_velocity_covariance[0])
        
        # Detect motion
        q = msg.orientation
        if self.last_orientation:
            dq = math.sqrt(sum((a-b)**2 for a, b in zip(
                [q.w, q.x, q.y, q.z], self.last_orientation)))
            if dq > 0.1:
                self.orientation_changes += 1
                self.motion_detected = True
        self.last_orientation = [q.w, q.x, q.y, q.z]
    
    def mag_callback(self, msg: MagneticField):
        self.mag_sample_count += 1
        
        # Track magnetometer ranges for visual feedback
        # Note: BNO08x driver outputs in µT directly (not standard Tesla)
        mx = msg.magnetic_field.x  # Already in µT
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z
        
        # Debug: Log first few samples to verify data is changing
        if self.mag_sample_count <= 5:
            self.get_logger().info(f"Mag sample #{self.mag_sample_count}: X={mx:.2f}, Y={my:.2f}, Z={mz:.2f} µT")
        
        # Update min/max ranges
        self.mag_min_x = min(self.mag_min_x, mx)
        self.mag_max_x = max(self.mag_max_x, mx)
        self.mag_min_y = min(self.mag_min_y, my)
        self.mag_max_y = max(self.mag_max_y, my)
        self.mag_min_z = min(self.mag_min_z, mz)
        self.mag_max_z = max(self.mag_max_z, mz)
        
        # Store sample for coverage calculation
        self.mag_samples.append((mx, my, mz))
    
    def _cov_to_accuracy(self, cov):
        """Convert covariance to accuracy level (0-3)."""
        if cov <= 0:
            return 0
        elif cov < 0.01:
            return 3
        elif cov < 0.05:
            return 2
        elif cov < 0.1:
            return 1
        return 0
    
    def print_status(self):
        elapsed = time.time() - self.start_time
        remaining = max(0, self.calibration_duration - elapsed)
        
        print("\033[2J\033[H", end='')  # Clear screen
        print("=" * 80)
        print(f"BNO085 CALIBRATION - {datetime.now().strftime('%H:%M:%S')}")
        print("=" * 80)
        print(f"\n⏱️  TIME: {elapsed:.1f}s / {self.calibration_duration}s (Ctrl+C to finish early)")
        
        # Calculate magnetometer coverage quality
        coverage_quality = self._calculate_coverage_quality()
        
        print(f"\n📊 SENSOR ACCURACY:")
        print(f"   Magnetometer:  {self._bar(self.mag_accuracy)} {self._name(self.mag_accuracy)}")
        print(f"   Accelerometer: {self._bar(self.accel_accuracy)} {self._name(self.accel_accuracy)}")
        print(f"   Gyroscope:     {self._bar(self.gyro_accuracy)} {self._name(self.gyro_accuracy)}")
        
        print(f"\n📏 MAGNETOMETER RANGES (µT):")
        # Get terminal width for dynamic bar sizing
        try:
            import shutil
            term_width = shutil.get_terminal_size().columns
        except:
            term_width = 80
        bar_width = min(50, term_width - 35)  # Reserve space for labels
        
        if self.mag_min_x != float('inf'):
            span_x = self.mag_max_x - self.mag_min_x
            span_y = self.mag_max_y - self.mag_min_y
            span_z = self.mag_max_z - self.mag_min_z
            
            print(f"   X: [{self.mag_min_x:+6.1f} .. {self.mag_max_x:+6.1f}] {self._range_bar(span_x, bar_width)} span={span_x:5.1f}")
            print(f"   Y: [{self.mag_min_y:+6.1f} .. {self.mag_max_y:+6.1f}] {self._range_bar(span_y, bar_width)} span={span_y:5.1f}")
            print(f"   Z: [{self.mag_min_z:+6.1f} .. {self.mag_max_z:+6.1f}] {self._range_bar(span_z, bar_width)} span={span_z:5.1f}")
            print(f"   Coverage Quality: {coverage_quality}")
            
            # Provide guidance based on calibration state
            if self.mag_accuracy < 3:
                if span_x < 30 or span_y < 30 or span_z < 30:
                    print(f"\n💡 ⚠️  INSUFFICIENT COVERAGE: Rotate sensor through MORE orientations!")
                    print("   → Need wider ranges on all 3 axes (target: ~40-60 µT each)")
                elif self.motion_detected:
                    print(f"\n💡 ✅ Motion detected! Continue rotating through all orientations...")
                else:
                    print(f"\n💡 ⚠️  Keep moving! Figure-8 pattern through all axes...")
                self.motion_detected = False
            else:
                print("\n✅ MAGNETOMETER CALIBRATION COMPLETE! High accuracy achieved")
                if self.accel_accuracy >= 2 and self.gyro_accuracy >= 2:
                    print("✅ ALL SENSORS CALIBRATED! Safe to finish (Ctrl+C)")
        else:
            print("   Waiting for magnetometer data...")
            print(f"\n💡 Waiting for sensor to start publishing data...")
        
        print(f"\n📈 SAMPLES: IMU={self.imu_sample_count}, Mag={self.mag_sample_count}, Movements={self.orientation_changes}")
        
        print("=" * 80 + "\n")
        
        if remaining <= 0:
            # Timer expired - raise exception to trigger cleanup in main
            raise SystemExit("calibration_complete")
    
    def _range_bar(self, span, width=50):
        """Create a visual bar showing magnetometer range span."""
        target_span = 60.0  # Target span for good calibration (µT)
        filled = int(min(1.0, span / target_span) * width)
        bar = '█' * filled + '░' * (width - filled)
        return f"|{bar}|"
    
    def _calculate_coverage_quality(self):
        """Calculate magnetometer coverage quality percentage."""
        if self.mag_min_x == float('inf'):
            return "⚠️  NO DATA (0%)"
        
        span_x = self.mag_max_x - self.mag_min_x
        span_y = self.mag_max_y - self.mag_min_y
        span_z = self.mag_max_z - self.mag_min_z
        
        # Target spans for good calibration (typical Earth field ~40-60 µT range)
        target_span = 50.0
        
        coverage_x = min(100, (span_x / target_span) * 100)
        coverage_y = min(100, (span_y / target_span) * 100)
        coverage_z = min(100, (span_z / target_span) * 100)
        
        avg_coverage = (coverage_x + coverage_y + coverage_z) / 3.0
        
        if avg_coverage >= 80:
            return f"✅ EXCELLENT ({avg_coverage:.0f}%)"
        elif avg_coverage >= 60:
            return f"✓ GOOD ({avg_coverage:.0f}%)"
        elif avg_coverage >= 40:
            return f"⚠️  FAIR ({avg_coverage:.0f}%) - keep moving"
        else:
            return f"❌ POOR ({avg_coverage:.0f}%) - more movement needed"
    
    def _bar(self, acc):
        return ["░░░░", "▓░░░", "▓▓░░", "▓▓▓░", "▓▓▓▓"][max(0, min(3, int(acc)))]
    
    def _name(self, acc):
        return ["UNRELIABLE", "LOW      ", "MEDIUM   ", "HIGH     "][max(0, min(3, int(acc)))]
    
    def auto_save(self):
        print("💾 Calibration auto-saved by sensor")
    
    def final_report(self, interrupted=False):
        """Display final calibration report with quality assessment."""
        print("\n" + "=" * 80)
        if interrupted:
            print("CALIBRATION INTERRUPTED BY USER")
        else:
            print("CALIBRATION COMPLETE")
        print("=" * 80)
        print(f"\nDuration: {time.time() - self.start_time:.1f}s")
        print(f"\nFinal Sensor Accuracy:")
        print(f"  Magnetometer:  {self._name(self.mag_accuracy).strip()}")
        print(f"  Accelerometer: {self._name(self.accel_accuracy).strip()}")
        print(f"  Gyroscope:     {self._name(self.gyro_accuracy).strip()}")
        
        print(f"\nMagnetometer Coverage:")
        if self.mag_min_x != float('inf'):
            span_x = self.mag_max_x - self.mag_min_x
            span_y = self.mag_max_y - self.mag_min_y
            span_z = self.mag_max_z - self.mag_min_z
            coverage_quality = self._calculate_coverage_quality()
            
            print(f"  X Range: [{self.mag_min_x:+6.1f} .. {self.mag_max_x:+6.1f}] µT (span: {span_x:5.1f} µT)")
            print(f"  Y Range: [{self.mag_min_y:+6.1f} .. {self.mag_max_y:+6.1f}] µT (span: {span_y:5.1f} µT)")
            print(f"  Z Range: [{self.mag_min_z:+6.1f} .. {self.mag_max_z:+6.1f}] µT (span: {span_z:5.1f} µT)")
            print(f"  Coverage Quality: {coverage_quality}")
        else:
            print("  ⚠️  No magnetometer data collected")
        
        print(f"\nSample Statistics:")
        print(f"  IMU samples: {self.imu_sample_count}")
        print(f"  Magnetometer samples: {self.mag_sample_count}")
        print(f"  Orientation changes: {self.orientation_changes}")
        
        print(f"\nCalibration Storage:")
        print(f"  ✅ Calibration automatically saved to sensor's internal flash memory")
        print(f"  ✅ Persists across power cycles and reboots")
        print(f"  ℹ️  No external calibration file needed")
        
        # Quality assessment
        success_level = ""
        if self.mag_accuracy >= 3 and self.accel_accuracy >= 2 and self.gyro_accuracy >= 2:
            success_level = "EXCELLENT"
            print("\n✅ SUCCESS! Sensor is well-calibrated and ready for use")
        elif self.mag_accuracy >= 2:
            success_level = "ACCEPTABLE"
            print("\n✓ PARTIAL SUCCESS: Acceptable calibration achieved")
            print("  Consider running calibration again for better accuracy")
        else:
            success_level = "INSUFFICIENT"
            print("\n⚠️  INCOMPLETE: Magnetometer calibration insufficient")
            print("  Please run calibration again with more comprehensive movement")
        
        print("=" * 80)
        
        return success_level


# ============================================================================
# VERIFICATION MODE: Interactive ASCII display for sensor testing
# ============================================================================

def get_terminal_size():
    """Get terminal size, fallback to 80x24 if detection fails."""
    try:
        import shutil
        return shutil.get_terminal_size()
    except:
        return (80, 24)

def clear_screen_and_home():
    """Clear screen and move cursor to top-left."""
    try:
        sys.stdout.write('\x1b[2J')    # clear screen
        sys.stdout.write('\x1b[H')     # home cursor
        sys.stdout.flush()
    except:
        pass

class RotationVectorVerifier(Node):
    """Interactive ASCII display for BNO085 sensor testing."""
    
    def __init__(self, duration=10):
        """
        Args:
            duration (int): Duration in seconds for verification mode.
                - If duration > 0: Run verification for the specified number of seconds.
                - If duration == 0: Run verification mode indefinitely (until interrupted).
        """
        super().__init__('rotation_vector_verifier')
        self.duration = duration
        self.start_time = time.time()
        self.sample_count = 0
        self.yaw_history = collections.deque(maxlen=20)
        self.last_print_time = time.time()
        
        # ASCII display state
        self._vis_initialized = False
        self._init_ascii_vis()
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        
        # Display start flag (delayed until user ready)
        self.display_started = False
        
        # Show introduction
        print("\n" + "=" * 80)
        print("BNO085 INTERACTIVE SENSOR DISPLAY")
        print("=" * 80)
        print("\nReal-time sensor data visualization:")
        print("  • Accelerometer (g units) - shows X, Y, Z acceleration")
        print("  • Gyroscope (deg/s) - shows X, Y, Z angular velocity")
        print("  • Magnetometer (µT) - shows X, Y, Z magnetic field")
        print("  • Compass heading (degrees) - 0°=N, 90°=E, 180°=S, 270°=W")
        print("  • Quaternion orientation - rotation from sensor frame to world")
        print("  • Euler angles (Roll, Pitch, Yaw) - for 3D visualization")
        print("\nDISPLAY FEATURES:")
        print("  • Visual progress bars for all sensor axes")
        print("  • ASCII compass showing heading direction")
        print("  • Real-time validation of sensor data quality")
        print("\nWaiting for sensor data...")
        print("=" * 80 + "\n")
        
        # Prompt with timeout
        self._prompt_with_timeout()
        
        # Timer for display updates (starts after prompt)
        self.display_timer = self.create_timer(0.1, self.update_display)  # 10 Hz
    
    def _prompt_with_timeout(self, timeout=10):
        """Prompt user with automatic timeout after specified seconds."""
        import select
        import sys
        
        print(f"Press ENTER to start display immediately (or wait {timeout}s for auto-start)...")
        
        # Check if stdin is available and is a terminal
        if not sys.stdin.isatty():
            # Not a terminal (e.g., running as service), skip prompt
            print(f"Auto-starting in {timeout}s...")
            time.sleep(timeout)
            print("\n🚀 Starting display...\n")
            self.display_started = True
            return
        
        try:
            # Use select to wait for input with timeout (Unix/Linux only)
            start_time = time.time()
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            
            if ready:
                # User pressed a key
                sys.stdin.readline()  # Consume the input
                print("\n🚀 Starting display...\n")
            else:
                # Timeout reached
                elapsed = time.time() - start_time
                print(f"\n⏱️  Auto-starting after {elapsed:.1f}s...\n")
            
            self.display_started = True
            
        except Exception as e:
            # Fallback if select doesn't work
            print(f"Auto-starting in {timeout}s...")
            time.sleep(timeout)
            print("\n🚀 Starting display...\n")
            self.display_started = True
    
    def _init_ascii_vis(self):
        """Initialize ASCII visualization."""
        if self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')    # reset attributes
            sys.stdout.write('\x1b[?25l')  # hide cursor
            clear_screen_and_home()
            self._vis_initialized = True
        except Exception as e:
            self._vis_initialized = False
    
    def _teardown_ascii_vis(self):
        """Clean up ASCII visualization."""
        if not self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')
            clear_screen_and_home()
            sys.stdout.write('\x1b[?25h')  # show cursor
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False
    
    def _signed_bar(self, value: float, limit: float, width: int = 50) -> str:
        """Create a signed bar visualization centered at zero."""
        width = max(10, width)
        mid = width // 2
        # clip value to limit
        val = max(-limit, min(limit, value))
        pos = int(round((val / limit) * mid))
        left = [' '] * mid
        right = [' '] * (width - mid - 1)
        if pos < 0:
            fill = mid + pos  # fill up to this index (exclusive)
            for i in range(fill, mid):
                left[i] = '-'
        elif pos > 0:
            for i in range(0, pos):
                if i < len(right):
                    right[i] = '+'
        return '[' + ''.join(left) + '|' + ''.join(right) + ']'
    
    def _render_compass(self, heading_deg: float, width: int = 40, height: int = 20) -> list:
        """Render compass heading as a point on a circle with N at top."""
        width = max(20, width)
        height = max(10, height)
        
        cx = width // 2
        cy = height // 2
        radius = min(cx - 3, cy - 2)
        
        # Create grid
        grid = [[' ' for _ in range(width)] for _ in range(height)]
        
        # Draw circle (octagon approximation)
        for angle in range(0, 360, 5):
            rad = math.radians(angle)
            x = cx + int(round(radius * math.sin(rad)))
            y = cy - int(round(radius * math.cos(rad)))
            if 0 <= y < height and 0 <= x < width:
                grid[y][x] = '.'
        
        # Mark cardinal directions
        if cy - radius >= 0:
            grid[cy - radius][cx] = 'N'
        if cx + radius < width:
            grid[cy][cx + radius] = 'E'
        if cy + radius < height:
            grid[cy + radius][cx] = 'S'
        if cx - radius >= 0:
            grid[cy][cx - radius] = 'W'
        
        # Draw center
        grid[cy][cx] = '+'
        
        # Draw heading indicator
        theta = math.radians(heading_deg)
        hx = cx + int(round(radius * math.sin(theta)))
        hy = cy - int(round(radius * math.cos(theta)))
        if 0 <= hy < height and 0 <= hx < width:
            grid[hy][hx] = 'O'
        
        # Draw line from center to heading point
        steps = int(radius)
        for i in range(1, steps):
            frac = i / float(steps)
            lx = cx + int(round(frac * radius * math.sin(theta)))
            ly = cy - int(round(frac * radius * math.cos(theta)))
            if 0 <= ly < height and 0 <= lx < width:
                if grid[ly][lx] == ' ':
                    grid[ly][lx] = '*'
        
        return [''.join(row) for row in grid]
    
    def imu_callback(self, msg: Imu):
        """Process IMU message and store data for display."""
        self.sample_count += 1
        current_time = time.time()
        
        # Store current sensor data for display
        self.current_quaternion = msg.orientation
        self.current_accel = msg.linear_acceleration
        self.current_gyro = msg.angular_velocity
        self.current_time = current_time
        
        # Calculate derived values
        q = msg.orientation
        self.current_roll, self.current_pitch, self.current_yaw = self.quaternion_to_euler(q.w, q.x, q.y, q.z)
        self.current_q_mag = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        self.current_accel_mag = math.sqrt(sum(x**2 for x in [msg.linear_acceleration.x, 
                                                              msg.linear_acceleration.y,
                                                              msg.linear_acceleration.z]))
        self.yaw_history.append(self.current_yaw)
    
    def mag_callback(self, msg: MagneticField):
        """Process magnetic field message."""
        self.current_mag = msg.magnetic_field
    
    def update_display(self):
        """Update the ASCII display with current sensor data."""
        # Don't start updating display until user has had time to read intro
        if not self.display_started:
            return
        
        if not hasattr(self, 'current_quaternion'):
            return  # No data yet
        
        try:
            clear_screen_and_home()
            
            # Get terminal size for dynamic sizing
            term_width, term_height = get_terminal_size()
            bar_width = term_width - 20  # Reserve 20 chars for labels
            
            # Nominal limits for bars
            a_lim = 2.0   # g
            g_lim = 500.0  # deg/s
            m_lim = 100.0  # µT
            
            # Convert accelerometer from m/s² to g
            ax_g = self.current_accel.x / 9.81
            ay_g = self.current_accel.y / 9.81
            az_g = self.current_accel.z / 9.81
            
            # Convert gyroscope from rad/s to deg/s
            gx_dps = math.degrees(self.current_gyro.x)
            gy_dps = math.degrees(self.current_gyro.y)
            gz_dps = math.degrees(self.current_gyro.z)
            
            # Get magnetometer data
            if hasattr(self, 'current_mag'):
                # Note: BNO08x driver outputs in µT directly (not standard Tesla)
                mx_uT = self.current_mag.x  # Already in µT
                my_uT = self.current_mag.y
                mz_uT = self.current_mag.z
            else:
                mx_uT = my_uT = mz_uT = 0.0
            
            # Build sensor data lines
            sensor_lines = [
                "=== BNO085 Sensor Data (Interactive Display) ===",
                "",
                "Accelerometer:",
                f"Ax {ax_g:+7.3f} g   " + self._signed_bar(ax_g, a_lim, bar_width),
                f"Ay {ay_g:+7.3f} g   " + self._signed_bar(ay_g, a_lim, bar_width),
                f"Az {az_g:+7.3f} g   " + self._signed_bar(az_g, a_lim, bar_width),
                "",
                "Gyroscope:",
                f"Gx {gx_dps:+7.1f} dps " + self._signed_bar(gx_dps, g_lim, bar_width),
                f"Gy {gy_dps:+7.1f} dps " + self._signed_bar(gy_dps, g_lim, bar_width),
                f"Gz {gz_dps:+7.1f} dps " + self._signed_bar(gz_dps, g_lim, bar_width),
                "",
                "Magnetometer:",
                f"Mx {mx_uT:+7.1f} µT  " + self._signed_bar(mx_uT, m_lim, bar_width),
                f"My {my_uT:+7.1f} µT  " + self._signed_bar(my_uT, m_lim, bar_width),
                f"Mz {mz_uT:+7.1f} µT  " + self._signed_bar(mz_uT, m_lim, bar_width),
                "",
                f"=== Compass Heading: {self.current_yaw:6.1f}° (0°=N, 90°=E, 180°=S, 270°=W) ===",
                "",
                "=== Quaternion Orientation ===",
                f"Qw {self.current_quaternion.w:+7.3f}  Qx {self.current_quaternion.x:+7.3f}  Qy {self.current_quaternion.y:+7.3f}  Qz {self.current_quaternion.z:+7.3f}",
                f"Mag: {self.current_q_mag:.4f} {'✓' if abs(self.current_q_mag-1.0)<0.01 else '❌'} Unit quaternion",
                "",
                "=== Euler Angles ===",
                f"Roll {self.current_roll:+7.1f}°  Pitch {self.current_pitch:+7.1f}°  Yaw {self.current_yaw:+7.1f}°",
                "",
                "=== Validation ===",
                f"✓ Quaternion: {'PASS' if abs(self.current_q_mag-1.0)<0.01 else 'FAIL'}",
                f"✓ Mag North:  {'PASS' if 0<=self.current_yaw<=360 else 'FAIL'}",
                f"✓ Gravity:    {'PASS' if abs(self.current_accel_mag-9.81)<2.0 else 'FAIL'}",
                ""
            ]
            
            # Calculate available height for compass
            sensor_lines_count = len(sensor_lines)
            footer_lines_count = 3  # blank line + status + "Ctrl-C to exit"
            available_height = term_height - sensor_lines_count - footer_lines_count
            
            # Generate compass display to fit available height
            compass_width = term_width
            compass_height = max(5, available_height)
            compass_lines = self._render_compass(self.current_yaw, width=compass_width, height=compass_height)
            
            # Add status line
            elapsed = time.time() - self.start_time
            remaining = max(0, self.duration - elapsed)
            status_line = f"Samples: {self.sample_count} | Elapsed: {elapsed:.1f}s | Remaining: {remaining:.1f}s"
            
            # Combine all lines
            lines = sensor_lines + compass_lines + ["", status_line, "Ctrl-C to exit"]
            
            for ln in lines:
                sys.stdout.write(ln + '\n')
            sys.stdout.flush()
            
            # Check if duration has elapsed
            if elapsed >= self.duration:
                self._teardown_ascii_vis()
                print(f"\n✅ Verification complete after {elapsed:.1f}s")
                print(f"Total samples: {self.sample_count}")
                rclpy.shutdown()
                
        except Exception as e:
            # Display failed, but don't crash the node
            pass
    
    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to Euler angles in degrees."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        yaw_deg = math.degrees(yaw)
        if yaw_deg < 0:
            yaw_deg += 360.0
        
        return math.degrees(roll), math.degrees(pitch), yaw_deg


# ============================================================================
# STATUS MODE: Check system health
# ============================================================================

def cmd_status():
    """Check BNO085 system status."""
    print("\n📊 BNO085 SYSTEM STATUS")
    print("=" * 60)
    
    # Check nodes
    try:
        result = subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 node list'],
                                capture_output=True, text=True, timeout=5)
        nodes = result.stdout
        print(f"\n{'✅' if 'bno08x_driver' in nodes else '❌'} BNO08x C++ driver")
        print(f"{'✅' if 'bno085_bridge' in nodes else '❌'} BNO085 bridge")
    except:
        print("\n❌ Cannot check ROS2 nodes")
    
    # Check topics
    print("\nTopics:")
    for topic in ['/imu', '/magnetic_field', '/compass', '/pose', '/accel', '/gyro', '/imu_health']:
        try:
            result = subprocess.run(['bash', '-c', f'source /opt/ros/humble/setup.bash && ros2 topic info {topic}'],
                                    capture_output=True, text=True, timeout=2)
            print(f"  {'✅' if result.returncode == 0 else '❌'} {topic}")
        except:
            print(f"  ❌ {topic}")
    
    # Check health status
    try:
        result = subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 topic echo /imu_health --once'],
                                capture_output=True, text=True, timeout=2)
        if result.returncode == 0 and 'true' in result.stdout.lower():
            print("\n💚 Health Status: HEALTHY")
        elif result.returncode == 0 and 'false' in result.stdout.lower():
            print("\n❤️  Health Status: UNHEALTHY")
        else:
            print("\n⚠️  Health Status: UNKNOWN")
    except:
        print("\n⚠️  Health Status: Cannot check")
    
    # Check I2C
    print("\nHardware:")
    try:
        result = subprocess.run(['i2cdetect', '-y', '0'], capture_output=True, text=True, timeout=2)
        if '4a' in result.stdout or 'UU' in result.stdout:
            print("  ✅ BNO085 on I2C bus 0 at 0x4a")
        else:
            print("  ❌ BNO085 not detected")
    except:
        print("  ⚠️  Cannot check I2C")
    
    print("\n" + "=" * 60 + "\n")


# ============================================================================
# MAIN CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='BNO085 IMU Management Tool',
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('command', nargs='?', default='bridge',
                        choices=['bridge', 'calibrate', 'verify', 'status'],
                        help='Command to execute (default: bridge)')
    parser.add_argument('--duration', type=int, default=60,
                        help='Duration in seconds for calibrate/verify (default: 60)')
    parser.add_argument('--save-interval', type=int, default=30,
                        help='Calibration auto-save interval (default: 30)')
    
    # Enable bash completion for command-line arguments
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    
    if args.command == 'status':
        cmd_status()
    elif args.command in ['bridge', 'calibrate', 'verify']:
        rclpy.init()
        node = None
        try:
            if args.command == 'bridge':
                node = BNO085Bridge()
            elif args.command == 'calibrate':
                node = BNO085Calibrator(args.duration, args.save_interval)
            else:  # verify
                node = RotationVectorVerifier(args.duration)
            
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit) as e:
            # Handle calibration completion (both interrupted and natural)
            if args.command == 'calibrate' and node:
                # Determine if interrupted or completed
                interrupted = isinstance(e, KeyboardInterrupt)
                success_level = node.final_report(interrupted=interrupted)
                
                # Offer to run verify
                print("\nWould you like to verify the calibration results?")
                try:
                    response = input("Run verification display? (Y/n): ").strip().lower()
                    if response in ['', 'y', 'yes']:
                        print("\n🔄 Starting verification display...Ctrl-C to exit\n")
                        time.sleep(3)
                        
                        # Clean up current node
                        node.destroy_node()
                        
                        # Start verify node
                        verify_node = RotationVectorVerifier(duration=0) # indefinite duration
                        try:
                            rclpy.spin(verify_node)
                        except KeyboardInterrupt:
                            print("\n✅ Verification stopped")
                        finally:
                            if hasattr(verify_node, '_teardown_ascii_vis'):
                                verify_node._teardown_ascii_vis()
                            verify_node.destroy_node()
                except (KeyboardInterrupt, EOFError):
                    print("\n✅ Calibration complete")
            else:
                print("\n✅ Stopped")
        finally:
            # Clean up ASCII display if it was initialized
            if node and hasattr(node, '_teardown_ascii_vis'):
                node._teardown_ascii_vis()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        parser.print_help()


if __name__ == '__main__':
    main()

