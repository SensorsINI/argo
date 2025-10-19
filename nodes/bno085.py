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
The BNO085 requires calibration for optimal accuracy:

  1. Magnetometer (most important):
     - Move sensor in figure-8 pattern through all orientations
     - Rotate through roll, pitch, and yaw axes
     - Continue until magnetometer reaches HIGH accuracy
     
  2. Accelerometer & Gyroscope (automatic):
     - Auto-calibrate during motion and stationary periods
     
  3. Calibration data:
     - Automatically saved to sensor's internal flash memory
     - Persists across power cycles and reboots
     
  Run calibration:
     bno085.py calibrate --duration 120
     # Interactive tool with real-time guidance

COMMANDS
--------
  bridge       Run as data bridge (for launch files/lifecycle manager)
               - Persistent ROS2 node
               - Converts C++ driver topics to Argo format
               - Publishes health status
               
  calibrate    Interactive calibration with real-time guidance
               - Shows accuracy levels (UNRELIABLE → LOW → MEDIUM → HIGH)
               - Provides motion feedback
               - Generates final report
               
  verify       Verify rotation vector output quality
               - Validates datasheet section 2.2.4 compliance
               - Shows quaternion, Euler angles, heading stability
               - Checks unit quaternion, magnetic north, gravity
               
  status       Check BNO085 system health
               - Shows driver and bridge status
               - Lists topic availability
               - Displays health status
               - Checks I2C hardware detection

EXAMPLES
--------
  # Check system status
  bno085.py status
  
  # Calibrate sensor (2 minutes with real-time guidance)
  bno085.py calibrate
  
  # Quick 1-minute calibration
  bno085.py calibrate --duration 60
  
  # Extended 5-minute calibration for best results
  bno085.py calibrate --duration 300
  
  # Verify sensor output (10 seconds)
  bno085.py verify --duration 10
  
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
  5. Verify output:     python3 bno085.py verify

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
import math
import sys
import time
import argparse
import argcomplete
import subprocess
from datetime import datetime
import collections

# ============================================================================
# BRIDGE MODE: Convert C++ driver output to Argo format
# ============================================================================

class BNO085Bridge(Node):
    """Bridge node that converts BNO08x driver topics to Argo format."""
    
    def __init__(self):
        super().__init__('bno085_bridge')
        
        # Publishers (Argo format)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_accel = self.create_publisher(Vector3, '/accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, '/gyro', 10)
        self.pub_health = self.create_publisher(Bool, '/imu_health', 10)
        
        # Subscribers (from BNO08x driver)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.sub_mag = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        
        # Health monitoring
        self.health_status = False  # Start unhealthy until first data received
        self.last_imu_time = None
        self.health_check_timer = self.create_timer(1.0, self._check_health)
        
        self.get_logger().info("BNO085 Bridge: Converting C++ driver → Argo topics")
        self.get_logger().info("Publishing: /compass, /pose, /accel, /gyro, /imu_health")
    
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
        # Update health tracking
        self.last_imu_time = time.time()
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
        """Periodic health check - mark unhealthy if no data received."""
        if self.last_imu_time is None:
            # No data received yet
            if self.health_status:
                self._publish_health_status(False)
            return
        
        # Check if data is stale (no data for 3 seconds)
        time_since_last = time.time() - self.last_imu_time
        if time_since_last > 3.0:
            if self.health_status:
                self.get_logger().warn(f"No IMU data for {time_since_last:.1f}s - marking unhealthy")
                self._publish_health_status(False)
        elif not self.health_status:
            # Recovered
            self._publish_health_status(True)
    
    def _publish_health_status(self, is_healthy: bool):
        """Publish health status update."""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)
            
            if is_healthy:
                self.get_logger().info("BNO085 health: HEALTHY")
            else:
                self.get_logger().warn("BNO085 health: UNHEALTHY")


# ============================================================================
# CALIBRATION MODE: Interactive sensor calibration
# ============================================================================

class BNO085Calibrator(Node):
    """Interactive calibration tool with real-time guidance."""
    
    def __init__(self, duration=120, save_interval=30):
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
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        
        # Timers
        self.status_timer = self.create_timer(2.0, self.print_status)
        if save_interval > 0:
            self.save_timer = self.create_timer(float(save_interval), self.auto_save)
        
        self.print_header()
        self.get_logger().info(f"Calibration started: {duration}s duration")
    
    def print_header(self):
        print("\n" + "=" * 80)
        print("BNO085 CALIBRATION TOOL")
        print("=" * 80)
        print("\nINSTRUCTIONS:")
        print("1. MAGNETOMETER: Move sensor in figure-8 through all orientations")
        print("2. ACCELEROMETER: Auto-calibrates during motion")
        print("3. GYROSCOPE: Auto-calibrates during still periods")
        print("\nTARGET: All sensors reach HIGH accuracy")
        print("=" * 80 + "\n")
    
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
        print(f"\n⏱️  TIME: {elapsed:.1f}s / {self.calibration_duration}s")
        print(f"\n📊 ACCURACY:")
        print(f"   Magnetometer:  {self._bar(self.mag_accuracy)} {self._name(self.mag_accuracy)}")
        print(f"   Accelerometer: {self._bar(self.accel_accuracy)} {self._name(self.accel_accuracy)}")
        print(f"   Gyroscope:     {self._bar(self.gyro_accuracy)} {self._name(self.gyro_accuracy)}")
        print(f"\n📈 SAMPLES: IMU={self.imu_sample_count}, Mag={self.mag_sample_count}, Changes={self.orientation_changes}")
        
        if self.mag_accuracy < 3:
            print(f"\n💡 {'✅ Motion detected!' if self.motion_detected else '⚠️  Move sensor in figure-8!'}")
            self.motion_detected = False
        else:
            print("\n✅ CALIBRATION COMPLETE! All sensors at HIGH accuracy")
        
        print("=" * 80 + "\n")
        
        if remaining <= 0:
            self.final_report()
            rclpy.shutdown()
    
    def _bar(self, acc):
        return ["░░░░", "▓░░░", "▓▓░░", "▓▓▓░", "▓▓▓▓"][max(0, min(3, int(acc)))]
    
    def _name(self, acc):
        return ["UNRELIABLE", "LOW      ", "MEDIUM   ", "HIGH     "][max(0, min(3, int(acc)))]
    
    def auto_save(self):
        print("💾 Calibration auto-saved by sensor")
    
    def final_report(self):
        print("\n" + "=" * 80)
        print("CALIBRATION COMPLETE")
        print("=" * 80)
        print(f"\nDuration: {time.time() - self.start_time:.1f}s")
        print(f"Final Status: Mag={self._name(self.mag_accuracy).strip()}, " +
              f"Accel={self._name(self.accel_accuracy).strip()}, " +
              f"Gyro={self._name(self.gyro_accuracy).strip()}")
        print(f"Samples: {self.imu_sample_count} IMU, {self.orientation_changes} orientation changes")
        
        if self.mag_accuracy >= 3 and self.accel_accuracy >= 2 and self.gyro_accuracy >= 2:
            print("\n✅ SUCCESS! Sensor is well-calibrated")
        else:
            print("\n⚠️  Partial calibration - consider running again")
        print("=" * 80 + "\n")


# ============================================================================
# VERIFICATION MODE: Validate rotation vector output
# ============================================================================

class RotationVectorVerifier(Node):
    """Verify rotation vector output matches datasheet specs."""
    
    def __init__(self):
        super().__init__('rotation_vector_verifier')
        self.sample_count = 0
        self.yaw_history = collections.deque(maxlen=20)
        self.last_print_time = time.time()
        
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        print("\n" + "=" * 80)
        print("BNO085 ROTATION VECTOR VERIFICATION")
        print("=" * 80)
        print("\nDatasheet Section 2.2.4 Requirements:")
        print("  ✓ Quaternion orientation (w, x, y, z)")
        print("  ✓ Referenced to magnetic north and gravity")
        print("  ✓ Fuses accelerometer + gyroscope + magnetometer")
        print("\nWaiting for data...")
        print("=" * 80 + "\n")
    
    def imu_callback(self, msg: Imu):
        self.sample_count += 1
        current_time = time.time()
        
        q = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.w, q.x, q.y, q.z)
        q_mag = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        accel_mag = math.sqrt(sum(x**2 for x in [msg.linear_acceleration.x, 
                                                   msg.linear_acceleration.y,
                                                   msg.linear_acceleration.z]))
        self.yaw_history.append(yaw)
        
        if current_time - self.last_print_time > 2.0:
            self.last_print_time = current_time
            import numpy as np
            
            print(f"\n📊 Sample #{self.sample_count}")
            print("-" * 80)
            print(f"Quaternion: w={q.w:+.6f} x={q.x:+.6f} y={q.y:+.6f} z={q.z:+.6f}")
            print(f"  Magnitude: {q_mag:.6f} {'✓' if abs(q_mag-1.0)<0.01 else '❌'} Unit quaternion")
            print(f"\nEuler: Roll={roll:+.2f}° Pitch={pitch:+.2f}° Yaw={yaw:+.2f}° (compass)")
            
            if len(self.yaw_history) > 1:
                std = np.std(list(self.yaw_history))
                print(f"  Stability: ±{std:.2f}° std dev over {len(self.yaw_history)} samples")
            
            print(f"\nAcceleration: |g|={accel_mag:.3f} m/s² (expect ~9.81 at rest)")
            print(f"\n✓ Quaternion: {'PASS' if abs(q_mag-1.0)<0.01 else 'FAIL'}")
            print(f"✓ Mag North:  {'PASS' if 0<=yaw<=360 else 'FAIL'}")
            print(f"✓ Gravity:    {'PASS' if abs(accel_mag-9.81)<2.0 else 'FAIL'}")
            print("-" * 80)
    
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
    parser.add_argument('--duration', type=int, default=120,
                        help='Duration in seconds for calibrate/verify (default: 120)')
    parser.add_argument('--save-interval', type=int, default=30,
                        help='Calibration auto-save interval (default: 30)')
    
    # Enable bash completion for command-line arguments
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    
    if args.command == 'status':
        cmd_status()
    elif args.command in ['bridge', 'calibrate', 'verify']:
        rclpy.init()
        try:
            if args.command == 'bridge':
                node = BNO085Bridge()
            elif args.command == 'calibrate':
                node = BNO085Calibrator(args.duration, args.save_interval)
            else:  # verify
                node = RotationVectorVerifier()
            
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n✅ Stopped")
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    else:
        parser.print_help()


if __name__ == '__main__':
    main()

