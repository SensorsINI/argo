#!/usr/bin/env python3
"""
IMU Sensor Node for Argo Autonomous Sailboat
============================================

This ROS2 node interfaces with the ICM-20948 9-axis IMU sensor (accelerometer, 
gyroscope, magnetometer) via I2C and publishes raw sensor data to ROS2 topics.

Hardware:
- SparkfunICM-20948 9-axis IMU sensor https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/#documentation https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html
- I2C bus 0, address 0x69
- AK09916 magnetometer (integrated in ICM-20948)

Axes/Coordinate Frame: (see https://cdn.sparkfun.com/assets/learn_tutorials/8/9/3/DS-000189-ICM-20948-v1.3.pdf section 15, figs 12-13)
For the ICM-20948, the coordinate frame is defined as follows for the accelerometer:
- +x: rightwards, starboard
- y: forwards, towards bow
- +z: up along mast, approx magnetic north

For the magnetometer, the coordinate frame is defined as follows:
- +x: rightwards, starboard
- +y: backwards, towards stern
- z: down into water
For the gyroscope, the coordinate frame is defined as follows:

Published Topics:
- /accel (geometry_msgs/Vector3): Raw accelerometer data in g (gravity units)
- /gyro (geometry_msgs/Vector3): Raw gyroscope data in deg/s (degrees per second)  
- /magnetometer (geometry_msgs/Vector3): Raw magnetometer data in uT (microtesla)
- /compass (std_msgs/Float64): Compass heading in degrees (0-360, 0=North, 90=East)
- /imu_health (std_msgs/Bool): Node health status (true=healthy, false=failed)

Note: This node publishes raw sensor values only. No sensor fusion or pose 
estimation is performed (unlike RTIMULib-based implementations).

Command Line Options:
--debug              Enable debug logging to show sensor values being published
--calib_compass      Run magnetometer calibration mode (interactive)
                     - Rotate device through all orientations
                     - Press Ctrl+C to finish and save calibration
                     - Saves calibration to invensense-20948-compass-calibration.json

Usage Examples:
  python3 imu.py                    # Normal operation
  python3 imu.py --debug            # With debug output
  python3 imu.py --calib_compass    # Calibrate magnetometer
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import os.path
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64
import struct
import time
import math
import argparse
import sys
import json
from datetime import datetime


def _to_int16(msb, lsb):
    return struct.unpack('>h', bytes([msb, lsb]))[0]


class ICM20948:
    def __init__(self, bus, address=0x69):
        self.bus = bus
        self.addr = address
        self._reg_bank = None

    # Low-level I2C helpers
    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read_bytes(self, reg, length):
        return list(self.bus.read_i2c_block_data(self.addr, reg, length))

    def select_bank(self, bank):
        if self._reg_bank != bank:
            self.write_byte(0x7F, bank)
            self._reg_bank = bank

    # Minimal bring-up
    def initialize(self):
        # Reset, then wake
        self.select_bank(0x00)
        self.write_byte(0x06, 0x80)  # PWR_MGMT_1: DEVICE_RESET
        time.sleep(0.1)
        self.write_byte(0x06, 0x01)  # auto clock, sleep off
        time.sleep(0.05)

        # Configure gyro/accel (bank 2): FSR and LPF
        self.select_bank(0x20)
        # Gyro: FSR=250 dps (00), LPF index ~4, enable LPF
        gyro_cfg = ((4 & 0x07) << 3) | (0 << 1) | 1
        self.write_byte(0x01, gyro_cfg)  # GYRO_CONFIG_1
        self.write_byte(0x02, 0x00)      # GYRO_CONFIG_2
        # Accel: FSR=2g (00), LPF index ~4, enable LPF
        accel_cfg = ((4 & 0x07) << 3) | (0 << 1) | 1
        self.write_byte(0x14, accel_cfg)  # ACCEL_CONFIG
        self.write_byte(0x15, 0x00)       # ACCEL_CONFIG_2

        # Enable bypass so AK09916 is accessible at 0x0C on the same bus
        self.select_bank(0x00)
        user_ctrl = self.bus.read_byte_data(self.addr, 0x03)
        user_ctrl &= ~0x20  # clear I2C_MST_EN
        self.write_byte(0x03, user_ctrl)
        time.sleep(0.01)
        self.write_byte(0x0F, 0x82)  # INT_PIN_CFG: BYPASS_EN
        time.sleep(0.05)

    # Raw reads
    def read_accel(self):
        """
        Read accelerometer data from ICM-20948.
        
        Returns:
            tuple: (ax_cnt, ay_cnt, az_cnt) raw accelerometer counts
        """
        self.select_bank(0x00)
        b = self.read_bytes(0x2D, 6)
        return (
            _to_int16(b[0], b[1]),
            _to_int16(b[2], b[3]),
            _to_int16(b[4], b[5]),
        )

    def read_gyro(self):
        """
        Read gyroscope data from ICM-20948.
        
        Returns:
            tuple: (gx_cnt, gy_cnt, gz_cnt) raw gyroscope counts
        """
        self.select_bank(0x00)
        b = self.read_bytes(0x33, 6)
        return (
            _to_int16(b[0], b[1]),
            _to_int16(b[2], b[3]),
            _to_int16(b[4], b[5]),
        )

    def read_magnetometer(self):
        """
        Read magnetometer data from AK09916 via I2C bypass.
        
        Returns:
            tuple: (mx_uT, my_uT, mz_uT) magnetometer readings in microtesla
        """
        mx_uT = my_uT = mz_uT = 0.0
        try:
            st1 = self.bus.read_byte_data(0x0C, 0x10)
            if st1 & 0x01:
                mb = list(self.bus.read_i2c_block_data(0x0C, 0x11, 8))
                # little-endian raw counts
                mx_cnt = struct.unpack('<h', bytes(mb[0:2]))[0]
                my_cnt = struct.unpack('<h', bytes(mb[2:4]))[0]
                mz_cnt = struct.unpack('<h', bytes(mb[4:6]))[0]
                # AK09916 sensitivity ~0.15 uT/LSB
                mx_uT = mx_cnt * 0.15
                my_uT = my_cnt * 0.15
                mz_uT = mz_cnt * 0.15
        except Exception:
            pass
        
        return mx_uT, my_uT, mz_uT


class ImuNode(Node):
    def __init__(self, debug=False):
        super().__init__('imu_node')
        self.debug = debug
        self.get_logger().info('Initializing IMU node...')

        if not self.debug:
            self.get_logger().info("Run with --debug to see sensor values being published.")

        # I2C setup
        # OrangePi uses bus 0 (confirmed by RTIMULib defaults)
        self.i2c_bus_num = 0
        try:
            try:
                from smbus2 import SMBus  # type: ignore
            except Exception:
                from smbus import SMBus  # type: ignore
            self.bus = SMBus(self.i2c_bus_num)
        except Exception as e:
            self.get_logger().error(
                f"Failed to open I2C bus {self.i2c_bus_num}: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # IMU device
        self.icm_addr = 0x69
        self.icm = ICM20948(self.bus, self.icm_addr)

        try:
            self.icm.initialize()
            # AK09916 setup via bypass
            AK_ADDR = 0x0C
            # soft reset
            self.bus.write_byte_data(AK_ADDR, 0x32, 0x01)
            time.sleep(0.05)
            # continuous measurement 100Hz
            self.bus.write_byte_data(AK_ADDR, 0x31, 0x08)
            time.sleep(0.01)
            self.get_logger().info("ICM-20948 init complete (raw mode)")
        except Exception as e:
            self.get_logger().fatal(f"FATAL: IMU init failed: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Load compass calibration if available
        self._compass_cal = None
        try:
            with open('invensense-20948-compass-calibration.json', 'r') as f:
                self._compass_cal = json.load(f)
                self.get_logger().info(
                    'Loaded compass calibration from invensense-20948-compass-calibration.json')
        except Exception:
            pass

        # Publishers
        self.pub_accel = self.create_publisher(Vector3, 'accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, 'gyro', 10)
        self.pub_magnetometer = self.create_publisher(Vector3, 'magnetometer', 10)
        self.pub_compass = self.create_publisher(Float64, 'compass', 10)

        # Health status publisher
        self.pub_health = self.create_publisher(Bool, 'imu_health', 10)
        self.health_status = False  # Track current health status

        # ASCII visual debug
        self._vis_ascii = self.debug
        self._vis_initialized = False
        if self._vis_ascii:
            self._init_ascii_vis()

        # Main loop timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Publish initial health status as healthy
        self._publish_health_status(True)

    def _publish_health_status(self, is_healthy: bool):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            if is_healthy:
                self.get_logger().info("IMU health status: HEALTHY")
            else:
                self.get_logger().warn("IMU health status: FAILED")

    def _init_ascii_vis(self):
        if self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[?25l')  # hide cursor
            sys.stdout.write('\x1b[2J')    # clear
            sys.stdout.write('\x1b[H')     # home
            sys.stdout.flush()
            self._vis_initialized = True
        except Exception:
            self._vis_initialized = False

    def _teardown_ascii_vis(self):
        if not self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')
            sys.stdout.write('\x1b[2J')
            sys.stdout.write('\x1b[H')
            sys.stdout.write('\x1b[?25h')
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False

    def _signed_bar(self, value: float, limit: float, width: int = 50) -> str:
        # Centered at zero; '-' for negative, '+' for positive; '|' marks zero
        width = max(10, width)
        mid = width // 2
        # clip
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
        # build
        return '[' + ''.join(left) + '|' + ''.join(right) + ']'

    def timer_callback(self):
        try:
            ax_cnt, ay_cnt, az_cnt = self.icm.read_accel()
            gx_cnt, gy_cnt, gz_cnt = self.icm.read_gyro()

            # Convert to physical units
            ax_g = ax_cnt / 16384.0
            ay_g = ay_cnt / 16384.0
            az_g = az_cnt / 16384.0

            gx_dps = gx_cnt / 131.072
            gy_dps = gy_cnt / 131.072
            gz_dps = gz_cnt / 131.072

            # Read magnetometer data
            mx_uT, my_uT, mz_uT = self.icm.read_magnetometer()

            if self._vis_ascii:
                try:
                    # Compute heading for debug display (same as main calculation)
                    # Magnetometer coordinate frame: x=starboard, y=stern, z=down
                    heading_rad = math.atan2(-my_uT, mx_uT)  # -y points to bow, x points to starboard
                    heading_deg = math.degrees(heading_rad) - 90.0  # Adjust so 0° = North
                    if heading_deg < 0:
                        heading_deg += 360.0
                    
                    sys.stdout.write('\x1b[H')  # home
                    # Nominal limits for bars
                    a_lim = 2.0   # g
                    g_lim = 500.0  # dps
                    m_lim = 100.0  # uT
                    lines = [
                        f"Ax {ax_g:+7.3f} g   " + self._signed_bar(ax_g, a_lim),
                        f"Ay {ay_g:+7.3f} g   " + self._signed_bar(ay_g, a_lim),
                        f"Az {az_g:+7.3f} g   " + self._signed_bar(az_g, a_lim),
                        f"Gx {gx_dps:+7.1f} dps " +
                        self._signed_bar(gx_dps, g_lim),
                        f"Gy {gy_dps:+7.1f} dps " +
                        self._signed_bar(gy_dps, g_lim),
                        f"Gz {gz_dps:+7.1f} dps " +
                        self._signed_bar(gz_dps, g_lim),
                        f"Mx {mx_uT:+7.1f} uT  " +
                        self._signed_bar(mx_uT, m_lim),
                        f"My {my_uT:+7.1f} uT  " +
                        self._signed_bar(my_uT, m_lim),
                        f"Mz {mz_uT:+7.1f} uT  " +
                        self._signed_bar(mz_uT, m_lim),
                        f"Hd {heading_deg:+7.1f}°    " +
                        self._signed_bar(heading_deg - 180, 180),  # Center on 180°
                        "Ctrl-C to exit"
                    ]
                    for ln in lines:
                        sys.stdout.write(ln + '\n')
                    sys.stdout.flush()
                except Exception:
                    pass

            # Apply compass calibration if present (min-max/diagonal soft-iron)
            if isinstance(self._compass_cal, dict) and self._compass_cal.get('method') in ('minmax', 'diag'):
                try:
                    bx, by, bz = self._compass_cal['bias_uT']
                    sx, sy, sz = self._compass_cal['scale_diag']
                    mx_uT = (mx_uT - bx) * sx
                    my_uT = (my_uT - by) * sy
                    mz_uT = (mz_uT - bz) * sz
                except Exception:
                    pass

            # Compute compass heading from magnetometer data
            # Magnetometer coordinate frame: x=starboard, y=stern (backward), z=down
            # To get heading: use x and -y (since y points to stern, -y points to bow)
            # atan2(-y, x) gives us the angle from the x-axis (starboard) to the bow direction
            # We want angle from North (0°), so we need to adjust by 90° (North is -90° from starboard)
            heading_rad = math.atan2(-my_uT, mx_uT)  # -y points to bow, x points to starboard
            heading_deg = math.degrees(heading_rad) - 90.0  # Adjust so 0° = North
            if heading_deg < 0:
                heading_deg += 360.0
            
            # Publish in physical units
            self.pub_accel.publish(Vector3(x=ax_g, y=ay_g, z=az_g))
            self.pub_gyro.publish(Vector3(x=gx_dps, y=gy_dps, z=gz_dps))
            self.pub_magnetometer.publish(Vector3(x=mx_uT, y=my_uT, z=mz_uT))
            self.pub_compass.publish(Float64(data=heading_deg))
        except Exception as e:
            self.get_logger().error(f'IMU read failed: {e}')
            self._publish_health_status(False)

    def _quiet_shutdown(self) -> None:
        # Publish health status as failed on shutdown
        self._publish_health_status(False)

        # Teardown ASCII view
        self._teardown_ascii_vis()
        # Attempt to close I2C bus if supported
        try:
            if hasattr(self, 'bus') and hasattr(self.bus, 'close'):
                self.bus.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='IMU Sensor Node')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging for sensor values')
    parser.add_argument('--calib_compass', action='store_true',
                        help='Collect magnetometer samples and save calibration')
    parsed_args = parser.parse_args(args=args)

    # If only calibration is requested, run a standalone collector
    if parsed_args.calib_compass:
        try:
            try:
                from smbus2 import SMBus  # type: ignore
            except Exception:
                from smbus import SMBus  # type: ignore
            bus = SMBus(0)
            icm = ICM20948(bus, 0x69)
            icm.initialize()
            AK_ADDR = 0x0C
            bus.write_byte_data(AK_ADDR, 0x32, 0x01)
            time.sleep(0.05)
            bus.write_byte_data(AK_ADDR, 0x31, 0x08)
            time.sleep(0.01)

            print(
                "Rotate the device slowly through all orientations (figure-8). Press Ctrl-C to finish and save.")
            samples = []
            # Tracking minima and maxima
            minx = miny = minz = float('inf')
            maxx = maxy = maxz = float('-inf')
            bar_width = 50
            # Terminal setup
            try:
                sys.stdout.write('\x1b[?25l')  # hide cursor
                sys.stdout.write('\x1b[2J')    # clear
                sys.stdout.flush()
                visual_ok = True
            except Exception:
                visual_ok = False

            def render_bars():
                # Use 100 uT full-scale for visualization
                fs = 100.0
                spans = [maxx - minx if minx != float('inf') else 0.0,
                         maxy - miny if miny != float('inf') else 0.0,
                         maxz - minz if minz != float('inf') else 0.0]
                lines = []
                for name, mn, mx, span in [("X", minx, maxx, spans[0]), ("Y", miny, maxy, spans[1]), ("Z", minz, maxz, spans[2])]:
                    frac = max(0.0, min(1.0, span / fs))
                    fill = int(round(frac * bar_width))
                    bar = '#' * fill + '-' * (bar_width - fill)
                    lines.append(
                        f"Mag {name} [{mn:+6.1f} .. {mx:+6.1f}] |{bar}| span={span:5.1f} uT")
                try:
                    sys.stdout.write('\x1b[H')  # home
                    for ln in [f"Samples: {len(samples)}", *lines, "Ctrl-C to finish..."]:
                        sys.stdout.write(ln + '\n')
                    sys.stdout.flush()
                except Exception:
                    pass

            try:
                while True:
                    try:
                        st1 = bus.read_byte_data(0x0C, 0x10)
                        if st1 & 0x01:
                            mb = list(bus.read_i2c_block_data(0x0C, 0x11, 8))
                            mx_cnt = struct.unpack('<h', bytes(mb[0:2]))[0]
                            my_cnt = struct.unpack('<h', bytes(mb[2:4]))[0]
                            mz_cnt = struct.unpack('<h', bytes(mb[4:6]))[0]
                            mx_uT = mx_cnt * 0.15
                            my_uT = my_cnt * 0.15
                            mz_uT = mz_cnt * 0.15
                            samples.append((mx_uT, my_uT, mz_uT))
                            if mx_uT < minx:
                                minx = mx_uT
                            if my_uT < miny:
                                miny = my_uT
                            if mz_uT < minz:
                                minz = mz_uT
                            if mx_uT > maxx:
                                maxx = mx_uT
                            if my_uT > maxy:
                                maxy = my_uT
                            if mz_uT > maxz:
                                maxz = mz_uT
                            if visual_ok:
                                render_bars()
                    except Exception:
                        pass
                    time.sleep(0.02)
            except KeyboardInterrupt:
                pass
            finally:
                if visual_ok:
                    try:
                        sys.stdout.write('\x1b[0m')
                        sys.stdout.write('\x1b[2J')
                        sys.stdout.write('\x1b[H')
                        sys.stdout.write('\x1b[?25h')
                        sys.stdout.flush()
                    except Exception:
                        pass

            if len(samples) < 100:
                print(
                    f"Warning: only {len(samples)} samples collected; calibration may be poor.")

            # Min-max calibration (diagonal soft-iron approximation)
            xs = [s[0] for s in samples]
            ys = [s[1] for s in samples]
            zs = [s[2] for s in samples]
            minx, maxx = min(xs), max(xs)
            miny, maxy = min(ys), max(ys)
            minz, maxz = min(zs), max(zs)
            bx = (maxx + minx) / 2.0
            by = (maxy + miny) / 2.0
            bz = (maxz + minz) / 2.0
            rx = (maxx - minx) / 2.0
            ry = (maxy - miny) / 2.0
            rz = (maxz - minz) / 2.0
            r_avg = (rx + ry + rz) / 3.0 if (rx + ry + rz) > 0 else 1.0
            sx = r_avg / rx if rx != 0 else 1.0
            sy = r_avg / ry if ry != 0 else 1.0
            sz = r_avg / rz if rz != 0 else 1.0

            calib = {
                'method': 'minmax',
                'bias_uT': [bx, by, bz],
                'scale_diag': [sx, sy, sz],
                'num_samples': len(samples),
                'ranges_uT': {'x': [minx, maxx], 'y': [miny, maxy], 'z': [minz, maxz]},
                'timestamp': datetime.utcnow().isoformat() + 'Z'
            }
            # Show summary and confirm save
            print("\nProposed compass calibration (min-max diag fit):")
            print(f"  bias_uT     = [{bx:+.2f}, {by:+.2f}, {bz:+.2f}]")
            print(f"  scale_diag  = [{sx:.4f}, {sy:.4f}, {sz:.4f}]")
            print(
                f"  ranges_uT   = X[{minx:+.1f}..{maxx:+.1f}] Y[{miny:+.1f}..{maxy:+.1f}] Z[{minz:+.1f}..{maxz:+.1f}] (n={len(samples)})")

            def _prompt_yes_no(message: str, default_yes: bool = True) -> bool:
                opts = "Y/n" if default_yes else "y/N"
                try:
                    resp = input(f"{message} [{opts}]: ").strip().lower()
                except Exception:
                    resp = ''
                if resp == '':
                    return default_yes
                if resp in ('y', 'yes'):
                    return True
                if resp in ('n', 'no'):
                    return False
                return default_yes

            if _prompt_yes_no("Save calibration to invensense-20948-compass-calibration.json?", default_yes=True):
                with open('invensense-20948-compass-calibration.json', 'w') as f:
                    json.dump(calib, f, indent=2)
                print(
                    "Saved compass calibration to invensense-20948-compass-calibration.json")
            else:
                print("Calibration discarded; file not saved.")
        except Exception as e:
            print(f"Calibration failed: {e}")
        finally:
            try:
                if 'bus' in locals() and hasattr(bus, 'close'):
                    bus.close()
            except Exception:
                pass
        return

    imu_node = ImuNode(debug=parsed_args.debug)

    if rclpy.ok():
        try:
            rclpy.spin(imu_node)
        except KeyboardInterrupt:
            # Quiet shutdown to avoid traceback on Ctrl+C
            imu_node._quiet_shutdown()
            try:
                imu_node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)
        except ExternalShutdownException:
            imu_node._quiet_shutdown()
            try:
                imu_node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)


if __name__ == '__main__':
    main()
