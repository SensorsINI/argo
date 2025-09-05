#!/usr/bin/env python3
# ROS2 version of imu.py
# Simplified: reads raw accel, gyro, and compass data from ICM-20948 over I2C and publishes.

# topics
# publishes
#  NOT publishing /pose NOW SINCE WE ARE NOT DOING FUSION WITH RTIMULib /pose, Vector3 .z is compass heading in degrees corrected by magnetometer calibration
# /accel raw sensor values in g
# /gyro raw sensor values in deg/s
# /compass raw sensor values in uT

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import os.path
from  geometry_msgs.msg import Vector3
import struct
import time
import math
import argparse
import sys
import json
from datetime import datetime

try:
    from tqdm import tqdm
    _HAS_TQDM = True
except Exception:
    _HAS_TQDM = False

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
        self.select_bank(0x00)
        b = self.read_bytes(0x2D, 6)
        return (
            _to_int16(b[0], b[1]),
            _to_int16(b[2], b[3]),
            _to_int16(b[4], b[5]),
        )

    def read_gyro(self):
        self.select_bank(0x00)
        b = self.read_bytes(0x33, 6)
        return (
            _to_int16(b[0], b[1]),
            _to_int16(b[2], b[3]),
            _to_int16(b[4], b[5]),
        )


class ImuNode(Node):
    def __init__(self, debug=False):
        super().__init__('imu_node')
        self.debug = debug
        self.get_logger().info('Initializing IMU node...')

        if not self.debug:
            self.get_logger().info("Run with --debug to see sensor values being published.")

        # I2C setup
        self.i2c_bus_num = 0  # OrangePi uses bus 0 (confirmed by RTIMULib defaults)
        try:
            try:
                from smbus2 import SMBus  # type: ignore
            except Exception:
                from smbus import SMBus  # type: ignore
            self.bus = SMBus(self.i2c_bus_num)
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus {self.i2c_bus_num}: {e}")
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
            self.get_logger().error(f"IMU init failed: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Load compass calibration if available
        self._compass_cal = None
        try:
            with open('invensense-20948-compass-calibration.json', 'r') as f:
                self._compass_cal = json.load(f)
                self.get_logger().info('Loaded compass calibration from invensense-20948-compass-calibration.json')
        except Exception:
            pass

        # Publishers
        self.pub_accel = self.create_publisher(Vector3, 'accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, 'gyro', 10)
        self.pub_compass = self.create_publisher(Vector3, 'compass', 10)

        # Optional tqdm bars when in debug
        self._bars = {}
        if self.debug and _HAS_TQDM:
            # Create progress bars for accel, gyro, compass, and yaw
            self._bars['accel_x'] = tqdm(total=100, desc='Accel X (g)', position=0, leave=False, dynamic_ncols=True)
            self._bars['accel_y'] = tqdm(total=100, desc='Accel Y (g)', position=1, leave=False, dynamic_ncols=True)
            self._bars['accel_z'] = tqdm(total=100, desc='Accel Z (g)', position=2, leave=False, dynamic_ncols=True)
            self._bars['gyro_x']  = tqdm(total=100, desc='Gyro X (dps)', position=3, leave=False, dynamic_ncols=True)
            self._bars['gyro_y']  = tqdm(total=100, desc='Gyro Y (dps)', position=4, leave=False, dynamic_ncols=True)
            self._bars['gyro_z']  = tqdm(total=100, desc='Gyro Z (dps)', position=5, leave=False, dynamic_ncols=True)
            self._bars['mag_x']   = tqdm(total=100, desc='Mag X (uT)', position=6, leave=False, dynamic_ncols=True)
            self._bars['mag_y']   = tqdm(total=100, desc='Mag Y (uT)', position=7, leave=False, dynamic_ncols=True)
            self._bars['mag_z']   = tqdm(total=100, desc='Mag Z (uT)', position=8, leave=False, dynamic_ncols=True)
            # yaw removed (no fusion)
        elif self.debug and not _HAS_TQDM:
            self.get_logger().warn('tqdm not available; falling back to text debug output')

        # Main loop timer
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz

    def _scale_pct(self, value, vmin, vmax):
        # Clamp and scale to 0-100 percentage for bars
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, accel, gyro_dps, compass, yawDeg):
        if not self._bars:
            return
        # Define nominal ranges
        a_rng = 2.0  # +/- 2 g
        g_rng = 500.0  # +/- 500 deg/s
        m_rng = 100.0  # +/- 100 uT
        # Accel
        self._bars['accel_x'].n = self._scale_pct(accel[0], -a_rng, a_rng)
        self._bars['accel_y'].n = self._scale_pct(accel[1], -a_rng, a_rng)
        self._bars['accel_z'].n = self._scale_pct(accel[2], -a_rng, a_rng)
        self._bars['accel_x'].set_postfix_str(f"{accel[0]:+.2f} g"); self._bars['accel_x'].refresh()
        self._bars['accel_y'].set_postfix_str(f"{accel[1]:+.2f} g"); self._bars['accel_y'].refresh()
        self._bars['accel_z'].set_postfix_str(f"{accel[2]:+.2f} g"); self._bars['accel_z'].refresh()
        # Gyro
        self._bars['gyro_x'].n = self._scale_pct(gyro_dps[0], -g_rng, g_rng)
        self._bars['gyro_y'].n = self._scale_pct(gyro_dps[1], -g_rng, g_rng)
        self._bars['gyro_z'].n = self._scale_pct(gyro_dps[2], -g_rng, g_rng)
        self._bars['gyro_x'].set_postfix_str(f"{gyro_dps[0]:+.1f} dps"); self._bars['gyro_x'].refresh()
        self._bars['gyro_y'].set_postfix_str(f"{gyro_dps[1]:+.1f} dps"); self._bars['gyro_y'].refresh()
        self._bars['gyro_z'].set_postfix_str(f"{gyro_dps[2]:+.1f} dps"); self._bars['gyro_z'].refresh()
        # Compass
        self._bars['mag_x'].n = self._scale_pct(compass[0], -m_rng, m_rng)
        self._bars['mag_y'].n = self._scale_pct(compass[1], -m_rng, m_rng)
        self._bars['mag_z'].n = self._scale_pct(compass[2], -m_rng, m_rng)
        self._bars['mag_x'].set_postfix_str(f"{compass[0]:+.1f} uT"); self._bars['mag_x'].refresh()
        self._bars['mag_y'].set_postfix_str(f"{compass[1]:+.1f} uT"); self._bars['mag_y'].refresh()
        self._bars['mag_z'].set_postfix_str(f"{compass[2]:+.1f} uT"); self._bars['mag_z'].refresh()
        # Yaw bar may not be present in raw mode
        if 'yaw' in self._bars:
            self._bars['yaw'].n = self._scale_pct(yawDeg, -180.0, 180.0)
            self._bars['yaw'].set_postfix_str(f"{yawDeg:+.1f} deg"); self._bars['yaw'].refresh()

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

            # Read AK09916 via bypass (if DRDY) and convert to microtesla
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

            if self.debug and not _HAS_TQDM:
                self.get_logger().info(
                    f"accel(g)=({ax_g:+.3f},{ay_g:+.3f},{az_g:+.3f}), gyro(dps)=({gx_dps:+.1f},{gy_dps:+.1f},{gz_dps:+.1f}), mag(uT)=({mx_uT:+.1f},{my_uT:+.1f},{mz_uT:+.1f})"
                )

            if self.debug and _HAS_TQDM:
                # show normalized bars using nominal ranges
                self._update_bars(
                    (ax_g, ay_g, az_g),
                    (gx_dps, gy_dps, gz_dps),
                    (mx_uT, my_uT, mz_uT),
                    0.0,
                )

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

            # Publish in physical units
            self.pub_accel.publish(Vector3(x=ax_g, y=ay_g, z=az_g))
            self.pub_gyro.publish(Vector3(x=gx_dps, y=gy_dps, z=gz_dps))
            self.pub_compass.publish(Vector3(x=mx_uT, y=my_uT, z=mz_uT))
        except Exception as e:
            self.get_logger().error(f'IMU read failed: {e}')

    def _quiet_shutdown(self) -> None:
        # Close progress bars without logging to avoid rosout errors during shutdown
        if hasattr(self, '_bars'):
            for b in self._bars.values():
                try:
                    b.close()
                except Exception:
                    pass
        # Attempt to close I2C bus if supported
        try:
            if hasattr(self, 'bus') and hasattr(self.bus, 'close'):
                self.bus.close()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='IMU Sensor Node')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging for sensor values')
    parser.add_argument('--calib_compass', action='store_true', help='Collect magnetometer samples and save calibration')
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

            print("Rotate the device slowly through all orientations (figure-8). Press Ctrl-C to finish and save.")
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
                    lines.append(f"Mag {name} [{mn:+6.1f} .. {mx:+6.1f}] |{bar}| span={span:5.1f} uT")
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
                            if mx_uT < minx: minx = mx_uT
                            if my_uT < miny: miny = my_uT
                            if mz_uT < minz: minz = mz_uT
                            if mx_uT > maxx: maxx = mx_uT
                            if my_uT > maxy: maxy = my_uT
                            if mz_uT > maxz: maxz = mz_uT
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
                print(f"Warning: only {len(samples)} samples collected; calibration may be poor.")

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
            print(f"  ranges_uT   = X[{minx:+.1f}..{maxx:+.1f}] Y[{miny:+.1f}..{maxy:+.1f}] Z[{minz:+.1f}..{maxz:+.1f}] (n={len(samples)})")

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
                print("Saved compass calibration to invensense-20948-compass-calibration.json")
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
