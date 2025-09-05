#!/usr/bin/env python3
# Combined ROS2 node: Battery/Water (MAX11612 ADC) + Temperature/Humidity (SHT45)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import sys
from rclpy.executors import ExternalShutdownException

try:
    import smbus2 as smbus2
    from smbus2 import i2c_msg
except Exception:
    smbus2 = None
    i2c_msg = None

# tqdm for terminal bars in debug mode
try:
    from tqdm import tqdm
    _HAS_TQDM = True
except Exception:
    _HAS_TQDM = False

# matplotlib for plotting RC-decay capture
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from datetime import datetime
    _HAS_MPL = True
except Exception:
    _HAS_MPL = False

class BatteryWaterNode(Node):
    def __init__(self):
        super().__init__('battery_water_node')
        self.get_logger().info('Initializing Battery/Water node...')

        # Debug flag
        self.debug = ('--debug' in sys.argv)
        # ADC electrical test mode (RC decay of REFOUT with sampling & PNG plot)
        self.test_adc = ('--test-adc' in sys.argv)
        self._test_state = 'decay'  # not used in one-shot capture
        self._test_t0 = time.monotonic()

        # Publishers
        self.pub_battery_voltage = self.create_publisher(Float32, 'battery_voltage', 10)
        self.pub_saltwater_voltage = self.create_publisher(Float32, 'saltwater_voltage', 10)
        self.pub_sail_current = self.create_publisher(Float32, 'sail_current', 10)
        self.pub_temperature = self.create_publisher(Float32, 'temperature', 10)
        self.pub_humidity = self.create_publisher(Float32, 'relative_humidity', 10)

        # I2C preferences
        self.use_smbus2 = (smbus2 is not None)
        if self.use_smbus2:
            self.get_logger().info('Using smbus2 for I2C transactions')
        else:
            self.get_logger().info('smbus2 not available; this node requires smbus2')

        # ADC (MAX11612)
        self.adc_addr = 0x34
        self.vref = 4.096
        self.lsb_value = self.vref / 4096.0

        # SHT45
        self.sht_addr = 0x44
        self.SHT45_HIGH_PRECISION_CMD = 0xFD
        self.SHT45_MEASUREMENT_DELAY = 0.01

        # Perform ADC setup (internal ref always on, AIN3 as analog input)
        try:
            setup_byte = self._build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(self.adc_addr, setup_byte)
            self.get_logger().info('ADC setup complete (internal ref always on).')
        except Exception as e:
            self.get_logger().error(f'ADC setup failed: {e}')

        # Debug bars (tqdm)
        self._bars = {}
        if self.debug and _HAS_TQDM:
            self._bars['bat'] = tqdm(total=100, desc='Battery (V)', position=0, leave=False, dynamic_ncols=True)
            self._bars['sw']  = tqdm(total=100, desc='Saltwater (V)', position=1, leave=False, dynamic_ncols=True)
            self._bars['cur'] = tqdm(total=100, desc='Sail Current (A)', position=2, leave=False, dynamic_ncols=True)
            self._bars['tmp'] = tqdm(total=100, desc='Temp (C)', position=3, leave=False, dynamic_ncols=True)
            self._bars['hum'] = tqdm(total=100, desc='Humidity (%)', position=4, leave=False, dynamic_ncols=True)

        # Timer: 1 Hz
        if self.test_adc:
            # Pause for user confirmation
            try:
                input('Connect a pulldown (e.g., scope probe 1 MΩ to GND) to AIN3, then press Enter to start the ADC RC-decay capture...')
            except Exception:
                pass
            # Perform a single capture: 50 ms REFOUT charge, 3 s decay with 10 ms sampling, then save PNG
            try:
                png_path = self._adc_rc_capture(duration_s=3.0, sample_dt=0.010)
                if png_path:
                    self.get_logger().info(f'RC-decay capture saved: {png_path}')
                else:
                    self.get_logger().warn('RC-decay capture completed but matplotlib is not available to save a plot.')
            except Exception as e:
                self.get_logger().error(f'RC-decay capture failed: {e}')
            # Keep node idle; no periodic publishing in test mode
            self.get_logger().info('Battery/Water node is idle after --test-adc capture. Press Ctrl+C to exit.')
        else:
            self.timer = self.create_timer(1.0, self.read_and_publish)
            self.get_logger().info('Battery/Water node initialized and reading at 1 Hz.')

    # ---------- I2C helpers ----------
    def _i2c_write(self, addr: int, byte_val: int) -> None:
        if self.use_smbus2 and i2c_msg is not None:
            msg = i2c_msg.write(addr, bytes([byte_val & 0xFF]))
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(msg)
        else:
            raise RuntimeError('smbus2 required for this node')

    def _i2c_read(self, addr: int, n: int) -> list:
        if self.use_smbus2 and i2c_msg is not None:
            r = i2c_msg.read(addr, n)
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(r)
            return list(r)
        else:
            raise RuntimeError('smbus2 required for this node')

    def _i2c_write_read(self, addr: int, write_byte: int, n: int) -> list:
        if self.use_smbus2 and i2c_msg is not None:
            w = i2c_msg.write(addr, bytes([write_byte & 0xFF]))
            r = i2c_msg.read(addr, n)
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(w, r)
            return list(r)
        else:
            raise RuntimeError('smbus2 required for this node')

    # ---------- MAX11612 helpers ----------
    def _build_setup(self, reg, sel, clk, bip_uni, rst, x):
        return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)

    def _build_config(self, reg, scan, cs, sgl_dif):
        return ((reg & 1) << 7) | ((scan & 0b11) << 5) | ((cs & 0b1111) << 1) | (sgl_dif & 1)

    def _read_adc_channel_avg(self, ch: int, repeats: int = 8, settle_delay_s: float = 0.001) -> int:
        # SCAN=01: convert selected input eight times
        cfg = self._build_config(reg=0, scan=0b01, cs=ch, sgl_dif=1)
        acc = 0
        try:
            self._i2c_write(self.adc_addr, cfg)
            time.sleep(settle_delay_s)
            for _ in range(repeats):
                d = self._i2c_read(self.adc_addr, 2)
                code = ((d[0] & 0x0F) << 8) | d[1]
                acc += code
            return acc // repeats
        except Exception as e:
            self.get_logger().error(f'ADC read ch{ch} failed: {e}')
            return 0

    # ---------- SHT45 helpers ----------
    def _sht_crc(self, data):
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
        return crc & 0xFF

    def _read_sht45(self):
        try:
            # pure write command
            w = i2c_msg.write(self.sht_addr, [self.SHT45_HIGH_PRECISION_CMD])
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(w)
            time.sleep(self.SHT45_MEASUREMENT_DELAY)
            # pure read of 6 bytes
            r = i2c_msg.read(self.sht_addr, 6)
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(r)
            data = list(r)
            temp_data = data[0:2]; temp_crc = data[2]
            humid_data = data[3:5]; humid_crc = data[5]
            if self._sht_crc(temp_data) != temp_crc or self._sht_crc(humid_data) != humid_crc:
                self.get_logger().warn('SHT45 CRC mismatch')
                return None, None
            raw_temp = (temp_data[0] << 8) | temp_data[1]
            raw_hum  = (humid_data[0] << 8) | humid_data[1]
            # Convert per datasheet
            temperature = -45.0 + 175.0 * (raw_temp / 65535.0)
            humidity = -6.0 + 125.0 * (raw_hum / 65535.0)
            humidity = max(0.0, min(100.0, humidity))
            return temperature, humidity
        except Exception as e:
            self.get_logger().error(f'SHT45 read failed: {e}')
            return None, None

    # ---------- Test: RC decay of REFOUT on AIN3 with sampling & PNG ----------
    def _set_ain3_mode(self, sel_bits: int):
        setup_byte = self._build_setup(reg=1, sel=sel_bits, clk=0, bip_uni=0, rst=1, x=0)
        self._i2c_write(self.adc_addr, setup_byte)

    def _adc_rc_capture(self, duration_s: float = 3.0, sample_dt: float = 0.010):
        # Charge AIN3 via REFOUT for 50 ms
        self._set_ain3_mode(sel_bits=0b111)  # REFOUT on AIN3
        time.sleep(0.05)
        # Switch AIN3 to analog input and sample decay
        self._set_ain3_mode(sel_bits=0b101)
        t0 = time.monotonic()
        times = []
        volts = []
        # Use minimal repeats to reduce loading; settle quickly
        while True:
            t = time.monotonic() - t0
            if t >= duration_s:
                break
            # One-shot sample on CH3
            try:
                raw = self._read_adc_channel_avg(3, repeats=1, settle_delay_s=0.0005)
            except Exception:
                raw = 0
            v = raw * self.lsb_value
            times.append(t)
            volts.append(v)
            # Sleep until next sample time
            rem = sample_dt - (time.monotonic() - t0 - t)
            if rem > 0:
                time.sleep(rem)
        # Save PNG if matplotlib is available
        if _HAS_MPL:
            plt.figure(figsize=(8, 4))
            plt.plot(times, volts, '-', linewidth=1.5)
            plt.title('AIN3 RC Decay (REFOUT -> Analog Input)')
            plt.xlabel('Time (s)')
            plt.ylabel('Voltage (V)')
            plt.grid(True, alpha=0.3)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = f'ain3_decay_{ts}.png'
            try:
                plt.savefig(path, dpi=120, bbox_inches='tight')
            finally:
                plt.close()
            return path
        return None

    def _scale_pct(self, value, vmin, vmax):
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, bat_v, sw_v, cur_a, temp_c, humid_pct):
        if not (self.debug and _HAS_TQDM and self._bars):
            return
        # Nominal ranges
        bat_max = 12.0
        sw_max  = 4.2
        cur_max = 4.2
        t_min, t_max = -20.0, 60.0
        h_min, h_max = 0.0, 100.0
        self._bars['bat'].n = self._scale_pct(bat_v, 0.0, bat_max)
        self._bars['bat'].set_postfix_str(f"{bat_v:.3f} V"); self._bars['bat'].refresh()
        self._bars['sw' ].n = self._scale_pct(sw_v, 0.0, sw_max)
        self._bars['sw' ].set_postfix_str(f"{sw_v:.3f} V"); self._bars['sw' ].refresh()
        self._bars['cur'].n = self._scale_pct(cur_a, 0.0, cur_max)
        self._bars['cur'].set_postfix_str(f"{cur_a:.3f} A"); self._bars['cur'].refresh()
        if temp_c is not None:
            self._bars['tmp'].n = self._scale_pct(temp_c, t_min, t_max)
            self._bars['tmp'].set_postfix_str(f"{temp_c:.2f} C"); self._bars['tmp'].refresh()
        if humid_pct is not None:
            self._bars['hum'].n = self._scale_pct(humid_pct, h_min, h_max)
            self._bars['hum'].set_postfix_str(f"{humid_pct:.2f} %"); self._bars['hum'].refresh()

    # ---------- Main read/publish ----------
    def read_and_publish(self):
        # ADC averages
        raw0 = self._read_adc_channel_avg(0)
        raw1 = self._read_adc_channel_avg(1)
        raw2 = self._read_adc_channel_avg(2)
        raw3 = self._read_adc_channel_avg(3)  # diagnostic only

        battery_voltage = raw0 * self.lsb_value * 2.0
        saltwater_voltage = raw1 * self.lsb_value
        sail_current = raw2 * self.lsb_value

        self.pub_battery_voltage.publish(Float32(data=battery_voltage))
        self.pub_saltwater_voltage.publish(Float32(data=saltwater_voltage))
        self.pub_sail_current.publish(Float32(data=sail_current))

        # SHT45
        temperature, humidity = self._read_sht45()
        if temperature is not None:
            self.pub_temperature.publish(Float32(data=temperature))
        if humidity is not None:
            self.pub_humidity.publish(Float32(data=humidity))

        # Update bars if enabled
        self._update_bars(battery_voltage, saltwater_voltage, sail_current, temperature, humidity)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryWaterNode()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            # Quiet shutdown to avoid traceback
            try:
                if hasattr(node, '_bars'):
                    for b in node._bars.values():
                        try:
                            b.close()
                        except Exception:
                            pass
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
        except ExternalShutdownException:
            try:
                if hasattr(node, '_bars'):
                    for b in node._bars.values():
                        try:
                            b.close()
                        except Exception:
                            pass
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
        else:
            # Normal shutdown path
            try:
                if hasattr(node, '_bars'):
                    for b in node._bars.values():
                        try:
                            b.close()
                        except Exception:
                            pass
            except Exception:
                pass
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
