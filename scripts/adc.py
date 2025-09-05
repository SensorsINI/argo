#!/usr/bin/env python
# - adapted from anem.py to read values over i2c from the MAX11612 ADC chip. The i2c address is 0x34.
# - the reference voltage is 4.096 volts, so each DN is 1mV for this 12-bit converter.
# - reads battery_voltage from AIN0 over a voltage divider with R1=R2.
# - reads saltwater voltage from AIN1 with 1MOhm resistor load.
# - reads sail winch current from AIN2 with a 1Ohm resistor shunt.
# - uses AIN3 only for testing the i2c communication.
# - The read frequency is 1 Hz.
# - sensor values are published to ros topics as float values with units of either voltage or current.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
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

try:
    import matplotlib.pyplot as plt
    plotting_enabled = True
except ImportError:
    plt = None
    plotting_enabled = False

class AdcNode(Node):
    def __init__(self):
        super().__init__('adc_node')

        # Set logger level to DEBUG if --debug flag is passed
        # Debug bars (tqdm) will be used instead of ROS debug logs
        # if '--debug' in sys.argv:
        #     self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Prefer smbus2 automatically if available
        self.use_smbus2 = (smbus2 is not None)
        if self.use_smbus2:
            self.get_logger().debug('Using smbus2 for I2C transactions')
        else:
            self.get_logger().debug('Using smbus (fallback) for I2C transactions')

        # Optional reference toggle test (oscilloscope aid)
        self.test_ref = ('--test_ref' in sys.argv)
        self._ref_is_output = True
        # Optional configuration toggle test (oscilloscope aid)
        self.test_cfg = ('--test_cfg' in sys.argv)
        self._cfg_includes_ain3 = False

        self.i2c_test_mode = ('--i2c_test' in sys.argv)

        self.get_logger().info('Initializing ADC node...')

        # Publishers
        self.pub_saltwater_voltage = self.create_publisher(Float32, 'saltwater_voltage', 10)
        self.pub_sail_current = self.create_publisher(Float32, 'sail_current', 10)
        self.pub_battery_voltage = self.create_publisher(Float32, 'battery_voltage', 10)

        # I2C setup
        self.i2c_addr = 0x34  # MAX11612 I2C address
        self.bus = None
        self.vref = 4.096 # Reference voltage in Volts
        self.lsb_value = self.vref / 4096.0 # Value of one LSB in Volts (12-bit ADC)

        try:
            self.bus = smbus.SMBus(0) # The default i2c bus
            self.get_logger().info('Opened i2c SMBus for ADC')

            if self.i2c_test_mode:
                self.i2c_test()
                return

            # Build setup byte from bitfields for inspection:
            # Setup byte format: REG(1b)<<7 | SEL[2:0]<<4 | CLK(1b)<<3 | BIP/UNI(1b)<<2 | RST(1b)<<1 | X(1b)
            def build_setup(reg, sel, clk, bip_uni, rst, x):
                return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)

            # Hard-reset configuration register, then enable internal reference always-on (AIN3 as input)
            setup_reset = build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(setup_reset)
            self.get_logger().debug(
                f'Sent setup: REG=1 SEL=101 CLK=0 BIP/UNI=0 RST=1 X=0 -> {bin(setup_reset)}'
            )

        except FileNotFoundError:
            self.get_logger().error("I2C bus not found. Is I2C enabled? Shutting down.")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"Failed to open SMBus: {e}. Shutting down.")
            rclpy.shutdown()
            return

        # Timer for periodic reading at 1 Hz, or REF toggle test
        if self.test_ref:
            self.toggle_timer = self.create_timer(2.0, self._toggle_refout)
            self.get_logger().info('ADC node initialized in --test_ref mode (toggling AIN3 REFOUT/input at 0.5 Hz).')
        elif self.test_cfg:
            self.toggle_timer = self.create_timer(2.0, self._toggle_cfg_include_ain3)
            self.get_logger().info('ADC node initialized in --test_cfg mode (toggling SCAN end CS between AIN2 and AIN3 at 0.5 Hz).')
        else:
            self.timer = self.create_timer(1.0, self.read_and_publish)
            self.get_logger().info('ADC node initialized and reading at 1 Hz.')

        # Debug bars (tqdm)
        self._bars = {}
        if _HAS_TQDM and ('--debug' in sys.argv) and not (self.test_ref or self.test_cfg):
            # Bars: battery (0-12 V), saltwater (0-4.2 V), sail current (0-4.2 A)
            self._bars['bat'] = tqdm(total=100, desc='Battery (V)', position=0, leave=False, dynamic_ncols=True)
            self._bars['sw']  = tqdm(total=100, desc='Saltwater (V)', position=1, leave=False, dynamic_ncols=True)
            self._bars['cur'] = tqdm(total=100, desc='Sail Current (A)', position=2, leave=False, dynamic_ncols=True)

    def _scale_pct(self, value, vmin, vmax):
        if vmax == vmin:
            return 0
        pct = int(100.0 * (max(min(value, vmax), vmin) - vmin) / (vmax - vmin))
        return max(0, min(100, pct))

    def _update_bars(self, bat_v, sw_v, cur_a):
        if not self._bars:
            return
        # Nominal ranges
        bat_max = 12.0
        sw_max  = 4.2
        cur_max = 4.2
        self._bars['bat'].n = self._scale_pct(bat_v, 0.0, bat_max)
        self._bars['bat'].set_postfix_str(f"{bat_v:.3f} V"); self._bars['bat'].refresh()
        self._bars['sw' ].n = self._scale_pct(sw_v, 0.0, sw_max)
        self._bars['sw' ].set_postfix_str(f"{sw_v:.3f} V"); self._bars['sw' ].refresh()
        self._bars['cur'].n = self._scale_pct(cur_a, 0.0, cur_max)
        self._bars['cur'].set_postfix_str(f"{cur_a:.3f} A"); self._bars['cur'].refresh()

    def _i2c_write_read(self, write_byte: int) -> list:
        """Perform write of 1 byte then read 2 bytes using repeated start (smbus2 if available)."""
        if not self.use_smbus2 or smbus2 is None or i2c_msg is None:
            self.bus.write_byte(self.i2c_addr, write_byte)
            time.sleep(0.005)
            return self.bus.read_i2c_block_data(self.i2c_addr, 0x00, 2)
        write = i2c_msg.write(self.i2c_addr, bytes([write_byte]))
        read = i2c_msg.read(self.i2c_addr, 2)
        with smbus2.SMBus(0) as b:
            b.i2c_rdwr(write, read)
        return list(read)

    def _i2c_write(self, byte_val: int) -> None:
        if self.use_smbus2 and i2c_msg is not None:
            msg = i2c_msg.write(self.i2c_addr, bytes([byte_val]))
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(msg)
        else:
            self.bus.write_byte(self.i2c_addr, byte_val)

    def _i2c_read1(self) -> int:
        if self.use_smbus2 and i2c_msg is not None:
            read = i2c_msg.read(self.i2c_addr, 1)
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(read)
            return list(read)[0]
        else:
            return self.bus.read_byte(self.i2c_addr)

    def _i2c_read_block(self, num_bytes: int) -> list:
        if self.use_smbus2 and i2c_msg is not None:
            read = i2c_msg.read(self.i2c_addr, num_bytes)
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(read)
            return list(read)
        else:
            return self.bus.read_i2c_block_data(self.i2c_addr, 0x00, num_bytes)

    def _i2c_read2(self) -> list:
        if self.use_smbus2 and i2c_msg is not None:
            read = i2c_msg.read(self.i2c_addr, 2)
            with smbus2.SMBus(0) as b:
                b.i2c_rdwr(read)
            return list(read)
        else:
            return self.bus.read_i2c_block_data(self.i2c_addr, 0x00, 2)

    def read_and_publish(self):
        """
        Read ADC channels sequentially (CH0..CH3) with multiple samples per channel,
        averaging per Table 5 using SCAN=01 (convert selected channel eight times).
        """
        # Config byte format: REG(1b)<<7 | SCAN[1:0]<<5 | CS[3:0]<<1 | SGL/DIF(1b)
        def build_config(reg, scan, cs, sgl_dif):
            return ((reg & 1) << 7) | ((scan & 0b11) << 5) | ((cs & 0b1111) << 1) | (sgl_dif & 1)

        repeats = 8  # SCAN=01: device converts selected channel eight times
        settle_delay_s = 0.001

        raw = [0, 0, 0, 0]
        try:
            for ch in range(4):
                # REG=0, SCAN=01 (convert selected input eight times), CS=ch, SGL/DIF=1
                cfg = build_config(reg=0, scan=0b01, cs=ch, sgl_dif=1)
                self._i2c_write(cfg)
                time.sleep(settle_delay_s)
                acc = 0
                for _ in range(repeats):
                    d = self._i2c_read2()
                    code = ((d[0] & 0x0F) << 8) | d[1]
                    acc += code
                raw[ch] = acc // repeats
            self.get_logger().debug(f'AVG raw (SCAN=01): CH0={raw[0]}, CH1={raw[1]}, CH2={raw[2]}, CH3={raw[3]}')
        except IOError as e:
            self.get_logger().error(f"I2C SCAN=01 average transaction failed: {e}")
            return

        # Publish readings
        battery_voltage = raw[0] * self.lsb_value * 2.0
        msg_b = Float32(); msg_b.data = battery_voltage
        self.pub_battery_voltage.publish(msg_b)
        self.get_logger().debug(f'Battery Voltage: {battery_voltage:.3f} V')

        saltwater_voltage = raw[1] * self.lsb_value
        msg_s = Float32(); msg_s.data = saltwater_voltage
        self.pub_saltwater_voltage.publish(msg_s)
        self.get_logger().debug(f'Saltwater Voltage: {saltwater_voltage:.3f} V')

        sail_current = raw[2] * self.lsb_value
        msg_i = Float32(); msg_i.data = sail_current
        self.pub_sail_current.publish(msg_i)
        self.get_logger().debug(f'Sail Current: {sail_current:.3f} A')

        # Update debug bars if enabled
        self._update_bars(battery_voltage, saltwater_voltage, sail_current)

        # CH3 averaged code logged for diagnostics only

    def i2c_test(self):
        """
        Verifies I2C communication by enabling REFOUT on AIN3, measuring it,
        then disabling it and plotting the voltage decay curve.
        """
        self.get_logger().info("--- Starting I2C communication test ---")

        # Helper functions
        def build_setup(reg, sel, clk, bip_uni, rst, x):
            return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)

        def build_config(reg, scan, cs, sgl_dif):
            return ((reg & 1) << 7) | ((scan & 0b11) << 5) | ((cs & 0b1111) << 1) | (sgl_dif & 1)

        try:
            # Part 1: Enable REFOUT on AIN3 and measure
            self.get_logger().info("Setting AIN3 to output internal reference voltage...")
            setup_refout = build_setup(reg=1, sel=0b111, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(setup_refout)
            time.sleep(0.005)  # Wait 5ms for reference to stabilize

            config_ain3 = build_config(reg=0, scan=0b01, cs=0b0011, sgl_dif=1)
            self._i2c_write(config_ain3)
            time.sleep(0.002) # Wait for conversion

            d = self._i2c_read2()
            raw = ((d[0] & 0x0F) << 8) | d[1]
            voltage = raw * self.lsb_value
            self.get_logger().info(f"AIN3 voltage with REFOUT enabled: {voltage:.3f} V (raw: {raw})")
            if raw > 4000: # Vref is 4.096V, raw should be close to 4095
                self.get_logger().info("Reference voltage level seems correct.")
            else:
                self.get_logger().warning("Reference voltage level seems low.")


            # Part 2: Set AIN3 to input and capture decay
            self.get_logger().info("Setting AIN3 to analog input to capture decay...")
            # Set SEL=101 for AIN3 as input. Crucially, set RST=0 to avoid resetting the config register.
            setup_ain3_in = build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=0, x=0)
            self._i2c_write(setup_ain3_in)

            # The config for reading AIN3 is still active from the previous step.
            
            captures = []
            self.get_logger().info("Capturing decay for 3 seconds...")
            start_time = time.time()
            
            while time.time() - start_time < 3.0:
                loop_start = time.time()

                # Write config to trigger a new conversion on AIN3, then read the 2-byte result
                d = self._i2c_write_read(config_ain3)
                raw = ((d[0] & 0x0F) << 8) | d[1]
                voltage = raw * self.lsb_value
                captures.append(voltage)
                
                elapsed = time.time() - loop_start
                if elapsed < 0.01:
                    time.sleep(0.01 - elapsed)

            self.get_logger().info(f"Captured {len(captures)} samples.")

            # Part 3: Plot results
            if not plotting_enabled:
                self.get_logger().warning("matplotlib not installed. Cannot display plot. Skipping.")
                self.get_logger().info("--- I2C communication test finished ---")
                return

            time_axis = [i * 10 for i in range(len(captures))]  # in ms
            plt.figure(figsize=(10, 6))
            plt.plot(time_axis, captures, marker='.', linestyle='-')
            plt.title("AIN3 Voltage Decay Test")
            plt.xlabel("Time (ms)")
            plt.ylabel("Voltage (V)")
            plt.grid(True)
            plt.ylim(0, self.vref * 1.05)
            plt.savefig('i2c_test.png')
            self.get_logger().info("Plot saved to i2c_test.png")

        except IOError as e:
            self.get_logger().error(f"I2C Communication Test: FAIL. I2C transaction failed: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during I2C test: {e}")

        self.get_logger().info("--- I2C communication test finished ---")

    def _toggle_refout(self) -> None:
        # Build setup byte for toggling REFOUT using bitfields
        def build_setup(reg, sel, clk, bip_uni, rst, x):
            return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)
        # REFOUT on AIN3: SEL=111; AIN3 as analog input: SEL=101
        byte_val = build_setup(reg=1, sel=(0b111 if not self._ref_is_output else 0b101), clk=0, bip_uni=0, rst=1, x=0)
        try:
            self._i2c_write(byte_val)
            self._ref_is_output = not self._ref_is_output
            mode = 'REFOUT on AIN3' if self._ref_is_output else 'AIN3 as analog input'
            self.get_logger().debug(f'Toggled AIN3: wrote {bin(byte_val)} ({mode})')
        except Exception as e:
            self.get_logger().error(f'Failed to toggle AIN3 REFOUT/input: {e}')

    def _toggle_cfg_include_ain3(self) -> None:
        # Build config byte from bitfields for inspection
        def build_config(reg, scan, cs, sgl_dif):
            return ((reg & 1) << 7) | ((scan & 0b11) << 5) | ((cs & 0b1111) << 1) | (sgl_dif & 1)
        # Alternate between CS=0010 (end at AIN2) and CS=0011 (end at AIN3), SCAN=00, REG=0, SGL/DIF=1
        cs = 0b0011 if not self._cfg_includes_ain3 else 0b0010
        cfg = build_config(reg=0, scan=0b00, cs=cs, sgl_dif=1)
        try:
            self._i2c_write(cfg)
            self._cfg_includes_ain3 = not self._cfg_includes_ain3
            self.get_logger().debug(
                f'Toggled config: REG=0 SCAN=00 CS={("0011" if self._cfg_includes_ain3 else "0010")} SGL/DIF=1 -> {bin(cfg)}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to toggle config include/exclude AIN3: {e}')

    def shutdown_adc(self) -> None:
        # On exit, disable REFOUT and leave AIN3 as analog input so the pin droops (SEL=101)
        def build_setup(reg, sel, clk, bip_uni, rst, x):
            return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)
        try:
            setup_disable_refout = build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(setup_disable_refout)
            self.get_logger().debug(f'Shutdown: wrote setup to disable REFOUT (SEL=101) -> {bin(setup_disable_refout)}')
        except Exception as e:
            self.get_logger().error(f'Failed to write shutdown setup: {e}')
        # Close tqdm bars if any
        if hasattr(self, '_bars'):
            for b in self._bars.values():
                try:
                    b.close()
                except Exception:
                    pass

    def _quiet_shutdown(self) -> None:
        # Same as shutdown_adc but without any logging to avoid rosout errors
        def build_setup(reg, sel, clk, bip_uni, rst, x):
            return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)
        try:
            setup_disable_refout = build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
            self._i2c_write(setup_disable_refout)
        except Exception:
            pass
        if hasattr(self, '_bars'):
            for b in self._bars.values():
                try:
                    b.close()
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    adc_node = AdcNode()
    if adc_node.i2c_test_mode:
        rclpy.shutdown()
        return
    if rclpy.ok():
        try:
            rclpy.spin(adc_node)
        except KeyboardInterrupt:
            # Perform non-logging shutdown to avoid rosout traceback
            adc_node._quiet_shutdown()
            try:
                adc_node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)
        except ExternalShutdownException:
            adc_node._quiet_shutdown()
            try:
                adc_node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)

if __name__ == '__main__':
    main()
