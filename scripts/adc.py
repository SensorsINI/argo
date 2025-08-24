#!/usr/bin/env python
# - adapted from anem.py to read values over i2c from the MAX11612 ADC chip. The i2c address is 0x34.
# - the reference voltage is 4.096 volts, so each DN is 1mV for this 12-bit converter.
# - reads saltwater voltage from AIN1 with 1MOhm resistor load.
# - reads sail winch current from AIN3 with a 1Ohm resistor shunt.
# - reads battery_voltage from AIN0 over a voltage divider with R1=R2.
# - The read frequency is 1 Hz.
# - sensor values are published to ros topics as float values with units of either voltage or current.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import time
import sys

class AdcNode(Node):
    def __init__(self):
        super().__init__('adc_node')

        # Set logger level to DEBUG if --debug flag is passed
        if '--debug' in sys.argv:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

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
        except FileNotFoundError:
            self.get_logger().error("I2C bus not found. Is I2C enabled? Shutting down.")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"Failed to open SMBus: {e}. Shutting down.")
            rclpy.shutdown()
            return

        # Timer for periodic reading at 1 Hz
        self.timer = self.create_timer(1.0, self.read_and_publish)
        self.get_logger().info('ADC node initialized and reading at 1 Hz.')

    def read_adc(self, channel):
        """
        Reads a single channel from the MAX11612 ADC.
        """
        # Setup byte for MAX11612:
        # Bit 7: 1 (SETUP bit, indicates a setup byte)
        # Bit 6-4: Channel select (SEL2-SEL0)
        # Bit 3-2: 00 (SCAN bits, single conversion on selected channel)
        # Bit 1: 1 (SGL/DIF bit, single-ended input)
        # Bit 0: 0 (CS_EN bit, not used for single conversion)
        # Construct the setup byte: 0b1_CCC_0010 where CCC are the channel bits
        config = 0b10000010 | (channel << 4)

        try:
            # The MAX11612 datasheet indicates a "read-twice" pattern is needed
            # when switching channels or after a period of inactivity.
            # The first I2C transaction (write config + read data) selects the
            # new channel, starts a conversion, and returns stale data from the
            # *previous* channel's conversion. We discard this stale data.
            self.bus.read_i2c_block_data(self.i2c_addr, config, 2)

            # The conversion time (t_CONV) is very short (max 3.5us). A small
            # delay ensures the conversion is complete before the next read.
            time.sleep(0.001)

            # The second I2C transaction re-sends the config, starts a new
            # conversion, and crucially, returns the result from the conversion
            # we initiated in the first step. This is the valid data we want.
            data = self.bus.read_i2c_block_data(self.i2c_addr, config, 2)

            # The result is a 12-bit value, right-justified.
            # The 4 MSBs of the first byte are don't care bits.
            raw_adc = ((data[0] & 0x0F) << 8) | data[1]
            return raw_adc
        except IOError as e:
            self.get_logger().error(f"I2C transaction failed for channel {channel}: {e}")
            return None

    def read_and_publish(self):
        """
        Timer callback to read ADC values, convert them, and publish them.
        """
        # Read battery voltage from AIN0
        raw_battery = self.read_adc(0)
        if raw_battery is not None:
            # V_adc = raw * lsb_value. V_battery = 2 * V_adc due to R1=R2 voltage divider.
            battery_voltage = raw_battery * self.lsb_value * 2.0
            msg = Float32()
            msg.data = battery_voltage
            self.pub_battery_voltage.publish(msg)
            self.get_logger().debug(f'Battery Voltage: {battery_voltage:.3f} V')

        # Read saltwater voltage from AIN1
        raw_saltwater = self.read_adc(1)
        if raw_saltwater is not None:
            saltwater_voltage = raw_saltwater * self.lsb_value
            msg = Float32()
            msg.data = saltwater_voltage
            self.pub_saltwater_voltage.publish(msg)
            self.get_logger().debug(f'Saltwater Voltage: {saltwater_voltage:.3f} V')

        # Read sail winch current from AIN3
        raw_sail_current = self.read_adc(2)
        if raw_sail_current is not None:
            # V_shunt = raw * lsb_value. I = V_shunt / R_shunt. R_shunt = 1 Ohm.
            sail_current = raw_sail_current * self.lsb_value
            msg = Float32()
            msg.data = sail_current
            self.pub_sail_current.publish(msg)
            self.get_logger().debug(f'Sail Current: {sail_current:.3f} A')

def main(args=None):
    rclpy.init(args=args)
    adc_node = AdcNode()
    if rclpy.ok():
        try:
            rclpy.spin(adc_node)
        except KeyboardInterrupt:
            pass
        finally:
            adc_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
