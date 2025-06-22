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

class AdcNode(Node):
    def __init__(self):
        super().__init__('adc_node')
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
            self.bus = smbus.SMBus(1) # The default i2c bus
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
        # Configuration byte for MAX11612:
        # Bit 7: 1 (Start conversion)
        # Bit 6: 1 (Single-ended input)
        # Bit 5: 0 (CS3, not used for channels 0-7)
        # Bit 4-2: Channel select (CS2-CS0)
        # Bit 1-0: 00 (Single conversion on selected channel)
        config = 0b11000000 | (channel << 3)

        try:
            # This command writes the config byte then reads 2 bytes of data
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
        raw_sail_current = self.read_adc(3)
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
