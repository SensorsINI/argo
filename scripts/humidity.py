#!/usr/bin/env python
# - Reads temperature and humidity from an SHT45-AD1B sensor over I2C.
# - The default I2C address for the SHT45 is 0x44.
# - The read frequency is 1 Hz.
# - Sensor values are published to ROS topics as float values.
#   - 'temperature' in degrees Celsius
#   - 'relative_humidity' in percent (%)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus, i2c_msg # Use smbus2 for more flexible I2C transactions
import time
import sys

class HumidityNode(Node):
    def __init__(self):
        super().__init__('humidity_node')

        # Set logger level to DEBUG if --debug flag is passed
        if '--debug' in sys.argv:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.get_logger().info('Initializing Humidity node...')

        # Publishers
        self.pub_temperature = self.create_publisher(Float32, 'temperature', 10)
        self.pub_humidity = self.create_publisher(Float32, 'relative_humidity', 10)

        # I2C setup for SHT45
        self.i2c_addr = 0x44  # SHT45 default I2C address
        self.bus = None
        
        # SHT45 Commands
        self.SHT45_HIGH_PRECISION_CMD = 0xFD
        self.SHT45_MEASUREMENT_DELAY = 0.01 # 10ms delay for high precision measurement

        try:
            self.bus = SMBus(0) # The default i2c bus
            self.get_logger().info('Opened i2c SMBus for SHT45 sensor')
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
        self.get_logger().info('Humidity node initialized and reading at 1 Hz.')

    def _check_crc(self, data):
        """
        Calculates the CRC-8 checksum for the SHT4x sensor data.
        Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
        Initialization: 0xFF
        """
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
        return crc & 0xFF

    def read_sensor(self):
        """
        Sends measurement command and reads data from the SHT45 sensor.
        """
        try:
            # Step 1: Send the measurement command using a pure I2C write.
            # This is a single transaction containing only the command byte.
            write_cmd = i2c_msg.write(self.i2c_addr, [self.SHT45_HIGH_PRECISION_CMD])
            self.bus.i2c_rdwr(write_cmd)
            
            # Step 2: Wait for the measurement to complete.
            time.sleep(self.SHT45_MEASUREMENT_DELAY)
            
            # Step 3: Read 6 bytes of data using a pure I2C read.
            # This transaction does not send any register/command byte beforehand.
            read_data = i2c_msg.read(self.i2c_addr, 6)
            self.bus.i2c_rdwr(read_data)
            data = list(read_data)
            
            # Verify checksums
            temp_data = data[0:2]
            temp_crc = data[2]
            if self._check_crc(temp_data) != temp_crc:
                self.get_logger().warn('Temperature CRC mismatch')
                return None, None

            humid_data = data[3:5]
            humid_crc = data[5]
            if self._check_crc(humid_data) != humid_crc:
                self.get_logger().warn('Humidity CRC mismatch')
                return None, None

            # Combine MSB and LSB to get raw values
            raw_temp = (temp_data[0] << 8) | temp_data[1]
            raw_humidity = (humid_data[0] << 8) | humid_data[1]
            
            return raw_temp, raw_humidity

        except IOError as e:
            self.get_logger().error(f"I2C transaction failed: {e}")
            return None, None

    def read_and_publish(self):
        """
        Timer callback to read sensor values, convert them, and publish.
        """
        raw_temp, raw_humidity = self.read_sensor()

        if raw_temp is not None and raw_humidity is not None:
            # Apply conversion formulas from the SHT45 datasheet
            temperature = -45.0 + 175.0 * (raw_temp / 65535.0)
            humidity = -6.0 + 125.0 * (raw_humidity / 65535.0)
            
            # Clamp humidity to 0-100% range
            humidity = max(0.0, min(100.0, humidity))

            # Publish temperature
            temp_msg = Float32()
            temp_msg.data = temperature
            self.pub_temperature.publish(temp_msg)
            self.get_logger().debug(f'Temperature: {temperature:.2f} C')

            # Publish humidity
            humid_msg = Float32()
            humid_msg.data = humidity
            self.pub_humidity.publish(humid_msg)
            self.get_logger().debug(f'Relative Humidity: {humidity:.2f} %')

def main(args=None):
    rclpy.init(args=args)
    humidity_node = HumidityNode()
    if rclpy.ok():
        try:
            rclpy.spin(humidity_node)
        except KeyboardInterrupt:
            pass
        finally:
            humidity_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
