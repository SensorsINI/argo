#!/usr/bin/env python3
"""
BNO085 I2C Communication Test Script

This script tests basic I2C communication with the BNO085 sensor
and implements the SHTP protocol for debugging purposes.

Usage:
    python3 test_bno085.py [--address 0x4a] [--debug]
"""

import sys
import time
import struct
import argparse
import smbus2

class BNO085Test:
    """Simple BNO085 test class for debugging I2C communication."""
    
    def __init__(self, bus_num=0, address=0x4a, debug=False):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.debug = debug
        self._sequence_number = 0
        
        # SHTP constants
        self.SHTP_HEADER_SIZE = 4
        self.MAX_PACKET_SIZE = 128
        
        # SHTP Channel numbers (from BNO080 datasheet)
        self.CHANNEL_COMMAND = 0x00      # SHTP command channel
        self.CHANNEL_EXECUTABLE = 0x01   # Executable channel
        self.CHANNEL_CONTROL = 0x02      # Sensor hub control channel (SH-2)
        self.CHANNEL_REPORTS = 0x03      # Input sensor reports
        
        # SH-2 Report IDs (from BNO080 datasheet)
        self.REPORT_PRODUCT_ID_REQUEST = 0xF9     # Product ID Request
        self.REPORT_PRODUCT_ID_RESPONSE = 0xF8    # Product ID Response
        self.REPORT_SET_FEATURE_COMMAND = 0xFD    # Set Feature Command
        
        # Sensor report IDs
        self.REPORT_ACCELEROMETER = 0x01      # Accelerometer
        self.REPORT_GYROSCOPE = 0x02          # Gyroscope
        self.REPORT_MAGNETOMETER = 0x03       # Magnetometer
    
    def debug_print(self, message):
        """Print debug message if debug mode is enabled."""
        if self.debug:
            print(f"BNO085 DEBUG: {message}")
    
    def test_basic_i2c(self):
        """Test basic I2C communication with the BNO085."""
        print(f"Testing basic I2C communication with BNO085 at address 0x{self.addr:02x}...")
        
        try:
            # Try to read a single byte
            test_data = self.bus.read_i2c_block_data(self.addr, 0, 1)
            print(f"✅ Basic I2C read successful: {test_data}")
            return True
        except Exception as e:
            print(f"❌ Basic I2C read failed: {e}")
            
            # Try to wake up the device
            print("Attempting to wake up BNO085...")
            try:
                self.bus.write_i2c_block_data(self.addr, 0, [0x00])
                time.sleep(0.1)
                
                test_data = self.bus.read_i2c_block_data(self.addr, 0, 1)
                print(f"✅ Wake-up successful: {test_data}")
                return True
            except Exception as e2:
                print(f"❌ Wake-up failed: {e2}")
                return False
    
    def _read_packet(self):
        """Read a complete SHTP packet from the BNO085 using proper I2C protocol."""
        try:
            # Read header first (4 bytes: length_low, length_high, channel, sequence)
            header = self.bus.read_i2c_block_data(self.addr, 0, self.SHTP_HEADER_SIZE)
            
            # Parse header according to SHTP specification from datasheet
            packet_length = (header[1] << 8) | header[0]  # Little-endian length
            channel = header[2]
            sequence_number = header[3]
            
            self.debug_print(f"Header: length={packet_length}, channel={channel}, seq={sequence_number}, raw={header}")
            
            # Validate packet length
            if packet_length > self.MAX_PACKET_SIZE or packet_length < 0:
                self.debug_print(f"Invalid packet length: {packet_length}")
                return None
                
            # If packet length is 0, return header-only packet
            if packet_length == 0:
                self.debug_print("Zero-length packet received")
                return {
                    'length': 0,
                    'sequence': sequence_number,
                    'channel': channel,
                    'payload': []
                }
                
            # Read payload in separate transaction
            payload_length = packet_length - self.SHTP_HEADER_SIZE
            if payload_length > 0:
                payload = self.bus.read_i2c_block_data(self.addr, 0, payload_length)
                self.debug_print(f"Payload: {payload}")
            else:
                payload = []
            
            return {
                'length': packet_length,
                'sequence': sequence_number,
                'channel': channel,
                'payload': payload
            }
        except Exception as e:
            self.debug_print(f"Error reading packet: {e}")
            return None
    
    def _write_packet(self, channel, data):
        """Write a SHTP packet to the BNO085."""
        try:
            # SHTP header format: Length LSB, Length MSB, Channel, SeqNum
            packet_length = len(data) + self.SHTP_HEADER_SIZE
            header = [
                packet_length & 0xFF,           # Length LSB (little-endian)
                (packet_length >> 8) & 0xFF,    # Length MSB
                channel & 0xFF,                 # Channel
                self._sequence_number & 0xFF    # Sequence number
            ]
            
            self.debug_print(f"Writing packet: channel={channel}, length={packet_length}, seq={self._sequence_number}, data={data}")
            
            # Write header
            self.bus.write_i2c_block_data(self.addr, 0, header)
            
            # Write payload if any
            if data:
                self.bus.write_i2c_block_data(self.addr, 0, data)
            
            self._sequence_number = (self._sequence_number + 1) % 256
            return True
        except Exception as e:
            self.debug_print(f"Error writing packet: {e}")
            return False
    
    def get_product_id(self):
        """Get BNO085 product ID and version information."""
        print("Requesting Product ID...")
        
        try:
            # Send Product ID request (Report ID 0xF9)
            self._write_packet(self.CHANNEL_CONTROL, [
                self.REPORT_PRODUCT_ID_REQUEST,  # Report ID
                0x00  # Reserved
            ])
            
            # Wait for response
            time.sleep(0.1)
            
            # Read response packets
            for _ in range(10):  # Try up to 10 packets
                packet = self._read_packet()
                if packet and packet['channel'] == self.CHANNEL_CONTROL:
                    self.debug_print(f"Received packet: length={packet['length']}, channel={packet['channel']}, payload={packet['payload']}")
                    
                    if (len(packet['payload']) > 0 and 
                        packet['payload'][0] == self.REPORT_PRODUCT_ID_RESPONSE and
                        len(packet['payload']) >= 16):
                        
                        # Parse Product ID response
                        reset_cause = packet['payload'][1]
                        sw_major = packet['payload'][2]
                        sw_minor = packet['payload'][3]
                        sw_part = (packet['payload'][7] << 24) | (packet['payload'][6] << 16) | (packet['payload'][5] << 8) | packet['payload'][4]
                        sw_build = (packet['payload'][11] << 24) | (packet['payload'][10] << 16) | (packet['payload'][9] << 8) | packet['payload'][8]
                        sw_patch = (packet['payload'][13] << 8) | packet['payload'][12]
                        
                        version_info = {
                            'reset_cause': reset_cause,
                            'sw_major': sw_major,
                            'sw_minor': sw_minor,
                            'sw_patch': sw_patch,
                            'sw_part': sw_part,
                            'sw_build': sw_build
                        }
                        
                        print(f"✅ Product ID Response:")
                        print(f"   Reset Cause: {reset_cause}")
                        print(f"   Version: {sw_major}.{sw_minor}.{sw_patch}")
                        print(f"   Part Number: 0x{sw_part:08X}")
                        print(f"   Build Number: {sw_build}")
                        return version_info
                        
            print("❌ No Product ID response received")
            return None
            
        except Exception as e:
            print(f"❌ Error getting Product ID: {e}")
            return None
    
    def enable_sensor_reports(self):
        """Enable accelerometer, gyroscope, and magnetometer reports."""
        print("Enabling sensor reports...")
        
        sensors = [
            (self.REPORT_ACCELEROMETER, "Accelerometer"),
            (self.REPORT_GYROSCOPE, "Gyroscope"),
            (self.REPORT_MAGNETOMETER, "Magnetometer")
        ]
        
        for report_id, name in sensors:
            try:
                # Enable sensor report using Set Feature Command
                success = self._write_packet(self.CHANNEL_CONTROL, [
                    self.REPORT_SET_FEATURE_COMMAND,  # Set Feature Command (0xFD)
                    0x00,  # Reserved
                    report_id,  # Report ID
                    0x00, 0x00,  # Feature flags
                    0x00, 0x00,  # Change sensitivity
                    0x00, 0x00, 0x00, 0x00,  # Report interval (0 = 50Hz)
                    0x00, 0x00, 0x00, 0x00,  # Batch interval
                    0x00, 0x00, 0x00, 0x00   # Sensor-specific config
                ])
                
                if success:
                    print(f"✅ {name} report enabled")
                else:
                    print(f"❌ Failed to enable {name} report")
                    
            except Exception as e:
                print(f"❌ Error enabling {name}: {e}")
    
    def read_sensor_data(self, max_attempts=10):
        """Try to read sensor data from the BNO085."""
        print("Attempting to read sensor data...")
        
        sensor_data = {
            'accelerometer': None,
            'gyroscope': None,
            'magnetometer': None
        }
        
        for attempt in range(max_attempts):
            packet = self._read_packet()
            if packet and packet['channel'] == self.CHANNEL_REPORTS and len(packet['payload']) >= 12:
                report_id = packet['payload'][0]
                
                if report_id == self.REPORT_ACCELEROMETER and sensor_data['accelerometer'] is None:
                    # Parse accelerometer data (3x 32-bit floats)
                    try:
                        x = struct.unpack('<f', bytes(packet['payload'][1:5]))[0]
                        y = struct.unpack('<f', bytes(packet['payload'][5:9]))[0]
                        z = struct.unpack('<f', bytes(packet['payload'][9:13]))[0]
                        sensor_data['accelerometer'] = (x, y, z)
                        print(f"✅ Accelerometer: x={x:.3f}, y={y:.3f}, z={z:.3f} m/s²")
                    except:
                        pass
                        
                elif report_id == self.REPORT_GYROSCOPE and sensor_data['gyroscope'] is None:
                    # Parse gyroscope data
                    try:
                        x = struct.unpack('<f', bytes(packet['payload'][1:5]))[0]
                        y = struct.unpack('<f', bytes(packet['payload'][5:9]))[0]
                        z = struct.unpack('<f', bytes(packet['payload'][9:13]))[0]
                        sensor_data['gyroscope'] = (x, y, z)
                        print(f"✅ Gyroscope: x={x:.3f}, y={y:.3f}, z={z:.3f} rad/s")
                    except:
                        pass
                        
                elif report_id == self.REPORT_MAGNETOMETER and sensor_data['magnetometer'] is None:
                    # Parse magnetometer data
                    try:
                        x = struct.unpack('<f', bytes(packet['payload'][1:5]))[0]
                        y = struct.unpack('<f', bytes(packet['payload'][5:9]))[0]
                        z = struct.unpack('<f', bytes(packet['payload'][9:13]))[0]
                        sensor_data['magnetometer'] = (x, y, z)
                        print(f"✅ Magnetometer: x={x:.3f}, y={y:.3f}, z={z:.3f} µT")
                    except:
                        pass
        
        # Report missing data
        for sensor, data in sensor_data.items():
            if data is None:
                print(f"❌ No {sensor} data received")
        
        return sensor_data
    
    def run_full_test(self):
        """Run complete BNO085 test sequence."""
        print("=" * 60)
        print("BNO085 I2C Communication Test")
        print("=" * 60)
        
        # Test 1: Basic I2C communication
        if not self.test_basic_i2c():
            print("❌ Basic I2C test failed - cannot proceed")
            return False
        
        print()
        
        # Test 2: Get Product ID
        version_info = self.get_product_id()
        if not version_info:
            print("❌ Product ID test failed - cannot proceed")
            return False
        
        print()
        
        # Test 3: Enable sensor reports
        self.enable_sensor_reports()
        
        print()
        
        # Test 4: Read sensor data
        sensor_data = self.read_sensor_data()
        
        print()
        print("=" * 60)
        print("Test Summary:")
        print("=" * 60)
        
        if version_info:
            print(f"✅ BNO085 detected and responsive")
            print(f"   Version: {version_info['sw_major']}.{version_info['sw_minor']}.{version_info['sw_patch']}")
        else:
            print("❌ BNO085 not responsive")
        
        data_count = sum(1 for data in sensor_data.values() if data is not None)
        print(f"✅ Sensor data received: {data_count}/3 sensors")
        
        return version_info is not None and data_count > 0


def main():
    parser = argparse.ArgumentParser(description='Test BNO085 I2C communication')
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x4a,
                       help='BNO085 I2C address (default: 0x4a)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output')
    
    args = parser.parse_args()
    
    # Create test instance
    tester = BNO085Test(address=args.address, debug=args.debug)
    
    # Run full test
    success = tester.run_full_test()
    
    if success:
        print("\n🎉 BNO085 test completed successfully!")
        sys.exit(0)
    else:
        print("\n💥 BNO085 test failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
