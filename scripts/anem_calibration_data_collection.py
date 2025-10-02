#!/usr/bin/env python3
"""
Anemometer Calibration Data Collection Script

This script systematically collects differential pressure data from the three
SDP3x sensors at known wind angles to calibrate the wind direction calculation.

Usage:
    python3 scripts/anem_calibration_data_collection.py

The script will:
1. Prompt you to set wind angles from -180° to +180° in 15° steps
2. Wait 5 seconds for you to position the wind source
3. Collect data for 5 seconds at each angle
4. Calculate mean and standard deviation for each sensor
5. Save results to anem-measurement-DATESTAMP.csv

Hardware Setup:
- Ensure anemometer is mounted and sensors are accessible
- Use a controlled wind source (fan, blower, etc.)
- Position wind source at the specified angles relative to boat front
- Wind speed should be approximately 7.4 m/s for calibration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import smbus
import time
import csv
import numpy as np
import sys
from datetime import datetime
import os

# I2C sensor addresses (matching anem.py)
I2C_CTR = 0x21  # Center sensor (front/back)
I2C_CW = 0x22   # Clockwise 60° from top (looking down on mast)
I2C_CCW = 0x23  # Counter-clockwise 60° from top (looking down on mast)

# Data collection parameters
WIND_SPEED_REF = 7.4  # m/s - reference wind speed for calibration
SETUP_TIME = 3.0      # seconds to wait for positioning
COLLECTION_TIME = 7.0  # seconds to collect data at each angle
SAMPLE_RATE = 10.0    # Hz - data collection rate
ANGLE_STEP = 30       # degrees - angle increment
ANGLE_RANGE = (-180, 180)  # degrees - total range

# CRC calculation functions (from anem.py)
def calculate_crc8(data_bytes):
    """Calculate CRC-8 checksum for SDP3x sensor data"""
    crc = 0xFF
    polynomial = 0x31
    
    for byte in data_bytes:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc = crc << 1
            crc &= 0xFF
    
    return crc

def verify_crc(data_bytes, received_crc):
    """Verify CRC checksum for sensor data"""
    calculated_crc = calculate_crc8(data_bytes)
    return calculated_crc == received_crc

def int_from_bytes(b):
    """Convert big-endian signed integer bytearray to int"""
    if not b:
        return 0
    n = b[0] & 0x7f
    for by in b[1:]:
        n = n * 256 + by
    if b[0] & 0x80:
        bits = 8*len(b)
        offset = 2**(bits-1)
        return n - offset
    else:
        return n

class AnemCalibrationCollector(Node):
    def __init__(self):
        super().__init__('anem_calibration_collector')
        
        # I2C setup
        self.i2cAddr = (I2C_CTR, I2C_CW, I2C_CCW)
        self.bus = None
        
        try:
            self.bus = smbus.SMBus(0)
            self.get_logger().info('Opened I2C SMBus')
        except FileNotFoundError:
            self.get_logger().error("CRITICAL: I2C bus not found. Is I2C enabled? Exiting.")
            sys.exit(1)
        
        # Setup sensors
        if not self.setup_sensors():
            self.get_logger().fatal("FATAL: Failed to setup anemometer sensors. Exiting.")
            sys.exit(1)
        
        # Data storage
        self.measurement_data = []
        
        # Create output filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = f"anem-measurement-{timestamp}.csv"
        
        print(f"\n{'='*60}")
        print("ANEMOMETER CALIBRATION DATA COLLECTION")
        print(f"{'='*60}")
        print(f"Wind Speed Reference: {WIND_SPEED_REF} m/s")
        print(f"Angle Range: {ANGLE_RANGE[0]}° to {ANGLE_RANGE[1]}° (step: {ANGLE_STEP}°)")
        print(f"Setup Time: {SETUP_TIME} seconds")
        print(f"Collection Time: {COLLECTION_TIME} seconds per angle")
        print(f"Output File: {self.output_file}")
        print(f"{'='*60}\n")

    def setup_sensors(self):
        """Initialize SDP3x sensors for continuous measurement"""
        try:
            # Stop any existing measurements
            for addr in self.i2cAddr:
                self.bus.write_i2c_block_data(addr, 0x3F, [0xF9])
            
            time.sleep(0.8)
            
            # Start continuous measurement (0x3615: differential pressure, average till read)
            for addr in self.i2cAddr:
                self.bus.write_i2c_block_data(addr, 0x36, [0x15])
            
            time.sleep(0.1)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup sensors: {e}")
            return False

    def read_sensor_data(self):
        """Read and validate sensor data with CRC checksum verification"""
        try:
            dp = [0.0, 0.0, 0.0]
            temps = [0.0, 0.0, 0.0]
            
            for i, addr in enumerate(self.i2cAddr):
                try:
                    b = self.bus.read_i2c_block_data(addr, 0, 9)
                except Exception as e:
                    self.get_logger().error(f"I2C read error: {e}")
                    return None
                
                # Verify CRC for differential pressure data
                if not verify_crc([b[0], b[1]], b[2]):
                    return None
                
                # Verify CRC for temperature data
                if not verify_crc([b[3], b[4]], b[5]):
                    return None
                
                # Verify CRC for scale factor data
                if not verify_crc([b[6], b[7]], b[8]):
                    return None
                
                # Convert validated data
                dp[i] = int_from_bytes([b[0], b[1]]) / 240.0  # Pascals
                temps[i] = int_from_bytes([b[3], b[4]]) / 200.0  # Celsius
            
            return dp, temps
            
        except Exception as e:
            self.get_logger().error(f"I2C read error: {e}")
            return None

    def collect_data_at_angle(self, angle_deg):
        """Collect data for specified time at given angle"""
        print(f"\n{'='*40}")
        print(f"COLLECTING DATA AT ANGLE: {angle_deg:+.0f}°")
        print(f"{'='*40}")
        
        # Wait for positioning
        print(f"Position wind source at {angle_deg:+.0f}° relative to boat front.")
        input("Press Enter when ready to collect data...")
        import os
        os.system('clear')
        print(f"\nStarting data collection for {COLLECTION_TIME:.0f} seconds...")
        
        # Collect data
        samples = []
        start_time = time.time()
        sample_count = 0
        
        while time.time() - start_time < COLLECTION_TIME:
            sensor_data = self.read_sensor_data()
            if sensor_data is not None:
                dp, temps = sensor_data
                samples.append({
                    'timestamp': time.time(),
                    'dp_ctr': dp[0],
                    'dp_cw': dp[1], 
                    'dp_ccw': dp[2],
                    'temp_ctr': temps[0],
                    'temp_cw': temps[1],
                    'temp_ccw': temps[2]
                })
                sample_count += 1
            
            time.sleep(1.0 / SAMPLE_RATE)
        
        if not samples:
            print("ERROR: No valid samples collected!")
            return None
        
        # Calculate statistics
        dp_ctr_values = [s['dp_ctr'] for s in samples]
        dp_cw_values = [s['dp_cw'] for s in samples]
        dp_ccw_values = [s['dp_ccw'] for s in samples]
        
        result = {
            'angle_deg': angle_deg,
            'wind_speed_ref': WIND_SPEED_REF,
            'sample_count': sample_count,
            'dp_ctr_mean': np.mean(dp_ctr_values),
            'dp_ctr_std': np.std(dp_ctr_values),
            'dp_cw_mean': np.mean(dp_cw_values),
            'dp_cw_std': np.std(dp_cw_values),
            'dp_ccw_mean': np.mean(dp_ccw_values),
            'dp_ccw_std': np.std(dp_ccw_values),
            'temp_mean': np.mean([s['temp_ctr'] for s in samples] + 
                                [s['temp_cw'] for s in samples] + 
                                [s['temp_ccw'] for s in samples])
        }
        
        # Display results
        print(f"\nResults for {angle_deg:+.0f}°:")
        print(f"  Samples collected: {sample_count}")
        print(f"  CTR sensor: {result['dp_ctr_mean']:+.3f} ± {result['dp_ctr_std']:.3f} Pa")
        print(f"  CW  sensor:  {result['dp_cw_mean']:+.3f} ± {result['dp_cw_std']:.3f} Pa")
        print(f"  CCW sensor: {result['dp_ccw_mean']:+.3f} ± {result['dp_ccw_std']:.3f} Pa")
        print(f"  Temperature: {result['temp_mean']:.1f}°C")
        
        input("Press Enter to continue...")
        
        return result

    def save_data(self):
        """Save collected data to CSV file"""
        if not self.measurement_data:
            print("No data to save!")
            return
        
        fieldnames = [
            'angle_deg', 'wind_speed_ref', 'sample_count',
            'dp_ctr_mean', 'dp_ctr_std',
            'dp_cw_mean', 'dp_cw_std', 
            'dp_ccw_mean', 'dp_ccw_std',
            'temp_mean'
        ]
        
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.measurement_data)
        
        print(f"\n{'='*60}")
        print(f"DATA COLLECTION COMPLETE!")
        print(f"Results saved to: {self.output_file}")
        print(f"Total measurements: {len(self.measurement_data)}")
        print(f"{'='*60}")
        
        # Automatically run plotting script
        self.run_plotting_script()

    def run_plotting_script(self):
        """Run the plotting script to visualize the collected data"""
        print("\nGenerating plots...")
        
        try:
            import subprocess
            result = subprocess.run([
                'python3', 'scripts/plot_anem_calibration_data.py', self.output_file
            ], capture_output=True, text=True, timeout=60)
            
            if result.returncode == 0:
                print("✅ Plots generated successfully!")
                print("Check the 'plots/' directory for visualization files.")
            else:
                print("⚠️  Plot generation had issues:")
                print(result.stderr)
                
        except subprocess.TimeoutExpired:
            print("⚠️  Plot generation timed out")
        except Exception as e:
            print(f"⚠️  Error running plot script: {e}")
            print("You can manually run: python3 scripts/plot_anem_calibration_data.py")

    def run_calibration(self):
        """Run the complete calibration sequence"""
        # Generate angle sequence
        angles = list(range(ANGLE_RANGE[0], ANGLE_RANGE[1] + 1, ANGLE_STEP))
        total_angles = len(angles)
        
        print(f"Will collect data at {total_angles} angles:")
        print(f"Angles: {angles}")
        
        input("\nPress Enter when ready to start data collection...")
        
        for i, angle in enumerate(angles, 1):
            print(f"\n[{i}/{total_angles}] ", end="")
            
            result = self.collect_data_at_angle(angle)
            if result:
                self.measurement_data.append(result)
            
            # # Ask if user wants to continue
            # if i < total_angles:
            #     response = input(f"\nContinue to next angle? (Y/n/q to quit): ").lower()
            #     if response == 'q':
            #         print("Calibration aborted by user.")
            #         break
            #     elif response == 'n':
            #         print("Skipping remaining angles...")
            #         break
        
        # Save data
        self.save_data()

def main():
    print("Initializing anemometer calibration data collection...")
    
    rclpy.init()
    collector = AnemCalibrationCollector()
    
    try:
        collector.run_calibration()
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
        collector.save_data()
    except Exception as e:
        print(f"\nError during calibration: {e}")
        collector.save_data()
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
