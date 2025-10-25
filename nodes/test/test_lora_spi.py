#!/usr/bin/env python3
"""
LoRa SPI Test Script for Hardware Diagnostics
==============================================

This script tests SPI communication with the SX1276 LoRa module
and provides oscilloscope-friendly output for hardware debugging.

Hardware Configuration:
- SPI Bus: 1
- SPI Device: 0 (CS0)
- SPI Mode: 0 (CPOL=0, CPHA=0)
- SPI Speed: 1 MHz (conservative for debugging)

GPIO Pins (Orange Pi Zero 2W):
- LORA_SEL (CS):  GPIO 266 (PI10) - Manual chip select
- LORA_RST (RST): GPIO 256 (PI0)  - Reset pin
- LORA_IRQ (DIO0): GPIO 271 (PI15) - Interrupt (not used in this test)

SX1276 Registers:
- 0x42: Version register (should read 0x12)
- 0x01: OpMode register (mode control)
- 0x06-0x08: Frequency registers

Usage:
    python3 test_lora_spi.py [test_mode]
    
Test Modes:
    basic    - Basic SPI read/write test (default)
    pins     - Toggle GPIO pins for oscilloscope verification
    reset    - Test reset sequence
    version  - Read chip version repeatedly
    regs     - Read multiple registers
    all      - Run all tests
"""

import time
import sys
import spidev
import gpiod

# SPI Configuration
SPI_BUS = 1
SPI_DEVICE = 0
SPI_MODE = 0b00  # Mode 0: CPOL=0, CPHA=0
SPI_MAX_SPEED = 1000000  # 1 MHz

# GPIO Configuration (gpiochip0 line numbers)
LORA_SEL = 266  # PI10 - Manual chip select
LORA_RST = 256  # PI0  - Reset
LORA_IRQ = 271  # PI15 - DIO0 interrupt (not used in this test)

# SX1276 Registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_VERSION = 0x42

class LoRaSPITest:
    def __init__(self):
        self.spi = None
        self.chip = None
        self.cs_line = None
        self.rst_line = None
        self.irq_line = None
        
    def init_gpio(self):
        """Initialize GPIO using gpiod"""
        print("Initializing GPIO...")
        
        # Open GPIO chip
        self.chip = gpiod.Chip('gpiochip0')
        print(f"  GPIO chip: {self.chip.name()}")
        
        # Configure CS (output, initially high = deselected)
        self.cs_line = self.chip.get_line(LORA_SEL)
        self.cs_line.request(consumer="lora_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        print(f"  LORA_SEL (GPIO {LORA_SEL}): Configured as output, initialized HIGH")
        
        # Configure RST (output, initially high = not in reset)
        self.rst_line = self.chip.get_line(LORA_RST)
        self.rst_line.request(consumer="lora_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        print(f"  LORA_RST (GPIO {LORA_RST}): Configured as output, initialized HIGH")
        
        # Configure IRQ (input)
        self.irq_line = self.chip.get_line(LORA_IRQ)
        self.irq_line.request(consumer="lora_test", type=gpiod.LINE_REQ_DIR_IN)
        print(f"  LORA_IRQ (GPIO {LORA_IRQ}): Configured as input")
        
    def init_spi(self):
        """Initialize SPI"""
        print("\nInitializing SPI...")
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.mode = SPI_MODE
        self.spi.max_speed_hz = SPI_MAX_SPEED
        self.spi.bits_per_word = 8
        print(f"  SPI Bus: {SPI_BUS}, Device: {SPI_DEVICE}")
        print(f"  Mode: {SPI_MODE}, Speed: {SPI_MAX_SPEED} Hz")
        print(f"  Bits per word: 8")
        
    def reset_module(self):
        """Reset the LoRa module"""
        print("\nResetting LoRa module...")
        print("  RST -> LOW")
        self.rst_line.set_value(0)
        time.sleep(0.1)  # 100ms reset pulse (increased for reliability)
        print("  RST -> HIGH")
        self.rst_line.set_value(1)
        time.sleep(0.01)  # 10ms after reset
        print("  Reset complete")
        
    def spi_write_register(self, reg_addr, value):
        """Write to SX1276 register via SPI"""
        # CS LOW
        self.cs_line.set_value(0)
        time.sleep(0.001)  # 1ms CS setup time
        
        # Write: command byte (0x80 | reg_addr) followed by data byte
        result = self.spi.xfer2([0x80 | reg_addr, value])
        
        # CS HIGH
        time.sleep(0.001)  # 1ms before CS release
        self.cs_line.set_value(1)
        
        return result
        
    def spi_read_register(self, reg_addr):
        """Read from SX1276 register via SPI"""
        # CS LOW
        self.cs_line.set_value(0)
        time.sleep(0.001)  # 1ms CS setup time
        
        # Read: command byte (reg_addr) followed by dummy byte
        result = self.spi.xfer2([reg_addr & 0x7F, 0x00])
        
        # CS HIGH
        time.sleep(0.001)  # 1ms before CS release
        self.cs_line.set_value(1)
        
        return result[1]  # Return data byte
        
    def test_basic(self):
        """Basic SPI communication test"""
        print("\n" + "="*60)
        print("TEST: Basic SPI Communication")
        print("="*60)
        
        # Read version register
        print("\nReading version register (0x42)...")
        version = self.spi_read_register(REG_VERSION)
        print(f"  Version: 0x{version:02X}")
        
        if version == 0x12:
            print("  ✓ Correct SX1276 version detected")
        elif version == 0x00 or version == 0xFF:
            print("  ✗ Communication failure (got 0x{:02X})".format(version))
            print("    Check: SPI wiring, power, module orientation")
        else:
            print(f"  ? Unexpected version: 0x{version:02X}")
            print("    Expected: 0x12 for SX1276")
            print("    Check: Module type, register address")
            
        # Read OpMode register
        print("\nReading OpMode register (0x01)...")
        opmode = self.spi_read_register(REG_OP_MODE)
        print(f"  OpMode: 0x{opmode:02X}")
        print(f"  Mode bits: {(opmode & 0x07):03b}")
        
        return version
        
    def test_pins(self):
        """Toggle GPIO pins for oscilloscope verification"""
        print("\n" + "="*60)
        print("TEST: GPIO Pin Toggling (for oscilloscope)")
        print("="*60)
        print("Press Ctrl+C to stop...")
        
        try:
            count = 0
            while True:
                count += 1
                print(f"\rToggle cycle {count}", end='', flush=True)
                
                # Toggle CS
                self.cs_line.set_value(0)
                time.sleep(0.01)
                self.cs_line.set_value(1)
                time.sleep(0.01)
                
                # Toggle RST
                self.rst_line.set_value(0)
                time.sleep(0.01)
                self.rst_line.set_value(1)
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n  Stopped by user")
            
    def test_reset(self):
        """Test reset sequence"""
        print("\n" + "="*60)
        print("TEST: Reset Sequence")
        print("="*60)
        
        for i in range(3):
            print(f"\nReset #{i+1}...")
            self.reset_module()
            time.sleep(0.5)
            
            # Read version after reset
            version = self.spi_read_register(REG_VERSION)
            print(f"  Version after reset: 0x{version:02X}")
            
    def test_version_repeated(self):
        """Read version register repeatedly"""
        print("\n" + "="*60)
        print("TEST: Repeated Version Reads")
        print("="*60)
        print("Press Ctrl+C to stop...")
        
        try:
            count = 0
            last_version = None
            while True:
                version = self.spi_read_register(REG_VERSION)
                count += 1
                
                if version != last_version:
                    print(f"\nRead #{count}: Version = 0x{version:02X}")
                    last_version = version
                else:
                    print(f"\rRead #{count}: Version = 0x{version:02X} (stable)", end='', flush=True)
                    
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n  Stopped by user")
            
    def test_registers(self):
        """Read multiple registers"""
        print("\n" + "="*60)
        print("TEST: Multiple Register Reads")
        print("="*60)
        
        registers = [
            (REG_OP_MODE, "OpMode"),
            (REG_FRF_MSB, "Freq MSB"),
            (REG_FRF_MID, "Freq MID"),
            (REG_FRF_LSB, "Freq LSB"),
            (REG_VERSION, "Version"),
        ]
        
        print("\nRegister dump:")
        for addr, name in registers:
            value = self.spi_read_register(addr)
            print(f"  0x{addr:02X} ({name:12s}): 0x{value:02X} ({value:3d}, {value:08b}b)")
            
    def cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up...")
        
        if self.cs_line:
            self.cs_line.set_value(1)  # CS high
            self.cs_line.release()
            print("  CS line released")
            
        if self.rst_line:
            self.rst_line.set_value(1)  # RST high
            self.rst_line.release()
            print("  RST line released")
            
        if self.irq_line:
            self.irq_line.release()
            print("  IRQ line released")
            
        if self.chip:
            self.chip.close()
            print("  GPIO chip closed")
            
        if self.spi:
            self.spi.close()
            print("  SPI closed")
            
def print_usage():
    """Print usage information"""
    print(__doc__)
    
def main():
    # Parse arguments
    test_mode = "basic"
    if len(sys.argv) > 1:
        if sys.argv[1] in ['--help', '-h', 'help']:
            print_usage()
            return 0
        test_mode = sys.argv[1]
        
    if test_mode not in ['basic', 'pins', 'reset', 'version', 'regs', 'all']:
        print(f"Unknown test mode: {test_mode}")
        print("Valid modes: basic, pins, reset, version, regs, all")
        return 1
        
    print("="*60)
    print("LoRa SPI Hardware Test")
    print("="*60)
    print(f"Test mode: {test_mode}")
    
    tester = LoRaSPITest()
    
    try:
        # Initialize
        tester.init_gpio()
        tester.init_spi()
        tester.reset_module()
        
        # Run tests
        if test_mode == 'basic' or test_mode == 'all':
            tester.test_basic()
            
        if test_mode == 'pins' or test_mode == 'all':
            tester.test_pins()
            
        if test_mode == 'reset' or test_mode == 'all':
            tester.test_reset()
            
        if test_mode == 'version' or test_mode == 'all':
            tester.test_version_repeated()
            
        if test_mode == 'regs' or test_mode == 'all':
            tester.test_registers()
            
        print("\n" + "="*60)
        print("Test complete")
        print("="*60)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 0
        
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        tester.cleanup()
        
if __name__ == '__main__':
    sys.exit(main())

