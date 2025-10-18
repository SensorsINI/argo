#!/usr/bin/env python3
"""
Test different SPI modes to diagnose LoRa module communication
"""
import time
import sys
import gpiod
import spidev

LORA_SEL_LINE = 266   # CS
LORA_RST_LINE = 256   # Reset

# SX1276 Version register
REG_VERSION = 0x42

def test_spi_mode(spi, mode, speed):
    """Test reading version register with specific SPI mode and speed"""
    spi.mode = mode
    spi.max_speed_hz = speed
    
    # Read version register
    response = spi.xfer2([REG_VERSION & 0x7F, 0x00])
    return response[1]

def main():
    print("=" * 60)
    print("SPI Mode Testing for LoRa Module")
    print("=" * 60)
    
    gpio_chip = gpiod.Chip("/dev/gpiochip0")
    
    # Setup CS and Reset
    lora_sel = gpio_chip.get_line(LORA_SEL_LINE)
    lora_sel.request(consumer="spi_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
    
    lora_rst = gpio_chip.get_line(LORA_RST_LINE)
    lora_rst.request(consumer="spi_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
    
    # Reset module
    print("\nResetting module...")
    lora_rst.set_value(0)
    time.sleep(0.01)
    lora_rst.set_value(1)
    time.sleep(0.15)  # Wait for crystal
    print("Reset complete\n")
    
    # Open SPI
    spi = spidev.SpiDev()
    spi.open(1, 0)
    
    # Test different modes and speeds
    modes = [0, 1, 2, 3]  # SPI modes
    speeds = [100000, 500000, 1000000]  # 100kHz, 500kHz, 1MHz
    
    print("Testing SPI Modes and Speeds:")
    print("-" * 60)
    print(f"{'Mode':<8} {'Speed':<12} {'Version':<12} {'Result'}")
    print("-" * 60)
    
    for mode in modes:
        for speed in speeds:
            lora_sel.set_value(0)  # CS low
            version = test_spi_mode(spi, mode, speed)
            lora_sel.set_value(1)  # CS high
            time.sleep(0.01)
            
            result = "✓ WORKING!" if version == 0x12 else ("? Unexpected" if version != 0x00 else "✗ No response")
            speed_str = f"{speed//1000}kHz"
            print(f"Mode {mode}   {speed_str:<12} 0x{version:02X}         {result}")
    
    print("-" * 60)
    print("\nExpected version: 0x12 for SX1276/SX1278")
    print("If all show 0x00: Module not responding (power/wiring issue)")
    print("If any show 0x12: Found working configuration!")
    
    # Cleanup
    lora_sel.set_value(1)
    lora_sel.release()
    lora_rst.set_value(1)
    lora_rst.release()
    gpio_chip.close()
    spi.close()

if __name__ == "__main__":
    main()

