#!/usr/bin/env python3
"""
Simple LoRa SPI Test Script
===========================
Exercises all SPI and GPIO pins for oscilloscope verification.

Toggles:
- LORA_RST (Pin 29, PI0) - Reset pin
- LORA_SEL (Pin 27, PI10) - Chip Select
- SPI MOSI/MISO/SCLK - Data lines
- LORA_IRQ (Pin 31, PI15) - Read interrupt pin state

Press Ctrl+C to stop.
"""

import time
import sys

try:
    import gpiod
except ImportError:
    print("ERROR: gpiod library not available")
    print("Install with: pip3 install gpiod")
    sys.exit(1)

try:
    import spidev
except ImportError:
    print("ERROR: spidev library not available")
    print("Install with: pip3 install spidev")
    sys.exit(1)

# GPIO line numbers (same as lora.py)
LORA_SEL_LINE = 266   # PI10 (Pin 27) - Chip Select
LORA_RST_LINE = 256   # PI0 (Pin 29) - Reset  
LORA_IRQ_LINE = 271   # PI15 (Pin 31) - Interrupt/DIO0

# SX1276 Register addresses
REG_VERSION = 0x42
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FIFO = 0x00

def main():
    print("=" * 60)
    print("LoRa SPI Test - Pin Toggle Test")
    print("=" * 60)
    print("\nPin Assignments:")
    print("  Pin 27 (PI10) = LORA_SEL (CS) - Should toggle during SPI")
    print("  Pin 29 (PI0)  = LORA_RST - Will be held HIGH (inactive)")
    print("  Pin 31 (PI15) = LORA_IRQ - Input (read state)")
    print("  Pin 19 (PH7)  = MOSI - SPI data out")
    print("  Pin 21 (PH8)  = MISO - SPI data in")
    print("  Pin 23 (PH6)  = SCLK - SPI clock")
    print("\nPress Ctrl+C to stop\n")
    
    gpio_chip = None
    lora_sel_line = None
    lora_rst_line = None
    lora_irq_line = None
    spi = None
    
    try:
        # Initialize GPIO
        print("[1] Opening GPIO chip...")
        gpio_chip = gpiod.Chip("/dev/gpiochip0")
        
        # Configure LORA_SEL (Chip Select) as output, initially high (inactive)
        print("[2] Configuring LORA_SEL (CS) as output...")
        lora_sel_line = gpio_chip.get_line(LORA_SEL_LINE)
        lora_sel_line.request(consumer="lora_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        print(f"    LORA_SEL (line {LORA_SEL_LINE}) = HIGH (CS inactive)")
        
        # Configure LORA_RST (Reset) as output, initially high (inactive)
        print("[3] Configuring LORA_RST as output...")
        lora_rst_line = gpio_chip.get_line(LORA_RST_LINE)
        lora_rst_line.request(consumer="lora_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        print(f"    LORA_RST (line {LORA_RST_LINE}) = HIGH (reset inactive)")
        
        # Configure LORA_IRQ (Interrupt) as input
        print("[4] Configuring LORA_IRQ as input...")
        lora_irq_line = gpio_chip.get_line(LORA_IRQ_LINE)
        lora_irq_line.request(consumer="lora_test", type=gpiod.LINE_REQ_DIR_IN)
        print(f"    LORA_IRQ (line {LORA_IRQ_LINE}) configured as input")
        
        # Initialize SPI
        print("[5] Opening SPI bus 1, device 0...")
        spi = spidev.SpiDev()
        spi.open(1, 0)  # SPI1, CS0
        spi.max_speed_hz = 100000  # 100 kHz for easy scope viewing
        spi.mode = 0  # SPI Mode 0 (CPOL=0, CPHA=0)
        spi.no_cs = True  # Disable automatic CS - we control it manually via GPIO
        print(f"    SPI initialized: 100 kHz, Mode 0, Manual CS control")
        
        print("\n" + "=" * 60)
        print("Starting test loop - watch with oscilloscope!")
        print("=" * 60 + "\n")
        
        loop_count = 0
        while True:
            loop_count += 1
            print(f"\n--- Loop {loop_count} ---")
            
            # Test 1: Reset pulse (should see a LOW pulse on scope)
            print("  [A] Toggling RESET (should see LOW pulse)...")
            lora_rst_line.set_value(0)  # Reset active (low)
            time.sleep(0.01)  # 10ms low
            lora_rst_line.set_value(1)  # Reset inactive (high)
            print("      RESET: LOW -> HIGH, waiting for crystal to stabilize...")
            time.sleep(0.1)  # Wait 100ms for crystal oscillator to stabilize
            print("      Wait complete (100ms)")
            
            # Test 2: Read Version register (0x42)
            print("  [B] Reading Version register (0x42)...")
            lora_sel_line.set_value(0)  # CS active (low) - BEFORE transaction
            response = spi.xfer2([REG_VERSION & 0x7F, 0x00])  # Read register while CS is LOW
            lora_sel_line.set_value(1)  # CS inactive (high) - AFTER transaction
            version = response[1]
            print(f"      CS: LOW during xfer, then HIGH. Version = 0x{version:02X}")
            
            # Test 3: Read Operating Mode register (0x01)
            print("  [C] Reading OpMode register (0x01)...")
            lora_sel_line.set_value(0)
            response = spi.xfer2([REG_OP_MODE & 0x7F, 0x00])
            lora_sel_line.set_value(1)
            opmode = response[1]
            print(f"      CS: LOW during xfer, then HIGH. OpMode = 0x{opmode:02X}")
            
            # Test 4: Read Frequency register (0x06)
            print("  [D] Reading Frequency MSB register (0x06)...")
            lora_sel_line.set_value(0)
            response = spi.xfer2([REG_FRF_MSB & 0x7F, 0x00])
            lora_sel_line.set_value(1)
            freq_msb = response[1]
            print(f"      CS: LOW during xfer, then HIGH. Freq MSB = 0x{freq_msb:02X}")
            
            # Test 5: Write test pattern to FIFO (0x00)
            print("  [E] Writing test pattern to FIFO...")
            test_pattern = [0xAA, 0x55, 0xFF, 0x00, 0xDE, 0xAD, 0xBE, 0xEF]
            lora_sel_line.set_value(0)
            spi.xfer2([REG_FIFO | 0x80] + test_pattern)  # Write to FIFO
            lora_sel_line.set_value(1)
            print(f"      CS: LOW during xfer, then HIGH. Wrote: {' '.join(f'{b:02X}' for b in test_pattern)}")
            
            # Test 6: Read IRQ line state
            irq_state = lora_irq_line.get_value()
            print(f"  [F] IRQ line state: {'HIGH' if irq_state else 'LOW'}")
            
            # Test 7: Burst SPI traffic for scope triggering
            print("  [G] Burst SPI traffic (10 transactions)...")
            for i in range(10):
                lora_sel_line.set_value(0)
                spi.xfer2([0x55, 0xAA])  # Alternating pattern
                lora_sel_line.set_value(1)
                time.sleep(0.005)  # 5ms between bursts
            print("      Burst complete")
            
            print(f"\n  Loop {loop_count} complete. Waiting 1 second...\n")
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("Test stopped by user")
        print("=" * 60)
        
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        print("\nCleaning up...")
        if lora_sel_line:
            lora_sel_line.set_value(1)  # CS high (inactive)
            lora_sel_line.release()
            print("  LORA_SEL released (set HIGH)")
        if lora_rst_line:
            lora_rst_line.set_value(1)  # Reset high (inactive)
            lora_rst_line.release()
            print("  LORA_RST released (set HIGH)")
        if lora_irq_line:
            lora_irq_line.release()
            print("  LORA_IRQ released")
        if gpio_chip:
            gpio_chip.close()
            print("  GPIO chip closed")
        if spi:
            spi.close()
            print("  SPI closed")
        print("\nDone!")

if __name__ == "__main__":
    main()

