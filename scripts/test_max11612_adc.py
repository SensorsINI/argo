#!/usr/bin/env python3
# Standalone diagnostic script for MAX11612 ADC at I2C address 0x34
# Tests I2C communication, setup, and reading

import sys
import time

# Try to import smbus2, fall back to smbus
try:
    import smbus2
    from smbus2 import i2c_msg
    USE_SMBUS2 = True
except ImportError:
    try:
        import smbus
        USE_SMBUS2 = False
    except ImportError:
        print("ERROR: Neither smbus2 nor smbus is available")
        print("Install with: sudo pip3 install smbus2")
        sys.exit(1)

I2C_BUS = 0
ADC_ADDR = 0x34

def build_setup(reg, sel, clk, bip_uni, rst, x):
    """Build MAX11612 setup byte."""
    return ((reg & 1) << 7) | ((sel & 0b111) << 4) | ((clk & 1) << 3) | ((bip_uni & 1) << 2) | ((rst & 1) << 1) | (x & 1)

def build_config(reg, scan, cs, sgl_dif):
    """Build MAX11612 configuration byte."""
    return ((reg & 1) << 7) | ((scan & 0b11) << 5) | ((cs & 0b1111) << 1) | (sgl_dif & 1)

def i2c_write(addr, byte_val):
    """Write a single byte to I2C device."""
    if USE_SMBUS2:
        msg = i2c_msg.write(addr, bytes([byte_val & 0xFF]))
        with smbus2.SMBus(I2C_BUS) as bus:
            bus.i2c_rdwr(msg)
    else:
        bus = smbus.SMBus(I2C_BUS)
        bus.write_byte(addr, byte_val & 0xFF)
        bus.close()
    print(f"  Write: 0x{byte_val:02x} to address 0x{addr:02x}")

def i2c_read(addr, n):
    """Read n bytes from I2C device."""
    if USE_SMBUS2:
        r = i2c_msg.read(addr, n)
        with smbus2.SMBus(I2C_BUS) as bus:
            bus.i2c_rdwr(r)
        data = list(r)
    else:
        bus = smbus.SMBus(I2C_BUS)
        data = []
        for i in range(n):
            data.append(bus.read_byte(addr))
        bus.close()
    print(f"  Read:  {[hex(b) for b in data]} from address 0x{addr:02x}")
    return data

def i2c_write_read(addr, write_byte, n):
    """Write then read in one transaction."""
    if USE_SMBUS2:
        w = i2c_msg.write(addr, bytes([write_byte & 0xFF]))
        r = i2c_msg.read(addr, n)
        with smbus2.SMBus(I2C_BUS) as bus:
            bus.i2c_rdwr(w, r)
        data = list(r)
    else:
        # Standard smbus doesn't support combined write-read, so do separately
        bus = smbus.SMBus(I2C_BUS)
        bus.write_byte(addr, write_byte & 0xFF)
        time.sleep(0.001)  # Small delay
        data = []
        for i in range(n):
            data.append(bus.read_byte(addr))
        bus.close()
    print(f"  Write-Read: Write 0x{write_byte:02x}, Read {[hex(b) for b in data]}")
    return data

def test_device_detection():
    """Test if device responds to I2C detection."""
    print("\n" + "="*60)
    print("Test 1: Device Detection")
    print("="*60)
    try:
        # Try a simple read to see if device ACKs
        print("Attempting to read 1 byte from device...")
        data = i2c_read(ADC_ADDR, 1)
        print(f"✅ Device responded: {data}")
        return True
    except Exception as e:
        print(f"❌ Device did not respond: {e}")
        return False

def test_setup_register():
    """Test writing to setup register."""
    print("\n" + "="*60)
    print("Test 2: Setup Register Write")
    print("="*60)
    try:
        # Setup byte: reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0
        # This matches argo_battery_water.py initialization
        setup_byte = build_setup(reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0)
        print(f"Setup byte: 0x{setup_byte:02x} (binary: {bin(setup_byte)})")
        print("  reg=1, sel=0b101, clk=0, bip_uni=0, rst=1, x=0")
        i2c_write(ADC_ADDR, setup_byte)
        print("✅ Setup register write successful")
        time.sleep(0.01)  # Small delay after setup
        return True
    except Exception as e:
        print(f"❌ Setup register write failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_config_and_read():
    """Test configuration and reading ADC channel."""
    print("\n" + "="*60)
    print("Test 3: Configuration and Read")
    print("="*60)
    try:
        # Config byte: reg=0, scan=0b01, cs=0, sgl_dif=1
        # This configures single-ended conversion of channel 0
        config_byte = build_config(reg=0, scan=0b01, cs=0, sgl_dif=1)
        print(f"Config byte: 0x{config_byte:02x} (binary: {bin(config_byte)})")
        print("  reg=0, scan=0b01 (convert selected input), cs=0 (channel 0), sgl_dif=1 (single-ended)")
        
        # Write config
        i2c_write(ADC_ADDR, config_byte)
        time.sleep(0.001)  # Settle delay
        
        # Try reading 2 bytes (ADC result)
        print("\nAttempting to read 2 bytes (ADC result)...")
        data = i2c_read(ADC_ADDR, 2)
        
        if len(data) == 2:
            # Decode ADC result
            code = ((data[0] & 0x0F) << 8) | data[1]
            print(f"✅ Read successful: Raw ADC code = {code} (0x{code:04x})")
            print(f"   Data bytes: [0x{data[0]:02x}, 0x{data[1]:02x}]")
            return True
        else:
            print(f"❌ Unexpected data length: {len(data)} bytes")
            return False
            
    except Exception as e:
        print(f"❌ Configuration and read failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_write_read_transaction():
    """Test write-then-read in single I2C transaction."""
    print("\n" + "="*60)
    print("Test 4: Write-Read Transaction (Single I2C Transaction)")
    print("="*60)
    try:
        config_byte = build_config(reg=0, scan=0b01, cs=0, sgl_dif=1)
        print(f"Config byte: 0x{config_byte:02x}")
        
        # Write config and read result in one transaction
        data = i2c_write_read(ADC_ADDR, config_byte, 2)
        
        if len(data) == 2:
            code = ((data[0] & 0x0F) << 8) | data[1]
            print(f"✅ Write-read transaction successful: Raw ADC code = {code} (0x{code:04x})")
            return True
        else:
            print(f"❌ Unexpected data length: {len(data)} bytes")
            return False
            
    except Exception as e:
        print(f"❌ Write-read transaction failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_multiple_channels():
    """Test reading multiple ADC channels."""
    print("\n" + "="*60)
    print("Test 5: Multiple Channel Reads")
    print("="*60)
    try:
        channels = [0, 1, 2, 3]
        results = {}
        
        for ch in channels:
            config_byte = build_config(reg=0, scan=0b01, cs=ch, sgl_dif=1)
            print(f"\nChannel {ch}:")
            i2c_write(ADC_ADDR, config_byte)
            time.sleep(0.001)
            data = i2c_read(ADC_ADDR, 2)
            
            if len(data) == 2:
                code = ((data[0] & 0x0F) << 8) | data[1]
                results[ch] = code
                print(f"  ✅ Channel {ch}: {code} (0x{code:04x})")
            else:
                print(f"  ❌ Channel {ch}: Failed to read")
                results[ch] = None
        
        print(f"\nChannel results: {results}")
        return all(v is not None for v in results.values())
        
    except Exception as e:
        print(f"❌ Multiple channel test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_scan_mode():
    """Test scan mode (convert selected input multiple times)."""
    print("\n" + "="*60)
    print("Test 6: Scan Mode (Multiple Conversions)")
    print("="*60)
    try:
        # Config: scan=0b01 means convert selected input multiple times
        config_byte = build_config(reg=0, scan=0b01, cs=0, sgl_dif=1)
        print(f"Config byte: 0x{config_byte:02x} (scan mode)")
        
        i2c_write(ADC_ADDR, config_byte)
        time.sleep(0.001)
        
        # Read multiple times (like argo_battery_water.py does)
        print("\nReading 8 conversions (like argo_battery_water.py):")
        codes = []
        for i in range(8):
            data = i2c_read(ADC_ADDR, 2)
            if len(data) == 2:
                code = ((data[0] & 0x0F) << 8) | data[1]
                codes.append(code)
                print(f"  Conversion {i+1}: {code} (0x{code:04x})")
            else:
                print(f"  Conversion {i+1}: Failed")
        
        if codes:
            avg = sum(codes) // len(codes)
            print(f"\n✅ Scan mode successful: Average = {avg} (0x{avg:04x})")
            print(f"   Range: {min(codes)} - {max(codes)}")
            return True
        else:
            print("❌ Scan mode failed: No successful reads")
            return False
            
    except Exception as e:
        print(f"❌ Scan mode test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all diagnostic tests."""
    print("="*60)
    print("MAX11612 ADC Diagnostic Tool")
    print(f"I2C Bus: {I2C_BUS}, Address: 0x{ADC_ADDR:02x}")
    print("="*60)
    
    results = {}
    
    # Test 1: Device detection
    results['detection'] = test_device_detection()
    
    if not results['detection']:
        print("\n❌ Device not responding. Check:")
        print("   - I2C bus is enabled")
        print("   - Device is powered")
        print("   - I2C address is correct (0x34)")
        print("   - Pull-up resistors are present")
        print("   - No bus lockup (SDA/SCL not stuck)")
        return 1
    
    # Test 2: Setup register
    results['setup'] = test_setup_register()
    
    if not results['setup']:
        print("\n❌ Setup register write failed. Check I2C communication.")
        return 1
    
    # Test 3: Config and read
    results['config_read'] = test_config_and_read()
    
    # Test 4: Write-read transaction
    results['write_read'] = test_write_read_transaction()
    
    # Test 5: Multiple channels
    results['multiple_channels'] = test_multiple_channels()
    
    # Test 6: Scan mode
    results['scan_mode'] = test_scan_mode()
    
    # Summary
    print("\n" + "="*60)
    print("Test Summary")
    print("="*60)
    for test_name, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:20s}: {status}")
    
    all_passed = all(results.values())
    if all_passed:
        print("\n✅ All tests passed! ADC communication is working.")
        return 0
    else:
        print("\n❌ Some tests failed. Review output above for details.")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
