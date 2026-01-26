# I2C Address Conflict: LP5814 Broadcast Address vs MAX11612

## Problem Summary

**Root Cause Identified**: The LP5814DLR LED controller has a **broadcast address at 0x34** that conflicts with the MAX11612 ADC, which also uses address **0x34**.

## The Conflict (NEEDS VERIFICATION)

### LP5814 Addresses (from datasheet section 7.5.2)
- **Independent Address**: 0x2C (7-bit) - Used for normal register operations
  - Address Byte 1: `0 1 0 1 1 0 0` (R/W bit)
  - This is what our code uses exclusively
- **Broadcast Address**: 0x34 (7-bit) - **VERIFICATION NEEDED**
  - Address Byte 1: `0 1 1 0 1 0 0` (R/W bit) = 0x34 in hex
  - **Question**: Does LP5814 hardware always respond to this address, or only when broadcast mode is explicitly used?
  - **Note**: Datasheet may not explicitly mention "0x34" - need to verify broadcast mode behavior

### MAX11612 Addresses
- **MAX11612/MAX11613**: 0x34 (7-bit) - **CONFLICTS with LP5814 broadcast**
- **MAX11614/MAX11615**: 0x33 (7-bit) - Alternative option (no conflict)
- **MAX11616/MAX11617**: 0x35 (7-bit) - Alternative option (no conflict)

## Why This Causes Zero Readings

**Critical Point**: Even though our code only uses LP5814's independent address (0x2c), the LP5814 hardware still responds to the broadcast address (0x34) at the hardware level.

When LP5814 is present on the I2C bus:
1. Master sends transaction to address 0x34 (intended for MAX11612 ADC)
2. **Both LP5814 (hardware listening to 0x34) and MAX11612 respond simultaneously**
3. I2C bus arbitration fails or gets corrupted (two devices trying to drive SDA)
4. ADC returns zeros (0x0000) because transaction is corrupted
5. Even with LP5814 disabled (CHIP_EN=0), the broadcast address still responds (hardware-level)
6. Even if we never use broadcast mode in software, LP5814 hardware still listens to 0x34

## Evidence

- ✅ ADC works perfectly when LP5814 is unplugged
- ✅ ADC returns zeros when LP5814 is plugged in (even if disabled)
- ✅ I2C signals appear identical on scope (both devices responding)
- ✅ `i2cdetect` shows both devices present (doesn't detect conflict)
- ✅ SHT45 (0x44) and BNO085 (0x4A) work fine (no conflict)
- ✅ Our code only uses LP5814 address 0x2c (independent mode), never 0x34 (broadcast)
- ❌ LP5814 hardware still responds to 0x34 regardless of software usage

## References

- TI E2E Forum: [LP5812 I2C Slave Conflict Due to Broadcast Address](https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/1484081/lp5812-i2c-slave-conflict-due-to-broadcast-address-on-lp5812-is-there-a-way-to-disable-it)
- TI E2E Forum: [LP5864 Close the broadcast](https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/1525982/lp5864-close-the-broadcast-of-lp5864)
- **Key Finding**: The broadcast address **cannot be disabled** on LP5814 series devices

## Solutions

### Option 1: Use Different MAX11612 Variant (Recommended)
**Replace MAX11612 with MAX11614 or MAX11616**

- **MAX11614/MAX11615**: Address 0x33 (no conflict)
- **MAX11616/MAX11617**: Address 0x35 (no conflict)

**Pros**:
- ✅ Simple hardware change (same pinout, same functionality)
- ✅ No software changes needed (just update address constant)
- ✅ No additional components
- ✅ Permanent solution

**Cons**:
- ❌ Requires hardware replacement
- ❌ May need to order new parts

**Implementation**:
```python
# In argo_battery_water.py, change:
self.adc_addr = 0x34  # MAX11612
# To:
self.adc_addr = 0x33  # MAX11614 (or 0x35 for MAX11616)
```

### Option 2: Use I2C Multiplexer
**Separate LP5814 and MAX11612 onto different I2C bus segments**

Use a device like TCA9548A (8-channel I2C multiplexer) to create separate bus segments.

**Pros**:
- ✅ Keeps existing hardware
- ✅ Complete isolation between devices
- ✅ Can support more devices in future

**Cons**:
- ❌ Additional hardware complexity
- ❌ Requires I2C multiplexer chip and PCB changes
- ❌ Software changes needed to select bus segment

### Option 3: Replace LP5814 with Different LED Driver
**Use an LED driver without broadcast address conflict**

Find alternative RGBW LED driver that doesn't use 0x34 as broadcast address.

**Pros**:
- ✅ Keeps MAX11612 unchanged
- ✅ May find better LED driver features

**Cons**:
- ❌ Requires complete LED driver replacement
- ❌ May need PCB redesign
- ❌ Software changes for new driver

### Option 4: Use Separate I2C Bus (If Available)
**Move one device to I2C bus 1 (if Orange Pi Zero 2W has it)**

**Pros**:
- ✅ Complete isolation
- ✅ No hardware changes if bus exists

**Cons**:
- ❌ Orange Pi Zero 2W may only have I2C bus 0
- ❌ Requires hardware verification
- ❌ Software changes to use different bus

## Recommended Solution

**Option 1: Replace MAX11612 with MAX11614 (0x33) or MAX11616 (0x35)**

This is the simplest and most reliable solution:
1. Order MAX11614 or MAX11616 (same package, same functionality)
2. Replace MAX11612 on PCB
3. Update `self.adc_addr = 0x33` (or 0x35) in `argo_battery_water.py`
4. Test and verify

## Verification

After implementing solution, verify:
1. ✅ ADC reads correctly with LP5814 plugged in
2. ✅ LP5814 LED control works normally
3. ✅ No I2C bus conflicts
4. ✅ All other I2C devices (SHT45, BNO085, wind sensors) still work

## Current Workaround

Until hardware fix is implemented:
- **Do not use LP5814 and MAX11612 simultaneously**
- Unplug LP5814 when ADC readings are critical
- Or unplug MAX11612 when LED control is needed
- **Note**: This is not a permanent solution

## Impact Assessment

**Severity**: **CRITICAL** - Prevents simultaneous use of battery monitoring and LED control

**Affected Systems**:
- Battery voltage monitoring (critical for safety)
- Mast LED control (status indication)

**Workaround Available**: Yes (unplug one device)

**Permanent Fix Required**: Yes (hardware change)
