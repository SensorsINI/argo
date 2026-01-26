# LP5814 Alternative RGBW LED Drivers

## Requirements
- **4-channel RGBW control** (or more)
- **I2C interface**
- **Different I2C address** (not 0x2c or 0x34)
- **Similar form factor** (small package)
- **Similar capabilities** (PWM dimming, individual channel control)

## Candidate Devices

### Option 1: KTD2027 (Kinetic Technologies) ⭐ BEST MATCH
**Specifications:**
- **Channels**: 4 RGBW (perfect match)
- **I2C Address**: Configurable via hardware pins
  - **No conflict with 0x34 or 0x2c** ✅
- **Package**: 8-pin UTDFN (1.5×1.5×0.5mm) - **Similar size to LP5814!**
- **Features**:
  - 192 current levels (0.125mA to 24mA per channel)
  - Individual channel control (brightness and on/off)
  - I2C interface
  - Individual registers for each LED channel
- **Voltage**: 2.7V to 5.5V
- **Pros**: 
  - ✅ **Exact channel count (4 channels)**
  - ✅ **Similar package size (8-pin)**
  - ✅ Configurable address (no conflict)
  - ✅ Similar form factor to LP5814
- **Cons**:
  - ❌ Different manufacturer (may need to verify availability)
  - ❌ Different register map (software changes needed)
  - ❌ Lower max current (24mA vs 51mA for LP5814)

### Option 2: TLC59116 (Texas Instruments)
**Specifications:**
- **Channels**: 16 (more than needed, but can use 4)
- **I2C Address**: Configurable via hardware pins (A0-A3)
  - Base address: 0x60-0x6F (depending on A0-A3)
  - **No conflict with 0x34 or 0x2c** ✅
- **Package**: 28-pin HTSSOP or 28-pin QFN
- **Features**:
  - 8-bit PWM (256 steps)
  - Constant current sink (5-120mA per channel)
  - FM+ I2C (up to 1MHz)
  - Individual channel control
- **Voltage**: 3.0V to 5.5V
- **Pros**: 
  - ✅ Configurable address (no conflict)
  - ✅ More channels available for future use
  - ✅ Well-documented
- **Cons**:
  - ❌ Larger package (28-pin vs 8-pin)
  - ❌ More channels than needed
  - ❌ Requires PCB redesign

### Option 3: PCA9632 (NXP)
**Specifications:**
- **Channels**: 4 (perfect match)
- **I2C Address**: Configurable via hardware pins (A0-A5)
  - Base address: 0x60-0x7F (depending on A0-A5)
  - **No conflict with 0x34 or 0x2c** ✅
- **Package**: 20-pin TSSOP or 20-pin HVQFN
- **Features**:
  - 8-bit PWM (256 steps)
  - Individual LED control
  - Group dimming/blinking
  - Standard I2C (100kHz/400kHz)
- **Voltage**: 2.3V to 5.5V
- **Pros**:
  - ✅ Exact channel count (4 channels)
  - ✅ Configurable address (no conflict)
  - ✅ Similar features to LP5814
- **Cons**:
  - ❌ Larger package (20-pin vs 8-pin)
  - ❌ Requires PCB redesign

### Option 3b: PCA9633 (NXP)
**Specifications:**
- **Channels**: 4 (perfect match)
- **I2C Address**: Configurable via hardware pins (A0-A5)
  - Base address: 0x60-0x7F (depending on A0-A5)
  - **No conflict with 0x34 or 0x2c** ✅
- **Package**: 20-pin TSSOP or 20-pin HVQFN
- **Features**:
  - 8-bit PWM (256 steps)
  - Individual LED control
  - Group dimming/blinking
  - Standard I2C (100kHz/400kHz)
- **Voltage**: 2.3V to 5.5V
- **Pros**:
  - ✅ Exact channel count (4 channels)
  - ✅ Configurable address (no conflict)
  - ✅ Similar features to LP5814
- **Cons**:
  - ❌ Larger package (20-pin vs 8-pin)
  - ❌ Requires PCB redesign

### Option 4: IS31FL3731 (Lumissil)
**Specifications:**
- **Channels**: 144 LEDs in matrix (overkill for RGBW)
- **I2C Address**: Configurable via AD pin
  - Addresses: 0x66, 0x77
  - **No conflict with 0x34 or 0x2c** ✅
- **Package**: 32-pin QFN or 28-pin SSOP
- **Features**:
  - 8-bit PWM (256 steps)
  - Matrix LED control
  - Audio modulation
- **Voltage**: 2.7V to 5.5V
- **Pros**:
  - ✅ Configurable address (no conflict)
- **Cons**:
  - ❌ Designed for LED matrices, not simple RGBW
  - ❌ Much larger package
  - ❌ Overkill for 4-channel application

## Comparison Table

| Device | Channels | I2C Address | Package | Size | Address Conflict? |
|--------|----------|------------|---------|------|-------------------|
| **LP5814** (current) | 4 | 0x2c (0x34 broadcast) | 8-pin DSBGA | Small | ❌ Yes (0x34) |
| **KTD2027** ⭐ | 4 | Configurable | 8-pin UTDFN | Small | ✅ No |
| **PCA9632** | 4 | 0x60-0x7F (configurable) | 20-pin TSSOP/HVQFN | Medium | ✅ No |
| **PCA9633** | 4 | 0x60-0x7F (configurable) | 20-pin TSSOP/HVQFN | Medium | ✅ No |
| **TLC59116** | 16 | 0x60-0x6F (configurable) | 28-pin HTSSOP/QFN | Large | ✅ No |
| **IS31FL3731** | 144 (matrix) | 0x66, 0x77 | 32-pin QFN/28-pin SSOP | Large | ✅ No |

## Recommended Options

### Best Match: KTD2027 ⭐
**Why:**
- ✅ **Exact channel count (4 channels)**
- ✅ **Similar package size (8-pin UTDFN vs 8-pin DSBGA)**
- ✅ Configurable I2C address (no conflict)
- ✅ Similar form factor to LP5814
- ✅ Designed for RGBW applications

**Trade-offs:**
- ❌ Lower max current (24mA vs 51mA) - may be sufficient
- ❌ Different manufacturer (verify availability)
- ❌ Different register map (software changes needed)

**Next Steps:**
1. Verify KTD2027 datasheet for exact I2C address configuration
2. Check availability and pricing
3. Verify package compatibility (8-pin UTDFN vs 8-pin DSBGA)
4. Test if 24mA max current is sufficient for your LEDs

### Alternative: PCA9632 or PCA9633
**Why:**
- ✅ Exact channel count (4 channels)
- ✅ Configurable I2C address (no conflict)
- ✅ Similar features to LP5814
- ✅ Well-documented and widely available
- ✅ Standard I2C interface

**Trade-offs:**
- ❌ Larger package (20-pin vs 8-pin) - requires PCB redesign
- ❌ More pins to route

### Alternative: TLC59116
**Why:**
- ✅ Configurable I2C address (no conflict)
- ✅ More channels available for future expansion
- ✅ Well-documented

**Trade-offs:**
- ❌ Much larger package (28-pin)
- ❌ More channels than needed
- ❌ Requires PCB redesign

## Implementation Notes

### Address Configuration
All recommended devices use hardware address pins:
- **PCA9632/9633**: A0-A5 pins set address (0x60-0x7F range)
- **TLC59116**: A0-A3 pins set address (0x60-0x6F range)

**Example for PCA9632:**
- Set A0-A5 to configure address (e.g., 0x60, 0x61, etc.)
- Ensure address doesn't conflict with:
  - MAX11612: 0x34
  - LP5814: 0x2c
  - SHT45: 0x44
  - BNO085: 0x4a
  - Wind sensors: 0x21, 0x22, 0x23

**Safe address choices:**
- 0x60, 0x61, 0x62, 0x63, 0x64, 0x65 (all safe)
- 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F (all safe)

### Software Changes Required
1. Update I2C address constant
2. Review register map (different from LP5814)
3. Update initialization sequence
4. Test PWM/brightness control

## Next Steps

1. **Review datasheets** for PCA9632/9633 or TLC59116
2. **Verify package compatibility** with PCB design
3. **Check availability** and pricing
4. **Design PCB changes** for new package
5. **Update software** for new device

## References

- [PCA9632 Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9632.pdf)
- [PCA9633 Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9633.pdf)
- [TLC59116 Datasheet](https://www.ti.com/lit/ds/symlink/tlc59116.pdf)
