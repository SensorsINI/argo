# PCB Design Files

This directory contains all the printed circuit board (PCB) design files, libraries, and documentation for the Argo autonomous sailboat project.

The design was done with KiCad version 9 stable.  
**It is important to use the stable KiCad version, not the development release, since KiCad will not open files from later versions!**  
- [Download KiCad stable here](https://www.kicad.org/download/)

## See also
Updated wind sensor design in [WindSensor](WindSensor).
## Current PCB
The current stable PCB [Argo PCB rev4](argo-v9-stable/argo-v9-stable-rev4) that will be tested at CapoCaccia 2026 is shown below, plugged into the OrangePiZero 2W. The only missing board is the GPS module, which sits on the standoffs. Note how the Ra-01 LORA module is soldered directly to the PCB to lower it to fit under the rudder winch. The white JST headers connect to water sensor, hatch power button with RGB LED, and 2S LiPo battery charger/balance cable. The large cables connect power from the LiPo battery, which sits alongside the keel/mast hull insert. The two black headers connect to the IMU on the rudder/winch servo chassis, and the wind sensor at the top of the mast. The board has a big white dual-coil power relay that completely cuts power when the boat is powered off.  The upper small black chip at lower left are the ADC that converts battery voltage, sail winch and rudder current, and water sensor voltage. The smaller chip below this is the humidity and PCB temperature sensor. The 4 large SMD caps at top left, below the inductor, are just to the left of the LiPo battery charger chip. The USB power input for charging is at lower right; it connects to the waterproof hatch USB port with a flex cable. Note how all cables exit on right because fore is the only available direction for connections; the PCB sits nestled up against the rudder insert and partly below the rudder/winch tray.

<img src="argo_v9_stable_rev4_pcb.jpg" width="800" alt="Argo v9 stable rev4 PCB with Orange Pi Zero 2W" />

Outstanding bugs: The rev4 PCB requires soldering a single 10k resistor to ground from the buzzer testpoint to prevent buzzer activation during boot. This resistor should be added to rev 5.

## Directory Structure

### `argo-v9-stable/`
The main PCB design files for the Argo v9 stable revision:
- **Schematic and PCB files**: KiCad project files for the main control board
- **3d-models/**: 3D model files for mechanical integration
- **Argo footprint Library.pretty/**: Custom KiCad footprint library with hand-solderable components
- **argo-v9-stable-rev4/**: Current production (rev4) manufacturing files (almopst fully functional, no modifications required except 10k pulldown on BUZZER and direct LORA soldering to PCB)
- **bom/**: Bill of Materials (BOM) files including interactive HTML BOM

### `orange-pi/`
Orange Pi Zero 2W integration files:
- **orange_pi_zero_2w.kicad_sym**: KiCad symbol for Orange Pi Zero 2W
- **orange_pi_zero_2w_pins.lib**: Pin definition library
- **3d/**: 3D models for Orange Pi Zero 2W
- **H616_Datasheet_V1.0_cleaned.pdf**: Allwinner H616 SoC datasheet
- **OPi_ZERO_2W_SCH.pdf**: Orange Pi Zero 2W schematic
- **OPi_ZERO_2W_V1_1_1A pcb layout.pdf**: PCB layout reference

## Key Components

### Main Control Board (Argo v9)
- **Microcontroller**: Orange Pi Zero 2W (Allwinner H618)
- **Power Management**: USB-C PD controller (AP33771C), power switching
- **Sensors**: 
  - GPS: u-blox NEO-M9N
  - IMU: Sparkfun BNO085
  - Wind: 3x Sensirion SDP3x differential pressure; see WindSensor folder
  - Environment: SHT45 temperature/humidity
  - Battery/Water: MAX11612 ADC
- **Interfaces**: I2C, UART, PWM capture/output, USB-C

### Design Philosophy
- **Hand-solderable components**: All footprints designed for manual assembly
- **Robust power management**: Dual coil relay, USB LiPo charging.
- **Robust I2C**: The wind sensor using different I2C bus than internal components, in case bus is shorted by water.


### Assembly Notes
- All components are hand-solderable. SMD pads have been made extra long to allow heat application and for excess solder.


## Development Tools

### KiCad Version
- **Required**: KiCad 9.0 or later
- **Footprint Library**: Custom "Argo footprint Library" with hand-solderable components

### 3D Models
- **Mechanical Integration**: 3D models have been built for all PCB components for enclosure design
- **Assembly Verification**: Use FreeCAD 1.1+ for clearance checking.

## Documentation


### Testing and Validation
- **In-Circuit Testing**: Test points provided for key signals

## Related documentation

- **[Main project README](../README.md)** — Overall system documentation
- **[Software nodes](../nodes/README.md)** — ROS2 nodes on the Orange Pi
- **[Launch](../launch/README.md)** — Startup scripts and lifecycle management
- **[Power control](../power_control/README.md)** — Battery, button, shutdown, and services; **[detailed reference](../power_control/ARGO_POWER_CONTROL_README.md)** — GPIO, hooks, and service behavior
- **[Wind sensor PCB](WindSensor/README.md)** — SDP3x wind sensor board design
- **[Orange Pi KiCad](orange-pi/README.md)** — KiCad symbol, footprint, and 3D models for the OPI header
- **[PCB v4 → v5 migration](../docs/README-argo-pcb-v4-v5-migration.md)** — Planned pin and bus changes for the next revision
- **[I2C buses](../docs/README-i2c.md)** — I2C configuration and recovery on the board
- **[Pin and bus reference](../docs/README-pins-i2c.md)** — Orange Pi 40-pin header: power, I2C, SPI, UART, PWM
