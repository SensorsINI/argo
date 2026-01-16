# Argo Wind Sensor - Revision 1

## Overview

This is the first revision (Rev 1) of the Argo Wind Sensor PCB. The board is designed to measure wind speed and direction for the Argo sailboat. 

It is based on Sensiorion reference design (since removed from sensorion because the page could not be migrated). Sensirion kindly supplied the step file as a starting point for the current design. The current design is combined with a mast mounting part that clamps to the mast backstay carbon fiber part.

The PCB design includes 3 SDP32 differential pressure sensors and 
an RGBW LED controller with RGBW LED. Sensors and LED controller are on the Argo I2C bus and are connected to Argo's computer via the 4-wire cable through the Molex SL-series SMD header. The mast mount part includes a set screw to provide strain relief for the SMD Molex header.

The PCB was designed in Kicad 9. The housing was designed in FreeCAD 1.1rc2.

The PCB was fabricated by JLCPCB.

See the argo ROS2 nodes anem.py and mastled.py for sensing and control of the wind sensor.

## Specifications

### Electrical
- **Supply Voltage**: 5V
- **Operating Current**: ~160mA max (3 LEDS, each max 50mA)~
- **Communication Interface**: I2C

### Physical
- **Board Dimensions**: 20x20mm
- **Number of Layers**: 2
- **Surface Finish**: ENIG

## Design Files

This directory contains:
- Schematic files (`.kicad_sch` Kicad Version 9)
- PCB layout files (`.kicad_pcb` )
- 3D models (if applicable)

## Bill of Materials (BOM)

The BOM is generated from Kicad
Parts include datasheet links and Digikey or Farnell part links.

## Manufacturing

### Gerber Files
Gerber files are generated from kicad in the `gerbers/` directory. These can be sent directly to a PCB manufacturer.

### Fabrication Notes
- Minimum trace width: 0.2mm
- Minimum via size: 0.3mm

## Known Issues

- None currently documented for Rev 1

## Revision History

### Rev 1 (Initial Release)
- Initial PCB design
- Date: TBD

### Rev 0 (initial design done by ETH semester student Andrea Mock, ca 2019)

## Contact

For questions or issues related to this PCB design, please contact Tobi Delbruck.

## License

LGPLv3


