# Argo LED Status Display Guide

## Quick Reference for Hatch Cover

### LED Slot System (3-second cycle)
- **10 slots** of 0.3 seconds each (0.03s on + 0.27s off)
- **Pattern format**: `g.... .....` (green in slot 1, then off)
- **Color combinations**: 
  - `g` = green, `r` = red, `b` = blue
  - `y` = yellow (red+green), `p` = purple (red+blue), `c` = cyan (green+blue)

### Status Patterns
- **Basic alive**: `g.... .....` (1 green flash)
- **Nodes running**: `gg.... ....` (2 green flashes)
- **Recording active**: `ggg.... ...` (3 green flashes)
- **Single failure**: `gr.... ....` (1 green + 1 red)
- **Multiple failures**: `grr.... ...` (1 green + 2 red)
- **Critical battery**: `b.... .....` (1 blue flash)

### Failure Indicators (Red Flashes)
- **1 red flash**: Anemometer not connected/failed
- **2 red flashes**: + Network/WiFi connection lost
- **3 red flashes**: + GPS fix lost  
- **4 red flashes**: + LoRa shore connection lost

## Complete Status Examples

### Normal Operation
- **Pattern**: `gg.... ....` (2 green flashes for nodes running)

### Recording Active  
- **Pattern**: `ggg.... ...` (3 green flashes for recording)

### Anemometer Failed
- **Pattern**: `ggr.... ...` (2 green + 1 red for anemometer failure)

### Multiple Failures
- **Pattern**: `grrr.... ..` (1 green + 3 red for multiple failures)

### Critical Battery
- **Pattern**: `b.... .....` (1 blue flash for critical battery)

### Complex States
- **Recording + Failures**: `gggr.... ..` (3 green + 2 red)
- **Critical + Failures**: `brr.... ...` (1 blue + 2 red)

## Timing Details
- **Total cycle**: 3 seconds (10 slots × 0.3s each)
- **Each slot**: 0.03s LED on + 0.27s LED off (10% duty cycle)
- **Pattern execution**: Sequential slots with no gaps
- **Status monitoring**: Real-time via ROS2 topic subscriptions
- **Network check**: Every 5 seconds

## Troubleshooting
- **No green LED**: System not running or power issue
- **No status flashes**: Check if Argo service is running
- **Red flashes**: Count indicates number of system failures
- **Blue SOS**: Critical battery - plug in charger immediately
- **All LEDs off**: System shutdown or hardware failure

---
*This guide should be printed and attached to Argo's hatch cover for quick reference during operation.*
