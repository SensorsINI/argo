# Argo Power Control System

This system provides intelligent power control for the Argo project using external relays and GPIO pins, implementing safe shutdown procedures and power button monitoring.

## Hardware Configuration

### Pin Assignments
- **PI3 (Pin 40)**: `!POW` - Open drain output to control power relay
- **PI9 (Pin 28)**: `!POW_BUT` - Input from power button (external pullup required)
- **PH4 (Pin 18)**: Green LED in power button
- **PI1 (Pin 12)**: Blue LED in power button
- **Red LED**: Directly connected to power button (not controlled by GPIO)

### Power Circuit Design
The system uses two PFETs to control the power relay:
1. **Direct PFET**: Connected to power button for immediate power-on
2. **GPIO PFET**: Connected to `!POW` pin for software-controlled power management

When `!POW` is pulled low (open drain), the relay is energized, providing power to the system.

## Software Components

### 1. Power Control Daemon (`argo_power_control.py`)
- Monitors power button press duration
- Controls LED indicators
- Manages power relay state
- Implements safe shutdown sequence
- Command-line interface with test mode
- Configurable button press threshold

### 2. Systemd Service (`argo-power-control.service`)
- Runs power control daemon as system service
- Ensures automatic startup and restart on failure
- Provides proper logging and error handling

### 3. Configuration File (`argo_power_control_config.yaml`)
- Centralized configuration for all settings
- Easy customization of thresholds and behavior

## Installation

### Prerequisites
```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y python3-gpiod
```

### Manual Installation
1. **Install systemd service**:
   ```bash
   sudo cp ~/argo/launch/argo-power-control.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable argo-power-control
   ```

2. **Make script executable**:
   ```bash
   sudo chmod +x ~/argo/launch/argo_power_control.py
   ```

3. **Start the service**:
   ```bash
   sudo systemctl start argo-power-control
   ```

## Usage

### Command Line Interface
```bash
# Show help
python3 ~/argo/launch/argo_power_control.py --help

# Normal operation
sudo ~/argo/launch/argo_power_control.py

# Test mode (safe - no shutdown/power control)
sudo ~/argo/launch/argo_power_control.py --test-mode

# Custom button press threshold
sudo ~/argo/launch/argo_power_control.py --threshold 2.0
```

### Power Button Behavior
- **Short press** (< threshold): No action
- **Long press** (≥ threshold): Initiate shutdown sequence
- **Default threshold**: 1.0 seconds (configurable)

### LED Indicators
- **Green LED**: System running (solid on)
- **Blue LED**: 
  - Fast blink during button press warning
  - Rapid blink during shutdown sequence
- **Red LED**: Directly connected to power button

### Service Management
```bash
# Check service status
sudo systemctl status argo-power-control

# View logs
sudo journalctl -u argo-power-control -f

# Restart service
sudo systemctl restart argo-power-control

# Stop service
sudo systemctl stop argo-power-control
```

## Testing and Verification

### 1. Test Mode (Recommended)
```bash
# Safe testing with all functionality except actual shutdown
sudo ~/argo/launch/argo_power_control.py --test-mode
```

### 2. Verify GPIO Pin Assignments
```bash
# Check GPIO line numbers
sudo gpioinfo | grep -E "(259|265|228|257)"
```

### 3. Manual GPIO Testing
```bash
# Check available GPIO lines
sudo gpioinfo

# Test specific pins manually
sudo gpioset gpiochip0 259=1  # Set power relay (PI3)
sudo gpioget gpiochip0 265    # Read power button (PI9)
sudo gpioset gpiochip0 257=1  # Set blue LED (PI1)
sudo gpioset gpiochip0 228=1  # Set green LED (PH4)
```

## Troubleshooting

### Common Issues

1. **GPIO Permission Denied**
   - Ensure running as root: `sudo ~/argo/launch/argo_power_control.py`
   - Check GPIO chip permissions

2. **Power Button Not Responding**
   - Verify external pullup resistor is connected
   - Check GPIO line number (should be 265 for PI9)
   - Test with `sudo gpioget gpiochip0 265`

3. **LED Not Working**
   - Check GPIO line numbers (257 for PI1, 228 for PH4)
   - Verify LED polarity and current limiting resistor
   - Test with `sudo gpioset gpiochip0 257=1`

4. **Relay Not Controlling Power**
   - Verify open drain configuration
   - Check PFET connections
   - Test relay operation manually
   - Use test mode first: `sudo ~/argo/launch/argo_power_control.py --test-mode`

### Log Analysis
```bash
# View power control logs
sudo journalctl -u argo-power-control --since "1 hour ago"

# Check system logs for GPIO errors
dmesg | grep -i gpio

# Monitor GPIO state changes
sudo gpiomon gpiochip0 233  # Monitor power button
```

## Safety Considerations

1. **Power Relay Control**: The system de-energizes the relay during shutdown, cutting power completely
2. **Open Drain Configuration**: Prevents damage if multiple sources try to control the relay
3. **External Pullup**: Required for power button to prevent floating input
4. **Graceful Shutdown**: System performs proper shutdown before cutting power

## Customization

### Modify Button Press Threshold
```bash
# Command line
sudo ~/argo/launch/argo_power_control.py --threshold 2.0

# Or edit argo_power_control.py
self.SHUTDOWN_THRESHOLD = 2.0  # Change to 2 seconds
```

### Adjust LED Blink Patterns
Edit `argo_power_control.py`:
```python
self.LED_BLINK_FAST = 0.05     # Faster blink
self.LED_BLINK_SLOW = 1.0      # Slower blink
```

### Configuration File
Edit `argo_power_control_config.yaml` for centralized configuration:
```yaml
power_button:
  shutdown_threshold: 2.0  # 2 second threshold
leds:
  blink_fast: 0.05         # Fast blink rate
  blink_slow: 1.0          # Slow blink rate
```

### GPIO Line Numbers
The system uses these verified GPIO line numbers:
- PI3 (Power Relay): Line 259
- PI9 (Power Button): Line 265  
- PH4 (Green LED): Line 228
- PI1 (Blue LED): Line 257

## System Architecture

### Why No Device Tree Overlay?
This system uses a **service-based approach** instead of kernel-level device tree overlays because:

- **Simpler**: No kernel modules or device tree complexity
- **More Flexible**: Easy to modify behavior and add features  
- **Easier to Debug**: Python logging and error handling
- **Already Working**: All GPIO pins are available without conflicts
- **Maintainable**: Standard Python code vs kernel development

### GPIO Access Method
- Uses `libgpiod` (modern Linux GPIO interface)
- User-space GPIO control (no kernel modules needed)
- Direct pin control with proper error handling

## References

- [Orange Pi Zero 2W Documentation](https://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-pi-Zero-2W.html)
- [Linux GPIO Documentation](https://www.kernel.org/doc/Documentation/gpio/)
- [libgpiod Documentation](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/about/)
- [Systemd Service Documentation](https://www.freedesktop.org/software/systemd/man/systemd.service.html)

