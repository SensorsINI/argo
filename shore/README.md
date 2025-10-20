# Shore-Side LoRa Communication

This directory contains shore-side ROS2 nodes for communicating with the Argo sailboat via LoRa radio.

## 📋 Quick Start

### Prerequisites

- **ROS2 Humble** installed (see [INSTALL.md](INSTALL.md) for setup)
- **pyserial** Python package
- **Waveshare USB-TO-LoRa module** connected

### Installation

```bash
# 1. Install ROS2 (one-time setup)
#    See INSTALL.md for complete instructions
sudo apt install ros-humble-ros-base

# 2. Install Python dependencies
pip3 install -r requirements.txt

# 3. Run shore node using launcher (handles conda/ROS2 automatically)
./run_lora_shore.sh
```

**Or manually:**
```bash
# If in conda: deactivate it first
conda deactivate

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Run shore node
python3 lora_shore.py
```

**👉 For detailed installation instructions, see [INSTALL.md](INSTALL.md)**

**⚠️ Conda Users**: If you use conda, either use `run_lora_shore.sh` or deactivate conda before running. See [INSTALL.md](INSTALL.md#error-ros2-not-found-but-youre-in-a-conda-environment) for details.

## Components

### lora_shore.py
Shore-side ROS2 node that interfaces with the Waveshare USB-TO-LoRa module.

**Functionality:**
- Receives status packets from Argo via LoRa
- Republishes Argo data to ROS2 topics with `argo/` prefix
- Sends commands to Argo via LoRa
- Monitors connection health

**Dependencies:**
- ROS2 Humble (`ros-humble-ros-base`)
- Python 3.10+
- pyserial (`pip3 install pyserial`)

## Hardware Requirements

- **Waveshare USB-TO-LoRa-LF-B** (SX1262) module
  - Or any SX1276/SX1278-based USB LoRa module
- USB connection to computer running ROS2
- Antenna connected to LoRa module
- Computer running Ubuntu 22.04 or compatible Linux

## Configuration

### Waveshare Module Setup

The Waveshare module should be configured with:
- **Network ID**: 18 (0x12) - Must match Argo
- **Frequency**: 433 MHz (Channel 23 for LF version)
- **Spreading Factor**: 7
- **Bandwidth**: 125 kHz (0)
- **Code Rate**: 4/5 (1)
- **Mode**: Stream (1)
- **Encryption**: Disabled (KEY=0)

**See `../nodes/README-LORA-TO-WAVESHARE.md` for AT command configuration details.**

### Serial Port Setup

```bash
# Find the Waveshare device
ls -l /dev/ttyACM*
# Should show: crw-rw---- 1 root dialout ...

# Ensure your user is in dialout group
groups | grep dialout

# If not, add yourself (one-time setup):
sudo usermod -a -G dialout $USER
# Then log out and back in
```

## Usage

### Running the Node

```bash
# Basic usage (defaults to /dev/ttyACM0 at 115200 baud)
cd /path/to/argo/shore
python3 lora_shore.py

# Or with ROS2 run
ros2 run argo lora_shore

# With custom serial port
ros2 run argo lora_shore --ros-args -p serial_port:=/dev/ttyUSB0

# With debug output
ros2 run argo lora_shore --ros-args -p enable_debug:=true
```

### Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyACM0` | Serial port for Waveshare module |
| `baud_rate` | int | `115200` | Serial baud rate |
| `enable_debug` | bool | `false` | Enable debug logging |

## ROS2 Topics

### Published Topics (Shore → Foxglove)

Data received from Argo and republished:

| Topic | Type | Description |
|-------|------|-------------|
| `/argo/gps_sog` | `std_msgs/Float64` | Speed over ground (knots) |
| `/argo/gps_cog` | `std_msgs/Float64` | Course over ground (degrees) |
| `/argo/battery_voltage` | `std_msgs/Float64` | Battery voltage |
| `/argo/human_controlled` | `std_msgs/Bool` | Control mode (persistent QoS) |
| `/argo/lora_rssi` | `std_msgs/Int32` | Signal strength (dBm) |
| `/argo/last_contact` | `std_msgs/String` | Last contact timestamp |
| `/argo/lora_raw` | `std_msgs/String` | Raw JSON packet data |
| `/shore/lora_status` | `std_msgs/String` | Shore-side status info |

### Subscribed Topics (Commands to Argo)

| Topic | Type | Description |
|-------|------|-------------|
| `/argo/remote_command` | `std_msgs/String` | Commands to send to Argo |

## Sending Commands to Argo

### Via Command Line

```bash
# Return home command
ros2 topic pub --once /argo/remote_command std_msgs/String "data: 'return_home'"

# Stop command
ros2 topic pub --once /argo/remote_command std_msgs/String "data: 'stop'"

# Custom JSON command
ros2 topic pub --once /argo/remote_command std_msgs/String "data: '{\"cmd\":\"waypoint\",\"lat\":37.123,\"lon\":-122.456}'"
```

### Via Foxglove

1. Add a **Publish Panel**
2. Set topic: `/argo/remote_command`
3. Set message type: `std_msgs/String`
4. Enter command in `data` field
5. Click **Publish**

### Via Python

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('command_sender')
pub = node.create_publisher(String, '/argo/remote_command', 10)

# Send command
msg = String()
msg.data = 'return_home'
pub.publish(msg)

rclpy.shutdown()
```

## Monitoring

### View Argo Status

```bash
# Watch all Argo data
ros2 topic echo /argo/gps_sog
ros2 topic echo /argo/battery_voltage
ros2 topic echo /argo/human_controlled

# Watch connection status
ros2 topic echo /shore/lora_status

# Watch raw LoRa packets
ros2 topic echo /argo/lora_raw
```

### Check Node Status

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /lora_shore_node

# View topics
ros2 topic list | grep argo
```

## Integration with Foxglove

### Recommended Panels

1. **Gauge Panels**: Battery voltage, RSSI signal strength
2. **Indicator Panel**: Last contact time, human_controlled status  
3. **Plot Panels**: SOG, COG trends over time
4. **Raw Messages**: View complete status packets
5. **Publish Panel**: Send commands to Argo

### Example Foxglove Layout

Create a layout file `shore/foxglove/argo_shore_monitor.json`:

```json
{
  "configById": {
    "battery_gauge": {
      "path": "/argo/battery_voltage",
      "minValue": 6.0,
      "maxValue": 8.5,
      "colorMap": "red-yellow-green"
    },
    "rssi_gauge": {
      "path": "/argo/lora_rssi",
      "minValue": -120,
      "maxValue": -30,
      "colorMap": "turbo"
    },
    "sog_plot": {
      "paths": [{"value": "/argo/gps_sog"}],
      "showXAxisLabels": true
    },
    "command_publish": {
      "topicName": "/argo/remote_command",
      "datatype": "std_msgs/String"
    }
  }
}
```

## Troubleshooting

### No Packets Received

1. **Check serial connection**:
   ```bash
   ls -l /dev/ttyACM0
   # Should exist and be readable
   ```

2. **Check Waveshare configuration**:
   ```bash
   # Run on shore computer
   picocom -b 115200 /dev/ttyACM0
   # Type: +++  (enter AT mode)
   # Type: AT+NETID?  (should show +NETID=18)
   # Ctrl+A, Ctrl+X to exit
   ```

3. **Check Argo is transmitting**:
   ```bash
   # On Argo
   ros2 topic echo /lora_tx_data
   ```

4. **Check antennas**: Both sides must have antennas connected!

### Commands Not Reaching Argo

1. **Check Argo is in RX mode**:
   ```bash
   # On Argo
   ros2 node list | grep lora
   ```

2. **Check command format**: Commands should be plain text or JSON strings

3. **Monitor Argo logs**:
   ```bash
   # On Argo
   journalctl -u argo-launch.service -f | grep lora
   ```

### Serial Port Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, then verify
groups | grep dialout
```

## Testing

### Test Without Argo (Loopback)

1. Configure Waveshare to transmit test packets:
   ```bash
   # Terminal 1: Run shore node
   python3 lora_shore.py --ros-args -p enable_debug:=true
   
   # Terminal 2: Watch for packets
   ros2 topic echo /argo/lora_raw
   
   # Terminal 3: Send test via serial (simulating Argo)
   echo '{"ts":1234567890,"test":"hello"}' > /dev/ttyACM0
   ```

### Test With Argo

1. Start shore node:
   ```bash
   python3 lora_shore.py
   ```

2. On Argo, start `lora.py`:
   ```bash
   # On Argo
   python3 nodes/lora.py
   ```

3. Watch for packets:
   ```bash
   ros2 topic echo /argo/lora_raw
   ```

4. Send command to Argo:
   ```bash
   ros2 topic pub --once /argo/remote_command std_msgs/String "data: 'test_command'"
   ```

## Performance

- **Packet interval**: 10 seconds (configurable in Argo's `lora.py`)
- **Latency**: ~100-500ms depending on distance and conditions
- **Range**: Up to 10km line-of-sight (tested at 4m)
- **Data rate**: ~250-300 bytes/s with SF7, 125kHz BW

## Future Enhancements

- Add GPS position to status packets
- Implement command acknowledgment system
- Add packet encryption
- Implement store-and-forward for queued commands
- Add adaptive data rate based on RSSI

## References

- [Argo LoRa Communication Documentation](../.cursor/rules/argo-lora-communication.mdc)
- [Waveshare Compatibility Guide](../nodes/README-LORA-TO-WAVESHARE.md)
- [Argo Main LoRa Node](../nodes/lora.py)

