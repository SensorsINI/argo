<!-- e7fcaf68-628b-4726-b8c5-df570030d85b 3771fb6e-74fd-46ba-94eb-2e450213693f -->
# LoRa and WiFi Seamless Integration Plan

## Overview

Integrate LoRa communication as a fallback channel when WiFi is unavailable, with bandwidth-optimized transmission, unified topic structure, and optional return-to-home triggering based on shore connection loss.

## Key Design Decisions

### Topic Naming Convention

- **LoRa topics**: Use `lora/` prefix (e.g., `lora/gps_sog`, `lora/gps_cog`, `lora/battery_voltage`)
- **WiFi topics**: Keep existing names without prefix (e.g., `/gps_sog`, `/gps_cog`, `/battery_voltage`)
- **Dashboard**: Subscribe to both sources and display with age indicators

### LoRa Bandwidth Constraints

- **Realistic throughput at 1km**: ~250-1000 bps with SF7-SF9 at 125kHz bandwidth
- **Target packet size**: <80 bytes per transmission to keep under 1 second air time
- **Transmission interval**: 10 seconds (configurable) to stay within bandwidth budget
- **Compression strategy**: Abbreviated JSON keys, rounded values, optional delta encoding

### Data Priority

1. GPS position (lat/lon) - CRITICAL for RTH
2. Course/Speed over ground
3. Battery voltage
4. Human control status
5. Compass heading (optional, bandwidth permitting)

## Implementation Tasks

### 1. Fix LoRa Packet Format Mismatch (nodes/lora.py)

**File**: `nodes/lora.py` lines 569-586

**Current issue**: Sends abbreviated keys (`sog`, `cog`, `bat`, `hum`) but shore expects full keys

**Action**: Add GPS position and expand packet format to match shore expectations while staying compact:

```python
def build_status_packet(self) -> str:
    """Build compact status packet for LoRa transmission"""
    self.boat_state['timestamp'] = int(time.time())
    
    # Critical data with abbreviated keys (save bandwidth)
    packet = {
        'ts': self.boat_state['timestamp'],
        'lat': round(self.boat_state['gps_latitude'], 6) if self.boat_state['gps_latitude'] is not None else None,
        'lon': round(self.boat_state['gps_longitude'], 6) if self.boat_state['gps_longitude'] is not None else None,
        'sog': round(self.boat_state['gps_sog'], 2) if self.boat_state['gps_sog'] is not None else None,
        'cog': round(self.boat_state['gps_cog'], 1) if self.boat_state['gps_cog'] is not None else None,
        'bat': round(self.boat_state['battery_voltage'], 2) if self.boat_state['battery_voltage'] is not None else None,
        'hum': self.boat_state['human_controlled'],
        'hdg': round(self.boat_state['compass_heading'], 1) if self.boat_state['compass_heading'] is not None else None
    }
    
    # Remove None values to minimize bandwidth
    packet = {k: v for k, v in packet.items() if v is not None}
    
    return json.dumps(packet, separators=(',', ':'))
```

**Add GPS subscriptions**: lines 212-220

- Subscribe to `/fix` (NavSatFix) for lat/lon
- Subscribe to `/pose` or `/compass` for heading
- Store in boat_state dictionary

### 2. Update Shore Parser (shore/lora_shore.py)

**File**: `shore/lora_shore.py` lines 258-306

**Action**: Parse abbreviated keys and republish to `lora/` prefixed topics:

```python
def process_argo_packet(self, packet: dict, raw_json: str):
    """Process received packet from Argo and publish to ROS2 topics"""
    # ... existing code ...
    
    # Parse abbreviated keys and publish to lora/ prefixed topics
    if 'sog' in packet:
        msg = Float64()
        msg.data = float(packet['sog'])
        self.pub_argo_gps_sog.publish(msg)
    
    if 'cog' in packet:
        msg = Float64()
        msg.data = float(packet['cog'])
        self.pub_argo_gps_cog.publish(msg)
    
    if 'bat' in packet:
        msg = Float64()
        msg.data = float(packet['bat'])
        self.pub_argo_battery.publish(msg)
    
    if 'hum' in packet:
        msg = Bool()
        msg.data = bool(packet['hum'])
        self.pub_argo_human_controlled.publish(msg)
    
    # NEW: GPS position
    if 'lat' in packet and 'lon' in packet:
        fix_msg = NavSatFix()
        fix_msg.latitude = float(packet['lat'])
        fix_msg.longitude = float(packet['lon'])
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_argo_gps_fix.publish(fix_msg)
    
    # NEW: Compass heading
    if 'hdg' in packet:
        heading_msg = Vector3()
        heading_msg.z = float(packet['hdg'])
        self.pub_argo_compass.publish(heading_msg)
```

**Add new publishers**: lines 124-132

- `lora/fix` (NavSatFix)
- `lora/compass` (Vector3)

### 3. Add Shore-Side Ping Mechanism (shore/lora_shore.py)

**File**: `shore/lora_shore.py`

**Action**: Add periodic ping transmission (every 5 seconds)

```python
# In __init__ (after line 153):
self.ping_timer = self.create_timer(5.0, self.send_ping)
self.ping_sequence = 0

def send_ping(self):
    """Send periodic ping to Argo for connection monitoring"""
    if not self.ser or not self.ser.is_open:
        return
    
    try:
        self.ping_sequence += 1
        ping_msg = json.dumps({'cmd': 'ping', 'seq': self.ping_sequence}, separators=(',', ':'))
        self.ser.write(ping_msg.encode('utf-8'))
        self.ser.flush()
        self.get_logger().debug(f"Sent ping #{self.ping_sequence}")
    except Exception as e:
        self.get_logger().debug(f"Error sending ping: {e}")
```

### 4. Add Ping Reception and RTH Logic (nodes/lora.py)

**File**: `nodes/lora.py` lines 653-687

**Action**: Track ping reception and publish connection status, optionally trigger RTH

```python
# In __init__ (after line 244):
self.last_ping_time = time.time()
self.ping_timeout_sec = 30.0  # Consider disconnected after 30s no pings
self.rth_on_ping_loss_enabled = False  # Configurable feature
self.declare_parameter('rth_on_ping_loss', False)
self.rth_on_ping_loss_enabled = self.get_parameter('rth_on_ping_loss').value

# Publisher for RTH command
self.pub_rth_command = self.create_publisher(String, 'lora_remote_command', 10)

def parse_received_data(self, data: str):
    """Parse received LoRa data and extract commands"""
    try:
        parsed = json.loads(data)
        
        # Handle ping messages
        if parsed.get('cmd') == 'ping':
            self.last_ping_time = time.time()
            self.get_logger().debug(f"Received ping #{parsed.get('seq', '?')}")
            return
        
        # ... existing command parsing ...
    except json.JSONDecodeError:
        # ... existing text parsing ...

def check_connection_health(self):
    """Check LoRa connection health and optionally trigger RTH"""
    current_time = time.time()
    was_connected = self.is_connected
    
    # Check ping timeout
    time_since_ping = current_time - self.last_ping_time
    if time_since_ping > self.ping_timeout_sec:
        self.is_connected = False
        if was_connected:
            self.get_logger().warn(
                f"Shore ping timeout - no pings for {time_since_ping:.0f}s")
            
            # Optional: Trigger RTH on connection loss
            if self.rth_on_ping_loss_enabled:
                self.get_logger().warn("Triggering RETURN TO HOME due to shore connection loss")
                cmd_msg = String()
                cmd_msg.data = 'return_home'
                self.pub_rth_command.publish(cmd_msg)
    else:
        if not self.is_connected:
            self.is_connected = True
            self.get_logger().info("Shore connection restored")
    
    # ... rest of existing health check ...
```

### 5. Update Web Dashboard for Dual-Source Topics (nodes/argo_web_dashboard.py)

**File**: `nodes/argo_web_dashboard.py` lines 121-130

**Action**: Subscribe to both WiFi and LoRa sources, track timestamps, display freshest data

```python
# Add state tracking for data sources
self.state = {
    # ... existing state ...
    
    # Source tracking
    'data_source': 'WiFi',  # 'WiFi', 'LoRa', or 'Offline'
    'wifi_data_age': None,  # seconds since last WiFi update
    'lora_data_age': None,  # seconds since last LoRa update
}

# Timestamps for each data type
self.last_wifi_update = {}
self.last_lora_update = {}

# Subscribe to both WiFi and LoRa sources
self.create_subscription(Float64, '/gps_sog', lambda msg: self.gps_sog_cb(msg, 'wifi'), 10)
self.create_subscription(Float64, 'lora/gps_sog', lambda msg: self.gps_sog_cb(msg, 'lora'), 10)

self.create_subscription(Float64, '/gps_cog', lambda msg: self.gps_cog_cb(msg, 'wifi'), 10)
self.create_subscription(Float64, 'lora/gps_cog', lambda msg: self.gps_cog_cb(msg, 'lora'), 10)

# ... repeat for all critical topics ...

def gps_sog_cb(self, msg, source='wifi'):
    """Unified callback that tracks source and timestamp"""
    now = time.time()
    
    with self.state_lock:
        # Always update if this is newer data or first data
        if source == 'wifi':
            self.last_wifi_update['gps_sog'] = now
            self.state['gps_sog'] = msg.data
            self.state['data_source'] = 'WiFi'
        elif source == 'lora':
            self.last_lora_update['gps_sog'] = now
            # Only use LoRa data if WiFi is stale (>2 seconds old)
            wifi_age = now - self.last_wifi_update.get('gps_sog', 0)
            if wifi_age > 2.0:
                self.state['gps_sog'] = msg.data
                self.state['data_source'] = 'LoRa'
        
        # Update data age indicators
        self.state['wifi_data_age'] = now - max(self.last_wifi_update.values(), default=0)
        self.state['lora_data_age'] = now - max(self.last_lora_update.values(), default=0)
```

### 6. Add Bandwidth Throttling to Argo LoRa Node (nodes/lora.py)

**File**: `nodes/lora.py` lines 633-651

**Action**: Monitor packet transmission timing and adjust interval dynamically

```python
def transmit_status(self) -> bool:
    """Transmit boat status via LoRa radio with bandwidth monitoring"""
    try:
        # Build status packet
        status_packet = self.build_status_packet()
        packet_bytes = status_packet.encode('ascii')
        
        # Bandwidth check: ensure we don't exceed limits
        packet_size = len(packet_bytes)
        estimated_air_time = self._estimate_air_time(packet_size)
        
        if estimated_air_time > 2.0:  # Warn if packet takes >2 seconds
            self.get_logger().warn(
                f"LoRa packet large: {packet_size} bytes, ~{estimated_air_time:.1f}s air time")
        
        # Log bandwidth usage
        self.get_logger().debug(
            f"LoRa TX: {packet_size} bytes, air time: ~{estimated_air_time:.2f}s")
        
        # Transmit
        success = self.transmit_packet(packet_bytes)
        return success
    except Exception as e:
        self.get_logger().error(f"Error transmitting status: {e}")
        return False

def _estimate_air_time(self, packet_size_bytes: int) -> float:
    """Estimate LoRa air time in seconds based on SF, BW, and packet size"""
    # Simplified formula for SF7, BW=125kHz, CR=4/5
    # Actual air time depends on preamble, header, payload, CRC
    # Rough estimate: ~8-12 ms per byte at SF7, ~40-50 ms per byte at SF9
    sf_multiplier = {7: 0.010, 8: 0.015, 9: 0.040, 10: 0.080, 11: 0.160, 12: 0.320}
    multiplier = sf_multiplier.get(self.spreading_factor, 0.040)
    return packet_size_bytes * multiplier
```

### 7. Create Shore-Side LoRa Test Script (shore/lora_test.py)

**File**: `shore/lora_test.py` (NEW)

**Purpose**: Standalone test script for shore-side LoRa communication without ROS2 dependency

```python
#!/usr/bin/env python3
"""
Shore-Side LoRa Test Script
Monitors LoRa communication, logs packets with timestamps, validates bandwidth
"""

import serial
import json
import time
import sys
from datetime import datetime
from pathlib import Path

class LoRaTestMonitor:
    def __init__(self, port='/dev/ttyACM0', baud=115200, log_dir='lora_logs'):
        self.port = port
        self.baud = baud
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        # Statistics
        self.packets_received = 0
        self.packets_sent = 0
        self.last_packet_time = None
        self.packet_intervals = []
        self.packet_sizes = []
        
        # Start log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"shore_lora_{timestamp}.log"
        
    def log(self, message):
        """Log message to console and file with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {message}"
        print(log_line)
        with open(self.log_file, 'a') as f:
            f.write(log_line + '\n')
    
    def connect(self):
        """Connect to LoRa module"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.log(f"✓ Connected to {self.port} at {self.baud} baud")
            return True
        except Exception as e:
            self.log(f"✗ Connection failed: {e}")
            return False
    
    def send_ping(self, seq):
        """Send ping message to Argo"""
        try:
            ping = json.dumps({'cmd': 'ping', 'seq': seq}, separators=(',', ':'))
            self.ser.write(ping.encode('utf-8'))
            self.ser.flush()
            self.packets_sent += 1
            self.log(f"→ PING #{seq} sent ({len(ping)} bytes)")
            return True
        except Exception as e:
            self.log(f"✗ Ping send failed: {e}")
            return False
    
    def send_command(self, command):
        """Send command to Argo"""
        try:
            cmd = json.dumps({'cmd': command}, separators=(',', ':'))
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.packets_sent += 1
            self.log(f"→ COMMAND '{command}' sent ({len(cmd)} bytes)")
            return True
        except Exception as e:
            self.log(f"✗ Command send failed: {e}")
            return False
    
    def process_packet(self, json_str):
        """Process and log received packet"""
        try:
            packet = json.loads(json_str)
            self.packets_received += 1
            
            # Calculate interval since last packet
            now = time.time()
            if self.last_packet_time:
                interval = now - self.last_packet_time
                self.packet_intervals.append(interval)
            self.last_packet_time = now
            
            # Track packet size
            self.packet_sizes.append(len(json_str))
            
            # Log packet details
            self.log(f"← PACKET #{self.packets_received} ({len(json_str)} bytes): {json_str}")
            
            # Parse and display key fields
            fields = []
            if 'lat' in packet and 'lon' in packet:
                fields.append(f"GPS: {packet['lat']:.6f}, {packet['lon']:.6f}")
            if 'sog' in packet:
                fields.append(f"SOG: {packet['sog']} kt")
            if 'cog' in packet:
                fields.append(f"COG: {packet['cog']}°")
            if 'bat' in packet:
                fields.append(f"BAT: {packet['bat']}V")
            if 'hum' in packet:
                fields.append(f"CTRL: {'Human' if packet['hum'] else 'Robot'}")
            if 'hdg' in packet:
                fields.append(f"HDG: {packet['hdg']}°")
            
            if fields:
                self.log(f"  └─ {' | '.join(fields)}")
                
        except json.JSONDecodeError as e:
            self.log(f"✗ JSON parse error: {e}")
            self.log(f"  Raw data: {json_str}")
    
    def print_statistics(self):
        """Print communication statistics"""
        self.log("\n" + "="*60)
        self.log("STATISTICS")
        self.log("="*60)
        self.log(f"Packets received: {self.packets_received}")
        self.log(f"Packets sent: {self.packets_sent}")
        
        if self.packet_intervals:
            avg_interval = sum(self.packet_intervals) / len(self.packet_intervals)
            min_interval = min(self.packet_intervals)
            max_interval = max(self.packet_intervals)
            self.log(f"Packet intervals: avg={avg_interval:.2f}s, min={min_interval:.2f}s, max={max_interval:.2f}s")
        
        if self.packet_sizes:
            avg_size = sum(self.packet_sizes) / len(self.packet_sizes)
            min_size = min(self.packet_sizes)
            max_size = max(self.packet_sizes)
            self.log(f"Packet sizes: avg={avg_size:.1f} bytes, min={min_size}, max={max_size}")
            
            # Estimate bandwidth usage
            if self.packet_intervals:
                avg_interval = sum(self.packet_intervals) / len(self.packet_intervals)
                bits_per_second = (avg_size * 8) / avg_interval
                self.log(f"Estimated bandwidth: {bits_per_second:.1f} bps")
        
        self.log("="*60)
    
    def run(self):
        """Main test loop"""
        if not self.connect():
            return
        
        self.log("\nStarting LoRa test monitor...")
        self.log("Commands: 'ping', 'rth' (return to home), 'stats', 'quit'\n")
        
        buffer = ""
        last_ping_time = 0
        ping_sequence = 0
        
        try:
            while True:
                # Auto-ping every 5 seconds
                now = time.time()
                if now - last_ping_time > 5.0:
                    ping_sequence += 1
                    self.send_ping(ping_sequence)
                    last_ping_time = now
                
                # Read from serial
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    text = data.decode('utf-8', errors='ignore')
                    buffer += text
                    
                    # Process complete JSON packets
                    while '{' in buffer and '}' in buffer:
                        json_start = buffer.index('{')
                        json_end = buffer.index('}', json_start) + 1
                        json_str = buffer[json_start:json_end].strip()
                        buffer = buffer[json_end:]
                        
                        self.process_packet(json_str)
                
                # Check for user input (non-blocking)
                # Note: This is simplified; real implementation would use select() or threads
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            self.log("\n\nTest interrupted by user")
            self.print_statistics()
            self.log(f"\nLog saved to: {self.log_file}")

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Shore-side LoRa test monitor')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--log-dir', default='lora_logs', help='Log directory')
    
    args = parser.parse_args()
    
    monitor = LoRaTestMonitor(args.port, args.baud, args.log_dir)
    monitor.run()
```

### 8. Create Argo-Side LoRa Test Script (scripts/lora_test_argo.py)

**File**: `scripts/lora_test_argo.py` (NEW)

**Purpose**: Test script that runs on Argo to monitor LoRa TX/RX and log timing

```python
#!/usr/bin/env python3
"""
Argo-Side LoRa Test Script
Monitors LoRa node behavior, logs packets with timestamps, validates reception
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
import time
from datetime import datetime
from pathlib import Path

class ArgoLoRaTestMonitor(Node):
    def __init__(self):
        super().__init__('argo_lora_test_monitor')
        
        # Setup logging
        self.log_dir = Path.home() / 'lora_logs'
        self.log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"argo_lora_{timestamp}.log"
        
        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.pings_received = 0
        self.last_ping_time = None
        self.ping_intervals = []
        
        # Subscribe to LoRa topics
        self.create_subscription(String, 'lora_rx_data', self.rx_callback, 10)
        self.create_subscription(String, 'lora_remote_command', self.cmd_callback, 10)
        self.create_subscription(Int32, 'lora_signal_strength', self.rssi_callback, 10)
        
        # Monitor what we're transmitting (would need to tap into lora node)
        # For now, we'll subscribe to the data sources that lora node uses
        self.create_subscription(Float64, 'gps_sog', self.data_update_callback, 10)
        
        self.log("Argo LoRa Test Monitor Started")
        
        # Timer for periodic statistics
        self.create_timer(30.0, self.print_statistics)
    
    def log(self, message):
        """Log with timestamp to console and file"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {message}"
        self.get_logger().info(message)
        with open(self.log_file, 'a') as f:
            f.write(log_line + '\n')
    
    def rx_callback(self, msg):
        """Monitor received LoRa data"""
        self.packets_received += 1
        self.log(f"← RX PACKET #{self.packets_received}: {msg.data}")
        
        try:
            data = json.loads(msg.data)
            if data.get('cmd') == 'ping':
                self.pings_received += 1
                now = time.time()
                if self.last_ping_time:
                    interval = now - self.last_ping_time
                    self.ping_intervals.append(interval)
                    self.log(f"  └─ PING #{data.get('seq', '?')} (interval: {interval:.2f}s)")
                self.last_ping_time = now
        except:
            pass
    
    def cmd_callback(self, msg):
        """Monitor parsed commands"""
        self.log(f"← COMMAND: {msg.data}")
    
    def rssi_callback(self, msg):
        """Monitor signal strength"""
        self.log(f"  └─ RSSI: {msg.data} dBm")
    
    def data_update_callback(self, msg):
        """Monitor when data is updated (triggers TX)"""
        # This is a simplified monitor - actual TX happens in lora node
        pass
    
    def print_statistics(self):
        """Print periodic statistics"""
        self.log("\n" + "="*60)
        self.log("STATISTICS (last 30 seconds)")
        self.log("="*60)
        self.log(f"Packets received: {self.packets_received}")
        self.log(f"Pings received: {self.pings_received}")
        
        if self.ping_intervals:
            avg_interval = sum(self.ping_intervals) / len(self.ping_intervals)
            self.log(f"Avg ping interval: {avg_interval:.2f}s (expected: 5.0s)")
            
            # Check for missing pings
            expected_pings = 30 / 5  # 30 second window, 5 second interval
            if self.pings_received < expected_pings * 0.8:
                self.log(f"⚠ WARNING: Missing pings! Expected ~{expected_pings}, got {self.pings_received}")
        
        self.log("="*60 + "\n")

def main():
    rclpy.init()
    monitor = ArgoLoRaTestMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.log("\nTest interrupted by user")
        monitor.print_statistics()
        monitor.log(f"\nLog saved to: {monitor.log_file}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 9. Create End-to-End Test Plan Document (tests/LORA_TEST_PLAN.md)

**File**: `tests/LORA_TEST_PLAN.md` (NEW)

```markdown
# LoRa Communication Test Plan

## Test Environment Setup

### Shore Side
1. Connect Waveshare USB-TO-LoRa module to laptop/PC
2. Verify serial port: `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`
3. Source ROS2: `source /opt/ros/humble/setup.bash`
4. Start test monitor: `python3 shore/lora_test.py --port /dev/ttyACM0`

### Argo Side
1. Ensure LoRa module connected via SPI (pins 19,21,23,27,29,31)
2. SSH to Argo: `ssh orangepi@<ARGO_IP>`
3. Start test monitor: `python3 scripts/lora_test_argo.py`
4. Start LoRa node: `ros2 run argo lora.py` (or via lifecycle manager)

## Test Cases

### TEST 1: Basic Connectivity
**Objective**: Verify bidirectional LoRa communication

**Steps**:
1. Start shore test monitor
2. Start Argo LoRa node
3. Observe ping messages flowing shore→argo (every 5s)
4. Observe status packets flowing argo→shore (every 10s)

**Expected Results**:
- Shore logs show: "← PACKET" messages every 10 seconds
- Argo logs show: "← RX PACKET" and "PING" messages every 5 seconds
- No JSON parse errors
- Packet sizes < 100 bytes

### TEST 2: Packet Content Validation
**Objective**: Verify all critical fields present and formatted correctly

**Steps**:
1. Start Argo with GPS active
2. Monitor shore-side logs for 60 seconds
3. Verify packet contains: lat, lon, sog, cog, bat, hum, hdg

**Expected Results**:
- All packets contain lat/lon with 6 decimal places
- SOG rounded to 2 decimals, COG to 1 decimal
- Battery voltage present and reasonable (11-13V)
- Human control flag is boolean

### TEST 3: Bandwidth Measurement
**Objective**: Measure actual bandwidth usage and air time

**Steps**:
1. Run for 5 minutes with standard 10s interval
2. Check shore test monitor statistics
3. Calculate: (avg_packet_size * 8) / avg_interval = bps

**Expected Results**:
- Average packet size: 80-100 bytes
- Average interval: 10 seconds
- Estimated bandwidth: 64-80 bps
- Well under SF7 limit (~1000 bps at 1km)

### TEST 4: Ping Timeout Detection
**Objective**: Verify Argo detects shore connection loss

**Steps**:
1. Start both sides, verify communication
2. Stop shore-side test monitor
3. Wait 35 seconds
4. Check Argo logs for timeout warning

**Expected Results**:
- After ~30 seconds: "Shore ping timeout" warning
- lora_connection_status topic publishes False
- If RTH enabled: "Triggering RETURN TO HOME" message

### TEST 5: Return-to-Home Command
**Objective**: Test RTH command transmission and reception

**Steps**:
1. Start both sides
2. From shore monitor, send: `{"cmd":"return_home"}`
3. Monitor Argo logs and controller behavior

**Expected Results**:
- Argo logs: "← COMMAND: return_home"
- Controller logs: "RETURN TO HOME activated"
- Boat state: return_to_home_active = True

### TEST 6: Dual-Source Dashboard
**Objective**: Verify dashboard shows correct data source

**Steps**:
1. Start Argo with both WiFi and LoRa active
2. Open web dashboard
3. Verify data source indicator shows "WiFi"
4. Disable WiFi connection
5. Wait 5 seconds
6. Verify data source switches to "LoRa"

**Expected Results**:
- Dashboard initially shows "WiFi" source
- After WiFi loss, switches to "LoRa" within 5 seconds
- Data continues updating via LoRa topics
- Age indicators show data freshness

### TEST 7: Signal Strength Monitoring
**Objective**: Test RSSI reporting at various distances

**Steps**:
1. Start at 10m distance, record RSSI
2. Move to 50m, 100m, 500m, 1km
3. Log RSSI at each distance

**Expected Results**:
- RSSI degrades with distance
- Communication maintained at 1km (RSSI > -120 dBm)
- Packet loss < 10% at maximum range

## Log Analysis

### Shore Log Format
```

[2025-01-15 10:30:45.123] → PING #42 sent (23 bytes)

[2025-01-15 10:30:47.456] ← PACKET #21 (87 bytes): {"ts":1234567890,...}

[2025-01-15 10:30:47.457]   └─ GPS: 37.123456, -122.234567 | SOG: 3.2 kt | ...

```

### Argo Log Format
```

[2025-01-15 10:30:45.124] ← RX PACKET #42: {"cmd":"ping","seq":42}

[2025-01-15 10:30:45.125]   └─ PING #42 (interval: 5.01s)

[2025-01-15 10:30:45.126]   └─ RSSI: -85 dBm

```

## Success Criteria

- ✓ Bidirectional communication established
- ✓ Packet loss < 5% at 100m, < 10% at 1km
- ✓ All critical fields present in packets
- ✓ Bandwidth usage < 100 bps average
- ✓ Ping timeout detected within 35 seconds
- ✓ RTH command successfully triggers controller
- ✓ Dashboard seamlessly switches between WiFi and LoRa

## Troubleshooting

### No packets received on shore
- Check serial port connection: `ls -l /dev/ttyACM*`
- Verify baud rate matches (115200)
- Check LoRa frequency matches both sides (433MHz)

### No pings received on Argo
- Verify LoRa node is running: `ros2 node list | grep lora`
- Check SPI connections and GPIO pins
- Monitor raw LoRa RX: `ros2 topic echo /lora_rx_data`

### High packet loss
- Check RSSI levels (< -110 dBm is marginal)
- Reduce distance or increase TX power
- Check for interference on 433MHz band
```

## Testing Strategy

1. **Unit Testing**: Individual component testing with test scripts
2. **Integration Testing**: Full system test with dual monitors running
3. **Range Testing**: Distance measurements with RSSI logging
4. **Bandwidth Validation**: Verify packet sizes and air time estimates
5. **Failover Testing**: WiFi disconnect scenarios with dashboard monitoring
6. **Duration Testing**: 24-hour continuous operation with periodic log analysis
7. **Stress Testing**: Rapid command transmission to test bandwidth limits

## Configuration Parameters

### nodes/lora.py

- `tx_interval_sec`: 10.0 (adjust based on bandwidth testing)
- `rth_on_ping_loss`: false (enable for production, disable for development)
- `ping_timeout_sec`: 30.0

### shore/lora_shore.py

- `ping_interval_sec`: 5.0
- `reconnect_interval`: 1.0

## Files Modified

1. `nodes/lora.py` - Add GPS position, ping reception, RTH logic, bandwidth monitoring
2. `shore/lora_shore.py` - Parse abbreviated keys, add GPS publishers, implement ping transmission
3. `nodes/argo_web_dashboard.py` - Dual-source subscriptions, data age tracking, source indication
4. `nodes/controller.py` - Already has LoRa command handling, no changes needed

## Expected Packet Format

**Argo to Shore** (every 10 seconds):

```json
{"ts":1234567890,"lat":37.123456,"lon":-122.123456,"sog":3.2,"cog":45.5,"bat":12.6,"hum":false,"hdg":47.3}
```

Size: ~90 bytes (fits within budget)

**Shore to Argo** (every 5 seconds):

```json
{"cmd":"ping","seq":123}
```

Size: ~25 bytes

**Shore to Argo** (RTH command):

```json
{"cmd":"return_home"}
```

Size: ~21 bytes

### To-dos

- [ ] Update nodes/lora.py to add GPS lat/lon, compass heading to status packet with abbreviated keys
- [ ] Update shore/lora_shore.py to parse abbreviated keys and add publishers for lora/fix and lora/compass topics
- [ ] Add periodic ping transmission to shore/lora_shore.py (every 5 seconds)
- [ ] Add ping reception tracking and optional RTH trigger to nodes/lora.py
- [ ] Update nodes/argo_web_dashboard.py to subscribe to both WiFi and lora/ topics with age tracking
- [ ] Add bandwidth throttling and air time estimation to nodes/lora.py
- [ ] Test dual-source operation, bandwidth limits, ping timeout, and RTH triggering