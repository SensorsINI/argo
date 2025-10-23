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
3. Verify packet contains: lat, lon, sog, cog, bat, hum, hdg, seq

**Expected Results**:
- All packets contain lat/lon with 6 decimal places
- SOG rounded to 2 decimals, COG to 1 decimal
- Battery voltage present and reasonable (6-9V range)
- Human control flag is boolean
- Sequence numbers increment properly

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
{"ts":1234567890,"seq":42,"lat":37.123456,"lon":-122.123456,"sog":3.2,"cog":45.5,"bat":12.6,"hum":false,"hdg":47.3}
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

