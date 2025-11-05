# Debug Commands for Foxglove Connection from Remote Host

## Quick Test Commands (run on tobidh87)

### 1. Test Basic Connectivity
```bash
# Ping the router
ping -c 3 sensors-tobi-router.lan.ini.uzh.ch

# Check DNS resolution
host sensors-tobi-router.lan.ini.uzh.ch
nslookup sensors-tobi-router.lan.ini.uzh.ch
```

### 2. Test Port Connectivity
```bash
# Using netcat (nc)
nc -zv sensors-tobi-router.lan.ini.uzh.ch 8765

# Using telnet
telnet sensors-tobi-router.lan.ini.uzh.ch 8765
# (Press Ctrl+] then type 'quit' to exit)

# Using curl (HTTP test - will show if port is open)
curl -v --connect-timeout 5 http://sensors-tobi-router.lan.ini.uzh.ch:8765
```

### 3. Test WebSocket Connection
```bash
# Install wscat if needed
npm install -g wscat

# Test WebSocket connection
wscat -c ws://sensors-tobi-router.lan.uzh.ch:8765

# Or using curl to test WebSocket handshake
curl -v --no-buffer \
  -H "Connection: Upgrade" \
  -H "Upgrade: websocket" \
  -H "Sec-WebSocket-Version: 13" \
  -H "Sec-WebSocket-Key: $(openssl rand -base64 16)" \
  http://sensors-tobi-router.lan.ini.uzh.ch:8765
```

### 4. Test with Python (if available)
```python
import websocket
import ssl

try:
    ws = websocket.create_connection("ws://sensors-tobi-router.lan.ini.uzh.ch:8765", timeout=5)
    print("✅ WebSocket connection successful!")
    ws.close()
except Exception as e:
    print(f"❌ Connection failed: {e}")
```

### 5. Network Path Tracing
```bash
# Trace route to router
traceroute sensors-tobi-router.lan.ini.uzh.ch

# Check if port is reachable through firewall
# (if you have access to router)
```

## Expected Behavior

**Correct Setup:**
- Router: `sensors-tobi-router.lan.ini.uzh.ch:8765` 
- Forwards to: `10.0.0.3:8765` (Argo's foxglove_bridge)
- Argo's foxglove_bridge listens on: `0.0.0.0:8765` (all interfaces)

**Connection Flow:**
```
tobidh87 → sensors-tobi-router.lan.ini.uzh.ch:8765 
         → Router NAT/Port Forward 
         → 10.0.0.3:8765 
         → foxglove_bridge
```

## Common Issues

1. **Port not accessible**: Router firewall or port forwarding misconfigured
2. **DNS resolution fails**: Check if router hostname resolves correctly
3. **Connection timeout**: Router not forwarding correctly, or Argo's foxglove_bridge not running
4. **WebSocket handshake fails**: Protocol mismatch or firewall blocking WebSocket upgrade

## Verify on Argo Side

Run these on Argo (10.0.0.3) to verify foxglove_bridge is ready:

```bash
# Check if foxglove_bridge is running
ps aux | grep foxglove_bridge | grep -v grep

# Check if port is listening
netstat -tlnp | grep 8765
# or
ss -tlnp | grep 8765

# Check ROS2 node
ros2 node list | grep foxglove
```
