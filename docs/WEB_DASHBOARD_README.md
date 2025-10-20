# Argo Web Dashboard

Mobile-friendly web interface for monitoring and controlling the Argo sailboat system.

## Quick Connection Guide

### 🖥️ **Desktop Access (Windows/Mac/Linux)**

#### Method 1: SSH Port Forwarding (Recommended)
```bash
# Forward local port 8081 to Orange Pi port 8081
ssh -L 8081:localhost:8081 orangepi@10.0.0.11

# Then open browser to:
http://localhost:8081
```

#### Method 2: Direct Network Access
```bash
# If on same network as Orange Pi
http://10.0.0.11:8081
```

### 📱 **Mobile Access**

#### Option 1: Mobile SSH App (Recommended)
1. **Install SSH app:**
   - **Android**: JuiceSSH, Termius, or ConnectBot
   - **iOS**: Termius, Prompt 3, or Blink Shell

2. **Configure connection:**
   - Host: `10.0.0.11`
   - Username: `orangepi`
   - Enable port forwarding: Local `8081` → Remote `localhost:8081`

3. **Access dashboard:**
   - Connect via SSH app
   - Open mobile browser to `http://localhost:8081`

#### Option 2: Router Port Forwarding
1. **Access router admin** (usually `http://10.0.0.1`)
2. **Configure port forwarding:**
   - External Port: `8082` (or any unused port)
   - Internal IP: `10.0.0.11`
   - Internal Port: `8081`
   - Protocol: `TCP`
3. **Access from anywhere:**
   ```
   http://ROUTER_EXTERNAL_IP:8082
   ```

#### Option 3: VPN Access
1. Set up VPN on router or Orange Pi
2. Connect phone to VPN
3. Access `http://10.0.0.11:8081` directly

### 🔧 **Troubleshooting Connection Issues**

#### Cannot Connect
- **Check Orange Pi IP**: `hostname -I` on Orange Pi
- **Verify web dashboard is running**: `ps aux | grep argo_web_dashboard`
- **Test local connection**: `curl -I http://localhost:8081` on Orange Pi
- **Check firewall**: Ensure port 8081 is not blocked

#### SSH Port Forwarding Not Working
- **Correct syntax**: `ssh -L LOCAL_PORT:localhost:REMOTE_PORT user@host`
- **For web dashboard**: `ssh -L 8081:localhost:8081 orangepi@10.0.0.11`
- **Alternative local port**: `ssh -L 8080:localhost:8081 orangepi@10.0.0.11` (then use `http://localhost:8080`)

#### Mobile SSH Apps
- **Android**: JuiceSSH (free) or Termius (free/paid)
- **iOS**: Termius (free/paid) or Prompt 3 (paid)
- **Port forwarding**: Enable in app settings before connecting

---

## Installation & Setup

### 1. Install Dependencies
```bash
pip3 install -r requirements-web-dashboard.txt
```

### 2. Start the Web Dashboard

#### Option A: Run directly
```bash
cd ~/argo
python3 nodes/argo_web_dashboard.py
```

#### Option B: Add to systemd (recommended for production)
Create `/etc/systemd/system/argo-web-dashboard.service`:
```ini
[Unit]
Description=Argo Web Dashboard
After=network.target

[Service]
Type=simple
User=orangepi
WorkingDirectory=/home/orangepi/argo
ExecStart=/usr/bin/python3 /home/orangepi/argo/nodes/argo_web_dashboard.py
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Then enable and start:
```bash
sudo systemctl enable argo-web-dashboard.service
sudo systemctl start argo-web-dashboard.service
```

### About the "development server" warning

When the dashboard starts, you may see this warning in the logs:

```
WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
```

**What it means**
- The dashboard uses Flask's built-in development server via `app.run(...)`. It's lightweight and convenient, but not intended for internet-facing production use.
- On a trusted local network (LAN) or when accessed through SSH port forwarding, this is acceptable.

**Safe ways to use it**
- **Local/LAN use**: Access from the same network (as shown above) is fine.
- **SSH port forwarding (recommended remotely)**: Keep the server as-is and tunnel with SSH; no public internet exposure.

**Hardening options (optional)**
- Bind only to localhost and use SSH port forwarding:
  - Change binding to `127.0.0.1` (instead of `0.0.0.0`) and connect via SSH port forwarding.
- Restrict access with a firewall: Allow port 8081 only from your management IP/network.
- Put a reverse proxy (e.g., nginx) in front for basic auth and TLS if you must expose beyond LAN.

Note: Migrating to a production WSGI server like `gunicorn` would require refactoring to expose a module-level Flask `app` separate from the ROS2 node. The current architecture embeds Flask inside a ROS2 node class for tight integration, so external WSGI runners cannot import `app` directly without that change.

---

## Features

### Status Monitoring
- **System Status**: Nodes running, system state, pause status
- **Battery**: Voltage, percentage, charging status, USB power, time estimates
- **GPS**: Lock status, position, heading, speed, course
- **Wind**: Speed, angle, temperature
- **Controller**: Mode (human/robot), controller type
- **Home Position**: Distance and bearing to home

### Control Actions

#### Controller Switching
- **Return to Home** 🏠: Activate emergency return-to-home mode
- **Proportional** 🎯: Basic heading maintenance controller
- **Wind Aware** 🌬️: Enhanced controller with wind-based sail control

#### System Control
- **Pause/Resume Nodes** ⏸️: Pause/unpause all non-critical nodes
- **Start/Stop Recording** 🔴/⏹️: Control data recording
- **Stop System** 🛑: Emergency system shutdown

## Mobile Optimization

The dashboard is optimized for mobile devices:
- Large touch targets (min 48x48px) for easy tapping
- Responsive layout that adapts to screen size
- Auto-refresh every 2 seconds for real-time updates
- Works in portrait and landscape modes
- Touch-friendly confirmation dialogs

## Architecture

### ROS2 Integration
The web dashboard integrates with existing Argo infrastructure:

**Topics Subscribed:**
- `/human_controlled`, `/battery_voltage`, `/battery_remaining_pct`
- `/compass`, `/pose`, `/gps_cog`, `/gps_sog`, `/fix`
- `/anem_speed_angle_temp`
- `/controller_state` (NEW - published by controller.py)

**Services Called:**
- `/toggle_pause` - Pause/unpause nodes (argo_lifecycle_manager)
- `/battery_status` - Get battery details (battery_water node)
- `/argo/recording/start` - Start recording (record node)
- `/argo/recording/stop` - Stop recording (record node)
- `/controller_node/switch_controller` - Switch controller type (NEW in controller.py)

### 3D Boat Visualization (optional but recommended)
The project includes a lightweight ROS2 node that publishes 3D visualization markers for the boat state. This is useful alongside the web dashboard for situational awareness during testing and can be viewed in both RViz and Foxglove's 3D panel.

**Node:** `nodes/argo_boat_visualization.py`

**What it shows:**
- Boat hull and mast orientation (using `/pose` and IMU-derived roll/pitch)
- Rudder and sail indicators (from `/rudder_sail_cmd`)
- Wind vector (from `/anem_speed_angle_temp`)
- GPS velocity vector (from `/gps_velocity`)
- Heading arrow and basic pose cues

**How to run:**
```bash
python3 nodes/argo_boat_visualization.py
```

The node publishes to `/visualization_marker` and `/visualization_marker_array`. View in:
- **RViz**: Add Marker and MarkerArray displays and subscribe to the above topics; set fixed frame to `map`.
- **Foxglove**: Add a 3D panel and subscribe to the same topics to see geometry in the scene.

### Resource Usage
- **RAM**: ~40 MB
- **CPU**: <1% idle, 3-8% when actively browsing
- **Network**: Port 8081 (HTTP)

## Controller Switching Implementation

The controller switching feature uses a ROS2 Trigger service. The implementation currently defaults to Return-to-Home mode for the RTH button. For full controller type selection, the service can be enhanced with a custom service type that accepts string parameters.

Current behavior:
- RTH button: Switches to `return_to_home` controller
- Other buttons: Show "not yet implemented" (service needs enhancement)

To fully implement controller switching:
1. Define custom service type with string parameter
2. Update controller.py service handler to parse controller type
3. Update web dashboard to pass controller type in service call

## Troubleshooting

### Status Not Updating
- Check ROS2 nodes are running: `ros2 node list`
- Verify topics are publishing: `ros2 topic list`
- Check browser console for JavaScript errors (F12)

### Service Calls Failing
- Ensure target services are running: `ros2 service list`
- Check service availability: `ros2 service call /toggle_pause std_srvs/srv/Trigger`
- Review web dashboard logs for error messages

## Development

### File Structure
```
nodes/
├── argo_web_dashboard.py          # Main ROS2 node with Flask server
├── templates/
│   └── dashboard.html              # Web UI
└── static/
    └── style.css                   # Responsive CSS
```

### Adding New Features
1. Add ROS2 subscriptions/services in `argo_web_dashboard.py`
2. Update state dictionary to store new data
3. Add API endpoints in Flask routes
4. Update HTML/JavaScript to display/control new features
5. Style new elements in CSS

### Testing
- **Backend**: Check ROS2 node logs and service calls
- **Frontend**: Use browser developer tools (F12) to debug JavaScript
- **Mobile**: Test on actual phone, not just browser resize

## Security Considerations

**Current Status**: No authentication - suitable for local network only

**For Production (internet-exposed)**:
- Add basic authentication (Flask-Login or HTTP Basic Auth)
- Use HTTPS with SSL certificate
- Implement rate limiting for API endpoints
- Add CSRF protection for POST requests
- Consider VPN access instead of public exposure

## Future Enhancements

Potential additions:
- [ ] Map view with Leaflet.js for GPS visualization
- [ ] Historical battery voltage graph
- [ ] Alert notifications (low battery, GPS loss, etc.)
- [ ] Settings page for parameter tuning
- [ ] Session logging and audit trail
- [ ] Full controller type selection (requires custom service type)
- [ ] Multiple boat support (fleet management)
- [ ] WebSocket for lower latency updates
- [ ] PWA (Progressive Web App) for offline functionality

## Credits

Developed for the Argo autonomous sailboat project.
Built with Flask, ROS2 Humble, and modern web technologies.
