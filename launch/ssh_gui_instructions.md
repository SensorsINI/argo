# Running Argo Status GUI Remotely via SSH

## 🖥️ Method 1: SSH X11 Forwarding (Recommended)

### From your local machine:

```bash
# Connect with X11 forwarding enabled
ssh -X orangepi@YOUR_ORANGE_PI_IP

# Or for better performance with compression
ssh -X -C orangepi@YOUR_ORANGE_PI_IP

# For trusted connections (faster but less secure)
ssh -Y orangepi@YOUR_ORANGE_PI_IP
```

### Once connected, run the GUI:
```bash
cd ~/argo/launch
./argo_status_gui.sh
```

### Troubleshooting X11 Forwarding:

**On your local machine (client):**
```bash
# Make sure X11 forwarding is enabled in SSH config
echo "ForwardX11 yes" >> ~/.ssh/config

# For macOS, install XQuartz first:
# brew install --cask xquartz
# Then restart and try again
```

**On Orange Pi (server):**
```bash
# Enable X11 forwarding in SSH daemon
sudo nano /etc/ssh/sshd_config

# Add/uncomment these lines:
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost yes

# Restart SSH service
sudo systemctl restart sshd

# Install X11 utilities if missing
sudo apt-get install xauth xorg-dev
```

---

## 🌐 Method 2: VNC Server (Better for slow connections)

### Install and setup VNC on Orange Pi:
```bash
# Install VNC server
sudo apt-get update
sudo apt-get install tightvncserver

# Start VNC server (first time setup)
vncserver :1

# You'll be prompted to set a password
```

### Create VNC startup script:
```bash
# Create vnc startup script
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
xrdb $HOME/.Xresources
startxfce4 &
EOF

chmod +x ~/.vnc/xstartup

# Restart VNC with desktop environment
vncserver -kill :1
vncserver :1 -geometry 1024x768 -depth 24
```

### Connect from your local machine:
```bash
# Create SSH tunnel for VNC
ssh -L 5901:localhost:5901 orangepi@YOUR_ORANGE_PI_IP

# Then connect with VNC viewer to: localhost:5901
# Or use browser-based VNC: http://localhost:5901
```

### Run GUI in VNC session:
```bash
cd ~/argo/launch
./argo_status_gui.sh
```

---

## 🔧 Method 3: Web-based Alternative

Since you already have Foxglove setup, consider creating a web-based status dashboard:

```bash
# Create simple web status server
cat > ~/argo/launch/web_status.py << 'EOF'
#!/usr/bin/env python3
"""Simple web status server for remote monitoring"""

from http.server import HTTPServer, BaseHTTPRequestHandler
import json
# Note: argo_status_check.py is deprecated, use argo_lifecycle_manager.py instead
# from argo_status_check import OptimizedArgoStatusChecker

class StatusHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/status':
            checker = OptimizedArgoStatusChecker()
            ros_info = checker.check_ros_nodes_fast()
            sys_info = checker.get_system_info_fast()
            
            status = {
                'ros_info': ros_info,
                'sys_info': sys_info,
                'timestamp': str(datetime.now())
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(status, indent=2).encode())
        else:
            self.send_response(404)
            self.end_headers()

if __name__ == '__main__':
    server = HTTPServer(('0.0.0.0', 8080), StatusHandler)
    print("Status server running on http://YOUR_PI_IP:8080/status")
    server.serve_forever()
EOF

chmod +x ~/argo/launch/web_status.py
```

---

## ⚡ Quick Test Commands

### Test X11 forwarding:
```bash
# After SSH with -X, test with:
xclock
# If a clock window appears, X11 forwarding works!
```

### Test GUI import:
```bash
ssh -X orangepi@YOUR_IP
cd ~/argo/launch
python3 -c "import tkinter; tkinter.Tk().mainloop()"
# Should show empty window
```

---

## 🎯 Recommendations

1. **For fast networks**: Use SSH X11 forwarding (Method 1)
2. **For slow/unreliable networks**: Use VNC (Method 2)  
3. **For monitoring only**: Use web status API (Method 3)
4. **For advanced users**: Combine with your existing Foxglove setup

The GUI will work seamlessly over any of these methods while maintaining all its real-time monitoring capabilities!



