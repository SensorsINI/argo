# Argo Shore Station for Windows 10/11

Complete guide for running the Argo shore station (LoRa receiver + Web Dashboard) on Windows.

---

## Quick Answer

**Q: Can I get a one-click .exe for Windows?**

**A: Not easily** - both scripts require ROS2 (Robot Operating System 2), which is a complex Linux-based system (several GB) that cannot be bundled into a simple Windows executable.

**But we have something almost as good**: A simple setup script that does everything automatically, then true one-click daily operation.

---

## 🌟 Recommended Approach: WSL2 + Setup Script

This is the **simplest and best** solution for Windows 10/11.

### Why This Approach?

| Benefit | Description |
|---------|-------------|
| **Simple Setup** | 3 commands, mostly automatic (~10-15 minutes one-time) |
| **Easy Daily Use** | One command to launch |
| **Full Functionality** | Complete ROS2 support, USB LoRa device works perfectly |
| **Easy Updates** | `git pull` to get latest code |
| **Maintainable** | Setup script is version-controlled in the repo |
| **Portable** | Works on any Windows 10/11 computer |

---

## Complete Setup Guide

### Prerequisites

- Windows 10 version 2004+ or Windows 11
- Internet connection (for initial setup)
- Waveshare LoRa USB device

---

### Step 1: Install WSL2 (5 minutes)

Open **PowerShell as Administrator** (right-click, "Run as administrator"):

```powershell
wsl --install -d Ubuntu-22.04
```

**Restart your computer** when prompted. After restart, Ubuntu will open automatically - create your username and password.

<details>
<summary>Troubleshooting: WSL2 won't install</summary>

If WSL2 fails to install:

1. **Check Windows version**: Press `Win+R`, type `winver`, press Enter. You need version 2004 or later.

2. **Enable virtualization in BIOS**: Restart computer, enter BIOS (usually F2, F12, or Delete key), enable VT-x/AMD-V.

3. **Enable Windows features**:
   ```powershell
   dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
   dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
   ```
   Then restart and try again.

</details>

---

### Step 2: Clone Argo Repository (30 seconds)

Open **Ubuntu** from the Windows Start menu:

```bash
cd ~
git clone https://github.com/SensorsINI/argo.git
cd argo
```

---

### Step 3: Run Setup Script (5-10 minutes, automatic)

Still in the Ubuntu terminal:

```bash
./shore/setup_shore_station.sh
```

**This script automatically**:
- ✅ Checks for ROS2 Humble (installs if missing)
- ✅ Checks for Python dependencies (installs if missing)  
- ✅ Configures ROS2 environment
- ✅ Sets up USB serial device access
- ✅ Creates launcher script
- ✅ Verifies installation

You may be prompted for your password (for `sudo` operations). Just let it run!

<details>
<summary>What if the script isn't executable?</summary>

If you get "Permission denied", make it executable first:

```bash
chmod +x shore/setup_shore_station.sh
./shore/setup_shore_station.sh
```

</details>

---

### Step 4: Setup USB Device Passthrough (One-Time)

WSL2 needs access to your USB LoRa device. Install the USB passthrough tool.

In **Windows PowerShell** (regular, not Administrator):

```powershell
# Install usbipd-win
winget install --interactive --exact dorssel.usbipd-win
```
You can also install the windows GUI for managing usbipd from https://gitlab.com/alelec/wsl-usb-gui

**Important**: After installation, **close and reopen PowerShell** to refresh the PATH environment variable. The `usbipd` command won't be recognized in the current session.

**Connect your Waveshare LoRa USB device**, then (in the new PowerShell window):

```powershell
# List all USB devices
usbipd.exe list
```

You should see output like this (your BUSID and COM port may differ):

```
Connected:
BUSID  VID:PID    DEVICE                                                        STATE
1-2    1a86:55d3  USB-Enhanced-SERIAL CH343 (COM9)                              Not shared
1-5    8087:0a2b  Intel(R) Wireless Bluetooth(R)                                Not shared
1-7    05c8:03c0  HD Camera                                                     Not shared
```

**Find your LoRa device** - it will be "USB-Enhanced-SERIAL CH343" (COM port can vary). Note the BUSID (e.g., 1-2 in the example above).

Now bind and attach the device:

```powershell
# Bind the device (one-time setup)
usbipd.exe bind --busid 1-2

# Attach to WSL2 (needed each time Windows restarts)
usbipd.exe attach --wsl --busid 1-2
```
After binding, `usbipd.exe list` should show the USB-to-LoRa device as **"Shared"**. After attaching, it should show as **"Attached"**.

<details>
<summary>Prefer a GUI? (Optional)</summary>

If you prefer a graphical interface instead of command line, you can use a third-party GUI tool:

**WSL USB GUI** (Community Project):
- Download: https://gitlab.com/alelec/wsl-usb-gui/-/releases
- Provides point-and-click interface for binding and attaching USB devices
- Note: This is a community project, not officially supported by Microsoft

The command-line tool (`usbipd.exe`) is faster and more reliable for scripting and daily use.

</details>

<details>
<summary>Getting "Access denied" error when binding?</summary>

**Problem**: 
```
usbipd: error: Access denied; this operation requires administrator privileges.
```

**Solution**: The `bind` command requires administrator privileges. Right-click on **Windows Terminal** or **PowerShell** in the Start menu and select **"Run as administrator"**, then run the bind command again:

```powershell
usbipd.exe bind --busid 1-2
```

**Note**: You only need administrator rights for the `bind` command. The `attach` command can run without administrator privileges.

</details>

<details>
<summary>Getting "usbipd command not found" error?</summary>

**Problem**: After installing usbipd-win, PowerShell says `usbipd : The term 'usbipd' is not recognized...`

**Solution**: Close PowerShell completely and open a new PowerShell window. The PATH environment variable needs to be refreshed to include the newly installed program.

**Alternative**: If you don't want to close PowerShell, you can use the full path:
```powershell
& "C:\Program Files\usbipd-win\usbipd.exe" list
```

</details>

Verify it's attached in Ubuntu:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
# Should show: /dev/ttyACM0 or /dev/ttyUSB0
```

**Note**: The CH343 device usually appears as `/dev/ttyACM0` in WSL2. If you see `/dev/ttyACM0`, that's correct!

<details>
<summary>USB device not showing up in WSL?</summary>

**Problem**: `ls /dev/ttyUSB*` shows "No such file or directory"

This is the most common issue. Follow these diagnostic steps:

### Step 1: Verify device is attached (not just bound)

In **PowerShell**, run:
```powershell
usbipd.exe list
```

Check the STATE column for your device:
- **"Shared"** = Device is bound but NOT attached to WSL
- **"Attached"** = Device is attached to WSL (this is what you want!)

If it shows **"Shared"** instead of **"Attached"**, the device was only bound but not attached. Run:
```powershell
usbipd.exe attach --wsl --busid 1-2
```

### Step 2: If attach command fails or hangs

Try detaching first, then re-attaching:
```powershell
# Detach (if previously attached)
usbipd.exe detach --busid 1-2

# Wait 2 seconds, then attach again
usbipd.exe attach --wsl --busid 1-2
```

### Step 3: Check if WSL2 can see USB devices

In **Ubuntu terminal**, run:
```bash
lsusb
```

You should see your device listed (look for "1a86:55d3" or "CH343"):
```
Bus 001 Device 002: ID 1a86:55d3 QinHeng Electronics USB-Enhanced-SERIAL CH343
```

If `lsusb` shows nothing or gives an error, your WSL2 kernel may not have USB support. Update WSL2:
```powershell
# In PowerShell
wsl --update
wsl --shutdown
```

Then restart Ubuntu and try attaching again.

### Step 4: Check kernel modules

In **Ubuntu terminal**, verify USB serial drivers are loaded:
```bash
lsmod | grep -E 'usbserial|ch341'
```

If no output, try loading the driver manually:
```bash
sudo modprobe usbserial
sudo modprobe ch341
```

### Step 5: Check dmesg for errors

In **Ubuntu terminal**, check system logs:
```bash
dmesg | tail -20
```

Look for USB-related messages. You should see something like:
```
usb 1-1: new full-speed USB device number 2 using vhci_hcd
ch341 1-1:1.0: ch341-uart converter detected
usb 1-1: ch341-uart converter now attached to ttyUSB0
```

### Step 6: Other common issues

1. **Device is busy in Windows**: Close any Windows programs that might be using the COM port (Device Manager, PuTTY, Arduino IDE, etc.)

2. **Try a different USB port**: Some USB ports work better than others with WSL2

3. **Restart WSL2 completely**:
   ```powershell
   # In PowerShell
   wsl --shutdown
   # Wait 5 seconds, then restart Ubuntu from Start menu
   ```

4. **Unplug and replug the device**: Physically disconnect and reconnect the USB device, then run attach again

**Note**: You need to run `usbipd.exe attach --wsl --busid YOUR_BUSID` each time you restart Windows or WSL2. The `bind` command only needs to be done once.

</details>

---

### Step 5: Launch Shore Station 🚀

In **Ubuntu terminal**:

```bash
~/argo/shore/launch_shore_station.sh
```

The launcher will:
- Automatically detect your LoRa USB port
- Start the LoRa receiver
- Start the web dashboard
- Show you the URL to access

**Open your browser** to: **http://localhost:8081**

**To stop**: Press `Ctrl+C` in the terminal.

---

## Daily Use

After the one-time setup, daily operation is simple:

1. **Attach USB device** (if Windows was restarted):
   ```powershell
   # In PowerShell
   usbipd.exe attach --wsl --busid 1-2
   ```

2. **Launch shore station**:
   ```bash
   # In Ubuntu terminal
   ~/argo/shore/launch_shore_station.sh
   ```

3. **Access dashboard**: http://localhost:8081

### Optional: Create a Convenient Alias

Add to `~/.bashrc` in Ubuntu:

```bash
alias argo_shore='~/argo/shore/launch_shore_station.sh'
```

Then source it:
```bash
source ~/.bashrc
```

Now you can just type:
```bash
argo_shore
```

---

## Updating Argo Code

When there are updates to the Argo repository:

```bash
cd ~/argo
git pull
```

That's it! The setup script will automatically use the new code.

---

## Accessing from Your Phone

To access the web dashboard from your phone on the same WiFi network:

1. **Find your computer's IP address**:
   ```bash
   # In Ubuntu
   hostname -I
   # Shows: 192.168.1.100 (example)
   ```

2. **Open on phone**: http://192.168.1.100:8081

---

## Troubleshooting

### Setup Issues

#### "wsl: command not found"

WSL2 requires Windows 10 version 2004+ or Windows 11. Run `winver` to check your version and update Windows if needed.

#### "ROS2 installation failed"

The setup script will show the error. Common causes:
- No internet connection
- Disk space full (need ~3-5 GB free)
- Ubuntu package servers temporarily down

Try running the setup script again: `./shore/setup_shore_station.sh`

#### "Python dependencies failed to install"

Manually install:
```bash
pip3 install pyserial flask flask-cors
```

### USB Issues

#### "No USB serial device found"

1. Check device is plugged into Windows
2. Verify it's attached to WSL2:
   ```powershell
   # PowerShell
   usbipd.exe list
   usbipd.exe attach --wsl --busid YOUR_BUSID
   ```
3. Check in Ubuntu:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   # Should show /dev/ttyACM0 or /dev/ttyUSB0
   ```

#### "Permission denied: /dev/ttyUSB0"

Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

Then **log out of Ubuntu and back in** (or restart WSL):
```powershell
# PowerShell
wsl --shutdown
# Then reopen Ubuntu
```

### Runtime Issues

#### "Port 8081 already in use"

Stop existing dashboard:
```bash
pkill -f argo_web_dashboard
```

#### "LoRa receiver not receiving packets"

1. Check LoRa device is attached: `ls /dev/ttyUSB*`
2. Check device permissions: `groups` should include "dialout"
3. Try different serial port: edit launcher script and change port
4. Check LoRa configuration matches Argo boat settings

### WSL2 Issues

#### "WSL2 is slow"

1. Ensure virtualization is enabled in BIOS
2. Allocate more resources in `.wslconfig`:
   ```
   # Create: C:\Users\YourName\.wslconfig
   [wsl2]
   memory=4GB
   processors=2
   ```
3. Restart WSL: `wsl --shutdown` in PowerShell

#### "Can't access Windows files from Ubuntu"

Windows drives are mounted at `/mnt/`:
- `C:\` → `/mnt/c/`
- `D:\` → `/mnt/d/`
- `F:\` → `/mnt/f/`

---

## Alternative Approaches

### Option 1: Automated Batch Installer (Legacy)

Before the setup script existed, we had Windows batch files that automate the process. These still work:

1. Run `install_wsl2_setup.bat` as Administrator
2. Follow prompts
3. Use `launch_argo_shore.bat` for daily operation

**Note**: The setup script approach (recommended above) is simpler and more maintainable.

### Option 2: Docker Desktop

If you prefer Docker:

1. Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
2. Run `launch_docker.bat`

**Caveat**: USB device passthrough on Windows Docker is complex. WSL2 is better for USB serial devices.

### Option 3: Standalone .exe (Not Yet Available)

A true standalone Windows executable without any dependencies would require:
- 2-3 days development work
- Removing all ROS2 dependencies
- Replacing with REST API communication
- ~100-300 MB portable package
- Some ROS2 integration features unavailable

Let us know if you need this option - it's a separate project.

---

## Verification

After setup, verify everything is working:

**Option A**: Run the setup script again
```bash
./shore/setup_shore_station.sh
```
It will check and report status of all components.

**Option B**: Use the verification batch file
```
Double-click: check_setup.bat
```
(Windows tool that checks WSL2, ROS2, files, etc.)

All checks should show ✓ (green checkmarks).

---

## Files in This Directory

| File | Purpose | When to Use |
|------|---------|-------------|
| **README.md** | This file - complete guide | **Read this first** |
| `launch_argo_shore.bat` | Windows launcher (legacy) | Alternative to terminal commands |
| `install_wsl2_setup.bat` | Automated setup wizard (legacy) | Alternative to manual setup |
| `check_setup.bat` | Setup verification tool | After setup to verify |
| `create_desktop_shortcut.bat` | Desktop icon creator | Optional convenience |
| `launch_docker.bat` | Docker launcher | If using Docker approach |
| `Dockerfile` | Docker container definition | If using Docker approach |
| `WHY_GIT_CLONE.md` | Technical explanation | For developers/curious users |

---

## Architecture

### Current System (WSL2 + Setup Script)

```
Windows 10/11
   ↓
WSL2 (Ubuntu 22.04) ← One-time setup with setup_shore_station.sh
   ↓
ROS2 Humble (installed by setup script)
   ↓
lora_shore.py + argo_web_dashboard.py (launched by launcher script)
   ↓
Web Browser → http://localhost:8081
```

### Why WSL2 and Not Native Windows?

| Aspect | WSL2 | Native Windows ROS2 |
|--------|------|---------------------|
| Setup time | 10-15 min (automated) | 2-3 hours (complex) |
| Compatibility | Perfect (it's Linux) | Many issues |
| Size | ~2-3 GB | ~5-8 GB |
| USB support | Excellent | Good |
| Maintenance | Easy (`git pull`) | Complex |
| Performance | Native speed | Native speed |
| Stability | Excellent | Many compatibility issues |

WSL2 is Microsoft's recommended way to run Linux applications on Windows.

---

## Performance

The WSL2 solution performs excellently:

- **Startup time**: ~5-10 seconds
- **No performance penalty** vs native Linux
- **USB serial communication**: Full speed
- **Web dashboard**: Fully responsive
- **Memory usage**: ~500 MB total
- **CPU usage**: Minimal when idle

---

## System Requirements

### Minimum

- Windows 10 version 2004 (May 2020 Update) or Windows 11
- 4 GB RAM (8 GB recommended)
- 10 GB free disk space
- x64 processor with virtualization support

### Recommended

- Windows 11
- 8 GB+ RAM
- 20 GB+ free disk space
- SSD for better performance

---

## Summary

**For most users**, follow this simple process:

```bash
# One-time setup (10-15 minutes)
wsl --install -d Ubuntu-22.04          # PowerShell as Admin, then restart
cd ~ && git clone https://github.com/SensorsINI/argo.git
cd argo && ./shore/setup_shore_station.sh
usbipd.exe bind --busid 1-2            # PowerShell as Admin (one-time)
usbipd.exe attach --wsl --busid 1-2    # PowerShell (after binding)

# Daily use (30 seconds)
usbipd.exe attach --wsl --busid 1-2    # PowerShell (if Windows restarted)
~/argo/shore/launch_shore_station.sh   # Ubuntu terminal
# Open browser: http://localhost:8081
```

**4 commands for setup, 2 commands for daily use!**

---

## Getting Help

1. **Check troubleshooting section** above for common issues
2. **Run verification**: `./shore/setup_shore_station.sh` (reports status)
3. **Check logs**: The launcher script shows error messages
4. **Verify USB device**: `ls /dev/ttyUSB* /dev/ttyACM*` should show your device

---

## For Developers

If you're modifying the Windows setup:

- **Primary method**: The `setup_shore_station.sh` script in the repo
- **Windows tools**: Batch files in this directory are now optional/legacy
- **Testing**: Test on fresh Windows 10 and Windows 11 VMs
- **Documentation**: Update this README for any new features

The setup script approach is preferred because:
- ✅ Version controlled in the main repo
- ✅ Works on Ubuntu, WSL2, Raspberry Pi, etc.
- ✅ Single source of truth
- ✅ Easier to maintain and update

---

## License

This project follows the main Argo repository license.

---

## Changelog

### v2.0 (Current) - Simplified Setup
- Added `setup_shore_station.sh` automated setup script
- Simplified to 3-command setup process
- Git clone instead of file copying (easier updates)
- Removed redundant documentation

### v1.0 - Initial Windows Support
- Created Windows batch file installers
- Detailed step-by-step manual setup
- File copying from Windows to WSL2
