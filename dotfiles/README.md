# Argo Dotfiles

Custom bash configuration and CLI tools for Argo sailboat autonomous control system.

## Installation

```bash
cd ~/argo
make install-argo-cli
source ~/.bashrc
```

This will add `source ~/argo/dotfiles/bashrc` to your `~/.bashrc` file.

## What's Included

### Bash Aliases (`dotfiles/bash_aliases`)
Quick commands for Argo robot control:
- `al` - Start Argo nodes
- `aq` - Stop Argo nodes  
- `ars` - Restart all Argo launch nodes (`aq && al`), or one node: `ars argo_web_dashboard`
- `as` - Show Argo status
- `am` - Monitor mode
- `ar` - Start recording
- `ac` - Stop recording
- `abat` - Get battery status (formatted JSON)
- `astore` - Toggle storage rundown (enable/disable discharge to 7.6V then shut down; mode cleared on reboot)
- `ah` - Show argo help
- `ag` - Start argo GUI
- `alog` - Show argo launch logs
- `asim` - Local simulation
- `asimr` - Remote simulation

### Bash Functions (`dotfiles/bashrc`)
- `argo_status()` - Manual status check (always shows full details)
- `argo_quick_timer()` - Throttled status checking (5+ minutes since last check)

### Tab Completion (`dotfiles/.bash_completion_argo`)
Intelligent argument completion for all Argo Python scripts:

**Completion works for BOTH invocation methods:**
- `python3 script.py --<TAB>` (via global argcomplete)
- `./script.py --<TAB>` (via global argcomplete)

**Scripts with completion enabled:**
- ✅ `launch/argo_lifecycle_manager.py` - Commands: run, stop, restart, status, simulate_local, simulate_remote
- ✅ `nodes/bno085.py` - Commands: bridge, calibrate, verify, status
- ✅ `nodes/controller.py` - Full argument completion
- ✅ `nodes/anem.py` - Full argument completion
- ✅ `nodes/gps.py` - Full argument completion
- ✅ `nodes/argo_battery_water.py` - Full argument completion
- ✅ `nodes/argo_unified_simulator_bridge.py` - Full argument completion
- ✅ `nodes/rudder_sail_radio.py` - Full argument completion
- ✅ `nodes/temp_monitor.py` - Full argument completion
- ✅ `nodes/lora.py` - Full argument completion
- ✅ `power_control/argo_power_control.py` - Full argument completion

### Tmux Configuration (`dotfiles/.tmux.conf`)
Custom tmux settings for terminal multiplexing.

### Compass calibration
With BNO085, calibration is stored on the sensor itself (on-chip) and does not
use a repo-managed JSON calibration file.

## Tab Completion Setup

### How It Works
The tab completion system uses Python's `argcomplete` library:

1. **One-time hook**: Run `activate-global-python-argcomplete3 --user` (or `activate-global-python-argcomplete --user`) once so `~/.bash_completion.d/python-argcomplete.sh` exists. Optional: `ARGO_BOOTSTRAP_ARGCOMPLETE=1 bash` runs the same from `.bash_completion_argo`. The script only **sources** the hook if present so every login stays fast.
2. **Auto-Detection**: Scripts with the `PYTHON_ARGCOMPLETE_OK` marker are automatically detected
3. **Works Everywhere**: Completion works with `python3 script.py` or `./script.py`

### Requirements for Scripts
Scripts need 3 elements for completion to work:

```python
#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK    # Marker for global argcomplete

import argparse
import argcomplete          # After argparse import

def main():
    parser = argparse.ArgumentParser(...)
    # ... add arguments ...
    
    argcomplete.autocomplete(parser)  # Before parse_args()
    args = parser.parse_args()
```

### Adding Completion to New Scripts
Use the helper script to automatically add all required elements:

```bash
bash scripts/add_argcomplete_to_script.sh path/to/script.py
```

This will:
1. Add `# PYTHON_ARGCOMPLETE_OK` marker (line 2)
2. Add `import argcomplete` (after argparse import)
3. Add `argcomplete.autocomplete(parser)` (before parse_args())

## Testing Tab Completion

Open a new terminal (or run `source ~/.bashrc`) and try:

```bash
# Lifecycle manager commands
python3 launch/argo_lifecycle_manager.py <TAB>
# Shows: run stop restart status simulate_local simulate_remote

# IMU node commands/args
python3 nodes/bno085.py --<TAB>

# Power control options
python3 power_control/argo_power_control.py --sim<TAB>
# Completes: --simulate-double-tap --simulate-triple-tap --simulate-critical-battery --simulate-low-battery

# Direct invocation also works
./nodes/bno085.py --<TAB>
```

## Troubleshooting

### Completion Not Working

1. **Verify argcomplete is installed:**
   ```bash
   pip3 show argcomplete
   ```

2. **Check if global argcomplete is set up:**
   ```bash
   ls -la ~/.bash_completion.d/python-argcomplete.sh
   ```
   If missing, it will be auto-installed when you reload bash.

3. **Reload bash configuration:**
   ```bash
   source ~/.bashrc
   ```

4. **Verify script has the marker:**
   ```bash
   head -3 nodes/bno085.py | grep PYTHON_ARGCOMPLETE_OK
   ```

### Performance

- Completion runs only when you press TAB
- Takes ~100-200ms (acceptable for interactive use)
- Uses efficient argparse structure parsing

## PATH Configuration

The dotfiles automatically add Argo directories to PATH:
```bash
PATH="$HOME/argo/launch:$HOME/argo/nodes:$HOME/argo/scripts:$PATH"
```

This allows you to run scripts from anywhere.

## Updates and Maintenance

### Update Dotfiles
```bash
cd ~/argo
git pull  # Get latest dotfiles
source ~/.bashrc  # Reload configuration
```

### Add New Aliases
Edit `dotfiles/bash_aliases` and run:
```bash
make aliases-install
source ~/.bashrc
```

### Add Completion to New Script
```bash
bash scripts/add_argcomplete_to_script.sh path/to/new_script.py
source ~/.bashrc
```

## Files

- `bash_aliases` - Argo command aliases
- `.bash_completion_argo` - Tab completion setup
- `.bashrc` - Main bash configuration (sources other files)
- `.tmux.conf` - Tmux configuration
- (No repo-managed IMU calibration JSON with BNO085)
- `README.md` - This file
