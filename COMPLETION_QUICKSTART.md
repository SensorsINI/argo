# CLI Tab Completion - Quick Start

## Instant Setup

1. **Reload bash configuration:**
   ```bash
   source ~/.bashrc
   ```

2. **Test completion immediately:**
   ```bash
   # Tab completion for argo_lifecycle_manager
   python3 launch/argo_lifecycle_manager.py <TAB>
   # Shows: run stop restart status simulate_local simulate_remote
   
   python3 launch/argo_lifecycle_manager.py --<TAB>
   # Shows: --help --debug --toggle_pause
   
   # Tab completion for imu.py
   python3 nodes/imu.py --<TAB>
   # Shows: --help --debug --calib_compass --plot_calib
   ```

## How to Add to Other Scripts

Add just **2 lines** to any Python script with argparse:

```python
import argparse
import argcomplete  # ADD THIS

def main():
    parser = argparse.ArgumentParser(...)
    parser.add_argument('--debug', ...)
    
    argcomplete.autocomplete(parser)  # ADD THIS (before parse_args)
    args = parser.parse_args()
```

## Scripts Ready for Completion

✅ **Already enabled:**
- `launch/argo_lifecycle_manager.py` 
- `nodes/imu.py`

📝 **Easy to enable** (just add 2 lines):
- `nodes/controller.py`
- `nodes/gps.py`
- `nodes/anem.py`
- `nodes/battery_water.py`
- `nodes/rudder_sail_radio.py`
- `nodes/temp_monitor.py`
- `nodes/lora.py`
- `nodes/argo_unified_simulator_bridge.py`

## Performance

- **When**: Only runs when you press TAB
- **Speed**: ~100-200ms (imperceptible for tab completion)
- **No overhead**: Zero cost when not using tab completion

## Full Documentation

See `docs/CLI_COMPLETION.md` for detailed information.

