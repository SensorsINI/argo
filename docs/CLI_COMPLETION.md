# CLI Tab Completion for Argo Python Scripts

## Overview

Argo uses **argcomplete** for efficient bash tab completion. This provides intelligent completion for command-line arguments without launching Python on every keypress.

## How It Works

- **Efficient**: Only invokes Python when you press TAB, not on every command
- **Smart**: Understands argparse structure for accurate completions  
- **Fast**: Uses bash's native completion framework with minimal overhead
- **Zero Config**: Works automatically once enabled in scripts

## Enabling Completion in Your Script

Add just **2 lines** to any Python script using `argparse`:

```python
#!/usr/bin/env python3

import argparse
import argcomplete  # ADD THIS LINE

def main():
    parser = argparse.ArgumentParser(description='Your script description')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    parser.add_argument('--config', type=str, help='Config file path')
    
    argcomplete.autocomplete(parser)  # ADD THIS LINE (before parse_args)
    args = parser.parse_args()
    
    # ... rest of your code
```

### Key Points

1. **Import after argparse**: `import argcomplete`
2. **Call before parse_args()**: `argcomplete.autocomplete(parser)`
3. **That's it!** No other changes needed

## Testing Completion

After adding argcomplete to a script:

```bash
# Reload bash configuration
source ~/.bashrc

# Test completion (press TAB after the script name)
python3 nodes/bno085.py --<TAB>         # Shows: bridge, calibrate, verify, status, --help, ...
```

## Performance Notes

**Q: Isn't launching Python slow?**

A: argcomplete is designed for this:
- Only runs when you press TAB (not on every command)
- Completes in ~100-200ms (acceptable for tab completion)
- Caches completion scripts when possible
- Uses fast parsing of argparse structure

**Q: Can I make it faster?**

A: For very frequently used scripts, you can:
1. Pre-generate static completion files (complex setup)
2. Use the current solution (already fast enough for most use cases)

The current approach is the best balance of simplicity and performance.

## Scripts Already Enabled

The following scripts have argcomplete enabled:
- ✅ `launch/argo_lifecycle_manager.py` - Commands: run, start, stop, restart, status, monitor, simulate_local, simulate_remote
- ✅ `nodes/bno085.py` - Commands: bridge, calibrate, verify, status

### To Enable on Other Scripts

Check these scripts that use argparse and would benefit:
- `nodes/controller.py`
- `nodes/gps.py`
- `nodes/anem.py`
- `nodes/argo_battery_water.py`
- `nodes/rudder_sail_radio.py`
- `nodes/temp_monitor.py`
- `nodes/lora.py`
- `nodes/argo_unified_simulator_bridge.py`

## Completion Configuration

The completion system is configured in:
- **Config**: `dotfiles/.bash_completion_argo` - Registers Python scripts
- **Activation**: `dotfiles/bashrc` - Sources completion config
- **Installation**: `make install-argo-cli` - Installs dotfiles

## Advanced Features

### Positional Arguments

```python
parser.add_argument('command', choices=['start', 'stop', 'restart'])
argcomplete.autocomplete(parser)
```

Tab completion will suggest: `start`, `stop`, `restart`

### Dynamic Completions

```python
def file_completer(prefix, **kwargs):
    return ['file1.txt', 'file2.txt', 'file3.txt']

parser.add_argument('--file').completer = file_completer
argcomplete.autocomplete(parser)
```

### Conditional Completions

```python
parser.add_argument('--mode', choices=['local', 'remote'])
parser.add_argument('--host')  # Only relevant when mode=remote
argcomplete.autocomplete(parser)
```

argcomplete automatically handles conditional argument visibility.

## Troubleshooting

### Completion Not Working

```bash
# Check if argcomplete is installed
pip3 show argcomplete

# Check if completion file is loaded
type register-python-argcomplete

# Reload bash configuration
source ~/.bashrc

# Test directly
eval "$(register-python-argcomplete imu.py)"
python3 imu.py --<TAB>
```

### Still Not Working

1. Verify script has argcomplete lines added
2. Check script is executable: `chmod +x script.py`
3. Ensure script is in PATH or use full path
4. Test with simple example first

## References

- **argcomplete docs**: https://kislyuk.github.io/argcomplete/
- **Argo completion config**: `dotfiles/.bash_completion_argo`
- **Installation**: Run `make install-argo-cli` from project root
