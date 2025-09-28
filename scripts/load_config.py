#!/usr/bin/env python3
"""
Configuration loader for remote simulator
Loads JSON configuration and provides access for both Python and shell scripts
"""

import json
import os
import sys
import subprocess

def get_config_file_path():
    """Get the path to the configuration file"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, 'remote_simulator_config.json')

def load_config():
    """Load configuration from JSON file"""
    config_file = get_config_file_path()
    
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Configuration file not found: {config_file}")
    
    with open(config_file, 'r') as f:
        config = json.load(f)
    
    # Expand paths
    if 'ssh' in config and 'key_path' in config['ssh']:
        config['ssh']['key_path'] = os.path.expanduser(config['ssh']['key_path'])
    
    return config

def get_config_value(key_path, default=None):
    """Get a configuration value using dot notation (e.g., 'remote.host')"""
    config = load_config()
    
    keys = key_path.split('.')
    value = config
    
    try:
        for key in keys:
            value = value[key]
        return value
    except (KeyError, TypeError):
        return default

def export_config_to_shell():
    """Export configuration as shell environment variables"""
    config = load_config()
    
    # Flatten nested config to environment variables
    env_vars = {}
    
    # Remote config
    if 'remote' in config:
        env_vars['REMOTE_HOST'] = config['remote']['host']
        env_vars['REMOTE_USER'] = config['remote']['user']
        env_vars['REMOTE_ARGO_DIR'] = config['remote']['argo_dir']
    
    # ROS2 config
    if 'ros2' in config:
        env_vars['ROS_DOMAIN_ID'] = str(config['ros2']['domain_id'])
    
    # Network config
    if 'network' in config:
        env_vars['LOCAL_PORT'] = str(config['network']['local_port'])
        env_vars['REMOTE_PORT'] = str(config['network']['remote_port'])
    
    # SSH config
    if 'ssh' in config:
        env_vars['SSH_TIMEOUT'] = str(config['ssh']['timeout'])
        env_vars['SSH_KEY_PATH'] = config['ssh']['key_path']
    
    return env_vars

def print_config():
    """Print configuration in a readable format"""
    config = load_config()
    
    print("🚢 Remote Simulator Configuration")
    print("=================================")
    
    def print_section(section_name, section_data, prefix=""):
        print(f"{prefix}{section_name.upper()}:")
        for key, value in section_data.items():
            print(f"{prefix}  {key}: {value}")
        print("")
    
    for section_name, section_data in config.items():
        print_section(section_name, section_data)

def main():
    """Main function for command line usage"""
    if len(sys.argv) > 1:
        if sys.argv[1] == '--export-shell':
            # Export as shell environment variables
            env_vars = export_config_to_shell()
            for key, value in env_vars.items():
                print(f"export {key}='{value}'")
        elif sys.argv[1] == '--get':
            # Get specific value
            if len(sys.argv) > 2:
                value = get_config_value(sys.argv[2])
                if value is not None:
                    print(value)
                else:
                    print(f"Configuration key not found: {sys.argv[2]}", file=sys.stderr)
                    sys.exit(1)
            else:
                print("Usage: --get <key.path>", file=sys.stderr)
                sys.exit(1)
        else:
            print("Usage: load_config.py [--export-shell|--get <key.path>]", file=sys.stderr)
            sys.exit(1)
    else:
        # Print full configuration
        print_config()

if __name__ == '__main__':
    main()


