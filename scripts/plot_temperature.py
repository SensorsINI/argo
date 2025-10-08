#!/usr/bin/env python3
"""
Temperature Log Plotter
Parses and plots temperature data from /var/log/temperature.log
"""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import re
import numpy as np

def parse_temperature_log(log_file_path):
    """Parse the temperature log file and extract data"""
    timestamps = []
    gpu_temps = []
    ve_temps = []
    cpu_temps = []
    ddr_temps = []
    
    with open(log_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
                
            # Parse timestamp and temperature data
            # Format: 2025-09-11 00:00:31,GPU:66.2°C,VE:65.0°C,CPU:67.6°C,DDR:65.0°C
            parts = line.split(',')
            if len(parts) != 5:
                continue
                
            # Parse timestamp
            timestamp_str = parts[0]
            try:
                timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
                timestamps.append(timestamp)
            except ValueError:
                continue
            
            # Parse temperature values
            temp_pattern = r'(\w+):(\d+\.?\d*)°C'
            temps = {}
            
            for part in parts[1:]:
                match = re.match(temp_pattern, part)
                if match:
                    component, temp = match.groups()
                    temps[component] = float(temp)
            
            # Extract temperatures for each component
            gpu_temps.append(temps.get('GPU', 0))
            ve_temps.append(temps.get('VE', 0))
            cpu_temps.append(temps.get('CPU', 0))
            ddr_temps.append(temps.get('DDR', 0))
    
    return timestamps, gpu_temps, ve_temps, cpu_temps, ddr_temps

def create_temperature_plot(timestamps, gpu_temps, ve_temps, cpu_temps, ddr_temps):
    """Create a comprehensive temperature plot"""
    
    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    
    # Plot 1: All temperatures on one graph
    ax1.plot(timestamps, gpu_temps, label='GPU', linewidth=1.5, alpha=0.8)
    ax1.plot(timestamps, ve_temps, label='VE', linewidth=1.5, alpha=0.8)
    ax1.plot(timestamps, cpu_temps, label='CPU', linewidth=1.5, alpha=0.8)
    ax1.plot(timestamps, ddr_temps, label='DDR', linewidth=1.5, alpha=0.8)
    
    ax1.set_title('System Temperature Monitoring - All Components', fontsize=16, fontweight='bold')
    ax1.set_ylabel('Temperature (°C)', fontsize=12)
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(50, 90)  # Set reasonable temperature range
    
    # Format x-axis
    ax1.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    ax1.xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.setp(ax1.xaxis.get_majorticklabels(), rotation=45)
    
    # Plot 2: Individual component plots
    components = [
        ('GPU', gpu_temps, 'red'),
        ('VE', ve_temps, 'blue'),
        ('CPU', cpu_temps, 'green'),
        ('DDR', ddr_temps, 'orange')
    ]
    
    for i, (name, temps, color) in enumerate(components):
        ax2.plot(timestamps, temps, label=name, color=color, linewidth=2, alpha=0.8)
    
    ax2.set_title('Individual Component Temperatures', fontsize=16, fontweight='bold')
    ax2.set_xlabel('Time', fontsize=12)
    ax2.set_ylabel('Temperature (°C)', fontsize=12)
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(50, 90)
    
    # Format x-axis
    ax2.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    ax2.xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45)
    
    # Add statistics text
    stats_text = f"""
Temperature Statistics:
GPU: Min={min(gpu_temps):.1f}°C, Max={max(gpu_temps):.1f}°C, Avg={np.mean(gpu_temps):.1f}°C
VE:  Min={min(ve_temps):.1f}°C, Max={max(ve_temps):.1f}°C, Avg={np.mean(ve_temps):.1f}°C
CPU: Min={min(cpu_temps):.1f}°C, Max={max(cpu_temps):.1f}°C, Avg={np.mean(cpu_temps):.1f}°C
DDR: Min={min(ddr_temps):.1f}°C, Max={max(ddr_temps):.1f}°C, Avg={np.mean(ddr_temps):.1f}°C
    """
    
    fig.text(0.02, 0.02, stats_text, fontsize=9, verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    return fig

def main():
    """Main function to parse and plot temperature data"""
    log_file_path = '/var/log/temperature.log'
    
    print("Parsing temperature log...")
    timestamps, gpu_temps, ve_temps, cpu_temps, ddr_temps = parse_temperature_log(log_file_path)
    
    if not timestamps:
        print("No valid temperature data found in the log file.")
        return
    
    print(f"Found {len(timestamps)} temperature readings")
    print(f"Time range: {timestamps[0]} to {timestamps[-1]}")
    
    print("Creating temperature plot...")
    fig = create_temperature_plot(timestamps, gpu_temps, ve_temps, cpu_temps, ddr_temps)
    
    # Save the plot under repo root derived from this file
    import os
    repo_root = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(repo_root, 'temperature_plot.png')
    fig.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Temperature plot saved to: {output_file}")
    
    # Also save as PDF for better quality
    output_pdf = os.path.join(repo_root, 'temperature_plot.pdf')
    fig.savefig(output_pdf, bbox_inches='tight')
    print(f"Temperature plot also saved as PDF: {output_pdf}")
    
    # Don't show the plot in headless mode
    # plt.show()

if __name__ == "__main__":
    main()
