#!/usr/bin/env python3
"""
Battery Water Data Plotting Script

This script reads CSV data from argo_battery_water.py and creates visualizations
for battery voltage decay, sensor trends, and alert patterns.

Features:
- Standard battery voltage decay plot
- Critical battery analysis with enhanced decay visualization
- Comprehensive sensor trends overview
- Alert pattern analysis

Usage:
    python3 plot-battery-water.py [csv_file_path]
    
If no file path is provided, it will look for today's CSV file in /var/log.hdd/persistent/

Examples:
    python3 plot-battery-water.py
    python3 plot-battery-water.py /var/log.hdd/persistent/battery-monitor-20251005.csv
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import sys
import os
from datetime import datetime, timedelta
import argparse
import gc
import subprocess

# Global font scale parameter for consistent sizing
FONT_SCALE = 2.0  # Base scale - can be adjusted for different display sizes

# Font size constants based on scale
TITLE_FONT_SIZE = int(14 * FONT_SCALE)
SUBTITLE_FONT_SIZE = int(12 * FONT_SCALE)
AXIS_FONT_SIZE = int(10 * FONT_SCALE)
LABEL_FONT_SIZE = int(9 * FONT_SCALE)
LEGEND_FONT_SIZE = int(8 * FONT_SCALE)

# Performance settings
MAX_DATA_POINTS = 5000  # Maximum points to plot to prevent OOM
SAMPLE_RATIO = 0.1  # Sample ratio for large datasets


def sample_data_for_plotting(df, max_points=MAX_DATA_POINTS):
    """Sample data to prevent OOM issues with large datasets"""
    if len(df) <= max_points:
        return df

    # Use every nth point to get approximately max_points
    step = len(df) // max_points
    sampled_df = df.iloc[::step].copy()
    print(
        f"Sampled {len(df)} points down to {len(sampled_df)} for plotting (every {step} points)")
    return sampled_df


def plot_with_gaps(ax, timestamps, values, color, linewidth, label, marker=None, markersize=None):
    """Plot data with line breaks at large time gaps"""
    if len(timestamps) < 2:
        return

    # Calculate time differences between consecutive points
    time_diffs = pd.Series(timestamps).diff().dt.total_seconds()

    # Define gap threshold (e.g., 10 minutes = 600 seconds for battery data)
    gap_threshold = 600  # 10 minutes - more appropriate for battery monitoring

    # Find indices where gaps exceed threshold
    gap_indices = time_diffs > gap_threshold

    if not gap_indices.any():
        # No gaps, plot normally
        if marker:
            ax.plot(timestamps, values, color=color, linewidth=linewidth,
                    label=label, marker=marker, markersize=markersize)
        else:
            ax.plot(timestamps, values, color=color,
                    linewidth=linewidth, label=label)
    else:
        # Plot segments between gaps - FIXED: don't include gap point in segments
        start_idx = 0
        segment_num = 0

        for i, is_gap in enumerate(gap_indices):
            if is_gap:
                # Plot segment up to (but not including) this gap point
                end_idx = i  # Don't include the gap point itself
                if end_idx > start_idx:
                    segment_times = timestamps[start_idx:end_idx]
                    segment_values = values[start_idx:end_idx]

                    # Only add label to first segment
                    segment_label = label if segment_num == 0 else None

                    if marker:
                        ax.plot(segment_times, segment_values, color=color, linewidth=linewidth,
                                label=segment_label, marker=marker, markersize=markersize)
                    else:
                        ax.plot(segment_times, segment_values, color=color, linewidth=linewidth,
                                label=segment_label)

                    segment_num += 1

                # Start new segment after gap (skip the gap point)
                start_idx = i + 1

        # Plot final segment if it exists
        if start_idx < len(timestamps):
            segment_times = timestamps[start_idx:]
            segment_values = values[start_idx:]

            segment_label = label if segment_num == 0 else None

            if marker:
                ax.plot(segment_times, segment_values, color=color, linewidth=linewidth,
                        label=segment_label, marker=marker, markersize=markersize)
            else:
                ax.plot(segment_times, segment_values, color=color, linewidth=linewidth,
                        label=segment_label)


def format_time_axis(ax, df):
    """Format time axis with smart tick spacing based on data duration"""
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))

    # Smart tick spacing based on data duration
    time_span = df['timestamp'].max() - df['timestamp'].min()
    if time_span.total_seconds() < 3600:  # Less than 1 hour
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=5))
    elif time_span.total_seconds() < 86400:  # Less than 1 day
        ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))
    else:  # More than 1 day
        ax.xaxis.set_major_locator(mdates.DayLocator(interval=1))
    plt.xticks(rotation=45)

    # Apply font scaling to axis labels
    ax.tick_params(axis='both', which='major', labelsize=LABEL_FONT_SIZE)


def find_latest_csv_files(num_files=3):
    """Find the most recent battery monitor CSV files
    
    Args:
        num_files: Number of recent files to return (default: 3)
        
    Returns:
        List of file paths sorted by modification time (most recent first)
    """
    log_dir = "/var/log.hdd/persistent"
    if not os.path.exists(log_dir):
        return []

    # Look for any battery-monitor CSV files
    csv_files = []
    for filename in os.listdir(log_dir):
        if filename.startswith("battery-monitor-") and filename.endswith(".csv"):
            filepath = os.path.join(log_dir, filename)
            csv_files.append((filepath, os.path.getmtime(filepath)))

    if csv_files:
        # Sort by modification time (most recent first) and return top N
        csv_files.sort(key=lambda x: x[1], reverse=True)
        result = [f[0] for f in csv_files[:num_files]]
        print(f"Found {len(result)} recent CSV file(s):")
        for f in result:
            print(f"  - {f}")
        return result

    return []


def load_battery_data(csv_file_paths):
    """Load and prepare battery data from one or more CSV files
    
    Args:
        csv_file_paths: Single file path (string) or list of file paths
        
    Returns:
        Combined DataFrame with all data sorted by timestamp
    """
    # Ensure we have a list
    if isinstance(csv_file_paths, str):
        csv_file_paths = [csv_file_paths]
    
    all_dfs = []
    
    for csv_file_path in csv_file_paths:
        if not os.path.exists(csv_file_path):
            print(f"Warning: CSV file not found at {csv_file_path}, skipping...")
            continue

        try:
            # Try to read with headers first
            df = pd.read_csv(csv_file_path)

            # Clean column names (remove any whitespace)
            df.columns = df.columns.str.strip()

            # Check if we have the expected columns
            if 'timestamp' not in df.columns:
                # If timestamp column is missing, try reading without headers
                # Updated column names to match current argo_battery_water.py output
                column_names = [
                    'timestamp', 'battery_voltage', 'battery_remaining_pct',
                    'saltwater_voltage', 'sail_current', 'pcb_temperature',
                    'relative_humidity', 'battery_low_alert', 'saltwater_alert',
                    'humidity_alert', 'argo_battery_water_health', 'charging_status', 'ac_power_present'
                ]
                df = pd.read_csv(csv_file_path, header=None, names=column_names)
                print(
                    f"Loaded {len(df)} data points from {os.path.basename(csv_file_path)} (no headers detected)")
            else:
                print(
                    f"Loaded {len(df)} data points from {os.path.basename(csv_file_path)} (headers detected)")

            # Convert timestamp to datetime
            df['timestamp'] = pd.to_datetime(df['timestamp'])
            all_dfs.append(df)

        except Exception as e:
            print(f"Error loading CSV file {csv_file_path}: {e}")
            continue
    
    if not all_dfs:
        print("Error: No valid CSV files could be loaded")
        return None
    
    # Concatenate all dataframes
    combined_df = pd.concat(all_dfs, ignore_index=True)
    
    # Sort by timestamp and remove duplicates
    combined_df = combined_df.sort_values('timestamp')
    combined_df = combined_df.drop_duplicates(subset=['timestamp'], keep='first')
    
    print(f"\nCombined dataset:")
    print(f"  Total data points: {len(combined_df)}")
    print(f"  Date range: {combined_df['timestamp'].min()} to {combined_df['timestamp'].max()}")
    print(f"  Files concatenated: {len(all_dfs)}")
    
    return combined_df


def plot_battery_voltage_decay(df, output_dir):
    """Plot battery voltage over time to show decay patterns"""
    # Sample data to prevent OOM
    plot_df = sample_data_for_plotting(df)

    plt.figure(figsize=(14, 12))  # Increased height for 3 subplots

    # Main voltage plot
    plt.subplot(3, 1, 1)
    plot_with_gaps(plt.gca(), plot_df['timestamp'], plot_df['battery_voltage'],
                   'b', 2, 'Battery Voltage')
    plt.title('Battery Voltage Decay Over Time',
              fontsize=TITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Voltage (V)', fontsize=AXIS_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=LEGEND_FONT_SIZE)

    # Format x-axis
    format_time_axis(plt.gca(), plot_df)

    # Battery percentage plot
    plt.subplot(3, 1, 2)
    plot_with_gaps(plt.gca(), plot_df['timestamp'], plot_df['battery_remaining_pct'],
                   'g', 2, 'Battery %')
    plt.title('Battery State of Charge',
              fontsize=TITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Percentage (%)', fontsize=AXIS_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=LEGEND_FONT_SIZE)

    # Format x-axis
    format_time_axis(plt.gca(), plot_df)

    # Charging and Power Status plot
    plt.subplot(3, 1, 3)

    # Plot charging status and AC power present
    plot_with_gaps(plt.gca(), plot_df['timestamp'], plot_df['charging_status'],
                   'orange', 2, 'Charging Active', marker='o', markersize=3)
    plot_with_gaps(plt.gca(), plot_df['timestamp'], plot_df['ac_power_present'],
                   'purple', 2, 'AC Power Present', marker='s', markersize=3)

    plt.title('Charging and Power Status',
              fontsize=TITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Status (0/1)', fontsize=AXIS_FONT_SIZE)
    plt.xlabel('Time', fontsize=AXIS_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=LEGEND_FONT_SIZE)
    plt.ylim(-0.1, 1.1)  # Set y-axis limits for binary status

    # Format x-axis
    format_time_axis(plt.gca(), plot_df)

    plt.tight_layout()

    # Save plot
    output_filename = f"battery_voltage_decay_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Battery voltage decay plot saved: {output_path}")
    plt.close()
    gc.collect()  # Force garbage collection
    return output_path  # Return path for display


def plot_critical_battery_analysis(df, output_dir):
    """Plot detailed battery voltage analysis for critical monitoring"""
    # Sample data more aggressively for this memory-intensive plot
    plot_df = sample_data_for_plotting(df, max_points=2000)

    plt.figure(figsize=(16, 12))

    # Main voltage plot with enhanced features
    plt.subplot(3, 1, 1)

    # Plot voltage with simplified coloring to prevent OOM
    voltage = plot_df['battery_voltage'].values
    timestamps = plot_df['timestamp'].values

    # Use scatter plot with color mapping instead of individual line segments
    scatter = plt.scatter(timestamps, voltage, c=voltage, cmap='RdYlBu_r',
                          s=20, alpha=0.7, edgecolors='none')
    plt.colorbar(scatter, label='Voltage (V)')

    # Also plot a simple line for trend
    plt.plot(timestamps, voltage, 'k-', linewidth=1, alpha=0.5)

    # Add reference lines for critical voltages
    min_voltage = voltage.min()
    max_voltage = voltage.max()
    avg_voltage = voltage.mean()

    plt.axhline(y=avg_voltage, color='green', linestyle='--', alpha=0.7,
                label=f'Average: {avg_voltage:.2f}V')
    plt.axhline(y=min_voltage, color='red', linestyle='--', alpha=0.7,
                label=f'Minimum: {min_voltage:.2f}V')
    plt.axhline(y=max_voltage, color='blue', linestyle='--', alpha=0.7,
                label=f'Maximum: {max_voltage:.2f}V')

    # Add critical voltage thresholds (typical for Li-ion)
    plt.axhline(y=3.2, color='orange', linestyle=':', alpha=0.8,
                label='Critical Low (3.2V)', linewidth=2)
    plt.axhline(y=3.7, color='yellow', linestyle=':', alpha=0.8,
                label='Low Battery (3.7V)', linewidth=2)

    plt.title('Critical Battery Voltage Analysis - Enhanced Decay Visualization',
              fontsize=TITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Voltage (V)', fontsize=AXIS_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',
               fontsize=LEGEND_FONT_SIZE)

    # Format x-axis
    format_time_axis(plt.gca(), plot_df)

    # Voltage decay rate analysis
    plt.subplot(3, 1, 2)

    # Calculate voltage decay rate (V/hour) using sampled data
    time_diff_hours = (plot_df['timestamp'].diff().dt.total_seconds() / 3600)
    voltage_diff = plot_df['battery_voltage'].diff()
    decay_rate = voltage_diff / time_diff_hours

    # Plot decay rate
    plt.plot(plot_df['timestamp'][1:], decay_rate[1:],
             'purple', linewidth=2, label='Voltage Decay Rate')
    plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    plt.axhline(y=-0.1, color='red', linestyle='--', alpha=0.7,
                label='Fast Decay Threshold (-0.1V/h)')

    plt.title('Battery Voltage Decay Rate Over Time',
              fontsize=TITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Decay Rate (V/hour)', fontsize=AXIS_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=LEGEND_FONT_SIZE)

    # Format x-axis
    format_time_axis(plt.gca(), plot_df)

    # Battery percentage with voltage overlay
    plt.subplot(3, 1, 3)

    # Create dual y-axis plot
    ax1 = plt.gca()
    color1 = 'tab:green'
    ax1.set_xlabel('Time', fontsize=AXIS_FONT_SIZE)
    ax1.set_ylabel('Battery Percentage (%)',
                   color=color1, fontsize=AXIS_FONT_SIZE)
    line1 = ax1.plot(plot_df['timestamp'], plot_df['battery_remaining_pct'],
                     color=color1, linewidth=2, label='Battery %')
    ax1.tick_params(axis='y', labelcolor=color1, labelsize=LABEL_FONT_SIZE)
    ax1.grid(True, alpha=0.3)

    # Second y-axis for voltage
    ax2 = ax1.twinx()
    color2 = 'tab:blue'
    ax2.set_ylabel('Battery Voltage (V)', color=color2,
                   fontsize=AXIS_FONT_SIZE)
    line2 = ax2.plot(plot_df['timestamp'], plot_df['battery_voltage'],
                     color=color2, linewidth=2, alpha=0.7, label='Battery Voltage')
    ax2.tick_params(axis='y', labelcolor=color2, labelsize=LABEL_FONT_SIZE)

    # Add legend
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper right', fontsize=LEGEND_FONT_SIZE)

    plt.title('Battery State of Charge vs Voltage Correlation',
              fontsize=TITLE_FONT_SIZE, fontweight='bold')

    # Format x-axis
    ax1.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    ax1.xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.xticks(rotation=45)
    ax1.tick_params(axis='x', labelsize=LABEL_FONT_SIZE)

    plt.tight_layout()

    # Save plot
    output_filename = f"critical_battery_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Critical battery analysis plot saved: {output_path}")
    plt.close()
    gc.collect()  # Force garbage collection


def plot_sensor_trends(df, output_dir):
    """Plot all sensor trends in a comprehensive view"""
    # Sample data to prevent OOM
    plot_df = sample_data_for_plotting(df)

    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Battery Water Sensor Trends',
                 fontsize=TITLE_FONT_SIZE, fontweight='bold')

    # Battery voltage
    axes[0, 0].plot(plot_df['timestamp'], plot_df['battery_voltage'],
                    'b-', linewidth=1.5)
    axes[0, 0].set_title('Battery Voltage', fontsize=SUBTITLE_FONT_SIZE)
    axes[0, 0].set_ylabel('Voltage (V)', fontsize=AXIS_FONT_SIZE)
    axes[0, 0].grid(True, alpha=0.3)

    # Battery percentage
    axes[0, 1].plot(plot_df['timestamp'],
                    plot_df['battery_remaining_pct'], 'g-', linewidth=1.5)
    axes[0, 1].set_title('Battery State of Charge',
                         fontsize=SUBTITLE_FONT_SIZE)
    axes[0, 1].set_ylabel('Percentage (%)', fontsize=AXIS_FONT_SIZE)
    axes[0, 1].grid(True, alpha=0.3)

    # Saltwater voltage
    axes[1, 0].plot(plot_df['timestamp'], plot_df['saltwater_voltage'],
                    'r-', linewidth=1.5)
    axes[1, 0].set_title('Saltwater Probe Voltage',
                         fontsize=SUBTITLE_FONT_SIZE)
    axes[1, 0].set_ylabel('Voltage (V)', fontsize=AXIS_FONT_SIZE)
    axes[1, 0].grid(True, alpha=0.3)

    # Sail current
    axes[1, 1].plot(plot_df['timestamp'],
                    plot_df['sail_current'], 'm-', linewidth=1.5)
    axes[1, 1].set_title('Sail Winch Current', fontsize=SUBTITLE_FONT_SIZE)
    axes[1, 1].set_ylabel('Current (A)', fontsize=AXIS_FONT_SIZE)
    axes[1, 1].grid(True, alpha=0.3)

    # PCB Temperature
    axes[2, 0].plot(plot_df['timestamp'], plot_df['pcb_temperature'],
                    'orange', linewidth=1.5)
    axes[2, 0].set_title('PCB Temperature', fontsize=SUBTITLE_FONT_SIZE)
    axes[2, 0].set_ylabel('Temperature (°C)', fontsize=AXIS_FONT_SIZE)
    axes[2, 0].grid(True, alpha=0.3)

    # Relative Humidity
    axes[2, 1].plot(plot_df['timestamp'], plot_df['relative_humidity'],
                    'c-', linewidth=1.5)
    axes[2, 1].set_title('Relative Humidity', fontsize=SUBTITLE_FONT_SIZE)
    axes[2, 1].set_ylabel('Humidity (%)', fontsize=AXIS_FONT_SIZE)
    axes[2, 1].grid(True, alpha=0.3)

    # Format all x-axes
    for ax in axes.flat:
        format_time_axis(ax, plot_df)

    plt.tight_layout()

    # Save plot
    output_filename = f"sensor_trends_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Sensor trends plot saved: {output_path}")
    plt.close()
    gc.collect()  # Force garbage collection


def plot_alerts(df, output_dir):
    """Plot alert patterns over time"""
    # Sample data to prevent OOM
    plot_df = sample_data_for_plotting(df)

    plt.figure(figsize=(14, 8))

    # Create alert timeline
    plt.subplot(3, 1, 1)
    plt.plot(plot_df['timestamp'], plot_df['battery_low_alert'],
             'r-', linewidth=2, label='Battery Low Alert')
    plt.plot(plot_df['timestamp'], plot_df['saltwater_alert'],
             'b-', linewidth=2, label='Saltwater Alert')
    plt.plot(plot_df['timestamp'], plot_df['humidity_alert'],
             'g-', linewidth=2, label='Humidity Alert')
    plt.title('Alert Status Over Time',
              fontsize=SUBTITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Alert Status (0/1)', fontsize=AXIS_FONT_SIZE)
    plt.legend(fontsize=LEGEND_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.1, 1.1)

    # Charging status
    plt.subplot(3, 1, 2)
    plt.plot(plot_df['timestamp'], plot_df['charging_status'],
             'orange', linewidth=2, label='Charging Status')
    plt.plot(plot_df['timestamp'], plot_df['ac_power_present'],
             'purple', linewidth=2, label='AC Power Present')
    plt.title('MP2672GD Charger Status',
              fontsize=SUBTITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Status (0/1)', fontsize=AXIS_FONT_SIZE)
    plt.legend(fontsize=LEGEND_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.1, 1.1)

    # Health status
    plt.subplot(3, 1, 3)
    plt.plot(plot_df['timestamp'], plot_df['argo_battery_water_health'],
             'k-', linewidth=2, label='System Health')
    plt.title('Battery Water System Health',
              fontsize=SUBTITLE_FONT_SIZE, fontweight='bold')
    plt.ylabel('Health Status (0/1)', fontsize=AXIS_FONT_SIZE)
    plt.xlabel('Time', fontsize=AXIS_FONT_SIZE)
    plt.legend(fontsize=LEGEND_FONT_SIZE)
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.1, 1.1)

    # Format x-axis
    for ax in plt.gcf().axes:
        format_time_axis(ax, plot_df)

    plt.tight_layout()

    # Save plot
    output_filename = f"alert_patterns_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Alert patterns plot saved: {output_path}")
    plt.close()
    gc.collect()  # Force garbage collection


def print_data_summary(df):
    """Print a summary of the data"""
    print("\n" + "="*60)
    print("BATTERY WATER DATA SUMMARY")
    print("="*60)

    print(f"Data points: {len(df)}")
    print(f"Time span: {df['timestamp'].max() - df['timestamp'].min()}")

    # Calculate sampling rate, handling single data point case
    time_span_seconds = (df['timestamp'].max() -
                         df['timestamp'].min()).total_seconds()
    if time_span_seconds > 0:
        sampling_rate = len(df) / (time_span_seconds / 3600)
        print(f"Sampling rate: ~{sampling_rate:.1f} points/hour")
    else:
        print("Sampling rate: N/A (single data point)")

    print(f"\nBattery Voltage:")
    print(f"  Min: {df['battery_voltage'].min():.3f} V")
    print(f"  Max: {df['battery_voltage'].max():.3f} V")
    print(f"  Avg: {df['battery_voltage'].mean():.3f} V")
    print(f"  Current: {df['battery_voltage'].iloc[-1]:.3f} V")

    print(f"\nBattery State of Charge:")
    print(f"  Min: {df['battery_remaining_pct'].min():.1f}%")
    print(f"  Max: {df['battery_remaining_pct'].max():.1f}%")
    print(f"  Avg: {df['battery_remaining_pct'].mean():.1f}%")
    print(f"  Current: {df['battery_remaining_pct'].iloc[-1]:.1f}%")

    print(f"\nSaltwater Probe:")
    print(f"  Min: {df['saltwater_voltage'].min():.3f} V")
    print(f"  Max: {df['saltwater_voltage'].max():.3f} V")
    print(f"  Current: {df['saltwater_voltage'].iloc[-1]:.3f} V")

    print(f"\nSail Current:")
    print(f"  Min: {df['sail_current'].min():.3f} A")
    print(f"  Max: {df['sail_current'].max():.3f} A")
    print(f"  Current: {df['sail_current'].iloc[-1]:.3f} A")

    print(f"\nPCB Temperature:")
    print(f"  Min: {df['pcb_temperature'].min():.1f}°C")
    print(f"  Max: {df['pcb_temperature'].max():.1f}°C")
    print(f"  Current: {df['pcb_temperature'].iloc[-1]:.1f}°C")

    print(f"\nRelative Humidity:")
    print(f"  Min: {df['relative_humidity'].min():.1f}%")
    print(f"  Max: {df['relative_humidity'].max():.1f}%")
    print(f"  Current: {df['relative_humidity'].iloc[-1]:.1f}%")

    # Alert summary
    battery_alerts = df['battery_low_alert'].sum()
    saltwater_alerts = df['saltwater_alert'].sum()
    humidity_alerts = df['humidity_alert'].sum()
    health_failures = (df['argo_battery_water_health'] == 0).sum()

    print(f"\nAlerts:")
    print(f"  Battery Low: {battery_alerts} occurrences")
    print(f"  Saltwater: {saltwater_alerts} occurrences")
    print(f"  High Humidity: {humidity_alerts} occurrences")
    print(f"  Health Failures: {health_failures} occurrences")

    # Charging status summary (if available)
    if 'charging_status' in df.columns and 'ac_power_present' in df.columns:
        charging_active = df['charging_status'].sum()
        ac_power_active = df['ac_power_present'].sum()
        print(f"\nCharging Status:")
        print(f"  Charging Active: {charging_active} samples")
        print(f"  AC Power Present: {ac_power_active} samples")
        print(
            f"  Charging Percentage: {(charging_active / len(df) * 100):.1f}%")
        print(
            f"  AC Power Percentage: {(ac_power_active / len(df) * 100):.1f}%")

    print("="*60)


def display_image(image_path):
    """Display image using available viewer with proper DISPLAY handling"""
    try:
        # Get current environment
        env = os.environ.copy()
        
        # Ensure DISPLAY is set for X11 forwarding
        if 'DISPLAY' not in env or not env['DISPLAY']:
            print(f"No DISPLAY variable set - cannot open image viewer")
            print(f"Plot saved to: {image_path}")
            print("To view: Use SSH with X11 forwarding (ssh -X argo) or view file locally")
            return False
        
        print(f"Using DISPLAY={env['DISPLAY']}")
        
        # Try different image viewers in order of preference
        viewers = ['eog', 'display', 'feh', 'xdg-open']
        for viewer in viewers:
            try:
                # Check if viewer exists
                result = subprocess.run(['which', viewer], check=True, 
                                      capture_output=True, timeout=1)
                viewer_path = result.stdout.decode().strip()
                
                # Viewer exists, try to open image with DISPLAY environment
                print(f"Attempting to open with {viewer}...")
                subprocess.Popen([viewer, image_path], 
                               env=env,
                               stdout=subprocess.DEVNULL, 
                               stderr=subprocess.DEVNULL)
                print(f"✅ Plot opened with {viewer}")
                print(f"   File: {image_path}")
                return True
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
                continue
        
        print(f"⚠️  No compatible image viewer found")
        print(f"Plot saved to: {image_path}")
        print("Install an image viewer: sudo apt install eog imagemagick feh")
        return False
    except Exception as e:
        print(f"Error opening image: {e}")
        print(f"Plot saved to: {image_path}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Plot battery water sensor data from CSV files (concatenates last 3 files by default)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 plot-battery-water.py                    # Plot last 3 CSV files (voltage decay only)
    python3 plot-battery-water.py --all              # Generate all plots
    python3 plot-battery-water.py --critical         # Add critical battery analysis
    python3 plot-battery-water.py --num-files 5      # Concatenate last 5 CSV files
    python3 plot-battery-water.py file.csv           # Plot single specific file
    python3 plot-battery-water.py --no-display       # Don't open plot viewer
        """
    )
    parser.add_argument('csv_file', nargs='?',
                        help='Path to specific CSV file (default: auto-concatenate latest files)')
    parser.add_argument('--num-files', type=int, default=3,
                        help='Number of recent CSV files to concatenate (default: 3)')
    parser.add_argument('--output-dir', default='/var/log.hdd/persistent',
                        help='Output directory for plots (default: /var/log.hdd/persistent)')
    parser.add_argument('--no-plots', action='store_true',
                        help='Only print data summary, do not generate plots')
    parser.add_argument('--no-display', action='store_true',
                        help='Do not open plot viewer after generating')
    parser.add_argument('--all', action='store_true',
                        help='Generate all plots (default: voltage decay only)')
    parser.add_argument('--critical', action='store_true',
                        help='Add critical battery analysis plot')
    parser.add_argument('--sensors', action='store_true',
                        help='Add sensor trends plot')
    parser.add_argument('--alerts', action='store_true',
                        help='Add alerts pattern plot')
    parser.add_argument('--font-scale', type=float, default=1.0,
                        help='Font scale factor for plot text sizing (default: 1.0)')
    parser.add_argument('--max-points', type=int, default=5000,
                        help='Maximum data points to plot (default: 5000)')

    args = parser.parse_args()

    # Update font scale and max points based on arguments
    global FONT_SCALE, MAX_DATA_POINTS
    FONT_SCALE = args.font_scale
    MAX_DATA_POINTS = args.max_points

    # Update font size constants based on new scale
    global TITLE_FONT_SIZE, SUBTITLE_FONT_SIZE, AXIS_FONT_SIZE, LABEL_FONT_SIZE, LEGEND_FONT_SIZE
    TITLE_FONT_SIZE = int(14 * FONT_SCALE)
    SUBTITLE_FONT_SIZE = int(12 * FONT_SCALE)
    AXIS_FONT_SIZE = int(10 * FONT_SCALE)
    LABEL_FONT_SIZE = int(9 * FONT_SCALE)
    LEGEND_FONT_SIZE = int(8 * FONT_SCALE)

    # Determine CSV file path(s)
    if args.csv_file:
        # Single file specified
        csv_file_paths = args.csv_file
        print(f"Using specified CSV file: {csv_file_paths}")
    else:
        # Auto-detect and concatenate recent files
        csv_file_paths = find_latest_csv_files(args.num_files)
        if not csv_file_paths:
            print("Error: No battery monitor CSV files found.")
            print("Make sure argo_battery_water.py is running and creating CSV files.")
            sys.exit(1)
        print(f"\nConcatenating {len(csv_file_paths)} CSV file(s)")

    # Load data (handles both single file and list of files)
    df = load_battery_data(csv_file_paths)
    if df is None:
        sys.exit(1)

    # Print summary
    print_data_summary(df)

    if not args.no_plots:
        # Create output directory if it doesn't exist
        os.makedirs(args.output_dir, exist_ok=True)

        # Generate plots based on options
        print(f"\nGenerating plots in {args.output_dir}...")
        
        # Always generate voltage decay plot (default)
        voltage_plot = None
        voltage_plot = plot_battery_voltage_decay(df, args.output_dir)
        
        # Generate optional plots
        if args.all or args.critical:
            plot_critical_battery_analysis(df, args.output_dir)
        if args.all or args.sensors:
            plot_sensor_trends(df, args.output_dir)
        if args.all or args.alerts:
            plot_alerts(df, args.output_dir)
        
        print("Plot generation complete!")
        
        # Display the main voltage decay plot
        if voltage_plot and not args.no_display:
            display_image(voltage_plot)


if __name__ == "__main__":
    main()
