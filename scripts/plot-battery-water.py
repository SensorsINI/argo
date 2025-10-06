#!/usr/bin/env python3
"""
Battery Water Data Plotting Script

This script reads CSV data from battery_water.py and creates visualizations
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


def find_latest_csv_file():
    """Find the most recent battery monitor CSV file"""
    log_dir = "/var/log.hdd/persistent"
    if not os.path.exists(log_dir):
        return None

    # Look for today's file first
    today = datetime.now().strftime('%Y%m%d')
    today_file = os.path.join(log_dir, f"battery-monitor-{today}.csv")
    if os.path.exists(today_file):
        return today_file

    # Look for any battery-monitor CSV files
    csv_files = []
    for filename in os.listdir(log_dir):
        if filename.startswith("battery-monitor-") and filename.endswith(".csv"):
            filepath = os.path.join(log_dir, filename)
            csv_files.append((filepath, os.path.getmtime(filepath)))

    if csv_files:
        # Return the most recent file
        csv_files.sort(key=lambda x: x[1], reverse=True)
        return csv_files[0][0]

    return None


def load_battery_data(csv_file_path):
    """Load and prepare battery data from CSV file"""
    if not os.path.exists(csv_file_path):
        print(f"Error: CSV file not found at {csv_file_path}")
        return None

    try:
        # Try to read with headers first
        try:
            df = pd.read_csv(csv_file_path)
            # Check if we have the expected columns
            if 'timestamp' not in df.columns:
                raise ValueError("No timestamp column found")
        except (ValueError, KeyError):
            # If that fails, read without headers and assign column names
            column_names = [
                'timestamp', 'battery_voltage', 'battery_remaining_pct',
                'saltwater_voltage', 'sail_current', 'pcb_temperature',
                'relative_humidity', 'battery_low_alert', 'saltwater_alert',
                'humidity_alert', 'battery_water_health', 'charging_status', 'ac_power_present'
            ]
            df = pd.read_csv(csv_file_path, header=None, names=column_names)
            print(
                f"Loaded {len(df)} data points from {csv_file_path} (no headers detected)")
        else:
            print(f"Loaded {len(df)} data points from {csv_file_path}")

        # Convert timestamp to datetime
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df = df.sort_values('timestamp')

        print(
            f"Date range: {df['timestamp'].min()} to {df['timestamp'].max()}")
        return df

    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


def plot_battery_voltage_decay(df, output_dir):
    """Plot battery voltage over time to show decay patterns"""
    plt.figure(figsize=(14, 8))

    # Main voltage plot
    plt.subplot(2, 1, 1)
    plt.plot(df['timestamp'], df['battery_voltage'],
             'b-', linewidth=2, label='Battery Voltage')
    plt.title('Battery Voltage Decay Over Time',
              fontsize=14, fontweight='bold')
    plt.ylabel('Voltage (V)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()

    # Format x-axis
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.xticks(rotation=45)

    # Battery percentage plot
    plt.subplot(2, 1, 2)
    plt.plot(df['timestamp'], df['battery_remaining_pct'],
             'g-', linewidth=2, label='Battery %')
    plt.title('Battery State of Charge', fontsize=14, fontweight='bold')
    plt.ylabel('Percentage (%)', fontsize=12)
    plt.xlabel('Time', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()

    # Format x-axis
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.xticks(rotation=45)

    plt.tight_layout()

    # Save plot
    output_filename = f"battery_voltage_decay_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Battery voltage decay plot saved: {output_path}")
    plt.close()


def plot_critical_battery_analysis(df, output_dir):
    """Plot detailed battery voltage analysis for critical monitoring"""
    plt.figure(figsize=(16, 12))

    # Main voltage plot with enhanced features
    plt.subplot(3, 1, 1)
    
    # Plot voltage with gradient coloring to show decay
    voltage = df['battery_voltage'].values
    timestamps = df['timestamp'].values
    
    # Create gradient line plot
    for i in range(len(voltage) - 1):
        # Color gradient from blue (high) to red (low)
        color_intensity = (voltage[i] - voltage.min()) / (voltage.max() - voltage.min())
        color = plt.cm.RdYlBu_r(color_intensity)  # Red-Yellow-Blue reversed
        plt.plot([timestamps[i], timestamps[i+1]], 
                [voltage[i], voltage[i+1]], 
                color=color, linewidth=2.5, alpha=0.8)
    
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
              fontsize=16, fontweight='bold')
    plt.ylabel('Voltage (V)', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Format x-axis
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.xticks(rotation=45)

    # Voltage decay rate analysis
    plt.subplot(3, 1, 2)
    
    # Calculate voltage decay rate (V/hour)
    time_diff_hours = (df['timestamp'].diff().dt.total_seconds() / 3600)
    voltage_diff = df['battery_voltage'].diff()
    decay_rate = voltage_diff / time_diff_hours
    
    # Plot decay rate
    plt.plot(df['timestamp'][1:], decay_rate[1:], 
             'purple', linewidth=2, label='Voltage Decay Rate')
    plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    plt.axhline(y=-0.1, color='red', linestyle='--', alpha=0.7, 
                label='Fast Decay Threshold (-0.1V/h)')
    
    plt.title('Battery Voltage Decay Rate Over Time', fontsize=14, fontweight='bold')
    plt.ylabel('Decay Rate (V/hour)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Format x-axis
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.xticks(rotation=45)

    # Battery percentage with voltage overlay
    plt.subplot(3, 1, 3)
    
    # Create dual y-axis plot
    ax1 = plt.gca()
    color1 = 'tab:green'
    ax1.set_xlabel('Time', fontsize=12)
    ax1.set_ylabel('Battery Percentage (%)', color=color1, fontsize=12)
    line1 = ax1.plot(df['timestamp'], df['battery_remaining_pct'], 
                     color=color1, linewidth=2, label='Battery %')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.grid(True, alpha=0.3)
    
    # Second y-axis for voltage
    ax2 = ax1.twinx()
    color2 = 'tab:blue'
    ax2.set_ylabel('Battery Voltage (V)', color=color2, fontsize=12)
    line2 = ax2.plot(df['timestamp'], df['battery_voltage'], 
                     color=color2, linewidth=2, alpha=0.7, label='Battery Voltage')
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Add legend
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper right')
    
    plt.title('Battery State of Charge vs Voltage Correlation', 
              fontsize=14, fontweight='bold')
    
    # Format x-axis
    ax1.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    ax1.xaxis.set_major_locator(mdates.HourLocator(interval=1))
    plt.xticks(rotation=45)

    plt.tight_layout()

    # Save plot
    output_filename = f"critical_battery_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Critical battery analysis plot saved: {output_path}")
    plt.close()


def plot_sensor_trends(df, output_dir):
    """Plot all sensor trends in a comprehensive view"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Battery Water Sensor Trends', fontsize=16, fontweight='bold')

    # Battery voltage
    axes[0, 0].plot(df['timestamp'], df['battery_voltage'],
                    'b-', linewidth=1.5)
    axes[0, 0].set_title('Battery Voltage')
    axes[0, 0].set_ylabel('Voltage (V)')
    axes[0, 0].grid(True, alpha=0.3)

    # Battery percentage
    axes[0, 1].plot(df['timestamp'],
                    df['battery_remaining_pct'], 'g-', linewidth=1.5)
    axes[0, 1].set_title('Battery State of Charge')
    axes[0, 1].set_ylabel('Percentage (%)')
    axes[0, 1].grid(True, alpha=0.3)

    # Saltwater voltage
    axes[1, 0].plot(df['timestamp'], df['saltwater_voltage'],
                    'r-', linewidth=1.5)
    axes[1, 0].set_title('Saltwater Probe Voltage')
    axes[1, 0].set_ylabel('Voltage (V)')
    axes[1, 0].grid(True, alpha=0.3)

    # Sail current
    axes[1, 1].plot(df['timestamp'], df['sail_current'], 'm-', linewidth=1.5)
    axes[1, 1].set_title('Sail Winch Current')
    axes[1, 1].set_ylabel('Current (A)')
    axes[1, 1].grid(True, alpha=0.3)

    # PCB Temperature
    axes[2, 0].plot(df['timestamp'], df['pcb_temperature'],
                    'orange', linewidth=1.5)
    axes[2, 0].set_title('PCB Temperature')
    axes[2, 0].set_ylabel('Temperature (°C)')
    axes[2, 0].grid(True, alpha=0.3)

    # Relative Humidity
    axes[2, 1].plot(df['timestamp'], df['relative_humidity'],
                    'c-', linewidth=1.5)
    axes[2, 1].set_title('Relative Humidity')
    axes[2, 1].set_ylabel('Humidity (%)')
    axes[2, 1].grid(True, alpha=0.3)

    # Format all x-axes
    for ax in axes.flat:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()

    # Save plot
    output_filename = f"sensor_trends_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Sensor trends plot saved: {output_path}")
    plt.close()


def plot_alerts(df, output_dir):
    """Plot alert patterns over time"""
    plt.figure(figsize=(14, 8))

    # Create alert timeline
    plt.subplot(3, 1, 1)
    plt.plot(df['timestamp'], df['battery_low_alert'],
             'r-', linewidth=2, label='Battery Low Alert')
    plt.plot(df['timestamp'], df['saltwater_alert'],
             'b-', linewidth=2, label='Saltwater Alert')
    plt.plot(df['timestamp'], df['humidity_alert'],
             'g-', linewidth=2, label='Humidity Alert')
    plt.title('Alert Status Over Time', fontsize=14, fontweight='bold')
    plt.ylabel('Alert Status (0/1)', fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.1, 1.1)

    # Charging status
    plt.subplot(3, 1, 2)
    plt.plot(df['timestamp'], df['charging_status'],
             'orange', linewidth=2, label='Charging Status')
    plt.plot(df['timestamp'], df['ac_power_present'],
             'purple', linewidth=2, label='AC Power Present')
    plt.title('MP2672GD Charger Status', fontsize=14, fontweight='bold')
    plt.ylabel('Status (0/1)', fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.1, 1.1)

    # Health status
    plt.subplot(3, 1, 3)
    plt.plot(df['timestamp'], df['battery_water_health'],
             'k-', linewidth=2, label='System Health')
    plt.title('Battery Water System Health', fontsize=14, fontweight='bold')
    plt.ylabel('Health Status (0/1)', fontsize=12)
    plt.xlabel('Time', fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.1, 1.1)

    # Format x-axis
    for ax in plt.gcf().axes:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()

    # Save plot
    output_filename = f"alert_patterns_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Alert patterns plot saved: {output_path}")
    plt.close()


def print_data_summary(df):
    """Print a summary of the data"""
    print("\n" + "="*60)
    print("BATTERY WATER DATA SUMMARY")
    print("="*60)

    print(f"Data points: {len(df)}")
    print(f"Time span: {df['timestamp'].max() - df['timestamp'].min()}")
    print(
        f"Sampling rate: ~{len(df) / ((df['timestamp'].max() - df['timestamp'].min()).total_seconds() / 3600):.1f} points/hour")

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
    health_failures = (df['battery_water_health'] == 0).sum()

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
        print(f"  Charging Percentage: {(charging_active / len(df) * 100):.1f}%")
        print(f"  AC Power Percentage: {(ac_power_active / len(df) * 100):.1f}%")

    print("="*60)


def main():
    parser = argparse.ArgumentParser(
        description='Plot battery water sensor data from CSV files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 plot-battery-water.py
    python3 plot-battery-water.py /var/log.hdd/persistent/battery-monitor-20251005.csv
    python3 plot-battery-water.py --output-dir /tmp/plots
        """
    )
    parser.add_argument('csv_file', nargs='?',
                        help='Path to CSV file (default: auto-detect latest)')
    parser.add_argument('--output-dir', default='/var/log.hdd/persistent',
                        help='Output directory for plots (default: /var/log.hdd/persistent)')
    parser.add_argument('--no-plots', action='store_true',
                        help='Only print data summary, do not generate plots')

    args = parser.parse_args()

    # Determine CSV file path
    if args.csv_file:
        csv_file_path = args.csv_file
    else:
        csv_file_path = find_latest_csv_file()
        if not csv_file_path:
            print("Error: No battery monitor CSV file found.")
            print("Make sure battery_water.py is running and creating CSV files.")
            sys.exit(1)
        print(f"Using latest CSV file: {csv_file_path}")

    # Load data
    df = load_battery_data(csv_file_path)
    if df is None:
        sys.exit(1)

    # Print summary
    print_data_summary(df)

    if not args.no_plots:
        # Create output directory if it doesn't exist
        os.makedirs(args.output_dir, exist_ok=True)

        # Generate plots
        print(f"\nGenerating plots in {args.output_dir}...")
        plot_battery_voltage_decay(df, args.output_dir)
        plot_critical_battery_analysis(df, args.output_dir)
        plot_sensor_trends(df, args.output_dir)
        plot_alerts(df, args.output_dir)
        print("All plots generated successfully!")


if __name__ == "__main__":
    main()
