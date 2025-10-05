#!/usr/bin/env python3
"""
Simple Anemometer Calibration Data Visualization Script

This is a fallback plotting script that creates simple text-based plots
and saves data in a format that can be easily imported into other tools.

Usage:
    python3 scripts/plot_anem_simple.py [data_file.csv]
"""

import pandas as pd
import numpy as np
import glob
import sys
import os
from datetime import datetime

def find_latest_data_file():
    """Find the most recent anem-measurement-*.csv file"""
    pattern = "anem-measurement-*.csv"
    files = glob.glob(pattern)
    
    if not files:
        print("No anem-measurement-*.csv files found!")
        return None
    
    # Sort by modification time, get the most recent
    latest_file = max(files, key=os.path.getmtime)
    print(f"Using data file: {latest_file}")
    return latest_file

def load_data(filename):
    """Load calibration data from CSV file"""
    try:
        df = pd.read_csv(filename)
        print(f"Loaded {len(df)} data points from {filename}")
        return df
    except Exception as e:
        print(f"Error loading data file {filename}: {e}")
        return None

def create_text_plot(df, output_dir="plots"):
    """Create a simple text-based plot"""
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("ANEMOMETER CALIBRATION DATA - TEXT PLOT")
    print("="*80)
    
    # Define sensors
    sensors = ['CTR', 'CW', 'CCW']
    colors = {'CTR': 'BLUE', 'CW': 'RED', 'CCW': 'ORANGE'}
    
    # Create angle range for plotting
    angles = df['angle_deg'].values
    min_angle, max_angle = angles.min(), angles.max()
    
    # Create pressure range
    all_pressures = []
    for sensor in sensors:
        mean_col = f'dp_{sensor.lower()}_mean'
        if mean_col in df.columns:
            all_pressures.extend(df[mean_col].values)
    
    min_pressure, max_pressure = min(all_pressures), max(all_pressures)
    pressure_range = max_pressure - min_pressure
    
    # Create text plot
    plot_width = 80
    plot_height = 20
    
    print(f"Pressure range: {min_pressure:.2f} to {max_pressure:.2f} Pa")
    print(f"Angle range: {min_angle:.0f} to {max_angle:.0f} degrees")
    print()
    
    # Create grid
    grid = [[' ' for _ in range(plot_width)] for _ in range(plot_height)]
    
    # Add axes
    zero_line = int(plot_height * 0.5)  # Middle line for zero pressure
    for x in range(plot_width):
        grid[zero_line][x] = '-'
    
    # Add angle markers
    for angle in range(0, 360, 30):
        if min_angle <= angle <= max_angle:
            x_pos = int((angle - min_angle) / (max_angle - min_angle) * (plot_width - 1))
            for y in range(plot_height):
                if grid[y][x_pos] == ' ':
                    grid[y][x_pos] = '|'
    
    # Plot each sensor
    for sensor in sensors:
        mean_col = f'dp_{sensor.lower()}_mean'
        if mean_col not in df.columns:
            continue
            
        print(f"\n{sensor} SENSOR ({colors[sensor]}):")
        print("-" * 40)
        
        # Sort data by angle
        sensor_data = df[['angle_deg', mean_col]].sort_values('angle_deg')
        
        for _, row in sensor_data.iterrows():
            angle = row['angle_deg']
            pressure = row[mean_col]
            
            # Calculate position
            x_pos = int((angle - min_angle) / (max_angle - min_angle) * (plot_width - 1))
            y_pos = int((pressure - min_pressure) / pressure_range * (plot_height - 1))
            y_pos = max(0, min(plot_height - 1, y_pos))
            
            # Mark point
            if 0 <= x_pos < plot_width and 0 <= y_pos < plot_height:
                grid[y_pos][x_pos] = 'o'
            
            # Print data point
            print(f"  {angle:6.0f}°: {pressure:+8.3f} Pa")
    
    # Print the grid
    print(f"\nTEXT PLOT (Pressure: {min_pressure:.1f} to {max_pressure:.1f} Pa):")
    print("+" + "-" * plot_width + "+")
    for y in range(plot_height - 1, -1, -1):
        line = "|" + "".join(grid[y]) + "|"
        if y == zero_line:
            line += " <- Zero pressure"
        print(line)
    print("+" + "-" * plot_width + "+")
    print("Angles: " + " ".join([f"{int(min_angle + i * (max_angle - min_angle) / plot_width):3d}" 
                                for i in range(0, plot_width, 10)]))

def create_summary_report(df, output_dir="plots"):
    """Create a summary report of the calibration data"""
    os.makedirs(output_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = os.path.join(output_dir, f"anem_calibration_report_{timestamp}.txt")
    
    with open(report_file, 'w') as f:
        f.write("ANEMOMETER CALIBRATION DATA REPORT\n")
        f.write("=" * 50 + "\n\n")
        
        # Basic statistics
        f.write(f"Data points collected: {len(df)}\n")
        f.write(f"Angle range: {df['angle_deg'].min():.0f}° to {df['angle_deg'].max():.0f}°\n")
        f.write(f"Wind speed reference: {df['wind_speed_ref'].iloc[0]:.1f} m/s\n\n")
        
        # Sensor statistics
        sensors = ['CTR', 'CW', 'CCW']
        for sensor in sensors:
            mean_col = f'dp_{sensor.lower()}_mean'
            std_col = f'dp_{sensor.lower()}_std'
            
            if mean_col in df.columns:
                f.write(f"{sensor} SENSOR:\n")
                f.write(f"  Pressure range: {df[mean_col].min():.3f} to {df[mean_col].max():.3f} Pa\n")
                f.write(f"  Average pressure: {df[mean_col].mean():.3f} Pa\n")
                f.write(f"  Pressure variation: {df[mean_col].std():.3f} Pa\n")
                f.write(f"  Average noise: {df[std_col].mean():.3f} Pa\n\n")
        
        # Data quality
        if 'sample_count' in df.columns:
            expected_samples = 50
            actual_samples = df['sample_count'].mean()
            f.write(f"DATA QUALITY:\n")
            f.write(f"  Expected samples per measurement: {expected_samples}\n")
            f.write(f"  Actual average samples: {actual_samples:.1f}\n")
            f.write(f"  Data completeness: {(actual_samples/expected_samples)*100:.1f}%\n\n")
        
        # Temperature analysis
        if 'temp_mean' in df.columns:
            f.write(f"TEMPERATURE:\n")
            f.write(f"  Range: {df['temp_mean'].min():.1f}°C to {df['temp_mean'].max():.1f}°C\n")
            f.write(f"  Average: {df['temp_mean'].mean():.1f}°C\n")
            f.write(f"  Variation: {df['temp_mean'].std():.1f}°C\n\n")
        
        # Raw data
        f.write("RAW DATA:\n")
        f.write("-" * 20 + "\n")
        f.write(df.to_string(index=False))
    
    print(f"Summary report saved to: {report_file}")

def create_csv_for_external_plotting(df, output_dir="plots"):
    """Create a CSV file optimized for external plotting tools"""
    os.makedirs(output_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = os.path.join(output_dir, f"anem_plot_data_{timestamp}.csv")
    
    # Create a clean dataset for plotting
    plot_data = df[['angle_deg']].copy()
    
    # Add sensor data
    sensors = ['CTR', 'CW', 'CCW']
    for sensor in sensors:
        mean_col = f'dp_{sensor.lower()}_mean'
        std_col = f'dp_{sensor.lower()}_std'
        
        if mean_col in df.columns:
            plot_data[f'{sensor}_pressure'] = df[mean_col]
            plot_data[f'{sensor}_error'] = df[std_col]
    
    # Add wind direction labels
    plot_data['wind_direction'] = plot_data['angle_deg'].apply(
        lambda x: 'Front' if -15 <= x <= 15 else
                 'Starboard' if 75 <= x <= 105 else
                 'Stern' if 165 <= x <= 195 else
                 'Port' if 255 <= x <= 285 else
                 'Other'
    )
    
    plot_data.to_csv(csv_file, index=False)
    print(f"Plot data saved to: {csv_file}")
    
    # Create instructions file
    instructions_file = os.path.join(output_dir, f"plot_instructions_{timestamp}.txt")
    with open(instructions_file, 'w') as f:
        f.write("INSTRUCTIONS FOR EXTERNAL PLOTTING\n")
        f.write("=" * 40 + "\n\n")
        f.write("Data file: anem_plot_data_*.csv\n\n")
        f.write("Recommended plot settings:\n")
        f.write("- X-axis: angle_deg (0-360 degrees)\n")
        f.write("- Y-axis: *_pressure columns (differential pressure in Pa)\n")
        f.write("- Error bars: *_error columns\n")
        f.write("- Colors: CTR=blue, CW=red, CCW=orange\n")
        f.write("- Y-axis range: -20 to +20 Pa\n")
        f.write("- X-axis range: 0 to 360 degrees\n")
        f.write("- Add reference lines at 0 Pa and cardinal directions\n\n")
        f.write("Expected pattern: Sinusoidal curves with 120° phase shift\n")
    
    print(f"Plotting instructions saved to: {instructions_file}")

def main():
    # Get data file
    if len(sys.argv) > 1:
        data_file = sys.argv[1]
    else:
        data_file = find_latest_data_file()
    
    if not data_file or not os.path.exists(data_file):
        print("No valid data file found!")
        return
    
    # Load data
    df = load_data(data_file)
    if df is None:
        return
    
    # Create outputs
    print("\nCreating text-based visualization...")
    create_text_plot(df)
    
    print("\nCreating summary report...")
    create_summary_report(df)
    
    print("\nCreating data for external plotting...")
    create_csv_for_external_plotting(df)
    
    print(f"\n✅ Analysis complete! Check the 'plots/' directory for results.")
    print("If you have access to Excel, Python with working matplotlib, or other plotting tools,")
    print("you can use the generated CSV file to create proper plots.")

if __name__ == '__main__':
    main()








