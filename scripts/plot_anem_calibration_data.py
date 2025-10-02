#!/usr/bin/env python3
"""
Anemometer Calibration Data Visualization Script

This script plots the collected anemometer calibration data to visualize
sensor response patterns and help identify calibration issues.

Usage:
    python3 scripts/plot_anem_calibration_data.py [data_file.csv]

If no data file is specified, it will look for the most recent anem-measurement-*.csv file.
"""

import pandas as pd
import numpy as np
import glob
import sys
import os
from datetime import datetime

# Handle NumPy compatibility issue
try:
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"Matplotlib import error: {e}")
    print("Trying to fix NumPy compatibility...")
    try:
        import numpy as np
        # Force NumPy 1.x compatibility
        if hasattr(np, '__version__') and np.__version__.startswith('2.'):
            print("Detected NumPy 2.x, attempting compatibility fix...")
            # This is a workaround for the NumPy 2.x compatibility issue
            import warnings
            warnings.filterwarnings('ignore', category=DeprecationWarning)
    except:
        pass
    
    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"Failed to import matplotlib: {e}")
        print("Please install matplotlib: pip3 install matplotlib")
        sys.exit(1)

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

def calculate_wind_direction_argo(dp_ctr, dp_cw, dp_ccw):
    """
    Calculate wind direction from three differential pressure sensors
    
    Based on the data analysis, the sensors are positioned at 60 degree intervals:
    - CTR: 0° (front/back)
    - CW: 60° (clockwise from front looking down on mast)  
    - CCW: -60° (counter-clockwise from front looking down on mast)
    
    This is the Sensioron wind sensor configuration.
    
    Args:
        dp_ctr, dp_cw, dp_ccw: Differential pressure readings (Pa)
    
    Returns:
        wind_direction: Wind direction in degrees (0-360°)
    """
    try:
        # Method: Use the three-sensor algorithm for 60° spaced sensors
        
        # Convert sensor positions to radians
        ctr_angle = 0 * np.pi / 180
        cw_angle = 60 * np.pi / 180  
        ccw_angle = -60 * np.pi / 180
        
        # Calculate x and y components using the three sensors
        # Each sensor contributes to both x and y based on its position
        x_component = (dp_ctr * np.cos(ctr_angle) + 
                      dp_cw * np.cos(cw_angle) + 
                      dp_ccw * np.cos(ccw_angle))
        
        y_component = -(dp_ctr * np.sin(ctr_angle) + 
                       dp_cw * np.sin(cw_angle) + 
                       dp_ccw * np.sin(ccw_angle))
        
        # Calculate wind direction using atan2, minus sign because of the sensor orientation
        wind_direction_rad = -np.arctan2(y_component, x_component)
        wind_direction_deg = np.degrees(wind_direction_rad)
        
        # Normalize to 0-360°
        if wind_direction_deg < 0:
            wind_direction_deg += 360
            
        return wind_direction_deg
        
    except Exception as e:
        print(f"Error calculating wind direction: {e}")
        return 0.0

def simple_sinusoidal_fit(angles_deg, pressures):
    """
    Simple sinusoidal fit using FFT approach
    
    Args:
        angles_deg: Wind angles in degrees
        pressures: Pressure readings (Pa)
    
    Returns:
        fitted_angles: Fitted wind angles
        fitted_pressures: Fitted pressure values
        amplitude: Fitted amplitude
        phase: Fitted phase offset
    """
    try:
        # Convert to radians
        angles_rad = np.deg2rad(angles_deg)
        
        # Simple approach: find the phase that maximizes correlation
        # with a cosine function
        best_phase = 0
        best_correlation = -1
        
        # Test different phase offsets
        for phase_test in np.linspace(0, 2*np.pi, 360):
            cos_ref = np.cos(angles_rad - phase_test)
            correlation = np.corrcoef(pressures, cos_ref)[0, 1]
            if abs(correlation) > best_correlation:
                best_correlation = abs(correlation)
                best_phase = phase_test
        
        # Calculate amplitude and offset
        cos_ref = np.cos(angles_rad - best_phase)
        amplitude = np.std(pressures) * 2 / np.std(cos_ref)
        offset = np.mean(pressures)
        
        # Generate fitted curve
        angles_fit = np.linspace(angles_deg.min(), angles_deg.max(), 100)
        angles_fit_rad = np.deg2rad(angles_fit)
        pressures_fit = amplitude * np.cos(angles_fit_rad - best_phase) + offset
        
        return angles_fit, pressures_fit, amplitude, np.degrees(best_phase)
        
    except Exception as e:
        print(f"Error fitting sinusoidal response: {e}")
        return angles_deg, pressures, 0, 0

def plot_sensor_response(df, output_dir="plots", source_filename=None):
    """Create comprehensive plots of sensor response data"""
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract timestamp from source filename if provided
    if source_filename:
        # Extract timestamp from filename like "anem-measurement-20251001_162926.csv"
        import re
        timestamp_match = re.search(r'anem-measurement-(\d{8}_\d{6})\.csv', source_filename)
        if timestamp_match:
            timestamp = timestamp_match.group(1)
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Set up the plotting style to match reference
    plt.style.use('default')
    plt.rcParams['font.size'] = 20
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['grid.linestyle'] = ':'
    
    # Define colors and styles to match reference plot
    colors = {'CTR': 'blue', 'CW': 'red', 'CCW': 'orange'}
    sensors = ['CTR', 'CW', 'CCW']
    
    # Create three plots: polar plot, DP vs angle plot, and fitted vs measured angle
    fig = plt.figure(figsize=(30, 10))
    
    # Left plot: Polar plot for wind direction visualization
    ax1 = plt.subplot(1, 3, 1, projection='polar')
    
    # Convert angles to radians for polar plot, converting -180° to +180° to 0° to 360°
    angles_deg_polar = df['angle_deg'].copy()
    angles_deg_polar[angles_deg_polar < 0] += 360
    angles_rad = np.deg2rad(angles_deg_polar)
    
    # Plot each sensor with both raw data and smoothed line on polar plot
    for sensor in sensors:
        mean_col = f'dp_{sensor.lower()}_mean'
        std_col = f'dp_{sensor.lower()}_std'
        
        if mean_col in df.columns:
            # Separate positive and negative values for different plotting
            pressures = df[mean_col].values
            errors = df[std_col].values
            
            # Plot connecting lines first
            ax1.plot(angles_rad, np.abs(pressures), 
                    color=colors[sensor], 
                    linewidth=3,
                    linestyle='-',
                    alpha=0.7)
            
            # Positive values (solid circles)
            pos_mask = pressures >= 0
            if np.any(pos_mask):
                ax1.errorbar(angles_rad[pos_mask], pressures[pos_mask], 
                           yerr=errors[pos_mask], 
                           label=f'{sensor} sensor (+)', 
                           color=colors[sensor], 
                           marker='o', 
                           markersize=8,
                           capsize=4, 
                           capthick=2,
                           linewidth=0,  # No line from errorbar
                           linestyle='None',
                           markeredgecolor='white',
                           markeredgewidth=1,
                           alpha=0.7)
            
            # Negative values (hollow squares with absolute values)
            neg_mask = pressures < 0
            if np.any(neg_mask):
                ax1.errorbar(angles_rad[neg_mask], np.abs(pressures[neg_mask]), 
                           yerr=errors[neg_mask], 
                           label=f'{sensor} sensor (-)', 
                           color=colors[sensor], 
                           marker='s', 
                           markersize=8,
                           capsize=4, 
                           capthick=2,
                           linewidth=0,  # No line from errorbar
                           linestyle='None',
                           markerfacecolor='white',
                           markeredgecolor=colors[sensor],
                           markeredgewidth=2,
                           alpha=0.7)
            
    
    # Polar plot formatting
    ax1.set_ylim(0, 40)  # Full scale limit of 40 Pa
    ax1.set_yticks(range(0, 45, 10))  # Radial ticks every 10 Pa
    ax1.set_theta_zero_location('N')  # 0° at top (North/Front)
    ax1.set_theta_direction(-1)  # Clockwise direction (standard for wind)
    
    # Set custom theta ticks to show 0 to 360 range with -180 to +180 labels
    theta_ticks = np.linspace(0, 2*np.pi, 9)  # 8 intervals from 0° to 360°
    theta_labels = ['0°', '45°', '90°', '135°', '180°/-180°', '225°/-135°', '270°/-90°', '315°/-45°', '360°/0°']
    ax1.set_thetagrids(np.degrees(theta_ticks), theta_labels)
    
    # Clean up the plot appearance
    ax1.spines['polar'].set_visible(True)  # Keep polar spine for outer rim
    ax1.set_facecolor('white')  # Ensure white background
    
    # Add radial grid lines (cleaner style)
    ax1.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    
    # Add reference circles for pressure levels (cleaner style)
    for pressure in [10, 20, 30, 40]:
        circle = plt.Circle((0, 0), pressure, fill=False, color='lightgray', 
                          linestyle='-', alpha=0.4, linewidth=0.8)
        ax1.add_patch(circle)
    
    # Add outer rim circle at 40 Pa
    outer_circle = plt.Circle((0, 0), 40, fill=False, color='black', 
                            linestyle='-', alpha=0.8, linewidth=2)
    ax1.add_patch(outer_circle)
    
    # Add cardinal direction labels with proper spacing
    ax1.text(0, 48, 'Front (0°)', ha='center', va='center', fontsize=14, fontweight='bold', 
            bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    ax1.text(np.pi/2, 48, 'Starboard (90°)', ha='center', va='center', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    ax1.text(np.pi, 48, 'Stern (180°/-180°)', ha='center', va='center', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    ax1.text(3*np.pi/2, 48, 'Port (270°/-90°)', ha='center', va='center', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    
    # Get wind speed from data for title
    wind_speed = df['wind_speed_ref'].iloc[0] if 'wind_speed_ref' in df.columns else 7.4
    ax1.set_title(f'Polar Response\n@ ~{wind_speed:.1f} m/s\n● = Positive DP, ■ = Negative DP', 
                fontsize=14, pad=30)
    
    # Legend for polar plot
    ax1.legend(loc='upper right', fontsize=12, framealpha=0.9, bbox_to_anchor=(1.3, 1.0))
    
    # Middle plot: DP vs angle plot
    ax2 = plt.subplot(1, 3, 2)
    
    # Plot each sensor's differential pressure vs angle
    for sensor in sensors:
        mean_col = f'dp_{sensor.lower()}_mean'
        std_col = f'dp_{sensor.lower()}_std'
        
        if mean_col in df.columns:
            # Plot with error bars
            ax2.errorbar(df['angle_deg'], df[mean_col], 
                        yerr=df[std_col], 
                        label=f'{sensor} sensor', 
                        color=colors[sensor], 
                        marker='o', 
                        markersize=8,
                        capsize=4, 
                        capthick=2,
                        linewidth=3,
                        markeredgecolor='white',
                        markeredgewidth=1,
                        alpha=0.7)
    
    ax2.set_xlabel('Wind Angle (degrees)', fontsize=14)
    ax2.set_ylabel('Differential Pressure (Pa)', fontsize=14)
    ax2.set_title(f'Linear Response\n@ ~{wind_speed:.1f} m/s', fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=12)
    ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    
    # Set reasonable axis limits
    ax2.set_xlim(-180, 180)
    
    # Right plot: Fitted vs Measured angle
    ax3 = plt.subplot(1, 3, 3)
    
    # Calculate fitted wind directions for each measurement
    fitted_angles = []
    measured_angles = []
    
    print("\n============================================================")
    print("WIND DIRECTION CALCULATION DEBUG")
    print("============================================================")
    
    for i, (_, row) in enumerate(df.iterrows()):
        dp_ctr = row['dp_ctr_mean']
        dp_cw = row['dp_cw_mean'] 
        dp_ccw = row['dp_ccw_mean']
        measured_angle = row['angle_deg']
        
        # Calculate fitted wind direction
        fitted_angle = calculate_wind_direction_argo(dp_ctr, dp_cw, dp_ccw)
        
        # Convert to -180 to +180 range for comparison
        if fitted_angle > 180:
            fitted_angle -= 360
            
        fitted_angles.append(fitted_angle)
        measured_angles.append(measured_angle)
        
        # Debug output for all measurements
        print(f"Measurement {i+1}: Angle={measured_angle:6.1f}°, "
              f"CTR={dp_ctr:6.2f}, CW={dp_cw:6.2f}, CCW={dp_ccw:6.2f} "
              f"-> Fitted={fitted_angle:6.1f}°")
    
    print(f"Total measurements: {len(fitted_angles)}")
    print("============================================================\nGenerating plots and fitting curves...")
    
    # Plot fitted vs measured angles
    ax3.scatter(measured_angles, fitted_angles, 
               color='blue', s=100, alpha=0.7, label='Data points')
    
    # Plot perfect correlation line
    min_angle = min(min(measured_angles), min(fitted_angles))
    max_angle = max(max(measured_angles), max(fitted_angles))
    ax3.plot([min_angle, max_angle], [min_angle, max_angle], 
             'r--', linewidth=2, label='Perfect correlation')
    
    # Calculate and display correlation
    correlation = np.corrcoef(measured_angles, fitted_angles)[0, 1]
    rmse = np.sqrt(np.mean((np.array(measured_angles) - np.array(fitted_angles))**2))
    
    ax3.set_xlabel('Measured Angle (degrees)', fontsize=14)
    ax3.set_ylabel('Fitted Angle (degrees)', fontsize=14)
    ax3.set_title(f'Wind Direction Calibration\nCorrelation: {correlation:.3f}, RMSE: {rmse:.1f}°', fontsize=14)
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=12)
    ax3.set_xlim(min_angle - 10, max_angle + 10)
    ax3.set_ylim(min_angle - 10, max_angle + 10)
    
    # Overall figure title
    fig.suptitle(f'Anemometer Calibration Data - {wind_speed:.1f} m/s', fontsize=18, y=0.95)
    
    plt.tight_layout()
    
    # Save the combined plot
    combined_plot_filename = os.path.join(output_dir, f"anem_combined_{timestamp}.png")
    plt.savefig(combined_plot_filename, dpi=300, bbox_inches='tight')
    print(f"Combined plot saved to: {combined_plot_filename}")
    
    # Try to display the plot (with error handling for headless systems)
    # try:
    #     plt.show()
    # except Exception as e:
    #     print(f"Could not display plot (likely no X11 display): {e}")
    #     print("Plot saved to file instead.")
    
    return combined_plot_filename

def analyze_data_quality(df):
    """Analyze and report data quality metrics"""
    print("\n" + "="*60)
    print("DATA QUALITY ANALYSIS")
    print("="*60)
    
    # Sample count analysis
    if 'sample_count' in df.columns:
        expected_samples = 50  # 5 seconds * 10 Hz
        actual_samples = df['sample_count'].mean()
        print(f"Average samples per measurement: {actual_samples:.1f} (expected: {expected_samples})")
        print(f"Sample count range: {df['sample_count'].min()} - {df['sample_count'].max()}")
        
        low_quality = df[df['sample_count'] < expected_samples * 0.8]
        if len(low_quality) > 0:
            print(f"WARNING: {len(low_quality)} measurements have low sample counts:")
            for _, row in low_quality.iterrows():
                print(f"  Angle {row['angle_deg']:+.0f}°: {row['sample_count']} samples")
    
    # Pressure range analysis
    for sensor in ['CTR', 'CW', 'CCW']:
        mean_col = f'dp_{sensor.lower()}_mean'
        if mean_col in df.columns:
            pressure_range = df[mean_col].max() - df[mean_col].min()
            print(f"{sensor} sensor pressure range: {pressure_range:.2f} Pa")
            print(f"  Min: {df[mean_col].min():.2f} Pa, Max: {df[mean_col].max():.2f} Pa")
    
    # Noise analysis
    print("\nNoise levels (standard deviation):")
    for sensor in ['CTR', 'CW', 'CCW']:
        std_col = f'dp_{sensor.lower()}_std'
        if std_col in df.columns:
            avg_noise = df[std_col].mean()
            max_noise = df[std_col].max()
            print(f"  {sensor} sensor: {avg_noise:.3f} Pa avg, {max_noise:.3f} Pa max")
    
    # Temperature stability
    if 'temp_mean' in df.columns:
        temp_range = df['temp_mean'].max() - df['temp_mean'].min()
        print(f"\nTemperature range: {temp_range:.1f}°C")
        print(f"  Min: {df['temp_mean'].min():.1f}°C, Max: {df['temp_mean'].max():.1f}°C")

def main():
    # I2C addresses for reference
    global I2C_CTR, I2C_CW, I2C_CCW
    I2C_CTR = 0x21
    I2C_CW = 0x22
    I2C_CCW = 0x23
    
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
    
    # Analyze data quality
    analyze_data_quality(df)
    
    # Create plots
    print("\nGenerating plots...")
    combined_plot = plot_sensor_response(df, source_filename=data_file)
    
    print(f"\nAnalysis complete! Check the 'plots/' directory for results.")
    print(f"Generated combined plot: {combined_plot}")
    print("\nKey things to look for:")
    print("- Sensor response patterns should be sinusoidal")
    print("- All three sensors should show different phase relationships")
    print("- Low noise levels indicate good data quality")
    print("- Consistent sample counts show reliable data collection")

if __name__ == '__main__':
    main()
