#!/usr/bin/env python3
"""
Analyze GPS noise characteristics from ROS2 bag recordings.

Extracts GPS fix, COG, and SOG data and analyzes temporal noise characteristics.
Creates visualizations showing GPS track with COG/SOG vectors.
"""

import sys
import os
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import math
import time

# ROS2 imports
try:
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import NavSatFix
    from geometry_msgs.msg import Vector3
    from std_msgs.msg import Float64
    from rclpy.clock import Clock
    from rclpy.time import Time
except ImportError as e:
    print(f"Error: ROS2 imports failed: {e}")
    print("Make sure ROS2 environment is sourced: source /opt/ros/humble/setup.bash")
    sys.exit(1)

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except ImportError:
    print("Error: rosbag2_py not available. Install with: pip3 install rosbag2_py")
    sys.exit(1)


def latlon_to_xy(lat, lon, base_lat, base_lon):
    """Convert lat/lon to local XY coordinates (meters).
    
    Uses simple approximation: 1 degree latitude ≈ 111,000 meters
    1 degree longitude ≈ 111,000 * cos(latitude) meters
    """
    lat_diff = lat - base_lat
    lon_diff = lon - base_lon
    
    x = lon_diff * 111000.0 * math.cos(math.radians(base_lat))  # East
    y = lat_diff * 111000.0  # North
    
    return x, y


def read_bag_file(bag_path):
    """Read GPS data from ROS2 bag file.
    
    Returns:
        dict with keys: 'fix' (list of NavSatFix), 'cog' (list of (time, value)),
        'sog' (list of (time, value)), 'velocity' (list of Vector3)
    """
    print(f"Reading bag file: {bag_path}")
    
    # Check if it's a directory (rosbag2 format)
    if os.path.isdir(bag_path):
        # Find the actual bag file
        bag_files = []
        for f in os.listdir(bag_path):
            if f.endswith('.db3') or f.endswith('.mcap'):
                bag_files.append(os.path.join(bag_path, f))
        
        if not bag_files:
            print(f"Error: No bag files found in {bag_path}")
            return None
        
        # Use the first bag file (rosbag2 can have multiple)
        bag_file = bag_files[0]
        bag_dir = bag_path
    else:
        bag_file = bag_path
        bag_dir = os.path.dirname(bag_path)
    
    print(f"Using bag file: {bag_file}")
    
    # Open bag file
    storage_options = StorageOptions(uri=bag_dir, storage_id='')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic metadata
    topic_types = reader.get_all_topics_and_types()
    topic_map = {topic.name: topic.type for topic in topic_types}
    
    print(f"Available topics: {list(topic_map.keys())}")
    
    # Find GPS topics
    fix_topic = None
    cog_topic = None
    sog_topic = None
    velocity_topic = None
    
    for topic_name in topic_map.keys():
        if topic_name == '/fix' or topic_name.endswith('/fix'):
            fix_topic = topic_name
        elif topic_name == '/gps_cog' or topic_name.endswith('/gps_cog'):
            cog_topic = topic_name
        elif topic_name == '/gps_sog' or topic_name.endswith('/gps_sog'):
            sog_topic = topic_name
        elif topic_name == '/gps_velocity' or topic_name.endswith('/gps_velocity'):
            velocity_topic = topic_name
    
    if not fix_topic:
        print("Error: /fix topic not found in bag file")
        return None
    
    print(f"Using topics:")
    print(f"  Fix: {fix_topic}")
    print(f"  COG: {cog_topic}")
    print(f"  SOG: {sog_topic}")
    print(f"  Velocity: {velocity_topic}")
    
    # Read messages
    fix_data = []
    cog_data = []
    sog_data = []
    velocity_data = []
    
    clock = Clock()
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        # Convert timestamp to seconds
        t_sec = timestamp / 1e9
        
        if topic == fix_topic:
            try:
                msg = deserialize_message(data, NavSatFix)
                # Accept any fix with valid coordinates (status >= 0, but check for valid lat/lon)
                if not (math.isnan(msg.latitude) or math.isnan(msg.longitude) or 
                        math.isinf(msg.latitude) or math.isinf(msg.longitude)):
                    fix_data.append((t_sec, msg))
            except Exception as e:
                print(f"Warning: Failed to deserialize NavSatFix: {e}")
        
        elif topic == cog_topic:
            try:
                msg = deserialize_message(data, Float64)
                cog_data.append((t_sec, msg.data))
            except Exception as e:
                print(f"Warning: Failed to deserialize COG: {e}")
        
        elif topic == sog_topic:
            try:
                msg = deserialize_message(data, Float64)
                sog_data.append((t_sec, msg.data))
            except Exception as e:
                print(f"Warning: Failed to deserialize SOG: {e}")
        
        elif topic == velocity_topic:
            try:
                msg = deserialize_message(data, Vector3)
                velocity_data.append((t_sec, msg))
            except Exception as e:
                print(f"Warning: Failed to deserialize velocity: {e}")
    
    # SequentialReader doesn't have close(), it's automatically cleaned up
    del reader
    
    print(f"Read {len(fix_data)} GPS fixes")
    print(f"Read {len(cog_data)} COG measurements")
    print(f"Read {len(sog_data)} SOG measurements")
    print(f"Read {len(velocity_data)} velocity measurements")
    
    return {
        'fix': fix_data,
        'cog': cog_data,
        'sog': sog_data,
        'velocity': velocity_data
    }


def analyze_noise_characteristics(data, name):
    """Analyze GPS noise characteristics from bag data.
    
    Calculates:
    - Position noise standard deviation
    - COG noise standard deviation
    - SOG noise standard deviation
    - Temporal correlation (autocorrelation)
    - Estimated time constants
    """
    print(f"\n{'='*70}")
    print(f"Analyzing: {name}")
    print(f"{'='*70}")
    
    if not data['fix']:
        print("No GPS fix data available")
        return None
    
    # Extract position data
    times = [t for t, _ in data['fix']]
    lats = [msg.latitude for _, msg in data['fix']]
    lons = [msg.longitude for _, msg in data['fix']]
    
    # Use first position as base
    base_lat = lats[0]
    base_lon = lons[0]
    
    # Convert to local XY coordinates
    x_coords = []
    y_coords = []
    for lat, lon in zip(lats, lons):
        x, y = latlon_to_xy(lat, lon, base_lat, base_lon)
        x_coords.append(x)
        y_coords.append(y)
    
    x_coords = np.array(x_coords)
    y_coords = np.array(y_coords)
    times = np.array(times)
    
    # Calculate position noise (deviation from smoothed track)
    # Use a simple moving average to estimate "true" position
    window_size = min(10, len(x_coords) // 10)  # 10% of data or 10 samples
    if window_size < 3:
        window_size = 3
    
    # Smooth the position data
    x_smooth = np.convolve(x_coords, np.ones(window_size)/window_size, mode='same')
    y_smooth = np.convolve(y_coords, np.ones(window_size)/window_size, mode='same')
    
    # Calculate noise (deviation from smoothed)
    x_noise = x_coords - x_smooth
    y_noise = y_coords - y_smooth
    
    # Position noise standard deviation
    position_noise_std = np.sqrt(np.var(x_noise) + np.var(y_noise))
    
    print(f"\nPosition Noise Analysis:")
    print(f"  Total samples: {len(x_coords)}")
    print(f"  Time span: {times[-1] - times[0]:.1f} seconds")
    print(f"  Position noise stddev: {position_noise_std:.2f} meters")
    print(f"  X noise stddev: {np.std(x_noise):.2f} meters")
    print(f"  Y noise stddev: {np.std(y_noise):.2f} meters")
    
    # Calculate autocorrelation to estimate time constant
    if len(x_noise) > 20:
        # Autocorrelation for X noise
        x_autocorr = np.correlate(x_noise, x_noise, mode='full')
        x_autocorr = x_autocorr[len(x_autocorr)//2:]
        x_autocorr = x_autocorr / x_autocorr[0]  # Normalize
        
        # Find time constant (where autocorrelation drops to 1/e ≈ 0.368)
        dt = np.mean(np.diff(times))
        tau_threshold = 1.0 / math.e  # ≈ 0.368
        
        x_tau_idx = np.where(x_autocorr < tau_threshold)[0]
        if len(x_tau_idx) > 0:
            x_tau = x_tau_idx[0] * dt
            print(f"  X noise time constant (1/e): {x_tau:.2f} seconds")
        else:
            print(f"  X noise time constant: > {len(x_autocorr) * dt:.2f} seconds")
    
    # COG analysis
    if data['cog'] and data['sog']:
        cog_times = np.array([t for t, _ in data['cog']])
        cog_values = np.array([v for _, v in data['cog']])
        sog_times = np.array([t for t, _ in data['sog']])
        sog_values = np.array([v for _, v in data['sog']])
        
        # Interpolate SOG to COG times
        sog_interp = np.interp(cog_times, sog_times, sog_values)
        
        # Filter out low-speed periods (COG is unreliable when stationary)
        # Convert SOG from knots to m/s if needed
        if np.mean(sog_interp) > 10:
            sog_interp = sog_interp * 0.514444  # knots to m/s
        
        # Only analyze COG when speed > 0.5 m/s (COG is meaningful)
        speed_threshold = 0.5  # m/s
        valid_mask = sog_interp > speed_threshold
        
        if np.sum(valid_mask) > 10:  # Need at least 10 valid samples
            cog_valid = cog_values[valid_mask]
            
            # Smooth COG
            cog_smooth = np.convolve(cog_valid, np.ones(window_size)/window_size, mode='same')
            cog_noise = cog_valid - cog_smooth
            
            # Handle circular nature of angles (0-360 degrees)
            cog_noise = np.array([((n + 180) % 360) - 180 for n in cog_noise])
            
            cog_std = np.std(cog_noise)
            print(f"\nCOG Noise Analysis:")
            print(f"  Total COG samples: {len(cog_values)}")
            print(f"  Valid samples (speed > {speed_threshold} m/s): {np.sum(valid_mask)}")
            print(f"  COG noise stddev: {cog_std:.2f} degrees")
        else:
            print(f"\nCOG Noise Analysis:")
            print(f"  Insufficient valid COG data (speed > {speed_threshold} m/s)")
            print(f"  Most samples are at low speed where COG is unreliable")
            cog_std = None
    elif data['cog']:
        # No SOG data, analyze all COG
        cog_times = np.array([t for t, _ in data['cog']])
        cog_values = np.array([v for _, v in data['cog']])
        
        # Smooth COG
        cog_smooth = np.convolve(cog_values, np.ones(window_size)/window_size, mode='same')
        cog_noise = cog_values - cog_smooth
        
        # Handle circular nature of angles (0-360 degrees)
        cog_noise = np.array([((n + 180) % 360) - 180 for n in cog_noise])
        
        cog_std = np.std(cog_noise)
        print(f"\nCOG Noise Analysis:")
        print(f"  COG noise stddev: {cog_std:.2f} degrees (no SOG filtering)")
    
    # SOG analysis
    if data['sog']:
        sog_times = np.array([t for t, _ in data['sog']])
        sog_values = np.array([v for _, v in data['sog']])
        
        # Smooth SOG
        sog_smooth = np.convolve(sog_values, np.ones(window_size)/window_size, mode='same')
        sog_noise = sog_values - sog_smooth
        
        sog_std = np.std(sog_noise)
        print(f"\nSOG Noise Analysis:")
        print(f"  SOG noise stddev: {sog_std:.3f} m/s ({sog_std * 1.94384:.3f} knots)")
    
    return {
        'position_std': position_noise_std,
        'x_std': np.std(x_noise),
        'y_std': np.std(y_noise),
        'cog_std': cog_std if data['cog'] else None,
        'sog_std': sog_std if data['sog'] else None,
        'times': times,
        'x_coords': x_coords,
        'y_coords': y_coords,
        'x_noise': x_noise,
        'y_noise': y_noise,
        'cog_data': data['cog'] if data['cog'] else None,
        'sog_data': data['sog'] if data['sog'] else None,
    }


def create_visualization(analysis, name, output_file):
    """Create visualization of GPS track with COG/SOG vectors."""
    print(f"\nCreating visualization: {output_file}")
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot GPS track
    x = analysis['x_coords']
    y = analysis['y_coords']
    
    # Plot track as connected line
    ax.plot(x, y, 'b-', linewidth=1, alpha=0.5, label='GPS Track')
    ax.scatter(x, y, c=analysis['times'], cmap='viridis', s=20, alpha=0.7, label='GPS Fix')
    
    # Add COG/SOG vectors
    if analysis['cog_data'] and analysis['sog_data']:
        # Interpolate COG/SOG to match fix times
        fix_times = analysis['times']
        cog_times = np.array([t for t, _ in analysis['cog_data']])
        cog_values = np.array([v for _, v in analysis['cog_data']])
        sog_times = np.array([t for t, _ in analysis['sog_data']])
        sog_values = np.array([v for _, v in analysis['sog_data']])
        
        # Interpolate to fix times
        cog_interp = np.interp(fix_times, cog_times, cog_values)
        sog_interp = np.interp(fix_times, sog_times, sog_values)
        
        # Convert SOG from knots to m/s if needed (assuming knots if > 10)
        if np.mean(sog_interp) > 10:
            sog_interp = sog_interp * 0.514444  # knots to m/s
        
        # Debug: Check COG variation
        cog_unique = len(np.unique(np.round(cog_interp, 1)))
        cog_std = np.std(cog_interp)
        print(f"  COG statistics: {cog_unique} unique values (rounded to 0.1°), stddev={cog_std:.2f}°")
        print(f"  COG range: {np.min(cog_interp):.1f}° to {np.max(cog_interp):.1f}°")
        print(f"  SOG range: {np.min(sog_interp):.3f} to {np.max(sog_interp):.3f} m/s")
        
        # Warn if COG appears to be constant (likely GPS didn't update COG during recording)
        if cog_unique <= 2:
            print(f"  ⚠️  WARNING: COG appears constant ({cog_unique} unique values)")
            print(f"     This suggests the GPS may not have been updating COG during this recording.")
            print(f"     Possible causes: GPS treated movement as stationary, or COG field was empty in NMEA sentences.")
        
        # Sample every Nth point to avoid clutter
        sample_rate = max(1, len(x) // 50)  # Show ~50 arrows
        
        # Scale factor: 1 m/s = arrow_length meters on plot
        scale_factor = 5.0  # meters of arrow length per m/s
        
        for i in range(0, len(x), sample_rate):
            if i < len(cog_interp) and i < len(sog_interp):
                cog_deg = cog_interp[i]
                sog_ms = sog_interp[i]
                
                # Scale arrow length (1 m/s = scale_factor meters arrow length)
                arrow_length = sog_ms * scale_factor
                
                if arrow_length > 0.1:  # Only show arrows for meaningful speed
                    # COG is degrees true (0° = North, 90° = East)
                    # Convert to radians and calculate components
                    cog_rad = math.radians(cog_deg)
                    dx = arrow_length * math.sin(cog_rad)  # East component
                    dy = arrow_length * math.cos(cog_rad)  # North component
                    
                    ax.arrow(x[i], y[i], dx, dy, 
                            head_width=0.5, head_length=0.5, 
                            fc='red', ec='red', alpha=0.6, length_includes_head=True)
        
        # Add scale bar for SOG vectors
        # Place scale bar in lower right corner
        x_range = np.max(x) - np.min(x)
        y_range = np.max(y) - np.min(y)
        scale_x = np.max(x) - 0.1 * x_range  # 10% from right edge
        scale_y = np.min(y) + 0.05 * y_range  # 5% from bottom
        
        # Draw reference arrow for 1 m/s
        ref_speed = 1.0  # m/s
        ref_length = ref_speed * scale_factor
        ref_dx = ref_length * math.sin(math.radians(0))  # Pointing North
        ref_dy = ref_length * math.cos(math.radians(0))
        
        ax.arrow(scale_x, scale_y, ref_dx, ref_dy,
                head_width=0.5, head_length=0.5,
                fc='blue', ec='blue', alpha=0.8, length_includes_head=True,
                linewidth=2)
        
        # Add text label for scale bar
        ax.text(scale_x + ref_length/2, scale_y - 1.0, 
                f'{ref_speed:.1f} m/s', 
                ha='center', va='top', fontsize=10,
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
    
    ax.set_xlabel('East (meters)', fontsize=12)
    ax.set_ylabel('North (meters)', fontsize=12)
    ax.set_title(f'GPS Track: {name}\nPosition stddev: {analysis["position_std"]:.2f}m', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Analyze GPS noise from ROS2 bag files')
    parser.add_argument('bag_paths', nargs='+', help='Path(s) to bag file(s) or bag directory(ies)')
    parser.add_argument('--output-dir', default='.', help='Output directory for plots (default: current directory)')
    
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Initialize ROS2
    rclpy.init()
    
    all_analyses = []
    
    for bag_path in args.bag_paths:
        if not os.path.exists(bag_path):
            print(f"Warning: Bag path not found: {bag_path}")
            continue
        
        # Read bag file
        data = read_bag_file(bag_path)
        if not data:
            continue
        
        # Analyze
        name = os.path.basename(bag_path.rstrip('/'))
        analysis = analyze_noise_characteristics(data, name)
        if analysis:
            all_analyses.append((name, analysis))
            
            # Create visualization
            output_file = os.path.join(args.output_dir, f"gps_track_{name.replace(' ', '_')}.png")
            create_visualization(analysis, name, output_file)
    
    # Summary comparison
    if len(all_analyses) > 1:
        print(f"\n{'='*70}")
        print("Summary Comparison")
        print(f"{'='*70}")
        print(f"{'Recording':<40} {'Pos stddev':<12} {'COG stddev':<12} {'SOG stddev':<12}")
        print("-" * 70)
        for name, analysis in all_analyses:
            cog_str = f"{analysis['cog_std']:.2f}°" if analysis['cog_std'] else "N/A"
            sog_str = f"{analysis['sog_std']:.3f} m/s" if analysis['sog_std'] else "N/A"
            print(f"{name:<40} {analysis['position_std']:<12.2f} {cog_str:<12} {sog_str:<12}")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

