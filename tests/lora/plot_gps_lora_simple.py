#!/usr/bin/env python3
"""
Plot GPS track with LoRa ping reception points (simplified version)
"""
import sys
import os
import json
from datetime import datetime

def plot_gps_lora_track(bag_path):
    """Create a plot of GPS track with LoRa ping reception points"""
    
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String, Int32
        from sensor_msgs.msg import NavSatFix
        import matplotlib.pyplot as plt
        import numpy as np
        
        # Open the bag
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        print(f"🗺️  Creating GPS + LoRa plot from: {bag_path}")
        print("=" * 60)
        
        # Data storage
        gps_points = []  # (lat, lon, timestamp, alt)
        lora_pings = []  # (lat, lon, timestamp, signal_strength, seq)
        signal_strength_data = []  # (timestamp, signal_strength)
        
        # Single pass through all messages
        print("📊 Processing bag data...")
        last_gps_fix = None
        
        # Progress tracking
        message_count = 0
        gps_count = 0
        lora_count = 0
        signal_count = 0
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            timestamp_sec = timestamp / 1e9
            message_count += 1
            
            # Progress reporting every 5000 messages
            if message_count % 5000 == 0:
                print(f"  📈 Processed {message_count} messages... (GPS: {gps_count}, LoRa: {lora_count}, Signal: {signal_count})")
            
            if topic == '/fix':
                try:
                    msg = deserialize_message(data, NavSatFix)
                    if msg.status.status >= 0:  # Valid GPS fix
                        last_gps_fix = (msg.latitude, msg.longitude, timestamp_sec, msg.altitude)
                        gps_points.append(last_gps_fix)
                        gps_count += 1
                except Exception as e:
                    pass
                    
            elif topic == '/lora_signal_strength':
                try:
                    msg = deserialize_message(data, Int32)
                    signal_strength_data.append((timestamp_sec, msg.data))
                    signal_count += 1
                except Exception as e:
                    pass
                    
            elif topic == '/lora_rx_data':
                try:
                    msg = deserialize_message(data, String)
                    if msg.data.strip():
                        # Try to parse as JSON
                        try:
                            data_json = json.loads(msg.data)
                            if data_json.get('cmd') == 'ping' and 'seq' in data_json:
                                # Find closest signal strength measurement
                                closest_signal = None
                                min_time_diff = float('inf')
                                
                                for sig_time, sig_strength in signal_strength_data:
                                    time_diff = abs(sig_time - timestamp_sec)
                                    if time_diff < min_time_diff and time_diff < 5.0:  # Within 5 seconds
                                        min_time_diff = time_diff
                                        closest_signal = sig_strength
                                
                                # Use last GPS fix if available
                                if last_gps_fix:
                                    lora_pings.append((
                                        last_gps_fix[0],  # lat
                                        last_gps_fix[1],  # lon
                                        timestamp_sec,
                                        closest_signal if closest_signal else -999,
                                        data_json['seq']
                                    ))
                                    lora_count += 1
                        except json.JSONDecodeError:
                            pass
                except Exception as e:
                    pass
        
        print(f"\n✅ Processing complete!")
        print(f"📊 Data collected:")
        print(f"  • Total messages processed: {message_count}")
        print(f"  • GPS points: {len(gps_points)}")
        print(f"  • LoRa pings: {len(lora_pings)}")
        print(f"  • Signal strength measurements: {len(signal_strength_data)}")
        
        if not gps_points:
            print("❌ No GPS data found")
            return
        
        if not lora_pings:
            print("❌ No LoRa ping data found")
            return
        
        # Create the plot
        print("🎨 Creating plot...")
        print("  📊 Extracting GPS coordinates...")
        
        # Extract data for plotting
        gps_lats = [p[0] for p in gps_points]
        gps_lons = [p[1] for p in gps_points]
        
        print("  📊 Extracting LoRa ping data...")
        
        ping_lats = [p[0] for p in lora_pings]
        ping_lons = [p[1] for p in lora_pings]
        ping_signals = [p[3] for p in lora_pings]
        ping_seqs = [p[4] for p in lora_pings]
        
        # Create figure with subplots
        print("  🎨 Creating figure and subplots...")
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Plot 1: GPS track with LoRa ping points
        print("  📍 Plotting GPS track...")
        ax1.plot(gps_lons, gps_lats, 'b-', linewidth=2, alpha=0.7, label='GPS Track')
        
        # Color-code ping points by signal strength
        print(f"  📡 Plotting {len(ping_lats)} LoRa ping points...")
        for i, (lat, lon, signal, seq) in enumerate(zip(ping_lats, ping_lons, ping_signals, ping_seqs)):
            if signal == -999:
                color = 'gray'
                size = 30
            elif signal >= -80:
                color = 'green'  # Good signal
                size = 60
            elif signal >= -90:
                color = 'orange'  # Fair signal
                size = 45
            else:
                color = 'red'  # Poor signal
                size = 30
            
            ax1.scatter(lon, lat, c=color, s=size, alpha=0.8, edgecolors='black', linewidth=0.5)
            
            # Add sequence number for some points
            if i % 5 == 0:  # Every 5th ping
                ax1.annotate(f'#{seq}', (lon, lat), xytext=(5, 5), 
                           textcoords='offset points', fontsize=8, alpha=0.7)
        
        ax1.set_xlabel('Longitude')
        ax1.set_ylabel('Latitude')
        ax1.set_title('GPS Track with LoRa Ping Reception Points')
        ax1.grid(True, alpha=0.3)
        
        # Add legend for signal strength colors
        legend_elements = [
            plt.Line2D([0], [0], color='blue', linewidth=2, label='GPS Track'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=8, label='Good (≥-80 dBm)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=8, label='Fair (-90 to -80 dBm)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Poor (<-90 dBm)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', markersize=8, label='No Signal Data')
        ]
        ax1.legend(handles=legend_elements, loc='upper right')
        
        # Plot 2: Signal strength over time
        print("  📊 Creating signal strength plot...")
        ping_times = [(p[2] - gps_points[0][2]) / 60 for p in lora_pings]  # Minutes from start
        
        # Filter out -999 values for cleaner plot
        valid_times = []
        valid_signals = []
        for time, signal in zip(ping_times, ping_signals):
            if signal != -999:
                valid_times.append(time)
                valid_signals.append(signal)
        
        if valid_times:
            ax2.scatter(valid_times, valid_signals, c='blue', alpha=0.6, s=30)
            ax2.plot(valid_times, valid_signals, 'b-', alpha=0.3, linewidth=1)
        
        ax2.set_xlabel('Time (minutes from start)')
        ax2.set_ylabel('Signal Strength (dBm)')
        ax2.set_title('LoRa Signal Strength Over Time')
        ax2.grid(True, alpha=0.3)
        
        # Add signal quality reference lines
        ax2.axhline(y=-80, color='green', linestyle='--', alpha=0.5, label='Good threshold')
        ax2.axhline(y=-90, color='orange', linestyle='--', alpha=0.5, label='Fair threshold')
        ax2.legend()
        
        # Adjust layout and save
        print("  💾 Saving plot...")
        plt.tight_layout()
        
        # Save plot
        output_file = '/tmp/gps_lora_track.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ Plot saved to: {output_file}")
        
        # Show plot
        print("  🖼️  Displaying plot...")
        plt.show()
        
        # Print summary statistics
        print("\n📊 Summary Statistics:")
        print(f"  • Total GPS points: {len(gps_points)}")
        print(f"  • Total LoRa pings: {len(lora_pings)}")
        print(f"  • Pings with signal data: {len([s for s in ping_signals if s != -999])}")
        
        if valid_signals:
            print(f"  • Average signal strength: {np.mean(valid_signals):.1f} dBm")
            print(f"  • Min signal strength: {min(valid_signals)} dBm")
            print(f"  • Max signal strength: {max(valid_signals)} dBm")
        
        # Calculate distance traveled
        if len(gps_points) > 1:
            total_distance = 0
            for i in range(1, len(gps_points)):
                # Simple distance calculation (not great circle)
                lat1, lon1 = gps_points[i-1][0], gps_points[i-1][1]
                lat2, lon2 = gps_points[i][0], gps_points[i][1]
                distance = np.sqrt((lat2-lat1)**2 + (lon2-lon1)**2) * 111000  # Rough conversion to meters
                total_distance += distance
            
            print(f"  • Approximate distance traveled: {total_distance:.0f} meters")
        
        # Show some sample ping data
        print(f"\n📡 Sample LoRa Ping Data:")
        for i, ping in enumerate(lora_pings[:10]):
            lat, lon, timestamp, signal, seq = ping
            print(f"  Ping #{seq}: Lat={lat:.6f}, Lon={lon:.6f}, Signal={signal} dBm")
        
    except ImportError as e:
        print(f"❌ Missing required library: {e}")
        print("Install with: pip install matplotlib numpy")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "~/argo/bags/argo_20251024_122903"
    bag_path = os.path.expanduser(bag_path)
    
    plot_gps_lora_track(bag_path)

if __name__ == "__main__":
    main()
