#!/usr/bin/env python3
"""
Quick GPS + LoRa plot with immediate progress feedback
"""
import sys
import os
import json
import matplotlib.pyplot as plt
import numpy as np

def quick_plot(bag_path):
    """Create a quick plot with immediate progress feedback"""
    
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String, Int32
        from sensor_msgs.msg import NavSatFix
        
        print(f"🗺️  Quick GPS + LoRa plot from: {bag_path}")
        print("=" * 60)
        
        # Open the bag
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Data storage
        gps_points = []
        lora_pings = []
        signal_data = []
        
        print("📊 Processing data with progress updates...")
        message_count = 0
        last_gps_fix = None
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            timestamp_sec = timestamp / 1e9
            message_count += 1
            
            # Progress every 10k messages
            if message_count % 100 == 0:
                print(f"  📈 {message_count} messages, GPS: {len(gps_points)}, LoRa: {len(lora_pings)}")
            
            if topic == '/fix':
                try:
                    msg = deserialize_message(data, NavSatFix)
                    if msg.status.status >= 0:
                        last_gps_fix = (msg.latitude, msg.longitude, timestamp_sec)
                        gps_points.append(last_gps_fix)
                except:
                    pass
                    
            elif topic == '/lora_signal_strength':
                try:
                    msg = deserialize_message(data, Int32)
                    signal_data.append((timestamp_sec, msg.data))
                except:
                    pass
                    
            elif topic == '/lora_rx_data':
                try:
                    msg = deserialize_message(data, String)
                    if msg.data.strip():
                        try:
                            data_json = json.loads(msg.data)
                            if data_json.get('cmd') == 'ping' and 'seq' in data_json and last_gps_fix:
                                # Find closest signal
                                closest_signal = None
                                min_time_diff = float('inf')
                                for sig_time, sig_strength in signal_data:
                                    time_diff = abs(sig_time - timestamp_sec)
                                    if time_diff < min_time_diff and time_diff < 5.0:
                                        min_time_diff = time_diff
                                        closest_signal = sig_strength
                                
                                lora_pings.append((
                                    last_gps_fix[0], last_gps_fix[1], timestamp_sec,
                                    closest_signal if closest_signal else -999,
                                    data_json['seq']
                                ))
                        except json.JSONDecodeError:
                            pass
                except:
                    pass
        
        print(f"\n✅ Processing complete!")
        print(f"📊 Data: {len(gps_points)} GPS points, {len(lora_pings)} LoRa pings")
        
        if not gps_points or not lora_pings:
            print("❌ Insufficient data for plotting")
            return
        
        # Create plot
        print("🎨 Creating plot...")
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Extract data
        gps_lats = [p[0] for p in gps_points]
        gps_lons = [p[1] for p in gps_points]
        ping_lats = [p[0] for p in lora_pings]
        ping_lons = [p[1] for p in lora_pings]
        ping_signals = [p[3] for p in lora_pings]
        ping_seqs = [p[4] for p in lora_pings]
        
        # Plot 1: GPS track with LoRa pings
        print("  📍 Plotting GPS track...")
        ax1.plot(gps_lons, gps_lats, 'b-', linewidth=2, alpha=0.7, label='GPS Track')
        
        print(f"  📡 Plotting {len(ping_lats)} LoRa ping points...")
        for i, (lat, lon, signal, seq) in enumerate(zip(ping_lats, ping_lons, ping_signals, ping_seqs)):
            if signal == -999:
                color = 'gray'
                size = 30
            elif signal >= -80:
                color = 'green'
                size = 60
            elif signal >= -90:
                color = 'orange'
                size = 45
            else:
                color = 'red'
                size = 30
            
            ax1.scatter(lon, lat, c=color, s=size, alpha=0.8, edgecolors='black', linewidth=0.5)
            
            # Add sequence numbers for some points
            if i % 10 == 0:
                ax1.annotate(f'#{seq}', (lon, lat), xytext=(5, 5), 
                           textcoords='offset points', fontsize=8, alpha=0.7)
        
        ax1.set_xlabel('Longitude')
        ax1.set_ylabel('Latitude')
        ax1.set_title('GPS Track with LoRa Ping Reception Points')
        ax1.grid(True, alpha=0.3)
        
        # Legend
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
        ping_times = [(p[2] - gps_points[0][2]) / 60 for p in lora_pings]
        
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
        ax2.axhline(y=-80, color='green', linestyle='--', alpha=0.5, label='Good threshold')
        ax2.axhline(y=-90, color='orange', linestyle='--', alpha=0.5, label='Fair threshold')
        ax2.legend()
        
        # Save plot
        print("  💾 Saving plot...")
        plt.tight_layout()
        output_file = 'gps_lora_track.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ Plot saved to: {output_file}")
        
        # Show plot
        print("  🖼️  Displaying plot...")
        plt.show()
        
        # Summary
        print(f"\n📊 Summary:")
        print(f"  • GPS points: {len(gps_points)}")
        print(f"  • LoRa pings: {len(lora_pings)}")
        print(f"  • Pings with signal data: {len([s for s in ping_signals if s != -999])}")
        
        if valid_signals:
            print(f"  • Average signal strength: {np.mean(valid_signals):.1f} dBm")
            print(f"  • Signal range: {min(valid_signals)} to {max(valid_signals)} dBm")
        
        # Distance calculation
        if len(gps_points) > 1:
            total_distance = 0
            for i in range(1, len(gps_points)):
                lat1, lon1 = gps_points[i-1][0], gps_points[i-1][1]
                lat2, lon2 = gps_points[i][0], gps_points[i][1]
                distance = np.sqrt((lat2-lat1)**2 + (lon2-lon1)**2) * 111000
                total_distance += distance
            print(f"  • Approximate distance: {total_distance:.0f} meters")
        
        print(f"  • Ping sequence range: {min(ping_seqs)} to {max(ping_seqs)}")
        
    except ImportError as e:
        print(f"❌ Missing library: {e}")
        print("Install with: pip install matplotlib numpy")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "~/argo/bags/argo_20251024_133837"
    bag_path = os.path.expanduser(bag_path)
    
    quick_plot(bag_path)

if __name__ == "__main__":
    main()
