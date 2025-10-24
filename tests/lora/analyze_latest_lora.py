#!/usr/bin/env python3
"""
Analyze the latest LoRa recording with GPS fix and shore-side running
"""
import sys
import os

def analyze_latest_lora(bag_path):
    """Analyze LoRa data from the latest recording"""
    
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String, Int32, Bool
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Float64
        from geometry_msgs.msg import Vector3
        
        # Open the bag
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        print(f"🔍 Analyzing LoRa data from: {bag_path}")
        print("=" * 70)
        
        # Statistics
        lora_rx_count = 0
        lora_signal_count = 0
        gps_fix_count = 0
        gps_sog_count = 0
        gps_cog_count = 0
        gps_velocity_count = 0
        
        # Data samples
        lora_rx_samples = []
        gps_samples = []
        
        # Process all messages
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == '/lora_rx_data':
                try:
                    msg = deserialize_message(data, String)
                    lora_rx_count += 1
                    
                    # Collect samples for analysis
                    if len(lora_rx_samples) < 10 and msg.data.strip():
                        lora_rx_samples.append(msg.data)
                        
                except Exception as e:
                    pass
                    
            elif topic == '/lora_signal_strength':
                try:
                    msg = deserialize_message(data, Int32)
                    lora_signal_count += 1
                except Exception as e:
                    pass
                    
            elif topic == '/fix':
                try:
                    msg = deserialize_message(data, NavSatFix)
                    if msg.status.status >= 0:  # Valid GPS fix
                        gps_fix_count += 1
                        if len(gps_samples) < 5:
                            gps_samples.append((msg.latitude, msg.longitude, msg.altitude))
                except Exception as e:
                    pass
                    
            elif topic == '/gps_sog':
                try:
                    msg = deserialize_message(data, Float64)
                    gps_sog_count += 1
                except Exception as e:
                    pass
                    
            elif topic == '/gps_cog':
                try:
                    msg = deserialize_message(data, Float64)
                    gps_cog_count += 1
                except Exception as e:
                    pass
                    
            elif topic == '/gps_velocity':
                try:
                    msg = deserialize_message(data, Vector3)
                    gps_velocity_count += 1
                except Exception as e:
                    pass
        
        print("📊 LoRa Communication Statistics:")
        print(f"  • LoRa RX Data Messages: {lora_rx_count}")
        print(f"  • LoRa Signal Strength Messages: {lora_signal_count}")
        print(f"  • GPS Fix Messages: {gps_fix_count}")
        print(f"  • GPS Speed over Ground: {gps_sog_count}")
        print(f"  • GPS Course over Ground: {gps_cog_count}")
        print(f"  • GPS Velocity Messages: {gps_velocity_count}")
        print()
        
        # Calculate rates
        duration = 338.2  # seconds from bag info
        print("📈 Communication Rates:")
        print(f"  • LoRa RX Rate: {lora_rx_count/duration:.2f} msg/sec")
        print(f"  • Signal Strength Rate: {lora_signal_count/duration:.2f} msg/sec")
        print(f"  • GPS Fix Rate: {gps_fix_count/duration:.2f} msg/sec")
        print()
        
        # Analyze LoRa data samples
        print("📡 LoRa Data Samples:")
        print("-" * 40)
        
        json_count = 0
        empty_count = 0
        binary_count = 0
        
        for i, sample in enumerate(lora_rx_samples):
            print(f"Sample {i+1}:")
            if not sample or not sample.strip():
                print("  → Empty/whitespace")
                empty_count += 1
            elif sample.startswith('{') and sample.endswith('}'):
                print(f"  → JSON: {sample}")
                json_count += 1
            elif sample.startswith('{'):
                print(f"  → Partial JSON: {sample[:50]}...")
                json_count += 1
            else:
                display_str = ''.join(c if 32 <= ord(c) <= 126 else '.' for c in sample[:50])
                print(f"  → Binary/Text: {display_str}")
                print(f"  → Length: {len(sample)} bytes")
                binary_count += 1
            print()
        
        print("📋 LoRa Data Analysis:")
        print(f"  • JSON packets: {json_count}")
        print(f"  • Empty packets: {empty_count}")
        print(f"  • Binary/other: {binary_count}")
        
        if json_count > 0:
            print("✅ SUCCESS: Valid JSON packets received!")
        else:
            print("❌ No valid JSON packets found")
        
        # GPS samples
        if gps_samples:
            print("\n🌍 GPS Fix Samples:")
            print("-" * 40)
            for i, (lat, lon, alt) in enumerate(gps_samples):
                print(f"Fix {i+1}: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m")
        
        # Check for lora/ prefixed topics (should be created by shore-side parsing)
        print("\n🔍 Checking for lora/ prefixed topics...")
        print("(These would indicate successful JSON parsing by shore-side)")
        
        # Get all topics from bag
        topic_types = reader.get_all_topics_and_types()
        lora_topics = [t.name for t in topic_types if t.name.startswith('/lora/')]
        
        if lora_topics:
            print("✅ Found lora/ prefixed topics:")
            for topic in lora_topics:
                print(f"  • {topic}")
        else:
            print("❌ No lora/ prefixed topics found")
            print("   This suggests shore-side JSON parsing may not be working")
        
    except ImportError:
        print("❌ rosbag2_py not available")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "~/argo/bags/argo_20251024_122903"
    bag_path = os.path.expanduser(bag_path)
    
    analyze_latest_lora(bag_path)

if __name__ == "__main__":
    main()
