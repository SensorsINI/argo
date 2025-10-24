#!/usr/bin/env python3
"""
Check LoRa signal strength data from the latest recording
"""
import sys
import os

def check_signal_strength(bag_path):
    """Check LoRa signal strength statistics"""
    
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Int32
        
        # Open the bag
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        print(f"📶 LoRa Signal Strength Analysis: {bag_path}")
        print("=" * 50)
        
        signal_values = []
        
        # Process all messages
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == '/lora_signal_strength':
                try:
                    msg = deserialize_message(data, Int32)
                    signal_values.append(msg.data)
                except Exception as e:
                    pass
        
        if signal_values:
            print(f"📊 Signal Strength Statistics:")
            print(f"  • Total measurements: {len(signal_values)}")
            print(f"  • Min signal strength: {min(signal_values)} dBm")
            print(f"  • Max signal strength: {max(signal_values)} dBm")
            print(f"  • Average signal strength: {sum(signal_values)/len(signal_values):.1f} dBm")
            print()
            
            # Show some samples
            print("📡 Signal Strength Samples:")
            for i, signal in enumerate(signal_values[:10]):
                print(f"  Sample {i+1}: {signal} dBm")
            
            if len(signal_values) > 10:
                print(f"  ... and {len(signal_values) - 10} more")
            
            # Signal quality assessment
            print("\n📋 Signal Quality Assessment:")
            good_signals = [s for s in signal_values if s >= -80]  # Good signal threshold
            fair_signals = [s for s in signal_values if -90 <= s < -80]  # Fair signal threshold
            poor_signals = [s for s in signal_values if s < -90]  # Poor signal threshold
            
            print(f"  • Good signals (≥-80 dBm): {len(good_signals)} ({len(good_signals)/len(signal_values)*100:.1f}%)")
            print(f"  • Fair signals (-90 to -80 dBm): {len(fair_signals)} ({len(fair_signals)/len(signal_values)*100:.1f}%)")
            print(f"  • Poor signals (<-90 dBm): {len(poor_signals)} ({len(poor_signals)/len(signal_values)*100:.1f}%)")
            
        else:
            print("❌ No signal strength data found")
        
    except ImportError:
        print("❌ rosbag2_py not available")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "~/argo/bags/argo_20251024_122903"
    bag_path = os.path.expanduser(bag_path)
    
    check_signal_strength(bag_path)

if __name__ == "__main__":
    main()
