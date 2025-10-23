#!/usr/bin/env python3
"""
Argo-Side LoRa Test Script
Monitors LoRa node behavior, logs packets with timestamps, validates reception
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
import json
import time
from datetime import datetime
from pathlib import Path

class ArgoLoRaTestMonitor(Node):
    def __init__(self):
        super().__init__('argo_lora_test_monitor')
        
        # Setup logging
        self.log_dir = Path.home() / 'lora_logs'
        self.log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"argo_lora_{timestamp}.log"
        
        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.pings_received = 0
        self.last_ping_time = None
        self.ping_intervals = []
        self.packet_loss_count = 0
        self.expected_sequence = 1
        
        # Subscribe to LoRa topics
        self.create_subscription(String, 'lora_rx_data', self.rx_callback, 10)
        self.create_subscription(String, 'lora_remote_command', self.cmd_callback, 10)
        self.create_subscription(Int32, 'lora_signal_strength', self.rssi_callback, 10)
        
        # Monitor what we're transmitting (would need to tap into lora node)
        # For now, we'll subscribe to the data sources that lora node uses
        self.create_subscription(Float64, 'gps_sog', self.data_update_callback, 10)
        
        self.log("Argo LoRa Test Monitor Started")
        
        # Timer for periodic statistics
        self.create_timer(30.0, self.print_statistics)
    
    def log(self, message):
        """Log with timestamp to console and file"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {message}"
        self.get_logger().info(message)
        with open(self.log_file, 'a') as f:
            f.write(log_line + '\n')
    
    def rx_callback(self, msg):
        """Monitor received LoRa data"""
        self.packets_received += 1
        self.log(f"← RX PACKET #{self.packets_received}: {msg.data}")
        
        try:
            data = json.loads(msg.data)
            if data.get('cmd') == 'ping':
                self.pings_received += 1
                now = time.time()
                if self.last_ping_time:
                    interval = now - self.last_ping_time
                    self.ping_intervals.append(interval)
                    self.log(f"  └─ PING #{data.get('seq', '?')} (interval: {interval:.2f}s)")
                self.last_ping_time = now
            elif 'seq' in data:
                # Check for packet loss
                received_seq = data['seq']
                if received_seq != self.expected_sequence:
                    self.packet_loss_count += 1
                    self.log(f"  └─ PACKET LOSS: expected {self.expected_sequence}, got {received_seq}")
                self.expected_sequence = received_seq + 1
        except:
            pass
    
    def cmd_callback(self, msg):
        """Monitor parsed commands"""
        self.log(f"← COMMAND: {msg.data}")
    
    def rssi_callback(self, msg):
        """Monitor signal strength"""
        self.log(f"  └─ RSSI: {msg.data} dBm")
    
    def data_update_callback(self, msg):
        """Monitor when data is updated (triggers TX)"""
        # This is a simplified monitor - actual TX happens in lora node
        pass
    
    def print_statistics(self):
        """Print periodic statistics"""
        self.log("\n" + "="*60)
        self.log("STATISTICS (last 30 seconds)")
        self.log("="*60)
        self.log(f"Packets received: {self.packets_received}")
        self.log(f"Pings received: {self.pings_received}")
        
        if self.ping_intervals:
            avg_interval = sum(self.ping_intervals) / len(self.ping_intervals)
            self.log(f"Avg ping interval: {avg_interval:.2f}s (expected: 5.0s)")
            
            # Check for missing pings
            expected_pings = 30 / 5  # 30 second window, 5 second interval
            if self.pings_received < expected_pings * 0.8:
                self.log(f"⚠ WARNING: Missing pings! Expected ~{expected_pings}, got {self.pings_received}")
        
        if self.packets_received > 0:
            loss_rate = (self.packet_loss_count / self.packets_received) * 100
            self.log(f"Packet loss rate: {loss_rate:.1f}% ({self.packet_loss_count}/{self.packets_received})")
            if loss_rate > 10.0:
                self.log(f"⚠ WARNING: High packet loss detected: {loss_rate:.1f}%")
        
        self.log("="*60 + "\n")

def main():
    rclpy.init()
    monitor = ArgoLoRaTestMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.log("\nTest interrupted by user")
        monitor.print_statistics()
        monitor.log(f"\nLog saved to: {monitor.log_file}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

