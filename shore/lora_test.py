#!/usr/bin/env python3
"""
Shore-Side LoRa Test Script
Monitors LoRa communication, logs packets with timestamps, validates bandwidth
"""

import serial
import json
import time
import sys
from datetime import datetime
from pathlib import Path

class LoRaTestMonitor:
    def __init__(self, port='/dev/ttyACM0', baud=115200, log_dir='lora_logs'):
        self.port = port
        self.baud = baud
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        # Statistics
        self.packets_received = 0
        self.packets_sent = 0
        self.last_packet_time = None
        self.packet_intervals = []
        self.packet_sizes = []
        
        # Start log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"shore_lora_{timestamp}.log"
        
    def log(self, message):
        """Log message to console and file with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {message}"
        print(log_line)
        with open(self.log_file, 'a') as f:
            f.write(log_line + '\n')
    
    def connect(self):
        """Connect to LoRa module"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.log(f"✓ Connected to {self.port} at {self.baud} baud")
            return True
        except Exception as e:
            self.log(f"✗ Connection failed: {e}")
            return False
    
    def send_ping(self, seq):
        """Send ping message to Argo"""
        try:
            ping = json.dumps({'cmd': 'ping', 'seq': seq}, separators=(',', ':'))
            self.ser.write(ping.encode('utf-8'))
            self.ser.flush()
            self.packets_sent += 1
            self.log(f"→ PING #{seq} sent ({len(ping)} bytes)")
            return True
        except Exception as e:
            self.log(f"✗ Ping send failed: {e}")
            return False
    
    def send_command(self, command):
        """Send command to Argo"""
        try:
            cmd = json.dumps({'cmd': command}, separators=(',', ':'))
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.packets_sent += 1
            self.log(f"→ COMMAND '{command}' sent ({len(cmd)} bytes)")
            return True
        except Exception as e:
            self.log(f"✗ Command send failed: {e}")
            return False
    
    def process_packet(self, json_str):
        """Process and log received packet"""
        try:
            packet = json.loads(json_str)
            self.packets_received += 1
            
            # Calculate interval since last packet
            now = time.time()
            if self.last_packet_time:
                interval = now - self.last_packet_time
                self.packet_intervals.append(interval)
            self.last_packet_time = now
            
            # Track packet size
            self.packet_sizes.append(len(json_str))
            
            # Log packet details
            self.log(f"← PACKET #{self.packets_received} ({len(json_str)} bytes): {json_str}")
            
            # Parse and display key fields
            fields = []
            if 'lat' in packet and 'lon' in packet:
                fields.append(f"GPS: {packet['lat']:.6f}, {packet['lon']:.6f}")
            if 'sog' in packet:
                fields.append(f"SOG: {packet['sog']} kt")
            if 'cog' in packet:
                fields.append(f"COG: {packet['cog']}°")
            if 'bat' in packet:
                fields.append(f"BAT: {packet['bat']}V")
            if 'hum' in packet:
                fields.append(f"CTRL: {'Human' if packet['hum'] else 'Robot'}")
            if 'hdg' in packet:
                fields.append(f"HDG: {packet['hdg']}°")
            if 'seq' in packet:
                fields.append(f"SEQ: {packet['seq']}")
            
            if fields:
                self.log(f"  └─ {' | '.join(fields)}")
                
        except json.JSONDecodeError as e:
            self.log(f"✗ JSON parse error: {e}")
            self.log(f"  Raw data: {json_str}")
    
    def print_statistics(self):
        """Print communication statistics"""
        self.log("\n" + "="*60)
        self.log("STATISTICS")
        self.log("="*60)
        self.log(f"Packets received: {self.packets_received}")
        self.log(f"Packets sent: {self.packets_sent}")
        
        if self.packet_intervals:
            avg_interval = sum(self.packet_intervals) / len(self.packet_intervals)
            min_interval = min(self.packet_intervals)
            max_interval = max(self.packet_intervals)
            self.log(f"Packet intervals: avg={avg_interval:.2f}s, min={min_interval:.2f}s, max={max_interval:.2f}s")
        
        if self.packet_sizes:
            avg_size = sum(self.packet_sizes) / len(self.packet_sizes)
            min_size = min(self.packet_sizes)
            max_size = max(self.packet_sizes)
            self.log(f"Packet sizes: avg={avg_size:.1f} bytes, min={min_size}, max={max_size}")
            
            # Estimate bandwidth usage
            if self.packet_intervals:
                avg_interval = sum(self.packet_intervals) / len(self.packet_intervals)
                bits_per_second = (avg_size * 8) / avg_interval
                self.log(f"Estimated bandwidth: {bits_per_second:.1f} bps")
        
        self.log("="*60)
    
    def run(self):
        """Main test loop"""
        if not self.connect():
            return
        
        self.log("\nStarting LoRa test monitor...")
        self.log("Commands: 'ping', 'rth' (return to home), 'stats', 'quit'\n")
        
        buffer = ""
        last_ping_time = 0
        ping_sequence = 0
        
        try:
            while True:
                # Auto-ping every 5 seconds
                now = time.time()
                if now - last_ping_time > 5.0:
                    ping_sequence += 1
                    self.send_ping(ping_sequence)
                    last_ping_time = now
                
                # Read from serial
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    text = data.decode('utf-8', errors='ignore')
                    buffer += text
                    
                    # Process complete JSON packets
                    while '{' in buffer and '}' in buffer:
                        json_start = buffer.index('{')
                        json_end = buffer.index('}', json_start) + 1
                        json_str = buffer[json_start:json_end].strip()
                        buffer = buffer[json_end:]
                        
                        self.process_packet(json_str)
                
                # Check for user input (non-blocking)
                # Note: This is simplified; real implementation would use select() or threads
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            self.log("\n\nTest interrupted by user")
            self.print_statistics()
            self.log(f"\nLog saved to: {self.log_file}")

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Shore-side LoRa test monitor')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--log-dir', default='lora_logs', help='Log directory')
    
    args = parser.parse_args()
    
    monitor = LoRaTestMonitor(args.port, args.baud, args.log_dir)
    monitor.run()

