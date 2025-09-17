#!/usr/bin/env python3
"""
Memory Leak Monitor for Argo ROS2 Nodes
========================================

Monitors system memory and per-node memory usage every 10 seconds for 30 minutes.
Saves data to CSV file for analysis.

Usage:
    nohup python3 memory_monitor.py > memory_monitor.log 2>&1 &
"""

import psutil
import time
import csv
import os
import sys
from datetime import datetime
import subprocess

# Import centralized node utilities
from argo_node_utils import ArgoNodeManager

class MemoryMonitor:
    def __init__(self, duration_minutes=30, interval_seconds=10):
        self.duration_minutes = duration_minutes
        self.interval_seconds = interval_seconds
        self.start_time = time.time()
        self.end_time = self.start_time + (duration_minutes * 60)
        
        # Discover Argo nodes dynamically
        argo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        node_manager = ArgoNodeManager(argo_root)
        discovered_nodes = node_manager.discover_nodes()
        self.expected_nodes = [f"{node}.py" for node in discovered_nodes if node != 'foxglove_bridge']
        
        # CSV file setup
        self.csv_file = f"/home/orangepi/argo/memory_usage_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.setup_csv()
        
        print(f"🔍 Starting memory monitor for {duration_minutes} minutes")
        print(f"📊 Recording every {interval_seconds} seconds")
        print(f"💾 Data will be saved to: {self.csv_file}")
        print(f"⏰ Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏰ Will finish at: {datetime.fromtimestamp(self.end_time).strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 60)
    
    def setup_csv(self):
        """Setup CSV file with headers"""
        with open(self.csv_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            headers = [
                'timestamp', 'elapsed_seconds', 'system_memory_total_gb', 
                'system_memory_used_gb', 'system_memory_percent', 
                'system_memory_available_gb', 'system_memory_cached_gb',
                'system_memory_buffers_gb', 'system_swap_total_gb', 
                'system_swap_used_gb', 'system_swap_percent'
            ]
            
            # Add headers for each expected node
            for node in self.expected_nodes:
                headers.extend([
                    f'{node}_pid', f'{node}_memory_mb', f'{node}_memory_percent',
                    f'{node}_cpu_percent', f'{node}_status'
                ])
            
            writer.writerow(headers)
    
    def get_node_processes(self):
        """Get process information for each expected node"""
        node_processes = {}
        
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'memory_info', 'cpu_percent']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    
                    for node in self.expected_nodes:
                        if node in cmdline and 'python' in cmdline.lower():
                            node_processes[node] = {
                                'pid': proc.pid,
                                'memory_mb': proc.info['memory_info'].rss / 1024 / 1024,  # MB
                                'memory_percent': proc.memory_percent(),
                                'cpu_percent': proc.cpu_percent(),
                                'status': 'RUNNING'
                            }
                            break
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
        except Exception as e:
            print(f"⚠️  Error getting process info: {e}")
        
        # Fill in missing nodes
        for node in self.expected_nodes:
            if node not in node_processes:
                node_processes[node] = {
                    'pid': None,
                    'memory_mb': 0,
                    'memory_percent': 0,
                    'cpu_percent': 0,
                    'status': 'STOPPED'
                }
        
        return node_processes
    
    def get_system_memory(self):
        """Get system memory information"""
        try:
            memory = psutil.virtual_memory()
            swap = psutil.swap_memory()
            
            return {
                'total_gb': memory.total / (1024**3),
                'used_gb': memory.used / (1024**3),
                'percent': memory.percent,
                'available_gb': memory.available / (1024**3),
                'cached_gb': memory.cached / (1024**3) if hasattr(memory, 'cached') else 0,
                'buffers_gb': memory.buffers / (1024**3) if hasattr(memory, 'buffers') else 0,
                'swap_total_gb': swap.total / (1024**3),
                'swap_used_gb': swap.used / (1024**3),
                'swap_percent': swap.percent
            }
        except Exception as e:
            print(f"⚠️  Error getting system memory: {e}")
            return {
                'total_gb': 0, 'used_gb': 0, 'percent': 0, 'available_gb': 0,
                'cached_gb': 0, 'buffers_gb': 0, 'swap_total_gb': 0, 
                'swap_used_gb': 0, 'swap_percent': 0
            }
    
    def record_memory_usage(self):
        """Record current memory usage to CSV"""
        current_time = time.time()
        elapsed_seconds = current_time - self.start_time
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Get system memory
        system_mem = self.get_system_memory()
        
        # Get node processes
        node_processes = self.get_node_processes()
        
        # Prepare CSV row
        row = [
            timestamp,
            f"{elapsed_seconds:.1f}",
            f"{system_mem['total_gb']:.2f}",
            f"{system_mem['used_gb']:.2f}",
            f"{system_mem['percent']:.1f}",
            f"{system_mem['available_gb']:.2f}",
            f"{system_mem['cached_gb']:.2f}",
            f"{system_mem['buffers_gb']:.2f}",
            f"{system_mem['swap_total_gb']:.2f}",
            f"{system_mem['swap_used_gb']:.2f}",
            f"{system_mem['swap_percent']:.1f}"
        ]
        
        # Add node data
        for node in self.expected_nodes:
            proc_info = node_processes[node]
            row.extend([
                proc_info['pid'] or '',
                f"{proc_info['memory_mb']:.1f}",
                f"{proc_info['memory_percent']:.2f}",
                f"{proc_info['cpu_percent']:.1f}",
                proc_info['status']
            ])
        
        # Write to CSV
        try:
            with open(self.csv_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row)
        except Exception as e:
            print(f"⚠️  Error writing to CSV: {e}")
        
        # Print status
        running_nodes = [node for node, info in node_processes.items() if info['status'] == 'RUNNING']
        print(f"[{timestamp}] Elapsed: {elapsed_seconds:.0f}s | "
              f"System: {system_mem['percent']:.1f}% | "
              f"Nodes: {len(running_nodes)}/{len(self.expected_nodes)} | "
              f"Total Memory: {system_mem['used_gb']:.1f}GB/{system_mem['total_gb']:.1f}GB")
    
    def run(self):
        """Run the memory monitoring loop"""
        try:
            while time.time() < self.end_time:
                self.record_memory_usage()
                
                # Sleep for the interval, but check if we should stop
                sleep_start = time.time()
                while time.time() - sleep_start < self.interval_seconds:
                    if time.time() >= self.end_time:
                        break
                    time.sleep(1)
            
            # Final recording
            self.record_memory_usage()
            
            print("\n" + "=" * 60)
            print(f"✅ Memory monitoring completed!")
            print(f"📊 Data saved to: {self.csv_file}")
            print(f"⏰ Finished at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print("=" * 60)
            
        except KeyboardInterrupt:
            print(f"\n🛑 Memory monitoring stopped by user")
            print(f"📊 Partial data saved to: {self.csv_file}")
        except Exception as e:
            print(f"\n❌ Error during monitoring: {e}")
            print(f"📊 Partial data saved to: {self.csv_file}")

def main():
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("Usage: python3 memory_monitor.py [duration_minutes]")
            print("Default duration: 30 minutes")
            duration = 30
    else:
        duration = 30
    
    monitor = MemoryMonitor(duration_minutes=duration)
    monitor.run()

if __name__ == '__main__':
    main()

