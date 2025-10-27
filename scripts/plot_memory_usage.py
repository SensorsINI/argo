#!/usr/bin/env python3
"""
Memory Usage Plotter for Argo ROS2 Nodes
========================================

Creates plots of memory usage over time from the CSV data collected by memory_monitor.py
Uses only matplotlib and built-in CSV module.
"""

import csv
import matplotlib.pyplot as plt
import os
import sys
from datetime import datetime

def plot_memory_usage(csv_file):
    """Plot memory usage over time for all nodes"""
    
    print(f"📊 Reading data from: {csv_file}")
    
    # Read CSV data
    data = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)
    
    if not data:
        print("❌ No data found in CSV file")
        return
    
    # Convert to lists for plotting
    elapsed_minutes = [float(row['elapsed_seconds']) / 60.0 for row in data]
    system_memory_gb = [float(row['system_memory_used_gb']) for row in data]
    
    # Get node names (excluding anem.py since it's not running)
    expected_nodes = ['pwm.py', 'gps.py', 'imu.py', 'control.py', 'argo_battery_water.py', 'temp_monitor.py']
    
    # Create the plot with dual y-axes
    fig, ax1 = plt.subplots(figsize=(14, 10))
    
    # Left y-axis for system memory
    ax1.plot(elapsed_minutes, system_memory_gb, 
             linewidth=3, color='black', linestyle='-', 
             label='System Total Memory (GB)', alpha=0.8)
    
    ax1.set_xlabel('Time (minutes)', fontsize=18, fontweight='bold')
    ax1.set_ylabel('System Memory Usage (GB)', fontsize=18, fontweight='bold', color='black')
    ax1.tick_params(axis='y', labelcolor='black', labelsize=16)
    ax1.tick_params(axis='x', labelsize=16)
    ax1.grid(True, alpha=0.3, linestyle='--')
    
    # Set system memory y-axis limits
    ax1.set_ylim(0, max(system_memory_gb) * 1.1)
    
    # Right y-axis for individual node memory
    ax2 = ax1.twinx()
    
    # Define colors for each node
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
    
    # Plot individual node memory usage on right axis
    max_node_memory = 0
    node_labels = []  # Store node labels for direct labeling
    
    for i, node in enumerate(expected_nodes):
        memory_col = f'{node}_memory_mb'
        status_col = f'{node}_status'
        
        if memory_col in data[0]:  # Check if column exists
            # Extract data for this node
            node_memory_mb = []
            node_times = []
            
            for j, row in enumerate(data):
                if row[status_col] == 'RUNNING' and row[memory_col]:
                    try:
                        memory_val = float(row[memory_col])
                        node_memory_mb.append(memory_val)  # Keep in MB for right axis
                        node_times.append(elapsed_minutes[j])
                        max_node_memory = max(max_node_memory, memory_val)
                    except (ValueError, TypeError):
                        continue
            
            if node_times:  # Only plot if we have data
                line = ax2.plot(node_times, node_memory_mb, 
                        linewidth=3, color=colors[i % len(colors)], 
                        linestyle='-', marker='o', markersize=5,
                        label=f'{node} (MB)', alpha=0.8)
                
                # Add direct label to the end of each trace
                if node_memory_mb:  # Only if we have data points
                    last_x = node_times[-1]
                    last_y = node_memory_mb[-1]
                    ax2.annotate(f'{node}', 
                               xy=(last_x, last_y), 
                               xytext=(10, 0), 
                               textcoords='offset points',
                               fontsize=14, 
                               fontweight='bold',
                               color=colors[i % len(colors)],
                               bbox=dict(boxstyle='round,pad=0.3', 
                                        facecolor='white', 
                                        alpha=0.8,
                                        edgecolor=colors[i % len(colors)]),
                               ha='left', va='center')
    
    ax2.set_ylabel('Individual Node Memory Usage (MB)', fontsize=18, fontweight='bold', color='blue')
    ax2.tick_params(axis='y', labelcolor='blue', labelsize=16)
    ax2.set_ylim(0, max_node_memory * 1.1)
    
    # Set x-axis limits
    ax1.set_xlim(0, max(elapsed_minutes))
    
    # Title
    plt.title('Argo ROS2 Nodes Memory Usage Over Time\n(30-minute monitoring period)', 
              fontsize=20, fontweight='bold', pad=20)
    
    # Add legend inside the plot area
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, 
              loc='upper right', fontsize=14, 
              framealpha=0.9, fancybox=True, shadow=True)
    
    # Add statistics text
    total_memory = float(data[0]['system_memory_total_gb'])
    max_used = max(system_memory_gb)
    min_used = min(system_memory_gb)
    memory_variation = max_used - min_used
    
    stats_text = f"""System Memory Stats:
Total: {total_memory:.1f} GB
Max Used: {max_used:.1f} GB ({max_used/total_memory*100:.1f}%)
Min Used: {min_used:.1f} GB ({min_used/total_memory*100:.1f}%)
Variation: {memory_variation:.1f} GB"""
    
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
             fontsize=14, fontfamily='monospace')
    
    # Adjust layout to prevent legend cutoff
    plt.tight_layout()
    
    # Save as PNG
    png_file = csv_file.replace('.csv', '_memory_plot.png')
    plt.savefig(png_file, dpi=300, bbox_inches='tight')
    print(f"📈 PNG plot saved: {png_file}")
    
    # Save as PDF
    pdf_file = csv_file.replace('.csv', '_memory_plot.pdf')
    plt.savefig(pdf_file, bbox_inches='tight')
    print(f"📄 PDF plot saved: {pdf_file}")
    
    # Show the plot
    plt.show()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("📊 MEMORY USAGE SUMMARY")
    print("="*60)
    print(f"Monitoring duration: {max(elapsed_minutes):.1f} minutes")
    print(f"Data points: {len(data)}")
    print(f"System memory total: {total_memory:.1f} GB")
    print(f"System memory range: {min_used:.1f} - {max_used:.1f} GB")
    print(f"Memory variation: {memory_variation:.1f} GB ({memory_variation/total_memory*100:.1f}%)")
    
    print("\nNode Memory Usage (final values):")
    for node in expected_nodes:
        memory_col = f'{node}_memory_mb'
        status_col = f'{node}_status'
        if memory_col in data[0]:
            final_memory = data[-1][memory_col]
            final_status = data[-1][status_col]
            if final_memory:
                try:
                    memory_gb = float(final_memory) / 1024
                    print(f"  {node}: {memory_gb:.1f} GB ({final_status})")
                except (ValueError, TypeError):
                    print(f"  {node}: N/A ({final_status})")
            else:
                print(f"  {node}: N/A ({final_status})")
    
    print("="*60)

def main():
    # Find the most recent CSV file in repo root (derived from this file)
    repo_root = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(repo_root)
    csv_files = [f for f in os.listdir(repo_root) if f.startswith('memory_usage_') and f.endswith('.csv')]
    
    if not csv_files:
        print(f"❌ No memory usage CSV files found in {repo_root}/")
        return 1
    
    # Use the most recent file
    csv_files.sort(reverse=True)
    csv_file = os.path.join(repo_root, csv_files[0])
    
    print(f"🔍 Found CSV file: {csv_file}")
    
    try:
        plot_memory_usage(csv_file)
        return 0
    except Exception as e:
        print(f"❌ Error creating plot: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(main())