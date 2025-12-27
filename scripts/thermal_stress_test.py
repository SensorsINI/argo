#!/usr/bin/env python3
"""
Thermal Stress Test Script
===========================
Measures CPU temperature at 1Hz while stressing CPU and DRAM.
- Starts CPU and memory stress test (all cores + 50% RAM)
- Stops stress when temperature reaches 95°C
- Continues monitoring until temperature returns to starting_level * 1.1
- Saves data to /tmp and plots results
"""

import time
import subprocess
import signal
import sys
import os
from datetime import datetime
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

# Thermal zone path for CPU temperature
CPU_THERMAL_ZONE = '/sys/class/thermal/thermal_zone2/temp'
SAMPLE_RATE = 1.0  # 1Hz
TARGET_TEMP = 95.0  # Stop stress at 95°C
COOLDOWN_MULTIPLIER = 1.1  # Continue until temp returns to start * 1.1

def read_cpu_temperature():
    """Read CPU temperature from thermal zone in Celsius"""
    try:
        with open(CPU_THERMAL_ZONE, 'r') as f:
            temp_millicelsius = int(f.read().strip())
        return temp_millicelsius / 1000.0
    except Exception as e:
        print(f"Error reading temperature: {e}", file=sys.stderr)
        return None

def start_cpu_memory_stress():
    """Start CPU and memory stress test using stress-ng
    
    Uses all CPU cores for CPU stress and memory stress (exercises DRAM).
    Memory stress uses 50% of available RAM to maximize thermal load.
    """
    # Use stress-ng to stress all CPU cores with CPU-bound workload
    # --cpu 0 means use all available CPU cores
    # --vm 0 means use all available CPU cores for memory stress (exercises DRAM)
    # --vm-bytes 50% uses 50% of available RAM for memory stress
    # --timeout 0 means run indefinitely until killed
    cmd = ['stress-ng', '--cpu', '0', '--vm', '0', '--vm-bytes', '50%', '--timeout', '0']
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )
        return process
    except FileNotFoundError:
        print("Error: stress-ng not found. Please install it with: sudo apt-get install stress-ng", file=sys.stderr)
        return None
    except Exception as e:
        print(f"Error starting stress test: {e}", file=sys.stderr)
        return None

def stop_cpu_memory_stress(process):
    """Stop CPU and memory stress test"""
    if process and process.poll() is None:
        try:
            # Kill the entire process group
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except Exception as e:
            print(f"Error stopping stress test: {e}", file=sys.stderr)

def main():
    """Main thermal stress test function"""
    print("=" * 60)
    print("Thermal Stress Test")
    print("=" * 60)
    
    # Get initial temperature
    initial_temp = read_cpu_temperature()
    if initial_temp is None:
        print("Failed to read initial temperature. Exiting.")
        sys.exit(1)
    
    print(f"Initial CPU temperature: {initial_temp:.1f}°C")
    print(f"Target temperature: {TARGET_TEMP}°C")
    print(f"Cooldown target: {initial_temp * COOLDOWN_MULTIPLIER:.1f}°C")
    print()
    
    # Data storage
    timestamps = []
    temperatures = []
    stress_active = []
    
    # Create output directory
    output_dir = '/tmp'
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_file = os.path.join(output_dir, f'thermal_stress_test_{timestamp_str}.csv')
    
    # Start time
    start_time = time.time()
    stress_process = None
    stress_started = False
    stress_stopped = False
    
    print("Starting thermal stress test...")
    print("Time (s)    Temp (°C)    Status")
    print("-" * 40)
    
    try:
        while True:
            elapsed_time = time.time() - start_time
            temp = read_cpu_temperature()
            
            if temp is None:
                print(f"{elapsed_time:8.1f}    ERROR      Failed to read temperature")
                time.sleep(SAMPLE_RATE)
                continue
            
            # Start stress if not started yet
            if not stress_started:
                stress_process = start_cpu_memory_stress()
                if stress_process is None:
                    print("Failed to start stress test. Exiting.")
                    sys.exit(1)
                stress_started = True
                print(f"{elapsed_time:8.1f}    {temp:6.1f}      STRESS STARTED (CPU+DRAM)")
            
            # Check if we should stop stress
            if not stress_stopped and temp >= TARGET_TEMP:
                stop_cpu_memory_stress(stress_process)
                stress_stopped = True
                print(f"{elapsed_time:8.1f}    {temp:6.1f}      STRESS STOPPED (reached {TARGET_TEMP}°C)")
            
            # Determine status string
            if not stress_started:
                status = "WAITING"
            elif stress_stopped:
                status = "COOLING"
            else:
                status = "STRESSING"
            
            # Store data
            timestamps.append(elapsed_time)
            temperatures.append(temp)
            stress_active.append(not stress_stopped)
            
            # Print current status
            print(f"{elapsed_time:8.1f}    {temp:6.1f}      {status}")
            
            # Check if we've cooled down enough
            if stress_stopped and temp <= initial_temp * COOLDOWN_MULTIPLIER:
                print(f"{elapsed_time:8.1f}    {temp:6.1f}      COOLDOWN COMPLETE")
                print()
                print(f"Test complete! Temperature returned to {temp:.1f}°C")
                break
            
            time.sleep(SAMPLE_RATE)
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        if stress_process:
            stop_cpu_memory_stress(stress_process)
    
    finally:
        # Ensure stress is stopped
        if stress_process and not stress_stopped:
            stop_cpu_memory_stress(stress_process)
    
    # Save data to CSV
    print()
    print("Saving data to CSV...")
    with open(csv_file, 'w') as f:
        f.write("time_s,temperature_c,stress_active\n")
        for t, temp, stress in zip(timestamps, temperatures, stress_active):
            f.write(f"{t:.2f},{temp:.2f},{int(stress)}\n")
    print(f"Data saved to: {csv_file}")
    
    # Create plot
    print("Creating plot...")
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Plot temperature
    ax.plot(timestamps, temperatures, 'b-', linewidth=2, label='CPU Temperature')
    
    # Mark stress start and stop points
    stress_start_idx = 0
    stress_stop_idx = len(stress_active) - 1
    
    if stress_started:
        for i, stress in enumerate(stress_active):
            if stress:
                stress_start_idx = i
                break
        
        for i in range(len(stress_active) - 1, -1, -1):
            if stress_active[i]:
                stress_stop_idx = i
                break
        
        if stress_stop_idx < len(timestamps):
            ax.axvline(x=timestamps[stress_start_idx], color='g', linestyle='--', 
                      linewidth=2, label='Stress Started', alpha=0.7)
            if stress_stopped:
                ax.axvline(x=timestamps[stress_stop_idx], color='r', linestyle='--', 
                          linewidth=2, label='Stress Stopped', alpha=0.7)
    
    # Mark target temperature
    ax.axhline(y=TARGET_TEMP, color='r', linestyle=':', linewidth=2, 
              label=f'Target ({TARGET_TEMP}°C)', alpha=0.7)
    
    # Mark initial and cooldown temperatures
    ax.axhline(y=initial_temp, color='g', linestyle=':', linewidth=1, 
              label=f'Initial ({initial_temp:.1f}°C)', alpha=0.5)
    ax.axhline(y=initial_temp * COOLDOWN_MULTIPLIER, color='orange', linestyle=':', 
              linewidth=1, label=f'Cooldown Target ({initial_temp * COOLDOWN_MULTIPLIER:.1f}°C)', alpha=0.5)
    
    # Fill stress region
    if stress_started:
        stress_times = [t for t, s in zip(timestamps, stress_active) if s]
        if stress_times:
            stress_start_time = min(stress_times)
            stress_stop_time = max(stress_times) if stress_stopped else timestamps[-1]
            ax.axvspan(stress_start_time, stress_stop_time, alpha=0.2, color='red', 
                      label='Stress Active')
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Temperature (°C)', fontsize=12)
    ax.set_title('Thermal Stress Test Results', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Add statistics text
    max_temp = max(temperatures)
    max_temp_time = timestamps[temperatures.index(max_temp)]
    
    if stress_started and stress_stopped:
        stress_duration = timestamps[stress_stop_idx] - timestamps[stress_start_idx]
        cooldown_duration = timestamps[-1] - timestamps[stress_stop_idx]
        stats_text = f"""Statistics:
Initial Temp: {initial_temp:.1f}°C
Max Temp: {max_temp:.1f}°C (at {max_temp_time:.1f}s)
Final Temp: {temperatures[-1]:.1f}°C
Test Duration: {timestamps[-1]:.1f}s
Stress Duration: {stress_duration:.1f}s
Cooldown Duration: {cooldown_duration:.1f}s"""
    else:
        stats_text = f"""Statistics:
Initial Temp: {initial_temp:.1f}°C
Max Temp: {max_temp:.1f}°C (at {max_temp_time:.1f}s)
Final Temp: {temperatures[-1]:.1f}°C
Test Duration: {timestamps[-1]:.1f}s"""
    
    fig.text(0.02, 0.02, stats_text, fontsize=9, verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    
    # Save plot
    plot_file = os.path.join(output_dir, f'thermal_stress_test_{timestamp_str}.png')
    fig.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {plot_file}")
    
    # Also save as PDF
    pdf_file = os.path.join(output_dir, f'thermal_stress_test_{timestamp_str}.pdf')
    fig.savefig(pdf_file, bbox_inches='tight')
    print(f"Plot also saved as PDF: {pdf_file}")
    
    print()
    print("=" * 60)
    print("Test Summary:")
    print(f"  Initial temperature: {initial_temp:.1f}°C")
    print(f"  Maximum temperature: {max_temp:.1f}°C")
    print(f"  Final temperature: {temperatures[-1]:.1f}°C")
    print(f"  Total duration: {timestamps[-1]:.1f} seconds")
    print(f"  Data file: {csv_file}")
    print(f"  Plot file: {plot_file}")
    print("=" * 60)

if __name__ == "__main__":
    main()
