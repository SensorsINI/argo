#!/usr/bin/env python3
"""
Argo Node Termination Test
==========================

A utility script to test if individual Argo ROS2 nodes shut down gracefully
when receiving a SIGTERM signal. This helps identify nodes that may cause
shutdown delays.

The script will:
1. Discover all available ROS2 nodes.
2. Sequentially launch each node.
3. Wait for a few seconds to allow initialization.
4. Send a SIGTERM signal.
5. Wait for the process to terminate.
6. Report success or failure for each node.

USAGE:
  cd argo/
  python3 scripts/test_node_termination.py
"""

import os
import sys
import subprocess
import time

print("--- Starting Node Termination Test Script ---")

# Add argo_node_utils to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'launch'))

try:
    from argo_node_utils import ArgoNodeManager
except ImportError:
    print("❌ Could not import ArgoNodeManager. Make sure you are running this script from the 'argo/' directory.")
    sys.exit(1)

# Configuration
NODE_INIT_WAIT_S = 5  # Time to wait for node to initialize
NODE_TERM_WAIT_S = 5  # Time to wait for node to terminate gracefully

def run_test():
    """Discover and test all nodes for graceful termination."""
    argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    nodes_dir = os.path.join(argo_dir, 'nodes')
    
    print("- Disovering Argo ROS2 nodes...")
    node_manager = ArgoNodeManager()
    # Exclude simulation-only nodes for this test
    discovered_nodes = node_manager.discover_nodes(exclude_simulation_only=True)
    
    if not discovered_nodes:
        print("❌ No nodes found. Make sure the 'nodes/' directory is populated.")
        return

    print(f"✅ Found {len(discovered_nodes)} nodes to test: {', '.join(discovered_nodes)}")
    
    passed_nodes = []
    failed_nodes = []

    for node_name in discovered_nodes:
        script_name = f"{node_name}.py"
        script_path = os.path.join(nodes_dir, script_name)
        
        if not os.path.exists(script_path):
            print(f"\n- Skipping {node_name} (script not found at {script_path})")
            continue

        print(f"\n- Testing node: {script_name}")
        
        log_path = f"/tmp/node_test_{node_name}.log"
        print(f"  📝 Logging output to {log_path}")
        log_file = open(log_path, 'w')

        try:
            # Launch the node
            cmd = ['bash', '-c', f'source /opt/ros/humble/setup.bash && python3 {script_path}']
            proc = subprocess.Popen(cmd, cwd=argo_dir, stdout=log_file, stderr=subprocess.STDOUT)
            
            print(f"  🚀 Launched {script_name} with PID {proc.pid}. Waiting {NODE_INIT_WAIT_S}s for initialization...")
            time.sleep(NODE_INIT_WAIT_S)
            
            # Check if it's still running
            if proc.poll() is not None:
                print(f"  ❌ {script_name} terminated prematurely with code {proc.poll()}.")
                failed_nodes.append(script_name)
                continue
                
            # Send SIGTERM
            print(f"  🛑 Sending SIGTERM to {script_name} (PID: {proc.pid})...")
            proc.terminate()
            
            # Wait for termination
            try:
                proc.wait(timeout=NODE_TERM_WAIT_S)
                print(f"  ✅ {script_name} terminated gracefully (exit code {proc.returncode}).")
                passed_nodes.append(script_name)
            except subprocess.TimeoutExpired:
                print(f"  ❌ {script_name} did not terminate within {NODE_TERM_WAIT_S}s.")
                print(f"  ⚡ Force killing {script_name} (PID: {proc.pid})...")
                proc.kill()
                proc.wait()
                failed_nodes.append(script_name)
        finally:
            log_file.close()
            
    # --- Summary ---
    print("\n\n--- TEST SUMMARY ---")
    if passed_nodes:
        print(f"✅ PASSED: {len(passed_nodes)} nodes terminated gracefully")
        for node in passed_nodes:
            print(f"  - {node}")
    
    if failed_nodes:
        print(f"❌ FAILED: {len(failed_nodes)} nodes did not terminate gracefully")
        for node in failed_nodes:
            print(f"  - {node}")
    else:
        print("\n🎉 All tested nodes terminated gracefully!")

if __name__ == '__main__':
    run_test()
