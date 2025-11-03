#!/usr/bin/env python3
"""
Argo Node Utilities
==================

Centralized utilities for detecting and managing Argo ROS2 nodes.
Automatically discovers nodes from the nodes/ folder and provides
consistent node detection across all Argo scripts.

Features:
- Auto-discovery of nodes from nodes/ folder
- Consistent node naming and detection
- Shared utilities for process monitoring
- Support for special nodes like foxglove_bridge
"""

import os
import sys
import subprocess
import signal
import psutil
from typing import List, Dict, Optional, Tuple

# Handle KeyboardInterrupt gracefully
def signal_handler(signum, frame):
    print("\n🛑 Interrupted by user")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
from pathlib import Path


class ArgoNodeManager:
    """Centralized manager for Argo ROS2 nodes"""
    
    def __init__(self, argo_root: Optional[str] = None):
        """
        Initialize the node manager
        
        Args:
            argo_root: Path to argo root directory. If None, auto-detect from script location.
        """
        if argo_root is None:
            # Auto-detect argo root from script location
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.argo_root = os.path.dirname(script_dir)
        else:
            self.argo_root = argo_root
            
        self.nodes_dir = os.path.join(self.argo_root, 'nodes')
        self._discovered_nodes = None
        self._special_nodes = ['foxglove_bridge']  # Nodes not in nodes/ folder
        self._simulation_only_nodes = ['argo_unified_simulator_bridge']  # Nodes only for simulation mode
        self._cached_processes = None  # Cache for get_all_ros_processes results
    
    def discover_nodes(self, exclude_simulation_only: bool = False) -> List[str]:
        """
        Discover all available nodes from the nodes/ folder
        
        Args:
            exclude_simulation_only: If True, exclude nodes that are only for simulation mode
        
        Returns:
            List of node names (without .py extension)
        """
        if self._discovered_nodes is not None and not exclude_simulation_only:
            return self._discovered_nodes
            
        nodes = []
        
        # Add nodes in nodes/ folder, but make sure they are really ros nodes 
        # by checking if they import rclpy and subclass Node
        if os.path.exists(self.nodes_dir):
            for file_path in os.listdir(self.nodes_dir):
                if file_path.endswith('.py') and not file_path.startswith('__'):
                    # Check if this is a real ROS node
                    if self._is_ros_node(os.path.join(self.nodes_dir, file_path)):
                        # Remove .py extension
                        node_name = file_path[:-3]
                        
                        # Skip simulation-only nodes if requested
                        if exclude_simulation_only and node_name in self._simulation_only_nodes:
                            continue
                            
                        nodes.append(node_name)
        
        # Add special nodes that aren't in the nodes/ folder
        nodes.extend(self._special_nodes)
        
        # Sort for consistent ordering
        nodes.sort()
        
        # Only cache if not excluding simulation nodes
        if not exclude_simulation_only:
            self._discovered_nodes = nodes
        return nodes
    
    def _is_ros_node(self, file_path: str) -> bool:
        """
        Check if a Python file is a real ROS node by examining its contents
        
        Args:
            file_path: Path to the Python file to check
            
        Returns:
            True if the file appears to be a ROS node, False otherwise
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Check for rclpy import
            if 'import rclpy' not in content and 'from rclpy' not in content:
                return False
            
            # Check for Node class usage (either import or subclass)
            if 'Node' not in content:
                return False
            
            # Check for common ROS node patterns
            ros_patterns = [
                'rclpy.init()',
                'rclpy.spin',
                'create_node',
                'create_publisher',
                'create_subscription',
                'create_service',
                'create_client',
                'Node(',
                'class.*Node'
            ]
            
            # At least one ROS pattern should be present
            has_ros_pattern = any(pattern in content for pattern in ros_patterns)
            
            return has_ros_pattern
            
        except Exception as e:
            # If we can't read the file, assume it's not a valid ROS node
            print(f"⚠️  Warning: Could not validate {file_path}: {e}")
            return False
    
    def get_all_ros_processes(self) -> Dict[str, List[Dict]]:
        """
        Get all running ROS processes for discovered nodes
        
        Returns:
            Dictionary mapping node names to lists of process info
        """
        # Return cached results if available (nodes don't change during runtime)
        if self._cached_processes is not None:
            return self._cached_processes
        
        nodes = self.discover_nodes()
        
        # Build grep pattern for all nodes
        node_patterns = []
        for node in nodes:
            if node == 'foxglove_bridge':
                # Special case for foxglove_bridge
                node_patterns.append('foxglove_bridge')
            else:
                # Regular nodes in nodes/ folder
                node_patterns.append(f'{node}\\.py')
        
        # Create grep command
        grep_pattern = '|'.join(node_patterns)
        cmd = f"ps aux | grep -E '({grep_pattern})' | grep -v grep"
        
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
            if result.returncode != 0 or not result.stdout.strip():
                return {}
        except (subprocess.TimeoutExpired, Exception):
            return {}
        
        processes = {}
        
        for line in result.stdout.strip().split('\n'):
            if not line.strip():
                continue
                
            parts = line.split()
            if len(parts) < 11:
                continue
                
            pid = int(parts[1])
            cpu = float(parts[2].replace('%', ''))
            mem = float(parts[3].replace('%', ''))
            cmd_line = ' '.join(parts[10:])
            
            # Determine which node this process belongs to
            matched_node = None
            for node in nodes:
                if node == 'foxglove_bridge':
                    if 'foxglove_bridge' in cmd_line:
                        matched_node = node
                        break
                else:
                    if f'/{node}.py' in cmd_line or f' {node}.py' in cmd_line:
                        matched_node = node
                        break
            
            if matched_node:
                if matched_node not in processes:
                    processes[matched_node] = []
                
                processes[matched_node].append({
                    'pid': pid,
                    'cpu': cpu,
                    'mem': mem,
                    'cmd': cmd_line
                })
        
        # Cache the result for future calls
        self._cached_processes = processes
        return processes
    
    def clear_process_cache(self):
        """
        Clear the cached process results to force a fresh lookup on next call.
        Useful when nodes are started/stopped or during testing.
        """
        self._cached_processes = None
    
    def get_node_status(self) -> Dict[str, Dict]:
        """
        Get detailed status for all discovered nodes
        
        Returns:
            Dictionary mapping node names to status info
        """
        nodes = self.discover_nodes()
        processes = self.get_all_ros_processes()
        
        status = {}
        for node in nodes:
            if node in processes and processes[node]:
                # Node is running
                node_processes = processes[node]
                total_cpu = sum(p['cpu'] for p in node_processes)
                total_mem = sum(p['mem'] for p in node_processes)
                pids = [p['pid'] for p in node_processes]
                
                status[node] = {
                    'running': True,
                    'processes': node_processes,
                    'total_cpu': total_cpu,
                    'total_mem': total_mem,
                    'pids': pids,
                    'count': len(node_processes)
                }
            else:
                # Node is not running
                status[node] = {
                    'running': False,
                    'processes': [],
                    'total_cpu': 0.0,
                    'total_mem': 0.0,
                    'pids': [],
                    'count': 0
                }
        
        return status
    
    def get_running_nodes(self) -> List[str]:
        """Get list of currently running node names"""
        status = self.get_node_status()
        return [node for node, info in status.items() if info['running']]
    
    def get_stopped_nodes(self) -> List[str]:
        """Get list of currently stopped node names"""
        status = self.get_node_status()
        return [node for node, info in status.items() if not info['running']]
    
    def get_node_summary(self) -> Dict[str, any]:
        """
        Get summary statistics for all nodes
        
        Returns:
            Dictionary with summary information
        """
        status = self.get_node_status()
        running_nodes = self.get_running_nodes()
        stopped_nodes = self.get_stopped_nodes()
        
        total_cpu = sum(info['total_cpu'] for info in status.values())
        total_mem = sum(info['total_mem'] for info in status.values())
        
        return {
            'total_nodes': len(status),
            'running_nodes': len(running_nodes),
            'stopped_nodes': len(stopped_nodes),
            'total_cpu': total_cpu,
            'total_mem': total_mem,
            'running_node_list': running_nodes,
            'stopped_node_list': stopped_nodes,
            'node_status': status
        }
    
    def format_node_status(self, show_pids: bool = True, show_stats: bool = True) -> str:
        """
        Format node status for display
        
        Args:
            show_pids: Whether to show process PIDs
            show_stats: Whether to show CPU/memory stats
            
        Returns:
            Formatted string showing node status
        """
        status = self.get_node_status()
        lines = []
        
        for node, info in status.items():
            if info['running']:
                if show_pids and show_stats:
                    # Show detailed info for each process
                    for proc in info['processes']:
                        lines.append(f"  {node}: PID:{proc['pid']} CPU:{proc['cpu']:.1f}% MEM:{proc['mem']:.1f}%")
                elif show_pids:
                    # Show just PIDs
                    pids = ','.join(map(str, info['pids']))
                    lines.append(f"  {node}: PID:{pids}")
                else:
                    # Show just running status
                    lines.append(f"  {node}: RUNNING")
            else:
                lines.append(f"  {node}: NOT RUNNING")
        
        return '\n'.join(lines)


# Convenience functions for backward compatibility
def get_argo_nodes(argo_root: Optional[str] = None, exclude_simulation_only: bool = False) -> List[str]:
    """Get list of all Argo nodes"""
    manager = ArgoNodeManager(argo_root)
    return manager.discover_nodes(exclude_simulation_only=exclude_simulation_only)


def get_argo_node_status(argo_root: Optional[str] = None) -> Dict[str, Dict]:
    """Get status of all Argo nodes"""
    manager = ArgoNodeManager(argo_root)
    return manager.get_node_status()


def get_argo_running_nodes(argo_root: Optional[str] = None) -> List[str]:
    """Get list of currently running Argo nodes"""
    manager = ArgoNodeManager(argo_root)
    return manager.get_running_nodes()


# Example usage and testing
if __name__ == '__main__':
    print("Argo Node Manager - Discovery Test")
    print("=" * 40)
    
    manager = ArgoNodeManager()
    
    print(f"Argo root: {manager.argo_root}")
    print(f"Nodes directory: {manager.nodes_dir}")
    print()
    
    print("Discovered nodes:")
    nodes = manager.discover_nodes()
    for node in nodes:
        print(f"  - {node}")
    print()
    
    print("Node status:")
    summary = manager.get_node_summary()
    print(f"  Total nodes: {summary['total_nodes']}")
    print(f"  Running: {summary['running_nodes']}")
    print(f"  Stopped: {summary['stopped_nodes']}")
    print(f"  Total CPU: {summary['total_cpu']:.1f}%")
    print(f"  Total MEM: {summary['total_mem']:.1f}%")
    print()
    
    print("Detailed status:")
    print(manager.format_node_status())
