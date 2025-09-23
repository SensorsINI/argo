#!/usr/bin/env python3
"""
Argo Status Desktop GUI
=======================

A lightweight desktop application for monitoring Argo sailboat system status.
Uses tkinter for the GUI and leverages existing Argo status infrastructure.

Features:
- Real-time system status monitoring
- Service status display
- ROS2 nodes monitoring
- System resource tracking
- Auto-refresh capabilities
- Color-coded status indicators

Usage: python3 argo_status_gui.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import os
import sys
from datetime import datetime
from typing import Dict, Any

# Add current directory to path to import argo modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from argo_status_check import OptimizedArgoStatusChecker
    from argo_node_utils import ArgoNodeManager
except ImportError as e:
    print(f"Error importing Argo modules: {e}")
    print("Make sure you're running this from the launch directory")
    sys.exit(1)


class ArgoStatusGUI:
    """Main GUI class for Argo status monitoring"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Argo Status Monitor")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')
        
        # Initialize status checker
        self.status_checker = OptimizedArgoStatusChecker()
        self.node_manager = ArgoNodeManager()
        
        # Status data
        self.last_update = None
        self.auto_refresh = True
        self.refresh_interval = 5  # seconds
        
        # Colors
        self.colors = {
            'bg': '#2b2b2b',
            'fg': '#ffffff',
            'success': '#4CAF50',
            'warning': '#FF9800',
            'error': '#F44336',
            'info': '#2196F3',
            'panel': '#3c3c3c',
            'border': '#555555'
        }
        
        self.setup_gui()
        self.start_auto_refresh()
        self.refresh_status()
    
    def setup_gui(self):
        """Setup the GUI layout"""
        # Configure style
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', 
                       background=self.colors['bg'], 
                       foreground=self.colors['fg'],
                       font=('Arial', 16, 'bold'))
        style.configure('Header.TLabel',
                       background=self.colors['panel'],
                       foreground=self.colors['fg'],
                       font=('Arial', 12, 'bold'))
        style.configure('Status.TLabel',
                       background=self.colors['panel'],
                       foreground=self.colors['fg'],
                       font=('Courier', 10))
        
        # Main title
        title_frame = tk.Frame(self.root, bg=self.colors['bg'])
        title_frame.pack(fill='x', padx=10, pady=5)
        
        title_label = ttk.Label(title_frame, text="🚢 ARGO STATUS MONITOR", style='Title.TLabel')
        title_label.pack(side='left')
        
        self.last_update_label = ttk.Label(title_frame, text="Last Update: Never", style='Title.TLabel')
        self.last_update_label.pack(side='right')
        
        # Control buttons
        control_frame = tk.Frame(self.root, bg=self.colors['bg'])
        control_frame.pack(fill='x', padx=10, pady=5)
        
        refresh_btn = tk.Button(control_frame, text="🔄 Refresh", 
                               command=self.refresh_status,
                               bg=self.colors['info'], fg='white',
                               font=('Arial', 10, 'bold'))
        refresh_btn.pack(side='left', padx=(0, 5))
        
        self.auto_refresh_var = tk.BooleanVar(value=self.auto_refresh)
        auto_refresh_cb = tk.Checkbutton(control_frame, text="Auto Refresh",
                                        variable=self.auto_refresh_var,
                                        command=self.toggle_auto_refresh,
                                        bg=self.colors['bg'], fg=self.colors['fg'],
                                        selectcolor=self.colors['panel'])
        auto_refresh_cb.pack(side='left', padx=5)
        
        # Main content area with scrollbar
        main_frame = tk.Frame(self.root, bg=self.colors['bg'])
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Canvas and scrollbar for scrolling
        self.canvas = tk.Canvas(main_frame, bg=self.colors['bg'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(main_frame, orient='vertical', command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas, bg=self.colors['bg'])
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=scrollbar.set)
        
        self.canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Bind mousewheel to canvas
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.root.bind_all("<Button-4>", self._on_mousewheel)
        self.root.bind_all("<Button-5>", self._on_mousewheel)
        
        # Status panels
        self.create_status_panels()
    
    def _on_mousewheel(self, event):
        """Handle mouse wheel scrolling"""
        if event.num == 4 or event.delta > 0:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5 or event.delta < 0:
            self.canvas.yview_scroll(1, "units")
    
    def create_status_panels(self):
        """Create the status display panels"""
        # System Services Panel
        self.services_panel = self.create_panel("📋 SYSTEMD SERVICES")
        
        # ROS Nodes Panel
        self.nodes_panel = self.create_panel("🤖 ROS NODES")
        
        # System Resources Panel
        self.system_panel = self.create_panel("💻 SYSTEM RESOURCES")
        
        # Summary Panel
        self.summary_panel = self.create_panel("📊 SUMMARY")
    
    def create_panel(self, title):
        """Create a status panel with title"""
        panel_frame = tk.Frame(self.scrollable_frame, 
                              bg=self.colors['panel'],
                              relief='raised',
                              borderwidth=1)
        panel_frame.pack(fill='x', padx=5, pady=5)
        
        # Panel header
        header_frame = tk.Frame(panel_frame, bg=self.colors['panel'])
        header_frame.pack(fill='x', padx=5, pady=5)
        
        header_label = ttk.Label(header_frame, text=title, style='Header.TLabel')
        header_label.pack(side='left')
        
        # Panel content
        content_frame = tk.Frame(panel_frame, bg=self.colors['panel'])
        content_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        return content_frame
    
    def clear_panel(self, panel):
        """Clear all widgets from a panel"""
        for widget in panel.winfo_children():
            widget.destroy()
    
    def add_status_line(self, panel, text, color=None):
        """Add a status line to a panel"""
        if color is None:
            color = self.colors['fg']
        
        label = tk.Label(panel, text=text, 
                        bg=self.colors['panel'], 
                        fg=color,
                        font=('Courier', 10),
                        anchor='w')
        label.pack(fill='x', pady=1)
        return label
    
    def get_status_color(self, status):
        """Get color for status text"""
        status_lower = status.lower()
        if 'active' in status_lower or 'running' in status_lower:
            return self.colors['success']
        elif 'inactive' in status_lower or 'not running' in status_lower:
            return self.colors['error']
        elif 'warning' in status_lower:
            return self.colors['warning']
        else:
            return self.colors['info']
    
    def refresh_status(self):
        """Refresh all status information"""
        try:
            # Update timestamp
            self.last_update = datetime.now()
            self.last_update_label.config(text=f"Last Update: {self.last_update.strftime('%H:%M:%S')}")
            
            # Get status data
            ros_info = self.status_checker.check_ros_nodes_fast()
            sys_info = self.status_checker.get_system_info_fast()
            
            # Update services panel
            self.update_services_panel()
            
            # Update ROS nodes panel
            self.update_nodes_panel(ros_info)
            
            # Update system resources panel
            self.update_system_panel(sys_info)
            
            # Update summary panel
            self.update_summary_panel(ros_info, sys_info)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to refresh status: {e}")
    
    def update_services_panel(self):
        """Update the services status panel"""
        self.clear_panel(self.services_panel)
        
        # Check argo-launch service
        launch_status = self.status_checker.get_service_status_fast("argo-launch.service")
        color = self.get_status_color(launch_status)
        
        if launch_status == "active":
            launch_pid = self.status_checker.get_service_pid("argo-launch.service")
            if launch_pid:
                stats = self.status_checker.get_process_stats(launch_pid)
                if stats:
                    text = f"  argo-launch: PID:{stats['pid']} CPU:{stats['cpu']:.1f}% MEM:{stats['mem']:.1f}%"
                else:
                    text = f"  argo-launch: ACTIVE (no stats available)"
            else:
                text = f"  argo-launch: ACTIVE (no main PID)"
        else:
            text = f"  argo-launch: {launch_status.upper()}"
        
        self.add_status_line(self.services_panel, text, color)
        
        # Recording service (ROS2-based)
        self.add_status_line(self.services_panel, "  argo-record: ros2-service", self.colors['info'])
    
    def update_nodes_panel(self, ros_info):
        """Update the ROS nodes panel"""
        self.clear_panel(self.nodes_panel)
        
        for node in self.node_manager.discover_nodes():
            node_info = ros_info['node_status'][node]
            
            if node_info['running']:
                for proc in node_info['processes']:
                    cpu_color = self.colors['success'] if proc['cpu'] < 20 else self.colors['warning'] if proc['cpu'] < 50 else self.colors['error']
                    mem_color = self.colors['success'] if proc['mem'] < 5 else self.colors['warning'] if proc['mem'] < 10 else self.colors['error']
                    
                    text = f"  {node}: PID:{proc['pid']}"
                    self.add_status_line(self.nodes_panel, text, self.colors['success'])
                    
                    # Add resource usage on separate line
                    resource_text = f"    CPU:{proc['cpu']:.1f}% MEM:{proc['mem']:.1f}%"
                    resource_label = tk.Label(self.nodes_panel, text=resource_text,
                                            bg=self.colors['panel'],
                                            font=('Courier', 9),
                                            anchor='w')
                    resource_label.pack(fill='x', pady=1)
                    
                    # Color the CPU and MEM parts differently
                    resource_label.config(fg=self.colors['fg'])
            else:
                self.add_status_line(self.nodes_panel, f"  {node}: NOT RUNNING", self.colors['error'])
    
    def update_system_panel(self, sys_info):
        """Update the system resources panel"""
        self.clear_panel(self.system_panel)
        
        # System load
        load_color = self.colors['success']
        try:
            load_val = float(sys_info['load_avg'])
            if load_val > 2.0:
                load_color = self.colors['error']
            elif load_val > 1.0:
                load_color = self.colors['warning']
        except:
            pass
        
        self.add_status_line(self.system_panel, f"  System Load: {sys_info['load_avg']}", load_color)
        
        # Memory usage
        mem_color = self.colors['success']
        try:
            mem_val = float(sys_info['mem_usage'])
            if mem_val > 80:
                mem_color = self.colors['error']
            elif mem_val > 60:
                mem_color = self.colors['warning']
        except:
            pass
        
        self.add_status_line(self.system_panel, f"  Memory Usage: {sys_info['mem_usage']}%", mem_color)
        
        # Storage
        storage_color = self.colors['success']
        try:
            used_val = float(sys_info['used_percent'])
            if used_val > 90:
                storage_color = self.colors['error']
            elif used_val > 75:
                storage_color = self.colors['warning']
        except:
            pass
        
        self.add_status_line(self.system_panel, f"  Storage: {sys_info['free_gb']}GB free ({sys_info['used_percent']}% used)", storage_color)
    
    def update_summary_panel(self, ros_info, sys_info):
        """Update the summary panel"""
        self.clear_panel(self.summary_panel)
        
        # Nodes summary
        nodes_color = self.colors['success'] if ros_info['running_nodes'] == ros_info['total_nodes'] else self.colors['warning']
        self.add_status_line(self.summary_panel, 
                           f"  Running Nodes: {ros_info['running_nodes']}/{ros_info['total_nodes']}", 
                           nodes_color)
        
        # Resource summary
        cpu_color = self.colors['success'] if ros_info['total_cpu'] < 50 else self.colors['warning']
        mem_color = self.colors['success'] if ros_info['total_mem'] < 20 else self.colors['warning']
        
        self.add_status_line(self.summary_panel, f"  Total CPU Usage: {ros_info['total_cpu']:.1f}%", cpu_color)
        self.add_status_line(self.summary_panel, f"  Total Memory Usage: {ros_info['total_mem']:.1f}%", mem_color)
        
        # Overall status
        if ros_info['running_nodes'] == ros_info['total_nodes']:
            overall_status = "🟢 ALL SYSTEMS OPERATIONAL"
            overall_color = self.colors['success']
        elif ros_info['running_nodes'] > 0:
            overall_status = "🟡 PARTIAL OPERATION"
            overall_color = self.colors['warning']
        else:
            overall_status = "🔴 SYSTEM DOWN"
            overall_color = self.colors['error']
        
        self.add_status_line(self.summary_panel, f"  Status: {overall_status}", overall_color)
    
    def toggle_auto_refresh(self):
        """Toggle auto refresh on/off"""
        self.auto_refresh = self.auto_refresh_var.get()
    
    def start_auto_refresh(self):
        """Start the auto refresh thread"""
        def refresh_loop():
            while True:
                if self.auto_refresh:
                    try:
                        self.root.after(0, self.refresh_status)
                    except:
                        break  # GUI was closed
                time.sleep(self.refresh_interval)
        
        refresh_thread = threading.Thread(target=refresh_loop, daemon=True)
        refresh_thread.start()
    
    def run(self):
        """Start the GUI main loop"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            print("\n🛑 GUI closed by user")


def main():
    """Main entry point"""
    try:
        app = ArgoStatusGUI()
        app.run()
    except Exception as e:
        print(f"Error starting Argo Status GUI: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
