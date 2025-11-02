#!/usr/bin/env python3
"""
Argo Keyboard Control Node
==========================

Standalone ROS2 node for keyboard control of Argo simulator.
Publishes control commands to /rudder_sail_radio topic for simulator control.

This node can be run in a separate terminal while simulation runs via 'asim'.

Features:
- Arrow key control (←→ rudder, ↑↓ sail)
- Visual display of current control positions
- Real-time status updates
- Clean shutdown with terminal restoration

Usage:
    python3 nodes/argo_keyboard_control.py

Keyboard Controls:
    ←  : Rudder left (decrease)
    →  : Rudder right (increase)
    ↑  : Sail out (increase)
    ↓  : Sail in (decrease)
    c  : Center both controls
    q  : Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import curses
import time
import queue
import signal
import sys

class KeyboardControlNode(Node):
    """ROS2 node for keyboard control of Argo simulator."""
    
    def __init__(self):
        super().__init__('argo_keyboard_control')
        
        # Publisher for control commands
        self.pub_rudder_sail_radio = self.create_publisher(Vector3, '/rudder_sail_radio', 10)
        
        # Control state
        self.rudder_position = 0.0  # -1.0 to +1.0
        self.sail_position = 0.0    # -1.0 to +1.0
        self.step_size = 1.0 / 8.0  # 8 steps for full scale (0.125)
        self.running = True
        
        # Status from simulator (for display)
        self.simulator_heading = 0.0
        self.simulator_speed = 0.0
        self.simulator_mode = "UNKNOWN"
        
        # Subscribe to status topics for display
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Vector3, '/gps_velocity', self.velocity_callback, 10)
        from std_msgs.msg import Bool
        self.create_subscription(Bool, '/human_controlled', self.control_mode_callback, 10)
        
        # Setup curses
        self.stdscr = curses.initscr()
        self._setup_curses()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGWINCH, self._signal_handler)
        
        # Control loop timer (publishes current control state)
        self.control_timer = self.create_timer(0.1, self.publish_control)
        
        # Display update timer
        self.display_timer = self.create_timer(0.2, self.update_display)
        
        # Keyboard input timer
        self.keyboard_timer = self.create_timer(0.05, self.handle_keyboard_input)
        
        self.get_logger().info('Keyboard control node ready')
        self.get_logger().info('Controls: ←→ Rudder | ↑↓ Sail | C=Center | Q=Quit')
    
    def _setup_curses(self):
        """Setup curses for terminal control."""
        try:
            curses.curs_set(0)  # Hide cursor
            curses.noecho()      # Don't echo keystrokes
            curses.cbreak()      # Immediate key input
            self.stdscr.keypad(True)  # Enable keypad
            self.stdscr.nodelay(True)  # Non-blocking input
            
            # Initialize colors if supported
            if curses.has_colors():
                curses.start_color()
                curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
                curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
                curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)
                curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)
                self.colors_available = True
            else:
                self.colors_available = False
        except curses.error:
            self.colors_available = False
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        if signum == signal.SIGWINCH:
            # Terminal resize - handled in update_display
            pass
        else:
            self.running = False
            self.cleanup()
            if rclpy.ok():
                rclpy.shutdown()
    
    def cleanup(self):
        """Restore terminal to normal state."""
        try:
            if hasattr(self, 'stdscr') and self.stdscr:
                self.stdscr.keypad(False)
                self.stdscr.nodelay(False)
            
            curses.nocbreak()
            curses.echo()
            curses.curs_set(1)
            curses.endwin()
        except:
            pass
    
    def pose_callback(self, msg):
        """Receive heading/pose from simulator."""
        self.simulator_heading = msg.z  # Heading in degrees
    
    def velocity_callback(self, msg):
        """Receive velocity from simulator."""
        self.simulator_speed = msg.z  # Speed in knots
    
    def control_mode_callback(self, msg):
        """Receive control mode status."""
        self.simulator_mode = "HUMAN" if msg.data else "ROBOT"
    
    def handle_keyboard_input(self):
        """Process keyboard input."""
        if not self.running:
            return
        
        try:
            key = self.stdscr.getch()
            if key == -1:
                return  # No key pressed
            
            # Handle arrow keys
            if key == curses.KEY_LEFT:
                self.adjust_rudder(-1)  # Rudder left
            elif key == curses.KEY_RIGHT:
                self.adjust_rudder(1)   # Rudder right
            elif key == curses.KEY_UP:
                self.adjust_sail(1)      # Sail out
            elif key == curses.KEY_DOWN:
                self.adjust_sail(-1)     # Sail in
            elif key == ord('c') or key == ord('C'):
                self.center_controls()
            elif key == ord('q') or key == ord('Q'):
                self.running = False
                self.cleanup()
                if rclpy.ok():
                    rclpy.shutdown()
            elif key == 3:  # Ctrl+C
                self.running = False
                self.cleanup()
                if rclpy.ok():
                    rclpy.shutdown()
        except:
            pass
    
    def adjust_rudder(self, direction):
        """Adjust rudder position by one step."""
        self.rudder_position += direction * self.step_size
        self.rudder_position = max(-1.0, min(1.0, self.rudder_position))
    
    def adjust_sail(self, direction):
        """Adjust sail position by one step."""
        self.sail_position += direction * self.step_size
        self.sail_position = max(-1.0, min(1.0, self.sail_position))
    
    def center_controls(self):
        """Center both rudder and sail."""
        self.rudder_position = 0.0
        self.sail_position = 0.0
    
    def publish_control(self):
        """Publish current control commands to /rudder_sail_radio."""
        if not self.running:
            return
        
        control_msg = Vector3()
        control_msg.x = self.rudder_position  # Rudder: -1=left, +1=right
        control_msg.y = self.sail_position    # Sail: -1=in, +1=out
        control_msg.z = 0.0                   # Reserved
        
        self.pub_rudder_sail_radio.publish(control_msg)
    
    def create_control_bar(self, value, width=20):
        """Create ASCII bar visualization for control position."""
        normalized = (value + 1.0) / 2.0  # -1..+1 -> 0..1
        filled_length = int(normalized * width)
        bar = "[" + "█" * filled_length + "░" * (width - filled_length) + "]"
        
        if value < -0.1:
            direction = "← LEFT"
        elif value > 0.1:
            direction = "RIGHT →"
        else:
            direction = "CENTER"
        
        return f"{bar} {direction}"
    
    def update_display(self):
        """Update the curses display."""
        if not self.running:
            return
        
        try:
            self.stdscr.clear()
            self.stdscr.border()
            
            height, width = self.stdscr.getmaxyx()
            
            # Title
            title = "🚢 ARGO KEYBOARD CONTROL"
            self.stdscr.addstr(1, (width - len(title)) // 2, title)
            
            # Empty line
            self.stdscr.addstr(2, 1, " " * (width - 2))
            
            # Rudder visualization
            rudder_bar = self.create_control_bar(self.rudder_position, 16)
            self.stdscr.addstr(3, 2, f"Rudder: {rudder_bar} ({self.rudder_position:+.3f})")
            
            # Sail visualization
            sail_bar = self.create_control_bar(self.sail_position, 16)
            self.stdscr.addstr(4, 2, f"Sail:   {sail_bar} ({self.sail_position:+.3f})")
            
            # Empty line
            self.stdscr.addstr(5, 1, " " * (width - 2))
            
            # Status info
            status_line = f"Mode: {self.simulator_mode} | Heading: {self.simulator_heading:.1f}° | Speed: {self.simulator_speed:.1f}kt"
            if len(status_line) > width - 4:
                status_line = status_line[:width - 7] + "..."
            self.stdscr.addstr(6, 2, status_line)
            
            # Controls
            controls_line = "Controls: ←→ Rudder | ↑↓ Sail | C=Center | Q=Quit"
            if len(controls_line) > width - 4:
                controls_line = controls_line[:width - 7] + "..."
            self.stdscr.addstr(7, 2, controls_line)
            
            # Topic info
            topic_line = f"Publishing to: /rudder_sail_radio"
            self.stdscr.addstr(8, 2, topic_line)
            
            self.stdscr.refresh()
        except:
            pass
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

