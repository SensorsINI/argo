#!/usr/bin/env python3
"""
Test script for patrol controller - runs 60s simulation and plots trajectory.

Usage:
    python3 scripts/test_patrol_controller.py
"""

import subprocess
import time
import sys
import os
import json
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from collections import deque

# Add nodes directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nodes', 'support'))
from geofence_manager import GeofenceManager


class DataRecorder(Node):
    """Record GPS position and heading data during simulation."""
    
    def __init__(self):
        super().__init__('patrol_test_recorder')
        self.gps_data = deque()  # (timestamp, lat, lon)
        self.heading_data = deque()  # (timestamp, heading_math)
        self.control_data = deque()  # (timestamp, human_controlled, auto_rudder, auto_sail)
        self.start_time = None
        
        # Subscribers
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Bool, '/human_controlled', self.control_callback, 10)
        self.create_subscription(Vector3, '/rudder_sail_cmd', self.auto_cmd_callback, 10)
        
        self.get_logger().info('Data recorder started - waiting for data...')
        self.human_controlled_samples = 0
        self.robot_controlled_samples = 0
        self.auto_cmd_count = 0
    
    def gps_callback(self, msg):
        """Record GPS position."""
        if self.start_time is None:
            self.start_time = time.time()
        
        if not (math.isnan(msg.latitude) or math.isnan(msg.longitude)):
            timestamp = time.time() - self.start_time if self.start_time else 0.0
            self.gps_data.append((timestamp, msg.latitude, msg.longitude))
    
    def pose_callback(self, msg):
        """Record heading (mathematical convention from /pose)."""
        if self.start_time is None:
            self.start_time = time.time()
        
        timestamp = time.time() - self.start_time if self.start_time else 0.0
        heading_math = msg.z % 360.0  # Mathematical convention (0° = East)
        self.heading_data.append((timestamp, heading_math))
    
    def control_callback(self, msg):
        """Record control authority status."""
        if self.start_time is None:
            self.start_time = time.time()
        
        timestamp = time.time() - self.start_time if self.start_time else 0.0
        human_controlled = msg.data
        if human_controlled:
            self.human_controlled_samples += 1
            # Log first few human control samples
            if self.human_controlled_samples <= 3:
                self.get_logger().warn(f"Human control detected at t={timestamp:.1f}s (sample #{self.human_controlled_samples})")
        else:
            self.robot_controlled_samples += 1
            # Log first few robot control samples
            if self.robot_controlled_samples <= 3:
                self.get_logger().info(f"Robot control active at t={timestamp:.1f}s (sample #{self.robot_controlled_samples})")
        self.control_data.append((timestamp, human_controlled))
    
    def auto_cmd_callback(self, msg):
        """Record autonomous commands from controller."""
        self.auto_cmd_count += 1
        # Log first few commands to verify they're non-zero
        if self.auto_cmd_count <= 10:
            self.get_logger().info(f"Auto command #{self.auto_cmd_count}: rudder={msg.x:.3f}, sail={msg.y:.3f}")
        # Log every 20th command to track ongoing activity
        elif self.auto_cmd_count % 20 == 0:
            self.get_logger().info(f"Auto command #{self.auto_cmd_count}: rudder={msg.x:.3f}, sail={msg.y:.3f}")
    
    def get_data(self):
        """Return recorded data as arrays."""
        if not self.gps_data:
            return None, None, None, None
        
        # Convert to arrays
        timestamps = np.array([d[0] for d in self.gps_data])
        lats = np.array([d[1] for d in self.gps_data])
        lons = np.array([d[2] for d in self.gps_data])
        
        # Get headings (interpolate to match GPS timestamps)
        headings = []
        if self.heading_data:
            heading_times = np.array([d[0] for d in self.heading_data])
            heading_values = np.array([d[1] for d in self.heading_data])
            
            for t in timestamps:
                # Find closest heading
                idx = np.argmin(np.abs(heading_times - t))
                headings.append(heading_values[idx])
        else:
            headings = [0.0] * len(timestamps)
        
        headings = np.array(headings)
        
        return timestamps, lats, lons, headings


def load_geofence(map_name="Argo Irchel pond sailing area"):
    """Load geofence polygon from GeoJSON file."""
    script_path = Path(__file__).resolve()
    argo_dir = script_path.parents[1]  # scripts -> argo
    geojson_path = argo_dir / "foxglove" / "maps" / f"{map_name}.geojson"
    
    if not geojson_path.exists():
        print(f"ERROR: GeoJSON file not found: {geojson_path}")
        return None, None
    
    with open(geojson_path, 'r') as f:
        geojson_data = json.load(f)
    
    # Find home waypoint
    home_lat = None
    home_lon = None
    for feature in geojson_data.get('features', []):
        props = feature.get('properties', {})
        if props.get('name') == 'home' and props.get('type') == 'waypoint':
            coords = feature['geometry']['coordinates']
            home_lon = coords[0]
            home_lat = coords[1]
            break
    
    # Find sailing area polygon
    boundary_coords = None
    for feature in geojson_data.get('features', []):
        props = feature.get('properties', {})
        geom = feature.get('geometry', {})
        if geom.get('type') == 'Polygon' and props.get('type') == 'sailing_area':
            coords = geom.get('coordinates', [])
            if coords:
                # Get outer ring (first ring)
                boundary_coords = coords[0]
                break
    
    return (home_lat, home_lon), boundary_coords


def lonlat_to_xy(lon, lat, base_lon, base_lat):
    """Convert longitude/latitude to local x/y meters from base location."""
    R = 6378137.0  # Earth radius in meters
    x = (math.radians(lon) - math.radians(base_lon)) * R * math.cos(math.radians(base_lat))
    y = (math.radians(lat) - math.radians(base_lat)) * R
    return x, y


def plot_trajectory(timestamps, lats, lons, headings, home_pos, boundary_coords, output_file="patrol_test_result.png"):
    """Plot trajectory with arrow sticks and geofence."""
    if len(timestamps) == 0:
        print("ERROR: No data to plot")
        return
    
    # Convert to local coordinates
    base_lat, base_lon = home_pos
    
    # Convert boundary to local XY
    boundary_xy = []
    if boundary_coords:
        for coord in boundary_coords:
            lon, lat = coord[0], coord[1]
            x, y = lonlat_to_xy(lon, lat, base_lon, base_lat)
            boundary_xy.append([x, y])
    
    # Convert trajectory to local XY
    trajectory_xy = []
    for lat, lon in zip(lats, lons):
        x, y = lonlat_to_xy(lon, lat, base_lon, base_lat)
        trajectory_xy.append([x, y])
    trajectory_xy = np.array(trajectory_xy)
    
    # Convert headings from mathematical convention (0°=East) to compass convention (0°=North)
    # Mathematical: 0°=East, 90°=North, 180°=West, 270°=South
    # Compass: 0°=North, 90°=East, 180°=South, 270°=West
    # Conversion: compass = (90 - math) % 360
    headings_compass = (90.0 - headings) % 360.0
    
    # Convert compass heading to radians (for arrow direction)
    # In local XY: +X = East, +Y = North
    # Compass 0° = North = +Y direction
    # Compass 90° = East = +X direction
    # So: angle_rad = math.radians(heading_compass)
    # But matplotlib quiver uses angle from +X axis, so we need to convert
    # North (0°) = 90° from +X axis = π/2 rad
    # East (90°) = 0° from +X axis = 0 rad
    # So: quiver_angle = math.radians(90 - heading_compass)
    headings_rad = np.radians(90.0 - headings_compass)
    
    # Create plot
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot geofence boundary
    if boundary_xy:
        boundary_array = np.array(boundary_xy)
        polygon = MplPolygon(boundary_array, closed=True, fill=False, 
                            edgecolor='red', linewidth=2, linestyle='--', 
                            label='Geofence Boundary')
        ax.add_patch(polygon)
    
    # Plot trajectory with arrow sticks
    # Sample data for arrow sticks with spacing requirements:
    # - At least 2m distance between arrows OR
    # - At least 12 degrees heading change
    sampled_indices = [0]  # Always include first point
    min_distance_m = 2.0
    min_heading_change_deg = 12.0
    min_heading_change_rad = math.radians(min_heading_change_deg)
    
    for i in range(1, len(trajectory_xy)):
        last_idx = sampled_indices[-1]
        
        # Calculate distance from last sampled point
        dx = trajectory_xy[i, 0] - trajectory_xy[last_idx, 0]
        dy = trajectory_xy[i, 1] - trajectory_xy[last_idx, 1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate heading change from last sampled point
        heading_change = abs(headings_rad[i] - headings_rad[last_idx])
        # Handle wrap-around (heading difference can be > π)
        if heading_change > math.pi:
            heading_change = 2*math.pi - heading_change
        
        # Add point if it meets spacing criteria
        if distance >= min_distance_m or heading_change >= min_heading_change_rad:
            sampled_indices.append(i)
    
    # Always include last point
    if sampled_indices[-1] != len(trajectory_xy) - 1:
        sampled_indices.append(len(trajectory_xy) - 1)
    
    sampled_xy = trajectory_xy[sampled_indices]
    sampled_headings = headings_rad[sampled_indices]
    
    # Calculate arrow directions (unit vectors)
    arrow_length = 2.0  # meters
    u = arrow_length * np.cos(sampled_headings)
    v = arrow_length * np.sin(sampled_headings)
    
    # Plot trajectory line
    ax.plot(trajectory_xy[:, 0], trajectory_xy[:, 1], 'b-', alpha=0.3, linewidth=1, label='Trajectory')
    
    # Plot arrow sticks
    ax.quiver(sampled_xy[:, 0], sampled_xy[:, 1], u, v, 
             angles='xy', scale_units='xy', scale=1, 
             width=0.003, headwidth=3, headlength=4, 
             color='blue', alpha=0.7, label='Heading')
    
    # Plot start and end points
    ax.plot(trajectory_xy[0, 0], trajectory_xy[0, 1], 'go', markersize=10, label='Start')
    ax.plot(trajectory_xy[-1, 0], trajectory_xy[-1, 1], 'ro', markersize=10, label='End')
    
    # Plot home position
    ax.plot(0, 0, 'k*', markersize=15, label='Home')
    
    # Set equal aspect ratio and labels
    ax.set_aspect('equal')
    ax.set_xlabel('East (meters)', fontsize=12)
    ax.set_ylabel('North (meters)', fontsize=12)
    ax.set_title('Patrol Controller Test - 60s Trajectory\n(Arrow sticks show boat heading)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    # Calculate statistics
    total_distance = 0.0
    for i in range(1, len(trajectory_xy)):
        dx = trajectory_xy[i, 0] - trajectory_xy[i-1, 0]
        dy = trajectory_xy[i, 1] - trajectory_xy[i-1, 1]
        total_distance += math.sqrt(dx*dx + dy*dy)
    
    # Check if boat stayed within geofence
    inside_count = 0
    outside_count = 0
    if boundary_xy:
        try:
            from shapely.geometry import Point, Polygon
            boundary_poly = Polygon(boundary_xy)
            for point_xy in trajectory_xy:
                point = Point(point_xy[0], point_xy[1])
                if boundary_poly.contains(point):
                    inside_count += 1
                else:
                    outside_count += 1
        except ImportError:
            # Fallback: simple point-in-polygon test using ray casting
            print("   ⚠️  shapely not available, using simple point-in-polygon test")
            boundary_array = np.array(boundary_xy)
            for point_xy in trajectory_xy:
                # Simple ray casting algorithm
                x, y = point_xy[0], point_xy[1]
                n = len(boundary_array)
                inside = False
                p1x, p1y = boundary_array[0]
                for i in range(1, n + 1):
                    p2x, p2y = boundary_array[i % n]
                    if y > min(p1y, p2y):
                        if y <= max(p1y, p2y):
                            if x <= max(p1x, p2x):
                                if p1y != p2y:
                                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                                if p1x == p2x or x <= xinters:
                                    inside = not inside
                    p1x, p1y = p2x, p2y
                if inside:
                    inside_count += 1
                else:
                    outside_count += 1
    
    # Add statistics text
    stats_text = f"Duration: {timestamps[-1]:.1f}s\n"
    stats_text += f"Total distance: {total_distance:.1f}m\n"
    stats_text += f"Average speed: {total_distance/timestamps[-1]:.2f}m/s\n"
    if boundary_xy:
        total_points = inside_count + outside_count
        inside_pct = (inside_count / total_points * 100) if total_points > 0 else 0
        stats_text += f"Inside geofence: {inside_pct:.1f}% ({inside_count}/{total_points} points)\n"
        if outside_count > 0:
            stats_text += f"⚠️ Geofence violations: {outside_count} points"
    
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
           fontsize=10, verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Save plot
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ Plot saved to: {output_file}")
    
    # Try to show plot (only if display available)
    try:
        import matplotlib
        if matplotlib.get_backend() != 'Agg':
            plt.show()
        else:
            print("   (Display not available - plot saved to file)")
    except Exception:
        print("   (Display not available - plot saved to file)")


def run_simulation_test(duration_sec=60.0):
    """Run simulation test and record data.
    
    Args:
        duration_sec: Test duration in seconds (default: 60.0)
    """
    print("=" * 70)
    print(f"Patrol Controller Test - {duration_sec:.0f}s Headless Simulation")
    print("=" * 70)
    
    # Check ROS2 environment
    ros_setup = os.environ.get('ROS_DISTRO')
    if not ros_setup:
        print("\n❌ ERROR: ROS2 environment not sourced!")
        print("   Please run: source /opt/ros/humble/setup.bash")
        sys.exit(1)
    print(f"\n1. ROS2 environment: {ros_setup}")
    
    # Change to argo directory
    script_path = Path(__file__).resolve()
    argo_dir = script_path.parents[1]
    os.chdir(argo_dir)
    print(f"   Working directory: {argo_dir}")
    
    # Start simulation in background
    print(f"\n2. Starting simulation with --force-mock...")
    print(f"   (This will run for {duration_sec:.0f} seconds)")
    
    # Use ros2 launch or direct node execution
    # We'll use ros2 run to start nodes directly
    simulator_cmd = [
        'python3', 'nodes/argo_unified_simulator_bridge.py',
        '--mode', 'local',
        '--force-mock',
        '--map', 'Argo Irchel pond sailing area'
    ]
    
    controller_cmd = [
        'python3', 'nodes/controller.py',
        '--ros-args', '--params-file', 'nodes/argo.yaml'
    ]
    
    # Start nodes
    print("   Starting simulator bridge...")
    sim_proc = subprocess.Popen(
        simulator_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=dict(os.environ, **{'PYTHONUNBUFFERED': '1'})
    )
    
    time.sleep(3)  # Wait for simulator to initialize
    
    print("   Starting controller...")
    ctrl_proc = subprocess.Popen(
        controller_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=dict(os.environ, **{'PYTHONUNBUFFERED': '1'})
    )
    
    time.sleep(2)  # Wait for controller to initialize
    
    # Initialize ROS2 and start recording
    print("\n3. Starting data recording...")
    rclpy.init()
    recorder = DataRecorder()
    
    # Record for specified duration
    print(f"   Recording data for {duration_sec:.0f} seconds...")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(recorder)
    
    start_time = time.time()
    last_print = -1
    try:
        while time.time() - start_time < duration_sec:
            # Check if nodes are still running
            if sim_proc.poll() is not None:
                # Simulator process exited
                stdout, stderr = sim_proc.communicate()
                print(f"\n❌ FATAL ERROR: Simulator node crashed with exit code {sim_proc.returncode}")
                if stderr:
                    print(f"   Error output:\n{stderr.decode('utf-8', errors='ignore')}")
                break
            if ctrl_proc.poll() is not None:
                # Controller process exited
                stdout, stderr = ctrl_proc.communicate()
                print(f"\n❌ FATAL ERROR: Controller node crashed with exit code {ctrl_proc.returncode}")
                if stderr:
                    print(f"   Error output:\n{stderr.decode('utf-8', errors='ignore')}")
                break
            
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                # ROS2 context was shut down externally, break out of loop
                print("\n   ⚠️  ROS2 context shutdown detected")
                break
            elapsed = time.time() - start_time
            elapsed_int = int(elapsed)
            if elapsed_int % 10 == 0 and elapsed_int != last_print:
                print(f"   ... {elapsed_int}s elapsed ({len(recorder.gps_data)} GPS points recorded)")
                last_print = elapsed_int
    except KeyboardInterrupt:
        print("\n   ⚠️  Recording interrupted by user")
    
    print("   Recording complete!")
    
    # Stop nodes
    print("\n4. Stopping simulation nodes...")
    sim_proc.terminate()
    ctrl_proc.terminate()
    sim_proc.wait(timeout=5)
    ctrl_proc.wait(timeout=5)
    
    # Get recorded data
    timestamps, lats, lons, headings = recorder.get_data()
    
    if timestamps is None or len(timestamps) == 0:
        print("\n❌ ERROR: No data recorded!")
        return
    
    print(f"\n5. Recorded {len(timestamps)} GPS points")
    
    # Print control authority statistics
    total_control_samples = recorder.human_controlled_samples + recorder.robot_controlled_samples
    if total_control_samples > 0:
        human_pct = (recorder.human_controlled_samples / total_control_samples) * 100
        robot_pct = (recorder.robot_controlled_samples / total_control_samples) * 100
        print(f"\n   Control Authority Statistics:")
        print(f"   - Human control: {recorder.human_controlled_samples} samples ({human_pct:.1f}%)")
        print(f"   - Robot control: {recorder.robot_controlled_samples} samples ({robot_pct:.1f}%)")
        print(f"   - Auto commands received: {recorder.auto_cmd_count}")
        
        if human_pct > 50:
            print(f"\n   ⚠️  WARNING: Boat was in HUMAN control mode most of the time!")
            print(f"   ⚠️  This suggests the controller never took control.")
            print(f"   ⚠️  Check that /human_controlled topic switches to False after timeout.")
    else:
        print(f"\n   ⚠️  WARNING: No control authority data recorded!")
    
    # Load geofence
    print("\n6. Loading geofence data...")
    home_pos, boundary_coords = load_geofence()
    if home_pos[0] is None:
        print("   ⚠️  Warning: Could not find home waypoint")
        home_pos = (lats[0], lons[0])  # Use first GPS point as home
    
    # Plot trajectory
    print("\n7. Plotting trajectory...")
    output_file = os.path.join(argo_dir, OUTPUT_FILENAME if 'OUTPUT_FILENAME' in globals() else 'patrol_test_result.png')
    plot_trajectory(timestamps, lats, lons, headings, home_pos, boundary_coords, output_file=output_file)
    
    print("\n✅ Test complete!")
    print("=" * 70)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Test patrol controller with simulation')
    parser.add_argument('--duration', type=float, default=60.0,
                       help='Test duration in seconds (default: 60.0)')
    parser.add_argument('--output', type=str, default='patrol_test_result.png',
                       help='Output plot filename (default: patrol_test_result.png)')
    args = parser.parse_args()
    
    try:
        # Store output filename globally for use in run_simulation_test
        global OUTPUT_FILENAME
        OUTPUT_FILENAME = args.output
        
        run_simulation_test(duration_sec=args.duration)
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

