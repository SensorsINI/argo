#!/usr/bin/env python3
# ruff: noqa: I001
"""
IMU Sensor Node for Argo Autonomous Sailboat
============================================

This ROS2 node interfaces with the ICM-20948 9-axis IMU sensor (accelerometer, 
gyroscope, magnetometer) via I2C and publishes raw sensor data to ROS2 topics.

Hardware:
- SparkfunICM-20948 9-axis 
IMU sensor https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/#documentation 
https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html
- I2C bus 0, address 0x69
- AK09916 magnetometer (integrated in ICM-20948)

Axes/Coordinate Frame: (see https://cdn.sparkfun.com/assets/learn_tutorials/8/9/3/DS-000189-ICM-20948-v1.3.pdf section 15, figs 12-13)
For the ICM-20948, the coordinate frame is defined as follows for the accelerometer:
- +x: rightwards, starboard
- +y: forwards, towards bow
- +z: down (gravity vector), measured as negative reaction force from sensor
  Note: z-axis is flipped from hardware convention to represent gravity vector direction

For the magnetometer, the coordinate frame is defined as follows:
- +x: rightwards, starboard
- +y: backwards, towards stern
- z: down into water
For the gyroscope, the coordinate frame is defined as follows:

Published Topics:
- /accel (geometry_msgs/Vector3): Accelerometer data in g (gravity units)
  Note: z-axis represents gravity vector direction (≈-1g when level, pointing down)
- /gyro (geometry_msgs/Vector3): Raw gyroscope data in deg/s (degrees per second)  
- /magnetometer (geometry_msgs/Vector3): Raw magnetometer data in uT (microtesla)
- /compass (std_msgs/Float64): Tilt-compensated compass heading in degrees (0-360, 0=North, 90=East)
- /imu_health (std_msgs/Bool): Node health status (true=healthy, false=failed)

Note: Compass heading is tilt-compensated using the gravity vector from the accelerometer.
The magnetic field is projected onto the horizontal plane (perpendicular to gravity) before
computing the heading. This provides accurate heading even when the boat is tilted.

Command Line Options:
--debug              Enable debug logging to show sensor values being published
--calib_compass      Run magnetometer calibration mode (interactive)
                     - Rotate device through all orientations
                     - Press Ctrl+C to finish and save calibration
                     - Robust against transient I2C errors with automatic recovery
                     - Choose between min-max or ellipsoid fitting methods
                     - Saves calibration to nodes/invensense-20948-compass-calibration.json
                     - Backs up old calibration to nodes/imu_calib_backups/
                     - Saves timestamped samples to nodes/ for persistent storage
                     - Generates time-series and 3D calibration plots in /tmp
--plot_calib         Plot the most recent calibration data from /tmp or nodes/
                     - Loads the latest calibration samples
                     - Generates time-series plot showing all 3 axes over time
                     - Creates interactive 3D plot (rotatable with mouse when DISPLAY set)
                     - Shows uncalibrated (red) vs calibrated (green) data with ideal sphere
                     - Prints calibration statistics

Calibration Methods:
1. Min-max (simple):    Diagonal soft-iron approximation, good for basic calibration
2. Ellipsoid (advanced): Full 3D ellipsoid fit with rotation correction, better accuracy

Usage Examples:
  python3 imu.py                    # Normal operation
  python3 imu.py --debug            # With debug output
  python3 imu.py --calib_compass    # Calibrate magnetometer (interactive method selection)
  python3 imu.py --plot_calib       # Review latest calibration data with 3D visualization
"""
# Standard library imports
# isort: off

import math
import time
import struct
import argparse
import json
from datetime import datetime
import os.path
import sys
import os
import shutil

# Check if we're in standalone mode (plot or calib) before importing ROS2
# This allows using the script without ROS2 for plotting/calibration
_STANDALONE_MODE = (
    '--plot-calib' in sys.argv or '--plot_calib' in sys.argv or
    '--calib-compass' in sys.argv or '--calib_compass' in sys.argv
)

if not _STANDALONE_MODE:
    # ROS imports - only needed for node operation
    from std_msgs.msg import Bool, Float64
    from geometry_msgs.msg import Vector3
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    import rclpy
    
    # Add custom import path BEFORE importing from it
    sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
    from toggle_pause_service import TogglePauseService
else:
    # Stub classes for standalone mode (plot/calib don't use ROS2)
    class Node:
        """Stub Node class for standalone mode"""
        pass
    
    class TogglePauseService:
        """Stub TogglePauseService for standalone mode"""
        pass

# isort: on


def get_terminal_size():
    """Get current terminal size (width, height), with fallback."""
    try:
        size = shutil.get_terminal_size()
        return size.columns, size.lines
    except:
        return 80, 24  # fallback width, height


def plot_magnetometer_calibration(timestamped_samples, calib, timestamp):
    """Generate and save magnetometer calibration plot.

    Args:
        timestamped_samples: List of (time, mx, my, mz) tuples
        calib: Calibration dict with bias_uT and scale_diag
        timestamp: Timestamp string for filename

    Returns:
        Path to saved plot file
    """
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt

    times = [s[0] for s in timestamped_samples]
    mx_vals = [s[1] for s in timestamped_samples]
    my_vals = [s[2] for s in timestamped_samples]
    mz_vals = [s[3] for s in timestamped_samples]

    minx, maxx = min(mx_vals), max(mx_vals)
    miny, maxy = min(my_vals), max(my_vals)
    minz, maxz = min(mz_vals), max(mz_vals)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    fig.suptitle(f'Magnetometer Calibration Data - {timestamp}', fontsize=14)

    axes[0].plot(times, mx_vals, 'r-', linewidth=0.5)
    axes[0].axhline(y=minx, color='r', linestyle='--',
                    alpha=0.5, label=f'min={minx:.1f}')
    axes[0].axhline(y=maxx, color='r', linestyle='--',
                    alpha=0.5, label=f'max={maxx:.1f}')
    axes[0].set_ylabel('Mx (µT)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, my_vals, 'g-', linewidth=0.5)
    axes[1].axhline(y=miny, color='g', linestyle='--',
                    alpha=0.5, label=f'min={miny:.1f}')
    axes[1].axhline(y=maxy, color='g', linestyle='--',
                    alpha=0.5, label=f'max={maxy:.1f}')
    axes[1].set_ylabel('My (µT)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, mz_vals, 'b-', linewidth=0.5)
    axes[2].axhline(y=minz, color='b', linestyle='--',
                    alpha=0.5, label=f'min={minz:.1f}')
    axes[2].axhline(y=maxz, color='b', linestyle='--',
                    alpha=0.5, label=f'max={maxz:.1f}')
    axes[2].set_ylabel('Mz (µT)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    # Add calibration info as text if available
    if calib:
        bias = calib.get('bias_uT', [0, 0, 0])
        scale = calib.get('scale_diag', [1, 1, 1])
        method = calib.get('method', 'unknown')
        info_text = f"Method: {method}\n"
        info_text += f"Bias: [{bias[0]:.2f}, {bias[1]:.2f}, {bias[2]:.2f}] µT\n"
        info_text += f"Scale: [{scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f}]"
        fig.text(0.02, 0.02, info_text, fontsize=10, family='monospace',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plot_file = f'/tmp/imu_calib_plot_{timestamp}.png'
    plt.savefig(plot_file, dpi=150)
    plt.close()

    return plot_file


def display_plot(plot_file, skip_open=False):
    """Attempt to display plot file in default image viewer if display is available.
    
    Args:
        plot_file: Path to the plot file
        skip_open: If True, don't try to open with xdg-open (e.g., already shown interactively)
    """
    print(f"Saved calibration plot to: {plot_file}")
    
    if skip_open:
        return
    
    if os.environ.get('DISPLAY'):
        try:
            import subprocess
            subprocess.Popen(['xdg-open', plot_file],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            print("Opening plot in default image viewer...")
        except Exception:
            print("Display available but failed to open image viewer")
    else:
        print("Display not available (headless system or DISPLAY not set)")


def fit_ellipsoid_numpy(points):
    """Fit an ellipsoid to 3D points using algebraic fit (pure numpy).
    
    Fits ellipsoid equation: ax^2 + by^2 + cz^2 + 2fxy + 2gxz + 2hyz + 2px + 2qy + 2rz + d = 0
    
    Args:
        points: Nx3 numpy array of (x, y, z) coordinates
        
    Returns:
        tuple: (center, radii, rotation_matrix) or None if fit fails
            center: [cx, cy, cz] center of ellipsoid
            radii: [rx, ry, rz] semi-axes lengths
            rotation_matrix: 3x3 rotation matrix
    """
    import numpy as np
    
    if len(points) < 9:
        print("Error: Need at least 9 points for ellipsoid fit")
        return None
        
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Build design matrix for algebraic ellipsoid fit
    # [x^2, y^2, z^2, 2xy, 2xz, 2yz, 2x, 2y, 2z, 1]
    D = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones(len(x))
    ])
    
    # Solve using least squares: D * v = 0, with constraint
    # Use SVD for numerical stability
    try:
        _, _, Vt = np.linalg.svd(D)
        v = Vt[-1, :]  # Last row of V (smallest singular value)
        
        # Extract algebraic form parameters
        a, b, c, f, g, h, p, q, r, d = v
        
        # Convert to center form
        # Build the A matrix for the quadratic form
        A = np.array([
            [a, f, g],
            [f, b, h],
            [g, h, c]
        ])
        
        # Center vector
        center = -np.linalg.solve(A, np.array([p, q, r]))
        
        # Translation to center
        T = np.eye(4)
        T[0:3, 3] = center
        
        # Evaluate constant at center
        d_center = d + p*center[0] + q*center[1] + r*center[2]
        
        # Eigenvalue decomposition for radii and rotation
        # Ensure the matrix is positive definite by using abs(d_center)
        eigvals, eigvecs = np.linalg.eigh(A / np.abs(d_center))
        radii = 1.0 / np.sqrt(eigvals)
        
        # Ensure right-handed coordinate system (det = +1)
        # eigh doesn't guarantee this, so we fix it if needed
        if np.linalg.det(eigvecs) < 0:
            eigvecs[:, 0] *= -1  # Flip first eigenvector
        
        return center, radii, eigvecs
        
    except np.linalg.LinAlgError as e:
        print(f"Error: Ellipsoid fit failed - {e}")
        return None


def apply_ellipsoid_calibration(points, center, radii, rotation):
    """Apply ellipsoid calibration to transform points to unit sphere.
    
    Args:
        points: Nx3 array of raw magnetometer readings
        center: [cx, cy, cz] ellipsoid center (hard iron offset)
        radii: [rx, ry, rz] ellipsoid semi-axes (soft iron scale)
        rotation: 3x3 rotation matrix (soft iron rotation)
        
    Returns:
        Nx3 array of calibrated points
    """
    import numpy as np
    
    # Center the points
    centered = points - center
    
    # Scale by radii to make sphere
    # Use average radius for normalization
    r_avg = np.mean(radii)
    scale = r_avg / radii
    
    # Apply rotation and scaling to transform ellipsoid to sphere
    # rotation matrix from eigh has eigenvectors as COLUMNS
    # Each column i is the direction of principal axis i
    # Multiplying by rotation projects centered points onto the principal axes
    # This aligns the ellipsoid axes with coordinate axes, then scaling makes it a sphere
    calibrated = centered @ rotation @ np.diag(scale)
    
    return calibrated


def plot_magnetometer_3d(samples, calib, timestamp, output_dir='/tmp', interactive=None):
    """Generate and save 3D visualization of uncalibrated vs calibrated magnetometer data.
    
    Args:
        samples: List of (mx, my, mz) tuples (raw magnetometer data)
        calib: Calibration dict with bias_uT, scale_diag, and optionally rotation matrix
        timestamp: Timestamp string for filename
        output_dir: Directory to save plot (default /tmp)
        interactive: If True, use interactive backend; if None, auto-detect from DISPLAY
        
    Returns:
        Path to saved plot file
    """
    try:
        import numpy as np
        import matplotlib
        
        # Determine if we should use interactive backend
        if interactive is None:
            interactive = bool(os.environ.get('DISPLAY'))
        
        if interactive:
            # Use default interactive backend (TkAgg, Qt5Agg, etc.)
            matplotlib.use('TkAgg')
        else:
            matplotlib.use('Agg')  # Non-interactive backend
            
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    except ImportError as e:
        print(f"Error: Required library not available - {e}")
        print("Install with: pip3 install numpy matplotlib")
        return None
    
    # Convert samples to numpy array
    points = np.array(samples)
    
    # Apply calibration
    bias = np.array(calib.get('bias_uT', [0, 0, 0]))
    scale = np.array(calib.get('scale_diag', [1, 1, 1]))
    
    # Check if we have rotation matrix (ellipsoid fit)
    if 'rotation' in calib:
        rotation = np.array(calib['rotation'])
        radii = np.array(calib.get('radii', [1, 1, 1]))
        # Full ellipsoid calibration
        calibrated = apply_ellipsoid_calibration(points, bias, radii, rotation)
    else:
        # Simple min-max calibration
        calibrated = (points - bias) * scale
    
    # Create single 3D plot with both datasets
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot uncalibrated data in red
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                c='red', marker='o', s=2, alpha=0.5, label='Uncalibrated (raw)')
    
    # Plot calibrated data in green
    ax.scatter(calibrated[:, 0], calibrated[:, 1], calibrated[:, 2],
                c='green', marker='o', s=2, alpha=0.5, label='Calibrated (corrected)')
    
    # Create ideal sphere mesh (target for calibrated data)
    # Calculate sphere radius as mean magnitude of calibrated points
    # Each point should lie on a sphere with radius = Earth's magnetic field strength
    cal_radius = np.mean(np.sqrt(calibrated[:, 0]**2 + calibrated[:, 1]**2 + calibrated[:, 2]**2))
    
    # Generate sphere mesh
    u = np.linspace(0, 2 * np.pi, 90)
    v = np.linspace(0, np.pi, 90)
    sphere_x = cal_radius * np.outer(np.cos(u), np.sin(v))
    sphere_y = cal_radius * np.outer(np.sin(u), np.sin(v))
    sphere_z = cal_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    
    # Plot ideal sphere as wireframe in light gray
    ax.plot_wireframe(sphere_x, sphere_y, sphere_z, 
                      color='blue', alpha=0.3, linewidth=.5,
                      label='Ideal sphere')
    
    ax.set_xlabel('X (µT)')
    ax.set_ylabel('Y (µT)')
    ax.set_zlabel('Z (µT)')
    ax.set_title('Magnetometer Calibration: Before (Red) vs After (Green)')
    ax.legend(loc='upper right')
    
    # Center axes on zero and use uniform scaling
    # Find maximum absolute value across all data points and all axes
    all_points = np.vstack([points, calibrated])
    max_val = np.abs(all_points).max()
    
    # Set all axes to same scale centered on zero
    ax.set_xlim(-max_val, max_val)
    ax.set_ylim(-max_val, max_val)
    ax.set_zlim(-max_val, max_val)
    
    # Add calibration info as text
    method = calib.get('method', 'unknown')
    num_samples = len(samples)
    info_text = f"Method: {method}\n"
    info_text += f"Samples: {num_samples}\n"
    info_text += f"Bias: [{bias[0]:.2f}, {bias[1]:.2f}, {bias[2]:.2f}] µT\n"
    info_text += f"Scale: [{scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f}]"
    
    fig.text(0.5, 0.02, info_text, fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
             ha='center')
    
    plt.suptitle(f'Magnetometer Calibration 3D View - {timestamp}', fontsize=14)
    plt.tight_layout()
    
    # Save to file
    plot_file = os.path.join(output_dir, f'imu_calib_3d_{timestamp}.png')
    plt.savefig(plot_file, dpi=150)
    
    # Show interactively if possible
    if interactive:
        try:
            plt.show()  # This will be interactive and rotatable
        except Exception:
            pass  # If show fails, just skip it
    
    plt.close()
    
    return plot_file


def _to_int16(msb, lsb):
    return struct.unpack('>h', bytes([msb, lsb]))[0]


def _prompt_yes_no(message: str, default_yes: bool = True) -> bool:
    """Prompt user for yes/no response."""
    opts = "Y/n" if default_yes else "y/N"
    try:
        resp = input(f"{message} [{opts}]: ").strip().lower()
    except Exception:
        resp = ''
    if resp == '':
        return default_yes
    if resp in ('y', 'yes'):
        return True
    if resp in ('n', 'no'):
        return False
    return default_yes


def _backup_existing_calibration(calib_filename: str) -> None:
    """Backup existing calibration file with datestamp from file creation date."""
    if not os.path.exists(calib_filename):
        return  # No existing file to backup

    # Get file modification time (creation date)
    file_mtime = os.path.getmtime(calib_filename)
    file_date = datetime.fromtimestamp(file_mtime).strftime('%Y%m%d_%H%M%S')

    # Create backup directory in nodes/ if it doesn't exist
    script_dir = os.path.dirname(os.path.abspath(__file__))
    backup_dir = os.path.join(script_dir, 'imu_calib_backups')
    os.makedirs(backup_dir, exist_ok=True)

    # Create backup filename with datestamp
    base_name = os.path.splitext(os.path.basename(calib_filename))[0]
    backup_filename = os.path.join(backup_dir, f"{base_name}_{file_date}.json")

    # Copy to backup
    shutil.copy2(calib_filename, backup_filename)
    print(f"Backed up previous calibration to: {backup_filename}")


def compute_calibration_from_samples(samples):
    """Compute calibration from magnetometer samples.
    
    Args:
        samples: List of (mx, my, mz) tuples
        
    Returns:
        Dictionary with calibration parameters, or None if computation failed
    """
    # Ask user which calibration method to use
    print("\nCalibration method:")
    print("  1) Min-max (simple, diagonal soft-iron approximation)")
    print("  2) Ellipsoid fit (advanced, full 3D rotation correction)")
    
    try:
        method_choice = input("Choose method (1 or 2) [2]: ").strip()
        use_ellipsoid = (method_choice == '' or method_choice == '2')
    except Exception:
        use_ellipsoid = True  # Default to ellipsoid
    
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    zs = [s[2] for s in samples]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    minz, maxz = min(zs), max(zs)
    
    if use_ellipsoid:
        # Ellipsoid fitting
        print("\nComputing ellipsoid fit...")
        try:
            import numpy as np
            points = np.array(samples)
            result = fit_ellipsoid_numpy(points)
            
            if result is None:
                print("Ellipsoid fit failed, falling back to min-max method")
                use_ellipsoid = False
            else:
                center, radii, rotation = result
                
                # Convert rotation matrix to list for JSON serialization
                rotation_list = rotation.tolist()
                
                # Compute scale factors from radii
                r_avg = np.mean(radii)
                scale = r_avg / radii
                
                calib = {
                    'method': 'ellipsoid',
                    'bias_uT': center.tolist(),
                    'scale_diag': scale.tolist(),
                    'rotation': rotation_list,
                    'radii': radii.tolist(),
                    'num_samples': len(samples),
                    'ranges_uT': {'x': [minx, maxx], 'y': [miny, maxy], 'z': [minz, maxz]},
                    'timestamp': datetime.utcnow().isoformat() + 'Z'
                }
                
                print("\nComputed compass calibration (ellipsoid fit):")
                print(f"  bias_uT     = [{center[0]:+.2f}, {center[1]:+.2f}, {center[2]:+.2f}]")
                print(f"  scale_diag  = [{scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f}]")
                print(f"  radii       = [{radii[0]:.2f}, {radii[1]:.2f}, {radii[2]:.2f}] µT")
                print(f"  rotation matrix:")
                for i in range(3):
                    print(f"    [{rotation[i,0]:+.4f}, {rotation[i,1]:+.4f}, {rotation[i,2]:+.4f}]")
                print(f"  ranges_uT   = X[{minx:+.1f}..{maxx:+.1f}] Y[{miny:+.1f}..{maxy:+.1f}] Z[{minz:+.1f}..{maxz:+.1f}] (n={len(samples)})")
                return calib
                
        except ImportError:
            print("Error: numpy required for ellipsoid fit, falling back to min-max")
            use_ellipsoid = False
        except Exception as e:
            print(f"Error: Ellipsoid fit failed - {e}, falling back to min-max")
            import traceback
            traceback.print_exc()
            use_ellipsoid = False
    
    if not use_ellipsoid:
        # Min-max calibration (diagonal soft-iron approximation)
        bx = (maxx + minx) / 2.0
        by = (maxy + miny) / 2.0
        bz = (maxz + minz) / 2.0
        rx = (maxx - minx) / 2.0
        ry = (maxy - miny) / 2.0
        rz = (maxz - minz) / 2.0
        r_avg = (rx + ry + rz) / 3.0 if (rx + ry + rz) > 0 else 1.0
        sx = r_avg / rx if rx != 0 else 1.0
        sy = r_avg / ry if ry != 0 else 1.0
        sz = r_avg / rz if rz != 0 else 1.0

        calib = {
            'method': 'minmax',
            'bias_uT': [bx, by, bz],
            'scale_diag': [sx, sy, sz],
            'num_samples': len(samples),
            'ranges_uT': {'x': [minx, maxx], 'y': [miny, maxy], 'z': [minz, maxz]},
            'timestamp': datetime.utcnow().isoformat() + 'Z'
        }
        
        print("\nComputed compass calibration (min-max diag fit):")
        print(f"  bias_uT     = [{bx:+.2f}, {by:+.2f}, {bz:+.2f}]")
        print(f"  scale_diag  = [{sx:.4f}, {sy:.4f}, {sz:.4f}]")
        print(f"  ranges_uT   = X[{minx:+.1f}..{maxx:+.1f}] Y[{miny:+.1f}..{maxy:+.1f}] Z[{minz:+.1f}..{maxz:+.1f}] (n={len(samples)})")
        return calib
    
    return None


class ICM20948:
    def __init__(self, bus, address=0x69):
        self.bus = bus
        self.addr = address
        self._reg_bank = None

    # Low-level I2C helpers
    def read_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read_bytes(self, reg, length):
        return list(self.bus.read_i2c_block_data(self.addr, reg, length))

    def select_bank(self, bank):
        if self._reg_bank != bank:
            self.write_byte(0x7F, bank)
            self._reg_bank = bank

    def set_sleep_mode(self, enable: bool) -> None:
        """Enable/disable sleep mode via PWR_MGMT_1[6] (SLEEP bit).

        When set, the chip enters sleep (all analog powered off). Clearing the
        bit wakes the chip. See ICM-20948 datasheet, PWR_MGMT_1 register.
        """
        # PWR_MGMT_1 is in bank 0 at address 0x06
        self.select_bank(0x00)
        try:
            val = self.read_byte(0x06)
        except Exception:
            # Fallback: assume default if read fails
            val = 0x01  # auto clock, sleep off default used in initialize()
        if enable:
            val |= 0x40  # set SLEEP bit (bit 6)
        else:
            val &= ~0x40  # clear SLEEP bit
        self.write_byte(0x06, val)
        time.sleep(0.01)

    # Minimal bring-up
    def initialize(self):
        # Reset, then wake
        self.select_bank(0x00)
        self.write_byte(0x06, 0x80)  # PWR_MGMT_1: DEVICE_RESET
        time.sleep(0.1)
        self.write_byte(0x06, 0x01)  # auto clock, sleep off
        time.sleep(0.05)

        # Configure gyro/accel (bank 2): FSR and LPF
        self.select_bank(0x20)
        # Gyro: FSR=250 dps (00), LPF index ~4, enable LPF
        gyro_cfg = ((4 & 0x07) << 3) | (0 << 1) | 1
        self.write_byte(0x01, gyro_cfg)  # GYRO_CONFIG_1
        self.write_byte(0x02, 0x00)      # GYRO_CONFIG_2
        # Accel: FSR=2g (00), LPF index ~4, enable LPF
        accel_cfg = ((4 & 0x07) << 3) | (0 << 1) | 1
        self.write_byte(0x14, accel_cfg)  # ACCEL_CONFIG
        self.write_byte(0x15, 0x00)       # ACCEL_CONFIG_2

        # Enable bypass so AK09916 is accessible at 0x0C on the same bus
        self.select_bank(0x00)
        user_ctrl = self.bus.read_byte_data(self.addr, 0x03)
        user_ctrl &= ~0x20  # clear I2C_MST_EN
        self.write_byte(0x03, user_ctrl)
        time.sleep(0.01)
        self.write_byte(0x0F, 0x82)  # INT_PIN_CFG: BYPASS_EN
        time.sleep(0.05)

    # Raw reads
    def read_accel(self):
        """
        Read accelerometer data from ICM-20948.

        Returns:
            tuple: (ax_cnt, ay_cnt, az_cnt) raw accelerometer counts
        """
        self.select_bank(0x00)
        b = self.read_bytes(0x2D, 6)
        return (
            _to_int16(b[0], b[1]),
            _to_int16(b[2], b[3]),
            _to_int16(b[4], b[5]),
        )

    def read_gyro(self):
        """
        Read gyroscope data from ICM-20948.

        Returns:
            tuple: (gx_cnt, gy_cnt, gz_cnt) raw gyroscope counts
        """
        self.select_bank(0x00)
        b = self.read_bytes(0x33, 6)
        return (
            _to_int16(b[0], b[1]),
            _to_int16(b[2], b[3]),
            _to_int16(b[4], b[5]),
        )

    def read_magnetometer(self):
        """
        Read magnetometer data from AK09916 via I2C bypass.

        Register mapping per ICM-20948 datasheet Table 20:
        - 0x10 (ST1):  Status 1, bit 0 = DRDY (data ready)
        - 0x11 (HXL):  X-axis data low byte
        - 0x12 (HXH):  X-axis data high byte
        - 0x13 (HYL):  Y-axis data low byte
        - 0x14 (HYH):  Y-axis data high byte
        - 0x15 (HZL):  Z-axis data low byte
        - 0x16 (HZH):  Z-axis data high byte
        - 0x18 (ST2):  Status 2 (included in 8-byte read but not used)

        Returns:
            tuple: (mx_uT, my_uT, mz_uT) magnetometer readings in microtesla
        """
        mx_uT = my_uT = mz_uT = 0.0
        try:
            # Read ST1 (0x10) to check if data is ready
            st1 = self.bus.read_byte_data(0x0C, 0x10)
            if st1 & 0x01:  # DRDY bit set
                # Read 8 bytes starting at HXL (0x11): HXL, HXH, HYL, HYH, HZL, HZH, ST2, reserved
                mb = list(self.bus.read_i2c_block_data(0x0C, 0x11, 8))

                # Assemble 16-bit values (little-endian format per AK09916 datasheet)
                # mb[0]=HXL (0x11), mb[1]=HXH (0x12) -> X-axis
                mx_cnt = struct.unpack('<h', bytes(mb[0:2]))[0]
                # mb[2]=HYL (0x13), mb[3]=HYH (0x14) -> Y-axis
                my_cnt = struct.unpack('<h', bytes(mb[2:4]))[0]
                # mb[4]=HZL (0x15), mb[5]=HZH (0x16) -> Z-axis
                mz_cnt = struct.unpack('<h', bytes(mb[4:6]))[0]

                # Convert to microtesla (AK09916 sensitivity: 0.15 µT/LSB)
                mx_uT = mx_cnt * 0.15
                my_uT = my_cnt * 0.15
                mz_uT = mz_cnt * 0.15
        except Exception:
            pass

        return mx_uT, my_uT, mz_uT


class ImuNode(Node):
    def __init__(self, debug=False):
        super().__init__('imu_node')

        # Initialize pause service with namespaced name
        self.pause_service = TogglePauseService(
            self, f'{self.get_name()}/toggle_pause')
        # Track pause state for sleep/wake transitions
        self._prev_paused = False

        self.debug = debug

        self.get_logger().info('Initializing IMU node...')

        if not self.debug:
            self.get_logger().info("Run with --debug to see sensor values being published.")

        # Define calibration file paths (in nodes/ directory)
        self._script_dir = os.path.dirname(os.path.abspath(__file__))
        self._calib_file = os.path.join(
            self._script_dir, 'invensense-20948-compass-calibration.json')
        self._backup_dir = os.path.join(self._script_dir, 'imu_calib_backups')

        # I2C setup
        # OrangePi uses bus 0 (confirmed by RTIMULib defaults)
        self.i2c_bus_num = 0
        try:
            try:
                from smbus2 import SMBus  # type: ignore
            except Exception:
                from smbus import SMBus  # type: ignore
            self.bus = SMBus(self.i2c_bus_num)
        except Exception as e:
            self.get_logger().error(
                f"Failed to open I2C bus {self.i2c_bus_num}: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # IMU device
        self.icm_addr = 0x69
        self.icm = ICM20948(self.bus, self.icm_addr)

        if not self._initialize_sensors():
            self.get_logger().fatal("FATAL: IMU init failed")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Load compass calibration if available
        self._compass_cal = None
        try:
            with open(self._calib_file, 'r') as f:
                self._compass_cal = json.load(f)
                self.get_logger().info(
                    f'Loaded compass calibration from nodes/{os.path.basename(self._calib_file)}')
                # Print calibration values for verification
                if isinstance(self._compass_cal, dict):
                    if 'bias_uT' in self._compass_cal:
                        bx, by, bz = self._compass_cal['bias_uT']
                        self.get_logger().info(
                            f'  Bias (µT): x={bx:.2f}, y={by:.2f}, z={bz:.2f}')
                    if 'scale_diag' in self._compass_cal:
                        sx, sy, sz = self._compass_cal['scale_diag']
                        self.get_logger().info(
                            f'  Scale: x={sx:.4f}, y={sy:.4f}, z={sz:.4f}')
                    if 'method' in self._compass_cal:
                        self.get_logger().info(
                            f'  Method: {self._compass_cal["method"]}')
        except Exception:
            pass

        # Publishers
        self.pub_accel = self.create_publisher(Vector3, 'accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, 'gyro', 10)
        self.pub_magnetometer = self.create_publisher(
            Vector3, 'magnetometer', 10)
        self.pub_compass = self.create_publisher(Float64, 'compass', 10)

        # Health status publisher
        self.pub_health = self.create_publisher(Bool, 'imu_health', 10)
        self.health_status = False  # Track current health status

        # Node health tracking for transient I2C failures
        self.node_healthy = True
        self._last_io_error_log_time = 0.0
        self._consecutive_io_errors = 0
        # Total errors since becoming unhealthy (for display)
        self._total_errors_this_session = 0
        self._last_successful_read_time = time.time()
        self._last_recovery_attempt_time = 0.0
        self._recovery_attempt_count = 0
        # For throttled unreachable sensor logging
        self._last_unreachable_log_time = 0.0

        # Magnetometer lowpass filter parameters
        self.mag_lowpass_cutoff_hz = 1.0  # Cutoff frequency in Hz
        self.sample_rate_hz = 10.0  # Timer runs at 10 Hz
        # Calculate filter coefficient alpha = dt / (dt + tau)
        # where tau = 1 / (2 * pi * fc)
        dt = 1.0 / self.sample_rate_hz
        tau = 1.0 / (2.0 * math.pi * self.mag_lowpass_cutoff_hz)
        self.mag_filter_alpha = dt / (dt + tau)
        self.get_logger().info(
            f'Magnetometer lowpass filter: {self.mag_lowpass_cutoff_hz} Hz cutoff (alpha={self.mag_filter_alpha:.4f})')
        # Filter state (previous filtered values)
        self.mag_filtered_x = 0.0
        self.mag_filtered_y = 0.0
        self.mag_filtered_z = 0.0
        self.mag_filter_initialized = False

        # ASCII visual debug
        self._vis_ascii = self.debug
        self._vis_initialized = False
        if self._vis_ascii:
            self._init_ascii_vis()

        # Main loop timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Publish initial health status as healthy
        self._publish_health_status(True)

    def _publish_health_status(self, is_healthy: bool):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            # Suppress health status logging when ASCII visualization is active
            # to avoid disrupting the display
            if not self._vis_ascii:
                if is_healthy:
                    self.get_logger().info("IMU health status: HEALTHY")
                else:
                    self.get_logger().warn("IMU health status: FAILED")

    def _handle_io_error(self, error):
        """Handle I2C IOError with health tracking and throttled logging"""
        current_time = time.time()
        self._consecutive_io_errors += 1
        self._total_errors_this_session += 1  # Track total for display

        # Log error with throttling (max once per 5 seconds)
        # Suppress logging when ASCII visualization is active
        if current_time - self._last_io_error_log_time >= 5.0:
            if not self._vis_ascii:
                self.get_logger().warn(
                    f"Transient I2C read error (attempt {self._consecutive_io_errors}): {error}")
            self._last_io_error_log_time = current_time

        # Mark node as unhealthy after first IO error
        if self.node_healthy:
            self.node_healthy = False
            self._publish_health_status(False)

            # Keep ASCII visualization active to show recovery status
            # Don't tear it down - we'll display recovery information instead

            if not self._vis_ascii:
                self.get_logger().warn(
                    "Node health set to UNHEALTHY due to I2C errors. Switching to 1Hz retry mode.")

            # Switch to low-frequency retry mode
            self._switch_to_retry_mode()

    def _switch_to_retry_mode(self):
        """Switch timer to low-frequency retry mode (1Hz)"""
        if hasattr(self, 'timer'):
            self.timer.destroy()
        self.timer = self.create_timer(1.0, self.timer_callback)

        if not self._vis_ascii:
            self.get_logger().info("Switched to 1Hz retry mode for I2C recovery")

        # Reset consecutive error counter for recovery tracking
        self._consecutive_io_errors = 0

    def _switch_to_normal_mode(self):
        """Switch timer back to normal frequency (10Hz)"""
        if hasattr(self, 'timer'):
            self.timer.destroy()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        if not self._vis_ascii:
            self.get_logger().info("Switched back to normal 10Hz mode - I2C communication recovered")

    def _initialize_sensors(self, verbose=True):
        """Initialize IMU sensors (ICM-20948 and AK09916 magnetometer)

        Args:
            verbose: If False, suppress logging (for throttled retry attempts)
        """
        try:
            # Initialize the ICM-20948
            self.icm.initialize()

            # Setup AK09916 magnetometer via bypass
            AK_ADDR = 0x0C
            # soft reset
            self.bus.write_byte_data(AK_ADDR, 0x32, 0x01)
            time.sleep(0.05)
            # continuous measurement 100Hz
            self.bus.write_byte_data(AK_ADDR, 0x31, 0x08)
            time.sleep(0.01)

            if verbose:
                self.get_logger().info("ICM-20948 init complete (raw mode)")
            return True

        except Exception as e:
            if verbose:
                self.get_logger().error(
                    f"IMU sensor initialization failed: {e}")
            return False

    def _reinitialize_sensors(self, verbose=True):
        """Re-initialize IMU sensors after I2C recovery

        Args:
            verbose: If False, suppress logging (for throttled retry attempts)
        """
        if verbose and not self._vis_ascii:
            self.get_logger().info("Re-initializing IMU sensors...")

        if self._initialize_sensors(verbose=verbose):
            if verbose and not self._vis_ascii:
                self.get_logger().info("IMU sensor re-initialization successful")
            return True
        else:
            if verbose and not self._vis_ascii:
                self.get_logger().error("IMU sensor re-initialization failed")
            return False

    def _check_io_recovery(self):
        """Check if I2C communication has recovered and switch back to normal mode"""
        current_time = time.time()
        time_since_last_success = current_time - self._last_successful_read_time
        time_since_last_recovery_attempt = current_time - self._last_recovery_attempt_time

        # Try recovery if we've been in retry mode for a while
        # This allows recovery even with ongoing errors, since sensors need re-initialization
        if (time_since_last_success > 5.0 and  # Been in retry mode for at least 5 seconds
                time_since_last_recovery_attempt > 3.0):  # Wait at least 3 seconds between attempts

            self._recovery_attempt_count += 1
            self._last_recovery_attempt_time = current_time

            # Determine if we should log verbosely (only on 60s intervals in normal mode)
            # In _vis_ascii mode, always be verbose
            # In normal mode, only log when we just logged the "unreachable" message
            time_since_unreachable_log = current_time - self._last_unreachable_log_time
            # Within 2s of last unreachable log
            verbose_logging = (
                self._vis_ascii or time_since_unreachable_log < 2.0)

            # Re-initialize sensors after I2C recovery
            if verbose_logging and not self._vis_ascii:
                self.get_logger().info(
                    f"Attempting IMU sensor re-initialization (attempt {self._recovery_attempt_count})...")

            if self._reinitialize_sensors(verbose=verbose_logging):
                self.node_healthy = True
                self._publish_health_status(True)

                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0  # Reset counter on success
                # Reset error count for next potential failure
                self._total_errors_this_session = 0
                # Reset magnetometer filter on recovery
                self.mag_filter_initialized = False
                # Reset unreachable log timer for next potential failure
                self._last_unreachable_log_time = 0.0
                # ASCII visualization will automatically resume showing sensor data

                if not self._vis_ascii:
                    self.get_logger().info(
                        f"IMU sensor re-initialization successful after {self._consecutive_io_errors} I2C errors")
            else:
                if verbose_logging and not self._vis_ascii:
                    self.get_logger().error(
                        f"IMU sensor re-initialization failed (attempt {self._recovery_attempt_count}), staying in retry mode")
                # Stay in retry mode and try again later

    def _init_ascii_vis(self):
        if self._vis_initialized:
            return
        try:
            # Force terminal reset on initialization
            sys.stdout.write('\x1b[0m')    # reset attributes
            sys.stdout.write('\x1b[?25l')  # hide cursor
            sys.stdout.write('\x1b[2J')    # clear screen
            sys.stdout.write('\x1b[H')     # home cursor
            sys.stdout.flush()
            self._vis_initialized = True
        except Exception as e:
            # If visualization initialization fails, disable it
            self._vis_ascii = False
            self._vis_initialized = False

    def _teardown_ascii_vis(self):
        if not self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')
            sys.stdout.write('\x1b[2J')
            sys.stdout.write('\x1b[H')
            sys.stdout.write('\x1b[?25h')
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False

    def _signed_bar(self, value: float, limit: float, width: int = 50) -> str:
        # Centered at zero; '-' for negative, '+' for positive; '|' marks zero
        width = max(10, width)
        mid = width // 2
        # clip
        val = max(-limit, min(limit, value))
        pos = int(round((val / limit) * mid))
        left = [' '] * mid
        right = [' '] * (width - mid - 1)
        if pos < 0:
            fill = mid + pos  # fill up to this index (exclusive)
            for i in range(fill, mid):
                left[i] = '-'
        elif pos > 0:
            for i in range(0, pos):
                if i < len(right):
                    right[i] = '+'
        # build
        return '[' + ''.join(left) + '|' + ''.join(right) + ']'

    def _render_compass(self, heading_deg: float, width: int = 40, height: int = 20) -> list:
        """Render compass heading as a point on a circle with N at top

        Args:
            heading_deg: Compass heading in degrees (0=North, 90=East, 180=South, 270=West)
            width: Width of the display in characters
            height: Height of the display in characters

        Returns:
            List of strings representing the compass display
        """
        # Ensure dimensions are reasonable
        width = max(20, width)
        height = max(10, height)

        cx = width // 2
        cy = height // 2
        radius = min(cx - 3, cy - 2)

        # Create grid
        grid = [[' ' for _ in range(width)] for _ in range(height)]

        # Draw circle (octagon approximation)
        for angle in range(0, 360, 5):
            rad = math.radians(angle)
            x = cx + int(round(radius * math.sin(rad)))
            y = cy - int(round(radius * math.cos(rad)))
            if 0 <= y < height and 0 <= x < width:
                grid[y][x] = '.'

        # Mark cardinal directions
        # North (0°) - top
        if cy - radius >= 0:
            grid[cy - radius][cx] = 'N'
        # East (90°) - right
        if cx + radius < width:
            grid[cy][cx + radius] = 'E'
        # South (180°) - bottom
        if cy + radius < height:
            grid[cy + radius][cx] = 'S'
        # West (270°) - left
        if cx - radius >= 0:
            grid[cy][cx - radius] = 'W'

        # Draw center
        grid[cy][cx] = '+'

        # Draw heading indicator (point on circle)
        theta = math.radians(heading_deg)
        hx = cx + int(round(radius * math.sin(theta)))
        hy = cy - int(round(radius * math.cos(theta)))
        if 0 <= hy < height and 0 <= hx < width:
            grid[hy][hx] = 'O'  # Heading marker

        # Draw line from center to heading point for better visibility
        # Simple line drawing using small steps
        steps = int(radius)
        for i in range(1, steps):
            frac = i / float(steps)
            lx = cx + int(round(frac * radius * math.sin(theta)))
            ly = cy - int(round(frac * radius * math.cos(theta)))
            if 0 <= ly < height and 0 <= lx < width:
                if grid[ly][lx] == ' ':
                    grid[ly][lx] = '*'

        # Convert grid to strings
        return [''.join(row) for row in grid]

    def _apply_compass_calibration(self, mx, my, mz):
        """Apply compass calibration if available (supports both minmax and ellipsoid methods)."""
        if not isinstance(self._compass_cal, dict):
            return mx, my, mz
        
        method = self._compass_cal.get('method')
        
        try:
            bias = self._compass_cal.get('bias_uT', [0, 0, 0])
            scale = self._compass_cal.get('scale_diag', [1, 1, 1])
            
            if method == 'ellipsoid' and 'rotation' in self._compass_cal:
                # Full ellipsoid calibration with rotation
                import numpy as np
                point = np.array([mx, my, mz])
                bias_arr = np.array(bias)
                rotation = np.array(self._compass_cal['rotation'])
                radii = np.array(self._compass_cal.get('radii', [1, 1, 1]))
                
                # Apply ellipsoid calibration
                calibrated = apply_ellipsoid_calibration(
                    point.reshape(1, 3), bias_arr, radii, rotation)
                
                return calibrated[0, 0], calibrated[0, 1], calibrated[0, 2]
                
            elif method in ('minmax', 'diag'):
                # Simple min-max calibration (bias + diagonal scale)
                bx, by, bz = bias
                sx, sy, sz = scale
                mx_cal = (mx - bx) * sx
                my_cal = (my - by) * sy
                mz_cal = (mz - bz) * sz
                return mx_cal, my_cal, mz_cal
            
            elif method == 'unity':
                # Unity/identity calibration - no transformation (for testing)
                return mx, my, mz
                
        except Exception as e:
            # If calibration fails, log once and return uncalibrated values
            if not hasattr(self, '_calib_error_logged'):
                self.get_logger().warn(f"Compass calibration application failed: {e}")
                self._calib_error_logged = True
        
        return mx, my, mz

    def _calculate_tilt_compensated_heading(self, mx, my, mz, ax, ay, az):
        """Compute tilt-compensated compass heading."""
        # Transform magnetometer readings to accelerometer coordinate frame
        # Accel frame: +x=starboard, +y=bow, +z=down (gravity vector)
        # Mag frame: +x=starboard, +y=stern, +z=down
        # Transform: mag_in_accel = (mx, -my, mz)
        mag_x_accel = mx
        mag_y_accel = -my  # stern -> bow
        mag_z_accel = mz  # both point down, same direction

        # Normalize gravity vector from accelerometer
        g_norm = math.sqrt(ax**2 + ay**2 + az**2)
        if g_norm < 0.1:  # Sanity check - avoid division by zero
            g_norm = 1.0

        # Normalized gravity vector (should point down when level, gz ≈ -1)
        gx = ax / g_norm
        gy = ay / g_norm
        gz = az / g_norm

        # Standard tilt compensation using pitch and roll
        # Calculate pitch and roll from accelerometer (gravity vector convention)
        # pitch = atan2(ay, sqrt(ax^2 + az^2))  (rotation about x-axis, starboard)
        # roll = atan2(-ax, az)  (rotation about y-axis, bow)
        # Note: az is negative when level, so formulas account for this
        pitch = math.atan2(gy, math.sqrt(gx**2 + gz**2))
        roll = math.atan2(-gx, gz)

        # Apply tilt compensation to magnetometer
        # Rotate magnetic field to compensate for pitch and roll
        # This gives us the magnetic field as if the sensor were level
        mag_x_comp = mag_x_accel * \
            math.cos(pitch) + mag_z_accel * math.sin(pitch)
        mag_y_comp = (mag_x_accel * math.sin(roll) * math.sin(pitch) +
                      mag_y_accel * math.cos(roll) -
                      mag_z_accel * math.sin(roll) * math.cos(pitch))

        # Calculate heading from compensated magnetometer values
        # In accel frame: x=starboard (east), y=bow (north)
        # Heading = atan2(east, north) = atan2(-x_comp, y_comp)
        # We use -x because east is negative starboard in compass convention
        heading_rad = math.atan2(-mag_x_comp, mag_y_comp)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360.0

        return heading_deg

    def timer_callback(self):
        # Handle pause/unpause transitions for IMU sleep management
        paused = self.pause_service.is_paused()
        if paused != self._prev_paused:
            if paused:
                self._enter_sleep_mode()
            else:
                # On unpause, fully reinitialize sensors (includes waking IMU)
                self._exit_pause_mode()
            self._prev_paused = paused

        # Skip processing while paused
        if paused:
            return

        current_time = time.time()

        # Check for recovery from I2C errors (do this even if no valid samples)
        if not self.node_healthy:
            self._check_io_recovery()

            # Log unreachable sensor status for normal operation (throttled to once per 60s)
            if not self._vis_ascii:
                time_since_last_log = current_time - self._last_unreachable_log_time
                # Log immediately on first occurrence, then throttle to 60s intervals
                if time_since_last_log >= 60.0 or self._last_unreachable_log_time == 0.0:
                    time_since_healthy = current_time - self._last_successful_read_time
                    self.get_logger().error(
                        f"IMU sensor unreachable for {time_since_healthy:.1f}s "
                        f"(recovery attempts: {self._recovery_attempt_count}, "
                        f"total errors: {self._total_errors_this_session})")
                    self._last_unreachable_log_time = current_time

            # Display recovery status in ASCII visualization
            if self._vis_ascii:
                try:
                    time_since_healthy = current_time - self._last_successful_read_time

                    sys.stdout.write('\x1b[2J')    # clear
                    sys.stdout.write('\x1b[H')     # home
                    sys.stdout.flush()

                    # Build recovery status display
                    term_width, term_height = get_terminal_size()

                    lines = [
                        "=" * term_width,
                        "=== IMU SENSOR RECOVERY IN PROGRESS ===".center(
                            term_width),
                        "=" * term_width,
                        "",
                        f"Status: ATTEMPTING RECOVERY (1Hz retry mode)",
                        f"Time since last valid read: {time_since_healthy:.1f}s",
                        f"Recovery attempts: {self._recovery_attempt_count}",
                        f"Total I2C errors: {self._total_errors_this_session}",
                        "",
                        "Waiting for sensor to reconnect...",
                        "",
                        "=" * term_width,
                        "",
                        "Ctrl-C to exit"
                    ]

                    for ln in lines:
                        sys.stdout.write(ln + '\n')
                    sys.stdout.flush()
                except Exception:
                    pass
            return  # Skip normal sensor processing when unhealthy

        try:
            ax_cnt, ay_cnt, az_cnt = self.icm.read_accel()
            gx_cnt, gy_cnt, gz_cnt = self.icm.read_gyro()

            # Convert to physical units
            # Note: Flipping z-axis to represent gravity vector (down) rather than reaction force (up)
            ax_g = ax_cnt / 16384.0
            ay_g = ay_cnt / 16384.0
            # Flip z to match gravity vector convention
            az_g = -(az_cnt / 16384.0)

            gx_dps = gx_cnt / 131.072
            gy_dps = gy_cnt / 131.072
            gz_dps = gz_cnt / 131.072

            # Read magnetometer data
            mx_uT, my_uT, mz_uT = self.icm.read_magnetometer()

            # Validate sensor data - check if all sensors return zeros (sensor not initialized properly)
            if (abs(ax_g) < 0.01 and abs(ay_g) < 0.01 and abs(az_g) < 0.01 and
                abs(gx_dps) < 0.1 and abs(gy_dps) < 0.1 and abs(gz_dps) < 0.1 and
                    abs(mx_uT) < 0.1 and abs(my_uT) < 0.1 and abs(mz_uT) < 0.1):
                # All sensors returning zeros - likely sensor not initialized
                if not self.node_healthy:
                    # Skip this reading and wait for proper initialization
                    return

            # Apply first-order IIR lowpass filter to magnetometer readings
            # Filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
            if not self.mag_filter_initialized:
                # Initialize filter with first reading
                self.mag_filtered_x = mx_uT
                self.mag_filtered_y = my_uT
                self.mag_filtered_z = mz_uT
                self.mag_filter_initialized = True
            else:
                # Apply IIR lowpass filter
                self.mag_filtered_x = self.mag_filter_alpha * mx_uT + \
                    (1.0 - self.mag_filter_alpha) * self.mag_filtered_x
                self.mag_filtered_y = self.mag_filter_alpha * my_uT + \
                    (1.0 - self.mag_filter_alpha) * self.mag_filtered_y
                self.mag_filtered_z = self.mag_filter_alpha * mz_uT + \
                    (1.0 - self.mag_filter_alpha) * self.mag_filtered_z

            # Use filtered values for further processing
            mx_uT = self.mag_filtered_x
            my_uT = self.mag_filtered_y
            mz_uT = self.mag_filtered_z

            # Update successful read time for recovery detection
            self._last_successful_read_time = current_time

            self._consecutive_io_errors = 0  # Reset error counter on success

            # If we were unhealthy but now have successful reads, recover to normal mode
            if not self.node_healthy:
                # Re-initialize sensors to ensure proper configuration after reconnection
                if not self._reinitialize_sensors():
                    # Don't mark as healthy if re-init fails
                    return

                self.node_healthy = True
                self._publish_health_status(True)
                self._switch_to_normal_mode()
                self._recovery_attempt_count = 0
                # Reset error count for next potential failure
                self._total_errors_this_session = 0
                # Reset magnetometer filter on recovery
                self.mag_filter_initialized = False
                # Reset unreachable log timer for next potential failure
                self._last_unreachable_log_time = 0.0
                # ASCII visualization will automatically resume showing sensor data

            # Centralized calibration and heading calculation
            mx_raw, my_raw, mz_raw = mx_uT, my_uT, mz_uT
            mx_cal, my_cal, mz_cal = self._apply_compass_calibration(
                mx_raw, my_raw, mz_raw)
            heading_deg = self._calculate_tilt_compensated_heading(
                mx_cal, my_cal, mz_cal, ax_g, ay_g, az_g)

            if self._vis_ascii:
                try:
                    # Values are pre-calculated, just render them
                    sys.stdout.write('\x1b[2J')    # clear
                    sys.stdout.write('\x1b[H')     # home
                    sys.stdout.flush()
                    # Nominal limits for bars
                    a_lim = 2.0   # g
                    g_lim = 500.0  # dps
                    m_lim = 100.0  # uT

                    # Get terminal size for dynamic sizing
                    term_width, term_height = get_terminal_size()
                    bar_width = term_width - 20  # Reserve 20 chars for labels

                    # Build sensor data lines
                    sensor_lines = [
                        "=== IMU Sensor Data (ICM-20948) ===",
                        "",
                        "Accelerometer:",
                        f"Ax {ax_g:+7.3f} g   " +
                        self._signed_bar(ax_g, a_lim, bar_width),
                        f"Ay {ay_g:+7.3f} g   " +
                        self._signed_bar(ay_g, a_lim, bar_width),
                        f"Az {az_g:+7.3f} g   " +
                        self._signed_bar(az_g, a_lim, bar_width),
                        "",
                        "Gyroscope:",
                        f"Gx {gx_dps:+7.1f} dps " +
                        self._signed_bar(gx_dps, g_lim, bar_width),
                        f"Gy {gy_dps:+7.1f} dps " +
                        self._signed_bar(gy_dps, g_lim, bar_width),
                        f"Gz {gz_dps:+7.1f} dps " +
                        self._signed_bar(gz_dps, g_lim, bar_width),
                        "",
                        "Magnetometer (Raw):",
                        f"Mx {mx_raw:+7.1f} uT  " +
                        self._signed_bar(mx_raw, m_lim, bar_width),
                        f"My {my_raw:+7.1f} uT  " +
                        self._signed_bar(my_raw, m_lim, bar_width),
                        f"Mz {mz_raw:+7.1f} uT  " +
                        self._signed_bar(mz_raw, m_lim, bar_width),
                        "",
                        "Magnetometer (Calibrated):",
                        f"Mx {mx_cal:+7.1f} uT  " +
                        self._signed_bar(mx_cal, m_lim, bar_width),
                        f"My {my_cal:+7.1f} uT  " +
                        self._signed_bar(my_cal, m_lim, bar_width),
                        f"Mz {mz_cal:+7.1f} uT  " +
                        self._signed_bar(mz_cal, m_lim, bar_width),
                        "",
                        f"=== Compass Heading: {heading_deg:6.1f}° (0°=N, 90°=E, 180°=S, 270°=W) ===",
                        ""
                    ]

                    # Calculate available height for compass (sensor data + footer takes ~20 lines)
                    sensor_lines_count = len(sensor_lines)
                    footer_lines_count = 2  # blank line + "Ctrl-C to exit"
                    available_height = term_height - sensor_lines_count - footer_lines_count

                    # Generate compass display to fit available height
                    compass_width = term_width
                    # Minimum 5 lines for compass
                    compass_height = max(5, available_height)
                    compass_lines = self._render_compass(
                        heading_deg, width=compass_width, height=compass_height)

                    # Combine all lines
                    lines = sensor_lines + compass_lines + \
                        ["", "Ctrl-C to exit"]

                    for ln in lines:
                        sys.stdout.write(ln + '\n')
                    sys.stdout.flush()
                except Exception:
                    pass

            # Publish in physical units
            self.pub_accel.publish(Vector3(x=ax_g, y=ay_g, z=az_g))
            self.pub_gyro.publish(Vector3(x=gx_dps, y=gy_dps, z=gz_dps))
            self.pub_magnetometer.publish(
                Vector3(x=mx_cal, y=my_cal, z=mz_cal))
            self.pub_compass.publish(Float64(data=heading_deg))

        except Exception as e:
            self._handle_io_error(e)

    def _enter_sleep_mode(self) -> None:
        """Place IMU into sleep mode when node is paused."""
        try:
            self.icm.set_sleep_mode(True)
            self.get_logger().info("IMU placed into SLEEP mode (PWR_MGMT_1[6]=1)")
        except Exception as e:
            self.get_logger().warn(f"Failed to set IMU sleep mode: {e}")

    def _exit_pause_mode(self) -> None:
        """Reinitialize sensors on unpause (wakes IMU and restores config)."""
        try:
            # Use existing recovery path for consistent setup
            if self._reinitialize_sensors():
                # Reset filter state after reinit
                self.mag_filter_initialized = False
        except Exception as e:
            self.get_logger().error(f"Failed to reinitialize IMU after unpause: {e}")

    def _quiet_shutdown(self) -> None:
        # Publish health status as failed on shutdown (only if ROS context is still valid)
        try:
            if rclpy.ok():
                self._publish_health_status(False)
        except Exception:
            pass  # Suppress any publishing errors during shutdown

        # Teardown ASCII view
        self._teardown_ascii_vis()

        # Attempt to close I2C bus if supported
        try:
            if hasattr(self, 'bus') and hasattr(self.bus, 'close'):
                self.bus.close()
        except Exception:
            pass


def main(args=None):
    # Parse arguments BEFORE importing ROS2 to support standalone modes
    parser = argparse.ArgumentParser(description='IMU Sensor Node')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging for sensor values')
    parser.add_argument('--calib_compass', action='store_true',
                        help='Collect magnetometer samples and save calibration')
    parser.add_argument('--plot_calib', action='store_true',
                        help='Plot the most recent calibration data from /tmp')
    parsed_args = parser.parse_args(args=args)

    # If plot calibration is requested, load and plot existing calibration (optionally recompute)
    # This mode doesn't require ROS2
    if parsed_args.plot_calib:
        import glob
        # Find calibration samples in both /tmp and nodes/ directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        samples_files = glob.glob('/tmp/imu_calib_samples_*.json')
        samples_files += glob.glob(os.path.join(script_dir, 'imu_calib_samples_*.json'))
        
        if not samples_files:
            print("Error: No calibration data found in /tmp or nodes/")
            print("Run with --calib_compass first to generate calibration data")
            return

        latest_file = max(samples_files, key=os.path.getmtime)
        print(f"Loading calibration data from: {latest_file}")

        try:
            with open(latest_file, 'r') as f:
                data = json.load(f)

            timestamped_samples = data['samples']
            timestamp = data.get('timestamp', 'unknown')
            
            # Extract just the (mx, my, mz) values for calibration
            samples = [[mx, my, mz] for _, mx, my, mz in timestamped_samples]

            # Check if there's existing calibration
            calib = data.get('calibration')
            should_save = False  # Track if we need to offer to save
            
            if calib:
                # Show existing calibration
                print(f"\nExisting calibration found:")
                method = calib.get('method', 'unknown')
                bias = calib.get('bias_uT', [0, 0, 0])
                scale = calib.get('scale_diag', [1, 1, 1])
                print(f"  Method: {method}")
                print(f"  Bias (µT): [{bias[0]:.2f}, {bias[1]:.2f}, {bias[2]:.2f}]")
                print(f"  Scale: [{scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f}]")
                if 'radii' in calib:
                    radii = calib['radii']
                    print(f"  Radii (µT): [{radii[0]:.2f}, {radii[1]:.2f}, {radii[2]:.2f}]")
                print(f"  Samples: {len(samples)}")
                
                # Ask if user wants to recompute
                if _prompt_yes_no("\nRecompute calibration with different method?", default_yes=False):
                    calib = compute_calibration_from_samples(samples)
                    if calib is None:
                        print("Error: Calibration computation failed, using existing calibration")
                        calib = data.get('calibration')
                    else:
                        should_save = True  # New calibration computed
            else:
                # No existing calibration, compute it
                print(f"\nNo existing calibration found (samples: {len(samples)})")
                if len(samples) < 100:
                    print(f"Warning: only {len(samples)} samples - calibration may be poor (recommend 100+)")
                
                calib = compute_calibration_from_samples(samples)
                if calib is None:
                    print("Error: Calibration computation failed")
                    return
                should_save = True  # New calibration computed

            # Generate and display plots
            try:
                # Standard time-series plot
                plot_file = plot_magnetometer_calibration(
                    timestamped_samples, calib, timestamp)
                display_plot(plot_file)
                
                # 3D visualization plot (interactive if DISPLAY available)
                has_display = bool(os.environ.get('DISPLAY'))
                plot_3d_file = plot_magnetometer_3d(
                    samples, calib, timestamp, output_dir='/tmp', interactive=has_display)
                if plot_3d_file:
                    # Skip xdg-open if we already showed it interactively
                    display_plot(plot_3d_file, skip_open=has_display)

            except ImportError:
                print("Error: matplotlib is required for plotting")
                print("Install with: pip3 install matplotlib")
            except Exception as e:
                print(f"Error generating plot: {e}")
                import traceback
                traceback.print_exc()
            
            # Update samples file if calibration was recomputed
            if should_save:
                data['calibration'] = calib
                with open(latest_file, 'w') as f:
                    json.dump(data, f, indent=2)
                print(f"\nUpdated {latest_file} with new calibration")
                
                # Offer to save calibration to main calibration file
                calib_file = os.path.join(script_dir, 'invensense-20948-compass-calibration.json')
                if _prompt_yes_no(f"\nSave this calibration to nodes/{os.path.basename(calib_file)}?", default_yes=True):
                    _backup_existing_calibration(calib_file)
                    with open(calib_file, 'w') as f:
                        json.dump(calib, f, indent=2)
                    print(f"Saved compass calibration to nodes/{os.path.basename(calib_file)}")
                else:
                    print("Calibration not saved to main calibration file")
                
        except Exception as e:
            print(f"Error loading calibration data: {e}")
            import traceback
            traceback.print_exc()

        return

    # If only calibration is requested, run a standalone collector
    if parsed_args.calib_compass:
        try:
            try:
                from smbus2 import SMBus  # type: ignore
            except Exception:
                from smbus import SMBus  # type: ignore
            bus = SMBus(0)
            icm = ICM20948(bus, 0x69)
            icm.initialize()
            AK_ADDR = 0x0C
            bus.write_byte_data(AK_ADDR, 0x32, 0x01)
            time.sleep(0.05)
            bus.write_byte_data(AK_ADDR, 0x31, 0x08)
            time.sleep(0.01)

            print(
                "Rotate the device slowly through all orientations (figure-8). Press Ctrl-C to finish and save.")
            samples = []  # List of (mx_uT, my_uT, mz_uT) tuples
            # List of (timestamp, mx_uT, my_uT, mz_uT) tuples
            timestamped_samples = []
            start_time = time.time()
            # Tracking minima and maxima
            minx = miny = minz = float('inf')
            maxx = maxy = maxz = float('-inf')
            
            # I2C error tracking for robust operation
            consecutive_errors = 0
            total_errors = 0
            last_error_time = 0.0
            last_successful_read = time.time()
            last_recovery_attempt = 0.0
            recovery_attempts = 0
            sensor_healthy = True
            
            # Terminal setup
            try:
                sys.stdout.write('\x1b[?25l')  # hide cursor
                sys.stdout.write('\x1b[2J')    # clear
                sys.stdout.flush()
                visual_ok = True
            except Exception:
                visual_ok = False

            def render_bars():
                # Use 100 uT full-scale for visualization
                fs = 100.0
                spans = [maxx - minx if minx != float('inf') else 0.0,
                         maxy - miny if miny != float('inf') else 0.0,
                         maxz - minz if minz != float('inf') else 0.0]
                lines = []
                # Get current terminal width for resize support
                term_width, _ = get_terminal_size()
                bar_width = term_width - 30  # Reserve space for labels
                for name, mn, mx, span in [("X", minx, maxx, spans[0]), ("Y", miny, maxy, spans[1]), ("Z", minz, maxz, spans[2])]:
                    frac = max(0.0, min(1.0, span / fs))
                    fill = int(round(frac * bar_width))
                    bar = '#' * fill + '-' * (bar_width - fill)
                    lines.append(
                        f"Mag {name} [{mn:+6.1f} .. {mx:+6.1f}] |{bar}| span={span:5.1f} uT")
                
                # Add status line with error info
                status_line = f"Samples: {len(samples)}"
                if not sensor_healthy:
                    time_since_healthy = time.time() - last_successful_read
                    status_line += f" | SENSOR ERROR (recovery attempts: {recovery_attempts}, errors: {total_errors}, time: {time_since_healthy:.1f}s)"
                elif total_errors > 0:
                    status_line += f" | Recovered (total errors: {total_errors})"
                
                try:
                    sys.stdout.write('\x1b[H')  # home
                    for ln in [status_line, *lines, "Ctrl-C to finish..."]:
                        sys.stdout.write(ln + '\n')
                    sys.stdout.flush()
                except Exception:
                    pass

            try:
                while True:
                    current_time = time.time()
                    
                    # Check if we need to attempt sensor recovery
                    if not sensor_healthy:
                        time_since_last_recovery = current_time - last_recovery_attempt
                        if time_since_last_recovery > 3.0:  # Try recovery every 3 seconds
                            recovery_attempts += 1
                            last_recovery_attempt = current_time
                            try:
                                # Reinitialize IMU and magnetometer
                                icm.initialize()
                                bus.write_byte_data(AK_ADDR, 0x32, 0x01)
                                time.sleep(0.05)
                                bus.write_byte_data(AK_ADDR, 0x31, 0x08)
                                time.sleep(0.01)
                                # Don't immediately mark as healthy - wait for successful read
                            except Exception as e:
                                # Recovery failed, will try again later
                                if visual_ok:
                                    render_bars()
                    
                    try:
                        st1 = bus.read_byte_data(0x0C, 0x10)
                        if st1 & 0x01:
                            mb = list(bus.read_i2c_block_data(0x0C, 0x11, 8))
                            mx_cnt = struct.unpack('<h', bytes(mb[0:2]))[0]
                            my_cnt = struct.unpack('<h', bytes(mb[2:4]))[0]
                            mz_cnt = struct.unpack('<h', bytes(mb[4:6]))[0]
                            mx_uT = mx_cnt * 0.15
                            my_uT = my_cnt * 0.15
                            mz_uT = mz_cnt * 0.15
                            
                            # Successful read - reset error tracking
                            if not sensor_healthy:
                                sensor_healthy = True
                                consecutive_errors = 0
                            
                            last_successful_read = current_time
                            sample_time = current_time - start_time
                            samples.append((mx_uT, my_uT, mz_uT))
                            timestamped_samples.append(
                                (sample_time, mx_uT, my_uT, mz_uT))
                            if mx_uT < minx:
                                minx = mx_uT
                            if my_uT < miny:
                                miny = my_uT
                            if mz_uT < minz:
                                minz = mz_uT
                            if mx_uT > maxx:
                                maxx = mx_uT
                            if my_uT > maxy:
                                maxy = my_uT
                            if mz_uT > maxz:
                                maxz = mz_uT
                            if visual_ok:
                                render_bars()
                    except Exception as e:
                        # Handle I2C errors
                        consecutive_errors += 1
                        total_errors += 1
                        last_error_time = current_time
                        
                        # Mark as unhealthy after first error
                        if sensor_healthy:
                            sensor_healthy = False
                        
                        # Update display to show error
                        if visual_ok:
                            render_bars()
                    
                    time.sleep(0.02)
            except KeyboardInterrupt:
                pass
            finally:
                if visual_ok:
                    try:
                        sys.stdout.write('\x1b[0m')
                        sys.stdout.write('\x1b[2J')
                        sys.stdout.write('\x1b[H')
                        sys.stdout.write('\x1b[?25h')
                        sys.stdout.flush()
                    except Exception:
                        pass
            
            # Print I2C error statistics if any errors occurred
            if total_errors > 0:
                print(f"\nI2C Error Statistics:")
                print(f"  Total errors: {total_errors}")
                print(f"  Recovery attempts: {recovery_attempts}")
                print(f"  Final sensor status: {'HEALTHY' if sensor_healthy else 'UNHEALTHY'}")
                if not sensor_healthy:
                    print(f"  WARNING: Sensor still unhealthy - calibration may be incomplete")

            if len(samples) < 100:
                print(f"Warning: only {len(samples)} samples - calibration may be poor (recommend 100+)")

            # Save timestamped samples to /tmp 
            calib_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            samples_file = f'/tmp/imu_calib_samples_{calib_timestamp}.json'
            samples_data = {
                'timestamp': calib_timestamp,
                'samples': [[t, mx, my, mz] for t, mx, my, mz in timestamped_samples]
            }
            with open(samples_file, 'w') as f:
                json.dump(samples_data, f, indent=2)
            print(f"\nSaved calibration samples to: {samples_file}")

            # Compute calibration from samples
            calib = compute_calibration_from_samples(samples)
            
            if calib is None:
                print("Error: Calibration computation failed")
                return

            # Generate and display plots
            try:
                # Standard time-series plot
                plot_file = plot_magnetometer_calibration(
                    timestamped_samples, calib, calib_timestamp)
                display_plot(plot_file)
                
                # 3D visualization plot (interactive if DISPLAY available)
                has_display = bool(os.environ.get('DISPLAY'))
                plot_3d_file = plot_magnetometer_3d(
                    samples, calib, calib_timestamp, output_dir='/tmp', interactive=has_display)
                if plot_3d_file:
                    # Skip xdg-open if we already showed it interactively
                    display_plot(plot_3d_file, skip_open=has_display)
            except ImportError:
                print("Warning: matplotlib not available, skipping plot generation")
            except Exception as e:
                print(f"Warning: Failed to generate plot: {e}")
            
            # Update samples file with calibration
            samples_data['calibration'] = calib
            with open(samples_file, 'w') as f:
                json.dump(samples_data, f, indent=2)

            # Define calibration file path in nodes/ directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            calib_file = os.path.join(
                script_dir, 'invensense-20948-compass-calibration.json')

            if _prompt_yes_no(f"\nSave calibration to nodes/{os.path.basename(calib_file)}?", default_yes=True):
                # Backup existing calibration before saving new one
                _backup_existing_calibration(calib_file)
                with open(calib_file, 'w') as f:
                    json.dump(calib, f, indent=2)
                print(f"Saved compass calibration to nodes/{os.path.basename(calib_file)}")
                
                # Move samples file from /tmp to nodes/ for persistent storage
                persistent_samples_file = os.path.join(
                    script_dir, f'imu_calib_samples_{calib_timestamp}.json')
                shutil.move(samples_file, persistent_samples_file)
                print(f"Moved calibration samples to: {persistent_samples_file}")
            else:
                print("Calibration not saved to main calibration file")
        except Exception as e:
            print(f"Calibration failed: {e}")
        finally:
            try:
                if 'bus' in locals() and hasattr(bus, 'close'):
                    bus.close()
            except Exception:
                pass
        return

    # ROS2 mode - modules already imported at top if not in standalone mode
    rclpy.init(args=args)

    imu_node = None
    try:
        imu_node = ImuNode(debug=parsed_args.debug)
        rclpy.spin(imu_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # This handles Ctrl+C or external shutdown requests gracefully.
        if imu_node:
            try:
                imu_node._quiet_shutdown()
            except Exception:
                pass  # Suppress any errors during quiet shutdown
    except Exception as e:
        # Handle any other unexpected exceptions
        if imu_node:
            imu_node.get_logger().error(f"Unexpected error: {e}")
        else:
            print(f"Error before node creation: {e}")
    finally:
        # Clean shutdown
        if imu_node:
            try:
                imu_node.destroy_node()
            except Exception:
                pass  # Suppress any errors during node destruction

        try:
            # Check if ROS is still initialized before shutdown
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Suppress ROS shutdown errors (like "already called" errors)
            pass


if __name__ == '__main__':
    main()
