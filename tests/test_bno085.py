#!/usr/bin/env python3
"""
BNO085 I2C Communication Test Script

This script tests basic I2C communication with the BNO085 sensor
and implements the SHTP protocol for debugging purposes.

The BNO08x speaks SHTP over I2C: plain read/write transactions, not SMBus
"register + read block" (read_i2c_block_data). Using SMBus block APIs makes a
probe at 0x4a look broken even when i2cdetect / ai2c see the device — same bus,
wrong protocol.

Pass criteria:
  • I2C + SHTP soft reset + Product ID (hardware path used by ``bno08x_driver``).
  • Unless ``--skip-ros``: run ``ros2 topic hz /imu`` (sources Humble) and require a
    non-zero observed rate when the IMU stack should be running. If ROS is unavailable,
    streaming is skipped with a warning and the test still passes on Product ID alone.

Usage (from repo root):
    python3 tests/test_bno085.py [--bus 0] [--address 0x4a] [--debug]
    python3 tests/test_bno085.py --skip-ros              # bench: chip only, no ros2
    python3 tests/test_bno085.py [--ros-hz-timeout 20]

Bus number matches ai2c "Bus 0" (i2cdetect -y 0) and bno08x_driver_argo.yaml
i2c.bus /dev/i2c-0 on Orange Pi Zero 2W.
"""

import argparse
import fcntl
import os
import re
import shlex
import subprocess
import sys
import time
import smbus2
from smbus2 import i2c_msg

# linux/i2c-dev.h — select slave before POSIX read() on /dev/i2c-* (matches vendor I2CInterface)
I2C_SLAVE = 0x0703

class BNO085Test:
    """Simple BNO085 test class for debugging I2C communication."""
    
    def __init__(self, bus_num=0, address=0x4a, debug=False):
        self.bus_num = bus_num
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.debug = debug
        self._sequence_number = 0
        
        # SHTP constants
        self.SHTP_HEADER_SIZE = 4
        # Must accept full SHTP frames (e.g. advertisements ~272B per sh2_hal.h)
        self.MAX_PACKET_SIZE = 384  # SH2_HAL_MAX_TRANSFER_IN in nodes/vendor/.../sh2/sh2_hal.h
        
        # SHTP Channel numbers (from BNO080 datasheet)
        self.CHANNEL_COMMAND = 0x00      # SHTP command channel
        self.CHANNEL_EXECUTABLE = 0x01   # Executable channel
        self.CHANNEL_CONTROL = 0x02      # Sensor hub control channel (SH-2)
        self.CHANNEL_REPORTS = 0x03      # Input sensor reports
        
        # SH-2 Report IDs (from BNO080 datasheet)
        self.REPORT_PRODUCT_ID_REQUEST = 0xF9     # Product ID Request
        self.REPORT_PRODUCT_ID_RESPONSE = 0xF8    # Product ID Response
    
    def debug_print(self, message):
        """Print debug message if debug mode is enabled."""
        if self.debug:
            print(f"BNO085 DEBUG: {message}")

    def _i2c_write_raw(self, buf):
        """Single I2C write transaction (SHTP); matches bno08x I2CInterface::write."""
        w = i2c_msg.write(self.addr, bytes(buf))
        self.bus.i2c_rdwr(w)

    def _ensure_slave(self):
        fcntl.ioctl(self.bus.fd, I2C_SLAVE, self.addr)

    def _i2c_read_raw(self, n):
        """Raw I2C read(s) via SMBus i2c_msg — OK for short probes; full SHTP frames use _read_packet."""
        if n <= 0:
            return []
        self._ensure_slave()
        out = []
        remaining = n
        while remaining > 0:
            chunk = min(32, remaining)
            r = i2c_msg.read(self.addr, chunk)
            self.bus.i2c_rdwr(r)
            out.extend(list(r))
            remaining -= chunk
        return out

    def _shtp_soft_reset(self):
        """Same 5-byte soft reset packet as vendor i2c_interface.hpp."""
        self._i2c_write_raw([5, 0, 1, 0, 1])
        time.sleep(1.0)

    def test_basic_i2c(self):
        """Test raw I2C + SHTP path used by the ROS driver (not SMBus register reads)."""
        print(f"I2C bus {self.bus_num} → /dev/i2c-{self.bus_num}")
        print(
            "  Align with: ai2c \"Bus 0\" = i2cdetect -y 0; "
            "nodes/vendor/bno08x_driver_argo.yaml → i2c.bus /dev/i2c-0"
        )
        print(f"Probing BNO085 at 0x{self.addr:02x} (SHTP soft reset + drain first packet)...")

        try:
            self._shtp_soft_reset()
            pkt = self._read_packet()
            if not pkt:
                print("❌ No SHTP packet after reset")
                return False
            print(
                f"✅ First SHTP packet: length={pkt['length']} "
                f"channel={pkt['channel']} seq={pkt['sequence']}"
            )
            return True
        except OSError as e:
            print(f"❌ Basic I2C failed: {e}")
            return False
    
    def _read_packet(self):
        """Read one SHTP packet — must match nodes/vendor/bno08x_driver/.../i2c_interface.hpp::read (POSIX read + chunk/skip)."""
        try:
            self._ensure_slave()
            hdr = os.read(self.bus.fd, self.SHTP_HEADER_SIZE)
            if len(hdr) != self.SHTP_HEADER_SIZE:
                return None
            header = list(hdr)

            packet_length = ((header[1] << 8) | header[0]) & 0x7FFF
            channel = header[2]
            sequence_number = header[3]

            self.debug_print(f"Header: length={packet_length}, channel={channel}, seq={sequence_number}, raw={header}")

            if packet_length > self.MAX_PACKET_SIZE:
                self.debug_print(f"Invalid packet length: {packet_length}")
                return None

            if packet_length == 0:
                self.debug_print("Zero-length packet received")
                return {
                    'length': 0,
                    'sequence': sequence_number,
                    'channel': channel,
                    'payload': []
                }

            # After SHTP header, one read for (packet_length - 4) bytes (Wire.requestFrom style)
            remainder = packet_length - self.SHTP_HEADER_SIZE
            buf = bytearray(header)
            if remainder > 0:
                chunk = os.read(self.bus.fd, remainder)
                if len(chunk) != remainder:
                    self.debug_print(f"short read: wanted {remainder}, got {len(chunk)}")
                    return None
                buf.extend(chunk)

            payload = list(buf[self.SHTP_HEADER_SIZE :])
            self.debug_print(f"Payload len={len(payload)}")
            
            return {
                'length': packet_length,
                'sequence': sequence_number,
                'channel': channel,
                'payload': payload
            }
        except OSError as e:
            self.debug_print(f"Error reading packet: {e}")
            return None
    
    def _write_packet(self, channel, data):
        """Write a SHTP packet to the BNO085."""
        try:
            # SHTP header format: Length LSB, Length MSB, Channel, SeqNum
            packet_length = len(data) + self.SHTP_HEADER_SIZE
            header = [
                packet_length & 0xFF,           # Length LSB (little-endian)
                (packet_length >> 8) & 0xFF,    # Length MSB
                channel & 0xFF,                 # Channel
                self._sequence_number & 0xFF    # Sequence number
            ]
            
            self.debug_print(f"Writing packet: channel={channel}, length={packet_length}, seq={self._sequence_number}, data={data}")

            frame = bytes(header + list(data))
            # Chunk writes to 32 bytes like vendor I2CInterface::write
            for i in range(0, len(frame), 32):
                self._i2c_write_raw(frame[i : i + 32])
            
            self._sequence_number = (self._sequence_number + 1) % 256
            return True
        except Exception as e:
            self.debug_print(f"Error writing packet: {e}")
            return False
    
    def get_product_id(self):
        """Get BNO085 product ID and version information."""
        print("Requesting Product ID...")
        
        try:
            # Send Product ID request (Report ID 0xF9)
            self._write_packet(self.CHANNEL_CONTROL, [
                self.REPORT_PRODUCT_ID_REQUEST,  # Report ID
                0x00  # Reserved
            ])
            
            # Wait for response (hub may need time after control-channel write)
            time.sleep(0.25)
            
            # Drain SHTP; Product ID (0xF8) is often embedded in ch.0 advertisement, not a bare ch.2 frame.
            for _ in range(40):
                packet = self._read_packet()
                if not packet:
                    continue
                pp = packet['payload']
                head = pp[:24] if len(pp) > 24 else pp
                self.debug_print(f"Received packet: length={packet['length']}, channel={packet['channel']}, payload[:24]={head}")
                pl = packet['payload']
                off = next((i for i, b in enumerate(pl) if b == self.REPORT_PRODUCT_ID_RESPONSE), None)
                if off is None or len(pl) - off < 16:
                    continue
                pl = pl[off:]

                reset_cause = pl[1]
                sw_major = pl[2]
                sw_minor = pl[3]
                sw_part = (pl[7] << 24) | (pl[6] << 16) | (pl[5] << 8) | pl[4]
                sw_build = (pl[11] << 24) | (pl[10] << 16) | (pl[9] << 8) | pl[8]
                sw_patch = (pl[13] << 8) | pl[12]

                version_info = {
                    'reset_cause': reset_cause,
                    'sw_major': sw_major,
                    'sw_minor': sw_minor,
                    'sw_patch': sw_patch,
                    'sw_part': sw_part,
                    'sw_build': sw_build
                }

                print(f"✅ Product ID Response:")
                print(f"   Reset Cause: {reset_cause}")
                print(f"   Version: {sw_major}.{sw_minor}.{sw_patch}")
                print(f"   Part Number: 0x{sw_part:08X}")
                print(f"   Build Number: {sw_build}")
                return version_info
                        
            print("❌ No Product ID response received")
            return None
            
        except Exception as e:
            print(f"❌ Error getting Product ID: {e}")
            return None

    def run_full_test(self, skip_ros_verification=False, ros_hz_seconds=14):
        """Run complete BNO085 test sequence."""
        print("=" * 60)
        print("BNO085 I2C Communication Test")
        print(f"I2C bus: {self.bus_num} (/dev/i2c-{self.bus_num})")
        print("=" * 60)
        
        # Test 1: Basic I2C communication
        if not self.test_basic_i2c():
            print("❌ Basic I2C test failed - cannot proceed")
            return False
        
        print()
        
        # Test 2: Get Product ID
        version_info = self.get_product_id()
        if not version_info:
            print("❌ Product ID test failed - cannot proceed")
            return False
        
        print()
        ros_ok = None
        ros_detail = ""

        if skip_ros_verification:
            print(
                "ROS verification skipped (--skip-ros).\n"
                "When the stack runs, confirm streaming with:\n"
                "  ros2 topic hz /imu\n"
                "  ros2 topic echo /imu_health --once"
            )
            ros_ok = True
        else:
            ros_ok, ros_detail = verify_ros_imu_streaming(seconds=ros_hz_seconds)

        print()
        print("=" * 60)
        print("Test Summary:")
        print("=" * 60)

        print("✅ BNO085 detected and responsive (Product ID)")
        print(
            f"   Version: {version_info['sw_major']}."
            f"{version_info['sw_minor']}.{version_info['sw_patch']}"
        )

        if not skip_ros_verification:
            if ros_ok is True:
                print(f"✅ ROS IMU streaming: {ros_detail}")
            elif ros_ok is False:
                print(f"❌ ROS IMU streaming: {ros_detail}")
            else:
                print(f"⚠️ ROS streaming not verified (I2C still OK): {ros_detail}")

        overall = version_info is not None
        if overall and not skip_ros_verification and ros_ok is False:
            overall = False
        return overall


def verify_ros_imu_streaming(seconds=14):
    """
    Prefer `ros2 topic hz /imu` — more reliable here than `echo --once` (type introspection timing).
    Returns:
      (True, msg) — /imu rate observed.
      (False, msg) — ros2 ran but no /imu traffic (stack down or wrong domain).
      (None, msg) — cannot run ros2; caller treats as pass-with-warning for bare hardware checks.
    """
    humble = '/opt/ros/humble/setup.bash'
    if not os.path.isfile(humble):
        return None, 'ROS Humble setup.bash missing; skipping streaming check.'
    shell = (
        f'source {shlex.quote(humble)} && '
        'command -v ros2 >/dev/null 2>&1 || exit 127; '
        f'timeout {int(seconds)} ros2 topic hz /imu'
    )
    proc = subprocess.run(
        ['/bin/bash', '-c', shell],
        capture_output=True,
        text=True,
        timeout=max(seconds + 4, 8),
    )
    out = (proc.stdout or '') + (proc.stderr or '')
    if proc.returncode == 127 or 'command not found' in out.lower():
        return None, 'ros2 not on PATH after sourcing Humble — skipping.'
    rate_m = re.search(r'average\s+rate:\s*([0-9.]+)', out, re.I)
    if rate_m:
        return True, f'/imu publishing (~{rate_m.group(1)} Hz observed in sample window)'
    warn_m = re.search(r'does\s+not\s+appear\s+to\s+be\s+published', out, re.I)
    missing_m = re.search(r'waiting\s+for\s+at\s+least\s+one\s+messaging\s+clients', out, re.I)
    hint = (
        'Start IMU stack: sudo systemctl start argo_bno085.service '
        '(then: ros2 topic hz /imu).'
    )
    if warn_m or missing_m:
        return False, f'No /imu traffic detected. {hint}'
    # timeout 124 sometimes with no hz line yet
    if proc.returncode in (124, 143):
        return False, f'No /imu rate observed within {seconds}s. {hint}'
    return False, f'Could not confirm /imu (ros2 exited {proc.returncode}). {hint}'


def main():
    parser = argparse.ArgumentParser(description='Test BNO085 I2C communication')
    parser.add_argument(
        '--bus', type=int, default=0,
        help='Linux I2C bus number (default: 0). Same as ai2c "Bus 0" / i2cdetect -y 0',
    )
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x4a,
                       help='BNO085 I2C address (default: 0x4a)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output')
    parser.add_argument(
        '--skip-ros',
        action='store_true',
        help='Only verify I2C + Product ID (no ros2 topic hz /imu)',
    )
    parser.add_argument(
        '--ros-hz-timeout',
        type=int,
        default=14,
        metavar='SEC',
        help='Seconds for ros2 topic hz /imu sample window (default: 14)',
    )

    args = parser.parse_args()

    tester = BNO085Test(bus_num=args.bus, address=args.address, debug=args.debug)

    success = tester.run_full_test(
        skip_ros_verification=args.skip_ros,
        ros_hz_seconds=max(8, args.ros_hz_timeout),
    )
    
    if success:
        print("\n🎉 BNO085 test completed successfully!")
        sys.exit(0)
    else:
        print("\n💥 BNO085 test failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
