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
  • I2C + SHTP soft reset + Product ID (hardware path used by ``bno08x_driver``):
    reads follow ``i2c_interface.hpp::read`` chunking; payloads are reassembled with
    ``shtp.c``-style ``rxAssemble`` (continuations, sequence).
  • Or ``--skip-product-id``: skip flaky F9/Product ID; send ``SET_FEATURE`` (0xFD) for
    calibrated accelerometer (same as ``BNO08x::enable_report``) and require plausible
    ``SH2_ACCELEROMETER`` (0x01) reports on SHTP channel 3.
  • Unless ``--skip-ros``: run ``ros2 topic hz /imu`` (sources Humble) and require a
    non-zero observed rate when the IMU stack should be running. If ROS is unavailable,
    streaming is skipped with a warning and the test still passes on Product ID alone.

Usage (from repo root):
    python3 tests/test_bno085.py [--bus 0] [--address 0x4a] [--debug]
    python3 tests/test_bno085.py --skip-ros              # bench: chip only, no ros2
    python3 tests/test_bno085.py [--ros-hz-timeout 20]
    python3 tests/test_bno085.py --expect-part 0x0098A6B4   # optional BOM lock
    python3 tests/test_bno085.py --skip-ros --skip-product-id   # bench: live accel only

Bus number matches ai2c "Bus 0" (i2cdetect -y 0) and bno085_i2c_argo.yaml
i2c.bus /dev/i2c-0 on Orange Pi Zero 2W.
"""

import argparse
from collections import Counter
import errno
import fcntl
import math
import os
import re
import struct
import shlex
import subprocess
import sys
import time
import smbus2
from smbus2 import i2c_msg
from typing import List, Optional, Tuple

# Match include/sh2/shtp.c (SH2_MAX_CHANS)
SHTP_HDR_LEN = 4
SH2_MAX_CHANS = 8

# linux/i2c-dev.h — select slave before POSIX read() on /dev/i2c-* (matches vendor I2CInterface)
I2C_SLAVE = 0x0703

class BNO085Test:
    """Simple BNO085 test class for debugging I2C communication."""
    
    def __init__(
        self,
        bus_num=0,
        address=0x4a,
        debug=False,
        expect_part: Optional[int] = None,
    ):
        self.bus_num = bus_num
        self.bus = smbus2.SMBus(bus_num)
        # smbus2 can leave /dev/i2c-* in non-blocking mode; vendor I2CInterface uses
        # blocking read(). Without this, os.read() may raise OSError errno 11 (EAGAIN).
        _fl = fcntl.fcntl(self.bus.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.bus.fd, fcntl.F_SETFL, _fl & ~os.O_NONBLOCK)
        self.addr = address
        self.debug = debug
        # If set, Product ID must match this 32-bit part (e.g. 0x0098A6B4 for Argo BOM).
        self.expect_part = expect_part
        self._sequence_number = 0
        
        # SHTP constants
        self.SHTP_HEADER_SIZE = SHTP_HDR_LEN
        # Must accept full SHTP frames (e.g. advertisements ~272B per sh2_hal.h)
        self.MAX_PACKET_SIZE = 384  # SH2_HAL_MAX_TRANSFER_IN in nodes/vendor/.../sh2/sh2_hal.h

        # rxAssemble state (include/sh2/shtp.c rxAssemble)
        self._rx_init_state()

        # SHTP Channel numbers (from BNO080 datasheet)
        self.CHANNEL_COMMAND = 0x00      # SHTP command channel
        self.CHANNEL_EXECUTABLE = 0x01   # Executable channel
        self.CHANNEL_CONTROL = 0x02      # Sensor hub control channel (SH-2)
        self.CHANNEL_REPORTS = 0x03      # Input sensor reports
        
        # SH-2 Report IDs (from BNO080 datasheet)
        self.REPORT_PRODUCT_ID_REQUEST = 0xF9     # Product ID Request
        self.REPORT_PRODUCT_ID_RESPONSE = 0xF8    # Product ID Response
        # sh2.c SENSORHUB_SET_FEATURE_CMD / sh2.h SH2_ACCELEROMETER
        self.REPORT_SET_FEATURE_CMD = 0xFD
        self.SH2_ACCELEROMETER = 0x01
        # decodeAccelerometer in sh2_SensorValue.c uses Q8 fixed-point (SCALE_Q(8)).
        self._accel_report_min_bytes = 10
    
    def debug_print(self, message):
        """Print debug message if debug mode is enabled."""
        if self.debug:
            print(f"BNO085 DEBUG: {message}")

    def _i2c_read_exact(self, nbytes: int, per_read_deadline_s: float = 10.0) -> Optional[bytes]:
        """
        Read exactly nbytes from the SMBus i2c-dev fd. Some kernels return EAGAIN / partial
        reads on chunked SHTP transfers even after clearing O_NONBLOCK — retry like poll+read.
        """
        out = bytearray()
        deadline = time.monotonic() + per_read_deadline_s
        while len(out) < nbytes:
            if time.monotonic() > deadline:
                self.debug_print(
                    f"_i2c_read_exact: timeout ({per_read_deadline_s}s) at {len(out)}/{nbytes} bytes"
                )
                return None
            try:
                chunk = os.read(self.bus.fd, nbytes - len(out))
            except BlockingIOError:
                time.sleep(0.002)
                continue
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EINTR):
                    time.sleep(0.002)
                    continue
                raise
            if not chunk:
                self.debug_print("_i2c_read_exact: EOF (0 bytes)")
                return None
            out.extend(chunk)
        return bytes(out)

    def _rx_init_state(self) -> None:
        """Reset SHTP assembly state (new session / after soft reset)."""
        self._asm_remain = 0
        self._asm_cursor = 0
        self._asm_chan = 0
        self._asm_payload = bytearray(self.MAX_PACKET_SIZE)
        self._next_in_seq = [0] * SH2_MAX_CHANS

    def _hal_read_full_transfer(self) -> Optional[bytes]:
        """
        One logical I2C read as implemented by bno08x_driver i2c_interface.hpp::read():
        4-byte header, then chunk reads (32 B max) with continuation strips (+4 rule).
        Returns buffer of length packet_size (full SHTP packet including header bytes).
        """
        try:
            self._ensure_slave()
            header = self._i2c_read_exact(SHTP_HDR_LEN)
            if not header or len(header) != SHTP_HDR_LEN:
                return None

            packet_size = (header[0] | (header[1] << 8)) & ~0x8000
            # Idle FIFO: [0,0,0,0] — never log (would flood --debug during drain loops).
            if packet_size == 0:
                return None

            self.debug_print(
                f"HAL read: declared packet_size={packet_size} raw_hdr={list(header)}"
            )

            if packet_size > self.MAX_PACKET_SIZE:
                self.debug_print("HAL read: packet_size too large")
                return None

            # Byte-for-byte match i2c_interface.hpp::read(): cargo_remaining starts at
            # packet_size (not packet_size-4). Each continuation read pulls +4 on-wire
            # bytes then discards 4 — total bytes read from the fd exceed the logical SHTP
            # length; shrinking reads to (packet_size-4) desynchronizes the hub.
            buf = bytearray(packet_size)
            cargo_remaining = packet_size
            offset = 0
            first_read = True
            while cargo_remaining > 0:
                if first_read:
                    read_size = min(32, cargo_remaining)
                    chunk = self._i2c_read_exact(read_size)
                    if not chunk or len(chunk) != read_size:
                        self.debug_print(
                            f"HAL read: short first chunk wanted {read_size} got {len(chunk) if chunk else 0}"
                        )
                        return None
                    buf[offset : offset + read_size] = chunk
                    offset += read_size
                    cargo_remaining -= read_size
                    first_read = False
                else:
                    read_size = min(32, cargo_remaining + 4)
                    chunk = self._i2c_read_exact(read_size)
                    if not chunk or len(chunk) != read_size:
                        self.debug_print(
                            f"HAL read: short cont chunk wanted {read_size} got {len(chunk) if chunk else 0}"
                        )
                        return None
                    cargo_amt = read_size - 4
                    buf[offset : offset + cargo_amt] = chunk[4 : 4 + cargo_amt]
                    offset += cargo_amt
                    cargo_remaining -= cargo_amt

            return bytes(buf)
        except OSError as e:
            self.debug_print(f"HAL read OSError: {e}")
            return None

    def _rx_assemble(self, wire: bytes) -> List[Tuple[int, bytes]]:
        """
        Port of sh2 shtp.c rxAssemble for one HAL transfer buffer `wire`.
        Returns list of (channel, cargo_payload) for completed assemblies (cargo only).
        """
        delivered: List[Tuple[int, bytes]] = []
        ln = len(wire)
        if ln < SHTP_HDR_LEN:
            return delivered

        payload_len = (wire[0] | (wire[1] << 8)) & (~0x8000)
        continuation = (wire[1] & 0x80) != 0
        chan = wire[2]
        seq = wire[3]

        self.debug_print(
            f"rxAssemble: len={ln} payloadLen={payload_len} cont={continuation} "
            f"chan={chan} seq={seq}"
        )

        if payload_len < SHTP_HDR_LEN:
            return delivered

        if chan >= SH2_MAX_CHANS:
            return delivered

        # Discard stale assembly if this fragment does not continue correctly
        if self._asm_remain:
            if (
                not continuation
                or (chan != self._asm_chan)
                or (seq != self._next_in_seq[chan])
            ):
                self._asm_remain = 0

        if self._asm_remain == 0:
            if payload_len > len(self._asm_payload):
                return delivered
            self._asm_cursor = 0
            self._asm_chan = chan

        frag_len = ln
        if frag_len > payload_len:
            frag_len = payload_len

        cargo_copy_len = frag_len - SHTP_HDR_LEN
        self._asm_payload[
            self._asm_cursor : self._asm_cursor + cargo_copy_len
        ] = wire[SHTP_HDR_LEN : frag_len]
        self._asm_cursor += cargo_copy_len
        self._asm_remain = payload_len - frag_len

        if self._asm_remain == 0:
            cargo = bytes(self._asm_payload[: self._asm_cursor])
            delivered.append((chan, cargo))

        self._next_in_seq[chan] = seq + 1
        return delivered

    def _i2c_write_raw(self, buf):
        """Single I2C write transaction (SHTP); matches bno08x I2CInterface::write."""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                w = i2c_msg.write(self.addr, bytes(buf))
                self.bus.i2c_rdwr(w)
                return
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EINTR):
                    time.sleep(0.003)
                    continue
                raise
        raise OSError(errno.EAGAIN, "i2c write retries exhausted (bus busy?)")

    def _ensure_slave(self):
        fcntl.ioctl(self.bus.fd, I2C_SLAVE, self.addr)

    def _probe_device_present(self) -> bool:
        """
        Fail-fast presence probe for addr on this bus.

        This avoids proceeding into soft reset / SHTP when the device is missing or
        the wrong bus was selected. Uses a 1-byte I2C read; missing devices typically
        raise ENXIO/EREMOTEIO/EIO.
        """
        try:
            self._ensure_slave()
            r = i2c_msg.read(self.addr, 1)
            self.bus.i2c_rdwr(r)
            return True
        except OSError as e:
            if e.errno in (errno.ENXIO, errno.EREMOTEIO, errno.EIO):
                return False
            # If the bus is busy (EAGAIN) or otherwise erroring, treat as not present
            # for the purposes of "missing sensor" fast-fail.
            return False

    def _i2c_read_raw(self, n):
        """Raw I2C read(s) via SMBus i2c_msg — short probes only; SHTP uses _hal_read_full_transfer."""
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

    def _drain_first_shtp_packet(self, drain_attempts: int = 40) -> Optional[bytes]:
        """After soft reset, poll until the first non-idle SHTP packet or timeout."""
        for _ in range(drain_attempts):
            raw = self._hal_read_full_transfer()
            if raw:
                return raw
            time.sleep(0.05)
        return None

    def test_basic_i2c(self):
        """Test raw I2C + SHTP path used by the ROS driver (not SMBus register reads)."""
        print(f"I2C bus {self.bus_num} → /dev/i2c-{self.bus_num}")
        print(
            "  Align with: ai2c \"Bus 0\" = i2cdetect -y 0; "
            "nodes/vendor/bno085_i2c_argo.yaml → i2c.bus /dev/i2c-0"
        )
        print(f"Probing BNO085 at 0x{self.addr:02x} (SHTP soft reset + drain first packet)...")

        try:
            if not self._probe_device_present():
                print(
                    f"❌ Missing sensor: no I²C ACK from 0x{self.addr:02x} on "
                    f"/dev/i2c-{self.bus_num} (check wiring/power/bus selection)"
                )
                return False
            # TWI0 can stay electrically stuck after a failed driver init or bus
            # contention; re-init + soft reset between rounds matches get_product_id().
            raw: Optional[bytes] = None
            for round_idx in range(3):
                if round_idx > 0:
                    print(
                        f"   Retry {round_idx + 1}/3 after re-init + soft reset "
                        "(TWI0 glitch recovery)..."
                    )
                self._rx_init_state()
                self._shtp_soft_reset()
                raw = self._drain_first_shtp_packet()
                if raw:
                    break
            if not raw:
                print(
                    "❌ No SHTP packet after reset (FIFO idle / zeros). "
                    "Stop other I²C users: pgrep -af bno08x_driver; "
                    "sudo systemctl stop argo_bno085.service argo_battery_water.service; "
                    "then: sudo bash ~/argo/scripts/reset_i2c_bus.sh --force "
                    "(or replug I2C cable if still stuck — see docs/README-i2c.md)."
                )
                return False
            h = raw[:4]
            length_field = (h[0] | (h[1] << 8)) & 0x7FFF
            ch, sq = h[2], h[3]
            self._rx_assemble(raw)
            print(
                f"✅ First SHTP packet (HAL): length={length_field} "
                f"channel={ch} seq={sq} bytes={len(raw)}"
            )
            return True
        except OSError as e:
            print(f"❌ Basic I2C failed: {e}")
            return False
    
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

    def _parse_product_id_cargo(self, pl: bytes):
        """
        If SH-2 cargo is a Product ID Response (0xF8) at offset 0, return fields dict.
        Layout matches sh2.c ProdIdResp_t. We do **not** scan for 0xF8 inside the buffer:
        a mid-payload 0xF8 byte often produces plausible-looking garbage versions/parts
        on a noisy bus. Vendor treats payload as starting with reportId (getProdIdRx).
        """
        if len(pl) < 16 or pl[0] != self.REPORT_PRODUCT_ID_RESPONSE:
            return None

        reset_cause = pl[1]
        sw_major = pl[2]
        sw_minor = pl[3]
        sw_part = (pl[7] << 24) | (pl[6] << 16) | (pl[5] << 8) | pl[4]
        sw_build = (pl[11] << 24) | (pl[10] << 16) | (pl[9] << 8) | pl[8]
        sw_patch = (pl[13] << 8) | pl[12]
        reserved0 = pl[14]
        reserved1 = pl[15]

        # BNO08x / SH-2: sane ranges; skip false positives from arbitrary payloads.
        if not (1 <= sw_major <= 31 and 0 <= sw_minor <= 127 and 0 <= sw_patch <= 65535):
            return None
        if sw_part == 0 or sw_part > 0xFFFFFFFF:
            return None
        if reserved0 != 0 or reserved1 != 0:
            return None
        # BNO08x on Argo: Bosch hub parts are 0x0098xxxx (see sh2.c FSP200 exception).
        if (sw_part >> 16) != 0x0098:
            return None

        return {
            'reset_cause': reset_cause,
            'sw_major': sw_major,
            'sw_minor': sw_minor,
            'sw_patch': sw_patch,
            'sw_part': sw_part,
            'sw_build': sw_build,
        }

    def _get_product_id_once(self, pre_drain: bool = True) -> Optional[dict]:
        """Single F9 request + drain. Product ID (0xF8) must parse on control channel only."""
        if pre_drain:
            # Drop a few pending frames so F9 lands on a cleaner queue (noisy TWI0 / mast).
            for _ in range(4):
                raw = self._hal_read_full_transfer()
                if not raw:
                    break
                self._rx_assemble(raw)
            time.sleep(0.05)

        self._write_packet(self.CHANNEL_CONTROL, [
            self.REPORT_PRODUCT_ID_REQUEST,
            0x00,
        ])
        time.sleep(0.35)

        for _ in range(200):
            raw = self._hal_read_full_transfer()
            if not raw:
                time.sleep(0.01)
                continue
            delivered = self._rx_assemble(raw)
            for ch, cargo in delivered:
                if ch == self.CHANNEL_CONTROL:
                    head = cargo[:24] if len(cargo) > 24 else cargo
                    self.debug_print(
                        f"assembled ch={ch} cargo_len={len(cargo)} "
                        f"cargo[:24]={list(head)}"
                    )
                if ch != self.CHANNEL_CONTROL:
                    continue
                info = self._parse_product_id_cargo(cargo)
                if info:
                    return info
            time.sleep(0.01)
        return None

    def get_product_id(self):
        """Get BNO085 product ID and version information."""
        print("Requesting Product ID...")

        def _emit(info: dict) -> dict:
            print("✅ Product ID Response:")
            print(f"   Reset Cause: {info['reset_cause']}")
            print(
                f"   Version: {info['sw_major']}."
                f"{info['sw_minor']}.{info['sw_patch']}"
            )
            print(f"   Part Number: 0x{info['sw_part']:08X}")
            print(f"   Build Number: {info['sw_build']}")
            return info

        try:
            # Majority vote on (part, major, minor, patch, build). Within each round,
            # do several F9 + drains without soft-reset between them so the hub stays
            # in a stable session (resetting before every sample often yielded 0 parses).
            # Between rounds, re-init + soft reset to recover from TWI0 glitches.
            vote_keys = ("sw_part", "sw_major", "sw_minor", "sw_patch", "sw_build")
            samples: List[Tuple[Tuple[int, ...], dict]] = []
            for round_idx in range(3):
                if round_idx > 0:
                    self.debug_print(
                        f"Product ID: round {round_idx + 1}/3 after re-init + soft reset"
                    )
                    self._rx_init_state()
                    self._shtp_soft_reset()
                    time.sleep(0.25 + 0.08 * round_idx)
                for s in range(3):
                    info = self._get_product_id_once(pre_drain=True)
                    if info:
                        sig = tuple(info[k] for k in vote_keys)
                        samples.append((sig, info))
                        self.debug_print(
                            "Product ID vote sample: "
                            f"part=0x{info['sw_part']:08X} ver={info['sw_major']}."
                            f"{info['sw_minor']}.{info['sw_patch']} build={info['sw_build']}"
                        )
                counts = Counter(sig for sig, _ in samples)
                if counts and counts.most_common(1)[0][1] >= 2:
                    break

            if not samples:
                print(
                    "❌ No Product ID response received (I²C contention: mast/wind/other "
                    "masters, or errno 11 on writes — stop argo_bno085 / isolate bus)."
                )
                return None

            best_sig, n = Counter(s[0] for s in samples).most_common(1)[0]
            if n < 2:
                print(
                    "❌ No Product ID consensus: need ≥2 matching (part, ver, build) "
                    f"fingerprints (got {len(samples)} parse(s) across ≤3 rounds, "
                    "no duplicate fingerprint)."
                )
                return None

            winner = next(info for sig, info in samples if sig == best_sig)
            if self.expect_part is not None and winner["sw_part"] != self.expect_part:
                print(
                    "❌ Product ID part mismatch: "
                    f"expected 0x{self.expect_part:08X}, got 0x{winner['sw_part']:08X} "
                    "(use --expect-part only if you lock BOM)."
                )
                return None

            return _emit(winner)

        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EINTR):
                print(
                    f"❌ Error getting Product ID: {e} — retry exhausted; "
                    "bus may be busy (other nodes, long I2C wires, EMI)."
                )
            else:
                print(f"❌ Error getting Product ID: {e}")
            return None
        except Exception as e:
            print(f"❌ Error getting Product ID: {e}")
            return None

    def _set_feature_bytes(self, feature_id: int, report_interval_us: int) -> bytes:
        """SetFeatureReport_t (sh2.c setSensorConfigStart), packed LE."""
        return struct.pack(
            '<BBBHIII',
            self.REPORT_SET_FEATURE_CMD,
            feature_id & 0xFF,
            0,
            0,
            int(report_interval_us) & 0xFFFFFFFF,
            0,
            0,
        )

    def _accel_triplet_at(
        self, cargo: bytes, off: int
    ) -> Optional[Tuple[float, float, float]]:
        """One SH2_ACCELEROMETER report at offset ``off`` (sh2_SensorValue.c decodeAccelerometer)."""
        if off + self._accel_report_min_bytes > len(cargo):
            return None
        if cargo[off] != self.SH2_ACCELEROMETER:
            return None
        chunk = cargo[off : off + self._accel_report_min_bytes]
        x = int.from_bytes(chunk[4:6], 'little', signed=True) / 256.0
        y = int.from_bytes(chunk[6:8], 'little', signed=True) / 256.0
        z = int.from_bytes(chunk[8:10], 'little', signed=True) / 256.0
        mag = math.sqrt(x * x + y * y + z * z)
        if 3.0 <= mag <= 30.0:
            return (x, y, z)
        return None

    def _iter_accel_triplets(self, cargo: bytes):
        """Scan channel-3 payload for concatenated / padded report streams."""
        i = 0
        n = len(cargo)
        while i + self._accel_report_min_bytes <= n:
            t = self._accel_triplet_at(cargo, i)
            if t is not None:
                yield t
                i += self._accel_report_min_bytes
                continue
            i += 1

    def test_imu_reports_via_accel(
        self,
        min_samples: int = 3,
        timeout_s: float = 12.0,
        report_interval_us: int = 200_000,
    ) -> Tuple[bool, str]:
        """
        Enable SH2_ACCELEROMETER via SET_FEATURE (same wire format as sh2_setSensorConfig)
        and require plausible reports on SHTP channel 3 (input / reports).
        """
        print(
            "Enabling calibrated accelerometer (SET_FEATURE 0xFD, "
            "sensor 0x01, decode per sh2_SensorValue.c)..."
        )
        self._rx_init_state()
        for _ in range(6):
            raw = self._hal_read_full_transfer()
            if not raw:
                break
            self._rx_assemble(raw)
            time.sleep(0.002)

        blob = self._set_feature_bytes(self.SH2_ACCELEROMETER, report_interval_us)
        if not self._write_packet(self.CHANNEL_CONTROL, list(blob)):
            return False, 'SET_FEATURE control write failed'

        time.sleep(0.2)
        samples: List[Tuple[float, float, float]] = []
        t0 = time.monotonic()
        reads = 0
        while time.monotonic() - t0 < timeout_s and len(samples) < min_samples:
            raw = self._hal_read_full_transfer()
            reads += 1
            if not raw:
                time.sleep(0.005)
                continue
            for ch, cargo in self._rx_assemble(raw):
                if ch != self.CHANNEL_REPORTS:
                    continue
                for triplet in self._iter_accel_triplets(cargo):
                    samples.append(triplet)
                    self.debug_print(f"accel sample: {triplet}")
            time.sleep(0.002)

        if len(samples) < min_samples:
            msg = (
                f'only {len(samples)} plausible accel report(s) in {timeout_s:g}s '
                f'({reads} HAL reads) — settle time, bus contention, or wrong channel'
            )
            print(f"❌ {msg}")
            return False, msg

        x, y, z = samples[-1]
        mag = math.sqrt(x * x + y * y + z * z)
        msg = (
            f'{len(samples)} SH2_ACCELEROMETER reports; last |g|≈{mag:.2f} m/s² '
            f'(xyz={x:.2f},{y:.2f},{z:.2f})'
        )
        print(f"✅ {msg}")
        return True, msg

    def run_full_test(
        self,
        skip_ros_verification=False,
        ros_hz_seconds=14,
        skip_product_id=False,
        report_interval_us=200_000,
    ):
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
        version_info: Optional[dict] = None
        imu_detail = ""

        if skip_product_id:
            print("Skipping Product ID check (--skip-product-id).")
            ok, imu_detail = self.test_imu_reports_via_accel(
                report_interval_us=report_interval_us,
            )
            if not ok:
                return False
        else:
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

        if skip_product_id:
            print("✅ BNO085 live reports (calibrated accelerometer)")
            print(f"   {imu_detail}")
        else:
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

        if skip_product_id:
            overall = True
        else:
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
        help='Skip ros2 topic hz /imu (bench: I2C + Product ID, or + --skip-product-id)',
    )
    parser.add_argument(
        '--skip-product-id',
        action='store_true',
        help=(
            'Skip Product ID (F9) consensus; enable SH2_ACCELEROMETER via SET_FEATURE '
            'and verify live SHTP channel-3 reports'
        ),
    )
    parser.add_argument(
        '--report-interval-us',
        type=int,
        default=200_000,
        metavar='US',
        help='Accel report interval for --skip-product-id (default: 200000 = 5 Hz)',
    )
    parser.add_argument(
        '--ros-hz-timeout',
        type=int,
        default=14,
        metavar='SEC',
        help='Seconds for ros2 topic hz /imu sample window (default: 14)',
    )
    parser.add_argument(
        '--expect-part',
        type=lambda x: int(x, 0),
        default=None,
        metavar='HEX',
        help='Require Product ID part number (sw_part) to match, e.g. 0x0098A6B4',
    )

    args = parser.parse_args()

    if args.skip_product_id and args.expect_part is not None:
        parser.error('--expect-part is incompatible with --skip-product-id')

    tester = BNO085Test(
        bus_num=args.bus,
        address=args.address,
        debug=args.debug,
        expect_part=args.expect_part,
    )

    success = tester.run_full_test(
        skip_ros_verification=args.skip_ros,
        ros_hz_seconds=max(8, args.ros_hz_timeout),
        skip_product_id=args.skip_product_id,
        report_interval_us=max(10_000, args.report_interval_us),
    )
    
    if success:
        print("\n🎉 BNO085 test completed successfully!")
        sys.exit(0)
    else:
        print("\n💥 BNO085 test failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
