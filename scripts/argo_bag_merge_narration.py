#!/usr/bin/env python3
"""
Merge SRT narration into a rosbag2 as /narration (std_msgs/String).

Syncs SRT relative timestamps to bag log time. Each SRT cue is stamped at
sync_anchor + cue.start_sec. Default sync anchor is a ``_YYMMDD_HHMMSS`` token in
the SRT or audio filename (local time). exiftool CreateDate (UTC on Android) is
shown for reference only — it is usually not the record-button time.

Usage
-----
  source /opt/ros/humble/setup.bash

  python3 scripts/argo_bag_merge_narration.py \\
    --bag ~/argo/bags/argo_20260602_141612 --dry-run

  python3 scripts/argo_bag_merge_narration.py \\
    --bag ~/argo/bags/argo_20260602_141612 \\
    --output ~/argo/bags/argo_20260602_141612_with_narration

  python3 scripts/argo_bag_merge_narration.py \\
    --bag ~/argo/bags/argo_20260602_141612 \\
    --output ~/argo/bags/argo_20260602_141612_with_narration \\
    --offset-sec 0.5 --force
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from typing import Iterator, Optional
from zoneinfo import ZoneInfo

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

try:
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
except ImportError as e:
    print(f"Error: ROS 2 Python imports failed: {e}", file=sys.stderr)
    print("Source ROS 2, e.g.: source /opt/ros/humble/setup.bash", file=sys.stderr)
    sys.exit(1)

try:
    from rosbag2_py import (
        SequentialReader,
        SequentialWriter,
        StorageOptions,
        ConverterOptions,
        TopicMetadata,
    )
except ImportError:
    print(
        "Error: rosbag2_py not available (install ROS 2 rosbag2 packages).",
        file=sys.stderr,
    )
    sys.exit(1)


NARRATION_TOPIC = "/narration"
NARRATION_TYPE = "std_msgs/msg/String"
DEFAULT_TIMEZONE = "Europe/Zurich"

_FILENAME_TS_RE = re.compile(r"(?<!\d)(\d{6})_(\d{6})(?!\d)")

_SRT_BLOCK_RE = re.compile(
    r"(?ms)^\s*(\d+)\s*\r?\n"
    r"(?:[^\r\n]*?\s)?"
    r"(\d{2}:\d{2}:\d{2},\d{3})\s*-->\s*(\d{2}:\d{2}:\d{2},\d{3})\s*\r?\n"
    r"(.*?)(?=\r?\n\s*\r?\n|\Z)"
)
# e.g. 260602_152344 in "Argo ... 260602_152344.m4a" → 2026-06-02 15:23:44
_FILENAME_TS_RE = re.compile(r"(?:^|[^\d])(\d{6})_(\d{6})(?:[^\d]|$)")


@dataclass(frozen=True)
class SrtCue:
    index: int
    start_sec: float
    end_sec: float
    text: str


def _resolve_bag_directory(path: str) -> str:
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.isdir(path):
        return path
    if os.path.isfile(path) and (path.endswith(".db3") or path.endswith(".mcap")):
        return os.path.dirname(path)
    raise FileNotFoundError(f"Not a bag directory or database file: {path}")


def _read_storage_id(bag_dir: str) -> str:
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if yaml is None or not os.path.isfile(meta_path):
        return "sqlite3"
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        info = meta.get("rosbag2_bagfile_information") or {}
        sid = info.get("storage_identifier")
        if isinstance(sid, str) and sid:
            return sid
    except (OSError, yaml.YAMLError):
        pass
    return "sqlite3"


def _bag_metadata_start_duration_ns(bag_dir: str) -> tuple[Optional[int], Optional[int]]:
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if yaml is None or not os.path.isfile(meta_path):
        return None, None
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        info = data.get("rosbag2_bagfile_information") or {}
        st = info.get("starting_time")
        dur = info.get("duration")
        start_ns = None
        if isinstance(st, dict):
            start_ns = st.get("nanoseconds_since_epoch")
            if start_ns is None:
                start_ns = st.get("nanoseconds")
        elif isinstance(st, (int, float)):
            start_ns = int(st)
        dur_ns = None
        if isinstance(dur, dict):
            dur_ns = dur.get("nanoseconds")
        elif isinstance(dur, (int, float)):
            dur_ns = int(dur)
        if start_ns is not None:
            start_ns = int(start_ns)
        if dur_ns is not None:
            dur_ns = int(dur_ns)
        return start_ns, dur_ns
    except (OSError, yaml.YAMLError, TypeError, ValueError):
        return None, None


def _strip_optional_zone_suffix(s: str) -> str:
    return re.sub(
        r"\s+(CEST|CET|BST|EEST|EET|GMT|UTC)\s*$",
        "",
        s.strip(),
        flags=re.IGNORECASE,
    ).strip()


def _parse_wall_time_to_ns(
    s: str,
    tz_name: str,
    *,
    naive_as_utc: bool = False,
) -> int:
    """Parse a wall-clock string to epoch ns. If naive and naive_as_utc, treat as UTC."""
    raw = _strip_optional_zone_suffix(s)
    iso_candidate = raw.replace("Z", "+00:00")
    if "T" not in iso_candidate and re.match(r"^\d{4}-\d{2}-\d{2}\s", iso_candidate):
        iso_candidate = re.sub(r"^(\d{4}-\d{2}-\d{2})\s+", r"\1T", iso_candidate, count=1)
    try:
        dt = datetime.fromisoformat(iso_candidate)
        if dt.tzinfo is None:
            tz = ZoneInfo("UTC") if naive_as_utc else ZoneInfo(tz_name)
            dt = dt.replace(tzinfo=tz)
        return int(dt.timestamp() * 1e9)
    except ValueError:
        pass
    tz = ZoneInfo("UTC") if naive_as_utc else ZoneInfo(tz_name)
    for fmt in (
        "%Y:%m:%d %H:%M:%S.%f",
        "%Y:%m:%d %H:%M:%S",
        "%Y-%m-%d %I:%M:%S.%f %p",
        "%Y-%m-%d %I:%M:%S %p",
        "%Y-%m-%d %H:%M:%S.%f",
        "%Y-%m-%d %H:%M:%S",
    ):
        try:
            dt_naive = datetime.strptime(raw, fmt)
            dt = dt_naive.replace(tzinfo=tz)
            return int(dt.timestamp() * 1e9)
        except ValueError:
            continue
    raise ValueError(
        f"Could not parse wall-clock time {s!r} "
        f"({'UTC' if naive_as_utc else tz_name!r})."
    )


def _parse_samsung_utc_offset_hours(exif: dict) -> Optional[float]:
    raw = exif.get("Samsung Android Utc Offset") or exif.get("SamsungAndroidUtcOffset")
    if raw is None:
        return None
    s = str(raw).strip()
    m = re.match(r"^([+-])(\d{2})(\d{2})$", s)
    if not m:
        return None
    sign = 1 if m.group(1) == "+" else -1
    return sign * (int(m.group(2)) + int(m.group(3)) / 60.0)


def _audio_start_from_filename(path: str, tz_name: str) -> Optional[tuple[int, str]]:
    """Parse _YYMMDD_HHMMSS in basename as local recording start (e.g. 260602_152344)."""
    m = _FILENAME_TS_RE.search(os.path.basename(path))
    if not m:
        return None
    yymmdd, hhmmss = m.group(1), m.group(2)
    try:
        dt = datetime.strptime(f"20{yymmdd}{hhmmss}", "%Y%m%d%H%M%S")
        dt = dt.replace(tzinfo=ZoneInfo(tz_name))
        label = (
            f"filename token {yymmdd}_{hhmmss} "
            f"({dt.strftime('%Y-%m-%d %H:%M:%S')} {tz_name})"
        )
        return int(dt.timestamp() * 1e9), label
    except ValueError:
        return None


def _bag_start_from_folder_name(bag_dir: str, tz_name: str) -> Optional[int]:
    base = os.path.basename(os.path.normpath(bag_dir))
    m = re.match(r"^argo_(\d{8})_(\d{6})$", base)
    if not m:
        return None
    try:
        dt = datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
        dt = dt.replace(tzinfo=ZoneInfo(tz_name))
        return int(dt.timestamp() * 1e9)
    except ValueError:
        return None


def _ns_to_local_str(ns: int, tz_name: str) -> str:
    dt = datetime.fromtimestamp(ns / 1e9, tz=ZoneInfo(tz_name))
    return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


_AUDIO_SUFFIXES = (".m4a", ".mp4", ".aac", ".wav", ".mp3")


def _discover_single_file(
    directories: list[str],
    suffixes: str | tuple[str, ...],
    label: str,
    flag_name: str,
) -> str:
    if isinstance(suffixes, str):
        suffixes = (suffixes,)
    search_dirs: list[str] = []
    seen_dirs: set[str] = set()
    for directory in directories:
        directory = os.path.abspath(os.path.expanduser(directory))
        if directory in seen_dirs or not os.path.isdir(directory):
            continue
        seen_dirs.add(directory)
        search_dirs.append(directory)

    matches: list[str] = []
    for directory in search_dirs:
        for f in os.listdir(directory):
            if any(f.lower().endswith(s.lower()) for s in suffixes):
                matches.append(os.path.join(directory, f))
    matches = sorted(set(matches))

    if not matches:
        dirs = ", ".join(repr(d) for d in search_dirs)
        suffix_list = ", ".join(f"*{s}" for s in suffixes)
        raise FileNotFoundError(
            f"No {label} file ({suffix_list}) found in {dirs}."
        )
    if len(matches) > 1:
        names = ", ".join(os.path.basename(p) for p in matches)
        raise FileExistsError(
            f"Multiple {label} files: {names}. Pass --{flag_name} explicitly."
        )
    return matches[0]


def _resolve_bag_and_sidecar_dirs(bag_dir: str) -> tuple[str, list[str]]:
    """Return (rosbag2 directory, search dirs for .srt/.audio sidecars)."""
    bag_dir = _resolve_bag_directory(bag_dir)
    sidecar_dirs = [bag_dir]
    parent = os.path.dirname(bag_dir)
    if parent and parent not in sidecar_dirs:
        sidecar_dirs.append(parent)
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.isfile(meta_path):
        for name in os.listdir(bag_dir):
            sub = os.path.join(bag_dir, name)
            if os.path.isdir(sub) and os.path.isfile(os.path.join(sub, "metadata.yaml")):
                return sub, sidecar_dirs
    return bag_dir, sidecar_dirs


def _parse_srt_timestamp(ts: str) -> float:
    hh, mm, rest = ts.strip().split(":")
    sec, ms = rest.split(",")
    return int(hh) * 3600 + int(mm) * 60 + int(sec) + int(ms) / 1000.0


def parse_srt(path: str) -> list[SrtCue]:
    with open(path, "r", encoding="utf-8-sig") as f:
        content = f.read()
    cues: list[SrtCue] = []
    for m in _SRT_BLOCK_RE.finditer(content):
        text = " ".join(line.strip() for line in m.group(4).strip().splitlines() if line.strip())
        if not text:
            continue
        cues.append(
            SrtCue(
                index=int(m.group(1)),
                start_sec=_parse_srt_timestamp(m.group(2)),
                end_sec=_parse_srt_timestamp(m.group(3)),
                text=text,
            )
        )
    if not cues:
        raise ValueError(f"No SRT cues parsed from {path!r}.")
    return cues


def _parse_duration_seconds(value: object) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    s = str(value).strip()
    if not s:
        return None
    m = re.match(r"^(\d+(?:\.\d+)?)\s*s(?:ec(?:onds)?)?$", s, re.IGNORECASE)
    if m:
        return float(m.group(1))
    parts = s.split(":")
    try:
        if len(parts) == 3:
            h, m_, sec = parts
            return int(h) * 3600 + int(m_) * 60 + float(sec)
        if len(parts) == 2:
            m_, sec = parts
            return int(m_) * 60 + float(sec)
    except ValueError:
        pass
    return None


def _exiftool_audio_metadata(audio_path: str) -> dict:
    try:
        result = subprocess.run(
            [
                "exiftool",
                "-json",
                "-CreateDate",
                "-MediaCreateDate",
                "-TrackCreateDate",
                "-Duration",
                "-SamsungAndroidUtcOffset",
                audio_path,
            ],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except FileNotFoundError as e:
        raise RuntimeError(
            "exiftool not found. Install with: sudo apt install libimage-exiftool-perl"
        ) from e
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        raise RuntimeError(f"exiftool failed on {audio_path!r}: {detail}")
    data = json.loads(result.stdout)
    if not data:
        raise RuntimeError(f"exiftool returned no metadata for {audio_path!r}.")
    return data[0]


def _audio_start_ns_exiftool_utc(exif: dict) -> tuple[int, str]:
    """Android 3GP CreateDate without +ZZ:ZZ is UTC (check File Modification ≈ Create+offset)."""
    for key in ("CreateDate", "MediaCreateDate", "TrackCreateDate"):
        raw = exif.get(key)
        if raw:
            ns = _parse_wall_time_to_ns(str(raw), "UTC", naive_as_utc=True)
            return ns, f"exiftool {key} (naive → UTC)"
    raise ValueError(
        "No CreateDate / MediaCreateDate / TrackCreateDate in exiftool output."
    )


def _is_cropped_srt(srt_path: Optional[str]) -> bool:
    if not srt_path:
        return False
    return "cropped" in os.path.basename(srt_path).lower()


def _resolve_sync_anchor_ns(
    audio_path: str,
    srt_path: Optional[str],
    exif: dict,
    tz_name: str,
    anchor: str,
    audio_start_override: Optional[str],
    bag_start_ns: int,
    bag_dir: str,
) -> tuple[int, str]:
    """Wall-clock time for SRT t=0."""
    if audio_start_override:
        ns = _parse_wall_time_to_ns(audio_start_override, tz_name)
        return ns, f"--audio-start ({tz_name})"

    from_srt = _audio_start_from_filename(srt_path, tz_name) if srt_path else None
    from_audio = _audio_start_from_filename(audio_path, tz_name)
    from_filename = from_srt or from_audio
    from_exif = _audio_start_ns_exiftool_utc(exif)
    from_bag_folder = _bag_start_from_folder_name(bag_dir, tz_name)

    if anchor == "bag-start":
        return bag_start_ns, f"bag metadata first message ({bag_start_ns})"
    if anchor == "bag-folder":
        if from_bag_folder is None:
            raise ValueError(
                f"--audio-anchor bag-folder but bag directory is not argo_YYYYMMDD_HHMMSS: {bag_dir!r}"
            )
        return from_bag_folder, "bag folder name argo_YYYYMMDD_HHMMSS (record button time)"
    if anchor == "exiftool":
        return from_exif
    if anchor == "filename":
        if from_filename is None:
            raise ValueError(
                "No YYMMDD_HHMMSS token in --srt or --audio basename. "
                "Add a timestamp to the filename, pass --audio-start, or use --audio-anchor bag-start."
            )
        return from_filename
    # auto: cropped SRT timestamps are relative to the crop (≈ bag start), not phone record start
    if _is_cropped_srt(srt_path):
        return (
            bag_start_ns,
            "bag metadata first message (auto: *_cropped.srt → SRT t=0 at bag start)",
        )
    if from_filename is not None:
        return from_filename
    return from_exif


def _cue_log_time_ns(audio_start_ns: int, cue_start_sec: float, offset_sec: float) -> int:
    return audio_start_ns + int((cue_start_sec + offset_sec) * 1e9)


def _window_bounds_ns(
    time_window: str,
    audio_start_ns: int,
    bag_start_ns: int,
    bag_end_ns: Optional[int],
) -> tuple[int, Optional[int], str]:
    if time_window == "bag":
        return bag_start_ns, bag_end_ns, "bag start → bag end"
    # session: phone narration from t=0 through end of rosbag recording
    return audio_start_ns, bag_end_ns, "audio/SRT t=0 → bag end"


def _open_reader(bag_dir: str, storage_id: str) -> SequentialReader:
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id=storage_id),
        converter_options,
    )
    return reader


def _iter_bag_messages(reader: SequentialReader) -> Iterator[tuple[str, bytes, int]]:
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        yield topic, data, timestamp


def _build_narration_messages(
    cues: list[SrtCue],
    audio_start_ns: int,
    offset_sec: float,
    window_start_ns: int,
    window_end_ns: Optional[int],
) -> tuple[list[tuple[int, bytes]], int]:
    """Build /narration messages; only cues with log time inside the selected window."""
    out: list[tuple[int, bytes]] = []
    skipped = 0
    for cue in cues:
        ts_ns = _cue_log_time_ns(audio_start_ns, cue.start_sec, offset_sec)
        if window_end_ns is not None and (ts_ns < window_start_ns or ts_ns > window_end_ns):
            skipped += 1
            continue
        msg = String(data=cue.text)
        out.append((ts_ns, serialize_message(msg)))
    out.sort(key=lambda item: item[0])
    return out, skipped


def _merge_write_bag(
    reader: SequentialReader,
    writer: SequentialWriter,
    narration: list[tuple[int, bytes]],
) -> tuple[int, int]:
    bag_iter = _iter_bag_messages(reader)
    narr_iter = iter(narration)
    next_bag = next(bag_iter, None)
    next_narr = next(narr_iter, None)
    bag_count = 0
    narr_count = 0

    while next_bag is not None or next_narr is not None:
        if next_bag is None:
            writer.write(NARRATION_TOPIC, next_narr[1], next_narr[0])
            narr_count += 1
            next_narr = next(narr_iter, None)
            continue
        if next_narr is None:
            writer.write(next_bag[0], next_bag[1], next_bag[2])
            bag_count += 1
            next_bag = next(bag_iter, None)
            continue
        if next_narr[0] <= next_bag[2]:
            writer.write(NARRATION_TOPIC, next_narr[1], next_narr[0])
            narr_count += 1
            next_narr = next(narr_iter, None)
        else:
            writer.write(next_bag[0], next_bag[1], next_bag[2])
            bag_count += 1
            next_bag = next(bag_iter, None)

    return bag_count, narr_count


def _print_sync_report(
    bag_dir: str,
    bag_start_ns: int,
    bag_start_source: str,
    bag_dur_ns: Optional[int],
    audio_path: str,
    srt_path: str,
    sync_anchor_ns: int,
    sync_anchor_key: str,
    phone_filename_ns: Optional[int],
    bag_folder_ns: Optional[int],
    audio_duration_sec: Optional[float],
    offset_sec: float,
    cues: list[SrtCue],
    tz_name: str,
    exif: dict,
    time_window: str,
) -> None:
    bag_end_ns = (bag_start_ns + bag_dur_ns) if bag_dur_ns is not None else None
    window_start_ns, window_end_ns, window_label = _window_bounds_ns(
        time_window, sync_anchor_ns, bag_start_ns, bag_end_ns
    )
    srt_span = cues[-1].end_sec - cues[0].start_sec if cues else 0.0

    print(f"Bag:           {bag_dir}")
    print(f"Bag start:     {_ns_to_local_str(bag_start_ns, tz_name)} ({bag_start_source})")
    print("  (first message in mcap — not the dashboard button instant)")
    if bag_end_ns is not None:
        print(f"Bag end:       {_ns_to_local_str(bag_end_ns, tz_name)}  ({bag_dur_ns / 1e9:.3f} s)")
    if bag_folder_ns is not None:
        delta = (bag_start_ns - bag_folder_ns) / 1e9
        print(
            f"Bag folder:    {_ns_to_local_str(bag_folder_ns, tz_name)} "
            f"(argo_* name when record.py started ros2 bag record; {delta:+.1f} s before first message)"
        )
    if phone_filename_ns is not None:
        delta = (bag_start_ns - phone_filename_ns) / 1e9
        print(
            f"Phone file:    {_ns_to_local_str(phone_filename_ns, tz_name)} "
            f"(YYMMDD_HHMMSS on .m4a; {delta:+.1f} s before first bag message)"
        )
    print(f"Audio:         {audio_path}")
    print(f"SRT:           {srt_path}")
    if _is_cropped_srt(srt_path):
        print("  (*_cropped.srt → SRT t=0 is crop start, usually ≈ bag start, not phone file token)")
    print(f"SRT t=0 anchor: {_ns_to_local_str(sync_anchor_ns, tz_name)} ({sync_anchor_key})")
    try:
        exif_utc_ns, exif_key = _audio_start_ns_exiftool_utc(exif)
        offset_h = _parse_samsung_utc_offset_hours(exif)
        print(
            f"Exiftool (ref): {_ns_to_local_str(exif_utc_ns, 'UTC')} UTC  "
            f"({_ns_to_local_str(exif_utc_ns, tz_name)} {tz_name})  [{exif_key}]"
        )
        print(
            "  Not used for sync when a filename YYMMDD_HHMMSS token is present "
            "(CreateDate is often container/export time, not record-button time)."
        )
        if offset_h is not None:
            print(f"  Samsung UTC offset: {offset_h:+.1f} h")
    except ValueError:
        pass
    if audio_duration_sec is not None and phone_filename_ns is not None:
        audio_end_ns = phone_filename_ns + int(audio_duration_sec * 1e9)
        print(
            f"Audio span:    {audio_duration_sec:.1f} s → ends "
            f"{_ns_to_local_str(audio_end_ns, tz_name)}"
        )
    print(f"SRT cues:      {len(cues)}")
    print(f"SRT span:      {srt_span:.3f} s (first cue start → last cue end)")
    print()
    print("Merge logic: cue log time = sync_anchor + cue.start_sec (+ --offset-sec).")
    print(f"  Write window ({time_window}): {window_label}.")
    if offset_sec:
        print(f"Manual --offset-sec: {offset_sec:+.3f} s")
    print()
    print("Sample cue mapping (first 5):")
    for cue in cues[:5]:
        ts_ns = _cue_log_time_ns(sync_anchor_ns, cue.start_sec, offset_sec)
        rel = (ts_ns - bag_start_ns) / 1e9
        preview = cue.text if len(cue.text) <= 72 else cue.text[:69] + "..."
        print(
            f"  #{cue.index:3d}  t={cue.start_sec:8.3f}s  "
            f"bag+{rel:8.3f}s  {_ns_to_local_str(ts_ns, tz_name)}  {preview!r}"
        )

    if window_end_ns is not None and cues:
        in_window = 0
        first_in: Optional[SrtCue] = None
        last_in: Optional[SrtCue] = None
        for cue in cues:
            ts_ns = _cue_log_time_ns(sync_anchor_ns, cue.start_sec, offset_sec)
            if window_start_ns <= ts_ns <= window_end_ns:
                in_window += 1
                if first_in is None:
                    first_in = cue
                last_in = cue
        print()
        if in_window == 0:
            print(
                "Warning: no SRT cues fall within the write window. "
                "Try --time-window session, --audio-start, or --offset-sec."
            )
        else:
            print(f"Cues to write: {in_window}/{len(cues)} ({window_label})")
            if first_in and last_in:
                print(
                    f"  First: #{first_in.index} at SRT t={first_in.start_sec:.1f}s  "
                    f"{first_in.text[:60]!r}..."
                )
                print(
                    f"  Last:  #{last_in.index} at SRT t={last_in.start_sec:.1f}s  "
                    f"{last_in.text[:60]!r}..."
                )
        if in_window < len(cues):
            print(f"  Omitted: {len(cues) - in_window} cues outside write window")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Merge SRT narration into a rosbag2 as /narration (std_msgs/String)."
    )
    parser.add_argument(
        "--bag",
        required=True,
        help="Input rosbag2 directory",
    )
    parser.add_argument(
        "--output",
        help="Output rosbag2 directory (required unless --dry-run)",
    )
    parser.add_argument(
        "--srt",
        help="SRT file (default: sole .srt in bag folder)",
    )
    parser.add_argument(
        "--audio",
        help="Source m4a/audio file for exiftool sync (default: sole .m4a in bag folder)",
    )
    parser.add_argument(
        "--offset-sec",
        type=float,
        default=0.0,
        help="Extra offset added to computed audio−bag sync (seconds)",
    )
    parser.add_argument(
        "--timezone",
        default=DEFAULT_TIMEZONE,
        help=f"IANA timezone for bag/filename display and --audio-start (default: {DEFAULT_TIMEZONE})",
    )
    parser.add_argument(
        "--audio-anchor",
        choices=("auto", "filename", "exiftool", "bag-start", "bag-folder"),
        default="auto",
        help=(
            "SRT t=0 wall time: auto=cropped SRT→bag first message, else filename token; "
            "filename=YYMMDD_HHMMSS; bag-start=metadata.yaml; bag-folder=argo_* dirname; "
            "exiftool=CreateDate UTC"
        ),
    )
    parser.add_argument(
        "--time-window",
        choices=("session", "bag"),
        default="session",
        help=(
            "Which cues to write: session=audio/SRT t=0 through bag end (keeps pre-bag "
            "phone narration); bag=only between bag start and bag end"
        ),
    )
    parser.add_argument(
        "--audio-start",
        help="Override audio/SRT t=0 wall time (ISO or 'YYYY-MM-DD HH:MM:SS', uses --timezone)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Remove output directory if it already exists before writing",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print sync diagnostics only; do not write a bag",
    )
    args = parser.parse_args()

    if not args.dry_run and not args.output:
        parser.error("--output is required unless --dry-run is set.")

    try:
        ZoneInfo(args.timezone)
    except Exception as e:
        print(f"Error: invalid --timezone {args.timezone!r}: {e}", file=sys.stderr)
        return 2

    try:
        bag_dir, sidecar_dirs = _resolve_bag_and_sidecar_dirs(args.bag)
        srt_path = (
            os.path.abspath(os.path.expanduser(args.srt))
            if args.srt
            else _discover_single_file(sidecar_dirs, ".srt", "SRT", "srt")
        )
        audio_path = (
            os.path.abspath(os.path.expanduser(args.audio))
            if args.audio
            else _discover_single_file(sidecar_dirs, _AUDIO_SUFFIXES, "audio", "audio")
        )
    except (FileNotFoundError, FileExistsError, OSError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    if not os.path.isfile(srt_path):
        print(f"Error: SRT not found: {srt_path}", file=sys.stderr)
        return 2
    if not os.path.isfile(audio_path):
        print(f"Error: audio not found: {audio_path}", file=sys.stderr)
        return 2

    try:
        cues = parse_srt(srt_path)
        exif = _exiftool_audio_metadata(audio_path)
    except (ValueError, RuntimeError, json.JSONDecodeError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    audio_duration_sec = _parse_duration_seconds(exif.get("Duration"))

    bag_start_ns, bag_dur_ns = _bag_metadata_start_duration_ns(bag_dir)
    bag_start_source = "metadata.yaml"
    if bag_start_ns is None:
        bag_start_ns = _bag_start_from_folder_name(bag_dir, args.timezone)
        bag_start_source = "folder name argo_YYYYMMDD_HHMMSS"
    if bag_start_ns is None:
        print(
            "Error: could not determine bag start time from metadata.yaml or folder name.",
            file=sys.stderr,
        )
        return 2

    try:
        sync_anchor_ns, sync_anchor_key = _resolve_sync_anchor_ns(
            audio_path,
            srt_path,
            exif,
            args.timezone,
            args.audio_anchor,
            args.audio_start,
            bag_start_ns,
            bag_dir,
        )
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    phone_filename_ns = _audio_start_from_filename(audio_path, args.timezone)
    if phone_filename_ns is not None:
        phone_filename_ns = phone_filename_ns[0]
    bag_folder_ns = _bag_start_from_folder_name(bag_dir, args.timezone)

    bag_end_ns = (bag_start_ns + bag_dur_ns) if bag_dur_ns is not None else None
    window_start_ns, window_end_ns, _ = _window_bounds_ns(
        args.time_window, sync_anchor_ns, bag_start_ns, bag_end_ns
    )

    _print_sync_report(
        bag_dir=bag_dir,
        bag_start_ns=bag_start_ns,
        bag_start_source=bag_start_source,
        bag_dur_ns=bag_dur_ns,
        audio_path=audio_path,
        srt_path=srt_path,
        sync_anchor_ns=sync_anchor_ns,
        sync_anchor_key=sync_anchor_key,
        phone_filename_ns=phone_filename_ns,
        bag_folder_ns=bag_folder_ns,
        audio_duration_sec=audio_duration_sec,
        offset_sec=args.offset_sec,
        cues=cues,
        tz_name=args.timezone,
        exif=exif,
        time_window=args.time_window,
    )

    if args.dry_run:
        print("Dry run: no output written.")
        return 0

    output_dir = os.path.abspath(os.path.expanduser(args.output))
    if os.path.abspath(bag_dir) == output_dir:
        print("Error: input and output paths must differ.", file=sys.stderr)
        return 2
    if os.path.exists(output_dir):
        if args.force:
            shutil.rmtree(output_dir)
        else:
            print(
                f"Error: output exists: {output_dir}\nRemove it or pass --force.",
                file=sys.stderr,
            )
            return 2

    parent = os.path.dirname(output_dir)
    if parent:
        os.makedirs(parent, exist_ok=True)

    storage_id = _read_storage_id(bag_dir)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    if bag_end_ns is None:
        print(
            "Warning: bag duration unknown; all SRT cues will be written (no time filter).",
            file=sys.stderr,
        )
    narration, skipped_cues = _build_narration_messages(
        cues,
        sync_anchor_ns,
        args.offset_sec,
        window_start_ns,
        window_end_ns,
    )
    if skipped_cues:
        print(f"Omitted {skipped_cues} cues outside {args.time_window} write window.")

    reader = _open_reader(bag_dir, storage_id)
    topics = reader.get_all_topics_and_types()
    topic_names = {t.name for t in topics}

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_dir, storage_id=storage_id),
        converter_options,
    )
    for meta in topics:
        writer.create_topic(meta)
    if NARRATION_TOPIC not in topic_names:
        writer.create_topic(
            TopicMetadata(
                name=NARRATION_TOPIC,
                type=NARRATION_TYPE,
                serialization_format="cdr",
            )
        )

    bag_count, narr_count = _merge_write_bag(reader, writer, narration)
    del reader
    writer.close()

    print(f"Output: {output_dir}")
    print(
        f"Wrote {bag_count} original messages and {narr_count} /narration messages "
        f"({len(cues)} SRT cues parsed, {skipped_cues} outside {args.time_window} window omitted)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
