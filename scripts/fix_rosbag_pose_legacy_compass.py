#!/usr/bin/env python3
"""
Fix legacy /pose in ROS 2 bags (BNO085 recordings before pose/convention fix).

Background
----------
The Python bridge used to publish the same compass heading on both `/compass`
and `/pose`. Argo's `argo_transform_publisher` and simulator bridge expect
`/pose.z` to be **ENU mathematical yaw**: 0° = East, increasing counter-clockwise
when viewed from above (same as `(450 - compass_deg) % 360`).

This script rewrites only `/pose` (geometry_msgs/msg/Vector3): sets
    z_new = (450.0 - z_old) % 360.0
treating z_old as compass degrees. All other topics and timestamps are copied
unchanged.

What this does *not* fix
-------------------------
- **Recorded /tf** (if present): transforms were computed with the old wrong yaw;
  they remain wrong unless you re-record (e.g. `scripts/argo_bag_rerecord.py` via
  `ros2 launch`) or
  add a separate TF rewriter.
- **Recorded visualization markers** (`/visualization_marker`, MarkerArray, etc.):
  geometry baked at record time is unchanged.
- **Simulator bags** where `/pose.z` was already mathematical: do not run this
  script on those bags (it would corrupt pose). If you mistake-run once, run the
  script again: applying the same map twice restores the original z.

Usage
-----
  source /opt/ros/humble/setup.bash
  python3 scripts/fix_rosbag_pose_legacy_compass.py INPUT_BAG_DIR OUTPUT_BAG_DIR

  python3 scripts/fix_rosbag_pose_legacy_compass.py INPUT_BAG_DIR OUTPUT_BAG_DIR --dry-run
  python3 scripts/fix_rosbag_pose_legacy_compass.py INPUT_BAG_DIR OUTPUT_BAG_DIR --force
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from geometry_msgs.msg import Vector3
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
    )
except ImportError:
    print(
        "Error: rosbag2_py not available (install ROS 2 rosbag2 packages).",
        file=sys.stderr,
    )
    sys.exit(1)


POSE_TOPIC_DEFAULT = "/pose"
POSE_TYPE = "geometry_msgs/msg/Vector3"


def _resolve_bag_directory(path: str) -> str:
    """Return rosbag2 directory URI (folder containing metadata.yaml or .db3)."""
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


def _scan_pose(reader: SequentialReader, pose_topic: str) -> tuple[int, int, list[tuple[float, float]]]:
    """Return (total_messages, pose_message_count, sample_old_new_z_deg)."""
    total = 0
    pose_n = 0
    samples: list[tuple[float, float]] = []
    while reader.has_next():
        topic, data, _ts = reader.read_next()
        total += 1
        if topic != pose_topic:
            continue
        pose_n += 1
        msg = deserialize_message(data, Vector3)
        old_z = float(msg.z) % 360.0
        new_z = (450.0 - old_z) % 360.0
        if len(samples) < 5:
            samples.append((old_z, new_z))
    return total, pose_n, samples


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Rewrite /pose.z in a rosbag2 from legacy compass-on-pose to ENU math yaw."
        )
    )
    parser.add_argument(
        "input_bag",
        help="Path to input rosbag2 folder (or path to .db3/.mcap inside it)",
    )
    parser.add_argument(
        "output_bag",
        help="Path to output rosbag2 folder (must not exist unless --force)",
    )
    parser.add_argument(
        "--pose-topic",
        default=POSE_TOPIC_DEFAULT,
        help=f"Topic name to rewrite (default: {POSE_TOPIC_DEFAULT})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Scan input only; print counts and sample conversions; do not write",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Remove output directory if it already exists before writing",
    )
    args = parser.parse_args()

    input_dir = _resolve_bag_directory(args.input_bag)
    output_dir = os.path.abspath(os.path.expanduser(args.output_bag))

    if os.path.abspath(input_dir) == output_dir:
        print("Error: input and output paths must differ.", file=sys.stderr)
        return 2

    storage_id = _read_storage_id(input_dir)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = _open_reader(input_dir, storage_id)
    topics = reader.get_all_topics_and_types()
    topic_names = {t.name for t in topics}
    if args.pose_topic not in topic_names:
        print(
            f"Error: topic {args.pose_topic!r} not in bag. Topics: {sorted(topic_names)}",
            file=sys.stderr,
        )
        return 2
    pose_meta = next(t for t in topics if t.name == args.pose_topic)
    if pose_meta.type != POSE_TYPE:
        print(
            f"Error: {args.pose_topic} has type {pose_meta.type!r}, expected {POSE_TYPE}.",
            file=sys.stderr,
        )
        return 2

    total_msgs, pose_n, pose_samples = _scan_pose(reader, args.pose_topic)
    del reader

    print(f"Input:  {input_dir}")
    print(f"Storage: {storage_id}")
    print(f"Total messages: {total_msgs}")
    print(f"{args.pose_topic} messages: {pose_n}")
    if pose_samples:
        print("Sample z conversions (old → new degrees):")
        for old_z, new_z in pose_samples:
            print(f"  {old_z:.3f} → {new_z:.3f}")

    if args.dry_run:
        print("Dry run: no output written.")
        return 0

    if os.path.exists(output_dir):
        if args.force:
            shutil.rmtree(output_dir)
        else:
            print(
                f"Error: output exists: {output_dir}\n"
                "Remove it or pass --force.",
                file=sys.stderr,
            )
            return 2

    parent = os.path.dirname(output_dir)
    if parent:
        os.makedirs(parent, exist_ok=True)

    reader = _open_reader(input_dir, storage_id)
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_dir, storage_id=storage_id),
        converter_options,
    )
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)

    n_out = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == args.pose_topic:
            msg = deserialize_message(data, Vector3)
            old_z = float(msg.z) % 360.0
            msg.z = (450.0 - old_z) % 360.0
            data = serialize_message(msg)
        writer.write(topic, data, timestamp)
        n_out += 1

    del reader
    writer.close()
    print(f"Output: {output_dir}")
    print(f"Wrote {n_out} messages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
