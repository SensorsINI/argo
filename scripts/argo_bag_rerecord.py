#!/usr/bin/env python3
"""
Argo Bag Re-recording with Visualization Markers
==================================================

Re-records an existing bag file with visualization markers added. This allows you to:
1. Process expensive visualization nodes once on a workstation
2. Import the resulting bag directly into Foxglove Studio
3. Get full visualization without running expensive nodes

The script:
- Plays back the original bag file with --clock option to publish simulated time
- Runs visualization nodes (argo_boat_visualization, argo_transform_publisher, sailing_area_publisher)
  - All nodes use use_sim_time:=true to preserve original timestamps
- Records everything (original topics + visualization markers) to a new bag file
- By default, omits from playback marker topics that are re-published live (boat
  visualization + sailing area markers), so the output bag does not duplicate them
- Uses MCAP format by default (configurable via nodes/record.yaml)
- Preserves original timestamps even when playing back at high speed (e.g., 100x)
- Automatically stops recording when playback completes
- Geofence map name is read from nodes/argo.yaml (/** ros__parameters geofence_map_name), same as
  the robot; with use_sailing_area true, an unknown name fails launch before playback.
  Sim time is passed with -p (not a second params YAML) so /** parameters are not replaced.

Optional time trim: set ``trim_start`` and ``trim_end`` to keep only messages whose
**bag log timestamps** fall in that window. Accepts ISO-8601 (recommended) or
12-hour wall clock with ``trim_timezone`` (default ``Europe/Zurich``). You can also
pass two **plain floating-point seconds** (seconds from the bag start per
``metadata.yaml``) instead of wall-clock strings. A temporary bag is written under
``bags/argo_rerecord_trim_<uuid>/`` (the directory must not pre-exist; empty stale
trim dirs are removed before creating a new one) and is deleted when the launch exits.

Note: The --clock option and use_sim_time ensure that re-recorded bags maintain
the original time scale, even when playback is accelerated for faster processing.

Usage (from repo root, with ROS 2 sourced):
    ros2 launch ./scripts/argo_bag_rerecord.py input_bag:=/path/to/original/bag output_bag:=output_name
    ros2 launch ./scripts/argo_bag_rerecord.py input_bag:=bags/argo_20251105_141014/ output_bag:=argo_20251105_141014_with_viz
    ros2 launch ./scripts/argo_bag_rerecord.py input_bag:=bags/foo output_bag:=foo_viz \\
      trim_start:="2026-05-08T16:32:33.466+02:00" trim_end:="2026-05-08T17:00:00+02:00"

Or use the same line with `python3 .../argo_bag_rerecord.py` (see --help).
Convenience wrapper: `scripts/argo_rerecord_bag.sh`.

CLI help (launch arguments and usage):
    python3 scripts/argo_bag_rerecord.py --help

If you run this file directly with launch-style arguments (containing ':='), it
re-invokes "ros2 launch" on itself (same as typing ros2 launch ...).

ROS 2 can also list declared arguments with:
    ros2 launch ./scripts/argo_bag_rerecord.py --show-args
"""

import os
import re
import shlex
import shutil
import uuid
import glob
import yaml
import tempfile
import atexit
import subprocess
import sys
from typing import Optional, Tuple
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime
from zoneinfo import ZoneInfo


def _print_help_message() -> None:
    """Print usage for direct invocation: python3 argo_bag_rerecord.py --help"""
    prog = os.path.basename(__file__)
    print(
        f"""Usage:
  ros2 launch ./scripts/{prog} input_bag:=PATH [launch-args...]
  ros2 launch /path/to/argo/scripts/{prog} input_bag:=PATH [launch-args...]

  python3 scripts/{prog} --help

Re-record a rosbag2: play the input bag with /clock, run visualization and transform
nodes, and record a new bag under bags/ (see module docstring at top of file).

Launch arguments (pass as name:=value):
  input_bag         Path to input rosbag2 directory (required for a useful run)
  output_bag        Output bag name only; written under bags/ (default: timestamped)
  use_sailing_area  true|false - sailing_area_publisher (default: true)
  use_visualization true|false - argo_boat_visualization (default: true)
  use_transform     true|false - argo_transform_publisher (default: true)
  playback_rate     Bag play rate multiplier, e.g. 100.0 (default: 100.0)
  storage_format      mcap|sqlite3 (default: from nodes/record.yaml)

  exclude_recorded_markers true|false - omit bag topics re-published live (default: true)
  trim_start          Start of log-time window (see below) or empty
  trim_end            End of log-time window (or empty to disable trim)
  trim_timezone       IANA zone for naive wall times (default: Europe/Zurich)

  Map: sailing_area_publisher and argo_transform_publisher load nodes/argo.yaml (geofence_map_name).
  With use_sailing_area true, an unknown map name fails launch before bag play.

  Trim: both trim_start and trim_end required to enable. Either:
    - ISO-8601: 2026-05-08T16:32:33.466+02:00
    - Naive + trim_timezone: "2026-05-08 4:32:33.466 PM" (suffix CEST/CET is stripped)
    - Seconds from bag start: 120.0 and 3600.0 (requires metadata.yaml in the bag)

Examples:
  ros2 launch ./scripts/{prog} \\
    input_bag:=bags/my_recording output_bag:=my_recording_with_viz

  python3 scripts/{prog} input_bag:=PATH output_bag:=NAME
    (same as: ros2 launch .../scripts/{prog} input_bag:=... output_bag:=...)

List arguments from ros2:
  ros2 launch ./scripts/{prog} --show-args
"""
    )


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        _print_help_message()
        raise SystemExit(0)
    launch_file = os.path.abspath(__file__)
    argv_rest = sys.argv[1:]
    if any(":=" in a for a in argv_rest):
        try:
            os.execvp("ros2", ["ros2", "launch", launch_file, *argv_rest])
        except FileNotFoundError:
            print(
                "ros2 not found in PATH. Source ROS 2, e.g.: source /opt/ros/humble/setup.bash",
                file=sys.stderr,
            )
            raise SystemExit(127)
    _prog = os.path.basename(__file__)
    print(
        f"{_prog}: pass launch arguments as name:=value (see --help), e.g.\n"
        f"  ros2 launch {launch_file} input_bag:=bags/my_bag output_bag:=out_name\n"
        f"  python3 {launch_file} input_bag:=bags/my_bag output_bag:=out_name",
        file=sys.stderr,
    )
    raise SystemExit(2)


# Topics published live during re-record; omit from bag play when exclude_recorded_markers
# is true and the corresponding node is enabled (avoids duplicate marker streams in output).
BOAT_VIZ_TOPICS_RECORDED = frozenset({
    '/visualization_marker',
    '/visualization_marker_array',
    '/visualization/debug/wind_direction_calculated',
})

SAILING_MARKER_TOPICS_RECORDED = frozenset({
    '/sailing_waypoints',
    '/sailing_boundaries',
    '/sailing_hazards',
})


def _launch_truthy(value: str) -> bool:
    return str(value).lower() in ('true', '1', 'yes', 'on')


def _load_geofence_map_name_from_argo_yaml(argo_yaml_path: str) -> Optional[str]:
    """Read /** ros__parameters geofence_map_name from nodes/argo.yaml (same source as the robot)."""
    try:
        with open(argo_yaml_path, 'r', encoding='utf-8') as f:
            root = yaml.safe_load(f)
        if not root or not isinstance(root, dict):
            return None
        gp = root.get('/**')
        if not isinstance(gp, dict):
            return None
        rp = gp.get('ros__parameters', {})
        if not isinstance(rp, dict):
            return None
        name = rp.get('geofence_map_name')
        if name is None:
            return None
        s = str(name).strip()
        return s or None
    except (OSError, yaml.YAMLError, TypeError):
        return None


def _validate_geofence_map_geojson_exists(map_name: str, argo_dir: str) -> None:
    """Raise RuntimeError if map_name does not match any foxglove/maps/*.geojson stem (case-insensitive)."""
    maps_dir = os.path.join(argo_dir, 'foxglove', 'maps')
    stems = sorted(
        {os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(maps_dir, '*.geojson'))}
    )
    if not stems:
        raise RuntimeError(
            f"No .geojson maps under {maps_dir}. "
            f"Cannot use geofence_map_name {map_name!r} from nodes/argo.yaml."
        )
    lower_index = {s.lower(): s for s in stems}
    if map_name.lower() not in lower_index:
        example = os.path.join(maps_dir, f'{map_name}.geojson')
        avail = ', '.join(stems)
        raise RuntimeError(
            f"geofence_map_name {map_name!r} from nodes/argo.yaml does not match any map. "
            f"Keys are GeoJSON basenames under {maps_dir}. "
            f"Example file: {example}. Loaded maps: {avail}"
        )


def _read_bag_storage_id(bag_dir: str) -> str:
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.isfile(meta_path):
        return ''
    try:
        with open(meta_path, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        info = meta.get('rosbag2_bagfile_information') or {}
        sid = info.get('storage_identifier')
        if isinstance(sid, str) and sid:
            return sid
    except (OSError, yaml.YAMLError, TypeError):
        pass
    return ''


def _list_bag_topic_names(bag_dir: str) -> list[str]:
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    except ImportError:
        return []
    storage_id = _read_bag_storage_id(bag_dir)
    candidates = []
    if storage_id:
        candidates.append(storage_id)
    candidates.extend(['sqlite3', 'mcap', ''])
    order = []
    seen = set()
    for c in candidates:
        if c not in seen:
            seen.add(c)
            order.append(c)
    co = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    last_err = None
    for sid in order:
        reader = SequentialReader()
        try:
            reader.open(StorageOptions(uri=bag_dir, storage_id=sid), co)
            return sorted({t.name for t in reader.get_all_topics_and_types()})
        except Exception as e:
            last_err = e
        finally:
            try:
                del reader
            except Exception:
                pass
    print(
        f'Warning: could not list bag topics ({last_err}); playing all topics.',
        file=sys.stderr,
    )
    return []


def _resolve_input_bag_path(input_bag: str, argo_dir: str, bags_dir: str) -> str:
    if not input_bag or not str(input_bag).strip():
        raise ValueError('input_bag argument is required')
    bag = str(input_bag).strip()
    if not os.path.isabs(bag):
        if os.path.exists(os.path.join(bags_dir, bag)):
            bag = os.path.join(bags_dir, bag)
        elif os.path.exists(os.path.join(argo_dir, bag)):
            bag = os.path.join(argo_dir, bag)
        else:
            bag = os.path.abspath(bag)
    if not os.path.exists(bag):
        raise FileNotFoundError(f'Input bag not found: {bag}')
    if not os.path.isdir(bag):
        raise ValueError(f'Input bag path is not a rosbag2 directory: {bag}')
    return bag


def _bag_metadata_start_duration_ns(bag_dir: str) -> Tuple[Optional[int], Optional[int]]:
    """Return (starting_time_ns, duration_ns) from metadata.yaml if present."""
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.isfile(meta_path):
        return None, None
    try:
        with open(meta_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        info = data.get('rosbag2_bagfile_information') or {}
        st = info.get('starting_time')
        dur = info.get('duration')
        start_ns = None
        if isinstance(st, dict):
            start_ns = st.get('nanoseconds_since_epoch')
            if start_ns is None:
                start_ns = st.get('nanoseconds')
        elif isinstance(st, (int, float)):
            start_ns = int(st)
        dur_ns = None
        if isinstance(dur, dict):
            dur_ns = dur.get('nanoseconds')
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
        r'\s+(CEST|CET|BST|EEST|EET|GMT|UTC)\s*$',
        '',
        s.strip(),
        flags=re.IGNORECASE,
    ).strip()


def _parse_trim_time_to_ns(s: str, tz_name: str) -> int:
    """Parse a single wall-clock time to Unix epoch nanoseconds (bag log time scale)."""
    raw = _strip_optional_zone_suffix(s)
    iso_candidate = raw.replace('Z', '+00:00')
    if 'T' not in iso_candidate and re.match(r'^\d{4}-\d{2}-\d{2}\s', iso_candidate):
        iso_candidate = re.sub(r'^(\d{4}-\d{2}-\d{2})\s+', r'\1T', iso_candidate, count=1)
    try:
        dt = datetime.fromisoformat(iso_candidate)
        if dt.tzinfo is None:
            if not tz_name:
                raise ValueError('Naive datetime requires trim_timezone (e.g. Europe/Zurich)')
            dt = dt.replace(tzinfo=ZoneInfo(tz_name))
        return int(dt.timestamp() * 1e9)
    except ValueError:
        pass
    for fmt in (
        '%Y-%m-%d %I:%M:%S.%f %p',
        '%Y-%m-%d %I:%M:%S %p',
        '%Y-%m-%d %H:%M:%S.%f',
        '%Y-%m-%d %H:%M:%S',
    ):
        try:
            dt_naive = datetime.strptime(raw, fmt)
            if not tz_name:
                raise ValueError('trim_timezone required for naive wall-clock times')
            dt = dt_naive.replace(tzinfo=ZoneInfo(tz_name))
            return int(dt.timestamp() * 1e9)
        except ValueError:
            continue
    raise ValueError(
        f'Could not parse trim time {s!r}. Use ISO-8601 (e.g. 2026-05-08T16:32:33.466+02:00) '
        f'or wall clock with AM/PM plus trim_timezone (default Europe/Zurich).'
    )


def _trim_strings_are_seconds_from_bag_start(trim_start: str, trim_end: str) -> bool:
    pat = re.compile(r'^-?\d+(\.\d+)?([eE][-+]?\d+)?$')
    return bool(pat.match(trim_start.strip()) and pat.match(trim_end.strip()))


def _parse_trim_pair_to_ns(trim_start: str, trim_end: str, tz_name: str, bag_dir: str) -> tuple[int, int]:
    ts = trim_start.strip()
    te = trim_end.strip()
    if _trim_strings_are_seconds_from_bag_start(ts, te):
        b_start, _b_dur = _bag_metadata_start_duration_ns(bag_dir)
        if b_start is None:
            raise ValueError(
                'trim_start/trim_end look like seconds-from-bag-start, but metadata.yaml '
                'has no starting_time; use wall-clock ISO times instead.'
            )
        t0 = b_start + int(float(ts) * 1e9)
        t1 = b_start + int(float(te) * 1e9)
        return t0, t1
    return _parse_trim_time_to_ns(ts, tz_name), _parse_trim_time_to_ns(te, tz_name)


def _open_sequential_reader(bag_dir: str):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    co = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    storage_id = _read_bag_storage_id(bag_dir)
    candidates = []
    if storage_id:
        candidates.append(storage_id)
    candidates.extend(['sqlite3', 'mcap', ''])
    order = []
    seen = set()
    for c in candidates:
        if c not in seen:
            seen.add(c)
            order.append(c)
    last_err = None
    for sid in order:
        reader = SequentialReader()
        try:
            reader.open(StorageOptions(uri=bag_dir, storage_id=sid), co)
            return reader, sid
        except Exception as e:
            last_err = e
            try:
                del reader
            except Exception:
                pass
    raise RuntimeError(f'Could not open bag {bag_dir!r}: {last_err}')


def _remove_empty_trim_workdirs(bags_dir: str) -> None:
    """Remove leftover empty argo_rerecord_trim_* dirs (e.g. failed mkdtemp + writer)."""
    if not bags_dir or not os.path.isdir(bags_dir):
        return
    pattern = os.path.join(bags_dir, 'argo_rerecord_trim_*')
    for p in glob.glob(pattern):
        try:
            if os.path.isdir(p) and not os.listdir(p):
                shutil.rmtree(p, ignore_errors=True)
        except OSError:
            pass


def _copy_bag_time_window(src_dir: str, dst_dir: str, t0_ns: int, t1_ns: int) -> int:
    """Copy messages whose bag log timestamp is in [t0_ns, t1_ns]. Returns number of messages written."""
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions

    # SequentialWriter requires dst_dir to NOT exist yet (same as ros2 bag record).
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, ignore_errors=True)

    reader, sid = _open_sequential_reader(src_dir)
    co = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    topics = reader.get_all_topics_and_types()
    writer = SequentialWriter()
    n_written = 0
    try:
        writer.open(StorageOptions(uri=dst_dir, storage_id=sid), co)
        for tm in topics:
            writer.create_topic(tm)
        while reader.has_next():
            topic, data, ts = reader.read_next()
            if t0_ns <= ts <= t1_ns:
                writer.write(topic, data, ts)
                n_written += 1
    except Exception:
        try:
            writer.close()
        except Exception:
            pass
        try:
            del reader
        except Exception:
            pass
        shutil.rmtree(dst_dir, ignore_errors=True)
        raise
    try:
        del reader
    except Exception:
        pass
    writer.close()
    return n_written


def check_mcap_plugin_available():
    """Check if MCAP storage plugin is available for ros2 bag record"""
    try:
        # Try to get help output which lists available storage plugins
        result = subprocess.run(
            ['ros2', 'bag', 'record', '--help'],
            capture_output=True,
            text=True,
            timeout=5
        )
        # Check if 'mcap' appears in the help output as a storage option
        if 'mcap' in result.stdout or 'mcap' in result.stderr:
            return True
        
        # Alternative: Try to check if the plugin package is installed
        # This works by checking if ros2 bag record accepts -s mcap without error
        test_result = subprocess.run(
            ['ros2', 'bag', 'record', '-s', 'mcap', '--help'],
            capture_output=True,
            text=True,
            timeout=5
        )
        # If it doesn't error with "invalid choice", the plugin is available
        if 'invalid choice' not in test_result.stderr.lower():
            return True
        
        return False
    except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
        # If ros2 command not found or times out, assume not available
        return False


def validate_and_print_paths(context):
    """Validate paths and print output bag location"""
    try:
        input_bag = context.launch_configurations.get('input_bag', '')
        output_bag = context.launch_configurations.get('output_bag', '')

        argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        bags_dir = os.path.join(argo_dir, 'bags')

        input_bag = _resolve_input_bag_path(input_bag, argo_dir, bags_dir)

        if not os.path.exists(bags_dir):
            error_msg = f"❌ Error: Bags directory does not exist: {bags_dir}"
            print(error_msg, file=sys.stderr)
            raise FileNotFoundError(error_msg)

        output_bag_full_path = os.path.join(bags_dir, output_bag)

        print(f"📦 Creating output bag at: {output_bag_full_path}", file=sys.stdout)
        print(f"   Input bag:  {input_bag}", file=sys.stdout)
        print(f"   Output bag: {output_bag_full_path}", file=sys.stdout)

        if os.path.exists(output_bag_full_path):
            print(
                f"⚠️  Warning: Output bag already exists and will be overwritten: {output_bag_full_path}",
                file=sys.stderr,
            )

        use_sail = _launch_truthy(context.launch_configurations.get('use_sailing_area', 'true'))
        if use_sail:
            argo_yaml_path = os.path.join(argo_dir, 'nodes', 'argo.yaml')
            geofence = _load_geofence_map_name_from_argo_yaml(argo_yaml_path)
            if geofence:
                _validate_geofence_map_geojson_exists(geofence, argo_dir)

        return []
    except Exception as e:
        error_msg = f"❌ Error in path validation: {str(e)}"
        print(error_msg, file=sys.stderr)
        import traceback
        print(traceback.format_exc(), file=sys.stderr)
        raise


def generate_launch_description():
    """Generate launch description for bag re-recording with visualization"""
    
    try:
        # Get Argo directory
        argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # Get bags directory (use project's bags/ directory, same as bash script)
        bags_dir = os.path.join(argo_dir, 'bags')
        trimmed_bag_cleanup = {'path': None}

        # Geofence / map name: nodes/argo.yaml (/** ros__parameters geofence_map_name), same as lifecycle launch.
        argo_yaml_path = os.path.join(argo_dir, 'nodes', 'argo.yaml')
        geofence_map_name = _load_geofence_map_name_from_argo_yaml(argo_yaml_path)

        # Load recording configuration from record.yaml (same as record.py)
        record_config_path = os.path.join(argo_dir, 'nodes', 'record.yaml')
        storage_format = 'mcap'  # Default
        mcap_config = {}
        preset_profile = None
        
        try:
            with open(record_config_path, 'r') as f:
                config = yaml.safe_load(f)
                storage_format = config.get('storage_format', 'mcap')
                if storage_format not in ['mcap', 'sqlite3']:
                    storage_format = 'mcap'
                mcap_config = config.get('mcap', {})
                preset_profile = config.get('preset_profile', None)
        except (FileNotFoundError, Exception):
            # Use defaults if config file not found or invalid
            pass
        
        # Check if MCAP plugin is available when MCAP format is requested
        if storage_format == 'mcap':
            if not check_mcap_plugin_available():
                error_msg = """
❌ ERROR: MCAP storage plugin is not installed!

The bag re-recording requires the MCAP storage plugin for ros2 bag record.
Install it with:

    source /opt/ros/humble/setup.bash
    make install-rosbag2-mcap

Or install all ROS2 dependencies:

    source /opt/ros/humble/setup.bash
    make install-deps

After installation, try the re-recording again.
"""
                print(error_msg, file=sys.stderr)
                sys.exit(1)
        
        # Launch arguments
        input_bag_arg = DeclareLaunchArgument(
            'input_bag',
            description='Path to input bag file to play back',
            default_value=''
        )
        
        output_bag_arg = DeclareLaunchArgument(
            'output_bag',
            description='Name for output bag (without path, will be saved to bags/)',
            default_value=PythonExpression([
                "'argo_rerecord_', str('", datetime.now().strftime('%Y%m%d_%H%M%S'), "')"
            ])
        )
        
        use_sailing_area_arg = DeclareLaunchArgument(
            'use_sailing_area',
            default_value='true',
            description='Launch sailing_area_publisher for boundaries/waypoints'
        )
        
        use_visualization_arg = DeclareLaunchArgument(
            'use_visualization',
            default_value='true',
            description='Launch visualization node to recreate markers'
        )
        
        use_transform_arg = DeclareLaunchArgument(
            'use_transform',
            default_value='true',
            description='Launch transform publisher for /tf transforms'
        )
        
        playback_rate_arg = DeclareLaunchArgument(
            'playback_rate',
            default_value='100.0',
            description='Playback rate multiplier (1.0 = realtime, higher = faster). Maximum rate for fastest processing.'
        )
        
        storage_format_arg = DeclareLaunchArgument(
            'storage_format',
            default_value=storage_format,
            description='Storage format for output bag: "mcap" or "sqlite3" (defaults to record.yaml config)'
        )

        exclude_recorded_markers_arg = DeclareLaunchArgument(
            'exclude_recorded_markers',
            default_value='true',
            description=(
                'If true, omit from bag play marker topics that are re-published live '
                '(when use_visualization / use_sailing_area are true), avoiding duplicates in output'
            ),
        )

        trim_start_arg = DeclareLaunchArgument(
            'trim_start',
            default_value='',
            description=(
                'Start of bag log-time window (ISO-8601, wall clock with trim_timezone, or seconds from bag start); '
                'must set trim_end as well'
            ),
        )
        trim_end_arg = DeclareLaunchArgument(
            'trim_end',
            default_value='',
            description='End of bag log-time window (same formats as trim_start)',
        )
        trim_timezone_arg = DeclareLaunchArgument(
            'trim_timezone',
            default_value='Europe/Zurich',
            description='IANA timezone for naive wall-clock trim_start / trim_end (e.g. Europe/Zurich)',
        )

        def prepare_trimmed_input_if_needed(context):
            ts = str(context.launch_configurations.get('trim_start', '') or '').strip()
            te = str(context.launch_configurations.get('trim_end', '') or '').strip()
            if not ts and not te:
                return []
            if not ts or not te:
                raise ValueError('Set both trim_start and trim_end, or leave both empty.')
            tz = str(context.launch_configurations.get('trim_timezone', 'Europe/Zurich') or '').strip()
            if not tz:
                tz = 'Europe/Zurich'
            try:
                ZoneInfo(tz)
            except Exception as e:
                raise ValueError(f'Invalid trim_timezone {tz!r}: {e}') from e
            raw_input = str(context.launch_configurations.get('input_bag', '') or '').strip()
            src = _resolve_input_bag_path(raw_input, argo_dir, bags_dir)
            t0_ns, t1_ns = _parse_trim_pair_to_ns(ts, te, tz, src)
            b_start, b_dur = _bag_metadata_start_duration_ns(src)
            if b_start is not None and b_dur is not None:
                bag_end = b_start + b_dur
                t0_ns = max(t0_ns, b_start)
                t1_ns = min(t1_ns, bag_end)
            if t0_ns >= t1_ns:
                raise ValueError(
                    f'Invalid trim window after clamp to bag span: start_ns={t0_ns} end_ns={t1_ns}'
                )
            _remove_empty_trim_workdirs(bags_dir)
            for _ in range(16):
                tmp = os.path.join(bags_dir, f'argo_rerecord_trim_{uuid.uuid4().hex}')
                if not os.path.exists(tmp):
                    break
            else:
                raise RuntimeError('Could not allocate a unique trim temp directory under bags/')
            try:
                n = _copy_bag_time_window(src, tmp, t0_ns, t1_ns)
            except Exception:
                shutil.rmtree(tmp, ignore_errors=True)
                raise
            if n == 0:
                shutil.rmtree(tmp, ignore_errors=True)
                raise ValueError(
                    'Trim produced no messages. Check trim_start/trim_end against bag log times '
                    'and trim_timezone (bag timestamps are Unix epoch ns in metadata).'
                )
            trimmed_bag_cleanup['path'] = tmp
            context.launch_configurations['input_bag'] = tmp
            print(f'Trim: copied {n} messages to temporary bag {tmp}', flush=True)
            print(f'      log-time window (ns): [{t0_ns}, {t1_ns}]', flush=True)
            return []

        # Build ros2 bag record command with storage format support
        # Start with base command
        # Use --use-sim-time to make ros2 bag record use /clock topic for log time
        # This ensures Foxglove "log time" (when message was recorded) matches original robot time
        # Without --use-sim-time, ros2 bag record uses wall-clock time for log time
        # Note: ros2 bag record will wait for /clock messages before recording starts
        record_base_cmd = 'ros2 bag record -a --include-hidden-topics --use-sim-time -o "$1"'
        
        # Add storage format option based on config
        # For MCAP, we'll create a temporary config file with unique name
        mcap_config_file = None
        if storage_format == 'mcap':
            record_base_cmd += ' -s mcap'
            if preset_profile:
                # Use preset profile (no config file needed)
                record_base_cmd += f' --storage-preset-profile {preset_profile}'
            elif mcap_config:
                # Create temporary config file with unique name (will be cleaned up)
                try:
                    # Use tempfile to get a unique filename, but create in bags_dir
                    fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix='mcap_config_', dir=bags_dir, text=True)
                    mcap_config_file = temp_path
                    with os.fdopen(fd, 'w') as f:
                        yaml.dump(mcap_config, f, default_flow_style=False)
                    record_base_cmd += f' --storage-config-file {shlex.quote(mcap_config_file)}'
                    # Register cleanup function for this file
                    def cleanup_config_file():
                        if os.path.exists(mcap_config_file):
                            try:
                                os.remove(mcap_config_file)
                            except Exception:
                                pass
                    atexit.register(cleanup_config_file)
                except Exception:
                    # If config file creation fails, continue without it (uses MCAP defaults)
                    if mcap_config_file and os.path.exists(mcap_config_file):
                        try:
                            os.remove(mcap_config_file)
                        except Exception:
                            pass
                    mcap_config_file = None
        
        # Bag recording process - records ALL topics (original + visualization)
        # Start recording after a short delay to let nodes initialize
        # Set WARNING log level to reduce verbosity
        # Uses MCAP format by default (configurable from record.yaml)
        # --use-sim-time makes ros2 bag record use /clock topic for log time
        # This ensures Foxglove "log time" (when message was recorded) matches original robot time
        # Original messages from bag play preserve their header stamps automatically
        # New visualization messages use /clock time for header stamps (via our node subscriptions)
        # This preserves original timestamps even when playing at high speed
        bag_record_cmd = f'source /opt/ros/humble/setup.bash && export RCUTILS_LOGGING_SEVERITY=WARNING && {record_base_cmd}'
        
        bag_record = ExecuteProcess(
        cmd=['bash', '-c', bag_record_cmd, '--', LaunchConfiguration('output_bag')],
        cwd=bags_dir,
        output='screen',
        name='bag_recording'
        )
        
        # Create environment dict with WARNING log level for all processes
        # This filters out INFO/DEBUG messages for minimal output during fast playback
        error_log_env = os.environ.copy()
        error_log_env['RCUTILS_LOGGING_SEVERITY'] = 'WARNING'
        
        # use_sim_time via -p so we do not add a second /** params-file that can wipe
        # geofence_map_name from nodes/argo.yaml (launch_ros temp dict becomes its own --params-file).
        _sim_time_ros_args = ['-p', 'use_sim_time:=true']
        # Boat viz emits INFO trace lines (e.g. VIS_SAILING_MERGE_TRACE); quiet for re-record.
        _boat_viz_ros_args = _sim_time_ros_args + ['--log-level', 'warn']

        # Visualization node - recreates markers from source topics in the bag
        # Use simulated time to preserve original timestamps from playback
        visualization_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_boat_visualization.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_visualization')),
        name='argo_boat_visualization',
        ros_arguments=_boat_viz_ros_args,
        env=error_log_env
        )
        
        # Transform publisher - provides /tf transforms for 3D visualization
        # Use simulated time to preserve original timestamps from playback
        transform_node = Node(
        executable='python3',
        arguments=[os.path.join(argo_dir, 'nodes', 'argo_transform_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_transform')),
        name='argo_transform_publisher',
        parameters=[argo_yaml_path],
        ros_arguments=_sim_time_ros_args,
        env=error_log_env
        )
        
        # Sailing area publisher - provides boundaries/waypoints/hazards
        # Use simulated time to preserve original timestamps from bag playback
        # nodes/argo.yaml is the only params file here so /** geofence_map_name is preserved
        sailing_area_node = Node(
            executable='python3',
            arguments=[os.path.join(argo_dir, 'nodes', 'sailing_area_publisher.py')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sailing_area')),
            name='sailing_area_publisher',
            parameters=[argo_yaml_path],
            ros_arguments=_sim_time_ros_args,
            env=error_log_env
        )
        
        # Process to stop all nodes when bag playback completes
        # Also clean up temporary MCAP config file if it exists
        # IMPORTANT: Wait for output bag directory to be created before stopping
        # This ensures recording has actually started and captured data
        cleanup_config = ''
        if mcap_config_file:
            cleanup_config = f'rm -f "{mcap_config_file}" 2>/dev/null || true && '
        
        # Function to create stop process with output_bag from context
        def create_stop_process(context):
            output_bag = context.launch_configurations.get('output_bag', '')
            output_path = os.path.join(bags_dir, output_bag)
            
            stop_cmd = (
            'source /opt/ros/humble/setup.bash && '
            f'OUTPUT_PATH="{output_path}" && '
            'echo "🛑 Bag playback completed, waiting for recording to finish..." && '
            # Wait for output bag directory to be created (max 10 seconds)
            # This ensures recording has actually started
            'WAIT_COUNT=0 && '
            'while [ ! -d "$OUTPUT_PATH" ] && [ $WAIT_COUNT -lt 20 ]; do '
            '  sleep 0.5 && '
            '  WAIT_COUNT=$((WAIT_COUNT + 1)) && '
            '  if [ $((WAIT_COUNT % 4)) -eq 0 ]; then '
            '    echo "   Waiting for recording to start... ($((WAIT_COUNT * 50))ms)" '
            '  fi '
            'done && '
            # If output bag still doesn't exist, wait a bit more and check again
            'if [ ! -d "$OUTPUT_PATH" ]; then '
            '  echo "⚠️  Warning: Output bag directory not found, waiting additional 2 seconds..." && '
            '  sleep 2 && '
            'fi && '
            # Now stop recording gracefully
            'echo "🛑 Stopping recording (allowing time for final writes)..." && '
            # Send SIGTERM to allow graceful shutdown
            'pkill -TERM -f "ros2 bag record" 2>/dev/null || true && '
            # Wait longer for bag to finish writing (MCAP format may need time to flush)
            'sleep 3 && '
            # Force kill if still running
            'pkill -KILL -f "ros2 bag record" 2>/dev/null || true && '
            'sleep 0.5 && '
            'echo "🛑 Stopping visualization nodes..." && '
            # Match python3 processes with the specific node scripts
            'pkill -f "python3.*argo_boat_visualization" 2>/dev/null || true && '
            'pkill -f "python3.*argo_transform_publisher" 2>/dev/null || true && '
            'pkill -f "python3.*sailing_area_publisher" 2>/dev/null || true && '
            # Also try matching by node name in case processes are still running
            'pkill -f "argo_boat_visualization" 2>/dev/null || true && '
            'pkill -f "argo_transform_publisher" 2>/dev/null || true && '
            'pkill -f "sailing_area_publisher" 2>/dev/null || true && '
            'sleep 1 && '
            + cleanup_config +
            'echo "✅ All processes stopped - re-recording complete!"'
            )
            
            return [ExecuteProcess(
            cmd=['bash', '-c', stop_cmd],
            output='screen',
            name='stop_all_processes'
        )]
        
        # Cleanup function for MCAP config file (runs on any shutdown)
        def cleanup_mcap_config(context):
            if mcap_config_file and os.path.exists(mcap_config_file):
                try:
                    os.remove(mcap_config_file)
                except Exception:
                    pass

        def cleanup_trimmed_temp(context):
            p = trimmed_bag_cleanup.get('path')
            if p and os.path.isdir(p):
                shutil.rmtree(p, ignore_errors=True)
                trimmed_bag_cleanup['path'] = None

        def cleanup_all_artifacts(context):
            cleanup_mcap_config(context)
            cleanup_trimmed_temp(context)
            _remove_empty_trim_workdirs(bags_dir)

        def add_bag_playback_actions(context):
            """Build ros2 bag play with --topics so recorded markers are not replayed when live nodes replace them."""
            input_bag_raw = context.launch_configurations.get('input_bag', '')
            playback_rate = context.launch_configurations.get('playback_rate', '100.0')
            input_bag = _resolve_input_bag_path(input_bag_raw, argo_dir, bags_dir)
            exclude_markers = _launch_truthy(
                context.launch_configurations.get('exclude_recorded_markers', 'true'))
            use_viz = _launch_truthy(context.launch_configurations.get('use_visualization', 'true'))
            use_sail = _launch_truthy(context.launch_configurations.get('use_sailing_area', 'true'))
            to_exclude = set()
            if exclude_markers:
                if use_viz:
                    to_exclude.update(BOAT_VIZ_TOPICS_RECORDED)
                if use_sail:
                    to_exclude.update(SAILING_MARKER_TOPICS_RECORDED)
            all_topics = _list_bag_topic_names(input_bag)
            topics_suffix = ''
            if to_exclude and all_topics:
                topics_play = [t for t in all_topics if t not in to_exclude]
                skipped = sorted(to_exclude.intersection(all_topics))
                if skipped:
                    print(
                        'Bag play: omitting '
                        f'{len(skipped)} recorded topic(s) (re-published live): '
                        + ', '.join(skipped),
                        flush=True,
                    )
                if not topics_play:
                    raise RuntimeError(
                        'After excluding recorded marker topics, no topics remain to play. '
                        'Set exclude_recorded_markers:=false, or enable more topics in the input bag.'
                    )
                topics_suffix = ' --topics ' + ' '.join(shlex.quote(t) for t in topics_play)
            elif to_exclude and not all_topics:
                print(
                    'Warning: could not list bag topics; playing entire bag '
                    '(duplicate marker topics possible).',
                    file=sys.stderr,
                )
            inner = (
                'source /opt/ros/humble/setup.bash && '
                'export RCUTILS_LOGGING_SEVERITY=ERROR && '
                'exec ros2 bag play '
                + shlex.quote(input_bag)
                + ' --rate '
                + shlex.quote(str(playback_rate))
                + ' --clock'
                + topics_suffix
            )
            bag_play = ExecuteProcess(
                cmd=['bash', '-c', inner],
                output='screen',
                name='bag_playback',
            )
            start_recording_handler = RegisterEventHandler(
                OnProcessStart(
                    target_action=bag_play,
                    on_start=[
                        TimerAction(
                            period=0.5,
                            actions=[bag_record],
                        ),
                    ],
                )
            )
            stop_all_handler = RegisterEventHandler(
                OnProcessExit(
                    target_action=bag_play,
                    on_exit=[
                        LogInfo(msg='Bag playback completed, waiting for recording to finish...'),
                        OpaqueFunction(function=create_stop_process),
                        TimerAction(
                            period=15.0,
                            actions=[
                                OpaqueFunction(function=cleanup_all_artifacts),
                                Shutdown(reason='Bag playback completed'),
                            ],
                        ),
                    ],
                )
            )
            return [bag_play, start_recording_handler, stop_all_handler]
        
        # Cleanup handler for any shutdown (Ctrl+C, errors, etc.)
        cleanup_on_shutdown = RegisterEventHandler(
            OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=cleanup_all_artifacts)
            ]
        )
        )
        
        return LaunchDescription([
        input_bag_arg,
        output_bag_arg,
        use_sailing_area_arg,
        use_visualization_arg,
        use_transform_arg,
        playback_rate_arg,
        storage_format_arg,
        exclude_recorded_markers_arg,
        trim_start_arg,
        trim_end_arg,
        trim_timezone_arg,
        # Resolve original input/output paths before optional trim rewrites input_bag
        OpaqueFunction(function=validate_and_print_paths),
        OpaqueFunction(function=prepare_trimmed_input_if_needed),
        LogInfo(msg='=== Argo Bag Re-recording with Visualization ==='),
        LogInfo(msg=['Input bag: ', LaunchConfiguration('input_bag')]),
        LogInfo(msg=['Output bag: ', LaunchConfiguration('output_bag')]),
        LogInfo(msg=['Storage format: ', storage_format, ' (from record.yaml config)']),
        LogInfo(msg=['Playback rate: ', LaunchConfiguration('playback_rate'), 'x (maximum speed)']),
        LogInfo(msg=[
            'Geofence map (nodes/argo.yaml geofence_map_name): ',
            geofence_map_name if geofence_map_name else 'unset (node default; transform may use first GPS for origin)',
        ]),
        LogInfo(msg='Recording will start automatically 0.5 seconds after playback begins'),
        LogInfo(msg='All processes will stop automatically when playback completes'),
        visualization_node,
        transform_node,
        sailing_area_node,
        OpaqueFunction(function=add_bag_playback_actions),
        cleanup_on_shutdown,
    ])
    except Exception as e:
        error_msg = f"❌ Error generating launch description: {str(e)}"
        print(error_msg, file=sys.stderr)
        import traceback
        print(traceback.format_exc(), file=sys.stderr)
        # Re-raise to let ros2 launch handle it (will result in nonzero exit)
        raise

