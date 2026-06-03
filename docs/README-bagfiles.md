# Argo rosbag2 recordings and tools

Argo records onboard data as **rosbag2** directories under `~/argo/bags/` (on the Orange Pi) or a synced copy on a workstation (e.g. `~/Dropbox/GitHub/SensorsINI/argo/bags/`). Folders are named `argo_YYYYMMDD_HHMMSS` (optional descriptive suffix). Default storage is **MCAP** (see [`nodes/record.yaml`](../nodes/record.yaml)).

This document covers **recording**, **playback in Foxglove**, **offline bag tools** in `scripts/`, and related analysis utilities.

## Recording on the boat

| Component | Role |
|-----------|------|
| [`nodes/record.py`](../nodes/record.py) | ROS 2 node: start/stop `ros2 bag record -a` |
| Services | `/argo/recording/start`, `/argo/recording/stop` (`std_srvs/Trigger`) |
| Topic | `/argo/recording/status` (`std_msgs/Bool`) |
| Power button | Double-tap toggles recording ([`power_control/argo_power_control.py`](../power_control/argo_power_control.py)) |
| CLI aliases | `ar` / `ac` start/stop recording (see [`dotfiles/.bash_aliases`](../dotfiles/.bash_aliases)) |

Each bag directory contains `metadata.yaml`, one or more `.mcap` (or `.db3`) files, and optionally sidecar files (audio, SRT, notes) you add after the sail.

## Playback and visualization

### Foxglove Studio (recommended)

1. Open the bag folder (or merged output folder) in Foxglove.
2. Load layout [`foxglove/argo_playback.json`](../foxglove/argo_playback.json):
   - 3D view, map, plots (battery, wind, GPS, …)
   - **Captain's log** → `/controller/captains_log.data`
   - **Narration** → `/narration.data` (after running the merge tool below)

Scrub the timeline; narration and controller logs use **bag log time** (epoch), so they align with `/fix` and markers when sync is correct.

### Live playback with ROS 2 nodes

[`launch/argo_bag_playback.py`](../launch/argo_bag_playback.py) plays a bag and can run visualization, transform, and sailing-area nodes (optional Foxglove bridge):

```bash
source /opt/ros/humble/setup.bash
ros2 launch launch/argo_bag_playback.py bag_file:=/path/to/bag
```

### `ros2 bag play` only

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /path/to/bag
ros2 topic echo /narration          # if merged
ros2 topic echo /controller/captains_log
ros2 bag info /path/to/bag
```

Use `--clock` when other nodes use `use_sim_time` (e.g. re-record pipeline).

## Offline bag tools (`scripts/`)

| Tool | Purpose |
|------|---------|
| [`argo_bag_merge_narration.py`](../scripts/argo_bag_merge_narration.py) | Inject synced voice notes as `/narration` |
| [`fix_rosbag_pose_legacy_compass.py`](../scripts/fix_rosbag_pose_legacy_compass.py) | Fix legacy `/pose.z` (compass → ENU math yaw) |
| [`argo_bag_rerecord.py`](../scripts/argo_bag_rerecord.py) | Re-record with live viz / TF / sailing area |
| [`argo_rerecord_bag.sh`](../scripts/argo_rerecord_bag.sh) | Interactive wrapper for re-record launch |
| [`argo_backup_latest_recording.sh`](../scripts/argo_backup_latest_recording.sh) | Tar + SCP latest bag to backup host |
| [`analyze_gps_noise.py`](../scripts/analyze_gps_noise.py) | GPS noise plots from a bag |

Brief summaries also appear in [`scripts/README.md`](../scripts/README.md).

---

### Merge narration (`argo_bag_merge_narration.py`)

Adds **`/narration`** (`std_msgs/msg/String`) to a **copy** of a bag: one message per SRT cue, plain text, timed to match the recording.

**Inputs**

- Rosbag2 directory (`--bag`; resolves nested `.../argo_YYYYMMDD_HHMMSS/argo_YYYYMMDD_HHMMSS/` automatically)
- SRT file (Whisper-style `Unknown speaker HH:MM:SS,mmm --> ...` or standard SRT)
- Audio file (`.m4a`, etc.) for sync metadata via **exiftool**

**Time sync**

- Each cue log time = **audio_start + cue.start_sec** (+ optional `--offset-sec`)
- **audio_start** (`--audio-anchor auto`, default):
  - **`filename`**: `_YYMMDD_HHMMSS` in the audio name (local time, `--timezone`, default `Europe/Zurich`) — e.g. `260602_152344` → 2026-06-02 15:23:44
  - **`exiftool`**: `CreateDate` / `MediaCreateDate` as **UTC** (typical Android 3GP; matches `File Modification` ≈ Create + Samsung UTC offset)
- **Bag span**: `metadata.yaml` `starting_time` + `duration`
- **Only cues inside the bag recording window** are written (pre/post narration is dropped)

**Dependencies**

```bash
source /opt/ros/humble/setup.bash
sudo apt install libimage-exiftool-perl   # if needed
```

**Example workflow**

```bash
cd ~/argo/bags/my_sail_folder   # contains argo_*, .srt, .m4a

python3 ~/argo/scripts/argo_bag_merge_narration.py \
  --bag argo_20260602_153051 \
  --audio "Argo irhchel thunderstorm forming 260602_152344.m4a" \
  --dry-run

python3 ~/argo/scripts/argo_bag_merge_narration.py \
  --bag argo_20260602_153051 \
  --audio "Argo irhchel thunderstorm forming 260602_152344.m4a" \
  --output merged --force
```

Use `--offset-sec 0.5` if lines are slightly early/late in Foxglove. Override anchor with `--audio-anchor exiftool` or `--audio-start "2026-06-02 15:23:44"`.

---

### Fix legacy `/pose` (`fix_rosbag_pose_legacy_compass.py`)

Older bags duplicated **compass** heading on `/pose.z`. Argo expects **`/pose.z` as ENU math yaw**. This script copies the bag and rewrites only `/pose` (`z_new = (450 - z_old) % 360`). It does **not** fix recorded `/tf` or baked markers.

```bash
python3 scripts/fix_rosbag_pose_legacy_compass.py INPUT_BAG_DIR OUTPUT_BAG_DIR --dry-run
```

See also [`docs/README-imu-bno085.md`](README-imu-bno085.md) (legacy bags section).

---

### Re-record with visualization (`argo_bag_rerecord.py`)

Plays an input bag with `/clock`, runs `argo_boat_visualization`, `argo_transform_publisher`, `sailing_area_publisher`, and records a **new** bag (original topics + fresh markers/TF). Fast playback (e.g. 100×) is supported; log times stay on sim time.

```bash
ros2 launch ./scripts/argo_bag_rerecord.py \
  input_bag:=bags/argo_20251105_141014 \
  output_bag:=argo_20251105_141014_with_viz
```

**Time trim:** `trim_start` + `trim_end` (ISO wall time, or seconds from bag start).  
**Markers:** default `exclude_recorded_markers:=true` avoids duplicate marker streams.

Wrapper: `./scripts/argo_rerecord_bag.sh`

---

### Backup latest bag (`argo_backup_latest_recording.sh`)

Finds the newest folder in `~/argo/bags`, creates `.tgz`, SCPs to a configured host (default under this repo’s `bags/`).

---

### GPS noise analysis (`analyze_gps_noise.py`)

Reads `/fix`, COG, SOG from a bag and generates noise / track plots (matplotlib).

```bash
python3 scripts/analyze_gps_noise.py /path/to/bag
```

---

## LoRa / bag analysis (`tests/lora/`)

Scripts that read rosbag2 for shore-side LoRa review (not in `scripts/`):

- `analyze_latest_lora.py`
- `check_signal_strength.py`
- `plot_gps_lora_simple.py`

See [`tests/lora/README.md`](../tests/lora/README.md).

## Typical post-sail workflow

1. Copy or sync the bag folder from the Pi (optional: `argo_backup_latest_recording.sh`).
2. Add **m4a** + **SRT** transcription in the same folder (or parent).
3. `argo_bag_merge_narration.py --dry-run` then write `merged/` (or `*_with_narration`).
4. If the bag predates the `/pose` fix: `fix_rosbag_pose_legacy_compass.py` (if needed).
5. Open in **Foxglove** with `argo_playback.json`, or re-record with viz for a self-contained review bag.
6. Optional: `analyze_gps_noise.py` or LoRa analysis scripts.

## Related docs

- [`docs/README-imu-bno085.md`](README-imu-bno085.md) — `/pose` vs `/compass`, legacy bags
- [`docs/README-3d-visualization.md`](README-3d-visualization.md) — visualization nodes
- [`docs/VISUALIZATION_MARKER_PERSISTENCE.md`](VISUALIZATION_MARKER_PERSISTENCE.md) — why re-record exists
