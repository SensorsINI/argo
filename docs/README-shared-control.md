# Human–robot shared control

Argo separates **who drives the servos** (human vs robot) from **which autonomous algorithm is loaded** (proportional, crosser, return-to-home, etc.). Radio activity and timeouts are arbitrated in [`nodes/rudder_sail_radio.py`](../nodes/rudder_sail_radio.py); planning and `/rudder_sail_cmd` come from [`nodes/controller.py`](../nodes/controller.py).

For the two-node architecture, control flow, and `argo.yaml` parameters (`human_override_timeout`, `deadband_threshold`), see [`nodes/SHARED_CONTROL.md`](../nodes/SHARED_CONTROL.md).

## Two questions (do not conflate them)

| Question | Source of truth | Not the same as |
|----------|-----------------|-----------------|
| **Who drives the boat right now?** | `rudder_sail_radio.py` → `/human_controlled` | Which controller is selected in `argo.yaml` |
| **Which controller algorithm is active?** | `controller.py` → `/controller_state` | Whether the human is on the sticks |

Examples:

- **Crosser selected, human on radio:** `/controller_state` may still be `CrosserController` while `/human_controlled` is `true`. The controller stops publishing autonomous commands (except when `HumanController` is active; see below).
- **Crosser selected, GPS/IMU unhealthy:** `controller.py` may switch to `HumanController` and publish `/release_servos` true even though `controller_type` in YAML is still `crosser`.
- **Power-button single tap:** temporarily sets `controller_type` to `human` (`HumanController`) for servo/radio setup; second tap restores the previous type via `/tmp/argo_power_controller_resume_type`.

## Nodes

| Node | Role |
|------|------|
| [`rudder_sail_radio.py`](../nodes/rudder_sail_radio.py) | Hardware I/O, human priority, timeout handover, final `/rudder_sail_servo` |
| [`controller.py`](../nodes/controller.py) | Sensor fusion into `BoatState`, active controller, `/rudder_sail_cmd` |
| Controllers under [`nodes/controllers/`](../nodes/controllers/) | Algorithms (`crosser`, `proportional`, `wind_aware`, `return_to_home`, `human`) |

Control arbitration (human wins on recent stick activity; robot after `human_override_timeout` with no human activity) is implemented only in `rudder_sail_radio.py`. `controller.py` subscribes to `/human_controlled` and adjusts behavior (target heading, data collection, whether to publish commands).

## Topics for authority (human vs robot)

Published by **`rudder_sail_radio.py`** — use these to see who has control **right now**.

| Topic | Type | Meaning |
|-------|------|---------|
| `/human_controlled` | `std_msgs/Bool` | **`true`** = human has authority; **`false`** = robot applies `/rudder_sail_cmd` |
| `/control_authority` | `geometry_msgs/Vector3` | **`x`**: 1 = human, 0 = robot; **`y`**: seconds since last human activity; **`z`**: seconds since last autonomous command |
| `/rudder_sail_radio` | `Vector3` | Normalized radio input (reference; also used by controller) |
| `/rudder_sail_servo` | `Vector3` | Commands actually sent to hardware |

`controller.py` subscribes to `/human_controlled` and `/control_authority` but does not republish them.

### HumanController and `/release_servos`

When `controller_type` is `human` ([`nodes/controllers/human.py`](../nodes/controllers/human.py)), the controller publishes:

| Topic | Type | Meaning |
|-------|------|---------|
| `/release_servos` | `std_msgs/Bool` | **`true`** = servos in high-impedance mode (radio direct); **`false`** = normal PWM |

This is **not** the same as `/human_controlled`: release mode is for setup/testing via power-button tap; authority on the water is still defined by radio activity and `/human_controlled`.

## Topics for active controller (algorithm)

Published by **`controller.py`** (~5 Hz control loop):

| Topic | Type | Meaning |
|-------|------|---------|
| `/controller_state` | `std_msgs/String` | Running controller **class name** (e.g. `CrosserController`, `HumanController`, `ReturnToHomeController`) |
| `/rudder_sail_cmd` | `Vector3` | Autonomous rudder/sail commands (−1…+1); meaningful when robot has authority |

Published via **`BaseController`** ([`nodes/controllers/base.py`](../nodes/controllers/base.py)) when the active controller calls `publish_state()` / `log_entry()`:

| Topic | Type | Meaning |
|-------|------|---------|
| `/controller/state` | `std_msgs/String` | Behavioral state for visualization (controller-specific) |
| `/controller/captains_log` | `std_msgs/String` | Log lines `[LEVEL] message` for analysis and Foxglove |

The YAML parameter `controller_type` (`proportional`, `crosser`, `human`, …) is **not** a live topic; watch `/controller_state` during playback or on the boat.

### Crosser-specific topics

[`nodes/controllers/crosser.py`](../nodes/controllers/crosser.py) does not define its own publishers; it uses the parent node and base helpers. When crosser is running with a loaded geofence, you also get:

| Topic | Type | Meaning |
|-------|------|---------|
| `/geofence/distance_to_boundary` | `std_msgs/Float64` | Signed distance to fence (see geofence manager) |
| `/geofence/violation` | `std_msgs/Bool` | Outside sailing area |
| `/grounding/detected` | `std_msgs/Bool` | Grounding heuristic |

Crosser depends on healthy GPS and IMU. `controller.py` subscribes to `/gps_node_health` and `/imu_health`; if either is false while crosser is selected and the human is not in control, it switches to `HumanController` until both recover (see crosser docstring in `crosser.py`).

### Crosser `/controller/state` values

Crosser publishes both **goal states** and **maneuvers** (captain’s log often shows both, e.g. `Toward Middle (TACKING)`):

| State (examples) | When |
|------------------|------|
| `launching` | Following `approach_1`, `approach_2`, … waypoints |
| `toward_middle` | Navigating to pond middle / center |
| `crossing_through` | Steady heading after passing middle |
| `turning_around` | Boundary turn (generic) |
| `tacking` / `jibing` / `tacking_upwind` | Wind-related maneuvers |

State transitions and detail are also written to `/controller/captains_log` (`STATE TRANSITION: …`).

## Foxglove visualization

Layouts: [`foxglove/argo_playback.json`](../foxglove/argo_playback.json) (bag playback, includes Captain’s log), [`foxglove/argo_3d_enhanced.json`](../foxglove/argo_3d_enhanced.json) (live-style plots including `/human_controlled` with rudder/sail). See also [`docs/README-bagfiles.md`](README-bagfiles.md) and [`foxglove/custom-argo-panel/README.md`](../foxglove/custom-argo-panel/README.md).

### Minimal dashboard (authority + algorithm)

| Panel | Topic / path | Answers |
|-------|----------------|---------|
| Plot or State Transitions | `/human_controlled.data` | Human vs robot **authority** |
| Raw Messages or plot | `/controller_state.data` | Which **controller class** is running |
| Raw Messages | `/controller/state.data` | Crosser (or other) **behavioral** state |

### Human vs robot (handoff debugging)

| Panel | Topic / path | Use |
|-------|----------------|-----|
| Plot | `/human_controlled.data` | Primary on/off for robot control |
| Plot | `/control_authority.x`, `.y`, `.z` | Authority flag, time since human, time since auto cmd |
| Plot overlay | `/rudder_sail_radio.*`, `/rudder_sail_cmd.*`, `/rudder_sail_servo.*` | Under human authority, servo tracks radio; under robot, servo tracks cmd |
| Plot | `/release_servos.data` | High-impedance / setup mode (`HumanController`) |

### Active controller and crosser detail

| Panel | Topic / path | Use |
|-------|----------------|-----|
| Raw Messages | `/controller_state.data` | `CrosserController` vs `ReturnToHomeController`, etc. |
| Raw Messages | `/controller/state.data` | `toward_middle`, `tacking_upwind`, combined mode strings |
| Raw Messages | `/controller/captains_log.data` | Transitions, periodic status (playback layout: “Captain's log”) |
| Plot | `/geofence/distance_to_boundary`, `/geofence/violation`, `/grounding/detected` | Boundary / tack debugging |

### Quick interpretation

| Observation | Likely meaning |
|-------------|----------------|
| `/human_controlled` = true | Human has authority (recent radio activity within timeout) |
| `/human_controlled` = false | Robot path; servos follow `/rudder_sail_cmd` when controller publishes |
| `/controller_state` = `CrosserController` | Crosser algorithm loaded and running control loop |
| `/controller_state` = `HumanController` | Setup mode or sensor fallback (often with `/release_servos` true) |
| `/controller/state` = `tacking_upwind` | Crosser tactical state (only when crosser is driving logic) |
| `/release_servos` = true | Servos released for direct radio (`HumanController`) |

## CLI checks

```bash
source /opt/ros/humble/setup.bash

# Authority
ros2 topic echo /human_controlled --once
ros2 topic echo /control_authority --once

# Active controller
ros2 topic echo /controller_state --once
ros2 topic echo /controller/state --once

# Commands vs hardware
ros2 topic echo /rudder_sail_cmd --once
ros2 topic echo /rudder_sail_servo --once
```

Parameter `human_override_timeout` for `rudder_sail_radio_node` lives in [`nodes/argo.yaml`](../nodes/argo.yaml) (default in repo is often 8 s; see `SHARED_CONTROL.md` for tuning examples).

## Related docs

- [`nodes/SHARED_CONTROL.md`](../nodes/SHARED_CONTROL.md) — architecture, control flow, configuration
- [`nodes/controller.py`](../nodes/controller.py) — controller list, captain’s log, parameters
- [`.cursor/rules/argo-radio-servo-control.mdc`](../.cursor/rules/argo-radio-servo-control.mdc) — arbitration patterns for developers
