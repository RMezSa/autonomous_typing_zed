# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

All ROS 2 commands run inside the Docker container (`ros2docker`) or with the workspace sourced. The ROS 2 workspace is at `~/ros2_ws`; this repo's `src/` packages live under `~/ros2_ws/src/autonomous_typing_zed/src/` (or symlinked there).

Source environment in every terminal before running any ROS command:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Build

Build the three main packages (always build `typing_interfaces` first or together, since `arm_ik` and `zed_aruco` depend on it):
```bash
colcon build --symlink-install --packages-select typing_interfaces arm_ik zed_aruco
source install/setup.bash
```

Fast iteration on vision/coordinator only:
```bash
colcon build --symlink-install --packages-select zed_aruco
source install/setup.bash
```

If build fails with `existing path cannot be removed: Is a directory`, clear the stale cache first:
```bash
rm -rf build/typing_interfaces
```

## Linting

Python packages use flake8 and pep257. Run per-package tests with:
```bash
colcon test --packages-select zed_aruco
colcon test-result --verbose
```

## Launch

**No-hardware simulation** (for development without a ZED or physical arm):
```bash
ros2 launch zed_aruco no_hardware_integration.launch.py \
  servo_mode_enabled:=false motion_enabled:=true use_tf_targeting:=false text:=hola
```

**Real hardware (competition mode — no TF required)**:
```bash
# Terminal A
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true

# Terminal B
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=false use_tf_targeting:=false require_transform_valid:=false
```

Then enable motion after the arm is positioned and calibrated:
```bash
ros2 param set /typing_coordinator motion_enabled true
```

See `RUNTIME_COMMANDS.md` for the full URC bring-up sequence with calibration steps.

## Architecture

### Packages and their roles

- **`typing_interfaces`** — defines the `ExecuteKey.action` message used across the system (`string key_label`, `float64 x/y/z/roll/pitch` → `bool success`, `string message`).
- **`arm_ik`** (C++) — `arm_node`: IK solver for a 4-link arm (l1=0.1m, l2=0.43m, l3=0.43m, l4=0.213m). Accepts goals via three interfaces: `/arm_ik/execute_key` action, `/goal` Float64MultiArray topic `[x,y,z,roll_deg,pitch_deg]`, and `/predefined` String topic (HOME, KEYBOARD_HOME, INTERMEDIATE, PREFLOOR, FLOOR, STORAGE, SET_KEYBOARD_HOME). Joint 5 output is remapped to servo microseconds `[88, 268]`. `publish_on_action:=false` puts the node in dry-run mode (IK solved, no joint publish).
- **`zed_aruco`** (Python) — contains three runnable nodes:
  - `zed_aruco_node`: ZED image subscriber → ArUco detection → keyboard homography → key localization → Kalman-filtered target tracking → integration topic publisher.
  - `typing_coordinator`: consumes vision topics, applies safety gates, runs the servo or action-based arm control state machine.
  - `calibration_probe`: optional TF-based calibration helper (projects pixel targets through TF into arm frame for error measurement).
  - `fake_vision_publisher` / `fake_execute_key_server`: no-hardware simulation replacements.
- **`aruco_py`** — standalone ArUco detector for non-ZED image sources (separate from `zed_aruco`).

### Vision pipeline (`zed_aruco_node`)

The node runs a detection fallback chain on each image frame:

1. **TRACKING** — optical-flow point tracking (Lucas-Kanade) of the last known key position. Fastest path; exits when flow fails.
2. **ARUCO** — four-corner ArUco markers detected; homography computed from inside-facing corners (TL→BR, TR→BL, BR→TL, BL→TR). Updates the `last_good_M` warp matrix.
3. **FLOW** — full homography updated via Lucas-Kanade on interior feature points when markers are partially occluded.
4. **HOLD** — last computed homography used for up to `homography_hold_seconds` (default 1.0s).
5. **SEARCHING** — no valid homography; publishes `target_valid=false`.

The `keyboard/state` topic reflects which mode is active. `typing_coordinator` requires `TRACKING` state by default (`required_state` param).

Target confidence is derived from state: TRACKING=1.0, ARUCO=0.9, FLOW=0.7, HOLD=0.5, else 0.0.

Keyboard geometry: Spanish layout, 18 total units wide × 6 rows. The node opens three OpenCV windows at runtime: "ArUco Detection", "Warped ROI", "Keyboard Layout (Canonical)".

**Runtime keyboard input** (click the OpenCV window): type `>holamundo` + Enter to queue autonomous typing; type a single key label + Enter to target one key; `d` to manually mark the current key done.

### Servo state machine (`typing_coordinator`)

When `servo_mode_enabled=true`, the coordinator drives the arm through a state machine (not via the action):

```
IDLE → WAIT_TARGET / WAIT_CONFIDENCE / WAIT_STATE → WAIT_SERVO_INIT
     → ALIGNING (XY correction via /goal) → ALIGN_HOLD → ALIGNED_READY_PRESS
     → PRESSING_Z (incremental Z steps via /goal, monitors contact)
     → RETRACTING → RETURNING_BASE → COMPLETE → WAIT_NEXT_KEY
```

Emergency stop (`keyboard/emergency_stop=true`) immediately enters `EMERGENCY_HOLD` and cancels active action goals; releasing it re-arms to IDLE.

When `servo_mode_enabled=false`, the coordinator uses the `ExecuteKey` action directly (one goal per key, blocks until result).

### Pixel-to-arm coordinate mapping

Two modes controlled by `use_tf_targeting`:

- **`false` (competition mode)**: heuristic linear mapping. `arm_x = base_x + (dy_px * scale_x_per_px)`, `arm_y = base_y + (dx_px * scale_y_per_px)`. Calibrate `base_x/base_y` by driving the arm over the image-center key and reading `/arm_ik/debug_status`.
- **`true` (TF mode)**: projects pixel into camera frame using intrinsics at `keyboard_plane_z_m` depth, then transforms via TF from camera frame → `arm_base`. Requires a valid `static_transform_publisher` or real TF from ZED wrapper.

### Key safety defaults

| Parameter | Default | Effect |
|---|---|---|
| `motion_enabled` | `false` | Master gate; arm will not move until set to `true` |
| `publish_on_action` | `false` | `arm_node` dry-run; no joint commands published |
| `require_transform_valid` | `true` (launch: `false`) | Blocks motion if TF lookup fails |
| `servo_press_direction_sign` | `-1.0` | Flip to `1.0` if press moves away from keyboard |

### Debug topics

| Topic | Content |
|---|---|
| `/keyboard/coordinator_debug` | JSON: mode, phase, gates, current key, servo xyz, last result |
| `/arm_ik/debug_status` | JSON: last target xyz/rp, command source, IK result, keyboard_home state |
| `/keyboard/servo_state` | Current servo phase string |
| `/keyboard/transform_valid` | Bool: TF lookup status |
