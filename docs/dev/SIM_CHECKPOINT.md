# Simulation Checkpoint — 2026-05-07

Practical bring-up + state guide for the Gazebo simulation testbed.
**These values are sim-only** — real hardware will need its own calibration.

---

## What this checkpoint covers

End-to-end **action mode** and **servo mode** typing pipelines run in Gazebo:
fake vision → coordinator → (`ExecuteKey` action *or* `/goal` servo loop) → arm_node IK → arm_bridge → JointTrajectoryController → arm motion.

For both modes, the arm visits each key of "hola" in the correct (y, z) on the vertical panel.

**Servo state machine** progresses correctly through every phase
(`IDLE → ALIGNING → ALIGN_HOLD → ALIGNED_READY_PRESS → PRESSING → RETRACTING → RETURNING_BASE → COMPLETE → WAIT_NEXT_KEY`)
when given a manual contact pulse.

---

## What we changed in code

### 1. `src/arm_ik/src/main.cpp` — IK joint mapping
Removed an unjustified `-90°` offset on q4. Final mapping:
```cpp
q1d = radToDeg(q1);
q2d = radToDeg(q2);
q3d = radToDeg(q3);
q4d = radToDeg(q4);
q5d = radToDeg(q5);
```

### 2. `src/zed_aruco/zed_aruco/typing_coordinator.py` — vertical keyboard mapping
- Removed `scale_x_per_px`, added `scale_z_per_px`.
- `pixel_to_arm_goal(px, py)` returns `(x, y, z)`:
  - `x = base_x` (fixed; panel face distance)
  - `y = base_y + dx_px * scale_y_per_px`
  - `z = target_z + dy_px * scale_z_per_px`

### 3. `src/zed_aruco/zed_aruco/typing_coordinator.py` — servo state machine re-axised
- `compute_xy_servo_delta` → `compute_yz_servo_delta`. Returns `(delta_y, delta_z)`.
  - Horizontal pixel error → arm-y (gain `servo_y_gain_m_per_px`).
  - Vertical pixel error → arm-z (gain `servo_z_gain_m_per_px`).
  - Sign convention now matches `pixel_to_arm_goal` (image-up = +z, image-left = +y).
- Press steps along **arm-x** (was arm-z): `next_x = servo_cmd_x + sign * servo_press_step_m`.
- Retract returns along arm-x to `servo_hover_x` (snapshot taken at press start).
- Press travel limit and workspace check now monitor `next_x`.
- `servo_press_direction_sign` default flipped `-1.0` → `+1.0` (press into panel = +arm-x).
- State `PRESSING_Z` → `PRESSING` (axis-agnostic).

---

## Build

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select typing_interfaces arm_ik zed_aruco
source install/setup.bash
```

(If `typing_interfaces` cache is stale: `rm -rf build/typing_interfaces` first.)

---

## Bring-up — 6 terminals

Source in each new terminal:
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

### Terminal 1 — Gazebo + rover + keyboard
```bash
ros2 launch mi_rover_description gazebo.launch.py
```

### Terminal 2 — arm_node (real publish)
```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

### Terminal 3 — arm_bridge (degrees → radians → JointTrajectoryController)
```bash
python3 src/mi_rover_description/src/arm_bridge.py
```

### Terminal 4 — fake vision publisher
```bash
ros2 run zed_aruco fake_vision_publisher --ros-args \
  -p text:=hola -p loop_text:=true
```

### Terminal 5 — typing_coordinator (action mode, default)
```bash
ros2 run zed_aruco typing_coordinator --ros-args \
  -p servo_mode_enabled:=false \
  -p motion_enabled:=true \
  -p use_tf_targeting:=false \
  -p require_transform_valid:=false \
  -p min_confidence:=0.2 \
  -p base_x:=0.7 \
  -p base_y:=0.0 \
  -p target_z:=0.35 \
  -p target_pitch:=0.0 \
  -p workspace_x_max:=1.0 \
  -p image_center_x:=520.0 \
  -p image_center_y:=180.0 \
  -p scale_y_per_px:=0.000288 \
  -p scale_z_per_px:=0.0004
```

### Terminal 5 (alt) — typing_coordinator (servo mode, sim-validated)
```bash
ros2 run zed_aruco typing_coordinator --ros-args \
  -p servo_mode_enabled:=true \
  -p motion_enabled:=true \
  -p use_tf_targeting:=false \
  -p require_transform_valid:=false \
  -p min_confidence:=0.2 \
  -p base_x:=0.7 \
  -p base_y:=0.0 \
  -p target_z:=0.35 \
  -p target_pitch:=0.0 \
  -p workspace_x_max:=1.0 \
  -p image_center_x:=520.0 \
  -p image_center_y:=180.0 \
  -p scale_y_per_px:=0.000288 \
  -p scale_z_per_px:=0.0004 \
  -p servo_align_enter_thresh_px:=500.0 \
  -p servo_align_exit_thresh_px:=600.0 \
  -p return_to_base_command:=HOME \
  -p servo_press_step_m:=0.0005 \
  -p servo_press_max_travel_m:=0.05 \
  -p servo_press_timeout_sec:=15.0 \
  -p servo_press_xy_scale:=0.0 \
```

Why these servo overrides are sim-only (do **not** carry to hardware):
- `servo_align_enter_thresh_px:=500.0` — fake vision emits a fixed pixel per key; without a real wrist camera the alignment loop has no feedback, so we force-pass the alignment threshold to exercise the rest of the state machine.
- `servo_press_xy_scale:=0.0` — disables YZ correction during press. With fake vision, the constant pixel error would otherwise drive arm-z monotonically out of the workspace mid-press.
- `return_to_base_command:=HOME` — uses the unconditional URDF `HOME` predefined pose so we don't have to call `SET_KEYBOARD_HOME` first.
- Slow press tuning (`servo_press_step_m:=0.0005`, `_max_travel:=0.05`, `_timeout:=15.0`) — extends the `PRESSING` window to ~8 s so the manual contact pulse can land on time.

Note: `goal_cooldown_sec`, `servo_align_stable_cycles`, and `return_to_base_wait_sec` are now
constants in typing_coordinator and are no longer ROS parameters. Edit
`zed_aruco/typing_coordinator.py` if these values need to change.

### Terminal 6 — debug watcher
```bash
ros2 topic echo /keyboard/coordinator_debug
```

### Terminal 7 — manual contact pulse (servo mode only)
While `servo_phase` is `PRESSING`:
```bash
ros2 topic pub --once /keyboard/contact_pressed std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /keyboard/contact_pressed std_msgs/msg/Bool '{data: false}'
```

---

## Calibration values (sim-only)

| Param | Value | Meaning |
|---|---|---|
| `base_x` | `0.7` | Arm-x at panel face (fixed) |
| `base_y` | `0.0` | Arm-y at keyboard horizontal center |
| `target_z` | `0.35` | Arm-z at the keyboard top reference pixel |
| `target_pitch` | `0.0` | Horizontal gripper (camera mode) |
| `image_center_x` | `520.0` | Pixel x of keyboard horizontal center (col 6.5 of fake-vision layout) |
| `image_center_y` | `180.0` | Pixel y of keyboard top edge in fake-vision layout |
| `scale_y_per_px` | `0.000288` | Empirically calibrated against observed wrist-reach (0.19 m / 660 px) |
| `scale_z_per_px` | `0.0004` | 0.12 m / 300 px (panel height / row pixel span) |
| `workspace_x_max` | `1.0` | Raised from 0.55 default to allow `base_x = 0.7` |

**Why scale_y_per_px is 0.19/660 instead of 0.30/720:** the panel is 30 cm wide (per SDF), but the gripper has visual width ~6 cm on each side, so the **wrist's** reachable y range across the panel is ~19 cm. We calibrated against where the wrist actually goes, not the panel dimensions.

---

## Quick smoke test

After bringing up Terminals 1–5, watch Gazebo and the debug topic.

**Action mode** — the arm should:
1. Visit each letter of "hola" in turn at `arm_x = 0.7`.
2. Have `arm_y` vary from ~+0.09 (for "a") to ~−0.04 (for "o", "l").
3. Have `arm_z` vary from ~0.31 (row 1) to ~0.29 (row 2).

`coordinator_debug`: `mode:action`, `goal_active:true` cycling per key, `current_key` advancing through h → o → l → a.

**Servo mode** — start with the alt Terminal 5 and Terminal 7 ready. Per key:
1. State cycles `WAIT_TARGET → ALIGNING → ALIGN_HOLD → ALIGNED_READY_PRESS → PRESSING`.
2. Arm steps **forward in +x** (toward the panel) during `PRESSING`.
3. Fire contact pulse on Terminal 7. Phase moves to `RETRACTING`, arm retreats in −x to `servo_hover_x`.
4. Phase moves to `RETURNING_BASE`, arm goes to URDF home.
5. `current_key` advances. Loop continues for `o → l → a`.

If contact pulse is missed, press hits max-travel and retract still runs — `last_goal_result` reflects a non-success retract and the same key retries.

---

## Sim validation results (2026-05-07)

What sim **proved**:
- State machine flow is correct in both modes (action and servo).
- Axis assignment for vertical panel: alignment uses Y/Z, press uses +X, retract restores hover X.
- Initial pose mapping (`pixel_to_arm_goal`) places the arm at the right per-key coordinates.
- Contact pulse → completion → next-key advancement works (`h → o → l → a`).
- IK + arm bridge end-to-end: every commanded pose is physically reachable in Gazebo.

What sim **did not** prove (these are real-hardware unknowns):
- Sign of the YZ pixel-error feedback in `compute_yz_servo_delta`. Fake vision emits a fixed pixel per key, so the feedback loop never closes — the broken-in-sim behavior was the reason for `servo_press_xy_scale:=0.0`.
- `servo_press_direction_sign`. Default `+1.0` assumes `+arm-x` is into the panel; depends on the rover arm-base frame orientation.
- All spatial calibration (`base_x/y`, `target_z`, `scale_y/z_per_px`, `image_center_x/y`).
- `/keyboard/contact_pressed` source. We pulsed it manually; no real publisher exists yet.
- ArUco/vision robustness (dropouts, lighting, occlusions).

---

## Hardware bring-up plan (staged)

Don't enable full servo on first hardware run. Catch the YZ sign-convention bug early by staging:

1. **Action mode + real vision, calibrate spatial constants.**
   Bring up real ZED + ArUco + arm_node + typing_coordinator with `servo_mode_enabled:=false`. Drive the arm to a known key with `motion_enabled:=true`. Read `/arm_ik/debug_status` and tune `base_x`, `base_y`, `target_z`, `scale_y/z_per_px`, `image_center_x/y` until the per-key arm-y/z match the physical key positions. (This is RUNTIME_COMMANDS.md section 8.)

2. **Servo mode with YZ feedback disabled.**
   Set `servo_mode_enabled:=true` and `servo_press_xy_scale:=0.0`. Use real (tight) `servo_align_enter_thresh_px:=8.0`. This validates state-machine flow and press direction in real conditions without trusting the YZ feedback loop. If the arm presses **away** from the panel, set `servo_press_direction_sign:=-1.0`.

3. **Servo mode with low YZ feedback gain.**
   Set `servo_y_gain_m_per_px:=0.0001` and `servo_z_gain_m_per_px:=0.0001`, leave `servo_press_xy_scale:=0.0` so press is still pure-X. Watch a single alignment cycle in `/keyboard/coordinator_debug`: if `target_px` moves **toward** `image_center_x/y` between cycles, the sign is right. If it moves **away**, flip the sign of the appropriate gain (or invert `dx_px`/`dy_px` in `compute_yz_servo_delta`). **This is the moment of truth.**

4. **Raise gains to nominal, enable press-time YZ correction.**
   Once signs are confirmed, raise gains to ~`0.00035` and set `servo_press_xy_scale:=0.6` so the wrist fine-tunes YZ during press as the camera image updates.

5. **Wire up a real `/keyboard/contact_pressed` source.**
   Pick one: physical contact sensor on the gripper, joint-effort spike detection in `arm_node`, vision-based key-depressed detection, or "press-on-max-travel = success" as a fallback.

---

## Open hardware questions

- **Contact detection mechanism** — which of the four options above is feasible for the URC build?
- **TF mode vs. heuristic mode** — `use_tf_targeting:=true` requires a valid TF tree from the ZED wrapper to `arm_base`. Worth choosing before competition because TF mode generalizes across rover poses, while heuristic mode requires recalibration if the rover or camera shifts.
- **Real `image_center_x/y`** — currently a static parameter. Should the vision node publish a dynamic reference (e.g., the keyboard centroid) so the servo loop tracks small camera/rover drift without recalibration?
- **`SET_KEYBOARD_HOME` capture procedure** — for the real run we'll want `KEYBOARD_HOME` (not URDF `HOME`) as the return-to-base pose. Need a documented "drive arm to safe hover, send `SET_KEYBOARD_HOME`" step in the bring-up sequence.

---

## Files modified across sessions

| File | Change |
|---|---|
| `src/arm_ik/src/main.cpp` | Removed −90° on q4d (line 102); updated comment |
| `src/zed_aruco/zed_aruco/typing_coordinator.py` | Vertical-keyboard mapping; `scale_x_per_px` → `scale_z_per_px`; servo state machine re-axised (YZ alignment, +x press, `PRESSING_Z` → `PRESSING`) |
| `CLAUDE.md`, `README.md`, `RUNTIME_COMMANDS.md` | Param-name and state-name renames synced with code |
| `CONTEXT_FOR_CLAUDE.md` | Resolution section appended |
| `SIM_CHECKPOINT.md` | This file |
