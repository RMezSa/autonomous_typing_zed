# Simulation Checkpoint — 2026-05-06

Practical bring-up + state guide for the Gazebo simulation testbed.
**These values are sim-only** — real hardware will need its own calibration.

---

## What this checkpoint covers

End-to-end **action-mode** typing pipeline runs in Gazebo:
fake vision → coordinator → ExecuteKey action → arm_node IK → arm_bridge → JointTrajectoryController → arm motion.

The arm visits each key of "hola" in the correct (y, z) on the vertical panel.

**Servo mode (state machine) is NOT yet adapted for vertical keyboards** — that's the next step.

---

## What we changed in code

### 1. `src/arm_ik/src/main.cpp` — IK joint mapping
Removed an unjustified `-90°` offset on q4. Final mapping:
```cpp
q1d = -radToDeg(q1);
q2d = -radToDeg(q2) + 90.0;   // q2 absorbs the URDF rest-pose offset (+Z vs +X)
q3d = -radToDeg(q3);
q4d = -radToDeg(q4);          // no offset — q4 measures relative to forearm
q5d = radToDeg(q5);
```

### 2. `src/zed_aruco/zed_aruco/typing_coordinator.py` — vertical keyboard mapping
- Removed `scale_x_per_px`, added `scale_z_per_px`.
- `pixel_to_arm_goal(px, py)` now returns `(x, y, z)`:
  - `x = base_x` (fixed; panel face distance)
  - `y = base_y + dx_px * scale_y_per_px`
  - `z = target_z + dy_px * scale_z_per_px`
- Both call sites updated to receive the third value.

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

### Terminal 5 — typing_coordinator (calibrated for sim)
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
  -p goal_cooldown_sec:=2.0 \
  -p workspace_x_max:=1.0 \
  -p image_center_x:=520.0 \
  -p image_center_y:=180.0 \
  -p scale_y_per_px:=0.000288 \
  -p scale_z_per_px:=0.0004
```

### Terminal 6 — debug watcher (optional)
```bash
ros2 topic echo /keyboard/coordinator_debug
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

After bringing up Terminals 1–5, watch Gazebo and the debug topic. The arm should:
1. Visit each letter of "hola" in turn at `arm_x = 0.7`.
2. Have `arm_y` vary from ~+0.09 (for "a") to ~−0.04 (for "o", "l").
3. Have `arm_z` vary from ~0.31 (row 1) to ~0.29 (row 2).

In `coordinator_debug` you should see `mode:action`, `goal_active:true` cycling per key, `current_key` advancing through h → o → l → a.

---

## What's broken / what's next

### Servo mode (not yet adapted for vertical keyboards)
`typing_coordinator.py` servo state machine still assumes a horizontal keyboard:
- **XY correction** uses `servo_xy_gain_x` (vertical pixel → arm-x) and `servo_xy_gain_y` (horizontal pixel → arm-y). For vertical panels this should be Y/Z (horizontal pixel → arm-y, vertical pixel → arm-z).
- **Press direction** is z-axis (`servo_press_direction_sign × servo_press_step_m` applied to `servo_cmd_z`). For vertical panels the press should push along **arm-x** (into the panel).
- Same for retract path and the workspace check inside the servo loop.

Lines to touch (per earlier audit):
- 50–51, 109–110: rename/reuse `servo_xy_gain_x` → `servo_z_gain`
- 403, 408: swap which axis the gain corrects
- 459, 461, 488: change `next_z` to `next_x` for press/retract

Until that's done, only run `servo_mode_enabled:=false` in sim.

### Real-hardware calibration (separate workflow)
None of the values above carry over to real hardware. Real bring-up will need:
- Empirical `base_x/y` and `target_z` from physical setup.
- Empirical `scale_y/z_per_px` from real ZED FOV + keyboard distance.
- `image_center_x/y` from where the actual keyboard top-center lands in the ZED frame (or, better, the vision node should publish a dynamic reference).

---

## Files modified this session

| File | Change |
|---|---|
| `src/arm_ik/src/main.cpp` | Removed −90° on q4d (line 102); updated comment |
| `src/zed_aruco/zed_aruco/typing_coordinator.py` | Vertical-keyboard mapping; `scale_x_per_px` → `scale_z_per_px`; call sites |
| `CONTEXT_FOR_CLAUDE.md` | Resolution section appended |
| `SIM_CHECKPOINT.md` | This file |
