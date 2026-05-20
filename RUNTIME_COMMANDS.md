# URC Bring-up Manual

Competition-mode bring-up for the autonomous keyboard-typing system. Follow the
sections in order. The first half (sections 1-7) is the actual run-day procedure;
the second half (sections 8+) is tuning, troubleshooting, and reference.

---

## 0. First-time setup (Jetson / new machine)

Clone the repo into the ROS 2 workspace:

```bash
git clone -b feature/strict-hardware-mode https://github.com/RMezSa/autonomous_typing_zed.git ~/ros2_ws/src/autonomous_typing_zed
```

Then proceed to section 2 (Build) before running anything.

---

## 1. Environment (every new terminal)

```bash
export ROS_LOCALHOST_ONLY=1   # prevents phantom-node name collisions over LAN
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. Build

```bash
colcon build --symlink-install --packages-select typing_interfaces arm_ik zed_aruco
source install/setup.bash
```

If build fails with `existing path cannot be removed: Is a directory`:

```bash
rm -rf build/typing_interfaces
```

then re-run the colcon command.

---

## 3. Pre-flight checks (run BEFORE motion)

These confirm wiring before the arm has permission to move. Do them every session.

### 3.1 — Park rover at ~20 cm from keyboard so the ZED sees the full panel.

### 3.2 — Start the arm node and the vision stack in separate terminals.

**Terminal A (arm node, real publish):**

```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

Wait for: `arm_node ready. publish_on_action=true max_step=1.50deg/tick @ 50.0Hz`

**Terminal B (vision + coordinator):**

```bash
ros2 launch zed_aruco zed_typing_integration.launch.py motion_enabled:=true
```

The launch defaults are already URC-correct:
- `servo_mode_enabled=true` (closed-loop)
- `use_tf_targeting=false` (no TF math needed)
- `require_transform_valid=false`
- `motion_enabled=true` (passed explicitly — bypasses the phantom-node `param set` race)

### 3.3 — Confirm topics are alive.

In a third terminal:

```bash
# ZED is publishing the image the vision node expects?
ros2 topic list | grep -E "zed2i.*(image|camera_info|depth)"

# Vision node detected the keyboard?
ros2 topic echo /keyboard/state --once
# Expect: "TRACKING" or "ARUCO". If "SEARCHING", check keyboard is in view.

# Joint feedback is reaching arm_node? The bridge node converts the firmware's
# /arm_feedback/jointN_deg (Float64 each, deg) into /joint_states (rad).
ros2 topic echo /joint_states --once
# Expect: name: [joint1, joint2, joint3, joint4] and position in radians.
# joint5 is intentionally omitted (no feedback on the gripper servo).
#
# If empty: check the raw firmware topics directly —
ros2 topic echo /arm_feedback/joint2_deg --once
# If those are silent, the firmware isn't publishing. arm_node will then ramp
# from start_pose_deg=[0,190,-140,-50,0] (the rover's documented rest pose).

# Coordinator picked up the camera intrinsics?
ros2 topic echo /keyboard/coordinator_debug --once | grep -o '"camera_intrinsics":{[^}]*}'
# Should show non-700 values for fx/cx once CameraInfo has arrived.
```
    
If any of these fail, stop and fix wiring before continuing.

---

## 4. Calibration (every new physical setup)

The coordinator can't know where the keyboard is in arm-frame coordinates. You teach
it by driving the arm to the keyboard once and recording two anchor poses.

### 4.1 — Drive arm to touch the center key.

Pick the key closest to the image center (visible in the OpenCV "Keyboard Layout"
window — usually `g` or `h` for Spanish layout).

Use `/predefined` or `/goal` to position the fingertip directly on that key:

```bash
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [0.35, 0.0, 0.30, 0.0, 0.0]}" --once
# Then nudge with more /goal messages until the tip is exactly on the center key.
```

### 4.2 — Read the arm's pose and capture the calibration anchors.

```bash
ros2 topic echo /arm_ik/debug_status --once
# Look for: "goal_xyz":{"x":<X>,"y":<Y>,"z":<Z>}
```

Record those three numbers. Then push them into the coordinator:

```bash
ros2 param set /typing_coordinator base_x   <X>   # arm-X at the panel face
ros2 param set /typing_coordinator base_y   <Y>   # arm-Y at the center key
ros2 param set /typing_coordinator target_z <Z>   # arm-Z at the center key
```

### 4.3 — (Optional, only for speed) Capture a closer between-presses parking pose.

By default the coordinator returns to the hardcoded `INTERMEDIATE` pose
(0.20 m forward, 0.60 m up) between every keypress. This always works without
calibration but adds a couple of seconds per key because the arm has to travel
back from there each time.

If you want faster typing, hand-drive the arm via `/goal` or `/predefined`
commands until the EE is a few cm behind the keyboard (clear of the panel,
camera still seeing the full layout). Then capture the **measured joint angles**
as the keyboard home (joint-space variant — replays exact joint pose, bypasses IK):

```bash
ros2 topic pub /predefined std_msgs/msg/String "{data: 'SET_KEYBOARD_HOME_JOINTS'}" --once
```

Verify it was captured:

```bash
ros2 topic echo /arm_ik/debug_status --once | grep joint_keyboard_home_set
# Expect: "joint_keyboard_home_set":true
```

Then re-launch (or `ros2 param set`) with
`return_to_base_command:=KEYBOARD_HOME_JOINTS`. Same exact pose every time, no IK
branch ambiguity, no Cartesian re-solve.

(The older Cartesian `SET_KEYBOARD_HOME` / `KEYBOARD_HOME` commands still exist
but capture the last *commanded* xyz instead of measured joints — joint-space is
preferred when `/joint_states` is healthy.)

You can leave this for later — start with `INTERMEDIATE`, get the system working,
then optimize.

### 4.4 — Drive the arm back to the camera-view position.

```bash
ros2 topic pub /predefined std_msgs/msg/String "{data: 'HOME'}" --once
```

The ZED should now see the full keyboard again.

---

## 5. Contact-button workaround (until limit switch is wired)

The press loop normally completes when a contact button on the end effector publishes
`true` on `/keyboard/contact_pressed`. Until that button exists, the code treats
**reaching `servo_press_max_travel_m` of commanded forward motion** as a successful
press — by that point, the EE has been stalled against the panel for the last portion
of the press attempt.

Nothing to configure for this — it's the in-code default behavior. Just be aware:

- `servo_press_max_travel_m` (default `0.015`, i.e. 15 mm) is now also the
  "definitely contacted" threshold. Tune if you change hover distance.
- If you happen to want to fake contact manually for one keypress:
  ```bash
  ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: true}" --once
  ```

When the real button is wired up, this fallback stays in place as a safety net: a
button failure no longer hangs the loop forever.

---

## 6. Enable motion and queue a word

### 6.1 — Queue the word (headless / no-monitor — recommended for competition)

Publish the word on `/keyboard/type_word`. Single char = one key, longer string = autonomous typing of the whole word:

```bash
# Whole word
ros2 topic pub /keyboard/type_word std_msgs/msg/String "{data: 'holamundo'}" --once

# Single key
ros2 topic pub /keyboard/type_word std_msgs/msg/String "{data: 'h'}" --once
```

No `>` prefix needed — the topic infers autonomous vs single-key from length.

### 6.1b — Alternative: queue via OpenCV window (only if a monitor is attached)

Click into the "ArUco Detection" window so it captures keystrokes. Type `>holamundo` + Enter for a word, or a single character + Enter for one key.

### 6.2 — Verify motion is armed:

If you launched with `motion_enabled:=true` (section 3.2), the arm is already armed — no action needed. The arm will begin executing keys as soon as the queue has an entry and vision is TRACKING.

If the coordinator self-halted mid-run (retry cap hit, or you launched without the flag), re-arm with:

```bash
ros2 param set /typing_coordinator motion_enabled true
```

The arm executes each key in sequence: align → press (max-travel reached) →
retract → return-to-base → next key.

---

## 7. Emergency stop and clean shutdown

> **Safety note.** In servo mode (the URC primary path), `arm_node` publishes joint
> commands regardless of `publish_on_action`. That flag only gates the legacy *action*
> mode. The single master gate for servo mode is `motion_enabled`: set it to `false`
> to stop the arm from accepting any new commands.

### 7.1 — Emergency hold during a run

```bash
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

The coordinator immediately enters `EMERGENCY_HOLD` and stops issuing /goal commands.
The arm's interpolator coasts to its last commanded pose and stops there.

Release:

```bash
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

### 7.2 — End-of-session

```bash
ros2 param set /typing_coordinator motion_enabled false
ros2 topic pub /predefined std_msgs/msg/String "{data: 'HOME'}" --once
# Then Ctrl+C in each terminal.
```

---

## 8. Live debug topics

```bash
# Coordinator state, gates, current key, servo command, intrinsics, PI state
ros2 topic echo /keyboard/coordinator_debug

# Arm internals: target/last_published, tracking error vs /joint_states
ros2 topic echo /arm_ik/debug_status

# Servo phase string
ros2 topic echo /keyboard/servo_state

# What the arm is being commanded to do
ros2 topic echo /goal
ros2 topic hz /goal       # should be ~10-12 Hz during ALIGNING
```

---

## 9. Tuning knobs (set via `ros2 param set /typing_coordinator <name> <value>`)

### Master gates

| Param | Default | Effect |
|---|---|---|
| `motion_enabled` | `false` | Master gate. Nothing moves until `true`. |
| `min_confidence` | `0.3` | Min target confidence (0–1) to act. |
| `required_state` | `TRACKING` | Vision state needed before motion. |

### Alignment loop

| Param | Default | Effect |
|---|---|---|
| `servo_xy_step_max_m` | `0.003` | Max YZ correction per tick (m). Higher = faster but riskier. |
| `servo_align_enter_thresh_px` | `12.0` | Pixel error band to enter "aligned." |
| `servo_align_exit_thresh_px` | `20.0` | Pixel error to leave "aligned" (hysteresis). |
| `servo_ki_y_per_px_per_s` | `0.0001` | Integral gain Y. Set 0 to disable I. |
| `servo_ki_z_per_px_per_s` | `0.0001` | Integral gain Z. |
| `servo_deadband_px` | `4.0` | Pixel error inside which P+I contribute zero. |

### Press / retract

| Param | Default | Effect |
|---|---|---|
| `servo_press_step_m` | `0.0015` | Per-tick forward step into the panel (m). |
| `servo_press_max_travel_m` | `0.015` | Max forward commanded travel. Also "contact reached" without a button. |
| `servo_press_timeout_sec` | `2.0` (real) / `10.0` (launch) | Press abort timer. |
| `servo_press_direction_sign` | `+1.0` | `+1` presses in +arm-X. Flip if press moves away from panel. |
| `servo_press_xy_scale` | `0.6` | YZ gain multiplier during press (still corrects, more gently). |
| `servo_retract_step_m` | `0.0025` | Per-tick backoff after press. |
| `return_to_base_command` | `INTERMEDIATE` | `/predefined` string sent between keys. `INTERMEDIATE` is always available; switch to `KEYBOARD_HOME_JOINTS` (section 4.3) for faster typing. |

### Calibration anchors (set during section 4)

| Param | Effect |
|---|---|
| `base_x` | Arm-X at the panel face. |
| `base_y` | Arm-Y at the image-center key. |
| `target_z` | Arm-Z at the image-center key. |
| `scale_y_per_px` | Meters of arm-Y per pixel of horizontal offset. |
| `scale_z_per_px` | Meters of arm-Z per pixel of vertical offset. |

### Arm-side (set via `ros2 param set /arm_node`)

| Param | Default | Effect |
|---|---|---|
| `publish_on_action` | `false` | Must be `true` for the arm to actually move. |
| `max_step_deg_per_tick` | `1.5` | Per-joint rate limit at 50 Hz (→ 75°/s). |
| `start_pose_deg` | `[0, 190, -140, -50, 0]` | Assumed start pose if `/joint_states` is absent. Matches rover rest pose. |
| `joint_state_topic` | `/joint_states` | Where to subscribe for joint feedback. The bridge node publishes here from `/arm_feedback/jointN_deg`. |

---

## 10. Troubleshooting

### Nothing moves after `motion_enabled=true`

1. `ros2 topic echo /keyboard/coordinator_debug --once` — read `servo_phase`:
   - `WAIT_TARGET`: vision lost the keyboard. Check the OpenCV window.
   - `WAIT_CONFIDENCE`: vision unsure. Lower `min_confidence` temporarily.
   - `WAIT_STATE`: tracker isn't in TRACKING yet. Look at `/keyboard/state`.
   - `WAIT_SERVO_INIT`: initial servo command failed (IK rejected — outside arm reach). Check `base_x`.
2. Confirm `publish_on_action=true` on arm_node.

### Arm moves away from the key, not toward it

`servo_press_direction_sign` is flipped. Set to the opposite of current value:

```bash
ros2 param set /typing_coordinator servo_press_direction_sign -1.0
```

### YZ alignment oscillates / overshoots

Either gains are too high or `camera_fx/fy` defaulted to 700 because CameraInfo
hasn't been received. Check:

```bash
ros2 topic echo /keyboard/coordinator_debug --once | grep -E "fx|servo_gains"
```

If `source: "depth"`, geometric gain is active. If `source: "param"`, lower
`servo_y_gain_m_per_px` and `servo_z_gain_m_per_px` until smooth.

### Press always reports failure

This shouldn't happen anymore — the max-travel workaround treats every full press as
success. If it does, check `/keyboard/coordinator_debug` for `last_goal_result` and
`servo_phase` transitions. Could be target loss mid-press (confidence drop), which is
an abort, not a press failure.

### Arm leaps when motion is first enabled

`/joint_states` wasn't being read at bootstrap. The arm_node default
`start_pose_deg=[0, 190, -140, -50, 0]` matches the rover's physical rest pose
(per the quantum_interface UI), so this should rarely happen with the rover.

If your arm is in a different physical pose at startup:

- Fix the firmware to publish `/joint_states` so bootstrap is automatic, or
- Restart arm_node with `start_pose_deg` matching the actual rest pose:
  ```bash
  ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true -p start_pose_deg:='[<q1>, <q2>, <q3>, <q4>, <q5>]'
  ```

---

## 11. No-hardware simulation

For development without a ZED or arm:

```bash
ros2 launch zed_aruco no_hardware_integration.launch.py servo_mode_enabled:=true motion_enabled:=true text:=hola
```

This swaps in `fake_vision_publisher` and `fake_execute_key_server`.
