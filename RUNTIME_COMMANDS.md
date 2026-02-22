# Runtime Commands Cheat Sheet

## 0) Environment

```bash
cd /home/roberd/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 1) Build

### Build only relevant packages

```bash
colcon build --symlink-install --packages-select typing_interfaces arm_ik zed_aruco
source install/setup.bash
```

### Build only zed_aruco (fast iteration)

```bash
colcon build --symlink-install --packages-select zed_aruco
source install/setup.bash
```

---

## 2) Real stack launch (camera + coordinator integration)

### Safe calibration-first bring-up (no motion)

```bash
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=false \
  enable_calibration_probe:=true \
  static_tf_enabled:=true
```

### Real execution bring-up (motion enabled)

```bash
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=true \
  use_tf_targeting:=true \
  require_transform_valid:=true \
  min_confidence:=0.2
```

### Run arm action server in real publish mode

```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

### Run arm action server in dry-run mode

```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=false
```

### Set and use keyboard-specific home (arm_ik)

```bash
# 1) Move arm manually (via /goal or predefined) until it reaches desired keyboard home
# 2) Save current arm target as keyboard home
ros2 topic pub /predefined std_msgs/msg/String "{data: 'SET_KEYBOARD_HOME'}" --once

# 3) Return to saved keyboard home later
ros2 topic pub /predefined std_msgs/msg/String "{data: 'KEYBOARD_HOME'}" --once
```

---

## 3) No-hardware simulation launch

### Basic simulation (legacy action flow)

```bash
ros2 launch zed_aruco no_hardware_integration.launch.py \
  servo_mode_enabled:=false \
  motion_enabled:=true \
  use_tf_targeting:=false \
  text:=hola
```

### Servo simulation (ALIGN + PRESS/RETRACT logic)

```bash
ros2 launch zed_aruco no_hardware_integration.launch.py \
  servo_mode_enabled:=true \
  motion_enabled:=true \
  use_tf_targeting:=false \
  text:=hola
```

---

## 4) Runtime parameter tuning

### Inspect coordinator params

```bash
ros2 param list /typing_coordinator
ros2 param get /typing_coordinator servo_mode_enabled
```

### Core gates

```bash
ros2 param set /typing_coordinator motion_enabled true        # master motion gate (true = allow coordinator motion commands)
ros2 param set /typing_coordinator min_confidence 0.2         # min target confidence [0..1] required to move
ros2 param set /typing_coordinator required_state TRACKING     # required vision state label before moving
```

### Servo XY tuning

```bash
ros2 param set /typing_coordinator servo_xy_gain_x_m_per_px 0.00035   # X correction gain [meters per pixel] from image error
ros2 param set /typing_coordinator servo_xy_gain_y_m_per_px 0.00035   # Y correction gain [meters per pixel] from image error
ros2 param set /typing_coordinator servo_xy_step_max_m 0.003           # max XY correction per control update [m] (3 mm)
ros2 param set /typing_coordinator servo_align_enter_thresh_px 8.0     # "aligned" threshold [px] to enter stable-aligned logic
ros2 param set /typing_coordinator servo_align_exit_thresh_px 12.0     # drift threshold [px] to leave aligned band (hysteresis)
ros2 param set /typing_coordinator servo_align_stable_cycles 4          # consecutive in-threshold cycles needed before press
ros2 param set /typing_coordinator servo_cmd_cooldown_sec 0.08          # min time between sent motion commands [s]
```

### Servo press/retract tuning

```bash
ros2 param set /typing_coordinator servo_press_step_m 0.0015            # Z step per press update [m] (1.5 mm)
ros2 param set /typing_coordinator servo_press_max_travel_m .15         # max total Z press travel [m] before abort/retract
ros2 param set /typing_coordinator servo_press_timeout_sec 10.0         # max press duration [s] before abort/retract
ros2 param set /typing_coordinator servo_press_direction_sign -1.0      # Z press direction (-1 or +1 depending on arm frame)
ros2 param set /typing_coordinator servo_press_xy_scale 0.6             # XY correction multiplier during press (0.6 = 60%)
ros2 param set /typing_coordinator servo_retract_step_m 0.0025          # Z step per retract update [m] (2.5 mm)
```

### Return-to-base behavior (after each key)

```bash
ros2 param set /typing_coordinator return_to_base_enabled true           # enable return-to-base state after retract
ros2 param set /typing_coordinator return_to_base_on_failure true        # also return to base when press fails
ros2 param set /typing_coordinator return_to_base_command KEYBOARD_HOME  # /predefined command sent for base return
ros2 param set /typing_coordinator return_to_base_wait_sec 1.0           # wait time [s] before considering return complete
```

Notes:
- `return_to_base_enabled=true` makes servo state machine call `/predefined` with `KEYBOARD_HOME` after retract.
- `return_to_base_on_failure=true` also returns to base when press failed (timeout/no-contact).
- Stop condition is timed (`return_to_base_wait_sec`) because real arm pose feedback is not yet integrated.

---

## 5) Emergency stop (HOLD, no retract)

### Assert emergency hold

```bash
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

### Release emergency hold

```bash
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

---

## 6) Contact topic (for press completion)

### If no physical button yet, manually simulate contact

```bash
ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: true}" --once
```

### Clear contact

```bash
ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: false}" --once
```

### Keyboard home commands in arm_ik

```bash
# Capture keyboard home from current arm target state
ros2 topic pub /predefined std_msgs/msg/String "{data: 'SET_KEYBOARD_HOME'}" --once

# Move to captured keyboard home
ros2 topic pub /predefined std_msgs/msg/String "{data: 'KEYBOARD_HOME'}" --once
```

---

## 7) Useful topic/action diagnostics

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 action info /arm_ik/execute_key
```

### Observe vision/coordinator pipeline

```bash
ros2 topic echo /keyboard/target_key
ros2 topic echo /keyboard/target_valid
ros2 topic echo /keyboard/target_confidence
ros2 topic echo /keyboard/state
ros2 topic echo /keyboard/servo_state
ros2 topic echo /keyboard/coordinator_debug
ros2 topic echo /keyboard/mark_done
ros2 topic echo /keyboard/transform_valid
```

### Observe command stream to arm (/goal)

```bash
ros2 topic echo /goal
ros2 topic hz /goal
```

### Observe arm execution debug status

```bash
ros2 topic echo /arm_ik/debug_status
```

Debug topic meaning:
- `/keyboard/coordinator_debug`: coordinator mode, current gate/state, key, confidence, active servo command xyz, and last action result.
- `/arm_ik/debug_status`: current arm target xyz/rp, last command source (`/goal`, `/predefined`, or action), and latest IK/action result.

---

## 8) Calibration/probe topics (real camera mode)

```bash
ros2 topic echo /keyboard/target_point_arm
ros2 topic echo /keyboard/reference_point_arm
ros2 topic echo /keyboard/calibration_error_m
```

---

## 9) Clean restart sequence

```bash
# 1) Stop running launches/processes (Ctrl+C)
# 2) Rebuild if code changed
colcon build --symlink-install --packages-select zed_aruco arm_ik typing_interfaces
# 3) Re-source
source install/setup.bash
# 4) Relaunch
```
```bash
#save home, keyboard
ros2 topic pub /predefined std_msgs/msg/String "{data: 'SET_KEYBOARD_HOME'}" --once
#return to home, keyboard
ros2 topic pub /predefined std_msgs/msg/String "{data: 'KEYBOARD_HOME'}" --once
```

---

## 10) Hardware bring-up debug checklist (first real run)

### A) Terminal setup

#### Terminal A: arm node (real publish)

```bash
cd /home/roberd/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

#### Terminal B: vision + coordinator (safe first)

```bash
cd /home/roberd/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=false \
  enable_calibration_probe:=true \
  static_tf_enabled:=true
```

#### Terminal C: live debug watchers

```bash
cd /home/roberd/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /keyboard/coordinator_debug
```

Open a 4th terminal (or split) for arm debug:

```bash
cd /home/roberd/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /arm_ik/debug_status
```

### B) Set typing target (word/key)

- Click the OpenCV window so keyboard input is captured.
- Type `>holamundo` then press Enter to queue autonomous typing.
- Type `a` then Enter to target one key only.

### C) Arm motion enable (after checks)

```bash
ros2 param set /typing_coordinator motion_enabled true
```

### D) Safety controls during run

```bash
# Emergency hold ON
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Emergency hold OFF
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

### E) Common scenarios and quick fixes

- No movement at all:
  - `ros2 param get /typing_coordinator motion_enabled` must be `true`.
  - arm node must run with `publish_on_action:=true`.
  - check `/keyboard/coordinator_debug` for gate state (`WAIT_TARGET`, `WAIT_STATE`, etc.).

- Stuck in `WAIT_CONFIDENCE`:
  - lower threshold temporarily:
    ```bash
    ros2 param set /typing_coordinator min_confidence 0.2
    ```

- Stuck in `WAIT_STATE`:
  - ensure tracker reaches `TRACKING`, or relax requirement:
    ```bash
    ros2 param set /typing_coordinator required_state TRACKING
    ```

- Press goes wrong way (away from key):
  - flip sign:
    ```bash
    ros2 param set /typing_coordinator servo_press_direction_sign 1.0
    ```
  - if that is worse, set it back to `-1.0`.

- Oscillation/jitter in XY align:
  - reduce gains / step:
    ```bash
    ros2 param set /typing_coordinator servo_xy_gain_x_m_per_px 0.00025
    ros2 param set /typing_coordinator servo_xy_gain_y_m_per_px 0.00025
    ros2 param set /typing_coordinator servo_xy_step_max_m 0.002
    ```

- Press timeout before contact:
  - verify contact topic toggles,
  - increase timeout slightly:
    ```bash
    ros2 param set /typing_coordinator servo_press_timeout_sec 12.0
    ```

- Workspace blocked warnings:
  - verify calibration/base frame,
  - then adjust workspace limits cautiously.

### F) End-of-session safe stop

```bash
# 1) Emergency hold (optional but recommended)
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: true}" --once

# 2) Return to keyboard home
ros2 topic pub /predefined std_msgs/msg/String "{data: 'KEYBOARD_HOME'}" --once

# 3) Stop launch/node processes with Ctrl+C
```