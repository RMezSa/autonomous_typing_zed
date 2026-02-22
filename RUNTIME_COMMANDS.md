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
  min_confidence:=0.7
```

### Run arm action server in real publish mode

```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

### Run arm action server in dry-run mode

```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=false
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
ros2 param set /typing_coordinator motion_enabled true
ros2 param set /typing_coordinator min_confidence 0.5
ros2 param set /typing_coordinator required_state TRACKING
```

### Servo XY tuning

```bash
ros2 param set /typing_coordinator servo_xy_gain_x_m_per_px 0.00035
ros2 param set /typing_coordinator servo_xy_gain_y_m_per_px 0.00035
ros2 param set /typing_coordinator servo_xy_step_max_m 0.003
ros2 param set /typing_coordinator servo_align_enter_thresh_px 8.0
ros2 param set /typing_coordinator servo_align_exit_thresh_px 12.0
ros2 param set /typing_coordinator servo_align_stable_cycles 4
ros2 param set /typing_coordinator servo_cmd_cooldown_sec 0.08
```

### Servo press/retract tuning

```bash
ros2 param set /typing_coordinator servo_press_step_m 0.0015
ros2 param set /typing_coordinator servo_press_max_travel_m 0.015
ros2 param set /typing_coordinator servo_press_timeout_sec 2.0
ros2 param set /typing_coordinator servo_press_direction_sign -1.0
ros2 param set /typing_coordinator servo_retract_step_m 0.0025
```

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
ros2 topic echo /keyboard/mark_done
ros2 topic echo /keyboard/transform_valid
```

### Observe command stream to arm (/goal)

```bash
ros2 topic echo /goal
ros2 topic hz /goal
```

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
