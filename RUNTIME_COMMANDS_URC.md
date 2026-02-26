# Runtime Commands - URC quick start (no TF, closed loop)

## 1) Build (only first time on this machine or after code changes)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Fix for symlink error if it appears
rm -rf build/typing_interfaces

# Build the 3 packages
colcon build --symlink-install --packages-select typing_interfaces arm_ik zed_aruco

source install/setup.bash
```

---

## 2) Start system (3 terminals)

### Terminal A - arm node

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

### Terminal B - camera + coordinator (closed loop, no TF)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_aruco zed_typing_integration.launch.py
```

### Terminal C - debug (optional)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /keyboard/coordinator_debug
```

---

## 3) Calibrate base position (once per session)

```bash
# Move arm near keyboard center
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [0.30, 0.0, 0.15, 0.0, -75.0]}" --once

# Mover más hacia adelante (x aumenta)
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [0.35, 0.0, 0.15, 0.0, -75.0]}" --once

# Mover a la izquierda (y positivo)
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [0.30, 0.05, 0.15, 0.0, -75.0]}" --once

# Subir (z aumenta)
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [0.30, 0.0, 0.20, 0.0, -75.0]}" --once

# Read current arm target
ros2 topic echo /arm_ik/debug_status --once
# Use the goal_xyz.x and goal_xyz.y values

# Set base reference
ros2 param set /typing_coordinator base_x <x_from_debug>
ros2 param set /typing_coordinator base_y <y_from_debug>
```

---

## 4) Enable motion

```bash
ros2 param set /typing_coordinator motion_enabled true
```

---

## 5) Manual contact (no limit switch)

```bash
# Press contact when the arm reaches the key
ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: true}" --once

# Clear contact to allow retract / next key
ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: false}" --once
```

---

## 6) Emergency stop

```bash
# Emergency hold ON
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Emergency hold OFF
ros2 topic pub /keyboard/emergency_stop std_msgs/msg/Bool "{data: false}" --once
```
