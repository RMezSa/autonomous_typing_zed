# Runtime Commands Cheat Sheet

---

## ⚡ URC FULL WORKFLOW — de cero a brazo moviéndose

### PASO 1 — Build (solo la primera vez en esta máquina o si hubo cambios de código)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Si el build falla con "existing path cannot be removed: Is a directory":
rm -rf build/typing_interfaces

# Build de los 3 paquetes del sistema
colcon build --symlink-install --packages-select typing_interfaces arm_ik zed_aruco

source install/setup.bash
```

> **¿Por qué el rm -rf?** Si alguna vez se corrió `colcon build` sin `--symlink-install`,
> queda un directorio basura en `build/typing_interfaces/` que bloquea futuros builds.
> Borrarlo lo resuelve completamente.

---

### PASO 2 — Abre 3 terminales y sourcea en cada una

```bash
# En cada terminal nueva:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

### PASO 3 — Terminal A: lanza el brazo

```bash
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true
```

Debes ver: `arm_node ready. Action publish_on_action=true`

---

### PASO 4 — Terminal B: lanza cámara + coordinador

```bash
ros2 launch zed_aruco zed_typing_integration.launch.py
```

Sin argumentos. Los defaults ya están configurados para URC:
- `servo_mode_enabled=true` → lazo cerrado
- `use_tf_targeting=false` → sin TF, sin mediciones físicas
- `require_transform_valid=false` → no depende de TF
- `motion_enabled=false` → seguro, el brazo no se mueve aún

Espera a ver `TRACKING` en la ventana de OpenCV (la cámara detecta el teclado).

---

### PASO 5 — Terminal C: calibración de posición base (una vez por sesión)

```bash
# Mueve el brazo manualmente cerca del centro del teclado
# Ajusta estos valores hasta que el brazo quede sobre el área central del teclado:
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [0.30, 0.0, 0.15, 0.0, -75.0]}" --once

# Lee la posición actual del brazo
ros2 topic echo /arm_ik/debug_status --once
# Busca: "goal_xyz":{"x":0.30,"y":0.0,...}

# Setea esos valores en el coordinador
ros2 param set /typing_coordinator base_x 0.30   # ← el x que leíste
ros2 param set /typing_coordinator base_y 0.0    # ← el y que leíste
```

---

### PASO 6 — Habilita movimiento

```bash
ros2 param set /typing_coordinator motion_enabled true
```

El brazo ahora responde a las teclas detectadas por la cámara.

---

### PASO 7 — Simula el contact durante cada keypress (sin limit switch físico)

```bash
# Cuando veas que el brazo llegó a la tecla → activa contact:
ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: true}" --once

# El coordinador retrae automáticamente y pasa a la siguiente tecla.
# Si quieres limpiar el contact manualmente:
ros2 topic pub /keyboard/contact_pressed std_msgs/msg/Bool "{data: false}" --once
```

---

### Por qué no hay errores anteriores

| Error anterior | Causa | Solución aplicada |
|---|---|---|
| `arm_base TF does not exist` | TF habilitado por defecto, sin bridge | Default cambiado a `use_tf_targeting=false` |
| `ExecuteKey action server not available` | arm_node no estaba corriendo | Se lanza explícito en Terminal A antes del launch |
| `typing_interfaces build failed` | Build cache corrupto | `rm -rf build/typing_interfaces` antes del colcon |
| `zed_aruco not found` | Cascada del error de build anterior | Resuelto al arreglar el build |

---

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
  use_tf_targeting:=false \
  require_transform_valid:=false \
  enable_calibration_probe:=false
```

### Real execution bring-up — competition mode (no TF, recommended)

> **This is the intended URC approach.** No camera-to-arm measurement needed.
> Set `base_x/base_y` = arm position when hovering over the key at the image center (see section 8 for calibration workflow).
> Tune `scale_x_per_px` and `scale_y_per_px` empirically until off-center keys land correctly.

```bash
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=true \
  use_tf_targeting:=false \
  require_transform_valid:=false \
  min_confidence:=0.2
```

Then at runtime set calibrated values:

```bash
ros2 param set /typing_coordinator base_x <arm_x_at_image_center>
ros2 param set /typing_coordinator base_y <arm_y_at_image_center>
ros2 param set /typing_coordinator scale_x_per_px 0.00035
ros2 param set /typing_coordinator scale_y_per_px 0.00035
```

### Real execution bring-up — TF mode (optional, requires physical calibration)

> Only use this if you have pre-measured the camera-to-arm transform.
> See section 8 for the full TF calibration workflow.

```bash
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=true \
  use_tf_targeting:=true \
  require_transform_valid:=true \
  min_confidence:=0.2 \
  static_tf_enabled:=true \
  static_tf_x:=<measured_x_m> \
  static_tf_y:=<measured_y_m> \
  static_tf_z:=<measured_z_m> \
  static_tf_roll:=-1.5708 \
  static_tf_pitch:=<camera_tilt_rad> \
  static_tf_yaw:=-1.5708
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

### Competition calibration workflow (no-TF mode, `use_tf_targeting:=false`)

This approach requires no tape measure or TF math. You calibrate by driving the arm to two reference positions.

**What the parameters mean:**
- `base_x/base_y`: arm XY position (in arm_base frame) when the arm is hovering over the key that sits at the **image center**
- `scale_x_per_px`: how many meters the arm moves in X per pixel of vertical image offset (default 0.00035)
- `scale_y_per_px`: how many meters the arm moves in Y per pixel of horizontal image offset (default 0.00035)

**Step 1 – Find base_x and base_y**

```bash
# Arm node must be running
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true

# Drive arm manually to hover over the key at the image center
ros2 topic pub /goal std_msgs/msg/Float64MultiArray "{data: [x, y, z, roll, pitch]}" --once

# Read current arm target xyz from debug
ros2 topic echo /arm_ik/debug_status
# Note the x and y values → these become base_x and base_y
```

**Step 2 – Set calibration values at runtime**

```bash
ros2 param set /typing_coordinator base_x <value_from_step1_x>
ros2 param set /typing_coordinator base_y <value_from_step1_y>
```

**Step 3 – Verify and tune scale factors**

1. Enable motion: `ros2 param set /typing_coordinator motion_enabled true`
2. Command a key that is far from image center (e.g. a corner key)
3. Observe where the arm lands vs where the key actually is
4. If overshooting: decrease scale. If undershooting: increase scale:
```bash
ros2 param set /typing_coordinator scale_x_per_px 0.00030
ros2 param set /typing_coordinator scale_y_per_px 0.00030
```
5. Repeat until the arm lands on the correct key consistently

**Step 4 – Save confirmed values**

Record the confirmed `base_x`, `base_y`, `scale_x_per_px`, `scale_y_per_px` values here for the next session.

```
# Confirmed calibration values (fill in after calibration):
# base_x        = 
# base_y        = 
# scale_x_per_px = 
# scale_y_per_px = 
```

---

### TF calibration workflow (optional — only if NOT using competition mode)

Only required if using `use_tf_targeting:=true`. Requires knowing the physical camera-to-arm transform.

**Step 1 – Orientation starting values**

For a camera mounted level and pointing forward (optical convention offset):
```
static_tf_roll  := -1.5708   # -π/2, always required for optical frame
static_tf_pitch := 0.0       # add downward tilt angle in radians if camera is angled
static_tf_yaw   := -1.5708   # -π/2, always required for optical frame
```

**Step 2 – Iterative refinement using calibration_probe**

```bash
# Terminal A – arm node
ros2 run arm_ik arm_node --ros-args -p publish_on_action:=true

# Terminal B – stack with TF mode
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=false \
  use_tf_targeting:=true \
  enable_calibration_probe:=true \
  static_tf_enabled:=true \
  static_tf_x:=<estimate_m> \
  static_tf_y:=<estimate_m> \
  static_tf_z:=<estimate_m> \
  static_tf_roll:=-1.5708 \
  static_tf_pitch:=0.0 \
  static_tf_yaw:=-1.5708

# Terminal C – watch error live (target: < 0.01 m)
ros2 topic echo /keyboard/calibration_error_m

# Terminal D – publish arm reference (arm tip current position)
ros2 topic pub /keyboard/reference_point_arm geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'arm_base'}, point: {x: 0.30, y: 0.05, z: 0.10}}" --once
```

Adjust `static_tf_x/y/z` until error < 0.01 m across 3+ different arm positions.

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

#### Terminal B: vision + coordinator (safe first, competition/no-TF mode)

```bash
cd /home/roberd/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_aruco zed_typing_integration.launch.py \
  motion_enabled:=false \
  use_tf_targeting:=false \
  require_transform_valid:=false
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
  - if using `use_tf_targeting:=false`, confirm `base_x/base_y` are set to the arm's actual center-key position (not 0.0):
    ```bash
    ros2 param get /typing_coordinator base_x
    ros2 param get /typing_coordinator base_y
    ```

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