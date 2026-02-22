# zed_aruco

A simple ROS 2 package for ArUco marker detection specifically tailored for use with Stereolabs ZED cameras.

## Features
- Detects ArUco markers from ZED camera streams.
- Performs pose estimation and publishes results as `visualization_msgs/MarkerArray`.
- Publishes a debug image with detections drawn.
- Configurable topics and marker sizes.

## Topics
- **Subscribes to**:
  - `image_topic` (default: `/zed2i/zed_node/left/image_rect_color`): The rectified color image from the camera.
  - `camera_info` (automatically derived from `image_topic`): The camera calibration information.
- **Publishes to**:
  - `aruco_markers` (`visualization_msgs/MarkerArray`): The detected markers in 3D space.
  - `aruco_debug_image` (`sensor_msgs/Image`): The image with detections and axes drawn.

## How to use

### Launching everything (ZED camera + ArUco)
```bash
ros2 launch zed_aruco zed_combined.launch.py camera_model:=zed2i marker_size:=0.1
```

### Launching only the ArUco node (if ZED is already running)
```bash
ros2 launch zed_aruco zed_aruco.launch.py image_topic:=/zed2i/zed_node/left/image_rect_color marker_size:=0.1
```

### Parameters
- `image_topic`: The topic to subscribe to for images.
- `marker_size`: The real-world size of the ArUco marker (in meters).
- `aruco_dictionary`: The dictionary to use (default: `DICT_4X4_50`).

## No-hardware integration (servo mode)

Use the integration launch to run fake vision + coordinator without arm hardware:

```bash
ros2 launch zed_aruco no_hardware_integration.launch.py servo_mode_enabled:=true motion_enabled:=true use_tf_targeting:=false text:=hola
```

Key servo parameters (in `typing_coordinator`):
- `servo_mode_enabled`: enable/disable continuous XY servo path.
- `servo_xy_step_max_m`: max XY increment per control update.
- `servo_align_enter_thresh_px`: pixel threshold to consider aligned.
- `servo_align_exit_thresh_px`: larger threshold to re-enter correction (hysteresis).
- `servo_align_stable_cycles`: consecutive aligned cycles required before press.
- `servo_press_step_m`: Z increment per press step.
- `servo_press_max_travel_m`: max total Z travel during a press attempt.
- `servo_press_timeout_sec`: timeout for contact detection.
- `servo_retract_step_m`: Z increment per retract step.

Contact input topic for press completion:
- `keyboard/contact_pressed` (`std_msgs/Bool`)

Emergency hold topic:
- `keyboard/emergency_stop` (`std_msgs/Bool`)
  - `true`: immediate hold (no new commands, no retract)
  - `false`: release hold and re-arm from IDLE
