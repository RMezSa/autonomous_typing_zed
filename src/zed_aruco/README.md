# zed_aruco

A simple ROS 2 package for ArUco marker detection specifically tailored for use with Stereolabs ZED cameras.

## Features
- Detects ArUco markers from ZED camera streams.
- Estimates keyboard pose, target-key pixel position, and keyboard tracking state.
- Publishes RViz markers, debug images, and typing integration topics.
- Includes no-hardware fake vision/action nodes for coordinator testing.

## Topics
- **Subscribes to**:
  - `image_topic` (default: `/zed2i/zed_node/rgb/color/rect/image`): The rectified color image from the camera.
  - `camera_info` (automatically derived from `image_topic`): The camera calibration information.
  - `depth_topic` (default: `/zed2i/zed_node/depth/depth_registered`): Registered depth image for keyboard-plane depth estimation.
  - `keyboard/mark_done` (`std_msgs/Bool`): Advances autonomous typing after a key is completed.
  - `keyboard/type_word` (`std_msgs/String`): Headless word/key input.
- **Publishes to**:
  - `aruco_markers_3d` (`visualization_msgs/MarkerArray`): The detected markers in 3D space.
  - `aruco_debug_image` (`sensor_msgs/Image`): The image with detections and axes drawn.
  - `keyboard/target_key`, `keyboard/target_point_px`, `keyboard/target_valid`, `keyboard/target_confidence`, `keyboard/state`: Integration topics consumed by `typing_coordinator`.
  - `keyboard/plane_z_m` (`std_msgs/Float32`): Smoothed keyboard-plane depth estimate.

## How to use

### Launching everything (ZED camera + ArUco)
```bash
ros2 launch zed_aruco zed_combined.launch.py camera_model:=zed2i marker_size:=0.1
```

### Launching only the ArUco node (if ZED is already running)
```bash
ros2 launch zed_aruco zed_aruco.launch.py image_topic:=/zed2i/zed_node/rgb/color/rect/image marker_size:=0.1
```

### Parameters
- `image_topic`: The topic to subscribe to for images.
- `depth_topic`: The registered depth topic used for plane-depth estimation.
- `marker_size`: The real-world size of the ArUco marker (in meters).
- `aruco_dictionary`: The dictionary to use (default: `DICT_4X4_50`).
- `target_key_label`: Optional key label to track.
- `homography_hold_seconds`: How long to reuse a recent homography after marker loss.
- `marker_ref_is_bottom_right`: Set true only for marker layouts whose reference corner is bottom-right instead of OpenCV's default top-left.

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
