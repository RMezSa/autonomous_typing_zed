# Quick Start Guide - ArUco Keyboard Tracking

## Installation Complete! ✓

All files have been successfully installed:
- Launch files: `install/aruco_py/share/aruco_py/launch/`
- Config files: `install/aruco_py/share/aruco_py/config/`
- Scripts: `install/aruco_py/share/aruco_py/scripts/`
- Documentation: `CAMERA_SETUP.md`

## Usage

### 1. Launch ZED2i + ArUco node
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch aruco_py aruco_optimized.launch.py
```

### 2. Use a specific image topic
```bash
ros2 launch aruco_py aruco_optimized.launch.py \
   image_topic:=/zed2i/zed_node/left/image_rect_color
```

### 3. Override ZED launch package or file
```bash
ros2 launch aruco_py aruco_optimized.launch.py \
   zed_launch_package:=zed_wrapper \
   zed_launch_file:=zed_camera.launch.py
```

## What Happens at Launch

1. **ZED Launch**:
   - Starts the ZED2i launch file
   - Publishes camera topics

2. **ArUco Node**:
   - Subscribes to the configured image topic
   - Detects 4 ArUco markers (2cm, DICT_7X7_50)
   - Tracks keyboard and keys with dual-mode pipeline
   - Phase 1 optimizations active:
     - CLAHE preprocessing for better contrast
     - Multi-threshold fallback detection
     - Optimized parameters for small distant markers

## Troubleshooting

### Launch file not found
```bash
cd ~/ros2_ws
colcon build --packages-select aruco_py
source install/setup.bash
```

### ZED image topic not found
```bash
ros2 topic list
```

## Next Steps

1. **Test Camera System**: Launch and verify detection quality
2. **Fine-tune Settings**: Edit config YAML files for your environment
3. **Phase 2 Optimizations** (future):
   - ROI-based detection
   - Smart parameter adaptation
   - Multi-scale detection

For detailed camera setup and tuning, see `CAMERA_SETUP.md`
