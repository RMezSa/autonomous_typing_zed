# Quick Start Guide - ArUco Keyboard Tracking

## Installation Complete! ✓

All files have been successfully installed:
- Launch files: `install/aruco_py/share/aruco_py/launch/`
- Config files: `install/aruco_py/share/aruco_py/config/`
- Scripts: `install/aruco_py/share/aruco_py/scripts/`
- Documentation: `CAMERA_SETUP.md`

## Usage

### 1. Launch with Logitech C920 (default)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch aruco_py aruco_optimized.launch.py
```

### 2. Launch with generic 720p camera
```bash
ros2 launch aruco_py aruco_optimized.launch.py camera_config:=camera_720p.yaml
```

### 3. Use different camera device
```bash
ros2 launch aruco_py aruco_optimized.launch.py device:=/dev/video2
```

### 4. Skip camera optimization (if already configured)
```bash
ros2 launch aruco_py aruco_optimized.launch.py optimize_camera:=false
```

### 5. Combine options
```bash
ros2 launch aruco_py aruco_optimized.launch.py \
  camera_config:=camera_720p.yaml \
  device:=/dev/video2 \
  optimize_camera:=true
```

## What Happens at Launch

1. **Camera Optimization** (if `optimize_camera:=true`):
   - Detects camera model automatically
   - Applies optimal settings for ArUco detection
   - Sets resolution, exposure, focus, gain
   - Shows ✓/✗/⊘ status for each control

2. **Camera Launch**:
   - Starts v4l2_camera node with config
   - Publishes to `/image_raw` topic

3. **ArUco Node**:
   - Subscribes to camera feed
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

### Camera not detected
```bash
# List available cameras
v4l2-ctl --list-devices

# Test specific camera
v4l2-ctl -d /dev/video0 --all
```

### Manual camera optimization
```bash
~/ros2_ws/install/aruco_py/share/aruco_py/scripts/optimize_camera.sh /dev/video0
```

### Permission denied on script
```bash
chmod +x ~/ros2_ws/install/aruco_py/share/aruco_py/scripts/optimize_camera.sh
```

## Next Steps

1. **Test Camera System**: Launch and verify detection quality
2. **Fine-tune Settings**: Edit config YAML files for your environment
3. **Phase 2 Optimizations** (future):
   - ROI-based detection
   - Smart parameter adaptation
   - Multi-scale detection

For detailed camera setup and tuning, see `CAMERA_SETUP.md`
