# Camera Optimization Guide
# Universal system for Logitech webcams

## Quick Start

### For C920 (1080p camera):
```bash
ros2 launch aruco_py aruco_optimized.launch.py camera_config:=camera_c920.yaml
```

### For 720p Logitech camera:
```bash
ros2 launch aruco_py aruco_optimized.launch.py camera_config:=camera_720p.yaml
```

### Without optimization (use camera defaults):
```bash
ros2 launch aruco_py aruco_optimized.launch.py optimize_camera:=false
```

### Custom device:
```bash
ros2 launch aruco_py aruco_optimized.launch.py device:=/dev/video1
```

## Manual Optimization

Run optimization script manually:
```bash
cd /home/roberd/ros2_ws/src/aruco_py
./scripts/optimize_camera.sh /dev/video0
```

## Adjusting Settings

### Edit config files:
- C920: `config/camera_c920.yaml`
- 720p: `config/camera_720p.yaml`

### Common adjustments:

**For better detection at distance:**
- Use higher resolution: `image_size: [1920, 1080]`

**For better performance:**
- Use lower resolution: `image_size: [640, 480]`

**Change focus (adjust in optimize_camera.sh):**
- Close range (~30-50cm): `focus_absolute=30-50`
- Medium range (~50cm-1m): `focus_absolute=15-30`
- Far range (>1m): `focus_absolute=0-15`

**Change exposure (adjust in optimize_camera.sh):**
- Bright light: `exposure_absolute=100-130`
- Normal light: `exposure_absolute=130-180`
- Low light: `exposure_absolute=180-250`

## Troubleshooting

**Camera not detected:**
```bash
ls -la /dev/video*
v4l2-ctl --list-devices
```

**Check current camera settings:**
```bash
v4l2-ctl -d /dev/video0 --list-ctrls
```

**Reset camera to defaults:**
```bash
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=3
v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=1
```

## Adding New Camera

1. Create config file: `config/camera_MODELNAME.yaml`
2. Copy from existing config and adjust resolution
3. Test: `ros2 launch aruco_py aruco_optimized.launch.py camera_config:=camera_MODELNAME.yaml`
