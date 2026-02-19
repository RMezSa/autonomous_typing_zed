# Camera Setup Guide
# ZED2i camera integration

## Quick Start

### Launch ZED2i + ArUco node:
```bash
ros2 launch aruco_py aruco_optimized.launch.py
```

## Adjusting Settings

### Override ZED launch package or file:
```bash
ros2 launch aruco_py aruco_optimized.launch.py \
	zed_launch_package:=zed_wrapper \
	zed_launch_file:=zed_camera.launch.py
```

### Use a specific image topic:
```bash
ros2 launch aruco_py aruco_optimized.launch.py \
	image_topic:=/zed2i/zed_node/left/image_rect_color
```

### Common adjustments:

**Use higher resolution:**
- Improve long-range detection at the cost of FPS.

**Use lower resolution:**
- Improve FPS if detection is too slow.

## Troubleshooting

**ZED node not running:**
- Verify the ZED launch package and file names.
- Check available image topics with `ros2 topic list`.
