# Executive Context (for Claude Pro)

Project: autonomous_typing_zed (ROS 2 Humble)

Goal
- Detect ArUco markers from a ZED camera, localize a keyboard, and command a robot arm to press keys autonomously.
- Primary target is Jetson Orin Nano with a ZED2i camera.

Core packages and roles
- zed_aruco
  - zed_aruco_node: ZED image ingest + ArUco detection + keyboard pose + target key tracking.
  - typing_coordinator: converts vision targets into arm goals, handles safety gates, servo mode, and action flow.
  - calibration_probe (optional): TF-based calibration helper.
  - fake_vision_publisher / fake_execute_key_server: simulation without hardware.
- typing_interfaces
  - ExecuteKey.action used for arm commands.
- arm_ik
  - arm_node implements ExecuteKey action server at /arm_ik/execute_key and keeps legacy /goal, /predefined topics.
- aruco_py
  - aruco_node: standalone ArUco detector for non-ZED sources.

Key ROS topics/actions
- From vision: keyboard/target_key, keyboard/target_point_px, keyboard/target_valid, keyboard/target_confidence, keyboard/state
- To arm: /arm_ik/execute_key (ExecuteKey action)
- Safety: keyboard/contact_pressed, keyboard/emergency_stop, keyboard/transform_valid

Recommended entry points
- Runtime quick start: RUNTIME_COMMANDS.md
- ZED + coordinator launch: src/zed_aruco/launch/zed_typing_integration.launch.py
- Node logic: src/zed_aruco/zed_aruco/zed_aruco_node.py, src/zed_aruco/zed_aruco/typing_coordinator.py

Current behavior notes
- Motion is gated (motion_enabled=false by default) and action publish can be dry-run.
- Servo mode provides closed-loop XY alignment + press/retract with contact input.

What to analyze
- Vision -> typing pipeline robustness, topic interfaces, safety gates, and configuration parameters.
- Mapping from image-space targets to arm-space goals (TF mode vs heuristic mode).
