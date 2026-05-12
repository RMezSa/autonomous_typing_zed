import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import json

from std_msgs.msg import String, Bool, Float32, Float64MultiArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from typing_interfaces.action import ExecuteKey


# Hardcoded tuning constants for the servo loop. These were ROS parameters in earlier
# revisions but ballooned the config surface; they're back to constants because they're
# safety/smoothing/pacing values, not per-deployment knobs. Edit here if behavior changes.
_PLANE_Z_STALE_SEC = 2.0        # measured depth older than this falls back to keyboard_plane_z_m param
_INTEGRAL_LEAK_PER_SEC = 0.5    # decays integrator state so stale bias fades
_INTEGRAL_CLAMP_M = 0.005       # max ±meters the integral term can ever command (anti-windup)
_GOAL_COOLDOWN_SEC = 0.3        # min interval between ExecuteKey action goals
_SERVO_CMD_COOLDOWN_SEC = 0.08  # min interval between servo /goal publishes (≈12.5 Hz)
_SERVO_ALIGN_STABLE_CYCLES = 4  # consecutive ticks within enter-threshold before alignment latches
_RETURN_TO_BASE_WAIT_SEC = 1.0  # how long RETURNING_BASE blocks before allowing next key


class TypingCoordinator(Node):
    def __init__(self):
        super().__init__('typing_coordinator')

        self.declare_parameter('action_name', '/arm_ik/execute_key')
        self.declare_parameter('done_topic', 'keyboard/mark_done')
        self.declare_parameter('target_z', 0.12)
        self.declare_parameter('target_roll', 0.0)
        self.declare_parameter('target_pitch', -75.0)
        self.declare_parameter('min_confidence', 0.3) #minimal confidence
        self.declare_parameter('required_state', 'TRACKING')
        self.declare_parameter('accept_dry_run_result', False)
        self.declare_parameter('use_tf_targeting', True)
        self.declare_parameter('arm_base_frame', 'arm_base')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('keyboard_plane_z_m', 0.45)
        self.declare_parameter('camera_fx', 700.0)
        self.declare_parameter('camera_fy', 700.0)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)
        self.declare_parameter('arm_z_offset', 0.0)
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('require_transform_valid', True)
        self.declare_parameter('servo_mode_enabled', False)
        self.declare_parameter('contact_topic', 'keyboard/contact_pressed')
        self.declare_parameter('servo_state_topic', 'keyboard/servo_state')
        self.declare_parameter('emergency_stop_topic', 'keyboard/emergency_stop')
        self.declare_parameter('servo_z_gain_m_per_px', 0.00035)
        self.declare_parameter('servo_y_gain_m_per_px', 0.00035)
        self.declare_parameter('keyboard_plane_z_topic', 'keyboard/plane_z_m')
        # CameraInfo topic. When messages arrive we overwrite camera_fx/fy/cx/cy and
        # image_center_x/y with the live values, so the geometric servo gain (depth/fx)
        # uses real intrinsics instead of the static parameter defaults.
        self.declare_parameter('camera_info_topic', '/zed2i/zed_node/rgb/camera_info')

        # PI integral gains and deadband for the YZ alignment loop. Integral term
        # is always on; defaults are sized to give roughly Kp / 5s integral action
        # at the typical geometric gain (~0.0007 m/px at 0.5 m keyboard depth).
        self.declare_parameter('servo_ki_y_per_px_per_s', 0.0001)
        self.declare_parameter('servo_ki_z_per_px_per_s', 0.0001)
        self.declare_parameter('servo_deadband_px', 4.0)
        self.declare_parameter('servo_xy_step_max_m', 0.003)
        self.declare_parameter('servo_align_enter_thresh_px', 8.0)
        self.declare_parameter('servo_align_exit_thresh_px', 12.0)
        self.declare_parameter('servo_press_step_m', 0.0015)
        self.declare_parameter('servo_press_max_travel_m', 0.015)
        self.declare_parameter('servo_press_timeout_sec', 2.0)
        self.declare_parameter('servo_press_direction_sign', 1.0)
        self.declare_parameter('servo_press_xy_scale', 0.6)
        self.declare_parameter('servo_retract_step_m', 0.0025)
        # 'INTERMEDIATE' (hardcoded pose in arm_node at 0.20/0/0.60) is the safe default:
        # always available without a prior SET_KEYBOARD_HOME step. Override at launch to
        # 'KEYBOARD_HOME' if you want faster between-key motion at the cost of an extra
        # calibration step.
        self.declare_parameter('return_to_base_command', 'INTERMEDIATE')
        self.declare_parameter('debug_status_topic', 'keyboard/coordinator_debug')
        self.declare_parameter('debug_publish_period_sec', 0.25)

        self.declare_parameter('image_center_x', 640.0)
        self.declare_parameter('image_center_y', 360.0)
        self.declare_parameter('base_x', 0.25)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('scale_y_per_px', 0.00035)
        self.declare_parameter('scale_z_per_px', 0.00035)

        action_name = self.get_parameter('action_name').value
        done_topic = self.get_parameter('done_topic').value

        self.target_z = float(self.get_parameter('target_z').value)
        self.target_roll = float(self.get_parameter('target_roll').value)
        self.target_pitch = float(self.get_parameter('target_pitch').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.required_state = str(self.get_parameter('required_state').value)
        self.accept_dry_run_result = bool(self.get_parameter('accept_dry_run_result').value)
        self.use_tf_targeting = bool(self.get_parameter('use_tf_targeting').value)
        self.arm_base_frame = str(self.get_parameter('arm_base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.keyboard_plane_z_m = float(self.get_parameter('keyboard_plane_z_m').value)
        self.camera_fx = float(self.get_parameter('camera_fx').value)
        self.camera_fy = float(self.get_parameter('camera_fy').value)
        self.camera_cx = float(self.get_parameter('camera_cx').value)
        self.camera_cy = float(self.get_parameter('camera_cy').value)
        self.arm_z_offset = float(self.get_parameter('arm_z_offset').value)

        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.require_transform_valid = bool(self.get_parameter('require_transform_valid').value)
        self.servo_mode_enabled = bool(self.get_parameter('servo_mode_enabled').value)
        self.contact_topic = str(self.get_parameter('contact_topic').value)
        self.servo_state_topic = str(self.get_parameter('servo_state_topic').value)
        self.emergency_stop_topic = str(self.get_parameter('emergency_stop_topic').value)
        self.servo_z_gain = float(self.get_parameter('servo_z_gain_m_per_px').value)
        self.servo_y_gain = float(self.get_parameter('servo_y_gain_m_per_px').value)
        plane_z_topic = str(self.get_parameter('keyboard_plane_z_topic').value)
        self.servo_ki_y = float(self.get_parameter('servo_ki_y_per_px_per_s').value)
        self.servo_ki_z = float(self.get_parameter('servo_ki_z_per_px_per_s').value)
        self.servo_deadband_px = float(self.get_parameter('servo_deadband_px').value)
        self.servo_xy_step_max = float(self.get_parameter('servo_xy_step_max_m').value)
        self.servo_align_enter_thresh_px = float(self.get_parameter('servo_align_enter_thresh_px').value)
        self.servo_align_exit_thresh_px = float(self.get_parameter('servo_align_exit_thresh_px').value)
        self.servo_press_step_m = float(self.get_parameter('servo_press_step_m').value)
        self.servo_press_max_travel_m = float(self.get_parameter('servo_press_max_travel_m').value)
        self.servo_press_timeout_sec = float(self.get_parameter('servo_press_timeout_sec').value)
        self.servo_press_direction_sign = float(self.get_parameter('servo_press_direction_sign').value)
        self.servo_press_xy_scale = float(self.get_parameter('servo_press_xy_scale').value)
        self.servo_retract_step_m = float(self.get_parameter('servo_retract_step_m').value)
        self.return_to_base_command = str(self.get_parameter('return_to_base_command').value)
        self.debug_status_topic = str(self.get_parameter('debug_status_topic').value)
        self.debug_publish_period_sec = float(self.get_parameter('debug_publish_period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_center_x = float(self.get_parameter('image_center_x').value)
        self.image_center_y = float(self.get_parameter('image_center_y').value)
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.scale_y_per_px = float(self.get_parameter('scale_y_per_px').value)
        self.scale_z_per_px = float(self.get_parameter('scale_z_per_px').value)

        self.action_client = ActionClient(self, ExecuteKey, action_name)

        self.done_pub = self.create_publisher(Bool, done_topic, 10)
        self.transform_valid_pub = self.create_publisher(Bool, 'keyboard/transform_valid', 10)
        self.goal_pub = self.create_publisher(Float64MultiArray, '/goal', 10)
        self.predefined_pub = self.create_publisher(String, '/predefined', 10)
        self.servo_state_pub = self.create_publisher(String, self.servo_state_topic, 10)
        self.debug_status_pub = self.create_publisher(String, self.debug_status_topic, 10)

        self.create_subscription(String, 'keyboard/target_key', self.on_target_key, 10)
        self.create_subscription(PointStamped, 'keyboard/target_point_px', self.on_target_point, 10)
        self.create_subscription(Bool, 'keyboard/target_valid', self.on_target_valid, 10)
        self.create_subscription(Float32, 'keyboard/target_confidence', self.on_target_confidence, 10)
        self.create_subscription(String, 'keyboard/state', self.on_state, 10)
        self.create_subscription(Bool, self.contact_topic, self.on_contact, 10)
        self.create_subscription(Bool, self.emergency_stop_topic, self.on_emergency_stop, 10)
        self.create_subscription(Float32, plane_z_topic, self.on_plane_z, 10)

        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.camera_info_received = False
        self.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, 10)

        # Measured keyboard plane distance from the ZED depth sensor (via zed_aruco_node).
        # When fresh and `use_geometric_servo_gains` is true, this replaces the hand-tuned
        # servo_y_gain / servo_z_gain with geometric values (depth / focal_length).
        self.measured_plane_z = None
        self.measured_plane_z_time = 0.0

        self.current_key = ''
        self.current_point = None
        self.current_point_msg = None
        self.target_valid = False
        self.target_confidence = 0.0
        self.current_state = 'INIT'

        self.goal_active = False
        self.last_goal_sent_time = 0.0
        self.last_sent_key = ''
        self.blocked_key = ''
        self.transform_valid = False

        self.contact_pressed = False
        self.emergency_stop = False
        self.servo_phase = 'IDLE'
        self.servo_cmd_initialized = False
        self.servo_cmd_x = 0.0
        self.servo_cmd_y = 0.0
        self.servo_cmd_z = self.target_z
        self.servo_aligned_cycles = 0
        self.servo_last_cmd_time = 0.0
        self.servo_cmd_key = ''
        self.servo_press_start_time = 0.0
        self.servo_press_start_x = 0.0
        self.servo_hover_x = 0.0
        self.servo_press_succeeded = False
        self.servo_return_started = False
        self.servo_return_start_time = 0.0
        self.current_goal_handle = None
        self.last_goal_result = 'none'
        self.last_goal_result_message = ''

        # PI state for the YZ alignment loop. Reset on new key and on phase enter to
        # ALIGNING. Components last_p/i are cached for debug exposure only.
        self.servo_pid_integral_y = 0.0
        self.servo_pid_integral_z = 0.0
        self.servo_pid_last_time = None
        self.servo_pid_last_p = (0.0, 0.0)
        self.servo_pid_last_i = (0.0, 0.0)
        self.servo_pid_last_in_deadband = False

        self.create_timer(0.1, self.tick)
        self.create_timer(max(0.05, self.debug_publish_period_sec), self.publish_debug_status)

        self.get_logger().info(
            f"typing_coordinator started (motion_enabled={self.motion_enabled}, "
            f"require_transform_valid={self.require_transform_valid}, servo_mode_enabled={self.servo_mode_enabled})"
        )

    def on_target_key(self, msg: String):
        previous_key = self.current_key
        self.current_key = msg.data.strip()
        if self.current_key != previous_key:
            self.blocked_key = ''
            self.servo_cmd_initialized = False
            self.servo_aligned_cycles = 0
            self.servo_cmd_key = ''
            self.servo_press_succeeded = False
            self.reset_servo_pid_state()
            self.set_servo_phase('IDLE')

    def on_target_point(self, msg: PointStamped):
        self.current_point = (float(msg.point.x), float(msg.point.y))
        self.current_point_msg = msg

    def on_target_valid(self, msg: Bool):
        self.target_valid = bool(msg.data)

    def on_target_confidence(self, msg: Float32):
        self.target_confidence = float(msg.data)

    def on_state(self, msg: String):
        self.current_state = msg.data

    def on_contact(self, msg: Bool):
        self.contact_pressed = bool(msg.data)

    def on_plane_z(self, msg: Float32):
        self.measured_plane_z = float(msg.data)
        self.measured_plane_z_time = self.get_clock().now().nanoseconds * 1e-9

    def on_camera_info(self, msg: CameraInfo):
        # CameraInfo.k is a row-major 3x3 intrinsics matrix:
        #   [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # We only need to read it once — these don't change during a run. After this fires,
        # geometric servo gains and the TF-mode pixel→3D math use the real camera values.
        if self.camera_info_received:
            return
        try:
            fx = float(msg.k[0])
            fy = float(msg.k[4])
            cx = float(msg.k[2])
            cy = float(msg.k[5])
        except (IndexError, TypeError):
            return
        if fx <= 0.0 or fy <= 0.0:
            return
        self.camera_fx = fx
        self.camera_fy = fy
        self.camera_cx = cx
        self.camera_cy = cy
        # The pixel-error reference for the servo loop should be the principal point, not
        # an assumed image center, so the loop converges to the same key the projection math
        # would produce.
        self.image_center_x = cx
        self.image_center_y = cy
        self.camera_info_received = True
        self.get_logger().info(
            f"CameraInfo received: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
        )

    def effective_plane_z(self):
        """Return measured plane depth (m) if fresh, else fall back to the parameter."""
        if self.measured_plane_z is None:
            return self.keyboard_plane_z_m
        age = (self.get_clock().now().nanoseconds * 1e-9) - self.measured_plane_z_time
        if age > _PLANE_Z_STALE_SEC:
            return self.keyboard_plane_z_m
        return self.measured_plane_z

    def effective_servo_gains(self):
        """Return (gain_y, gain_z) for compute_yz_servo_delta.

        When a fresh ZED depth measurement is available, return (depth / fx, depth / fy)
        — the pinhole-projection truth. Otherwise fall back to the hand-tuned parameter
        values, which is what an older sim or no-depth deployment expects.
        """
        if self.measured_plane_z is None:
            return self.servo_y_gain, self.servo_z_gain
        age = (self.get_clock().now().nanoseconds * 1e-9) - self.measured_plane_z_time
        if age > _PLANE_Z_STALE_SEC:
            return self.servo_y_gain, self.servo_z_gain
        if self.camera_fx <= 0.0 or self.camera_fy <= 0.0:
            return self.servo_y_gain, self.servo_z_gain
        depth = float(self.measured_plane_z)
        return depth / self.camera_fx, depth / self.camera_fy

    def on_emergency_stop(self, msg: Bool):
        previous = self.emergency_stop
        self.emergency_stop = bool(msg.data)

        if self.emergency_stop and not previous:
            self.set_servo_phase('EMERGENCY_HOLD')
            self.get_logger().warn('Emergency stop asserted: holding current position, no new commands will be sent.')
            self.servo_cmd_initialized = False
            self.servo_aligned_cycles = 0
            self.servo_press_succeeded = False
            if self.goal_active and self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
        elif (not self.emergency_stop) and previous:
            self.get_logger().info('Emergency stop released: controller re-arming from IDLE.')
            self.set_servo_phase('IDLE')
            self.servo_cmd_initialized = False
            self.servo_aligned_cycles = 0
            self.servo_cmd_key = ''

    def pixel_to_arm_goal(self, px: float, py: float):
        # Vertical keyboard panel: arm-x is fixed (panel face distance).
        # Horizontal pixel offset → arm-y (left/right on panel).
        # Vertical pixel offset → arm-z (up/down on panel).
        dx_px = self.image_center_x - px
        dy_px = self.image_center_y - py

        x = self.base_x
        y = self.base_y + (dx_px * self.scale_y_per_px)
        z = self.target_z + (dy_px * self.scale_z_per_px)
        return x, y, z

    def pixel_to_camera_point(self, px: float, py: float, frame_id: str):
        z = self.effective_plane_z()
        x = (px - self.camera_cx) * z / self.camera_fx
        y = (py - self.camera_cy) * z / self.camera_fy

        cam_point = PointStamped()
        cam_point.header.stamp = self.get_clock().now().to_msg()
        cam_point.header.frame_id = frame_id
        cam_point.point.x = float(x)
        cam_point.point.y = float(y)
        cam_point.point.z = float(z)
        return cam_point

    def to_arm_frame_goal(self, px: float, py: float):
        if self.current_point_msg is None:
            return None

        source_frame = self.current_point_msg.header.frame_id or self.camera_frame
        if not source_frame:
            return None

        cam_point = self.pixel_to_camera_point(px, py, source_frame)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                source_frame,
                Time()
            )
            arm_point = do_transform_point(cam_point, transform)
            self.transform_valid = True
        except TransformException as ex:
            self.transform_valid = False
            self.get_logger().warn(f"TF transform failed ({source_frame} -> {self.arm_base_frame}): {ex}")
            return None

        gx = float(arm_point.point.x)
        gy = float(arm_point.point.y)
        gz = float(arm_point.point.z + self.arm_z_offset)
        return gx, gy, gz

    def publish_transform_valid(self):
        msg = Bool()
        msg.data = bool(self.transform_valid)
        self.transform_valid_pub.publish(msg)

    def publish_servo_state(self):
        msg = String()
        msg.data = self.servo_phase
        self.servo_state_pub.publish(msg)

    def publish_debug_status(self):
        gain_y, gain_z = self.effective_servo_gains()
        depth_fresh = (
            self.measured_plane_z is not None
            and ((self.get_clock().now().nanoseconds * 1e-9) - self.measured_plane_z_time) <= _PLANE_Z_STALE_SEC
        )
        debug = {
            'mode': 'servo' if self.servo_mode_enabled else 'action',
            'motion_enabled': bool(self.motion_enabled),
            'emergency_stop': bool(self.emergency_stop),
            'servo_phase': self.servo_phase,
            'goal_active': bool(self.goal_active),
            'current_key': self.current_key,
            'blocked_key': self.blocked_key,
            'last_sent_key': self.last_sent_key,
            'current_state': self.current_state,
            'required_state': self.required_state,
            'target_valid': bool(self.target_valid),
            'target_confidence': float(self.target_confidence),
            'transform_valid': bool(self.transform_valid),
            'press_contact': bool(self.contact_pressed),
            'servo_cmd': {
                'x': float(self.servo_cmd_x),
                'y': float(self.servo_cmd_y),
                'z': float(self.servo_cmd_z),
            },
            'target_px': {
                'x': float(self.current_point[0]) if self.current_point is not None else None,
                'y': float(self.current_point[1]) if self.current_point is not None else None,
            },
            'plane_z_m': {
                'measured': float(self.measured_plane_z) if self.measured_plane_z is not None else None,
                'fresh': bool(depth_fresh),
                'effective': float(self.effective_plane_z()),
                'source': ('measured' if depth_fresh else 'parameter'),
            },
            'servo_gains': {
                'effective_y': float(gain_y),
                'effective_z': float(gain_z),
                'source': ('depth' if depth_fresh else 'param'),
            },
            'servo_pi': {
                'deadband_px': float(self.servo_deadband_px),
                'in_deadband': bool(self.servo_pid_last_in_deadband),
                'p': {'y': float(self.servo_pid_last_p[0]), 'z': float(self.servo_pid_last_p[1])},
                'i': {'y': float(self.servo_pid_last_i[0]), 'z': float(self.servo_pid_last_i[1])},
                'integral_state': {
                    'y': float(self.servo_pid_integral_y),
                    'z': float(self.servo_pid_integral_z),
                },
            },
            'last_goal_result': self.last_goal_result,
            'last_goal_result_message': self.last_goal_result_message,
        }

        msg = String()
        msg.data = json.dumps(debug, separators=(',', ':'))
        self.debug_status_pub.publish(msg)

    def set_servo_phase(self, phase: str):
        if phase != self.servo_phase:
            self.get_logger().info(f"Servo phase: {self.servo_phase} -> {phase}")
            previous_phase = self.servo_phase
            self.servo_phase = phase
            # Re-entering ALIGNING from a non-aligning phase should restart the loop
            # from a clean slate so that stale integral / derivative state from a prior
            # key (or a failed press) doesn't bias the new attempt.
            if phase == 'ALIGNING' and previous_phase not in ('ALIGNING', 'ALIGN_HOLD'):
                self.reset_servo_pid_state()

    def reset_servo_pid_state(self):
        self.servo_pid_integral_y = 0.0
        self.servo_pid_integral_z = 0.0
        self.servo_pid_last_time = None
        self.servo_pid_last_p = (0.0, 0.0)
        self.servo_pid_last_i = (0.0, 0.0)
        self.servo_pid_last_in_deadband = False

    @staticmethod
    def clamp(value: float, low: float, high: float):
        return max(low, min(high, value))

    def publish_cartesian_goal(self, gx: float, gy: float, gz: float):
        goal_msg = Float64MultiArray()
        goal_msg.data = [
            float(gx),
            float(gy),
            float(gz),
            float(self.target_roll),
            float(self.target_pitch),
        ]
        self.goal_pub.publish(goal_msg)

    def command_return_to_base(self):
        msg = String()
        msg.data = self.return_to_base_command
        self.predefined_pub.publish(msg)

    def initialize_servo_command_pose(self):
        if self.current_point is None:
            return False

        px, py = self.current_point

        if self.use_tf_targeting:
            arm_goal = self.to_arm_frame_goal(px, py)
            if arm_goal is None:
                return False
            gx, gy, gz = arm_goal
            if self.require_transform_valid and not self.transform_valid:
                return False
        else:
            gx, gy, gz = self.pixel_to_arm_goal(px, py)

        self.servo_cmd_x = float(gx)
        self.servo_cmd_y = float(gy)
        self.servo_cmd_z = float(gz)
        self.servo_cmd_initialized = True
        self.servo_cmd_key = self.current_key
        self.servo_aligned_cycles = 0
        self.servo_last_cmd_time = 0.0
        self.servo_hover_x = self.servo_cmd_x
        self.servo_press_succeeded = False
        return True

    def compute_yz_servo_delta(self, px: float, py: float, scale: float = 1.0):
        # Pixel error. Sign convention matches pixel_to_arm_goal: positive dx → target
        # is left of image center → positive arm-y motion (analogously for dy → arm-z).
        # Don't change signs without verifying camera↔arm orientation (project memory
        # flags this as the biggest unverified risk on hardware).
        dx_px = self.image_center_x - px
        dy_px = self.image_center_y - py

        gain_y, gain_z = self.effective_servo_gains()

        # Deadband: inside it, contribute nothing and don't accumulate the integrator.
        # Stops the loop from chasing sub-pixel jitter and prevents windup on noise.
        in_deadband = (
            abs(dx_px) <= self.servo_deadband_px
            and abs(dy_px) <= self.servo_deadband_px
        )
        self.servo_pid_last_in_deadband = in_deadband

        # P term.
        p_y = 0.0 if in_deadband else dx_px * gain_y * scale
        p_z = 0.0 if in_deadband else dy_px * gain_z * scale

        # I term: leaky integrator with anti-windup. Always active (no PID toggle).
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.servo_pid_last_time is None:
            dt = 0.0
        else:
            # Clamp dt so a long idle pause can't smash the integrator on resume.
            dt = max(0.0, min(1.0, now_sec - self.servo_pid_last_time))
        self.servo_pid_last_time = now_sec

        if dt > 0.0:
            # Decay first (so stale bias fades even if we're currently in deadband),
            # then accumulate fresh error only when outside the deadband.
            decay = max(0.0, 1.0 - _INTEGRAL_LEAK_PER_SEC * dt)
            self.servo_pid_integral_y *= decay
            self.servo_pid_integral_z *= decay
            if not in_deadband:
                self.servo_pid_integral_y += dx_px * self.servo_ki_y * dt
                self.servo_pid_integral_z += dy_px * self.servo_ki_z * dt

        self.servo_pid_integral_y = self.clamp(
            self.servo_pid_integral_y, -_INTEGRAL_CLAMP_M, _INTEGRAL_CLAMP_M,
        )
        self.servo_pid_integral_z = self.clamp(
            self.servo_pid_integral_z, -_INTEGRAL_CLAMP_M, _INTEGRAL_CLAMP_M,
        )
        i_y = self.servo_pid_integral_y * scale if not in_deadband else 0.0
        i_z = self.servo_pid_integral_z * scale if not in_deadband else 0.0

        # Cache components for the debug topic.
        self.servo_pid_last_p = (p_y, p_z)
        self.servo_pid_last_i = (i_y, i_z)

        # Per-tick output clamp: no single tick commands more than ±servo_xy_step_max.
        delta_y = self.clamp(p_y + i_y, -self.servo_xy_step_max, self.servo_xy_step_max)
        delta_z = self.clamp(p_z + i_z, -self.servo_xy_step_max, self.servo_xy_step_max)
        return delta_y, delta_z

    def start_press_phase(self, now_sec: float):
        self.servo_press_start_time = now_sec
        self.servo_press_start_x = self.servo_cmd_x
        self.servo_hover_x = self.servo_cmd_x
        self.servo_press_succeeded = False
        self.set_servo_phase('PRESSING')

    def tick_press_phase(self, now_sec: float):
        if self.contact_pressed:
            self.servo_press_succeeded = True
            self.set_servo_phase('RETRACTING')
            return

        if not self.target_valid or self.current_point is None:
            self.get_logger().warn('Target lost during PRESSING. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if self.target_confidence < self.min_confidence:
            self.get_logger().warn('Confidence dropped during PRESSING. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if (now_sec - self.servo_press_start_time) > self.servo_press_timeout_sec:
            self.get_logger().warn('Press timeout reached before contact. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if abs(self.servo_cmd_x - self.servo_press_start_x) >= self.servo_press_max_travel_m:
            # No physical contact sensor yet, so reaching max travel is our success proxy:
            # if we commanded the full press travel without an abort, the EE has been
            # stalled against the panel for the last portion of that travel. Treat that
            # as a successful press. When a real contact_pressed publisher comes online,
            # it will trip earlier (line above) and this branch becomes the fallback.
            self.get_logger().info('Press max travel reached — treating as contact (no button wired).')
            self.servo_press_succeeded = True
            self.set_servo_phase('RETRACTING')
            return

        if (now_sec - self.servo_last_cmd_time) < _SERVO_CMD_COOLDOWN_SEC:
            return

        px, py = self.current_point
        delta_y, delta_z = self.compute_yz_servo_delta(px, py, self.servo_press_xy_scale)

        next_x = self.servo_cmd_x + (self.servo_press_direction_sign * self.servo_press_step_m)
        next_y = self.servo_cmd_y + delta_y
        next_z = self.servo_cmd_z + delta_z

        self.publish_cartesian_goal(next_x, next_y, next_z)
        self.servo_cmd_x = next_x
        self.servo_cmd_y = next_y
        self.servo_cmd_z = next_z
        self.servo_last_cmd_time = now_sec

    def tick_retract_phase(self, now_sec: float):
        if (now_sec - self.servo_last_cmd_time) < _SERVO_CMD_COOLDOWN_SEC:
            return

        next_y = self.servo_cmd_y
        next_z = self.servo_cmd_z
        dx_to_hover = self.servo_hover_x - self.servo_cmd_x

        if abs(dx_to_hover) <= self.servo_retract_step_m:
            next_x = self.servo_hover_x
            at_hover = True
        else:
            next_x = self.servo_cmd_x + self.clamp(dx_to_hover, -self.servo_retract_step_m, self.servo_retract_step_m)
            at_hover = False

        self.publish_cartesian_goal(next_x, next_y, next_z)
        self.servo_cmd_x = next_x
        self.servo_last_cmd_time = now_sec

        if not at_hover:
            return

        self.contact_pressed = False
        self.servo_cmd_initialized = False
        self.servo_cmd_key = ''
        self.servo_aligned_cycles = 0
        self.servo_return_started = False
        self.servo_return_start_time = 0.0

        # After every press attempt (success or failure) we return to base. The two
        # toggles that used to gate this were dropped — operationally we always want
        # the arm clear of the keyboard before evaluating the next key.
        self.set_servo_phase('RETURNING_BASE')

    def tick_return_to_base_phase(self, now_sec: float):
        if not self.servo_return_started:
            self.command_return_to_base()
            self.servo_return_start_time = now_sec
            self.servo_return_started = True
            return

        if (now_sec - self.servo_return_start_time) < _RETURN_TO_BASE_WAIT_SEC:
            return

        self.servo_return_started = False
        self.servo_return_start_time = 0.0

        if self.servo_press_succeeded:
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            self.set_servo_phase('COMPLETE')
            self.get_logger().info(f"Servo typing completed for '{self.current_key}' after return-to-base")
        else:
            self.set_servo_phase('ALIGNING')

    def tick_servo_mode(self, now_sec: float):
        if not self.motion_enabled:
            self.set_servo_phase('IDLE')
            self.servo_aligned_cycles = 0
            return

        if self.servo_phase == 'PRESSING':
            self.tick_press_phase(now_sec)
            return

        if self.servo_phase == 'RETRACTING':
            self.tick_retract_phase(now_sec)
            return

        if self.servo_phase == 'RETURNING_BASE':
            self.tick_return_to_base_phase(now_sec)
            return

        if self.servo_phase == 'COMPLETE':
            self.set_servo_phase('WAIT_NEXT_KEY')
            return

        if self.goal_active:
            return

        if not self.target_valid or not self.current_key or self.current_point is None:
            self.set_servo_phase('WAIT_TARGET')
            self.servo_aligned_cycles = 0
            return

        if self.target_confidence < self.min_confidence:
            self.set_servo_phase('WAIT_CONFIDENCE')
            self.servo_aligned_cycles = 0
            return

        if self.required_state and self.current_state != self.required_state:
            self.set_servo_phase('WAIT_STATE')
            self.servo_aligned_cycles = 0
            return

        if (not self.servo_cmd_initialized) or (self.servo_cmd_key != self.current_key):
            if not self.initialize_servo_command_pose():
                self.set_servo_phase('WAIT_SERVO_INIT')
                return

        px, py = self.current_point
        dx_px = self.image_center_x - px
        dy_px = self.image_center_y - py

        within_enter = (
            abs(dx_px) <= self.servo_align_enter_thresh_px and
            abs(dy_px) <= self.servo_align_enter_thresh_px
        )
        outside_exit = (
            abs(dx_px) > self.servo_align_exit_thresh_px or
            abs(dy_px) > self.servo_align_exit_thresh_px
        )

        if within_enter:
            self.servo_aligned_cycles += 1
            if self.servo_aligned_cycles >= _SERVO_ALIGN_STABLE_CYCLES:
                self.set_servo_phase('ALIGNED_READY_PRESS')
                self.start_press_phase(now_sec)
            else:
                self.set_servo_phase('ALIGN_HOLD')
            return

        if outside_exit:
            self.servo_aligned_cycles = 0

        if (now_sec - self.servo_last_cmd_time) < _SERVO_CMD_COOLDOWN_SEC:
            self.set_servo_phase('ALIGN_RATE_LIMIT')
            return

        delta_y, delta_z = self.compute_yz_servo_delta(px, py)

        next_x = self.servo_cmd_x
        next_y = self.servo_cmd_y + delta_y
        next_z = self.servo_cmd_z + delta_z

        self.publish_cartesian_goal(next_x, next_y, next_z)
        self.servo_cmd_y = next_y
        self.servo_cmd_z = next_z
        self.servo_last_cmd_time = now_sec
        self.set_servo_phase('ALIGNING')

        if self.servo_aligned_cycles > 0:
            self.servo_aligned_cycles = 0

    def tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.publish_transform_valid()
        self.publish_servo_state()

        if self.emergency_stop:
            self.set_servo_phase('EMERGENCY_HOLD')
            return

        if self.servo_mode_enabled:
            self.tick_servo_mode(now_sec)
            return

        if self.goal_active:
            return

        if not self.target_valid or not self.current_key or self.current_point is None:
            return

        if self.blocked_key and self.current_key == self.blocked_key:
            return

        if self.target_confidence < self.min_confidence:
            return

        if self.required_state and self.current_state != self.required_state:
            return

        if (now_sec - self.last_goal_sent_time) < _GOAL_COOLDOWN_SEC:
            return

        if not self.motion_enabled:
            return

        if not self.action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('ExecuteKey action server not available yet')
            return

        px, py = self.current_point

        if self.use_tf_targeting:
            arm_goal = self.to_arm_frame_goal(px, py)
            if arm_goal is None:
                return
            gx, gy, gz = arm_goal
            if self.require_transform_valid and not self.transform_valid:
                return
        else:
            gx, gy, gz = self.pixel_to_arm_goal(px, py)

        goal_msg = ExecuteKey.Goal()
        goal_msg.key_label = self.current_key
        goal_msg.x = float(gx)
        goal_msg.y = float(gy)
        goal_msg.z = float(gz)
        goal_msg.roll = float(self.target_roll)
        goal_msg.pitch = float(self.target_pitch)

        self.goal_active = True
        self.last_goal_sent_time = now_sec
        self.last_sent_key = self.current_key

        self.get_logger().info(
            f"Sending ExecuteKey for '{goal_msg.key_label}' at xyz=({goal_msg.x:.3f}, {goal_msg.y:.3f}, {goal_msg.z:.3f})"
        )

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_active = False
            self.last_goal_result = 'rejected'
            self.last_goal_result_message = 'ExecuteKey goal rejected'
            self.get_logger().warn('ExecuteKey goal rejected')
            return

        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        self.goal_active = False
        self.current_goal_handle = None
        result = future.result().result

        if self.emergency_stop:
            self.last_goal_result = 'ignored_emergency_stop'
            self.last_goal_result_message = result.message
            self.get_logger().warn('ExecuteKey result ignored because emergency stop is active.')
            return

        result_msg = (result.message or '').lower()
        is_dry_run = 'dry-run' in result_msg

        if result.success:
            if is_dry_run and not self.accept_dry_run_result:
                self.blocked_key = self.last_sent_key
                self.last_goal_result = 'dry_run_blocked'
                self.last_goal_result_message = result.message
                self.get_logger().warn(
                    f"ExecuteKey dry-run for '{self.last_sent_key}'. Not marking done. "
                    "Enable arm motion (publish_on_action:=true) or set accept_dry_run_result:=true."
                )
                return

            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            self.blocked_key = ''
            self.last_goal_result = 'success'
            self.last_goal_result_message = result.message
            self.get_logger().info(f"ExecuteKey succeeded for '{self.last_sent_key}': {result.message}")
        else:
            self.last_goal_result = 'failed'
            self.last_goal_result_message = result.message
            self.get_logger().warn(f"ExecuteKey failed for '{self.last_sent_key}': {result.message}")


def main(args=None):
    rclpy.init(args=args)
    node = TypingCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
