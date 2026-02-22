import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import json

from std_msgs.msg import String, Bool, Float32, Float64MultiArray
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from typing_interfaces.action import ExecuteKey


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
        self.declare_parameter('goal_cooldown_sec', 0.3)
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
        self.declare_parameter('workspace_x_min', -0.10)
        self.declare_parameter('workspace_x_max', 0.55)
        self.declare_parameter('workspace_y_min', -0.55)
        self.declare_parameter('workspace_y_max', 0.55)
        self.declare_parameter('workspace_z_min', 0.02)
        self.declare_parameter('workspace_z_max', 0.80)
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('require_transform_valid', True)
        self.declare_parameter('servo_mode_enabled', False)
        self.declare_parameter('contact_topic', 'keyboard/contact_pressed')
        self.declare_parameter('servo_state_topic', 'keyboard/servo_state')
        self.declare_parameter('emergency_stop_topic', 'keyboard/emergency_stop')
        self.declare_parameter('servo_xy_gain_x_m_per_px', 0.00035)
        self.declare_parameter('servo_xy_gain_y_m_per_px', 0.00035)
        self.declare_parameter('servo_xy_step_max_m', 0.003)
        self.declare_parameter('servo_align_enter_thresh_px', 8.0)
        self.declare_parameter('servo_align_exit_thresh_px', 12.0)
        self.declare_parameter('servo_align_stable_cycles', 4)
        self.declare_parameter('servo_cmd_cooldown_sec', 0.08)
        self.declare_parameter('servo_press_step_m', 0.0015)
        self.declare_parameter('servo_press_max_travel_m', 0.015)
        self.declare_parameter('servo_press_timeout_sec', 2.0)
        self.declare_parameter('servo_press_direction_sign', -1.0)
        self.declare_parameter('servo_press_xy_scale', 0.6)
        self.declare_parameter('servo_retract_step_m', 0.0025)
        self.declare_parameter('return_to_base_enabled', True)
        self.declare_parameter('return_to_base_on_failure', True)
        self.declare_parameter('return_to_base_command', 'KEYBOARD_HOME')
        self.declare_parameter('return_to_base_wait_sec', 1.0)
        self.declare_parameter('debug_status_topic', 'keyboard/coordinator_debug')
        self.declare_parameter('debug_publish_period_sec', 0.25)

        self.declare_parameter('image_center_x', 640.0)
        self.declare_parameter('image_center_y', 360.0)
        self.declare_parameter('base_x', 0.25)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('scale_x_per_px', 0.00035)
        self.declare_parameter('scale_y_per_px', 0.00035)

        action_name = self.get_parameter('action_name').value
        done_topic = self.get_parameter('done_topic').value

        self.target_z = float(self.get_parameter('target_z').value)
        self.target_roll = float(self.get_parameter('target_roll').value)
        self.target_pitch = float(self.get_parameter('target_pitch').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.required_state = str(self.get_parameter('required_state').value)
        self.goal_cooldown_sec = float(self.get_parameter('goal_cooldown_sec').value)
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

        self.workspace_x_min = float(self.get_parameter('workspace_x_min').value)
        self.workspace_x_max = float(self.get_parameter('workspace_x_max').value)
        self.workspace_y_min = float(self.get_parameter('workspace_y_min').value)
        self.workspace_y_max = float(self.get_parameter('workspace_y_max').value)
        self.workspace_z_min = float(self.get_parameter('workspace_z_min').value)
        self.workspace_z_max = float(self.get_parameter('workspace_z_max').value)
        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.require_transform_valid = bool(self.get_parameter('require_transform_valid').value)
        self.servo_mode_enabled = bool(self.get_parameter('servo_mode_enabled').value)
        self.contact_topic = str(self.get_parameter('contact_topic').value)
        self.servo_state_topic = str(self.get_parameter('servo_state_topic').value)
        self.emergency_stop_topic = str(self.get_parameter('emergency_stop_topic').value)
        self.servo_xy_gain_x = float(self.get_parameter('servo_xy_gain_x_m_per_px').value)
        self.servo_xy_gain_y = float(self.get_parameter('servo_xy_gain_y_m_per_px').value)
        self.servo_xy_step_max = float(self.get_parameter('servo_xy_step_max_m').value)
        self.servo_align_enter_thresh_px = float(self.get_parameter('servo_align_enter_thresh_px').value)
        self.servo_align_exit_thresh_px = float(self.get_parameter('servo_align_exit_thresh_px').value)
        self.servo_align_stable_cycles = int(self.get_parameter('servo_align_stable_cycles').value)
        self.servo_cmd_cooldown_sec = float(self.get_parameter('servo_cmd_cooldown_sec').value)
        self.servo_press_step_m = float(self.get_parameter('servo_press_step_m').value)
        self.servo_press_max_travel_m = float(self.get_parameter('servo_press_max_travel_m').value)
        self.servo_press_timeout_sec = float(self.get_parameter('servo_press_timeout_sec').value)
        self.servo_press_direction_sign = float(self.get_parameter('servo_press_direction_sign').value)
        self.servo_press_xy_scale = float(self.get_parameter('servo_press_xy_scale').value)
        self.servo_retract_step_m = float(self.get_parameter('servo_retract_step_m').value)
        self.return_to_base_enabled = bool(self.get_parameter('return_to_base_enabled').value)
        self.return_to_base_on_failure = bool(self.get_parameter('return_to_base_on_failure').value)
        self.return_to_base_command = str(self.get_parameter('return_to_base_command').value)
        self.return_to_base_wait_sec = float(self.get_parameter('return_to_base_wait_sec').value)
        self.debug_status_topic = str(self.get_parameter('debug_status_topic').value)
        self.debug_publish_period_sec = float(self.get_parameter('debug_publish_period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_center_x = float(self.get_parameter('image_center_x').value)
        self.image_center_y = float(self.get_parameter('image_center_y').value)
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.scale_x_per_px = float(self.get_parameter('scale_x_per_px').value)
        self.scale_y_per_px = float(self.get_parameter('scale_y_per_px').value)

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
        self.servo_press_start_z = 0.0
        self.servo_hover_z = 0.0
        self.servo_press_succeeded = False
        self.servo_return_started = False
        self.servo_return_start_time = 0.0
        self.current_goal_handle = None
        self.last_goal_result = 'none'
        self.last_goal_result_message = ''

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
        dx = self.image_center_x - px
        dy = py - self.image_center_y

        x = self.base_x + (dy * self.scale_x_per_px)
        y = self.base_y + (dx * self.scale_y_per_px)
        return x, y

    def pixel_to_camera_point(self, px: float, py: float, frame_id: str):
        z = self.keyboard_plane_z_m
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

    def is_within_workspace(self, gx: float, gy: float, gz: float):
        return (
            self.workspace_x_min <= gx <= self.workspace_x_max and
            self.workspace_y_min <= gy <= self.workspace_y_max and
            self.workspace_z_min <= gz <= self.workspace_z_max
        )

    def publish_transform_valid(self):
        msg = Bool()
        msg.data = bool(self.transform_valid)
        self.transform_valid_pub.publish(msg)

    def publish_servo_state(self):
        msg = String()
        msg.data = self.servo_phase
        self.servo_state_pub.publish(msg)

    def publish_debug_status(self):
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
            'last_goal_result': self.last_goal_result,
            'last_goal_result_message': self.last_goal_result_message,
        }

        msg = String()
        msg.data = json.dumps(debug, separators=(',', ':'))
        self.debug_status_pub.publish(msg)

    def set_servo_phase(self, phase: str):
        if phase != self.servo_phase:
            self.get_logger().info(f"Servo phase: {self.servo_phase} -> {phase}")
            self.servo_phase = phase

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
            gx, gy = self.pixel_to_arm_goal(px, py)
            gz = self.target_z

        if not self.is_within_workspace(gx, gy, gz):
            return False

        self.servo_cmd_x = float(gx)
        self.servo_cmd_y = float(gy)
        self.servo_cmd_z = float(gz)
        self.servo_cmd_initialized = True
        self.servo_cmd_key = self.current_key
        self.servo_aligned_cycles = 0
        self.servo_last_cmd_time = 0.0
        self.servo_hover_z = self.servo_cmd_z
        self.servo_press_succeeded = False
        return True

    def compute_xy_servo_delta(self, px: float, py: float, scale: float = 1.0):
        dx_px = self.image_center_x - px
        dy_px = py - self.image_center_y

        delta_x = self.clamp(
            dy_px * self.servo_xy_gain_x * scale,
            -self.servo_xy_step_max,
            self.servo_xy_step_max,
        )
        delta_y = self.clamp(
            dx_px * self.servo_xy_gain_y * scale,
            -self.servo_xy_step_max,
            self.servo_xy_step_max,
        )
        return delta_x, delta_y

    def start_press_phase(self, now_sec: float):
        self.servo_press_start_time = now_sec
        self.servo_press_start_z = self.servo_cmd_z
        self.servo_hover_z = self.servo_cmd_z
        self.servo_press_succeeded = False
        self.set_servo_phase('PRESSING_Z')

    def tick_press_phase(self, now_sec: float):
        if self.contact_pressed:
            self.servo_press_succeeded = True
            self.set_servo_phase('RETRACTING')
            return

        if not self.target_valid or self.current_point is None:
            self.get_logger().warn('Target lost during PRESSING_Z. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if self.target_confidence < self.min_confidence:
            self.get_logger().warn('Confidence dropped during PRESSING_Z. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if (now_sec - self.servo_press_start_time) > self.servo_press_timeout_sec:
            self.get_logger().warn('Press timeout reached before contact. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if abs(self.servo_cmd_z - self.servo_press_start_z) >= self.servo_press_max_travel_m:
            self.get_logger().warn('Press max travel reached before contact. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        if (now_sec - self.servo_last_cmd_time) < self.servo_cmd_cooldown_sec:
            return

        px, py = self.current_point
        delta_x, delta_y = self.compute_xy_servo_delta(px, py, self.servo_press_xy_scale)

        next_x = self.servo_cmd_x + delta_x
        next_y = self.servo_cmd_y + delta_y
        next_z = self.servo_cmd_z + (self.servo_press_direction_sign * self.servo_press_step_m)

        if not self.is_within_workspace(next_x, next_y, next_z):
            self.get_logger().warn('Press blocked by workspace bounds. Retracting.')
            self.servo_press_succeeded = False
            self.set_servo_phase('RETRACTING')
            return

        self.publish_cartesian_goal(next_x, next_y, next_z)
        self.servo_cmd_x = next_x
        self.servo_cmd_y = next_y
        self.servo_cmd_z = next_z
        self.servo_last_cmd_time = now_sec

    def tick_retract_phase(self, now_sec: float):
        if (now_sec - self.servo_last_cmd_time) < self.servo_cmd_cooldown_sec:
            return

        next_x = self.servo_cmd_x
        next_y = self.servo_cmd_y
        dz_to_hover = self.servo_hover_z - self.servo_cmd_z

        if abs(dz_to_hover) <= self.servo_retract_step_m:
            next_z = self.servo_hover_z
            at_hover = True
        else:
            next_z = self.servo_cmd_z + self.clamp(dz_to_hover, -self.servo_retract_step_m, self.servo_retract_step_m)
            at_hover = False

        if not self.is_within_workspace(next_x, next_y, next_z):
            self.get_logger().warn('Retract blocked by workspace bounds.')
            at_hover = True
            next_z = self.servo_cmd_z

        self.publish_cartesian_goal(next_x, next_y, next_z)
        self.servo_cmd_z = next_z
        self.servo_last_cmd_time = now_sec

        if not at_hover:
            return

        self.contact_pressed = False
        self.servo_cmd_initialized = False
        self.servo_cmd_key = ''
        self.servo_aligned_cycles = 0
        self.servo_return_started = False
        self.servo_return_start_time = 0.0

        should_return_base = self.return_to_base_enabled and (
            self.servo_press_succeeded or self.return_to_base_on_failure
        )

        if should_return_base:
            self.set_servo_phase('RETURNING_BASE')
            return

        if self.servo_press_succeeded:
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            self.set_servo_phase('COMPLETE')
            self.get_logger().info(f"Servo typing completed for '{self.current_key}'")
        else:
            self.set_servo_phase('ALIGNING')

    def tick_return_to_base_phase(self, now_sec: float):
        if not self.servo_return_started:
            self.command_return_to_base()
            self.servo_return_start_time = now_sec
            self.servo_return_started = True
            return

        if (now_sec - self.servo_return_start_time) < self.return_to_base_wait_sec:
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

        if self.servo_phase == 'PRESSING_Z':
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
        dy_px = py - self.image_center_y

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
            if self.servo_aligned_cycles >= self.servo_align_stable_cycles:
                self.set_servo_phase('ALIGNED_READY_PRESS')
                self.start_press_phase(now_sec)
            else:
                self.set_servo_phase('ALIGN_HOLD')
            return

        if outside_exit:
            self.servo_aligned_cycles = 0

        if (now_sec - self.servo_last_cmd_time) < self.servo_cmd_cooldown_sec:
            self.set_servo_phase('ALIGN_RATE_LIMIT')
            return

        delta_x, delta_y = self.compute_xy_servo_delta(px, py)

        next_x = self.servo_cmd_x + delta_x
        next_y = self.servo_cmd_y + delta_y
        next_z = self.servo_cmd_z

        if not self.is_within_workspace(next_x, next_y, next_z):
            self.set_servo_phase('ALIGN_BLOCKED_WORKSPACE')
            self.get_logger().warn(
                f"Servo align blocked by workspace: x={next_x:.3f} y={next_y:.3f} z={next_z:.3f}"
            )
            return

        self.publish_cartesian_goal(next_x, next_y, next_z)
        self.servo_cmd_x = next_x
        self.servo_cmd_y = next_y
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
            print("Not enough confidence")
            return

        if self.required_state and self.current_state != self.required_state:
            return

        if (now_sec - self.last_goal_sent_time) < self.goal_cooldown_sec:
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
            gx, gy = self.pixel_to_arm_goal(px, py)
            gz = self.target_z

        if not self.is_within_workspace(gx, gy, gz):
            self.get_logger().warn(
                f"Target out of workspace: x={gx:.3f} y={gy:.3f} z={gz:.3f}"
            )
            return

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
