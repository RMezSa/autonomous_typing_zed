import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from std_msgs.msg import String, Bool, Float32
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
        self.declare_parameter('min_confidence', 0.7)
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

        self.create_subscription(String, 'keyboard/target_key', self.on_target_key, 10)
        self.create_subscription(PointStamped, 'keyboard/target_point_px', self.on_target_point, 10)
        self.create_subscription(Bool, 'keyboard/target_valid', self.on_target_valid, 10)
        self.create_subscription(Float32, 'keyboard/target_confidence', self.on_target_confidence, 10)
        self.create_subscription(String, 'keyboard/state', self.on_state, 10)

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

        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"typing_coordinator started (motion_enabled={self.motion_enabled}, require_transform_valid={self.require_transform_valid})"
        )

    def on_target_key(self, msg: String):
        previous_key = self.current_key
        self.current_key = msg.data.strip()
        if self.current_key != previous_key:
            self.blocked_key = ''

    def on_target_point(self, msg: PointStamped):
        self.current_point = (float(msg.point.x), float(msg.point.y))
        self.current_point_msg = msg

    def on_target_valid(self, msg: Bool):
        self.target_valid = bool(msg.data)

    def on_target_confidence(self, msg: Float32):
        self.target_confidence = float(msg.data)

    def on_state(self, msg: String):
        self.current_state = msg.data

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

    def tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.publish_transform_valid()

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
            self.get_logger().warn('ExecuteKey goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        self.goal_active = False
        result = future.result().result

        result_msg = (result.message or '').lower()
        is_dry_run = 'dry-run' in result_msg

        if result.success:
            if is_dry_run and not self.accept_dry_run_result:
                self.blocked_key = self.last_sent_key
                self.get_logger().warn(
                    f"ExecuteKey dry-run for '{self.last_sent_key}'. Not marking done. "
                    "Enable arm motion (publish_on_action:=true) or set accept_dry_run_result:=true."
                )
                return

            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            self.blocked_key = ''
            self.get_logger().info(f"ExecuteKey succeeded for '{self.last_sent_key}': {result.message}")
        else:
            self.get_logger().warn(f"ExecuteKey failed for '{self.last_sent_key}': {result.message}")


def main(args=None):
    rclpy.init(args=args)
    node = TypingCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
