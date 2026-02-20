import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import PointStamped

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

        self.image_center_x = float(self.get_parameter('image_center_x').value)
        self.image_center_y = float(self.get_parameter('image_center_y').value)
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.scale_x_per_px = float(self.get_parameter('scale_x_per_px').value)
        self.scale_y_per_px = float(self.get_parameter('scale_y_per_px').value)

        self.action_client = ActionClient(self, ExecuteKey, action_name)

        self.done_pub = self.create_publisher(Bool, done_topic, 10)

        self.create_subscription(String, 'keyboard/target_key', self.on_target_key, 10)
        self.create_subscription(PointStamped, 'keyboard/target_point_px', self.on_target_point, 10)
        self.create_subscription(Bool, 'keyboard/target_valid', self.on_target_valid, 10)
        self.create_subscription(Float32, 'keyboard/target_confidence', self.on_target_confidence, 10)
        self.create_subscription(String, 'keyboard/state', self.on_state, 10)

        self.current_key = ''
        self.current_point = None
        self.target_valid = False
        self.target_confidence = 0.0
        self.current_state = 'INIT'

        self.goal_active = False
        self.last_goal_sent_time = 0.0
        self.last_sent_key = ''
        self.blocked_key = ''

        self.create_timer(0.1, self.tick)

        self.get_logger().info('typing_coordinator started')

    def on_target_key(self, msg: String):
        previous_key = self.current_key
        self.current_key = msg.data.strip()
        if self.current_key != previous_key:
            self.blocked_key = ''

    def on_target_point(self, msg: PointStamped):
        self.current_point = (float(msg.point.x), float(msg.point.y))

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

    def tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

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

        if not self.action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('ExecuteKey action server not available yet')
            return

        px, py = self.current_point
        gx, gy = self.pixel_to_arm_goal(px, py)

        goal_msg = ExecuteKey.Goal()
        goal_msg.key_label = self.current_key
        goal_msg.x = float(gx)
        goal_msg.y = float(gy)
        goal_msg.z = float(self.target_z)
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
