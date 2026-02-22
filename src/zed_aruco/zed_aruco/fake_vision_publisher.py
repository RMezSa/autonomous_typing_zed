import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32, String


class FakeVisionPublisher(Node):
    def __init__(self):
        super().__init__('fake_vision_publisher')

        self.declare_parameter('text', 'test')
        self.declare_parameter('loop_text', True)
        self.declare_parameter('frame_id', 'zed2i_left_camera_optical_frame')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('base_px', 640.0)
        self.declare_parameter('base_py', 360.0)
        self.declare_parameter('jitter_px', 2.0)
        self.declare_parameter('confidence', 0.95)
        self.declare_parameter('state_when_valid', 'TRACKING')
        self.declare_parameter('state_when_invalid', 'SEARCHING')
        self.declare_parameter('dropout_every_n_keys', 0)
        self.declare_parameter('dropout_duration_sec', 0.0)

        raw_text = str(self.get_parameter('text').value)
        self.loop_text = bool(self.get_parameter('loop_text').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.base_px = float(self.get_parameter('base_px').value)
        self.base_py = float(self.get_parameter('base_py').value)
        self.jitter_px = float(self.get_parameter('jitter_px').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.state_when_valid = str(self.get_parameter('state_when_valid').value)
        self.state_when_invalid = str(self.get_parameter('state_when_invalid').value)
        self.dropout_every_n_keys = int(self.get_parameter('dropout_every_n_keys').value)
        self.dropout_duration_sec = float(self.get_parameter('dropout_duration_sec').value)

        self.sequence = []
        for char in raw_text:
            if char == ' ':
                self.sequence.append('space')
            elif char.strip():
                self.sequence.append(char.lower())

        self.sequence_index = 0
        self.completed_count = 0
        self.dropout_until = 0.0

        self.target_key_pub = self.create_publisher(String, 'keyboard/target_key', 10)
        self.target_point_pub = self.create_publisher(PointStamped, 'keyboard/target_point_px', 10)
        self.target_valid_pub = self.create_publisher(Bool, 'keyboard/target_valid', 10)
        self.target_conf_pub = self.create_publisher(Float32, 'keyboard/target_confidence', 10)
        self.state_pub = self.create_publisher(String, 'keyboard/state', 10)

        self.create_subscription(Bool, 'keyboard/mark_done', self.on_mark_done, 10)

        timer_period = 1.0 / max(1.0, self.publish_rate_hz)
        self.create_timer(timer_period, self.tick)

        self.get_logger().info(
            f"fake_vision_publisher started with {len(self.sequence)} targets (loop_text={self.loop_text})"
        )

    def get_current_key(self):
        if not self.sequence:
            return ''
        if self.sequence_index >= len(self.sequence):
            return ''
        return self.sequence[self.sequence_index]

    def on_mark_done(self, msg: Bool):
        if not msg.data:
            return

        current_key = self.get_current_key()
        if not current_key:
            return

        self.completed_count += 1
        self.sequence_index += 1

        if self.sequence_index >= len(self.sequence):
            if self.loop_text and self.sequence:
                self.sequence_index = 0

        if self.dropout_every_n_keys > 0 and self.completed_count % self.dropout_every_n_keys == 0:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self.dropout_until = now_sec + max(0.0, self.dropout_duration_sec)

    def tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        in_dropout = now_sec < self.dropout_until

        current_key = self.get_current_key()
        valid = bool(current_key) and not in_dropout

        state_msg = String()
        state_msg.data = self.state_when_valid if valid else self.state_when_invalid
        self.state_pub.publish(state_msg)

        key_msg = String()
        key_msg.data = current_key
        self.target_key_pub.publish(key_msg)

        valid_msg = Bool()
        valid_msg.data = bool(valid)
        self.target_valid_pub.publish(valid_msg)

        conf_msg = Float32()
        conf_msg.data = float(self.confidence if valid else 0.0)
        self.target_conf_pub.publish(conf_msg)

        if not valid:
            return

        px = self.base_px + self.jitter_px * math.sin(now_sec * 2.0)
        py = self.base_py + self.jitter_px * math.cos(now_sec * 2.0)

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.frame_id
        point_msg.point.x = float(px)
        point_msg.point.y = float(py)
        point_msg.point.z = 0.0
        self.target_point_pub.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
