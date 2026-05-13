#!/usr/bin/env python3
# Bridge /arm_feedback/jointN_deg (Float64, degrees) -> /joint_states (JointState, radians).
# The rover firmware publishes per-joint Float64 topics for joints 1..4; arm_node consumes
# a sensor_msgs/JointState. joint5 (gripper servo) has no feedback and is omitted; arm_node
# tolerates partial joint sets.
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


PUBLISH_RATE_HZ = 30.0
JOINT_STATE_TOPIC = '/joint_states'
FEEDBACK_TOPIC_FMT = '/arm_feedback/joint{idx}_deg'
FED_JOINTS = (1, 2, 3, 4)


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        self.latest_deg = {i: None for i in FED_JOINTS}

        for i in FED_JOINTS:
            topic = FEEDBACK_TOPIC_FMT.format(idx=i)
            self.create_subscription(
                Float64,
                topic,
                lambda msg, idx=i: self._on_joint(idx, msg),
                10,
            )

        self.pub = self.create_publisher(JointState, JOINT_STATE_TOPIC, 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info(
            f'joint_state_bridge: subscribing to '
            f'/arm_feedback/joint{{1..4}}_deg -> {JOINT_STATE_TOPIC} '
            f'at {PUBLISH_RATE_HZ:.0f} Hz'
        )

    def _on_joint(self, idx, msg):
        try:
            self.latest_deg[idx] = float(msg.data)
        except (TypeError, ValueError):
            pass

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in FED_JOINTS:
            v = self.latest_deg[i]
            if v is None:
                continue
            msg.name.append(f'joint{i}')
            msg.position.append(math.radians(v))
        if not msg.name:
            return
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
