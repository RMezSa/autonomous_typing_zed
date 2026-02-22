import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from typing_interfaces.action import ExecuteKey


class FakeExecuteKeyServer(Node):
    def __init__(self):
        super().__init__('fake_execute_key_server')

        self.declare_parameter('action_name', '/arm_ik/execute_key')
        self.declare_parameter('result_mode', 'success')
        self.declare_parameter('delay_sec', 0.2)
        self.declare_parameter('fail_every_n', 0)
        self.declare_parameter('alternate_dry_run', False)

        action_name = str(self.get_parameter('action_name').value)
        self.result_mode = str(self.get_parameter('result_mode').value).lower()
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.fail_every_n = int(self.get_parameter('fail_every_n').value)
        self.alternate_dry_run = bool(self.get_parameter('alternate_dry_run').value)

        self.goal_count = 0

        self.server = ActionServer(
            self,
            ExecuteKey,
            action_name,
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"fake_execute_key_server started on {action_name} "
            f"(result_mode={self.result_mode}, fail_every_n={self.fail_every_n})"
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Goal received for '{goal_request.key_label}' at "
            f"x={goal_request.x:.3f} y={goal_request.y:.3f} z={goal_request.z:.3f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def pick_result_mode(self):
        self.goal_count += 1

        if self.fail_every_n > 0 and self.goal_count % self.fail_every_n == 0:
            return 'fail'

        if self.alternate_dry_run and self.goal_count % 2 == 0:
            return 'dry_run'

        if self.result_mode in ('success', 'dry_run', 'fail'):
            return self.result_mode

        return 'success'

    def execute(self, goal_handle):
        mode = self.pick_result_mode()

        feedback = ExecuteKey.Feedback()
        feedback.stage = 'simulating'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        time.sleep(max(0.0, self.delay_sec))

        if goal_handle.is_cancel_requested:
            result = ExecuteKey.Result()
            result.success = False
            result.message = 'Cancelled'
            goal_handle.canceled()
            return

        feedback = ExecuteKey.Feedback()
        feedback.stage = 'done'
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        result = ExecuteKey.Result()
        if mode == 'fail':
            result.success = False
            result.message = 'Simulated failure'
            goal_handle.abort()
        elif mode == 'dry_run':
            result.success = True
            result.message = 'IK solved (dry-run, no publish)'
            goal_handle.succeed()
        else:
            result.success = True
            result.message = 'Simulated execution published'
            goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeExecuteKeyServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
