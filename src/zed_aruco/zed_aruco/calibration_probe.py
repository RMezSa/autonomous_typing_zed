import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class CalibrationProbe(Node):
    def __init__(self):
        super().__init__('calibration_probe')

        self.declare_parameter('arm_base_frame', 'arm_base')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('keyboard_plane_z_m', 0.45)
        self.declare_parameter('camera_fx', 700.0)
        self.declare_parameter('camera_fy', 700.0)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)

        self.arm_base_frame = str(self.get_parameter('arm_base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.keyboard_plane_z_m = float(self.get_parameter('keyboard_plane_z_m').value)
        self.camera_fx = float(self.get_parameter('camera_fx').value)
        self.camera_fy = float(self.get_parameter('camera_fy').value)
        self.camera_cx = float(self.get_parameter('camera_cx').value)
        self.camera_cy = float(self.get_parameter('camera_cy').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_target_px = None
        self.latest_target_msg = None
        self.latest_arm_point = None
        self.latest_reference = None

        self.target_arm_pub = self.create_publisher(PointStamped, 'keyboard/target_point_arm', 10)
        self.error_pub = self.create_publisher(Float32, 'keyboard/calibration_error_m', 10)

        self.create_subscription(PointStamped, 'keyboard/target_point_px', self.on_target_px, 10)
        self.create_subscription(PointStamped, 'keyboard/reference_point_arm', self.on_reference_point, 10)

        self.create_timer(0.1, self.tick)
        self.get_logger().info('calibration_probe started')

    def on_target_px(self, msg: PointStamped):
        self.latest_target_px = (float(msg.point.x), float(msg.point.y))
        self.latest_target_msg = msg

    def on_reference_point(self, msg: PointStamped):
        self.latest_reference = msg

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

    def tick(self):
        if self.latest_target_px is None or self.latest_target_msg is None:
            return

        px, py = self.latest_target_px
        source_frame = self.latest_target_msg.header.frame_id or self.camera_frame
        if not source_frame:
            return

        cam_point = self.pixel_to_camera_point(px, py, source_frame)

        try:
            transform = self.tf_buffer.lookup_transform(self.arm_base_frame, source_frame, Time())
            arm_point = do_transform_point(cam_point, transform)
        except TransformException as ex:
            self.get_logger().warn(f'TF transform failed ({source_frame} -> {self.arm_base_frame}): {ex}')
            return

        self.latest_arm_point = arm_point
        self.target_arm_pub.publish(arm_point)

        if self.latest_reference is None:
            return

        if self.latest_reference.header.frame_id and self.latest_reference.header.frame_id != self.arm_base_frame:
            return

        dx = arm_point.point.x - self.latest_reference.point.x
        dy = arm_point.point.y - self.latest_reference.point.y
        dz = arm_point.point.z - self.latest_reference.point.z
        err = (dx * dx + dy * dy + dz * dz) ** 0.5

        err_msg = Float32()
        err_msg.data = float(err)
        self.error_pub.publish(err_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationProbe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
