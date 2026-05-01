import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class TestTFPublisher(Node):
    def __init__(self):
        super().__init__("test_tf_publisher")

        self.frame_id = self.declare_parameter("frame_id", "demo_scene").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.publish_hz = float(self.declare_parameter("publish_hz", 20.0).value)
        self.orbit_radius = float(self.declare_parameter("orbit_radius", 0.8).value)
        self.height = float(self.declare_parameter("height", 0.6).value)

        self.br = TransformBroadcaster(self)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_hz, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        scene_x = self.orbit_radius * math.cos(elapsed)
        scene_y = self.orbit_radius * math.sin(elapsed)
        scene_z = self.height + 0.1 * math.sin(elapsed * 1.7)
        yaw = elapsed * 0.6
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.frame_id
        tf_msg.transform.translation.x = scene_x
        tf_msg.transform.translation.y = scene_y
        tf_msg.transform.translation.z = scene_z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.br.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = TestTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
