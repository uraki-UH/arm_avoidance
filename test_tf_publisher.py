import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class TestTFPublisher(Node):
    def __init__(self):
        super().__init__('test_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_timer_callback)
        self.start_time = self.get_clock().now()

    def broadcast_timer_callback(self):
        t = TransformStamped()
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'topoarm_base_link'

        # 円軌道で動かす
        radius = 0.5
        t.transform.translation.x = radius * math.cos(elapsed)
        t.transform.translation.y = radius * math.sin(elapsed)
        t.transform.translation.z = 0.5

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = TestTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
