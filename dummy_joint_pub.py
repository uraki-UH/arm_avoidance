import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class DummyJointPublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_publisher', namespace='topoarm_dual')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 双腕(topoarm_dual)の関節名
        msg.name = [
            'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4', 
            'left_joint5', 'left_joint6', 'left_joint7',
            'left_gripper_left_joint', 'left_gripper_right_joint',
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 
            'right_joint5', 'right_joint6', 'right_joint7',
            'right_gripper_left_joint', 'right_gripper_right_joint'
        ]
        
        # ゆっくりとしたサイン波で両腕を動かす
        val = math.sin(self.t) * 0.5
        val_r = math.cos(self.t) * 0.5
        msg.position = [
            val, val * 0.8, val * 0.6, val * 0.4, val * 0.2, 0.0, 0.0, 0.0, 0.0, # Left
            val_r, val_r * 0.8, val_r * 0.6, val_r * 0.4, val_r * 0.2, 0.0, 0.0, 0.0, 0.0 # Right
        ]
        
        self.publisher_.publish(msg)
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = DummyJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
