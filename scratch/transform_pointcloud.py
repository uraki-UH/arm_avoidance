import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')
        
        # Declare parameters for calibration (Default values from user)
        self.declare_parameter('x', 0.434, ParameterDescriptor(description='Translation X'))
        self.declare_parameter('y', -0.693, ParameterDescriptor(description='Translation Y'))
        self.declare_parameter('z', 0.279, ParameterDescriptor(description='Translation Z'))
        self.declare_parameter('roll', -103.8, ParameterDescriptor(description='Rotation Roll (deg)'))
        self.declare_parameter('pitch', -28.9, ParameterDescriptor(description='Rotation Pitch (deg)'))
        self.declare_parameter('yaw', -3.4, ParameterDescriptor(description='Rotation Yaw (deg)'))
        
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('input_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/transformed_points')

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.listener_callback,
            qos)
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        
        self.get_logger().info(f'Transforming {input_topic} to {self.target_frame} on {output_topic}')
        self.get_logger().info('Calibration parameters (x, y, z, roll, pitch, yaw) are available via ROS 2 parameters.')

    def listener_callback(self, msg):
        # Get latest parameters
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value
        roll = self.get_parameter('roll').get_parameter_value().double_value
        pitch = self.get_parameter('pitch').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value

        # Build transform matrix
        # User's Eigen logic: R = Rz * Ry * Rx
        # In Scipy, this corresponds to intrinsic 'zyx' with [yaw, pitch, roll] 
        # or extrinsic 'XYZ' with [roll, pitch, yaw].
        # We use 'zyx' and pass [yaw, pitch, roll] to match the Z-Y-X multiplication order.
        rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
        
        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.target_frame
        t.child_frame_id = msg.header.frame_id
        
        t.transform.translation = Vector3(x=x, y=y, z=z)
        q = rot.as_quat()
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        try:
            # Transform the point cloud
            transformed_msg = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(msg, t)
            
            # Update header
            transformed_msg.header.stamp = msg.header.stamp
            transformed_msg.header.frame_id = self.target_frame
            
            self.publisher.publish(transformed_msg)
        except Exception as e:
            self.get_logger().error(f'Transform failed: {str(e)}', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
