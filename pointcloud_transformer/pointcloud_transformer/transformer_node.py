import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_sensor_msgs
from tf2_ros import TransformException
import time

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')

        self.declare_parameter('target_frame', 'arm_base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            PointCloud2,
            '/transformed_points',
            10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info(f'PointCloudTransformer node started. Target frame: {self.target_frame}')

    def listener_callback(self, msg):
        # Check if the point cloud is already in the target frame
        if msg.header.frame_id == self.target_frame:
            self.publisher.publish(msg)
            return

        try:
            # Lookup the transform from the point cloud's frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time() # Use the latest available transform
            )
            
            # Transform the point cloud
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, transform)
            
            # Publish the transformed point cloud
            self.publisher.publish(transformed_cloud)
            # self.get_logger().info(f'Transformed point cloud from {msg.header.frame_id} to {self.target_frame}')

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {msg.header.frame_id} to {self.target_frame}: {ex}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')


def main(args=None):
    rclpy.init(args=args)
    pointcloud_transformer = PointCloudTransformer()
    rclpy.spin(pointcloud_transformer)
    pointcloud_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
