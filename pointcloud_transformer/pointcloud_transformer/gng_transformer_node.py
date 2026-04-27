import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # This is key for transforming geometry_msgs types

# Assuming the message definitions are from a package named 'ais_gng_msgs'
# You might need to adjust this based on your actual package structure
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from geometry_msgs.msg import PointStamped, Point32


class GngTransformerNode(Node):
    def __init__(self):
        super().__init__("gng_transformer_node")

        # Declare parameters
        self.declare_parameter("target_frame", "arm_base_link")
        self.declare_parameter("input_topic", "/gng_map")
        self.declare_parameter("output_topic", "/gng_map_transformed")

        # Get parameters
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        # TF2 listener and buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscription
        self.subscription = self.create_subscription(
            TopologicalMap, input_topic, self.map_callback, 10
        )

        # Publisher
        self.publisher = self.create_publisher(TopologicalMap, output_topic, 10)

        self.get_logger().info(
            f"GNG Transformer Node started. "
            f"Listening on '{input_topic}', "
            f"publishing on '{output_topic}', "
            f"transforming to '{self.target_frame}'."
        )

    def map_callback(self, msg: TopologicalMap):
        try:
            # 1. Look up the transform from the source frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            # 2. Create a new map to hold the transformed data
            transformed_map = TopologicalMap()
            transformed_map.header.stamp = msg.header.stamp
            transformed_map.header.frame_id = self.target_frame  # Update the frame_id!
            transformed_map.edges = msg.edges
            transformed_map.clusters = msg.clusters

            # 3. Transform each node's position
            for node in msg.nodes:
                # To use do_transform_point, we need a PointStamped
                point_stamped = PointStamped()
                point_stamped.header = msg.header
                # Point32 doesn't have 'z', so handle potential conversion issues if needed
                point_stamped.point.x = float(node.pos.x)
                point_stamped.point.y = float(node.pos.y)
                point_stamped.point.z = float(node.pos.z)

                # Perform the transformation
                transformed_point_stamped = tf2_geometry_msgs.do_transform_point(
                    point_stamped, transform
                )

                # Create a new node with the transformed position
                new_node = TopologicalNode()
                new_node.id = node.id
                new_node.pos = Point32(
                    x=transformed_point_stamped.point.x,
                    y=transformed_point_stamped.point.y,
                    z=transformed_point_stamped.point.z,
                )
                new_node.normal = node.normal
                new_node.rho = node.rho
                new_node.label = node.label
                new_node.age = node.age
                new_node.inpcl_ids = node.inpcl_ids

                transformed_map.nodes.append(new_node)

            # 4. Publish the transformed map
            self.publisher.publish(transformed_map)

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {msg.header.frame_id} to {self.target_frame}: {ex}",
                throttle_duration_sec=1.0,  # Avoid spamming logs
            )
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GngTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
