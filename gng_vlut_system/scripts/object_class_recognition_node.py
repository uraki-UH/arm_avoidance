#!/usr/bin/env python3
"""テンプレート候補からのクラス・属性適合度配信。既存照合へのフィードバックなし。"""

import json
import time
from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml

from object_class_recognition import class_recognizer, finite_value


class object_class_recognition_node(Node):
    def __init__(self, **kwargs):
        super().__init__('object_class_recognition_node', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        default_file = str(Path(get_package_share_directory('gng_vlut_system')) /
                           'config' / 'object_class_recognition.yaml')
        config_file = self.declare_parameter('config_file', default_file, descriptor).value
        template_ids = self.declare_parameter(
            'template_ids', rclpy.Parameter.Type.STRING_ARRAY, descriptor).value or []
        candidate_topics = self.declare_parameter(
            'candidate_topics', rclpy.Parameter.Type.STRING_ARRAY, descriptor).value or []
        output_topic = self.declare_parameter('output_topic', '/object_recognition/classes', descriptor).value
        publish_hz = finite_value(self.declare_parameter('publish_hz', 5.0, descriptor).value,
                            'publish_hz', 0.1, 100.0)
        if len(candidate_topics) != len(template_ids) or len(set(candidate_topics)) != len(candidate_topics):
            raise ValueError('candidate_topicsとtemplate_idsの対応が不正です。')
        self.recognizer = class_recognizer(yaml.safe_load(Path(config_file).read_text()), template_ids)
        self.output_pub = self.create_publisher(
            String, output_topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.candidate_subs = [
            self.create_subscription(String, topic, partial(self.on_candidate, template_id), 10)
            for template_id, topic in zip(template_ids, candidate_topics)]
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_snapshot)
        self.publish_snapshot()

    def on_candidate(self, template_id, message):
        try:
            self.recognizer.update(template_id, json.loads(message.data), time.monotonic())
        except (ValueError, TypeError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self.publish_snapshot()

    def publish_snapshot(self):
        result = self.recognizer.snapshot(time.monotonic())
        self.output_pub.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))


def main():
    rclpy.init()
    node = None
    try:
        node = object_class_recognition_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
