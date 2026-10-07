#!/usr/bin/env python3
"""移動状態グラフの事前生成・保存・原点近傍表示。走行指令なし。"""
from dataclasses import fields
import json
from pathlib import Path
import math
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
import xacro

from mobile_state_graph import drive_model, graph_config, load_or_build, rollout


class mobile_state_graph_node(Node):
    def __init__(self):
        super().__init__('mobile_state_graph')
        begin = time.perf_counter()
        self.is_build_only = self.declare_parameter('build_only', False).value
        enable_rebuild = self.declare_parameter('enable_rebuild', False).value
        path = self.declare_parameter('urdf_path', '').value
        if path.startswith('package://'):
            package, relative = path[10:].split('/', 1)
            path = str(Path(get_package_share_directory(package))/relative)
        xml = xacro.process_file(path).toxml() if path.endswith('.xacro') else Path(path).read_text()
        self.model = drive_model.from_urdf(xml)
        defaults = graph_config()
        self.config = graph_config(**{field.name: self.declare_parameter(
            'motion_graph.'+field.name, getattr(defaults, field.name)).value for field in fields(defaults)})
        output_path = self.declare_parameter('output_path', '').value
        if not output_path:
            raise ValueError('グラフの保存先output_pathが必要です')
        self.graph, has_generated = load_or_build(output_path, self.config, self.model, enable_rebuild)
        self.get_logger().info(
            f'移動状態グラフ: nodes={len(self.graph["states"])}, edges={len(self.graph["edges"])}, '
            f'generated={has_generated}, node_limit={self.graph["has_node_limit"]}, '
            f'load_build_ms={1000*(time.perf_counter()-begin):.2f}, file={output_path} '
            '(衝突未検査・走行指令なし)')
        if self.is_build_only:
            return
        self.frame = self.declare_parameter('frame_id', 'world').value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'state_graph/markers', qos)
        self.data_pub = self.create_publisher(String, 'state_graph/data', qos)
        self.data_pub.publish(String(data=json.dumps({**self.graph, 'frame_id': self.frame}, allow_nan=False)))
        self.marker_pub.publish(self.make_markers())

    def make_markers(self):
        markers = []
        for idx, kind in enumerate((Marker.SPHERE_LIST, Marker.LINE_LIST, Marker.LINE_LIST)):
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = 'motion_states', idx, kind
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.025 if idx == 0 else 0.004
            marker.color = ColorRGBA(r=0.3, g=0.7, b=1.0, a=0.5)
            markers.append(marker)
        def point(state):
            return Point(x=float(state[0]), y=float(state[1]), z=0.02)
        for state in self.graph['states']:
            ratio = (state[3]-self.config.min_speed)/(self.config.max_speed-self.config.min_speed)
            color = ColorRGBA(r=float(1-ratio), g=float(ratio), b=0.3, a=0.9)
            markers[0].points.append(point(state))
            markers[0].colors.append(color)
            markers[1].points.extend((point(state), Point(
                x=state[0]+0.05*math.cos(state[2]), y=state[1]+0.05*math.sin(state[2]), z=0.02)))
            markers[1].colors.extend((color, color))
        for source, _, acceleration, angular_acceleration, _ in self.graph['edges']:
            samples = rollout(self.graph['states'][source], acceleration, angular_acceleration, self.config, self.model)
            for start, end in zip(samples, samples[1:]):
                markers[2].points.extend((point(start), point(end)))
        return MarkerArray(markers=markers)


def main():
    rclpy.init()
    node = None
    try:
        node = mobile_state_graph_node()
        if not node.is_build_only:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
