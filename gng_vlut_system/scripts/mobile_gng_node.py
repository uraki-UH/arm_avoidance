#!/usr/bin/env python3
"""移動状態GNGの学習・読込と、位置・前方・速度ベクトルのROS表示。"""
from dataclasses import fields
import json
import math
from pathlib import Path
import signal
import time
import uuid

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode, TopologicalCluster
from ais_gng_feature_msgs.msg import TopologicalNodeFeature, TopologicalNodeFeatureArray
from geometry_msgs.msg import Point32
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
import xacro

from mobile_state_graph import drive_model, graph_config
from mobile_gng import training_config, train, validate_model, save_json


class mobile_gng_node(Node):
    def __init__(self):
        super().__init__('mobile_gng')
        path = self.declare_parameter('urdf_path', '').value
        if path.startswith('package://'):
            package, relative = path[10:].split('/', 1)
            path = str(Path(get_package_share_directory(package))/relative)
        xml = xacro.process_file(path).toxml() if path.endswith('.xacro') else Path(path).read_text()
        model = drive_model.from_urdf(xml)
        self.training = training_config(**{field.name: self.declare_parameter('mobile_gng.'+field.name,
            getattr(training_config(), field.name)).value for field in fields(training_config())})
        motion = graph_config(**{field.name: self.declare_parameter('motion_graph.'+field.name,
            getattr(graph_config(), field.name)).value for field in fields(graph_config())})
        self.is_build_only = self.declare_parameter('enable_build_only', False).value
        enable_rebuild = self.declare_parameter('enable_rebuild', False).value
        model_path = Path(self.declare_parameter('model_path', '').value)
        if not str(model_path) or str(model_path) == '.':
            raise ValueError('状態GNGのmodel_pathが必要です')
        if model_path.is_file() and not enable_rebuild:
            self.data = json.loads(model_path.read_text())
            validate_model(self.data, self.training, motion, model)
            self.get_logger().info('保存済み状態GNGの読込: '+str(model_path))
        else:
            artifacts = Path(self.declare_parameter('artifacts_path', '').value)/('training_'+uuid.uuid4().hex[:8])
            native = Path(get_package_prefix('gng_vlut_system'))/'lib/gng_vlut_system/mobile_gng_trainer'
            self.get_logger().info('移動状態GNGの学習開始: artifacts='+str(artifacts))
            self.data = train(self.training, motion, model, native, artifacts)
            save_json(model_path, self.data)
            save_json(artifacts/'state_gng.json', self.data)
        self.get_logger().info(f'状態GNG: nodes={len(self.data["states"])}, edges={len(self.data["edges"])}, '
            f'train_ms={self.data["train_ms"]:.2f}, total_ms={self.data["total_ms"]:.2f}, file={model_path} '
            '(近傍辺・走行可否未検査・衝突未検査)')
        if self.is_build_only:
            return
        self.frame = self.declare_parameter('frame_id', 'world').value
        self.origin_x = self.declare_parameter('origin_x', 0.0).value
        self.origin_y = self.declare_parameter('origin_y', 0.0).value
        self.origin_yaw = math.radians(self.declare_parameter('origin_yaw_deg', 0.0).value)
        if not all(math.isfinite(value) for value in (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('表示原点の非有限値')
        self.display_defaults = {'origin_x': self.origin_x, 'origin_y': self.origin_y,
            'origin_yaw_deg': math.degrees(self.origin_yaw), 'min_display_yaw_deg': -180.0,
            'max_display_yaw_deg': 180.0, 'min_display_speed': motion.min_speed,
            'max_display_speed': motion.max_speed, 'min_display_angular_speed': -motion.max_angular_speed,
            'max_display_angular_speed': motion.max_angular_speed}
        for name, default in self.display_defaults.items():
            if name.startswith(('min_', 'max_')):
                self.declare_parameter(name, default)
        result = self.on_display_parameters([])
        if not result.successful:
            raise ValueError(result.reason)
        self.add_on_set_parameters_callback(self.on_display_parameters)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(TopologicalMap, 'state_gng/map', latched)
        self.feature_pub = self.create_publisher(TopologicalNodeFeatureArray, 'state_gng/node_features', latched)
        self.data_pub = self.create_publisher(String, 'state_gng/data', latched)
        self.frame_num = 0
        self.publish_graph()
        self.create_timer(1.0, self.publish_graph)

    def on_display_parameters(self, parameters):
        settings = {name: self.get_parameter(name).value for name in self.display_defaults}
        for parameter in parameters:
            if parameter.name not in settings or type(parameter.value) not in (int, float) or not math.isfinite(parameter.value):
                return SetParametersResult(successful=False, reason='表示原点・状態範囲の有限数値のみ変更可能')
            settings[parameter.name] = parameter.value
        if (any(type(value) not in (int, float) or not math.isfinite(value) for value in settings.values()) or
                any(abs(settings[name]) > 180 for name in ('min_display_yaw_deg', 'max_display_yaw_deg')) or
                settings['min_display_speed'] > settings['max_display_speed'] or
                settings['min_display_angular_speed'] > settings['max_display_angular_speed']):
            return SetParametersResult(successful=False, reason='状態表示の範囲不正')
        return SetParametersResult(successful=True)

    def publish_graph(self):
        settings = {name: self.get_parameter(name).value for name in self.display_defaults}
        self.origin_x, self.origin_y = settings['origin_x'], settings['origin_y']
        self.origin_yaw = math.radians(settings['origin_yaw_deg'])
        selected = []
        for idx, state in enumerate(self.data['states']):
            yaw = math.degrees(state[2])
            lower, upper = settings['min_display_yaw_deg'], settings['max_display_yaw_deg']
            has_yaw = lower <= yaw <= upper if lower <= upper else yaw >= lower or yaw <= upper
            if has_yaw and settings['min_display_speed'] <= state[3] <= settings['max_display_speed'] and settings['min_display_angular_speed'] <= state[4] <= settings['max_display_angular_speed']:
                selected.append(idx)
        ids = {node_id: idx for idx, node_id in enumerate(selected)}
        self.frame_num = (self.frame_num+1) % 4294967296
        message = TopologicalMap(frame_number=self.frame_num)
        message.header.frame_id = self.frame
        message.header.stamp = self.get_clock().now().to_msg()
        features = TopologicalNodeFeatureArray(header=message.header)
        cos = math.cos(self.origin_yaw)
        sin = math.sin(self.origin_yaw)
        for idx in selected:
            x, y, yaw, speed, angular = self.data['states'][idx]
            angle = yaw+self.origin_yaw
            node = TopologicalNode(id=idx)
            node.pos = Point32(x=float(self.origin_x+cos*x-sin*y), y=float(self.origin_y+sin*x+cos*y), z=0.02)
            # normalは車体前方。環境表面の法線・安全ラベルとは異なる表示属性。
            node.normal = Point32(x=math.cos(angle), y=math.sin(angle), z=0.0)
            node.label = TopologicalMap.DEFAULT
            message.nodes.append(node)
            feature = TopologicalNodeFeature(node_id=idx)
            feature.ee_pose.position.x, feature.ee_pose.position.y, feature.ee_pose.position.z = node.pos.x, node.pos.y, node.pos.z
            feature.ee_pose.orientation.z, feature.ee_pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            features.features.append(feature)
            # 各GNG代表点の状態領域。速度矢印のための領域属性。
            cluster = TopologicalCluster(id=idx, nodes=[idx], label=TopologicalMap.DEFAULT)
            cluster.pos = node.pos
            cluster.scale = Point32(x=0.025, y=0.025, z=0.025)
            cluster.quat = feature.ee_pose.orientation
            cluster.velocity = Point32(x=speed*math.cos(angle), y=speed*math.sin(angle), z=0.0)
            if hasattr(cluster, 'has_velocity_observation'):
                cluster.has_velocity_observation = False
            message.clusters.append(cluster)
        message.edges = [ids[idx] for edge in self.data['edges'] if edge[0] in ids and edge[1] in ids for idx in edge]
        self.map_pub.publish(message)
        self.feature_pub.publish(features)
        self.data_pub.publish(String(data=json.dumps({**self.data, 'frame_id': self.frame,
            'normal_semantics': 'robot_forward', 'velocity_semantics': 'model_state_velocity', 'display_filters': settings, 'num_display_nodes': len(selected), 'display_origin': [self.origin_x, self.origin_y, self.origin_yaw]}, allow_nan=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exit_state = {'is_requested': False}
    handlers = {kind: signal.signal(kind, lambda *_: exit_state.update(is_requested=True)) for kind in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = mobile_gng_node()
        if not node.is_build_only:
            while rclpy.ok() and not exit_state['is_requested']:
                rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for kind, handler in handlers.items():
            signal.signal(kind, handler)


if __name__ == '__main__':
    main()
