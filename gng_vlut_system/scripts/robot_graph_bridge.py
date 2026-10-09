#!/usr/bin/env python3
"""既存のロボットTmap・関節特徴を同一時刻で照合する入力アダプタ。"""
import json
import math
import uuid


class graph_packet_builder:
    def __init__(self, robot_id, joint_names):
        self.robot_id = robot_id
        self.joint_names = list(joint_names)
        self.graph_id = str(uuid.uuid4())
        self.revision = 0
        self.previous = None
        self.last_stamp = None

    def build(self, graph, features):
        def key(message):
            return (message.header.frame_id, message.header.stamp.sec, message.header.stamp.nanosec)
        if key(graph) != key(features):
            raise ValueError('グラフと関節特徴が同一更新時刻ではありません')
        angles = {feature.node_id: list(feature.weight_angle) for feature in features.features}
        ids = [node.id for node in graph.nodes]
        if (len(angles) != len(features.features) or len(set(ids)) != len(ids) or set(angles) != set(ids)
                or len(graph.edges) % 2):
            raise ValueError('グラフと関節特徴のノードID不一致')
        if any(len(pose) != len(self.joint_names) or not all(math.isfinite(value) for value in pose)
               for pose in angles.values()):
            raise ValueError('関節角の寸法・有限値不正')
        nodes = [dict(id=node.id, positions=angles[node.id], can_traverse=node.label == 1) for node in graph.nodes]
        can_traverse = {node['id']: node['can_traverse'] for node in nodes}
        edges = {}
        for first, second in zip(graph.edges[::2], graph.edges[1::2]):
            if first >= len(ids) or second >= len(ids) or first == second:
                raise ValueError('辺のノード添字不正')
            pair = tuple(sorted((ids[first], ids[second])))
            edges[pair] = dict(nodes=list(pair), can_traverse=all(can_traverse[value] for value in pair))
        body = dict(joint_names=self.joint_names, nodes=sorted(nodes, key=lambda node: node['id']),
                    edges=[edges[key] for key in sorted(edges)])
        stamp = (graph.header.stamp.sec, graph.header.stamp.nanosec)
        if self.last_stamp is not None and (stamp < self.last_stamp or
                (stamp == self.last_stamp and body != self.previous)):
            raise ValueError('時刻巻戻り・同一時刻の異なる更新。発行元の世代識別が必要です')
        self.last_stamp = stamp
        # 新規購読者・欠落復旧用の全体スナップショット。差分生成は独立した拡張点。
        if body != self.previous:
            self.revision += 1
            self.previous = body
        return dict(kind='snapshot', robot_id=self.robot_id, graph_id=self.graph_id, revision=self.revision,
                    stamp_sec=graph.header.stamp.sec + graph.header.stamp.nanosec * 1e-9, **body)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from ais_gng_msgs.msg import TopologicalMap
    from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray

    rclpy.init()
    node = Node('robot_graph_bridge')
    try:
        robot_id = node.declare_parameter('robot_id', 'topo_dual_arm_max_long').value
        joint_names = node.declare_parameter('joint_names', ['']).value
        if not joint_names or any(not value for value in joint_names) or len(set(joint_names)) != len(joint_names):
            raise ValueError('学習時の順序でjoint_namesを指定してください')
        graph_topic = node.declare_parameter('graph_topic', 'Tmap_static').value
        feature_topic = node.declare_parameter('feature_topic', 'topological_node_features').value
        output_topic = node.declare_parameter('output_topic', 'robot_Tmap_updates').value
        frame_id = node.declare_parameter('frame_id', '').value
        if not frame_id:
            raise ValueError('対象ロボットのframe_idを指定してください')
        builder = graph_packet_builder(robot_id, joint_names)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(String, output_topic, 10)
        pending = ({}, {})
        def receive(message, side):
            if message.header.frame_id != frame_id:
                node.get_logger().warning('対象ロボットのframe_id不一致')
                return
            key = (message.header.stamp.sec, message.header.stamp.nanosec)
            pending[side][key] = message
            while len(pending[side]) > 10:
                del pending[side][next(iter(pending[side]))]
            if key not in pending[1-side]:
                return
            graph, features = pending[0].pop(key), pending[1].pop(key)
            try:
                packet = builder.build(graph, features)
                publisher.publish(String(data=json.dumps(packet, allow_nan=False)))
            except ValueError as error:
                node.get_logger().error(str(error))
        node.create_subscription(TopologicalMap, graph_topic, lambda message: receive(message, 0), qos)
        node.create_subscription(TopologicalNodeFeatureArray, feature_topic, lambda message: receive(message, 1), qos)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
