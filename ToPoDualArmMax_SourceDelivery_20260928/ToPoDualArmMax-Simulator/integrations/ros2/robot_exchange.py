"""ブラウザ内ロボット専用の状態・TF配信と軌道受信。実機指令への接続なし。"""
import math
import threading
import time
from pathlib import Path
import xml.etree.ElementTree as element_tree


def finite_values(values, size):
    return isinstance(values, list) and len(values) == size and all(type(v) in (int, float) and math.isfinite(v) for v in values)


def validate_state(state):
    if not isinstance(state, dict) or state.get('robot_model') not in ('standard', 'long'):
        raise ValueError('モデルが不正です')
    pose = state.get('robot_pose')
    if not isinstance(pose, dict) or len(pose) > 64 or any(not isinstance(k, str) or type(v) not in (int, float) or not math.isfinite(v) for k, v in pose.items()):
        raise ValueError('関節状態が不正です')
    transforms = state.get('transforms')
    if not isinstance(transforms, list) or not 1 <= len(transforms) <= 256:
        raise ValueError('TFの数が不正です')
    parents = {}
    for transform in transforms:
        if not isinstance(transform, dict):
            raise ValueError('TFの形式が不正です')
        parent, child = transform.get('parent'), transform.get('child')
        if not isinstance(parent, str) or not isinstance(child, str) or not parent or not child or child in parents or child == 'world':
            raise ValueError('TFの親子関係が不正です')
        if not finite_values(transform.get('translation'), 3) or not finite_values(transform.get('rotation'), 4):
            raise ValueError('TFの数値が不正です')
        if abs(sum(v*v for v in transform['rotation']) - 1) > 1e-5:
            raise ValueError('TFの回転が不正です')
        parents[child] = parent
    if parents.get('base_footprint') != 'world':
        raise ValueError('worldからbase_footprintへのTFが必要です')
    for child in parents:
        seen = set()
        while child != 'world':
            if child in seen or child not in parents:
                raise ValueError('TFの循環または未接続があります')
            seen.add(child)
            child = parents[child]
    return state


def trajectory_payload(message):
    names = list(message.joint_names)
    if not names or len(names) > 64 or len(names) != len(set(names)) or not 1 <= len(message.points) <= 1000:
        raise ValueError('軌道の関節名・点数が不正です')
    if message.header.stamp.sec or message.header.stamp.nanosec:
        raise ValueError('軌道は受信後の相対再生のみ対応です。header stampを0にしてください')
    points, last_sec = [], 0.
    for point in message.points:
        sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        positions = list(point.positions)
        if not last_sec < sec <= 120 or not finite_values(positions, len(names)) or point.velocities or point.accelerations or point.effort:
            raise ValueError('位置のみ・正の増加時刻・120秒以内の軌道が必要です')
        points.append(dict(time_sec=sec, positions=positions))
        last_sec = sec
    return dict(joint_names=names, points=points)


class RobotExchange:
    def __init__(self, node, joints, tf_topic='/tf'):
        from geometry_msgs.msg import PoseStamped
        from tf2_msgs.msg import TFMessage
        from trajectory_msgs.msg import JointTrajectory
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.node, self.joints = node, joints
        self.tf = node.create_publisher(TFMessage, '/sim/tf', 10)
        self.standard_tf = node.create_publisher(TFMessage, tf_topic, 10) if tf_topic != '/sim/tf' else None
        self.base_pose = node.create_publisher(PoseStamped, '/sim/base_pose', 10)
        self.description = node.create_publisher(String, '/sim/robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.model = None
        self.commands = {}
        self.sequence = 0
        self.lock = threading.Lock()
        self.subscriptions = [node.create_subscription(JointTrajectory, f'/sim/command/{model}/joint_trajectory', lambda message, model=model: self.receive(model, message), 1) for model in ('standard', 'long')]
        app = Path(__file__).resolve().parents[2] / 'app'
        self.descriptions = {model: (app / path).read_text() for model, path in [('standard', 'models/standard/source.urdf'), ('long', 'source.urdf')]}
        self.parents = {}
        self.joint_names = {}
        for model, xml in self.descriptions.items():
            root = element_tree.fromstring(xml)
            self.parents[model] = {joint.find('child').get('link'): joint.find('parent').get('link') for joint in root.findall('joint')}
            self.joint_names[model] = {joint.get('name') for joint in root.findall('joint') if joint.get('type') != 'fixed' and joint.find('mimic') is None}

    def validate(self, state):
        validate_state(state)
        model = state['robot_model']
        expected = dict(self.parents[model], base_footprint='world')
        actual = {t['child']: t['parent'] for t in state['transforms']}
        if 'sim_camera_depth_optical_frame' in actual:
            expected['sim_camera_depth_optical_frame'] = 'base_footprint'
        if 'sim_mid360_frame' in actual:
            expected['sim_mid360_frame'] = 'base_footprint'
        if actual != expected or set(state['robot_pose']) != self.joint_names[model]:
            raise ValueError('URDFと状態の構成が一致しません')

    def publish(self, state, stamp=None):
        from geometry_msgs.msg import TransformStamped, PoseStamped
        from tf2_msgs.msg import TFMessage
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        self.validate(state)
        stamp = stamp or self.node.get_clock().now().to_msg()
        transforms = []
        for value in state['transforms']:
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, value['parent'], value['child']
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, value['translation'])
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, value['rotation'])
            transforms.append(msg)
            if value['child'] == 'base_footprint':
                base = PoseStamped()
                base.header = msg.header
                base.pose.position.x, base.pose.position.y, base.pose.position.z = map(float, value['translation'])
                base.pose.orientation = msg.transform.rotation
                self.base_pose.publish(base)
        self.tf.publish(TFMessage(transforms=transforms))
        if self.standard_tf is not None:
            self.standard_tf.publish(TFMessage(transforms=transforms))
        joints = JointState()
        joints.header.stamp, joints.header.frame_id = stamp, 'base_footprint'
        joints.name = list(state['robot_pose'])
        joints.position = list(map(float, state['robot_pose'].values()))
        self.joints.publish(joints)
        if self.model != state['robot_model']:
            self.model = state['robot_model']
            self.description.publish(String(data=self.descriptions[self.model]))
        return dict(model=self.model, stamp_sec=stamp.sec, stamp_nanosec=stamp.nanosec)

    def receive(self, model, message):
        try:
            command = trajectory_payload(message)
            if not set(command['joint_names']) <= self.joint_names[model]:
                raise ValueError('モデルの関節名と一致しません')
            with self.lock:
                self.sequence += 1
                self.commands[model] = (self.sequence, time.monotonic(), command)
        except ValueError as error:
            self.node.get_logger().warning(str(error))

    def latest(self, model):
        if model not in self.descriptions:
            raise ValueError('モデルが不正です')
        with self.lock:
            sequence, received, command = self.commands.get(model, (0, 0., None))
            return dict(sequence=sequence, command=command if time.monotonic() - received < 5 else None)
