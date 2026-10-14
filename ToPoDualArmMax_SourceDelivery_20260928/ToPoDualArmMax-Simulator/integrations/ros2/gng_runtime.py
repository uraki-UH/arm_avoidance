"""記録ROIとMuJoCo実測を接続する、シミュレータ専用GNG/VLUT実行環境。"""
import copy
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


runtime_lock = threading.Lock()
active_runtimes = set()


def transform_matrix(transform):
    """ROS TFの剛体変換行列。"""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([transform.rotation.x, transform.rotation.y,
                                        transform.rotation.z, transform.rotation.w]).as_matrix()
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


def source_world_tf(buffer, source_frame, world_source):
    """記録基台の明示重畳。既存経路の保持と、未接続ルートだけのworld接続。"""
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_ros import TransformException
    try:
        existing = buffer.lookup_transform('world', source_frame, Time())
    except TransformException:
        existing = None
    if existing is not None:
        if not np.allclose(transform_matrix(existing.transform), world_source, atol=1e-6, rtol=0):
            raise ValueError('記録基台のworld配置とシミュレーション基台が異なります。ROIの重畳はできません')
        return None
    frames = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
    root, seen = source_frame, set()
    while root in frames:
        if root in seen:
            raise ValueError('記録基台TFの循環')
        seen.add(root)
        root = frames[root]['parent']
    if root == 'world':
        # 接続経路のTF未到着。別の親への付替え防止。
        raise LookupError('記録基台TFの入力待機')
    root_source = np.eye(4) if root == source_frame else transform_matrix(
        buffer.lookup_transform(root, source_frame, Time()).transform)
    world_root = world_source @ np.linalg.inv(root_source)
    result = TransformStamped()
    result.header.frame_id, result.child_frame_id = 'world', root
    result.transform.translation.x, result.transform.translation.y, result.transform.translation.z = map(float, world_root[:3, 3])
    quat = Rotation.from_matrix(world_root[:3, :3]).as_quat()
    result.transform.rotation.x, result.transform.rotation.y, result.transform.rotation.z, result.transform.rotation.w = map(float, quat)
    return result


def same_grid(left, right):
    """記録自己マスクと環境ROIの座標・索引契約。"""
    return (left.header.frame_id.lstrip('/') == right.header.frame_id.lstrip('/')
            and abs(left.voxel_size-right.voxel_size) < 1e-9
            and all(getattr(left, name) == getattr(right, name) for name in (
                'x_shift', 'y_shift', 'z_shift', 'offset', 'origin_x', 'origin_y', 'origin_z')))


def recorded_environment(roi, mask):
    """記録ロボットの残像除去。現在のシミュレーション自己除去とは独立。"""
    if not same_grid(roi, mask):
        raise ValueError('記録ROIと記録自己マスクの座標・ボクセル規格が異なります')
    if roi.labels and len(roi.labels) != len(roi.data):
        raise ValueError('記録ROIのラベル数が不正です')
    excluded = set(mask.data)
    kept = [idx for idx, value in enumerate(roi.data) if value not in excluded]
    result = copy.deepcopy(roi)
    result.data = [roi.data[idx] for idx in kept]
    result.labels = [roi.labels[idx] for idx in kept] if roi.labels else []
    return result


def simulation_parameters(params, namespace):
    """機体設定の再利用と、記録姿勢・実機入力からの名前空間分離。"""
    result = copy.deepcopy(params)
    result.update(robot_name=namespace, enable_joint_state_publisher=False,
                  enable_dynamixel_joint_state_input=False, enable_dynamixel_current_pose=False,
                  enable_realsense_mount_tf=False, joint_state_topic='/'+namespace+'/joint_states')
    result['mid360'] = dict(result.get('mid360', {}), enable_input=False, enable_mount_tf=False)
    environment = result['environment_voxelization']
    environment.update(input_kind='voxel', voxel_topic='/'+namespace+'/self_filter_roi_voxels',
                       enable_static_tf=False)
    result['self_recognition'].update(enable_self_recognition_viz=True, enable_environment_self_filter=True,
        raw_environment_voxel_topic='roi_voxels', filtered_environment_voxel_topic='self_filter_roi_voxels',
        mask_topic='self_voxel')
    # ROS YAMLの型未確定な空配列を回避。各ノードのstring配列の空既定値の使用。
    if result.get('collision', {}).get('environment_ignore_links') == []:
        del result['collision']['environment_ignore_links']
    return result


class gng_runtime:
    """一物理セッションによるライブ姿勢・入力再時刻化・所有launchの管理。"""
    def __init__(self, node, params, settings, scene, root_link):
        from sensor_msgs.msg import JointState, PointCloud2
        from voxel_msgs.msg import Voxel
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        from gng_control_msgs.srv import CheckArmPair
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        self.node, self.scene = node, scene
        self.namespace = 'sim_'+params['robot_name']
        self.params = simulation_parameters(params, self.namespace)
        self.frame = self.namespace+'/'+root_link
        source_frame = params['robot_name']+'/'+params['environment_voxelization'].get('base_frame', root_link)
        self.source_frame = settings.get('source_frame', source_frame)
        self.roi_topic = settings.get('roi_topic', '/'+params['robot_name']+'/roi_voxels')
        self.mask_topic = settings.get('recorded_self_topic', '/'+params['robot_name']+'/self_voxel')
        self.points_topic = settings.get('points_topic', params.get('mid360', {}).get('points_topic', '/livox/lidar'))
        from rclpy.validate_full_topic_name import validate_full_topic_name
        for topic in (self.roi_topic, self.mask_topic, self.points_topic):
            validate_full_topic_name(topic)
            if topic.startswith('/'+self.namespace+'/'):
                raise ValueError('記録入力にシミュレータ出力を指定できません')
        self.enable_recorded_self_filter = settings.get('enable_recorded_self_filter', True)
        if not isinstance(self.enable_recorded_self_filter, bool):
            raise ValueError('enable_recorded_self_filterは真偽値が必要です')
        self.lock = threading.Lock()
        self.subscriptions = []
        self.publishers = []
        self.pair_client = None
        self.checked_pair = None
        self.process = None
        self.directory = None
        self.log = None
        self.is_closed = False
        self.pending_roi = self.recorded_mask = None
        self.recorded_transforms = {}
        self.roi_received_sec = self.mask_received_sec = self.points_received_sec = None
        self.last_feedback_sec = 0.
        self.next_tf_check_sec = time.monotonic() + .5
        self.has_source_world_tf = False
        self.num_input_voxels = self.num_environment_voxels = self.num_points = 0
        with runtime_lock:
            if self.namespace in active_runtimes or node.count_publishers('/'+self.namespace+'/joint_states'):
                raise ValueError('同じ機体のGNG回避セッションが既に存在します')
            active_runtimes.add(self.namespace)
        try:
            self.pair_client = node.create_client(CheckArmPair, '/'+self.namespace+'/check_arm_pair')
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.voxels = node.create_publisher(Voxel, '/'+self.namespace+'/roi_voxels', qos)
            self.publishers.append(self.voxels)
            self.joints = node.create_publisher(JointState, '/'+self.namespace+'/joint_states', qos)
            self.publishers.append(self.joints)
            self.static_tf = node.create_publisher(TFMessage, '/tf_static', qos)
            self.publishers.append(self.static_tf)
            for kind, topic, callback in ((Voxel, self.roi_topic, self.on_roi),
                                          (PointCloud2, self.points_topic, self.on_points)):
                self.subscriptions.append(node.create_subscription(kind, topic, callback, qos_profile_sensor_data))
            if self.enable_recorded_self_filter:
                self.subscriptions.append(node.create_subscription(Voxel, self.mask_topic, self.on_mask, qos_profile_sensor_data))
            tf = TransformStamped()
            tf.header.stamp = node.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = 'world', self.frame
            base = scene.mujoco.mj_name2id(scene.model, scene.mujoco.mjtObj.mjOBJ_BODY, 'robot_base')
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, scene.data.xpos[base])
            q = scene.data.xquat[base]
            tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = map(float, q)
            self.base_transforms = [tf]
            self.static_tf.publish(TFMessage(transforms=self.base_transforms))
            recorded_tf_topic = settings.get('recorded_tf_topic')
            if recorded_tf_topic:
                # bagループの時刻巻戻しを隔離。座標値・親子関係を保持したROS現在時刻への対応。
                self.dynamic_tf = node.create_publisher(TFMessage, '/tf', 10)
                self.publishers.append(self.dynamic_tf)
                static_topic = settings['recorded_static_tf_topic']
                for topic in (recorded_tf_topic, static_topic):
                    validate_full_topic_name(topic)
                    if topic in ('/tf', '/tf_static'):
                        raise ValueError('記録TF入力と配信先の循環は禁止です')
                self.subscriptions.append(node.create_subscription(TFMessage, recorded_tf_topic,
                    lambda msg: self.on_recorded_tf(msg, False), qos_profile_sensor_data))
                self.subscriptions.append(node.create_subscription(TFMessage, static_topic,
                    lambda msg: self.on_recorded_tf(msg, True),
                    QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            source_link = params['environment_voxelization'].get('base_frame', root_link)
            body = scene.mujoco.mj_name2id(scene.model, scene.mujoco.mjtObj.mjOBJ_BODY, 'robot_'+source_link)
            if body < 0:
                raise ValueError('記録基台に対応するURDFリンクがありません: '+source_link)
            self.world_source = np.eye(4)
            self.world_source[:3, 3] = scene.data.xpos[body]
            self.world_source[:3, :3] = Rotation.from_quat(scene.data.xquat[body][[1, 2, 3, 0]]).as_matrix()
            self.directory = tempfile.TemporaryDirectory(prefix='topo-gng-simulation-')
            params_file = Path(self.directory.name) / 'params.yaml'
            params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': self.params}}, allow_unicode=True))
            self.log = open(Path(self.directory.name) / 'runtime.log', 'w')
            self.command = ['ros2', 'launch', 'gng_vlut_system', 'gng_viewer_bridge.launch.py',
                'params_file:='+str(params_file), 'joint_control_backend:=external', 'edge_mode:=0',
                'state_topic:=/'+self.namespace+'/joint_states', 'enable_joint_state_publisher:=false',
                'enable_environment_voxelization:=true', 'enable_dynamixel_joint_state_input:=false',
                'enable_dynamixel_current_pose:=false', 'enable_realsense_mount_tf:=false']
            self.process = subprocess.Popen(self.command, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        except Exception:
            self.close()
            raise

    def on_points(self, message):
        with self.lock:
            if not self.is_closed:
                self.points_received_sec = time.monotonic()
                self.num_points = message.width*message.height

    def on_mask(self, message):
        with self.lock:
            if not self.is_closed:
                self.recorded_mask = message
                self.mask_received_sec = time.monotonic()

    def on_roi(self, message):
        with self.lock:
            if not self.is_closed:
                self.pending_roi = message
                self.roi_received_sec = time.monotonic()

    def publish_static_tf(self):
        """記録・両基台の全静的TFを同一スナップショットで保持。呼出し側のlock所有。"""
        from tf2_msgs.msg import TFMessage
        for own in self.base_transforms:
            recorded = self.recorded_transforms.get(own.child_frame_id)
            if recorded is not None and (recorded.header.frame_id != own.header.frame_id or
                    not np.allclose(transform_matrix(recorded.transform), transform_matrix(own.transform), atol=1e-6, rtol=0)):
                raise ValueError('記録TFと基台接続の競合: '+own.child_frame_id)
        transforms = dict(self.recorded_transforms)
        transforms.update((tf.child_frame_id, tf) for tf in self.base_transforms)
        self.static_tf.publish(TFMessage(transforms=list(transforms.values())))

    def on_recorded_tf(self, message, enable_static):
        with self.lock:
            if self.is_closed:
                return
            result = copy.deepcopy(message)
            stamp = self.node.get_clock().now().to_msg()
            for tf in result.transforms:
                if tf.child_frame_id.startswith(self.namespace+'/'):
                    raise ValueError('記録TFによるMuJoCo姿勢の上書きは禁止です')
                tf.header.stamp = stamp
            if enable_static:
                self.recorded_transforms.update((tf.child_frame_id, tf) for tf in result.transforms)
                self.publish_static_tf()
            else:
                self.dynamic_tf.publish(result)

    def ensure_world_tf(self, buffer):
        """既存TFバッファの再利用。固定配置照合の低頻度実行。"""
        from tf2_ros import TransformException
        now = time.monotonic()
        if now < self.next_tf_check_sec:
            return self.has_source_world_tf
        self.next_tf_check_sec = now + .5
        try:
            tf = source_world_tf(buffer, self.source_frame, self.world_source)
        except (TransformException, LookupError):
            self.has_source_world_tf = False
            return False
        if tf is not None:
            tf.header.stamp = self.node.get_clock().now().to_msg()
            # TRANSIENT_LOCAL depth=1で両基台のTFを保持。遅延参加Viewerへの配信。
            with self.lock:
                self.base_transforms = [self.base_transforms[0], tf]
                self.publish_static_tf()
        self.has_source_world_tf = True
        return True

    def update(self, positions, names, buffer):
        from sensor_msgs.msg import JointState
        now = time.monotonic()
        if self.process.poll() is not None:
            self.log.flush()
            detail = (Path(self.directory.name) / 'runtime.log').read_text(errors='replace')[-2500:]
            raise RuntimeError('GNG/VLUT処理が終了しました: '+detail)
        if now-self.last_feedback_sec >= .02:
            message = JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = self.frame
            message.name, message.position = list(names), list(map(float, positions))
            motion = self.scene.robot.motion_state()
            message.velocity = [motion['velocity'][name] for name in names]
            message.effort = [motion['effort'][name] for name in names]
            self.joints.publish(message)
            self.last_feedback_sec = now
        with self.lock:
            roi, mask = self.pending_roi, self.recorded_mask
            if roi is None or not self.num_points or self.points_received_sec is None or now-self.points_received_sec > 1.:
                return
            if now-self.roi_received_sec > 1.:
                self.pending_roi = None
                return
            if self.enable_recorded_self_filter:
                if mask is None or now-self.mask_received_sec > 1.:
                    return
                first = roi.header.stamp.sec+roi.header.stamp.nanosec*1e-9
                second = mask.header.stamp.sec+mask.header.stamp.nanosec*1e-9
                if abs(first-second) > .2:
                    return
            self.pending_roi = None
        if roi.header.frame_id.lstrip('/') != self.source_frame:
            raise ValueError('記録ROIの基台フレームが設定と異なります: '+roi.header.frame_id)
        if not self.ensure_world_tf(buffer):
            return
        # 記録基台と仮想基台を重ねる、明示的な基台座標対応。入力データの保持。
        result = recorded_environment(roi, mask) if self.enable_recorded_self_filter else copy.deepcopy(roi)
        result.header.frame_id = self.frame
        result.header.stamp = self.node.get_clock().now().to_msg()
        self.num_input_voxels, self.num_environment_voxels = len(roi.data), len(result.data)
        self.voxels.publish(result)

    def input_message(self):
        """入力欠落・停止・記録自己マスクの同期不成立の表示。"""
        now = time.monotonic()
        with self.lock:
            topics = [(self.points_topic, self.points_received_sec), (self.roi_topic, self.roi_received_sec)]
            if self.enable_recorded_self_filter:
                topics.append((self.mask_topic, self.mask_received_sec))
            missing = [topic for topic, received_sec in topics if received_sec is None or now-received_sec > 1.]
            if missing:
                return '入力待ち: '+', '.join(missing)
            if not self.num_points:
                return '空の点群: '+self.points_topic
            if self.enable_recorded_self_filter and self.pending_roi is not None and self.recorded_mask is not None:
                first = self.pending_roi.header.stamp
                second = self.recorded_mask.header.stamp
                if abs(first.sec-second.sec+(first.nanosec-second.nanosec)*1e-9) > .2:
                    return '記録ROIと自己マスクの時刻不一致'
        return ''

    def check_node_pair(self, pair_ids):
        """左右経由ノードの組合せ姿勢のみの照合。区間検査なし、直近1組だけの再利用。"""
        from gng_control_msgs.srv import CheckArmPair
        key = (pair_ids['left_arm'], pair_ids['right_arm'])
        if self.checked_pair is not None and self.checked_pair[0] == key:
            return self.checked_pair[1]
        if not self.pair_client.service_is_ready():
            return None
        request = CheckArmPair.Request(left_node_id=int(key[0]), right_node_id=int(key[1]))
        future = self.pair_client.call_async(request)
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(.5):
            future.cancel()
            return None
        result = future.result()
        if result is None or not result.is_valid or result.has_path_check:
            return None
        self.checked_pair = (key, bool(result.is_collision_free))
        return self.checked_pair[1]

    def status(self):
        return dict(namespace=self.namespace, joint_state_topic='/'+self.namespace+'/joint_states',
                    roi_topic=self.roi_topic, num_points=self.num_points,
                    num_input_voxels=self.num_input_voxels, num_environment_voxels=self.num_environment_voxels)

    def close(self):
        if self.is_closed:
            return
        with self.lock:
            self.is_closed = True
        try:
            if self.process is not None and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        self.process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self.process.wait(timeout=2)
            # launch終了後の残留子ノードも、所有セッションのプロセスグループで停止。
            if self.process is not None:
                for sig in (signal.SIGTERM, signal.SIGKILL):
                    try:
                        os.killpg(self.process.pid, sig)
                    except ProcessLookupError:
                        break
                    if sig == signal.SIGTERM:
                        end_sec = time.monotonic()+3.
                        while time.monotonic() < end_sec:
                            try:
                                os.killpg(self.process.pid, 0)
                            except ProcessLookupError:
                                break
                            time.sleep(.05)
        finally:
            if self.pair_client is not None:
                self.node.destroy_client(self.pair_client)
            for subscription in self.subscriptions:
                self.node.destroy_subscription(subscription)
            for publisher in self.publishers:
                self.node.destroy_publisher(publisher)
            if self.log is not None:
                self.log.close()
            if self.directory is not None:
                self.directory.cleanup()
            with runtime_lock:
                active_runtimes.discard(self.namespace)
