#!/usr/bin/env python3
"""取付校正の永続化・単一TF配信・利用ノードの反映状態管理。"""
import argparse
from copy import deepcopy
from enum import Enum
import fcntl
import hashlib
import json
import os
import signal
import stat
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

from ais_gng_transform_config import read_pose


class calibration_status(str, Enum):
    ready = 'ready'
    conflict = 'conflict'
    pending = 'pending'
    blocked = 'blocked'


def pose_matrix(pos, quat):
    pos, quat = np.asarray(pos, dtype=float), np.asarray(quat, dtype=float)
    if pos.shape != (3,) or quat.shape != (4,) or not np.isfinite(pos).all() or not np.isfinite(quat).all():
        raise ValueError('位置3個・クォータニオン4個の有限値が必要です')
    if abs(float(np.linalg.norm(quat)) - 1.) > 1e-6:
        raise ValueError('正規化したクォータニオンが必要です')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
    matrix[:3, 3] = pos
    return matrix


def matrix_pose(matrix):
    return matrix[:3, 3].tolist(), Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()


def validate_frame(frame):
    if not isinstance(frame, str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):
        raise ValueError('frame名には先頭スラッシュ・空白なしの文字列が必要です')
    return frame


class calibration_store:
    """公称取付と補正の分離。外部編集・古い版からの上書き防止。"""
    def __init__(self, path, mounts, runtime_dir):
        self.path = Path(path).expanduser().resolve()
        self._store_lock_path = Path(runtime_dir) / ('store_' + self.path.name + '.lock')
        self._content = self.path.read_bytes() if self.path.exists() else b''
        self._document = yaml.safe_load(self._content) if self._content else {'schema_version': 1, 'calibrations': {}}
        if (not isinstance(self._document, dict) or self._document.get('schema_version') != 1
                or not isinstance(self._document.get('calibrations'), dict)):
            raise ValueError('校正ファイルの形式が不正です')
        self._mounts = {}
        children = set()
        for raw in mounts:
            mount = deepcopy(raw)
            name = mount['mount_id']
            parent, child = [validate_frame(mount[key]) for key in ('parent_frame', 'child_frame')]
            if not isinstance(name, str) or not name or name in self._mounts or child in children or parent == child:
                raise ValueError('取付ID・子frameの重複または親子frameの一致')
            nominal = pose_matrix(mount['pos'], mount['quat'])
            signature = hashlib.sha256(json.dumps([parent, child, nominal.tolist()]).encode()).hexdigest()
            mount.update(nominal=nominal, nominal_hash=signature)
            mount.setdefault('imu_topics', [])
            self._mounts[name] = mount
            children.add(child)
            record = self._document['calibrations'].get(name)
            if record:
                if record['nominal_hash'] != signature:
                    raise ValueError(f'{name}: 公称取付の変更。保存済み校正の再確認が必要です')
                pose_matrix(record['pos'], record['quat'])

        parents = {mount['child_frame']: mount['parent_frame'] for mount in self._mounts.values()}
        for child in parents:
            visited = set()
            while child in parents:
                if child in visited:
                    raise ValueError('公称取付TFの循環です')
                visited.add(child)
                child = parents[child]

    def names(self):
        return list(self._mounts)

    def mount(self, name):
        if name not in self._mounts:
            raise ValueError('未登録の取付IDです')
        return deepcopy(self._mounts[name])

    def revision(self, name):
        self.mount(name)
        return self._document['calibrations'].get(name, {}).get('revision', 0)

    def last_operation(self, name):
        record = self._document['calibrations'].get(name, {})
        return record.get('last_operation_id', ''), record.get('operation_hash', '')

    def effective(self, name):
        mount = self.mount(name)
        record = self._document['calibrations'].get(name)
        correction = pose_matrix(record['pos'], record['quat']) if record else np.eye(4)
        return mount['nominal'] @ correction

    def commit(self, name, revision, transform, operation_id, operation_hash):
        with self._store_lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            mount = self.mount(name)
            if type(revision) is not int or revision != self.revision(name):
                raise ValueError('校正版が更新されています。再取得が必要です')
            if (self.path.read_bytes() if self.path.exists() else b'') != self._content:
                raise ValueError('校正ファイルが外部で変更されています。再起動・再確認が必要です')
            correction = np.linalg.inv(mount['nominal']) @ transform
            pos, quat = matrix_pose(correction)
            pose_matrix(pos, quat)
            document = deepcopy(self._document)
            document['calibrations'][name] = dict(nominal_hash=mount['nominal_hash'], revision=revision + 1, pos=pos, quat=quat,
                                                 last_operation_id=operation_id, operation_hash=operation_hash)
            content = yaml.safe_dump(document, allow_unicode=True, sort_keys=False).encode()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            file_stat = self.path.stat() if self.path.exists() else self.path.parent.stat()
            mode = stat.S_IMODE(file_stat.st_mode) if self.path.exists() else 0o644
            try:
                with tempfile.NamedTemporaryFile(dir=self.path.parent, delete=False) as stream:
                    temporary = Path(stream.name)
                    os.fchown(stream.fileno(), file_stat.st_uid, file_stat.st_gid)
                    os.fchmod(stream.fileno(), mode)
                    stream.write(content)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
                descriptor = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
            finally:
                if temporary and temporary.exists():
                    temporary.unlink()
            self._document, self._content = document, content


class sensor_calibration_node(Node):
    def __init__(self):
        super().__init__('sensor_calibration')
        store_file = self.declare_parameter('store_file', '').value
        runtime_dir = self.declare_parameter('runtime_dir', '').value
        mounts = json.loads(self.declare_parameter('mounts_json', '[]').value)
        if not store_file or not runtime_dir:
            raise ValueError('store_file・runtime_dirの指定が必要です')
        Path(runtime_dir).mkdir(parents=True, exist_ok=True)
        self._authority_lock = (Path(runtime_dir) / (os.environ.get('ROS_DOMAIN_ID', '0') + '.lock')).open('a')
        fcntl.flock(self._authority_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.store = calibration_store(store_file, mounts, runtime_dir)
        self._records, self._consumers, self._imu_frames = {}, {}, {}
        self._started = time.monotonic()
        self._published_revisions = {}
        self._last_state = ''
        self._static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tf_pub = self.create_publisher(TFMessage, '/tf_static', self._static_qos)
        self._state_pub = self.create_publisher(String, '/sensor_calibration/state', self._static_qos)
        self._response_pub = self.create_publisher(String, '/sensor_calibration/response', 20)
        self.create_subscription(String, '/sensor_calibration/request', self.handle_request, 20)
        self._tf_publishers = {}
        self._observer_at = 0.
        self.create_subscription(String, '/sensor_calibration/tf_sources', self.observe_tf, self._static_qos)
        for topic in sorted({topic for mount in mounts for topic in mount.get('imu_topics', [])}):
            self.create_subscription(Imu, topic, self.imu_callback(topic),
                                     rclpy.qos.qos_profile_sensor_data)
        self.create_timer(.5, self.tick)

    def imu_callback(self, topic):
        def callback(message):
            self._imu_frames[topic] = message.header.frame_id
        return callback

    def full_name(self, endpoint):
        return (endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name)

    def own_name(self):
        return self.get_namespace().rstrip('/') + '/' + self.get_name()

    def observe_tf(self, message):
        snapshot = json.loads(message.data)
        self._tf_publishers = snapshot['publishers']
        self._records = {(row['child_frame'], row['publisher_id']): row for row in snapshot['records']}
        self._observer_at = time.monotonic()

    def foreign_parent(self, child):
        publishers = self._tf_publishers
        matches = [value for (frame, publisher_id), value in self._records.items()
                   if frame == child and publisher_id in publishers and publishers[publisher_id] != self.own_name()]
        if len(matches) != 1:
            raise ValueError(f'{child}: 一意な内部TFが未取得です')
        return matches[0]

    def relative(self, body, source):
        result, visited = np.eye(4), set()
        while source != body:
            if source in visited:
                raise ValueError('内部TFの循環です')
            visited.add(source)
            transform = self.foreign_parent(source)
            if not transform['is_static']:
                raise ValueError('取付校正の入力には固定のセンサ内部TFが必要です')
            result = pose_matrix(transform['pos'], transform['quat']) @ result
            source = transform['parent_frame']
        return result

    def status(self, name):
        mount = self.store.mount(name)
        if time.monotonic() - self._observer_at > 2.:
            return calibration_status.blocked, 'TF配信元の観測待ちです'
        publishers = self._tf_publishers
        own_ids = {key for key, value in publishers.items() if value == self.own_name()}
        if not own_ids:
            return calibration_status.blocked, '取付TF配信元の発見待ちです'
        if len(own_ids) != 1:
            return calibration_status.conflict, '校正管理ノード名が重複しています'
        own_id = next(iter(own_ids))
        for (child, publisher_id), _ in self._records.items():
            if child == mount['child_frame'] and publisher_id in publishers and publisher_id != own_id:
                return calibration_status.conflict, f'{child}: 別のTF配信元 {publishers[publisher_id]}'
        # 自動的な外部推定器の再初期化なし。利用側からの継続的な反映確認
        for topic in mount['imu_topics']:
            users = {self.full_name(row) for row in self.get_subscriptions_info_by_topic(topic)} - {self.own_name()}
            if users:
                source = self._imu_frames.get(topic, '')
                try:
                    self.relative(mount['child_frame'], validate_frame(source))
                except ValueError:
                    return calibration_status.blocked, f'{topic}: IMUの内部TFが未確認です'
            for user in users:
                consumer = self._consumers.get((name, user))
                if not consumer or time.monotonic() - consumer['received_at'] > 5:
                    return calibration_status.blocked, f'{user}: 校正反映の対応が未確認です'
                if consumer['mode'] != 'tf':
                    return calibration_status.blocked, f'{user}: 校正変更には停止・再初期化が必要です'
                if consumer['revision'] != self.store.revision(name):
                    return calibration_status.pending, f'{user}: 校正版の反映待ち'
        return calibration_status.ready, ''

    def state(self, source_frame=''):
        mounts = []
        for name in self.store.names():
            mount = self.store.mount(name)
            status, reason = self.status(name)
            pos, quat = matrix_pose(self.store.effective(name))
            is_tf_applied = (self._published_revisions.get(name) == self.store.revision(name)
                             and status != calibration_status.conflict and time.monotonic() - self._observer_at <= 2.)
            can_apply = status == calibration_status.ready and is_tf_applied
            if source_frame:
                try:
                    self.relative(mount['child_frame'], source_frame)
                except ValueError as error:
                    can_apply, reason = False, str(error)
            mounts.append(dict(mount_id=name, parent_frame=mount['parent_frame'], child_frame=mount['child_frame'],
                               revision=self.store.revision(name), pos=pos, quat=quat, status=status.value,
                               reason=reason, can_apply=can_apply, is_tf_applied=is_tf_applied, last_operation_id=self.store.last_operation(name)[0]))
        return dict(mounts=mounts, store_file=str(self.store.path))

    def publish_tf(self, allow_pending=False):
        transforms, published = [], []
        for name in self.store.names():
            status = self.status(name)[0]
            if status != calibration_status.ready and not (allow_pending and status == calibration_status.pending):
                continue
            mount = self.store.mount(name)
            pos, quat = matrix_pose(self.store.effective(name))
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = mount['parent_frame'], mount['child_frame']
            transform.header.stamp = self.get_clock().now().to_msg()
            value = transform.transform
            value.translation.x, value.translation.y, value.translation.z = pos
            value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w = quat
            transforms.append(transform)
            published.append(name)
        if transforms:
            self._tf_pub.publish(TFMessage(transforms=transforms))
        for name in published:
            self._published_revisions[name] = self.store.revision(name)

    def tick(self):
        for name in self.store.names():
            if self.status(name)[0] == calibration_status.conflict:
                self._published_revisions.pop(name, None)
        if time.monotonic() - self._started >= 1. and any(
                self._published_revisions.get(name) != self.store.revision(name)
                and self.status(name)[0] == calibration_status.ready for name in self.store.names()):
            self.publish_tf()
        state = json.dumps(self.state(), ensure_ascii=False, sort_keys=True)
        if state != self._last_state:
            self._state_pub.publish(String(data=state))
            self._last_state = state

    def handle_request(self, message):
        request = {}
        try:
            request = json.loads(message.data)
            ident, method, params = request['id'], request['method'], request.get('params', {})
            if not isinstance(ident, str) or not ident or not isinstance(params, dict):
                raise ValueError('要求ID・引数が不正です')
            if method == 'calibration.list':
                result = self.state(params.get('source_frame', ''))
            elif method == 'calibration.consumer':
                name, node_name = params['mount_id'], params['node_name']
                self.store.mount(name)
                if params['mode'] not in ('tf', 'restart') or type(params['revision']) is not int:
                    raise ValueError('利用側の校正対応情報が不正です')
                self._consumers[(name, node_name)] = dict(mode=params['mode'], revision=params['revision'], received_at=time.monotonic())
                result = self.state()
            elif method == 'calibration.apply':
                name = params['mount_id']
                operation_id = params['operation_id']
                if not isinstance(operation_id, str) or not operation_id or len(operation_id) > 128:
                    raise ValueError('校正操作IDが不正です')
                operation_hash = hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest()
                if self.store.last_operation(name)[0] == operation_id:
                    if (self._published_revisions.get(name) != self.store.revision(name) or time.monotonic() - self._observer_at > 2.
                            or self.status(name)[0] == calibration_status.conflict):
                        raise ValueError('保存済み校正のTF反映が未確認です')
                    if self.store.last_operation(name)[1] != operation_hash:
                        raise ValueError('同じ校正操作IDに異なる内容が指定されています')
                    self._response_pub.publish(String(data=json.dumps(dict(id=ident, ok=True,
                        result=dict(is_tf_applied=True, **self.state(params['source_frame']))), ensure_ascii=False)))
                    return
                status, reason = self.status(name)
                if status != calibration_status.ready or self._published_revisions.get(name) != self.store.revision(name):
                    raise ValueError(reason or 'TF配信の初期確認待ちです')
                mount = self.store.mount(name)
                relative = self.relative(mount['child_frame'], validate_frame(params['source_frame']))
                pos, quat = read_pose(params['delta'])
                transform = self.store.effective(name) @ relative @ pose_matrix(pos, quat) @ np.linalg.inv(relative)
                self.store.commit(name, params['revision'], transform, operation_id, operation_hash)
                self.publish_tf(allow_pending=True)
                result = dict(is_tf_applied=self._published_revisions.get(name) == self.store.revision(name),
                              **self.state(params['source_frame']))
            else:
                raise ValueError('未対応の校正操作です')
            response = dict(id=ident, ok=True, result=result)
        except (ValueError, KeyError, TypeError, OSError) as error:
            response = dict(id=request.get('id', '') if isinstance(request, dict) else '', ok=False,
                            error=dict(code='CALIBRATION_REJECTED', message=str(error)))
        self._response_pub.publish(String(data=json.dumps(response, ensure_ascii=False)))

    def destroy_node(self):
        result = super().destroy_node()
        self._authority_lock.close()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', help='校正API要求のJSON（method・params）')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        if args.request:
            node = rclpy.create_node('sensor_calibration_cli')
            responses = []
            request = json.loads(args.request)
            request['id'] = uuid.uuid4().hex
            publisher = node.create_publisher(String, '/sensor_calibration/request', 20)
            node.create_subscription(String, '/sensor_calibration/response',
                                     lambda msg: responses.append(json.loads(msg.data)), 20)
            end = time.monotonic() + 5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
                if publisher.get_subscription_count():
                    publisher.publish(String(data=json.dumps(request)))
                    break
            end = time.monotonic() + 5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
                for response in responses:
                    if response.get('id') == request['id']:
                        print(json.dumps(response, ensure_ascii=False))
                        return 0 if response['ok'] else 1
            raise RuntimeError('校正管理ノードからの応答がありません')
        node = sensor_calibration_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
