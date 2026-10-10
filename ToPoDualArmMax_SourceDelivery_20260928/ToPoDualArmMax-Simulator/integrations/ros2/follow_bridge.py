"""共通追従構成のブラウザ窓口。ROS設定変更と既存実機サービスへの接続。"""
import json
import math
import threading
import time

from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger


class follow_bridge:
    def __init__(self, node, outputs):
        self.node = node
        self.lock = threading.Lock()
        self.config, self.status = {}, {}
        self.simulator_owner = None
        self.simulator_owner_session_id = ''
        self.config_sec = -math.inf
        self.simulator_pub = outputs.create_publisher(JointState, '/robot_follow/simulator_state', 2)
        self.simulator_status_pub = outputs.create_publisher(String, '/robot_follow/simulator_status', 2)
        self.heartbeat = outputs.create_publisher(Empty, '/robot_follow/follower/heartbeat', 1)
        node.create_subscription(String, '/robot_follow/config', self.on_config,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(String, '/robot_follow/status', self.on_status, 1)
        self.profile_client = node.create_client(SetParametersAtomically, '/robot_follow/manager/set_parameters_atomically')
        self.clients = {name: node.create_client(kind, '/robot_follow/follower/'+name)
                        for name, kind in [('enable', SetBool), ('follow', SetBool), ('stop', Trigger), ('reset', Trigger), ('torque_off', Trigger)]}

    def on_config(self, message):
        try:
            config = json.loads(message.data)
            if isinstance(config, dict) and config.get('profile') in config.get('profiles', {}):
                with self.lock:
                    self.config, self.config_sec = config, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            if isinstance(status, dict):
                with self.lock:
                    self.status = status
        except (ValueError, TypeError):
            pass

    def snapshot(self):
        with self.lock:
            return {'has_manager': time.monotonic()-self.config_sec < 2., 'config': dict(self.config), 'status': dict(self.status)}

    def wait_result(self, client, request):
        if not client.service_is_ready():
            raise ValueError('追従管理サービスが未起動です')
        future = client.call_async(request)
        end = time.monotonic()+3.
        while not future.done() and time.monotonic() < end:
            time.sleep(.01)
        if not future.done():
            raise TimeoutError('追従管理サービスの応答時間超過')
        return future.result()

    def perform(self, data):
        snapshot = self.snapshot()
        if not snapshot['has_manager']:
            raise ValueError('robot_follow.launch.pyの起動が必要です')
        config = snapshot['config']
        if data.get('session_id') != config['session_id']:
            raise ValueError('構成が変更されています。一覧を更新してください')
        action = data.get('action')
        if action == 'profile':
            name = data.get('profile')
            if not isinstance(name, str) or name not in config['profiles']:
                raise ValueError('未登録の構成です')
            response = self.wait_result(self.profile_client, SetParametersAtomically.Request(parameters=[
                Parameter(name='profile', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=name))]))
            if not response.result.successful:
                raise ValueError(response.result.reason)
            return {'success': True}
        if action == 'heartbeat':
            if not config['allow_hardware_output']:
                raise ValueError('実機出力はlaunchでOFFです')
            self.heartbeat.publish(Empty())
            return {'success': True}
        if action not in self.clients:
            raise ValueError('未対応の操作です')
        if action in ('enable', 'follow') and (not config['allow_hardware_output'] or config['profiles'][config['profile']]['follower_source'] == 'none'):
            raise ValueError('実機出力許可とfの入力元の選択が必要です')
        request = SetBool.Request(data=True) if action in ('enable', 'follow') else Trigger.Request()
        response = self.wait_result(self.clients[action], request)
        if not response.success:
            raise ValueError(response.message)
        return {'success': True, 'message': response.message}

    def claim_simulator(self, owner, state):
        """同じ管理構成での複数ブラウザ送信の競合拒否。"""
        with self.lock:
            if not self.config or state.get('follow_session_id') != self.config.get('session_id'):
                return
            if self.simulator_owner_session_id == self.config['session_id'] and self.simulator_owner is not None and self.simulator_owner is not owner:
                raise ValueError('sの状態送信は別のブラウザが使用中です')
            self.simulator_owner = owner
            self.simulator_owner_session_id = self.config['session_id']

    def release_simulator(self, owner):
        """sの送信端切断時の所有解除と、s追従中のfへの停止要求。"""
        with self.lock:
            if self.simulator_owner is not owner:
                return
            self.simulator_owner = None
            config = self.config
            has_simulator_target = bool(config) and config['profiles'][config['profile']]['follower_source'] == 's'
        if has_simulator_target and self.clients['stop'].service_is_ready():
            self.clients['stop'].call_async(Trigger.Request())

    def publish_simulator(self, state, result):
        snapshot = self.snapshot()
        config = snapshot['config']
        if not snapshot['has_manager'] or state.get('follow_session_id') != config.get('session_id'):
            return
        age_sec = (time.time()*1000-state['captured_at_ms'])/1000
        if not 0 <= age_sec < config['max_state_age_sec'] or state['robot_model'] != config['robot_model']:
            return
        status = state.get('follow_simulation')
        if not isinstance(status, dict):
            return
        status = {**status, 'session_id': config['session_id']}
        self.simulator_status_pub.publish(String(data=json.dumps(status)))
        sample = JointState(name=list(state['robot_pose']), position=list(map(float, state['robot_pose'].values())))
        sample.header.stamp.sec, sample.header.stamp.nanosec = result['stamp_sec'], result['stamp_nanosec']
        sample.header.frame_id = 'robot_follow_s:'+config['session_id']
        self.simulator_pub.publish(sample)
