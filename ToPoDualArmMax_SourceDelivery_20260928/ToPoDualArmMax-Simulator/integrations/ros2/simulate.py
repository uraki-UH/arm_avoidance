"""ブラウザと共通のMuJoCo・回避処理によるCLI実行。有限記録・連続表示の選択。"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener


simulator_dir = Path(__file__).resolve().parents[2]


def nonnegative(value):
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise argparse.ArgumentTypeError('有限の非負値が必要です')
    return result


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('live','chasing_sphere'), default='live', help='既存ROS入力／指定リンク追跡障害物の有限試験')
    parser.add_argument('--scenario-file', type=Path, help='追跡球の試験設定JSON')
    parser.add_argument('--case-dir', type=Path, help='追跡球の結果保存先。省略時は日時付きの新規ディレクトリ')
    parser.add_argument('--stream-frames', dest='enable_frame_stream', action='store_true', help='追跡実験の実測フレームの逐次JSONL出力')
    parser.add_argument('--target-link', help='追跡対象のURDFリンク名。追跡試験のみ、既定L_tcp')
    parser.add_argument('--target-offset', type=float, nargs=3, metavar=('X','Y','Z'), help='追跡リンク内の対象点 [m]')
    parser.add_argument('--model', choices=('long', 'standard'), default='long')
    parser.add_argument('--avoidance', choices=('gng', 'none', 'oscbf', 'ros'), default='gng')
    parser.add_argument('--params-file', type=Path, help='機体YAML。ROS外部制御時の基台配置と共用')
    parser.add_argument('--state-topic', default='/sim/joint_states', help='ROS外部制御時のMuJoCo実測出力')
    parser.add_argument('--command-topic', default='/avoidance/joint_command', help='ROS外部制御時の即時関節指令入力')
    parser.add_argument('--gng-strategy', choices=('graph', 'avoidance_first', 'vrm_predictive'), default='graph',
                        help='GNG方策。既定のノード列／回避優先／予測・不一致適応')
    parser.add_argument('--roi-visibility', choices=('fov_shadow', 'ray', 'strict', 'legacy'), default='fov_shadow',
                        help='視野内free＋遮蔽／観測光線／観測制約なし。strictは旧ray指定の互換名')
    parser.add_argument('--pose', choices=('forward', 'home', 'zero'), default='forward')
    parser.add_argument('--pose-file', type=Path, help='初期関節角のJSON辞書 [rad]。プリセットへの部分上書き')
    parser.add_argument('--goal-file', type=Path, help='復帰目標のJSON辞書 [rad]。初期姿勢への部分上書き')
    parser.add_argument('--task-file', type=Path, help='往復タスクJSON。開始姿勢と二つの手先目標、実測到達による切替')
    parser.add_argument('--min-round-trips', type=int, default=0, help='終了時の実測往復回数の検証下限')
    parser.add_argument('--duration', type=nonnegative, help='実行時間 [sec]。ROS入力の既定値30、追跡球は設定JSON。0はROS入力のみ')
    parser.add_argument('--report-sec', type=nonnegative, default=1., help='JSONL出力周期 [sec]')
    parser.add_argument('--output', type=Path, help='JSONL保存先。既存ファイルへの上書き禁止')
    parser.add_argument('--viewer-bridge-port', type=int, help='実測姿勢のViewer受信用ROSブリッジHTTPポート')
    parser.add_argument('--viewer-observe-port', type=int, help='同じROSドメインのTopics・点群・Tmap表示サーバーポート')
    parser.add_argument('--domain-id', type=int, choices=range(233), metavar='0..232')
    parser.add_argument('--points-topic', default='/livox/lidar')
    parser.add_argument('--input-kind', choices=('pointcloud', 'voxel'), default='pointcloud', help='観測入力。既定は点群・TFからの再生成')
    parser.add_argument('--roi-topic', default='/topo_dual_arm_max_long/roi_voxels')
    parser.add_argument('--recorded-self-topic', default='/topo_dual_arm_max_long/self_voxel')
    parser.add_argument('--bag', type=Path, help='CLI所有のbag再生。省略時は既存ROS入力を使用')
    parser.add_argument('--start-offset', type=nonnegative, default=0., help='bag開始位置 [sec]')
    parser.add_argument('--rate', type=nonnegative, default=1., help='bag再生速度')
    parser.add_argument('--min-motion-deg', type=nonnegative, default=0., help='終了時の腕関節変化の検証下限 [deg]')
    parser.add_argument('--nonstop-simulation', action='store_true', help='実機非接続の非停止往復・bag回避。終了はCtrl+C')
    args = parser.parse_args()
    if args.nonstop_simulation:
        if args.scenario != 'live' or args.avoidance not in ('gng', 'ros'):
            parser.error('非停止実験はROS入力のMuJoCo実行専用です')
        if args.state_topic != '/sim/joint_states' or args.command_topic not in ('/avoidance/joint_command', '/sim/nonstop/joint_command'):
            parser.error('非停止実験の実測・指令はMuJoCo専用トピックです')
        args.avoidance, args.command_topic = 'ros', '/sim/nonstop/joint_command'
        args.task_file = args.task_file or simulator_dir.parents[1]/'benchmarks/shuttle_bag/task.json'
        args.params_file = args.params_file or simulator_dir.parents[1]/'gng_vlut_system/config/topo_dual_arm_max_long.yaml'
        args.domain_id = 168 if args.domain_id is None else args.domain_id
        args.viewer_bridge_port = args.viewer_bridge_port or 18879
        args.viewer_observe_port = args.viewer_observe_port or 19001
        args.duration = 0. if args.duration is None else args.duration
    if args.scenario == 'chasing_sphere':
        if args.model != 'long' or args.avoidance != 'gng' or args.bag or args.viewer_bridge_port or args.viewer_observe_port or args.pose_file or args.goal_file or args.task_file:
            parser.error('追跡球はLongの独立物理試験です。bag・Viewer接続・個別姿勢入力との併用は未対応です')
        if args.duration is not None and not 1 <= args.duration <= 180:
            parser.error('追跡球の--durationは1～180 secです')
    else:
        if args.scenario_file or args.case_dir or args.target_link is not None or args.target_offset is not None or args.enable_frame_stream:
            parser.error('--scenario-fileと--case-dirは追跡球用です')
        if args.duration is None:
            args.duration = 30.
    if args.report_sec == 0 or args.rate == 0:
        parser.error('--report-secと--rateには正値が必要です')
    if args.bag and args.avoidance != 'gng' and not args.nonstop_simulation:
        parser.error('--bagはGNG回避用です')
    if args.avoidance == 'ros' and args.goal_file:
        parser.error('ROS外部制御の目標は回避ノードのgoalトピックへ指定してください')
    if args.gng_strategy != 'graph' and args.avoidance != 'gng' and not args.nonstop_simulation:
        parser.error('--gng-strategyはGNG回避用です')
    if args.task_file and (args.pose_file or args.goal_file or args.avoidance not in ('gng', 'none') and not args.nonstop_simulation):
        parser.error('--task-fileはgng／none用です。--pose-file・--goal-fileとの併用不可')
    if args.min_round_trips < 0 or args.min_round_trips and not args.task_file:
        parser.error('--min-round-tripsには--task-fileと非負整数が必要です')
    if args.viewer_bridge_port is not None and not 1024 <= args.viewer_bridge_port <= 65534:
        parser.error('--viewer-bridge-portは1024～65534です')
    if args.viewer_bridge_port and args.avoidance not in ('gng', 'ros', 'none'):
        parser.error('--viewer-bridge-portはGNG・ROS外部制御・回避なしの実測JointState出力用です')
    if args.viewer_observe_port is not None:
        if not args.viewer_bridge_port or not 1024 <= args.viewer_observe_port <= 65535:
            parser.error('--viewer-observe-portには有効なポートと--viewer-bridge-portが必要です')
        if args.viewer_observe_port in (args.viewer_bridge_port, args.viewer_bridge_port+1):
            parser.error('Topics表示と関節ブリッジのポートが重複しています')
    return args


def read_pose(path):
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or any(not isinstance(name, str) or type(angle) not in (int, float)
                                         or not math.isfinite(angle) for name, angle in value.items()):
        raise ValueError('姿勢JSONには関節名と有限の角度 [rad] が必要です')
    return value


class shuttle_task:
    """実測手先の位置・姿勢・速度による二点往復。微小関節変位による往復数の代用なし。"""
    def __init__(self, path, model, params_file=None):
        import numpy as np
        import yaml
        sys.path.insert(0, str(simulator_dir.parents[1]/'gng_vlut_system/scripts'))
        from dual_arm_avoidance_geometry import robot_geometry
        from gng_sphere_avoidance import cartesian_task
        config = json.loads(path.read_text())
        if config.get('model') != model or set(config.get('poses_deg', {})) != {'a', 'b'}:
            raise ValueError('往復タスクの機体または二姿勢の指定不正')
        config_path = params_file or simulator_dir.parents[1]/('gng_vlut_system/config/topo_dual_arm_max'+('_long' if model == 'long' else '')+'.yaml')
        params = yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
        self.geometry = robot_geometry(params['urdf_path'], environment_shapes=())
        if config.get('frame') != self.geometry.root_link:
            raise ValueError('往復タスクの目標座標系がURDF基準と異なります')
        names = self.geometry.joint_names
        self.poses = {}
        for key in ('a', 'b'):
            values = config['poses_deg'][key]
            if (not isinstance(values, dict) or set(values)-set(names)
                    or any(type(value) not in (int, float) or not math.isfinite(value) for value in values.values())):
                raise ValueError('往復タスクの関節名または角度の指定不正')
            positions = np.radians([values.get(name, 0.) for name in names])
            if np.any(positions < self.geometry.limits[:, 0]) or np.any(positions > self.geometry.limits[:, 1]):
                raise ValueError('往復タスクの関節角度がURDF可動域外です')
            self.poses[key] = positions
        profile = config.get('active_profile')
        self.active_profile = profile
        profiles = params.get('gng', {}).get('profiles', {})
        if profile not in profiles:
            raise ValueError('往復タスクの操作グループの指定不正')
        # URDFの操作チェーンからの抽出。回避なしの対照試験に学習済みGNG/VLUTは不要。
        group = profiles[profile]
        chain = {entry[2]: entry for entry in self.geometry.joints}
        active, link = [], group['eef']
        while link != group['root']:
            if link not in chain:
                raise ValueError('往復タスクの操作チェーンがURDFにありません')
            name, parent = chain[link][:2]
            if name in names and name not in group.get('dof_exclude', []):
                active.append(names.index(name))
            link = parent
        if any(self.poses['a'][idx] != self.poses['b'][idx] for idx in range(len(names)) if idx not in active):
            raise ValueError('往復タスクの非操作関節に姿勢差あり')
        self.task = cartesian_task(self.geometry, active, self.poses, config.get('eef_link', 'L_tcp'),
            config.get('max_eef_position_error_th', .015), math.radians(config.get('max_eef_orientation_error_deg_th', 3.)))
        first, second = (self.task.goals[key] for key in ('a', 'b'))
        from gng_sphere_avoidance import rotation_error
        if (np.linalg.norm(first[:3,3]-second[:3,3]) <= 2*self.task.max_position_error_th
                and np.linalg.norm(rotation_error(first[:3,:3], second[:3,:3])) <= 2*self.task.max_orientation_error_rad_th):
            raise ValueError('往復目標の到達許容領域に重複あり')
        self.task.goal = 'a'
        self.num_arrivals = 0
        self.has_started = False
        self.last_frame = self.last_sec = None
        self.max_eef_motion_m = 0.
        self.status = {}

    @property
    def initial_pose(self):
        return dict(zip(self.geometry.joint_names, map(float, self.poses['a'])))

    def attach(self, scene):
        """制御基台の配置に基づく目標座標系の確定。観測bagの基台姿勢の転用なし。"""
        world_base = scene.link_transform(self.geometry.root_link)
        self.task.goals = {key: world_base@value for key, value in self.task.goals.items()}
        self.reference = self.task.metadata('joint')

    def update(self, scene):
        import numpy as np
        from gng_sphere_avoidance import rotation_error
        frame, now = scene.link_transform(self.task.eef_link), float(scene.data.time)
        elapsed = 0. if self.last_sec is None else now-self.last_sec
        linear = np.zeros(3) if elapsed <= 0 else (frame[:3,3]-self.last_frame[:3,3])/elapsed
        angular = np.zeros(3) if elapsed <= 0 else rotation_error(frame[:3,:3], self.last_frame[:3,:3])/elapsed
        position_error, orientation_error, can_arrive = self.task.measure(frame, linear, angular)
        self.last_frame, self.last_sec = frame.copy(), now
        dist_m = float(np.linalg.norm(frame[:3,3]-self.task.goals['a'][:3,3]))
        self.max_eef_motion_m = max(self.max_eef_motion_m, dist_m)
        if elapsed > 0 and can_arrive:
            if self.has_started:
                self.num_arrivals += 1
            self.has_started = True
            self.task.goal = 'b' if self.task.goal == 'a' else 'a'
            position_error, orientation_error, _ = self.task.measure(frame, linear, angular)
        self.status = dict(self.reference, goal=self.task.goal, num_arrivals=self.num_arrivals,
            num_round_trips=self.num_arrivals//2, has_started=self.has_started,
            position_error_m=position_error, orientation_error_rad=orientation_error,
            eef_m=frame[:3,3].tolist(), max_eef_motion_m=self.max_eef_motion_m)
        return dict(zip(self.geometry.joint_names, map(float, self.poses[self.task.goal])))


def stop_process(process):
    """CLI所有のプロセスグループだけの停止。"""
    if process is None or process.poll() is not None:
        return
    for sig, wait_sec in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait_sec)
            return
        except subprocess.TimeoutExpired:
            pass


def wait_for_viewer_service(process, port, stop_event, enable_http=False):
    """表示サーバーの初期化完了後だけのViewer URL通知。"""
    opener = build_opener(ProxyHandler({}))
    end_sec = time.monotonic()+30.
    while not stop_event.is_set() and time.monotonic() < end_sec:
        if process.poll() is not None:
            raise RuntimeError('Viewer受信用ROSブリッジの起動失敗')
        try:
            if enable_http:
                with opener.open(f'http://127.0.0.1:{port}/api/points/status', timeout=.5) as response:
                    if response.status == 200:
                        return True
            else:
                with socket.create_connection(('127.0.0.1', port), timeout=.5):
                    return True
        except (URLError, OSError):
            pass
        stop_event.wait(.1)
    if stop_event.is_set():
        return False
    raise RuntimeError('Viewer受信用ROSブリッジの起動時間超過')


def read_chase_config(args):
    """保存先を開く前の追跡実験設定と出力条件の検査。"""
    config_file = args.scenario_file or simulator_dir.parents[1]/'benchmarks/chasing_sphere/default.json'
    if str(config_file)=='-':
        body = sys.stdin.read(8193)
        if len(body)>8192:
            raise ValueError('実験設定の容量超過')
        config = json.loads(body)
    else:
        config = json.loads(config_file.read_text())
    if not isinstance(config,dict):
        raise ValueError('試験設定はJSONオブジェクトの指定が必要')
    if args.target_link is not None:
        config['target_link'] = args.target_link
    if args.target_offset is not None:
        config['target_offset_m'] = args.target_offset
    if args.duration is not None:
        config['duration_sec'] = args.duration
    if config.get('execution_mode')=='continuous' and (args.case_dir or args.output):
        raise ValueError('連続実験は記録なし。--case-dir・--outputの併用不可')
    return config


def run(args, emit, chase_config=None):
    if args.scenario == 'chasing_sphere':
        from sphere_chase import run_case
        config = chase_config if chase_config is not None else read_chase_config(args)
        if config.get('execution_mode')=='continuous':
            stop_event = threading.Event()
            handlers = {sig:signal.signal(sig,lambda *_:stop_event.set()) for sig in (signal.SIGINT,signal.SIGTERM)}
            try:
                emit(dict(type='started',scenario=args.scenario,execution_mode='continuous',has_recording=False))
                result = run_case(config,emit=emit if args.enable_frame_stream else None,stop_event=stop_event)
                emit(dict(type='experiment_finished',metrics=result['metrics'],failure_reason=result['failure_reason'],
                          failure_details=result['failure_details'],is_valid_scenario=result['is_valid_scenario'],outcome=result['outcome']))
                return 0
            finally:
                for sig,handler in handlers.items():
                    signal.signal(sig,handler)
        output_dir = args.case_dir or simulator_dir.parents[1]/'log/artifacts/chasing_sphere'/time.strftime('manual_%Y%m%d_%H%M%S')
        emit(dict(type='started',scenario=args.scenario,case_dir=str(output_dir)))
        result = run_case(config,output_dir,emit=emit if args.enable_frame_stream else None)
        from sphere_replay import export_catalog, export_recording
        try:
            export_catalog(output_dir)
            recording = export_recording(Path(output_dir),simulator_dir/'app/exports')
            emit(dict(type='replay',viewer_url='http://127.0.0.1:8877/?model=long&replay=latest',recording_url=recording['url']))
        except (OSError,ValueError,KeyError,TypeError) as error:
            emit(dict(type='replay_unavailable',message=str(error)))
        emit(dict(type='summary',scenario=args.scenario,case_dir=str(output_dir),
                  failure_reason=result['failure_reason'],**result['metrics']))
        return 0
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from physics_scene import PhysicsScene

    presets = json.loads((simulator_dir/'app/robot-poses.json').read_text())
    if args.nonstop_simulation and args.bag is None:
        import yaml
        args.bag = Path(yaml.safe_load((simulator_dir.parents[1]/'scripts/arm_avoidance_bag.yaml').read_text())['bag_path'])
    task = shuttle_task(args.task_file, args.model, args.params_file) if args.task_file else None
    pose = {name: math.radians(angle) for name, angle in presets[args.pose].items()}
    if task is not None:
        pose.update(task.initial_pose)
    if args.pose_file:
        pose.update(read_pose(args.pose_file))
    goal = read_pose(args.goal_file) if args.goal_file else None
    if args.bag and not (args.bag/'metadata.yaml').is_file():
        raise ValueError('bagディレクトリのmetadata.yamlが必要です')
    if args.domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('dual_arm_simulation_cli')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()
    errors = []

    def spin():
        try:
            executor.spin()
        except Exception as error:
            errors.append(error)
            stop_event.set()

    thread = threading.Thread(target=spin, name='simulation-ros')
    thread.start()
    handlers = {sig: signal.signal(sig, lambda *_: stop_event.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    scene, bag, bridge, observer, controller = None, None, None, None, None
    session, state_pub, goal_pub = None, None, None
    latest_avoidance = {}
    last_task_goal = None
    last_task_goal_sec = 0.
    phases = collections.Counter()
    max_motion_deg = 0.
    num_frames = 0
    is_ready = args.avoidance != 'gng'
    has_goal = False
    result = None
    try:
        with tempfile.TemporaryDirectory(prefix='topo-simulation-cli-') as directory:
            with open(Path(directory)/'bag.log', 'w+') as bag_log:
                if args.bag:
                    # 重複再生の禁止。既存publisherの検出用の有限待機。
                    stop_event.wait(.5)
                    topics = (args.points_topic,) if args.input_kind == 'pointcloud' else (args.points_topic, args.roi_topic, args.recorded_self_topic)
                    if any(node.count_publishers(topic) for topic in topics):
                        raise ValueError('入力publisherが既に存在します。--bagを省略して既存入力を使用してください')
                    command = ['bash', str(simulator_dir.parents[1]/'scripts/play_bag.sh'), str(args.bag),
                               '--topics-file', '/dev/null']
                    for topic in topics:
                        command.extend(('--topic', topic))
                    # 記録側のセンサ・リンクTF。MuJoCo姿勢入力とは独立。
                    prefix = '/recorded_topo_dual_arm_max'+('_long' if args.model == 'long' else '')
                    command.extend(('--topic', '/tf', '--topic', '/tf_static', '--remap',
                                    '/tf:='+prefix+'/tf', '/tf_static:='+prefix+'/tf_static'))
                    command.extend(('--loop', '--start-offset', str(args.start_offset), '--rate', str(args.rate)))
                    bag = subprocess.Popen(command, stdout=bag_log, stderr=bag_log, start_new_session=True)
                settings = dict(mode='none' if args.avoidance == 'ros' else args.avoidance)
                if args.params_file and args.avoidance == 'gng':
                    settings['params_file'] = str(args.params_file)
                if args.avoidance == 'gng':
                    settings.update(input_kind=args.input_kind, gng_strategy=args.gng_strategy, enable_roi_visibility=args.roi_visibility != 'legacy',
                                    visibility_model='ray' if args.roi_visibility in ('ray', 'strict') else 'fov_shadow',
                                    points_topic=args.points_topic,
                                    roi_topic=args.roi_topic, recorded_self_topic=args.recorded_self_topic)
                if args.bag:
                    settings.update(recorded_tf_topic=prefix+'/tf', recorded_static_tf_topic=prefix+'/tf_static')
                robot_config = dict(model=args.model, pose=pose, position=[0, 0, 0], quaternion=[0, 0, 0, 1])
                if args.avoidance == 'ros':
                    from physics_stream import physics_session
                    from sensor_msgs.msg import JointState
                    from scipy.spatial.transform import Rotation
                    from rclpy.qos import qos_profile_sensor_data
                    if args.params_file:
                        import yaml
                        config = yaml.safe_load(args.params_file.read_text())['/**']['ros__parameters'].get('avoidance', {})
                        robot_config.update(position=config.get('base_pos', [0., 0., 0.]),
                            quaternion=Rotation.from_euler('xyz', config.get('base_rot_deg', [0., 0., 0.]), degrees=True).as_quat().tolist())
                    robot_config.update(command_source='ros', command_topic=args.command_topic)
                    if args.nonstop_simulation:
                        robot_config['enable_nonstop_simulation'] = True
                    session = physics_session(lambda bodies, robot, avoidance: PhysicsScene(bodies, robot, avoidance, node), node)
                    session.start(dict(bodies=[], robot=robot_config, avoidance=settings))
                    scene = session.scene
                    state_pub = node.create_publisher(JointState, args.state_topic, qos_profile_sensor_data)
                    if args.nonstop_simulation:
                        from std_msgs.msg import String
                        goal_pub = node.create_publisher(JointState, '/sim/nonstop/goal', 1)
                        def receive_avoidance(message):
                            nonlocal latest_avoidance
                            latest_avoidance = json.loads(message.data)
                        node.create_subscription(String, '/avoidance/status', receive_avoidance, 10)
                        controller = subprocess.Popen([sys.executable, '-B', str(Path(__file__).with_name('gng_ros.py')),
                            '--params-file', str(args.params_file), '--model', args.model, '--nonstop-simulation',
                            '--active-profile', task.active_profile,
                            '--gng-strategy', args.gng_strategy, '--source-tf-topic', prefix+'/tf',
                            '--source-static-tf-topic', prefix+'/tf_static'], stdout=sys.stderr, stderr=sys.stderr,
                            start_new_session=True)
                else:
                    scene = PhysicsScene([], robot_config, settings, node)
                if task is not None:
                    task.attach(scene)
                if args.viewer_bridge_port and state_pub is None and args.avoidance == 'none':
                    from sensor_msgs.msg import JointState
                    from rclpy.qos import qos_profile_sensor_data
                    state_pub = node.create_publisher(JointState, args.state_topic, qos_profile_sensor_data)
                if args.viewer_bridge_port:
                    bridge = subprocess.Popen(['bash', str(simulator_dir/'start_ros.sh'), '--bridge',
                        '--port', str(args.viewer_bridge_port), '--allow-origin', 'http://127.0.0.1:8877',
                        '--allow-origin', 'http://localhost:8877'], stdout=sys.stderr, stderr=sys.stderr,
                        start_new_session=True)
                    if not wait_for_viewer_service(bridge, args.viewer_bridge_port, stop_event, enable_http=True):
                        return 0
                if args.viewer_observe_port:
                    # 表示と回避計算で共通のROS_DOMAIN_ID。操作ノードを含まない既存Gatewayの利用。
                    observer = subprocess.Popen(['ros2', 'run', 'topo_fuzzy_viewer', 'viewer_ws_gateway_node',
                        '--ros-args', '-r', '__node:=simulation_viewer_gateway', '-p',
                        'port:='+str(args.viewer_observe_port)], stdout=sys.stderr, stderr=sys.stderr,
                        start_new_session=True)
                    if not wait_for_viewer_service(observer, args.viewer_observe_port, stop_event):
                        return 0
                initial = scene.robot.state()
                target = scene.robot.validate(goal or {}, base_pose=initial)
                emit(dict(type='start', model=args.model, avoidance=args.avoidance, initial_joints=initial,
                          goal_joints=target, bag=str(args.bag) if args.bag else None,
                          viewer_url=('http://127.0.0.1:8877/?model='+args.model+'&bridge=http://127.0.0.1:'
                                      +str(args.viewer_bridge_port)+'&state_source=simulation&joint_topic='+(scene.avoidance.runtime.status()['joint_state_topic'] if args.avoidance == 'gng' else args.state_topic)
                                      +('&observe=ws://127.0.0.1:'+str(args.viewer_observe_port)+'/observe' if observer is not None else ''))
                                      if bridge is not None else None))
                started = time.monotonic()
                next_report_sec = started
                while not stop_event.is_set():
                    now = time.monotonic()
                    if args.duration and now-started >= args.duration:
                        break
                    if bag is not None and bag.poll() is not None:
                        bag_log.seek(0)
                        raise RuntimeError('bag再生が終了しました: '+bag_log.read()[-2000:])
                    if bridge is not None and bridge.poll() is not None:
                        raise RuntimeError('Viewer受信用ROSブリッジが終了しました。ポート競合・依存ライブラリを確認してください')
                    if observer is not None and observer.poll() is not None:
                        raise RuntimeError('Topics表示サーバーが終了しました。ポート競合・依存ライブラリを確認してください')
                    if controller is not None and controller.poll() is not None:
                        raise RuntimeError('非停止実験の回避ノードが終了しました')
                    if goal is not None and not has_goal:
                        scene.set_goal(target)
                        has_goal = True
                    if task is not None:
                        task_goal = task.update(scene)
                        if goal_pub is None:
                            scene.set_goal(task_goal)
                        elif task_goal != last_task_goal or now-last_task_goal_sec >= .2:
                            goal_pub.publish(JointState(name=list(task_goal), position=list(task_goal.values())))
                            last_task_goal = task_goal
                            last_task_goal_sec = now
                    result = session.step() if session is not None else scene.step()
                    if controller is not None:
                        result['avoidance'] = dict(latest_avoidance)
                    if task is not None:
                        result['task'] = task.status
                    if state_pub is not None:
                        message = JointState()
                        message.header.stamp = node.get_clock().now().to_msg()
                        message.name = list(result['joints'])
                        message.position = [result['joints'][name] for name in message.name]
                        message.velocity = [result['motion']['velocity'][name] for name in message.name]
                        message.effort = [result['motion']['effort'][name] for name in message.name]
                        state_pub.publish(message)
                    num_frames += 1
                    is_ready = is_ready or bool(getattr(scene.avoidance, 'is_ready', False))
                    phase = (result.get('avoidance') or {}).get('phase', 'none')
                    phases[phase] += 1
                    motion_deg = max(abs(value-initial[name])*180/math.pi for name, value in result['joints'].items()
                                     if name.startswith(('L_joint', 'R_joint')))
                    max_motion_deg = max(max_motion_deg, motion_deg)
                    if now >= next_report_sec:
                        emit(dict(result, type='state', elapsed_sec=now-started, max_joint_motion_deg=max_motion_deg))
                        next_report_sec = now+args.report_sec
                    # 1呼出し5ステップ×2 ms。処理超過時の遅延蓄積・追付き連打なし。
                    stop_event.wait(max(0., .01-(time.monotonic()-now)))
                if errors:
                    raise errors[0]
                emit(dict(type='summary', is_ready=is_ready, num_frames=num_frames, elapsed_sec=time.monotonic()-started,
                          simulation_sec=float(scene.data.time), max_joint_motion_deg=max_motion_deg,
                          phases=dict(phases), joints=scene.robot.state(),
                          task=task.status if task is not None else None,
                          avoidance=dict(latest_avoidance) if controller is not None else scene.avoidance.status if scene.avoidance else None))
                has_round_trips = task is None or task.num_arrivals//2 >= args.min_round_trips
                return 0 if is_ready and max_motion_deg >= args.min_motion_deg and has_round_trips else 3
    finally:
        try:
            if scene is not None and scene.avoidance is not None and hasattr(scene.avoidance, 'close'):
                scene.avoidance.close()
        finally:
            stop_process(bridge)
            stop_process(observer)
            stop_process(controller)
            stop_process(bag)
            if session is not None:
                session.close().result(10)
            executor.shutdown()
            thread.join()
            node.destroy_node()
            rclpy.shutdown()
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def main():
    args = parse_arguments()
    if os.environ.get('TOPO_ROS_SETUP_READY') != '1':
        os.execv('/bin/bash', ['bash', str(simulator_dir/'start_ros.sh'), '--simulate', *sys.argv[1:]])
    try:
        chase_config = read_chase_config(args) if args.scenario=='chasing_sphere' else None
        stream = args.output.open('x') if args.output else sys.stdout
    except (OSError,ValueError) as error:
        print(json.dumps(dict(type='error', message=str(error)), ensure_ascii=False), file=sys.stderr)
        return 2
    try:
        def emit(value):
            print(json.dumps(value, ensure_ascii=False, allow_nan=False), file=stream, flush=True)
        try:
            return run(args, emit, chase_config)
        except (ImportError, ValueError, RuntimeError, OSError) as error:
            emit(dict(type='error', message=str(error)))
            print(str(error), file=sys.stderr)
            return 2
    finally:
        if stream is not sys.stdout:
            stream.close()


if __name__ == '__main__':
    sys.exit(main())
