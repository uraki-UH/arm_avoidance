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
    parser.add_argument('--avoidance', choices=('gng', 'none', 'oscbf'), default='gng')
    parser.add_argument('--gng-strategy', choices=('graph', 'avoidance_first', 'vrm_predictive'), default='graph',
                        help='GNG方策。既定のノード列／回避優先／予測・不一致適応')
    parser.add_argument('--roi-visibility', choices=('fov_shadow', 'ray', 'strict', 'legacy'), default='fov_shadow',
                        help='視野内free＋遮蔽／観測光線／観測制約なし。strictは旧ray指定の互換名')
    parser.add_argument('--pose', choices=('forward', 'home', 'zero'), default='forward')
    parser.add_argument('--pose-file', type=Path, help='初期関節角のJSON辞書 [rad]。プリセットへの部分上書き')
    parser.add_argument('--goal-file', type=Path, help='復帰目標のJSON辞書 [rad]。初期姿勢への部分上書き')
    parser.add_argument('--duration', type=nonnegative, help='実行時間 [sec]。ROS入力の既定値30、追跡球は設定JSON。0はROS入力のみ')
    parser.add_argument('--report-sec', type=nonnegative, default=1., help='JSONL出力周期 [sec]')
    parser.add_argument('--output', type=Path, help='JSONL保存先。既存ファイルへの上書き禁止')
    parser.add_argument('--viewer-bridge-port', type=int, help='実測姿勢のViewer受信用ROSブリッジHTTPポート')
    parser.add_argument('--viewer-observe-port', type=int, help='同じROSドメインのTopics・点群・Tmap表示サーバーポート')
    parser.add_argument('--domain-id', type=int, choices=range(233), metavar='0..232')
    parser.add_argument('--points-topic', default='/livox/lidar')
    parser.add_argument('--roi-topic', default='/topo_dual_arm_max_long/roi_voxels')
    parser.add_argument('--recorded-self-topic', default='/topo_dual_arm_max_long/self_voxel')
    parser.add_argument('--bag', type=Path, help='CLI所有のbag再生。省略時は既存ROS入力を使用')
    parser.add_argument('--start-offset', type=nonnegative, default=0., help='bag開始位置 [sec]')
    parser.add_argument('--rate', type=nonnegative, default=1., help='bag再生速度')
    parser.add_argument('--min-motion-deg', type=nonnegative, default=0., help='終了時の腕関節変化の検証下限 [deg]')
    args = parser.parse_args()
    if args.scenario == 'chasing_sphere':
        if args.model != 'long' or args.avoidance != 'gng' or args.bag or args.viewer_bridge_port or args.viewer_observe_port or args.pose_file or args.goal_file:
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
    if args.bag and args.avoidance != 'gng':
        parser.error('--bagはGNG回避用です')
    if args.gng_strategy != 'graph' and args.avoidance != 'gng':
        parser.error('--gng-strategyはGNG回避用です')
    if args.viewer_bridge_port is not None and not 1024 <= args.viewer_bridge_port <= 65534:
        parser.error('--viewer-bridge-portは1024～65534です')
    if args.viewer_bridge_port and args.avoidance != 'gng':
        parser.error('--viewer-bridge-portはGNGの実測JointState出力用です')
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
    pose = {name: math.radians(angle) for name, angle in presets[args.pose].items()}
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
    scene, bag, bridge, observer = None, None, None, None
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
                    topics = (args.points_topic, args.roi_topic, args.recorded_self_topic)
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
                settings = dict(mode=args.avoidance)
                if args.avoidance == 'gng':
                    settings.update(gng_strategy=args.gng_strategy, enable_roi_visibility=args.roi_visibility != 'legacy',
                                    visibility_model='ray' if args.roi_visibility in ('ray', 'strict') else 'fov_shadow',
                                    points_topic=args.points_topic,
                                    roi_topic=args.roi_topic, recorded_self_topic=args.recorded_self_topic)
                if args.bag:
                    settings.update(recorded_tf_topic=prefix+'/tf', recorded_static_tf_topic=prefix+'/tf_static')
                scene = PhysicsScene([], dict(model=args.model, pose=pose, position=[0, 0, 0], quaternion=[0, 0, 0, 1]),
                                     settings, node)
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
                                      +str(args.viewer_bridge_port)+'&state_source=simulation&joint_topic='+scene.avoidance.runtime.status()['joint_state_topic']
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
                    if goal is not None and not has_goal:
                        if scene.avoidance and hasattr(scene.avoidance, 'set_targets'):
                            scene.avoidance.set_targets(target)
                        else:
                            scene.robot.set_targets(target)
                        has_goal = True
                    result = scene.step()
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
                          avoidance=scene.avoidance.status if scene.avoidance else None))
                return 0 if is_ready and max_motion_deg >= args.min_motion_deg else 3
    finally:
        try:
            if scene is not None and scene.avoidance is not None and hasattr(scene.avoidance, 'close'):
                scene.avoidance.close()
        finally:
            stop_process(bridge)
            stop_process(observer)
            stop_process(bag)
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
