#!/usr/bin/env python3
"""実機の小動作・Gazebo追従と、両系統への優先停止要求。"""
import argparse
import json
import os
import select
import signal
import time

from dual_arm_control_keyboard import terminal_key_decoder, control_request
from gazebo_stop_keyboard import terminal_input, stop_request, status_label, is_fresh_age


gazebo_key_help = 'A:回避/保持  B:停止解除→保持  Space:両方停止  Ctrl+C:停止して終了'


def gazebo_status_label(latest, received, now):
    """Gazeboの短縮状態表示。停止要求・実測確認・入力失効の区別。"""
    mode = latest.get('sim', {}).get('mode')
    if not is_fresh_age(now-received.get('sim', -1e9)):
        mode = '未受信/失効'
    else:
        mode = {'avoidance': 'avoid', 'leader': 'follow', 'switching': '切替中',
                'stopped': 'stop'}.get(mode, mode or '不明')
    safety = status_label(latest.get('safety', {}), now-received.get('safety', -1e9))
    safety = {
        '実測停止: 確認済み': '停止解除待ち(B) | 静止=確認済み',
        '停止ラッチ: ON / 実測停止: 未確認': '停止解除待ち(B) | 静止=未確認',
        '停止ラッチ: OFF': '',
        '停止状態: 未受信・失効': '停止状態=未受信/失効',
        '停止状態: 診断形式不正': '停止状態=不正',
    }[safety]
    state = latest.get('demo', {}).get('state')
    if not is_fresh_age(now-received.get('demo', -1e9)):
        state = '未受信/失効'
    else:
        state = {'waiting': '準備中', 'idle': '開始待ち', 'running': '実行中',
                 'completed': '完了', 'stopped': '開始待ち', 'fault': '異常'}.get(state, state or '不明')
        if latest.get('demo', {}).get('state') == 'running' and latest['demo'].get('phase') == 'obstacle_wait':
            state = ('障害物待ち（離れたら自動再開）'
                     if latest['demo'].get('enable_obstacle_auto_resume') is True
                     else '障害物待ち（Aで保持へ）')
    label = ' | '.join(part for part in (f'gazebo | mode={mode}', safety, f'回避={state}') if part)
    if mode == 'stop':
        # 停止理由も同じ状態行へ集約。改行を含む詳細ログの端末展開なし
        detail = ' '.join(str(latest.get('sim', {}).get('detail', '')).split())
        detail = detail.replace('切替操作の拒否: switch_start_demo / ', '回避開始拒否: ')
        if detail:
            label += ' | 理由='+detail[:120]
        operation = latest.get('sim_operation', '')
        if operation:
            label += ' | B='+operation
    return label


def gazebo_reset_feedback(message):
    """停止解除の操作応答を状態行へ載せる短縮表現。詳細ログの追加なし。"""
    prefix = 'sim_reset: '
    if not message.startswith(prefix):
        return None
    return ' '.join(message[len(prefix):].split())[:180]


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import Empty, String
    from std_srvs.srv import SetBool, Trigger

    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--sim-namespace', required=True)
    parser.add_argument('--tty-path', required=True)
    args = parser.parse_args(remove_ros_args()[1:])
    stream = os.fdopen(os.open(args.tty_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK), 'r+b', buffering=0)
    def emit(message):
        node.get_logger().info(message)
    def emit_status(message):
        try:
            os.write(stream.fileno(), ('\r\n'+message+'\r\n').encode())
        except OSError:
            pass
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('dynamixel_sim_keyboard_'+str(os.getpid()))
    hw, sim = '/'+args.namespace.strip('/'), '/'+args.sim_namespace.strip('/')
    latest, received = {}, {}
    def on_status(message, key):
        try:
            latest[key] = json.loads(message.data)
            received[key] = time.monotonic()
        except ValueError:
            pass
    node.create_subscription(String, hw+'/status', lambda msg: on_status(msg, 'hardware'), 1)
    node.create_subscription(String, sim+'/control/status', lambda msg: on_status(msg, 'sim'), 1)
    node.create_subscription(String, sim+'/safety/status', lambda msg: on_status(msg, 'safety'), 1)
    node.create_subscription(String, sim+'/avoidance/status', lambda msg: on_status(msg, 'demo'), 1)
    clients = {name: node.create_client(kind, topic) for name, kind, topic in [
        ('enable', SetBool, hw+'/enable'), ('follow', SetBool, hw+'/follow'),
        ('positive', Trigger, hw+'/jog_positive'), ('negative', Trigger, hw+'/jog_negative'),
        ('reset', Trigger, hw+'/reset'), ('sim_reset', Trigger, sim+'/control/reset'),
        ('avoidance', SetBool, sim+'/control/avoidance')]}
    requests = [stop_request(node.create_client(Trigger, topic), Trigger.Request, emit)
                for topic in (hw+'/stop', sim+'/control/stop')]
    torque_request = stop_request(node.create_client(Trigger, hw+'/torque_off'), Trigger.Request,
                                 lambda message: emit('トルクOFF: '+message))
    all_requests = requests+[torque_request]
    heartbeats = [node.create_publisher(Empty, topic, 1) for topic in (hw+'/heartbeat', sim+'/control/heartbeat')]
    def emit_operation(message):
        emit(message)
        feedback = gazebo_reset_feedback(message)
        if feedback is not None:
            latest['sim_operation'] = feedback
    operation = control_request(emit_operation)
    exit_state = {'is_requested': False}
    handlers = {kind: signal.signal(kind, lambda *_: exit_state.update(is_requested=True))
                for kind in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    decoder = terminal_key_decoder({b'h': 'enable', b'j': 'positive', b'k': 'negative',
                                    b'f': 'follow', b'r': 'reset', b'b': 'sim_reset', b'a': 'avoidance', b'e': 'torque_off'})
    is_exiting = False
    exit_deadline = next_heartbeat = 0.
    previous = None
    previous_sim = None
    emit('H: 実機出力ON／停止（保持目標読返し後の対象IDトルクONを含む）')
    emit('J/K: 実機の単関節±小角度 / F: 実機のGazebo追従ON/OFF / R: 実機停止解除・出力OFF')
    emit('gazebo操作 | '+gazebo_key_help)
    emit('E: 選択IDのトルクOFF＋指令停止ラッチ（出力許可時のみ）。脱力・落下に備えた腕の支持が必要')
    emit('PC/USB故障時の独立非常停止とは別のソフト停止。実測停止表示を確認')
    try:
        with terminal_input(stream) as descriptor:
            while rclpy.ok():
                action = 'quit' if exit_state['is_requested'] and not is_exiting else None
                if not is_exiting and select.select([descriptor], [], [], 0)[0]:
                    action = decoder.read_action(os.read(descriptor, 1))
                if action == 'torque_off':
                    operation.cancel()
                    torque_request.begin()
                    requests[1].begin()
                elif action in ('stop', 'quit'):
                    latest.pop('sim_operation', None)
                    operation.cancel()
                    for request in requests:
                        request.begin()
                    if action == 'quit':
                        is_exiting, exit_deadline = True, time.monotonic()+3.
                elif action and not any(request.is_pending for request in all_requests):
                    state = latest.get('hardware', {})
                    if action not in ('sim_reset', 'avoidance') and time.monotonic()-received.get('hardware', -1e9) > .5:
                        emit('実機状態が未受信・失効。停止キーだけ利用可能')
                    else:
                        request = Trigger.Request()
                        if action == 'enable':
                            request = SetBool.Request(data=state.get('mode') == 'off')
                        elif action == 'follow':
                            request = SetBool.Request(data=state.get('mode') != 'follow')
                        elif action == 'avoidance':
                            if time.monotonic()-received.get('sim', -1e9) > .5:
                                emit('Gazebo制御状態が未受信・失効')
                                continue
                            if (time.monotonic()-received.get('demo', -1e9) > .5 or
                                    latest.get('demo', {}).get('state') not in ('idle', 'running', 'completed', 'stopped')):
                                emit('Gazebo回避ノードの準備待ち・入力失効。Aを再操作してください')
                                continue
                            request = SetBool.Request(data=latest['sim'].get('mode') != 'avoidance')
                        operation.begin(clients[action], request, action)
                for request in all_requests:
                    request.poll()
                operation.poll()
                if time.monotonic() >= next_heartbeat:
                    for publisher in heartbeats:
                        publisher.publish(Empty())
                    next_heartbeat = time.monotonic()+.1
                rclpy.spin_once(node, timeout_sec=.02)
                state = latest.get('hardware', {})
                label = (state.get('mode'), state.get('detail'), state.get('ids'), state.get('is_stopped'),
                         time.monotonic()-received.get('hardware', -1e9) < .5, state.get('has_torque_off_report'))
                if label != previous:
                    emit(f'実機 ID={label[2]} mode={label[0]} / {label[1]} / 実測停止={label[3] if label[4] else "未確認・状態失効"}')
                    if state.get('is_torque_off_latched'):
                        emit(f'トルクOFF報告={label[5] if label[4] else "未確認・状態失効"}（電源遮断の確認とは別）')
                    previous = label
                sim_label = gazebo_status_label(latest, received, time.monotonic())
                if latest.get('sim', {}).get('mode') != 'stopped':
                    latest.pop('sim_operation', None)
                if sim_label != previous_sim:
                    emit_status(sim_label+'\r\n  '+gazebo_key_help)
                    previous_sim = sim_label
                if is_exiting and (time.monotonic() >= exit_deadline or
                        (not any(request.is_pending for request in all_requests) and state.get('is_stopped') and label[4])):
                    emit('実測停止確認済み' if state.get('is_stopped') and label[4] else '実測停止未確認。独立停止手段で確認')
                    break
    finally:
        if not is_exiting and rclpy.ok():
            for request in requests:
                request.begin()
            deadline = time.monotonic()+2.
            while rclpy.ok() and time.monotonic() < deadline and any(request.is_pending for request in all_requests):
                for request in all_requests:
                    request.poll()
                rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()
        for kind, handler in handlers.items():
            signal.signal(kind, handler)
        stream.close()


if __name__ == '__main__':
    main()
