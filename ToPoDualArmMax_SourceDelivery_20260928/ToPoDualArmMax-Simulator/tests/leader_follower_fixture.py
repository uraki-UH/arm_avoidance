"""隔離ROSとMuJoCoのブラウザ試験用ブリッジ。実機への出力なし。"""
import argparse
import os
import sys
from pathlib import Path
from types import SimpleNamespace
import threading

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'integrations/ros2'))
from joint_stream import start_joint_stream


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '98' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('隔離domain98・localhost限定が必要です')
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True, type=int)
    parser.add_argument('--origin', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('browser_leader_fixture')
    exchange = SimpleNamespace(joint_names={'long': {'R_joint1', 'L_joint7'}},
                               joints=node.create_publisher(JointState, '/leader/joint_states', 1))
    stop = None
    try:
        stop = start_joint_stream(node, exchange, '127.0.0.1', args.port, {args.origin})
        exit_state = {'is_requested': False}
        def watch_input():
            sys.stdin.read()
            exit_state['is_requested'] = True
        threading.Thread(target=watch_input, daemon=True).start()
        while rclpy.ok() and not exit_state['is_requested']:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if stop is not None:
            stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
