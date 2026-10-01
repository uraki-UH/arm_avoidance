"""複数ロボットの片側終了・再接続キャッシュ・同名再起動の回帰検証。"""

import json
import os
import subprocess
import sys
import tempfile
import time

from test_stream_restart import Client, stop


def publish(tag, generation):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('robot_lifecycle_' + tag)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, '/viewer/internal/stream/robot/description', qos)
    publisher.publish(String(data=json.dumps({
        'type': 'stream.robot.description', 'tag': tag,
        'robot': {'generation': generation, 'jointNames': [], 'jointValues': []}})))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    env = dict(os.environ, ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1')
    port = 19094
    processes, clients, robots = [], [], {}
    deleted = set()

    def observe(value):
        if not isinstance(value, dict):
            return
        if value.get('type') == 'stream.robot.description':
            robots[value['tag']] = value['robot']['generation']
        elif value.get('type') == 'stream.robot.delete':
            robots.pop(value['tag'], None)
            deleted.add(value['tag'])

    with tempfile.TemporaryFile() as log:
        def start(args):
            process = subprocess.Popen(args, env=env, stdout=log, stderr=log)
            processes.append(process)
            return process

        def start_robot(tag, generation):
            return start([sys.executable, __file__, '--publisher', tag, str(generation)])

        def connect():
            end = time.monotonic() + 15
            while time.monotonic() < end:
                try:
                    client = Client(port)
                    clients.append(client)
                    return client
                except ConnectionRefusedError:
                    time.sleep(.1)
            raise AssertionError('試験用Gateway接続の時間超過')

        def wait(client, condition):
            client.until(lambda value: (observe(value), condition(value))[1])

        try:
            start([os.environ.get('GATEWAY_BINARY',
                   '/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node'),
                   '--ros-args', '-p', f'port:={port}'])
            real_robot = start_robot('ToPoDualArm', 1)
            sim_robot = start_robot('sim_ToPoDualArm', 1)
            client = connect()
            wait(client, lambda _: len(robots) == 2)
            stop(sim_robot)
            wait(client, lambda _: 'sim_ToPoDualArm' in deleted)
            assert robots == {'ToPoDualArm': 1} and 'ToPoDualArm' not in deleted
            assert real_robot.poll() is None

            # 再接続時にも、停止したロボットのdescription復活なし。
            robots.clear()
            second = connect()
            wait(second, lambda _: 'ToPoDualArm' in robots)
            second.send({'id': 'cache_checked', 'method': 'sources.list'})
            wait(second, lambda v: isinstance(v, dict) and v.get('id') == 'cache_checked')
            assert robots == {'ToPoDualArm': 1}

            sim_robot = start_robot('sim_ToPoDualArm', 2)
            wait(second, lambda _: robots.get('sim_ToPoDualArm') == 2)
            assert robots.get('ToPoDualArm') == 1
            deleted.clear()
            stop(sim_robot)
            wait(second, lambda _: 'sim_ToPoDualArm' in deleted)
            stop(real_robot)
            wait(second, lambda _: 'ToPoDualArm' in deleted)
            assert not robots
            print('PASS: SIGINTで片側だけ削除、再接続キャッシュ削除、同名再起動、全終了')
        except BaseException:
            log.seek(0)
            print(log.read().decode(errors='replace'), file=sys.stderr)
            raise
        finally:
            for client in clients:
                client.sock.close()
            for process in reversed(processes):
                stop(process)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--publisher':
        publish(sys.argv[2], int(sys.argv[3]))
    else:
        main()
