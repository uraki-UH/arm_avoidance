"""隔離ROSドメインでのメッシュ更新通知検証。原本・既存ノードの変更なし。"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-path', type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID'] == '0':
        parser.error('既存環境と異なるROS_DOMAIN_IDの明示が必要')
    source = Path(__file__).resolve().parents[2]/'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'
    rclpy.init()
    listener = rclpy.create_node('mesh_revision_check_listener')
    received = {}
    def on_description(message):
        payload = json.loads(message.data)
        received[payload['tag']] = payload['robot']['urdf']
    listener.create_subscription(String, '/mesh_revision_check/description', on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        with tempfile.TemporaryDirectory(prefix='robot-mesh-revision-') as temporary_dir:
            folder = Path(temporary_dir)
            cover = folder/'cover.stl'
            shutil.copyfile(source.parent/'meshes/chest_lidar_color_0.stl', cover)
            text = source.read_text(encoding='utf-8-sig')
            def resolve_mesh(match):
                path = source.parent/match[1]
                if path.name == 'chest_lidar_color_0.stl':
                    path = cover
                return f'filename="file://{path}"'
            (folder/'robot.urdf').write_text(re.sub(r'filename="(meshes/[^\"]+)"', resolve_mesh, text))
            revisions = []
            for idx in range(3):
                if idx == 2:
                    state = cover.stat()
                    os.utime(cover, ns=(state.st_atime_ns, state.st_mtime_ns + 1_000_000_000))
                tag = f'mesh_revision_check_{idx}'
                command = [str(args.node_path), '--ros-args', '-r', f'__node:={tag}',
                           '-p', f'robot_name:={tag}', '-p', f'urdf_path:={folder}/robot.urdf',
                           '-p', 'stream_topic:=/mesh_revision_check']
                with (folder/f'node_{idx}.log').open('w+') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                    try:
                        deadline = time.monotonic() + 25
                        while tag not in received and time.monotonic() < deadline and process.poll() is None:
                            rclpy.spin_once(listener, timeout_sec=0.1)
                        if tag not in received:
                            log.seek(0)
                            raise AssertionError('description未受信: '+log.read())
                        revision = re.search(r'viewer_mesh_revision: (\d+)', received[tag])
                        assert revision, 'メッシュ更新情報の欠落'
                        revisions.append(revision[1])
                    finally:
                        if process.poll() is None:
                            process.send_signal(signal.SIGINT)
                            try:
                                process.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait(timeout=5)
            assert revisions[0] == revisions[1], revisions
            assert revisions[1] != revisions[2], revisions
            print('PASS: 原本未変更の再起動で同一revision、メッシュ更新後のみrevision変更')
    finally:
        listener.destroy_node()
        rclpy.shutdown()
        print('試験起動: robot_viewer_bridge_node --ros-args（隔離ドメイン）。全試験ノード停止済み')


if __name__ == '__main__':
    main()
