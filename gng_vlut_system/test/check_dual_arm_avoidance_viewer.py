#!/usr/bin/env python3
"""起動済み回避デモのViewerストリーム受信検証。試験gatewayのみ起動・終了。"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test'))
from test_marker_stream import RenderClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--namespace', default='sim_topo_dual_arm_max')
    parser.add_argument('--enable-gng-lidar', action='store_true')
    parser.add_argument('--expected-nodes', type=int, default=10000)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    client = None
    with (args.output/'viewer.log').open('w') as log:
        command = ['/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node', '--ros-args', '-p', f'port:={port}']
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        print('started_gateway', process.pid, command, flush=True)
        try:
            deadline = time.monotonic()+20
            while client is None and time.monotonic() < deadline:
                try:
                    client = RenderClient(port)
                except ConnectionRefusedError:
                    time.sleep(.1)
            assert client is not None
            description = client.until(lambda value: isinstance(value, dict) and
                                        value.get('type') == 'stream.robot.description', sec=20)
            pose = client.until(lambda value: isinstance(value, dict) and
                                value.get('type') == 'stream.robot.pose', sec=20)
            topic = f'/{args.namespace}/avoidance/markers'
            deadline = time.monotonic()+20
            while True:
                client.send({'id': 'list', 'method': 'sources.list'})
                sources = client.until(lambda value: isinstance(value, dict) and value.get('id') == 'list')
                if topic in [source['id'] for source in sources['result']['sources']]:
                    break
                assert time.monotonic() < deadline, sources
                time.sleep(.1)
            client.send({'id': 'on', 'method': 'sources.setActive', 'params': {'sourceId': topic, 'active': True}})
            client.until(lambda value: isinstance(value, dict) and value.get('id') == 'on')
            markers = client.until(lambda value: isinstance(value, dict) and
                                   value.get('type') == 'stream.marker_array', sec=20)
            assert len(markers['markers']) == 7
            assert {item['type'] for item in markers['markers']} == {'sphere', 'cylinder', 'line_list', 'line_strip', 'text'}
            assert description['tag'] == args.namespace and pose['tag'] == args.namespace
            report = {'result': 'passed', 'robot': args.namespace, 'marker_topic': topic,
                      'marker_types': [item['type'] for item in markers['markers']]}
            if args.enable_gng_lidar:
                topics = [f'/{args.namespace}/'+name for name in ('lidar_points', 'Tmap_static', 'plan_Tmap')]
                for source in topics:
                    client.send({'id': source, 'method': 'sources.setActive',
                                 'params': {'sourceId': source, 'active': True}})
                received = set()
                def has_streams(value):
                    if isinstance(value, bytes) and value[:4] == b'TMG1':
                        size = struct.unpack_from('<I', value, 8)[0]
                        source = value[36:36+size].decode()
                        frame_size = struct.unpack_from('<I', value, 12)[0]
                        frame = value[36+size:36+size+frame_size].decode()
                        assert frame == args.namespace+'/base_link', frame
                        if source == topics[1]:
                            assert struct.unpack_from('<I', value, 20)[0] == args.expected_nodes, (source, struct.unpack_from('<9I', value), args.expected_nodes)
                            received.add(source)
                        elif source == topics[2]:
                            received.add(source)
                    elif isinstance(value, bytes) and value:
                        body = value[1+value[0]:]
                        if len(body) >= 32 and struct.unpack_from('<I', body)[0] == 0x50434458:
                            received.add(topics[0])
                    return set(topics) <= received
                try:
                    client.until(has_streams, sec=180)
                except TimeoutError as error:
                    raise AssertionError(f'未受信: {set(topics)-received}') from error
                report['streams'] = sorted(received)
            (args.output/'viewer_report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(report, flush=True)
        finally:
            if client is not None:
                client.sock.close()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            print('stopped_gateway', process.pid, flush=True)


if __name__ == '__main__':
    main()
