"""隔離ROSでの実ノード計測。試験プロセスのみ終了、通常配布先への書込みなし。"""
import argparse
from array import array
import csv
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
from voxel_msgs.msg import Voxel
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml

root = Path(__file__).resolve().parents[2]
output = root / 'artifacts/shared_voxel_cost_20260926' / os.environ.get('SHARED_COST_VARIANT', '')

def stop(process):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            break
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            pass
    assert process.poll() is not None
    print('STOP', process.pid, process.returncode, flush=True)

def process_usage(pid):
    fields = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
    cpu_sec = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    rss_mib = int(Path(f'/proc/{pid}/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE') / 2**20
    return cpu_sec, rss_mib

def describe(values):
    return {'num': len(values), 'mean': float(np.mean(values)),
            'p50': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'max': float(np.max(values))} if values else {'num': 0}

def source(num):
    rng = np.random.default_rng(260926)
    points = np.empty((num, 4), dtype='<f4')
    points[:, :2] = rng.uniform(-8., 8., (num, 2))
    points[:, 2] = rng.uniform(-.2, 2.5, num)
    # 道路平面70%、壁面15%、残り立体点群の固定分布。
    points[:num*7//10, 2] = rng.normal(0., .006, num*7//10)
    points[num*7//10:num*85//100, 1] = 6. + rng.normal(0., .006, num*15//100)
    points[:, 3] = rng.uniform(0, 255, num)
    msg = PointCloud2(); msg.header.frame_id = 'sensor'
    msg.height, msg.width, msg.point_step = 1, num, 16
    msg.row_step = num*16
    msg.fields = [PointField(name=name, offset=idx*4, datatype=7, count=1)
                  for idx, name in enumerate(('x', 'y', 'z', 'intensity'))]
    msg.data = points.tobytes()
    topo = TopologicalMap(); topo.header.frame_id = 'world'
    for p in points[np.linspace(0, num-1, 5000, dtype=int)]:
        item = TopologicalNode()
        item.pos.x = .45 + math.cos(.6)*float(p[0]) - math.sin(.6)*float(p[1])
        item.pos.y = -.2 + math.sin(.6)*float(p[0]) + math.cos(.6)*float(p[1])
        item.pos.z = .3 + float(p[2]); item.label = len(topo.nodes) % 4
        topo.nodes.append(item)
    return msg, topo

def run_case(node, case, trial, args):
    num, mode, narrow_fvg, wide_roi = case
    has_fvg = mode != 'world'
    has_tmap = mode in ('tmap', 'markers')
    enable_markers = mode == 'markers'
    name = f'{num}_{mode}_{"narrow" if narrow_fvg else "wide"}_{"wide_roi" if wide_roi else "local_roi"}_{trial}'
    if args.installed:
        name = 'installed_' + name
    elif args.threads != 2:
        name = f'threads{args.threads}_' + name
    if args.case_prefix:
        name = args.case_prefix + '_' + name
    trace = output / f'{name}.csv'
    world = {'input_topic': '/cost/points', 'output_topic': '/cost/roi',
        'world_bucket_topic': '/cost/buckets', 'world_frame_id': 'world', 'target_frame_id': 'roi',
        'shared_point_store': 'cost' if has_fvg else '', 'bucket_size': .2, 'voxel_size': .02,
        'enable_world_bucket_publish': enable_markers, 'allow_unconnected_source_as_world': False,
        'min_reachability_x': -1.8, 'max_reachability_x': .8,
        'min_reachability_y': -.455, 'max_reachability_y': .355,
        'min_reachability_z': 0., 'max_reachability_z': 2.95}
    settings = {'shared_point_store': 'cost', 'input_topological_map_topic': '/cost/map',
        'marker_array_topic': '/cost/markers', 'voxel_centers_topic': '/cost/centers',
        'publish_voxel_centers': False, 'print_processing_time': False, 'update_rate_hz': 10.,
        'use_exclusion_box': False, 'voxel_size_x': .1, 'voxel_size_y': .1, 'voxel_size_z': .1}
    for axis in 'xyz':
        world[f'reachability_margin_{axis}'] = 0.
        lo, hi = (-12., 12.) if axis != 'z' else (-1., 4.)
        if wide_roi:
            world[f'min_reachability_{axis}'], world[f'max_reachability_{axis}'] = lo, hi
        if narrow_fvg:
            lo, hi = world[f'min_reachability_{axis}'], world[f'max_reachability_{axis}']
        settings.update({f'range_min_{axis}': lo, f'range_max_{axis}': hi})
    params = output / f'{name}.yaml'
    params.write_text(yaml.safe_dump({'world_index_to_voxel_node': {'ros__parameters': world},
                                    'voxel_grid_node': {'ros__parameters': settings}}))
    publisher = node.create_publisher(PointCloud2, '/cost/points', 1)
    map_pub = node.create_publisher(TopologicalMap, '/cost/map', 1)
    received = {}; marker_received = {}; sizes = {'markers': [], 'buckets': []}
    roi_sub = node.create_subscription(Voxel, '/cost/roi',
        lambda msg: received.setdefault(msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec,
                                       (time.monotonic_ns(), len(msg.data))), qos_profile_sensor_data)
    visual_subs = []
    def visual_callback(data, key):
        now_ns = time.monotonic_ns()
        sizes[key].append((now_ns, len(data)))
        if key == 'markers':
            # HumbleのCDR: 4byte encapsulation、Marker配列長、先頭headerのstamp。
            sec, nsec = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 8)
            marker_received.setdefault(sec*1000000000+nsec, now_ns)
    if enable_markers:
        for kind, topic, key in [(MarkerArray, '/cost/markers', 'markers'), (Voxel, '/cost/buckets', 'buckets')]:
            visual_subs.append(node.create_subscription(kind, topic,
                lambda data, key=key: visual_callback(data, key),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), raw=True))
    msg, topo = source(num)
    if args.moving:
        base_points = np.frombuffer(msg.data, dtype='<f4').reshape(-1, 4).copy()
        moved_points = base_points.copy()
        base_nodes = [(item.pos.x, item.pos.y, item.pos.z) for item in topo.nodes]
    command = [str(output/'build/shared_voxel_cost'), '--ros-args', '--params-file', str(params),
               '--log-level', 'warn']
    if args.installed:
        command = ['ros2', 'launch', 'fuzzy_voxel_grid', 'shared_world_voxel.launch.py',
            'input_topic:=/cost/points', 'target_frame:=roi',
            'world_params_file:=' + str(params), 'fvg_params_file:=' + str(params)]
    env = dict(os.environ, SHARED_BENCH_TRACE=str(trace), SHARED_BENCH_FVG=str(int(has_fvg)),
               SHARED_BENCH_MARKERS=str(int(enable_markers)), SHARED_BENCH_THREADS=str(args.threads))
    process = None
    start_ns = end_ns = 0
    sent = {}; peak_rss = 0.
    try:
        with (output/f'{name}.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=log, env=env, start_new_session=True)
            print('START', process.pid, name, command, flush=True)
            deadline = time.monotonic() + 15
            while publisher.get_subscription_count() != 1 or (has_fvg and map_pub.get_subscription_count() != 1):
                assert process.poll() is None
                if time.monotonic() > deadline:
                    raise RuntimeError('DDS接続の待機期限')
                rclpy.spin_once(node, timeout_sec=.02)
            ready = time.monotonic() + .4
            while time.monotonic() < ready:
                rclpy.spin_once(node, timeout_sec=.02)
            target_pid = process.pid
            if args.installed:
                children = Path(f'/proc/{process.pid}/task/{process.pid}/children').read_text().split()
                candidates = [int(pid) for pid in children
                              if b'component_container_mt' in Path(f'/proc/{pid}/cmdline').read_bytes()]
                assert len(candidates) == 1, children
                target_pid = candidates[0]
            idle_rss = process_usage(target_pid)[1]
            num_threads = len(list(Path(f'/proc/{target_pid}/task').iterdir()))
            warm_num = 10
            for idx in range(warm_num + args.frames):
                frame_start = time.monotonic()
                if args.moving:
                    # 全点の微小変位によるセル所属変更。Tmapも同じworld変位。
                    dx, dy, dz = .08*math.sin(idx*.7), .06*math.cos(idx*.6), .02*math.sin(idx*.5)
                    moved_points[:, :3] = base_points[:, :3] + np.array([dx, dy, dz], dtype='<f4')
                    msg.data = array('B', moved_points.tobytes())
                    wx, wy = math.cos(.6)*dx-math.sin(.6)*dy, math.sin(.6)*dx+math.cos(.6)*dy
                    for item, pos in zip(topo.nodes, base_nodes):
                        item.pos.x, item.pos.y, item.pos.z = pos[0]+wx, pos[1]+wy, pos[2]+dz
                if idx == warm_num:
                    start_ns = time.monotonic_ns(); cpu_start = process_usage(target_pid)[0]
                stamp_ns = node.get_clock().now().nanoseconds
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
                if has_tmap:
                    topo.header.stamp = msg.header.stamp; topo.frame_number = idx
                    map_pub.publish(topo)
                sent[stamp_ns] = time.monotonic_ns()
                publisher.publish(msg)
                deadline = frame_start + .1
                while time.monotonic() < deadline:
                    assert process.poll() is None
                    rclpy.spin_once(node, timeout_sec=min(.01, max(0., deadline-time.monotonic())))
                peak_rss = max(peak_rss, process_usage(target_pid)[1])
            end_ns = time.monotonic_ns(); cpu_end = process_usage(target_pid)[0]
            wait_end = time.monotonic() + .3
            while time.monotonic() < wait_end:
                rclpy.spin_once(node, timeout_sec=.02)
    finally:
        if process is not None:
            stop(process)
        node.destroy_publisher(publisher); node.destroy_publisher(map_pub)
        for sub in [roi_sub, *visual_subs]:
            node.destroy_subscription(sub)
    assert process.returncode == 0
    stats = {}; rows = []
    for row in ([] if args.installed else csv.reader(trace.open())):
        kind, begin, wall, cpu, stamp, count = row
        if start_ns <= int(begin) < end_ns:
            rows.append((kind, int(begin), float(wall), float(cpu), int(stamp), int(count)))
    for kind in sorted(set(row[0] for row in rows)):
        selected = [row for row in rows if row[0] == kind]
        stats[kind] = {'wall_ms': describe([r[2] for r in selected]),
                       'cpu_ms': describe([r[3] for r in selected]),
                       'num_values': describe([r[5] for r in selected])}
    measured_stamps = {stamp: value for stamp, value in sent.items() if start_ns <= value < end_ns}
    roi_latency = [(received[s][0]-v)/1e6 for s, v in measured_stamps.items() if s in received]
    fvg_done = {}
    if args.installed:
        fvg_done = {stamp: (marker_received[stamp]-sent_ns)/1e6
                    for stamp, sent_ns in measured_stamps.items() if stamp in marker_received}
    for kind, begin, wall, cpu, stamp, count in rows:
        if kind == 'fvg_state' and stamp in measured_stamps:
            fvg_done.setdefault(stamp, (begin-measured_stamps[stamp])/1e6)
    result = {'name': name, 'installed': args.installed, 'executor_threads': None if args.installed else args.threads,
        'process_threads': num_threads, 'points': num, 'mode': mode, 'narrow_fvg': narrow_fvg, 'wide_roi': wide_roi,
        'has_moving_points': args.moving,
        'trial': trial, 'sent': len(measured_stamps), 'received_roi': len(roi_latency),
        'window_ns': [start_ns, end_ns],
        'received_fvg': len(fvg_done), 'duration_sec': (end_ns-start_ns)/1e9,
        'process_cpu_ms_per_input': (cpu_end-cpu_start)*1000/len(measured_stamps),
        'process_cpu_core_pct': (cpu_end-cpu_start)/((end_ns-start_ns)/1e9)*100,
        'idle_rss_mib': idle_rss, 'peak_rss_mib': peak_rss,
        'roi_latency_ms': describe(roi_latency), 'fvg_latency_ms': describe(list(fvg_done.values())),
        'roi_cells': describe([received[s][1] for s in measured_stamps if s in received]),
        'visual_bytes_per_sec': {key: sum(size for stamp, size in vals if start_ns <= stamp < end_ns)/((end_ns-start_ns)/1e9)
                                 for key, vals in sizes.items()}, 'stages': stats}
    assert len(roi_latency) > args.frames*.8, result
    assert not has_fvg or len(fvg_done) > args.frames*.5, result
    (output/f'{name}.json').write_text(json.dumps(result, indent=2))
    print('RESULT', name, 'cpu_ms/input', round(result['process_cpu_ms_per_input'], 2),
          'rss_mib', round(peak_rss, 1), 'roi/fvg', len(roi_latency), len(fvg_done), flush=True)
    return result

def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '182'
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=40)
    parser.add_argument('--trials', type=int, default=2)
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--installed', action='store_true')
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--case-prefix', default='')
    parser.add_argument('--moving', action='store_true')
    args = parser.parse_args()
    if args.moving:
        args.case_prefix = 'moving' + ('_' + args.case_prefix if args.case_prefix else '')
    rclpy.init(); node = rclpy.create_node('shared_voxel_cost_driver')
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for child, xyz, yaw in [('sensor', (.45, -.2, .3), .6), ('roi', (-.2, .3, 0.), -.4)]:
        tf = TransformStamped(); tf.header.frame_id = 'world'; tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    cases = [(num, mode, False, False) for num in (100000, 200000)
             for mode in ('world', 'points', 'tmap', 'markers')]
    cases += [(200000, 'markers', True, False), (200000, 'markers', False, True)]
    if args.smoke:
        cases = [(100000, 'markers', False, False)]
    if args.installed:
        cases = [(100000, 'markers', False, False), (200000, 'markers', False, False),
                 (200000, 'markers', True, False)]
    elif args.threads != 2:
        cases = [(200000, 'markers', False, False)]
    if args.moving:
        cases = [(200000, 'markers', False, False)]
    if args.smoke:
        cases = cases[:1]
    results = []
    try:
        for trial in range(args.trials):
            for case in (cases if trial % 2 == 0 else list(reversed(cases))):
                results.append(run_case(node, case, trial, args))
    finally:
        node.destroy_node(); rclpy.shutdown()
    filename = 'installed.json' if args.installed else ('smoke.json' if args.smoke else 'results.json')
    if not args.installed and args.threads != 2:
        filename = f'threads{args.threads}.json'
    if args.case_prefix:
        filename = args.case_prefix + '_' + filename
    (output/filename).write_text(json.dumps(results, indent=2))

if __name__ == '__main__':
    main()
