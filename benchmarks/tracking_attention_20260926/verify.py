"""粗いセル重点化のOFF同値性、合成追従、実bagのROS接続・時間の検証。"""
import argparse
import ctypes as ct
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import statistics
import subprocess
import time

import numpy as np
import yaml

root = Path('/ros2_ws/src')
output = root / 'artifacts/tracking_attention_20260926'
core = root / 'ais_gng_cpu/src/gng_cpu'
bag = '/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3'
spec = importlib.util.spec_from_file_location('existing_benchmark',
    root / 'benchmarks/gng_followup_efficiency_20260924/benchmark.py')
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)


class node_ref(ct.Structure):
    _fields_ = [('id', ct.c_uint32), ('frame', ct.c_uint32)]


class tracking_input(ct.Structure):
    _fields_ = [('nonplane_nodes', ct.POINTER(node_ref)), ('num_nonplane_nodes', ct.c_uint32),
        ('ratio', ct.c_double), ('cell_size', ct.c_double), ('min_points', ct.c_uint32),
        ('min_nonplane_nodes', ct.c_uint32), ('max_points_per_node_th', ct.c_double),
        ('min_centroid_dist_ratio_th', ct.c_double), ('max_centroid_dist_ratio', ct.c_double), ('mode', ct.c_uint32)]


class builtin_input(ct.Structure):
    _fields_ = [('grasp_boxes', ct.c_void_p), ('num_grasp_boxes', ct.c_uint32), ('grasp_ratio', ct.c_double),
        ('boundary_points', ct.c_void_p), ('num_boundary_points', ct.c_uint32),
        ('boundary_radius', ct.c_double), ('boundary_ratio', ct.c_double), ('tracking', tracking_input)]


class sampling_stats(ct.Structure):
    _fields_ = [(name, ct.c_uint32) for name in ('num_cells', 'num_cell_evaluations',
        'num_point_evaluations', 'num_entries', 'num_priority_samples')] + [('has_invalid_score', ct.c_uint8)]


def run(command, name, timeout=300):
    print('START', list(map(str, command)), flush=True)
    with (output / name).open('w') as log:
        subprocess.run(list(map(str, command)), check=True, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    print('DONE', name, flush=True)


def prepare(args):
    for name in (('before', 'after') if not args.tag else ('after',)):
        source = output / ('det_' + args.tag + name + '_src')
        shutil.copytree(output / 'before/gng_cpu' if name == 'before' else core, source)
        # 再現用コピーだけの乱数・時刻固定。本番コードの変更なし。
        for relative, old, new in (
            ('src/cpu/cugng.cpp', 'mt19937 mt(rnd());', 'mt19937 mt(20260926);'),
            ('src/utils/utils.hpp', 'dt = LIMIT(dt, 0.1, 0.5);', 'dt = 0.1;')):
            path = source / relative
            text = path.read_text(); assert text.count(old) == 1
            path.write_text(text.replace(old, new))
        target = output / ('det_' + args.tag + name)
        run(['cmake', '-S', source, '-B', target, '-DCMAKE_BUILD_TYPE=Release',
             '-DGNG_ENABLE_FRAME_LOG=OFF'], 'det_' + args.tag + name + '_configure.log')
        run(['cmake', '--build', target, '--target', 'gng_cpu', '-j4'], 'det_' + args.tag + name + '_build.log')


def off_compare(args):
    results = {}
    for trial in range(2):
        for name in (('before', 'after') if trial == 0 else ('after', 'before')):
            target = output / ('det_' + (args.tag if name == 'after' else '') + name)
            path = output / f'{args.tag}off_{name}_{trial}.json'
            run(['taskset', '-c', '4', 'python3', root / 'benchmarks/gng_followup_efficiency_20260924/benchmark.py',
                 '--library', target / 'libgng_cpu.so', '--config', output / 'before/at128.yaml',
                 '--bag', bag, '--frames', '80', '--warmup', '20', '--voxel', '.5', '--no-legacy-priority',
                 '--features', '--output', path], f'{args.tag}off_{name}_{trial}.log')
            results[f'{name}_{trial}'] = json.loads(path.read_text())
    reference = results['before_0']['records']
    for result in results.values():
        for left, right in zip(reference, result['records']):
            assert {k:v for k,v in left.items() if not k.endswith('_ms')} == {
                k:v for k,v in right.items() if not k.endswith('_ms')}
    summary = {name: value['mean']['total_ms'] for name, value in results.items()}
    (output / (args.tag+'off_summary.json')).write_text(json.dumps(summary, indent=2))
    print('PASS OFF all public output hashes, 320 frames', summary, flush=True)


def motion(args):
    from scipy.spatial import cKDTree
    lib = ct.CDLL(str(args.library or output / ('det_'+args.tag+'after/libgng_cpu.so')))
    lib.gng_setParameter.argtypes = [ct.c_char_p, ct.c_uint32, ct.c_float]
    lib.gng_setPointCloud.argtypes = [ct.c_void_p, ct.c_uint32, ct.POINTER(common.lidar)]
    lib.gng_getTopologicalMap.restype = common.tmap
    lib.gng_set_builtin_sampling.argtypes = [ct.POINTER(builtin_input)]
    lib.gng_get_sampling_stats.restype = sampling_stats
    experiment = None
    modes = {'np_points': 1, 'np_cells': 2, 'np_error': 3, 'all_error': 4,
        'change_points': 5, 'change_cells': 6, 'np_change': 7, 'np_hold': 8}
    if args.mode in modes:
        if not args.sampler_library:
            raise ValueError('--sampler-libraryが必要')
        experiment = ct.CDLL(str(args.sampler_library))
        experiment.configure_experiment.argtypes = [ct.c_void_p, ct.POINTER(node_ref), ct.c_uint32,
            ct.c_int, ct.c_double, ct.c_double, ct.c_double]
        if args.mode == 'np_hold':
            experiment.configure_retention.argtypes = [ct.c_uint32]
            experiment.retained_mass_ratio.restype = ct.c_double
            assert experiment.configure_retention(args.hold_frames)
    settings = yaml.safe_load((output / 'before/at128.yaml').read_text())['ais_gng_node']['ros__parameters']
    settings.update({'node.num_max': 4000, 'node.learning_num': 4000, 'input.point_cloud_num': 100000,
        'input.voxel_grid_unit': .1, 'input.local_coordinates': True,
        'input.x_min': -12., 'input.x_max': 12., 'input.y_min': -12., 'input.y_max': 12.,
        'input.z_min': -2., 'input.z_max': 4.})
    if args.mode == 'unknown':
        settings['node.unknown_learning_rate'] = args.ratio
    for name, value in settings.items():
        for idx, scalar in enumerate(value if isinstance(value, list) else [value]):
            if isinstance(scalar, (int, float, bool)):
                lib.gng_setParameter(name.encode(), idx, float(scalar))
    assert lib.gng_init() == 0
    rng = np.random.default_rng(args.seed)
    ground = np.column_stack([rng.uniform(-10, 10, (95000, 2)), np.zeros(95000)]).astype('f4')
    angle = rng.uniform(0, 2*np.pi, 5000)
    body = np.column_stack([.25*np.cos(angle), .25*np.sin(angle), rng.uniform(.15, 1.8, 5000)]).astype('f4')
    sensor = common.lidar(common.vec(0,0,0), common.quat(0,0,0,1), 12)
    refs = (node_ref * 0)()
    records = []
    for frame in range(args.frames):
        if args.resample_ground:
            ground[:, :2] = rng.uniform(-10, 10, (95000, 2))
        center = np.array([1.2 + max(0, frame-args.warmup_frames+1)*args.step, .23, 0], dtype='f4')
        moving = body + center
        points = np.concatenate([ground, moving])
        begin = time.perf_counter_ns()
        lib.gng_setPointCloud(points.ctypes.data, len(points), ct.byref(sensor))
        after_input = time.perf_counter_ns()
        if experiment:
            has_label_gap = args.label_gap_frames and frame % args.label_gap_period < args.label_gap_frames
            num_refs = 0 if has_label_gap else len(refs)
            assert experiment.configure_experiment(ct.cast(lib.gng_set_sampling_rules, ct.c_void_p),
                refs, num_refs, modes[args.mode], args.ratio, args.min_dist, args.max_dist)
        elif args.mode == 'builtin_nearest' or (args.cell_size and args.mode == 'tracking'):
            value = builtin_input()
            value.tracking = tracking_input(refs, len(refs), args.ratio, args.cell_size or .5, 20, 3, 50, .1, .5,
                1 if args.mode == 'builtin_nearest' else 0)
            assert lib.gng_set_builtin_sampling(ct.byref(value))
        after_prepare = time.perf_counter_ns()
        lib.gng_exec()
        after_exec = time.perf_counter_ns()
        result = lib.gng_getTopologicalMap()
        end = time.perf_counter_ns()
        stats = lib.gng_get_sampling_stats()
        assert not stats.has_invalid_score and stats.num_priority_samples <= 4000*args.ratio+1
        # 合成形状の既知の非平面部分。実クラスタ所属の接続は別のROS試験。
        refs = (node_ref * sum(result.nodes[i].pos.z > .08 for i in range(result.node_num)))(
            *[node_ref(result.nodes[i].id, result.nodes[i].frame) for i in range(result.node_num)
              if result.nodes[i].pos.z > .08])
        positions = np.array([[result.nodes[i].pos.x, result.nodes[i].pos.y, result.nodes[i].pos.z]
                              for i in range(result.node_num)])
        tree = cKDTree(positions)
        object_dist = tree.query(moving[::5])[0]
        ground_dist = tree.query(ground[::95])[0]
        # 通過済み円筒の掃引領域に残る高さ付きノード。位置許容は入力voxel幅の0.1 m。
        swept_x = np.clip(positions[:, 0], 1.2, center[0])
        swept_dist = np.hypot(positions[:, 0]-swept_x, positions[:, 1]-center[1])
        current_dist = np.linalg.norm(positions[:, :2]-center[:2], axis=1)
        has_height = (positions[:, 2] > .1) & (positions[:, 2] < 1.9)
        num_trail_nodes = int(np.count_nonzero(has_height & (swept_dist <= .35) & (current_dist > .35)))
        records.append(dict(frame=frame, total_ms=(end-begin)/1e6, prepare_ms=(after_prepare-after_input)/1e6,
            exec_ms=(after_exec-after_prepare)/1e6, object_mean_dist=float(object_dist.mean()),
            object_p95_dist=float(np.quantile(object_dist,.95)), ground_mean_dist=float(ground_dist.mean()),
            priority=stats.num_priority_samples, point_evaluations=stats.num_point_evaluations,
            num_trail_nodes=num_trail_nodes,
            retained_mass_ratio=experiment.retained_mass_ratio() if args.mode == 'np_hold' else 0.))
    summary = {key: float(np.mean([r[key] for r in records[args.warmup_frames:]])) for key in records[0] if key != 'frame'}
    path = args.output or output / f'{args.tag}motion_{args.cell_size}_{args.seed}.json'
    path.write_text(json.dumps(dict(summary=summary, records=records), indent=2))
    if args.output:
        path.with_name('metrics.json').write_text(json.dumps(summary, indent=2))
    print('RESULT motion', args.mode, args.cell_size, args.seed, summary, flush=True)


def suite(args):
    cases = []
    for item in args.cases:
        fields = item.split(':')
        mode = fields[0]
        ratio = float(fields[1]) if len(fields) > 1 else args.ratio
        min_dist = float(fields[2]) if len(fields) > 2 else args.min_dist
        hold_frames = int(fields[3]) if len(fields) > 3 else args.hold_frames
        command = ['taskset', '-c', str(args.cpu), 'python3', str(Path(__file__).resolve()), 'motion',
            '--tag', args.tag, '--mode', mode, '--ratio', str(ratio), '--min-dist', str(min_dist),
            '--max-dist', str(args.max_dist),
            '--hold-frames', str(hold_frames), '--label-gap-frames', str(args.label_gap_frames),
            '--label-gap-period', str(args.label_gap_period),
            '--cell-size', '.5', '--frames', str(args.frames), '--warmup-frames', str(args.warmup_frames),
            '--step', str(args.step), '--seed', '@seed@', '--output', '@case_dir@/result.json',
            '--sampler-library', str(args.sampler_library or output/'experiment_sampler.so')]
        if args.library:
            command += ['--library', str(args.library)]
        cases.append(dict(name=item.replace(':','_').replace('.','_'), argv=command,
            metrics='metrics.json', cwd=str(root), env={'PYTHONDONTWRITEBYTECODE':'1'}))
        if args.resample_ground:
            command += ['--resample-ground']
    if not args.output:
        raise ValueError('--outputに新しいmanifestのパスが必要')
    with args.output.open('x') as file:
        json.dump(dict(cases=cases), file, indent=2)


def summarize(args):
    results = {}
    for path in args.reports:
        report = json.loads(path.read_text())
        if report['status'] != 'completed' or not all(r['cleanup_ok'] for r in report['records']):
            raise ValueError(f'未完了または後片付け未確認: {path}')
        baseline = {r['seed']: r['metrics'] for r in report['records'] if r['name'] == 'off'}
        cases = {}
        for name in dict.fromkeys(r['name'] for r in report['records']):
            rows = [r for r in report['records'] if r['name'] == name]
            cases[name] = dict(seeds=[r['seed'] for r in rows], metrics={})
            for key in rows[0]['metrics']:
                values = [r['metrics'][key] for r in rows]
                delta = [r['metrics'][key]-baseline[r['seed']][key] for r in rows]
                cases[name]['metrics'][key] = dict(mean=statistics.mean(values),
                    median=statistics.median(values),
                    std=statistics.stdev(values) if len(values)>1 else None,
                    delta_mean=statistics.mean(delta), delta_values=delta)
        results[path.parent.name] = dict(report=str(path), total=report['total'],
            elapsed_sec=report['elapsed_sec'], cases=cases)
    if not args.output or not results:
        raise ValueError('--reportsと新しい--outputが必要')
    with args.output.open('x') as file:
        json.dump(results, file, indent=2, allow_nan=False)
    print('RESULT summary', args.output, sum(r['total'] for r in results.values()), flush=True)


def ros(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from ais_gng_msgs.msg import TopologicalMap
    from visualization_msgs.msg import MarkerArray, Marker
    assert os.environ.get('ROS_DOMAIN_ID') == '183'
    config_dir = root / 'ais_gng_cpu/src/ais_gng/config'
    settings = yaml.safe_load((config_dir / 'gng_cpu/at128.yaml').read_text())['ais_gng_node']['ros__parameters']
    planes = yaml.safe_load((config_dir / 'plane_cluster_incremental.yaml').read_text())['plane_cluster_incremental_node']['ros__parameters']
    settings.update({'plane_cluster.'+k:v for k,v in planes.items()})
    settings.update({'input.local_coordinates': True, 'input.point_cloud_num': args.max_points,
        'input.sampling_mode': 'uniform', 'input.visualize': False, 'classify.human': False,
        'classify.car': False, 'performance.log_interval_ms': 1,
        'tracking_attention.enable_pointcloud': bool(args.cell_size) and (args.validation or args.publish_candidates), 'tracking_attention.timeout_sec': 5.,
        'tracking_attention.mode': args.tracking_mode, 'tracking_attention.ratio': args.ratio,
        'enable_node_insertion': args.node_insertion,
        'plane_contact.enable_voxels': args.plane_contact,
        'plane_contact.interval_sec': args.plane_contact_interval,
        'enable_tracking_attention': bool(args.cell_size), 'tracking_attention.cell_size': args.cell_size or .5})
    if args.validation:
        # 点群出力の宣言既定値も検証対象。
        settings.pop('tracking_attention.enable_pointcloud')
    case = f'ros_{args.tracking_mode}_{args.cell_size}_{args.seed}' + ('_validation' if args.validation else '')
    namespace = '/' + case.replace('.', '_')
    case_dir = args.output.parent if args.output else output
    settings['input.topic_names'] = [namespace+'/input']
    path = case_dir / (case+'.yaml'); path.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
    with sqlite3.connect('file:'+bag+'?mode=ro', uri=True) as db:
        topic = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        frames = [deserialize_message(data, PointCloud2) for data, in db.execute(
            'select data from messages where topic_id=? order by timestamp limit 60', (topic,))]
    rclpy.init(); observer = rclpy.create_node('tracking_verification'); cache = {}; counts = []
    observer.create_subscription(TopologicalMap, namespace+'/topological_map', lambda msg: cache.__setitem__('map', msg), qos_profile_sensor_data)
    observer.create_subscription(PointCloud2, namespace+'/downsampling/tracking', lambda msg: cache.__setitem__('points', msg), qos_profile_sensor_data)
    contact_counts = []
    contact_times = []
    if args.subscribe_plane_contact:
        assert args.plane_contact
        def receive_contacts(msg):
            cache['contacts'] = msg
            contact_times.append(time.monotonic())
            if args.plane_contact_interval > 0:
                assert len(msg.markers) == 2 and all(m.type == Marker.CUBE_LIST for m in msg.markers)
                contact_counts.append(sum(len(m.points) for m in msg.markers))
        observer.create_subscription(MarkerArray, namespace+'/plane_contact_voxels',
            receive_contacts, 1)
    publisher = observer.create_publisher(PointCloud2, namespace+'/input', 1)
    process = None; log_path = case_dir/(case+'.log')
    try:
        with log_path.open('w') as log:
            command = [str(args.executable), '--ros-args', '-r', '__ns:='+namespace,
                       '--params-file', str(path)]
            # runnerと同一グループの子ノード。強制中断時もグループ単位の回収対象。
            process = subprocess.Popen(command, stdout=log, stderr=log)
            print('START', process.pid, command, flush=True)
            deadline = time.monotonic()+20
            while not publisher.get_subscription_count():
                assert process.poll() is None and time.monotonic()<deadline
                rclpy.spin_once(observer, timeout_sec=.05)
            ready = time.monotonic()+1
            while time.monotonic()<ready: rclpy.spin_once(observer, timeout_sec=.05)
            loaded_libraries = sorted({line.split()[-1] for line in Path(f'/proc/{process.pid}/maps').read_text().splitlines()
                if 'libgng_cpu.so' in line or 'libais_gng_component_cpu.so' in line})
            for idx, msg in enumerate(frames):
                msg.header.frame_id = 'test_world'; msg.header.stamp.sec = 1000+idx//10
                msg.header.stamp.nanosec = idx%10*100000000
                if args.validation and idx == 56: msg.header.frame_id = 'changed_frame'
                if args.validation and idx == 58: msg.header.stamp.sec = 1
                cache.clear(); publisher.publish(msg); deadline = time.monotonic()+15
                required = ('map','points') if args.cell_size and (args.validation or args.publish_candidates) else ('map',)
                while not all(key in cache and cache[key].header == msg.header for key in required):
                    assert process.poll() is None and time.monotonic()<deadline, (case,idx,cache.keys())
                    rclpy.spin_once(observer, timeout_sec=.02)
                if args.subscribe_plane_contact and args.plane_contact_interval == 0:
                    while 'contacts' not in cache or cache['contacts'].markers[0].header != msg.header:
                        assert process.poll() is None and time.monotonic() < deadline
                        rclpy.spin_once(observer, timeout_sec=.02)
                    markers = cache['contacts'].markers
                    assert len(markers) == 2
                    assert [m.ns for m in markers] == ['plane_node_cells', 'adjacent_input_cells']
                    size = settings['input.voxel_grid_unit']
                    origin = [settings['input.'+dim+'_min'] for dim in ('x','y','z')]
                    for marker in markers:
                        assert marker.type == Marker.CUBE_LIST and marker.header == msg.header
                        assert marker.pose.orientation.w == 1
                        if marker.points:
                            assert marker.scale.x == marker.scale.y == marker.scale.z == size
                        assert marker.action == (Marker.ADD if marker.points else Marker.DELETE)
                        for point in marker.points:
                            for pos, start in zip((point.x,point.y,point.z), origin):
                                cell = (pos-start)/size-.5
                                assert abs(cell-round(cell)) < 1e-4
                    assert markers[0].color.g > markers[0].color.r
                    assert markers[1].color.r > markers[1].color.g
                    contact_counts.append(sum(len(m.points) for m in markers))
                assert cache['map'].nodes
                if args.cell_size and (args.validation or args.publish_candidates):
                    counts.append(cache['points'].width * cache['points'].height)
                    assert counts[-1] <= args.max_points
                    if idx == 0 or (args.validation and idx >= 56): assert counts[-1] == 0, (idx, counts[-1])
            if args.cell_size and (args.validation or args.publish_candidates): assert max(counts[1:55]) > 0
            else: assert observer.count_publishers(namespace+'/downsampling/tracking') == 0
            assert observer.count_publishers(namespace+'/plane_contact_voxels') == int(args.plane_contact)
            if args.subscribe_plane_contact and args.plane_contact_interval == 0:
                assert len(contact_counts) == 60 and max(contact_counts) > 0
            elif args.subscribe_plane_contact:
                assert len(contact_counts) >= 2 and max(contact_counts) > 0
                assert all(b-a >= args.plane_contact_interval-.1 for a,b in zip(contact_times, contact_times[1:]))
    finally:
        if process:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                if process.poll() is not None: break
                process.send_signal(sig)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired: pass
            assert process.poll() is not None
            print('STOP', process.pid, process.returncode, flush=True)
        observer.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    elapsed = [float(v) for v in re.findall(r'processing=([0-9.]+) ms', log_path.read_text())]
    assert len(elapsed) == 60
    gng_elapsed = [float(v) for v in re.findall(r'GNG: ([0-9.]+) ms', log_path.read_text())]
    assert len(gng_elapsed) == 60
    if args.cell_size:
        assert f'mode={args.tracking_mode} ratio={args.ratio:.2f}' in log_path.read_text()
    inserted = [int(v) for v in re.findall(r'Insert: ([0-9]+)', log_path.read_text())]
    if args.node_insertion:
        assert len(inserted) == 60 and max(inserted) <= settings['node.num_max']
        assert inserted[0] == 0
        if args.validation: assert all(value == 0 for value in inserted[56:])
        assert max(inserted[1:55]) > 0, '直接挿入の実行なし'
    result = dict(processing_ms=float(np.mean(elapsed[20:55])), gng_ms=float(np.mean(gng_elapsed[20:55])), candidates=counts, frames=60)
    if contact_counts:
        result['num_contact_voxels'] = float(np.mean(contact_counts[20:55] if len(contact_counts) == 60 else contact_counts))
        result['num_contact_messages'] = len(contact_counts)
    if args.node_insertion:
        result['inserted_nodes'] = inserted
        result['num_inserted_nodes'] = sum(inserted)
    result['loaded_libraries'] = loaded_libraries
    (args.output or case_dir/(case+'.json')).write_text(json.dumps(result, indent=2))
    if args.output:
        args.output.with_name('metrics.json').write_text(json.dumps({k:v for k,v in result.items() if isinstance(v, (int,float))}))
    print('RESULT', case, result['processing_ms'], flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('prepare','off_compare','motion','ros','suite','summarize'))
    parser.add_argument('--cell-size', type=float, default=0)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--validation', action='store_true')
    parser.add_argument('--tag', default='')
    parser.add_argument('--library', type=Path)
    parser.add_argument('--sampler-library', type=Path)
    parser.add_argument('--mode', choices=('tracking','builtin_nearest','off','unknown','np_points','np_cells','np_error','all_error',
        'change_points','change_cells','np_change','np_hold'), default='tracking')
    parser.add_argument('--ratio', type=float, default=.25)
    parser.add_argument('--min-dist', type=float, default=.05)
    parser.add_argument('--max-dist', type=float, default=.2)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--warmup-frames', type=int, default=30)
    parser.add_argument('--step', type=float, default=.06)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cases', nargs='+', default=['off','tracking','np_points','np_error'])
    parser.add_argument('--cpu', type=int, default=4)
    parser.add_argument('--resample-ground', action='store_true')
    parser.add_argument('--reports', type=Path, nargs='+', default=[])
    parser.add_argument('--hold-frames', type=int, default=0)
    parser.add_argument('--label-gap-frames', type=int, default=0)
    parser.add_argument('--label-gap-period', type=int, default=5)
    parser.add_argument('--tracking-mode', choices=('coarse','nearest_nonplane'), default='coarse')
    parser.add_argument('--max-points', type=int, default=100000)
    parser.add_argument('--publish-candidates', action='store_true')
    parser.add_argument('--node-insertion', action='store_true')
    parser.add_argument('--plane-contact', action='store_true')
    parser.add_argument('--subscribe-plane-contact', action='store_true')
    parser.add_argument('--plane-contact-interval', type=float, default=.5)
    parser.add_argument('--executable', type=Path, default=Path('/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu'))
    args = parser.parse_args()
    if args.frames <= args.warmup_frames or args.warmup_frames < 0 or not 0 < args.ratio < 1:
        parser.error('frames/warmup-frames/ratioの不正値')
    if not 0 <= args.hold_frames <= 1000 or not 0 <= args.label_gap_frames < args.label_gap_period:
        parser.error('hold-frames/label-gap-frames/label-gap-periodの不正値')
    if args.stage == 'prepare': prepare(args)
    elif args.stage == 'off_compare': off_compare(args)
    elif args.stage == 'motion': motion(args)
    elif args.stage == 'suite': suite(args)
    elif args.stage == 'summarize': summarize(args)
    else: ros(args)
