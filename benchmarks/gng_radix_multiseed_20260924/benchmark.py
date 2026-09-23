import argparse
import ctypes as ct
import hashlib
import json
import sqlite3
import time
import warnings
from pathlib import Path

import numpy as np
import yaml
with warnings.catch_warnings():
    warnings.simplefilter("ignore", UserWarning)
    from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


class vec(ct.Structure):
    _fields_ = [('x', ct.c_float), ('y', ct.c_float), ('z', ct.c_float)]


class quat(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ('x', 'y', 'z', 'w')]


class lidar(ct.Structure):
    _fields_ = [('pos', vec), ('quat', quat), ('point_step', ct.c_uint32)]


class node(ct.Structure):
    _fields_ = [('id', ct.c_uint16), ('pos', vec), ('normal', vec), ('rho', ct.c_float),
                ('label', ct.c_uint8), ('frame', ct.c_uint32), ('inpcl_ids', ct.c_void_p), ('inpcl_num', ct.c_uint32)]


class cluster(ct.Structure):
    _fields_ = [('id', ct.c_uint32), ('label', ct.c_uint8), ('label_reliability', ct.c_float),
                ('pos', vec), ('scale', vec), ('quat', quat), ('frame', ct.c_uint32), ('match', ct.c_float),
                ('velocity', vec), ('nodes', ct.c_void_p), ('node_num', ct.c_uint32)]


class tmap(ct.Structure):
    _fields_ = [('frame_number', ct.c_uint32), ('node_num', ct.c_uint32), ('cluster_num', ct.c_uint32),
                ('edge_num', ct.c_uint32), ('nodes', ct.POINTER(node)), ('clusters', ct.POINTER(cluster)),
                ('edges', ct.c_void_p), ('edges_dist', ct.c_void_p)]


class efficiency_view(ct.Structure):
    _fields_ = [('ms', ct.POINTER(ct.c_double)), ('indices', ct.c_void_p), ('ranges', ct.c_void_p),
                ('points', ct.c_void_p), ('input_num', ct.c_uint32), ('voxel_num', ct.c_uint32),
                ('attention_num', ct.c_uint32), ('learning_num', ct.c_uint64)]


class training_event(ct.Structure):
    _fields_ = [('winner_node_id', ct.c_uint16), ('winner_rank', ct.c_uint16),
                ('winner_node_frame', ct.c_uint32), ('residual', vec)]


class map_delta(ct.Structure):
    _fields_ = [(name, ct.c_uint32) for name in ('version', 'frame_number', 'node_num', 'edge_num')] + [
        ('nodes', ct.c_void_p), ('edges', ct.c_void_p)]


class node_statistics(ct.Structure):
    _fields_ = [('winner_point_count', ct.c_double), ('winner_point_covariance', ct.c_double * 9),
                ('support_weight_sum', ct.c_double), ('support_moment', ct.c_double * 9)]


class pixel_view(ct.Structure):
    _fields_ = [('mode', ct.c_uint8), ('data', ct.c_void_p), ('data_size', ct.c_uint64),
                ('selected_ids', ct.c_void_p)] + [(name, ct.c_uint32) for name in (
                    'selected_num', 'point_num', 'width', 'height', 'point_step', 'row_step',
                    'image_width', 'image_height', 'first_offset', 'second_offset')] + [
                ('first_size', ct.c_uint8), ('second_size', ct.c_uint8), ('is_bigendian', ct.c_bool)]


class observation_input(ct.Structure):
    _fields_ = [('origin', vec), ('has_origin', ct.c_uint8), ('pixels', pixel_view),
                ('angle_table', ct.c_void_p), ('table_num', ct.c_uint32)]


class angle_range(ct.Structure):
    _fields_ = [(name, ct.c_uint16) for name in ('min_yaw', 'max_yaw', 'min_pitch', 'max_pitch')] + [
        ('has_support', ct.c_bool), ('has_yaw', ct.c_bool)]


def digest_structure(value, omitted=()):
    # アドレスとパディングを除外した、浮動小数点ビット列の比較。
    data = bytearray()
    for name, field_type in value._fields_:
        if name not in omitted:
            data.extend(ct.string_at(ct.addressof(value) + getattr(type(value), name).offset, ct.sizeof(field_type)))
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--library', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--frames', type=int, default=300)
    parser.add_argument('--warmup', type=int, default=50)
    parser.add_argument('--voxel', type=float, required=True)
    parser.add_argument('--features', dest='enable_features', action='store_true')
    parser.add_argument('--seed', type=int, default=20260924)
    parser.add_argument('--observe', dest='enable_observation', action='store_true')
    parser.add_argument('--last-representative', dest='enable_last_representative', action='store_true')
    parser.add_argument('--quality-every', type=int, default=10)
    args = parser.parse_args()
    assert args.frames > args.warmup >= 0 and args.quality_every > 0
    assert np.allclose(cKDTree([[0, 0, 0], [2, 0, 0]]).query([[0.5, 0, 0], [3, 0, 0]])[0], [0.5, 1])
    lib = ct.CDLL(args.library)
    lib.gng_setParameter.argtypes = [ct.c_char_p, ct.c_uint32, ct.c_float]
    lib.gng_setPointCloud.argtypes = [ct.c_void_p, ct.c_uint32, ct.POINTER(lidar)]
    lib.gng_getTopologicalMap.restype = tmap
    lib.gng_getDownSampling.argtypes = [ct.POINTER(ct.c_uint32)]
    lib.gng_getDownSampling.restype = ct.c_void_p
    lib.gng_get_voxel_sort_ms.restype = ct.c_double
    lib.gng_set_trial_input_capture.argtypes = [ct.c_uint8]
    lib.gng_set_trial_seed.argtypes = [ct.c_uint32]
    lib.gng_set_trial_last_representative.argtypes = [ct.c_uint8]
    lib.gng_get_trial_inputs.argtypes = [ct.POINTER(ct.c_uint32)]
    lib.gng_get_trial_inputs.restype = ct.c_void_p
    has_efficiency_view = hasattr(lib, 'gng_get_efficiency_view')
    if has_efficiency_view:
        lib.gng_get_efficiency_view.restype = efficiency_view
    lib.gng_getTrainingEvents.argtypes = [ct.POINTER(ct.c_uint32)]
    lib.gng_getTrainingEvents.restype = ct.c_void_p
    lib.gng_getTopologicalMapDelta.restype = ct.POINTER(map_delta)
    lib.gng_get_node_statistics.argtypes = [ct.c_uint16]
    lib.gng_get_node_statistics.restype = node_statistics
    lib.gng_get_observation_angle_range.argtypes = [ct.c_uint16]
    lib.gng_get_observation_angle_range.restype = angle_range
    lib.gng_set_weighted_priority_input.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_uint32, ct.c_float]
    lib.gng_set_observation_input.argtypes = [ct.POINTER(observation_input)]
    params = yaml.safe_load(Path(args.config).read_text())['ais_gng_node']['ros__parameters']
    params['input.voxel_grid_unit'] = args.voxel
    if args.enable_observation:
        params['node.enable_observation_support'] = True
    if args.enable_features:
        params.update({'node.enable_observation_support': True, 'node.covariance_enabled': True,
                       'node.enable_support': True, 'node.covariance_winner_rank_max': 2})
    accepted, ignored = {}, {}
    for name, value in params.items():
        for idx, scalar in enumerate(value if isinstance(value, list) else [value]):
            if isinstance(scalar, (int, float, bool)):
                destination = accepted if lib.gng_setParameter(name.encode(), idx, float(scalar)) else ignored
                destination.setdefault(name, []).append(scalar)
    assert accepted['node.learning_num'] == [4000]
    assert lib.gng_init() == 0
    lib.gng_set_trial_seed(args.seed)
    lib.gng_set_trial_last_representative(args.enable_last_representative)
    lib.gng_set_trial_input_capture(1)
    if args.enable_features:
        lib.gng_setTrainingEventCapture(1)
        lib.gng_setTrainingEventMaxWinnerRank(2)
        lib.gng_setMapDeltaCapture(1)
    with sqlite3.connect('file:' + args.bag + '?mode=ro', uri=True) as db:
        topic_id = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        rows = db.execute('select data from messages where topic_id=? order by timestamp limit ?',
                          (topic_id, args.frames)).fetchall()
    assert len(rows) == args.frames
    clouds = []
    for row in rows:
        msg = deserialize_message(row[0], PointCloud2)
        assert [(field.name, field.offset) for field in msg.fields[:3]] == [('x', 0), ('y', 4), ('z', 8)]
        clouds.append(((ct.c_uint8 * len(msg.data)).from_buffer_copy(msg.data), msg.width * msg.height,
                       lidar(vec(0, 0, 0), quat(0, 0, 0, 1), msg.point_step)))
    node_dtype = np.dtype({'names': ['id', 'pos', 'normal', 'rho', 'label', 'frame', 'inpcl_num'],
        'formats': [np.uint16, (np.float32, 3), (np.float32, 3), np.float32, np.uint8, np.uint32, np.uint32],
        'offsets': [getattr(node, name).offset for name in ('id', 'pos', 'normal', 'rho', 'label', 'frame', 'inpcl_num')],
        'itemsize': ct.sizeof(node)})
    records = []
    for frame_idx, (data, num_points, config) in enumerate(clouds):
        begin = time.perf_counter_ns()
        lib.gng_setPointCloud(data, num_points, ct.byref(config))
        after_input = time.perf_counter_ns()
        if args.enable_features:
            point_ids = np.arange(0, num_points, 101, dtype=np.uint32)
            weights = np.linspace(1, 2, len(point_ids), dtype=np.float32)
            assert lib.gng_set_weighted_priority_input(point_ids.ctypes.data, weights.ctypes.data, len(point_ids), 0.3)
        if args.enable_features or args.enable_observation:
            observation = observation_input()
            observation.has_origin = 1
            assert lib.gng_set_observation_input(ct.byref(observation))
        before_exec = time.perf_counter_ns()
        lib.gng_exec()
        after_exec = time.perf_counter_ns()
        result = lib.gng_getTopologicalMap()
        after_output = time.perf_counter_ns()
        # ハッシュ計算・入力読込は実行時間の計測対象外。
        nodes = np.frombuffer(ct.string_at(result.nodes, result.node_num * ct.sizeof(node)), dtype=node_dtype)
        assert result.node_num > 0 and np.isfinite(nodes['pos']).all() and np.isfinite(nodes['normal']).all()
        edges = np.frombuffer(ct.string_at(result.edges, result.edge_num * 2), dtype=np.uint16)
        assert result.edge_num % 2 == 0 and (len(edges) == 0 or int(edges.max()) < result.node_num)
        graph_digest = hashlib.sha256()
        for name in node_dtype.names:
            graph_digest.update(nodes[name].tobytes())
        graph_digest.update(edges.tobytes())
        topology_digest = hashlib.sha256()
        for name in ('id', 'label', 'frame'):
            topology_digest.update(nodes[name].tobytes())
        topology_digest.update(edges.tobytes())
        cluster_digest = hashlib.sha256()
        for idx in range(result.cluster_num):
            value = result.clusters[idx]
            cluster_digest.update(digest_structure(value, ('nodes',)))
            cluster_digest.update(ct.string_at(value.nodes, value.node_num * 2))
        label_num = ct.c_uint32()
        labels = lib.gng_getDownSampling(ct.byref(label_num))
        label_digest = hashlib.sha256(ct.string_at(labels, label_num.value)).hexdigest()
        record = dict(frame=frame_idx, input_num=num_points, nodes=result.node_num, edges=result.edge_num // 2,
            clusters=result.cluster_num, input_ms=(after_input-begin)/1e6,
            exec_ms=(after_exec-before_exec)/1e6, output_ms=(after_output-after_exec)/1e6,
            total_ms=(after_input-begin+after_output-before_exec)/1e6,
            topology_sha256=topology_digest.hexdigest(), sort_ms=lib.gng_get_voxel_sort_ms(),
            graph_sha256=graph_digest.hexdigest(), cluster_sha256=cluster_digest.hexdigest(), label_sha256=label_digest)
        if has_efficiency_view:
            view = lib.gng_get_efficiency_view()
            assert view.learning_num == 4000
            voxel_digest = hashlib.sha256(ct.string_at(view.indices, view.input_num * 8))
            voxel_digest.update(ct.string_at(view.ranges, view.voxel_num * 8))
            voxel_digest.update(ct.string_at(view.points, view.voxel_num * 12))
            record.update(voxel_sha256=voxel_digest.hexdigest(), voxel_num=view.voxel_num,
                          kept_input_num=view.input_num, attention_num=view.attention_num, learning_num=view.learning_num)
            record.update(zip(('voxel_ms', 'attention_ms', 'learn_ms', 'label_ms', 'maintenance_ms', 'cluster_ms'), view.ms[:6]))
            indices = np.frombuffer(ct.string_at(view.indices, view.input_num * 8), dtype=np.uint32).reshape(-1, 2)
            ranges = np.frombuffer(ct.string_at(view.ranges, view.voxel_num * 8), dtype=np.uint32).reshape(-1, 2)
            centroids = np.frombuffer(ct.string_at(view.points, view.voxel_num * 12), dtype=np.float32).reshape(-1, 3)
            assert np.isfinite(centroids).all()
            assert np.all(indices[1:, 0] >= indices[:-1, 0])
            cell_by_raw = np.full(num_points, np.iinfo(np.uint32).max, dtype=np.uint32)
            cell_by_raw[indices[:, 1]] = indices[:, 0]
            assert np.count_nonzero(cell_by_raw != np.iinfo(np.uint32).max) == view.input_num
            record['cell_membership_sha256'] = hashlib.sha256(cell_by_raw.tobytes()).hexdigest()
            record['cell_sequence_sha256'] = hashlib.sha256(indices[:, 0].tobytes()).hexdigest()
            record['ranges_sha256'] = hashlib.sha256(ranges.tobytes()).hexdigest()
            record['centroids_sha256'] = hashlib.sha256(centroids.tobytes()).hexdigest()
        trial_num = ct.c_uint32()
        trial_pointer = lib.gng_get_trial_inputs(ct.byref(trial_num))
        trial_inputs = np.frombuffer(ct.string_at(trial_pointer, trial_num.value * 12), dtype=np.float32).reshape(-1, 3)
        if frame_idx == 0 or args.enable_features:
            assert trial_num.value == 4000
            record['learning_inputs_sha256'] = hashlib.sha256(trial_inputs.tobytes()).hexdigest()
        if frame_idx == 0 and not args.enable_features:
            lib.gng_set_trial_input_capture(0)
        if frame_idx % args.quality_every == 0 or frame_idx == args.frames - 1:
            # 評価用だけの範囲・非有限値・原点除外。学習へ渡す入力は元の全点。
            input_points = np.ndarray((num_points, 3), dtype=np.float32, buffer=data, strides=(config.point_step, 4))
            is_valid = np.isfinite(input_points).all(axis=1) & np.any(input_points != 0, axis=1)
            for axis, name in enumerate('xyz'):
                is_valid &= (input_points[:, axis] >= params[f'input.{name}_min']) & (input_points[:, axis] <= params[f'input.{name}_max'])
            quality_points = input_points[is_valid]
            distances = cKDTree(nodes['pos']).query(quality_points)[0]
            assert len(distances) > 0 and np.isfinite(distances).all()
            record['quality'] = dict(num_points=len(distances), mean_dist=float(np.mean(distances)),
                p95_dist=float(np.percentile(distances, 95)), coverage_0_2=float(np.mean(distances <= 0.2)),
                coverage_0_4=float(np.mean(distances <= 0.4)))
        if (args.enable_features or args.enable_observation) and (frame_idx % args.quality_every == 0 or frame_idx == args.frames - 1):
            observation_digest = hashlib.sha256()
            yaw_spans, pitch_spans = [], []
            for node_id in nodes['id']:
                bounds = lib.gng_get_observation_angle_range(int(node_id))
                observation_digest.update(digest_structure(bounds))
                if bounds.has_support:
                    pitch_spans.append((int(bounds.max_pitch) - int(bounds.min_pitch) + 1) * 180 / 65536)
                    if bounds.has_yaw:
                        yaw_spans.append(((int(bounds.max_yaw) - int(bounds.min_yaw)) % 65536 + 1) * 360 / 65536)
            assert pitch_spans and yaw_spans
            record['observation_sha256'] = observation_digest.hexdigest()
            record['observation'] = dict(supported_num=len(pitch_spans), supported_ratio=len(pitch_spans) / result.node_num,
                mean_yaw_span_deg=float(np.mean(yaw_spans)), mean_pitch_span_deg=float(np.mean(pitch_spans)),
                p95_yaw_span_deg=float(np.percentile(yaw_spans, 95)), p95_pitch_span_deg=float(np.percentile(pitch_spans, 95)))
        if frame_idx in (0, args.frames - 1):
            np.savez_compressed(str(Path(args.output).with_suffix('')) + f'_frame_{frame_idx}.npz',
                indices=indices, ranges=ranges, centroids=centroids, learning_inputs=trial_inputs,
                representative_points=np.ndarray((num_points, 3), dtype=np.float32, buffer=data, strides=(config.point_step, 4))[indices[ranges[:, 1] - 1 if args.enable_last_representative else ranges[:, 0], 1]],
                node_ids=nodes['id'], positions=nodes['pos'], labels=nodes['label'], edges=edges,
                input_labels=np.frombuffer(ct.string_at(labels, label_num.value), dtype=np.uint8))

        if args.enable_features:
            event_num = ct.c_uint32()
            events = lib.gng_getTrainingEvents(ct.byref(event_num))
            record['events_sha256'] = hashlib.sha256(ct.string_at(events, event_num.value * ct.sizeof(training_event))).hexdigest()
            record['events_num'] = event_num.value
            delta = lib.gng_getTopologicalMapDelta().contents
            delta_digest = hashlib.sha256(ct.string_at(delta.nodes, delta.node_num * 12))
            delta_digest.update(ct.string_at(delta.edges, delta.edge_num * 20))
            record['delta_sha256'] = delta_digest.hexdigest()
            stats_digest, observation_digest = hashlib.sha256(), hashlib.sha256()
            for node_id in nodes['id']:
                stats_digest.update(digest_structure(lib.gng_get_node_statistics(int(node_id))))
                observation_digest.update(digest_structure(lib.gng_get_observation_angle_range(int(node_id))))
            record['statistics_sha256'] = stats_digest.hexdigest()
            record['observation_sha256'] = observation_digest.hexdigest()
        records.append(record)
    summary = {name: float(np.mean([record[name] for record in records[args.warmup:]]))
               for name in records[-1] if name.endswith('_ms')}
    Path(args.output).write_text(json.dumps(dict(library=args.library, config=args.config, voxel=args.voxel,
        has_features=args.enable_features, has_observation=args.enable_observation, seed=args.seed,
        has_last_representative=args.enable_last_representative, warmup=args.warmup, accepted=accepted, ignored=ignored,
        records=records, mean=summary), indent=2) + '\n')
    print(json.dumps({'output': args.output, 'frames': len(records), 'mean': summary}), flush=True)


if __name__ == '__main__':
    main()
