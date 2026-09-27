"""共有world点群・別座標ROI・FVG件数・freeze属性保持の隔離ROS検証。"""
import collections
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger
from composition_interfaces.srv import LoadNode
from rclpy.parameter import Parameter
from voxel_msgs.msg import Voxel
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
import yaml


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '181'
    output = Path(os.environ.get('FVG_TEST_OUTPUT', '/ros2_ws/src/artifacts/shared_voxels_20260926'))
    output.mkdir(parents=True, exist_ok=True)
    world = {'output_topic': '/shared_test/roi', 'world_bucket_topic': '/shared_test/buckets',
             'voxel_size': .05, 'bucket_size': .2, 'enable_reachability_filter': True,
             'allow_unconnected_source_as_world': False}
    settings = {'input_pointcloud_topic': '/shared_test/direct',
                'input_topological_map_topic': '/shared_test/map',
                'voxel_centers_topic': '/shared_test/centers', 'publish_voxel_centers': True,
                'filtered_new_points_topic': '/shared_test/filtered', 'update_rate_hz': 30.,
                'voxel_size_x': .1, 'voxel_size_y': .2, 'voxel_size_z': .15,
                'grid_origin_x': .025, 'grid_origin_y': -.04, 'grid_origin_z': .05,
                'use_exclusion_box': True, 'filter_target_label': 3,
                'filter_distance_threshold': .1, 'print_processing_time': False}
    for axis in 'xyz':
        world.update({f'min_reachability_{axis}': -1., f'max_reachability_{axis}': 1.,
                      f'reachability_margin_{axis}': 0.})
        settings.update({f'range_min_{axis}': -3., f'range_max_{axis}': 3.,
                         f'exclude_min_{axis}': -.05, f'exclude_max_{axis}': .05})
    if 'FVG_TEST_DENSE_LIMIT' in os.environ:
        settings['max_dense_voxel_num'] = int(os.environ['FVG_TEST_DENSE_LIMIT'])
    for name, params in [('world', world), ('fvg', settings)]:
        (output / f'{name}.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    rclpy.init()
    node = rclpy.create_node('shared_world_test')
    publisher = node.create_publisher(PointCloud2, '/shared_test/input', 1)
    direct_pub = node.create_publisher(PointCloud2, '/shared_test/direct', 1)
    map_pub = node.create_publisher(TopologicalMap, '/shared_test/map', 1)
    cache = {}
    snapshots = {}
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, key=key: cache.__setitem__(key, msg), qos_profile_sensor_data)
        for key, kind, topic in [('roi', Voxel, '/shared_test/roi'),
            ('buckets', Voxel, '/shared_test/buckets'), ('centers', PointCloud2, '/shared_test/centers'),
            ('filtered', PointCloud2, '/shared_test/filtered'), ('markers', MarkerArray, '/voxel_markers')]]
    broadcaster = StaticTransformBroadcaster(node)

    def transform(child, x, y, z, yaw):
        msg = TransformStamped(); msg.header.frame_id = 'world'; msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = x, y, z
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return msg

    broadcaster.sendTransform([transform('sensor', .45, -.2, .3, .6), transform('roi', -.2, .3, 0., -.4)])

    def apply(point, x, y, z, yaw):
        return (x + math.cos(yaw)*point[0] - math.sin(yaw)*point[1],
                y + math.sin(yaw)*point[0] + math.cos(yaw)*point[1], z + point[2])

    def f32(point):
        return struct.unpack('<fff', struct.pack('<fff', *point))

    def cloud(points, sec, frame='sensor'):
        msg = PointCloud2(); msg.header.frame_id = frame; msg.header.stamp.sec = sec
        msg.height, msg.width, msg.point_step = 1, len(points), 16
        msg.row_step = 16 * len(points)
        msg.fields = [PointField(name=name, offset=idx*4, datatype=7, count=1)
                      for idx, name in enumerate(('x', 'y', 'z', 'intensity'))]
        msg.data = b''.join(struct.pack('<ffff', *p) for p in points)
        return msg

    def spin_until(predicate, process, sec=15):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            assert process.poll() is None, '試験ノードの早期終了'
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate(): return
        raise AssertionError('待機条件の未達: ' + str(cache.keys()))

    def publish_until(publisher, msg, key, process):
        # BestEffort購読の初回discovery中の欠落への再送。
        timer = node.create_timer(.2, lambda: publisher.publish(msg))
        try:
            publisher.publish(msg)
            spin_until(lambda: key in cache and cache[key].header.stamp == msg.header.stamp, process)
        finally:
            node.destroy_timer(timer)

    def cells(msg):
        offsets = {field.name: field.offset for field in msg.fields}
        result = {}
        for idx in range(msg.width):
            pos = [struct.unpack_from('<f', msg.data, idx*msg.point_step + offsets[a])[0] for a in 'xyz']
            key = tuple(math.floor((v-o)/s) for v, o, s in zip(pos, (.025, -.04, .05), (.1, .2, .15)))
            result[key] = tuple(struct.unpack_from('<I', msg.data, idx*msg.point_step + offsets[a])[0]
                                for a in ('point_count', 'node_count'))
        return result

    def expected(points, bound=3.):
        result = collections.Counter()
        for p in points:
            if not all(math.isfinite(v) and -bound <= v <= bound for v in p): continue
            if all(-.05 <= v <= .05 for v in p): continue
            result[tuple(math.floor((v-o)/s) for v, o, s in zip(p, (.025, -.04, .05), (.1, .2, .15)))] += 1
        return dict(result)

    def capture(name):
        # 並び順に依存しない全フィールドの比較用記録。
        msg = cache['centers']
        snapshots[name] = {'frame': msg.header.frame_id, 'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec],
            'fields': [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
            'rows': sorted(bytes(msg.data[idx*msg.point_step:(idx+1)*msg.point_step]).hex()
                           for idx in range(msg.width))}
        spin_until(lambda: 'markers' in cache and len(cache['markers'].markers) == 4 and
            [m.id for m in cache['markers'].markers] == ([0, 1, 3, 2] if name == 'frozen' else [0, 1, 2, 3]) and
            all(m.header == msg.header for m in cache['markers'].markers) and
            sum(len(m.points) for m in cache['markers'].markers) == msg.width, process)
        snapshots[name]['markers'] = [{
            'id': m.id, 'ns': m.ns, 'type': m.type, 'action': m.action,
            'scale': [m.scale.x, m.scale.y, m.scale.z],
            'color': [m.color.r, m.color.g, m.color.b, m.color.a],
            'pose': [m.pose.position.x, m.pose.position.y, m.pose.position.z,
                     m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w],
            'points': sorted((p.x, p.y, p.z) for p in m.points)
        } for m in cache['markers'].markers]

    def packed_ids(points, size):
        # voxel codecと同じfloatのセル幅。
        size = struct.unpack('<f', struct.pack('<f', size))[0]
        return {sum((math.floor(v/size) + 1000000) << shift for v, shift in zip(p, (42, 21, 0))) for p in points}

    def stop(process):
        if process is None: return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if process.poll() is not None: break
            os.killpg(process.pid, sig)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired: pass
        assert process.poll() is not None
        print('STOP', process.pid, process.returncode, flush=True)

    command = ['ros2', 'launch', 'fuzzy_voxel_grid', 'shared_world_voxel.launch.py',
        'input_topic:=/shared_test/input', 'target_frame:=roi',
        'world_params_file:=' + str(output / 'world.yaml'), 'fvg_params_file:=' + str(output / 'fvg.yaml')]
    process = None
    try:
        with (output / 'ros_shared.log').open('w') as log:
            print('START', command, flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            spin_until(lambda: publisher.get_subscription_count() == 1 and map_pub.get_subscription_count() == 1, process)
            source = [(0.013, .127, .081, 71.), (.027, .135, .099, 82.),
                      (-.441, .039, -.23, 93.), (2.33, .17, .5, 104.), (float('nan'), 0., 0., 115.),
                      (*apply((-.45, .2, -.3), 0., 0., 0., -.6), 126.)]
            original = cloud(source, 101)
            world_points = [f32(apply(p, .45, -.2, .3, .6)) for p in [f32(s[:3]) for s in source]]
            publisher.publish(original)
            spin_until(lambda: all(key in cache and cache[key].header.stamp.sec == 101
                                   for key in ('roi', 'buckets', 'centers')), process)
            actual = cells(cache['centers'])
            assert {key: value[0] for key, value in actual.items()} == expected(world_points), actual
            assert all(v[1] == 0 for v in actual.values())
            local_points = [f32(apply((p[0]+.2, p[1]-.3, p[2]), 0., 0., 0., .4)) for p in world_points
                            if all(math.isfinite(v) for v in p)]
            local_points = [p for p in local_points if all(-1 <= v <= 1 for v in p)]
            assert set(cache['roi'].data) == packed_ids(local_points, .05)
            assert cache['roi'].header.frame_id == 'roi'
            finite = [p for p in world_points if all(math.isfinite(v) for v in p)]
            # 検索bucket幅はdouble、ROI幅は既存codecのfloat。
            expected_buckets = {sum((math.floor(v/.2)+1000000) << s for v, s in zip(p, (42, 21, 0))) for p in finite}
            assert set(cache['buckets'].data) == expected_buckets
            assert cache['centers'].header.frame_id == 'world'
            assert direct_pub.get_subscription_count() == 0 and publisher.get_subscription_count() == 1
            capture('shared_points')
            print('PASS shared ownership route, TF, anisotropic grid, origin, ROI, nonfinite', flush=True)
            topo = TopologicalMap(); topo.header.frame_id = 'world'; topo.header.stamp.sec = 101
            target = TopologicalNode(); target.pos.x, target.pos.y, target.pos.z = world_points[0]
            topo.nodes.append(target); map_pub.publish(topo)
            spin_until(lambda: sum(v[1] for v in cells(cache['centers']).values()) == 1, process)
            capture('shared_tmap')
            # 同一frameでも古過ぎる／未来のTmapは共有点群へ混在不可。
            for stamp in (98, 102):
                topo.header.stamp.sec = stamp; map_pub.publish(topo)
                spin_until(lambda: all(v[1] == 0 for v in cells(cache['centers']).values()), process)
                topo.header.stamp.sec = 101; map_pub.publish(topo)
                spin_until(lambda: sum(v[1] for v in cells(cache['centers']).values()) == 1, process)
            freeze = node.create_client(Trigger, '/freeze_voxel_state')
            resume = node.create_client(Trigger, '/resume_voxel_update')
            assert freeze.wait_for_service(timeout_sec=5) and resume.wait_for_service(timeout_sec=5)
            future = freeze.call_async(Trigger.Request())
            spin_until(future.done, process); assert future.result().success
            incoming = cloud([source[0], source[3]], 102)
            publisher.publish(incoming)
            spin_until(lambda: 'filtered' in cache and cache['filtered'].header.stamp.sec == 102, process)
            filtered = cache['filtered']
            assert filtered.header.frame_id == 'sensor' and filtered.width == 1
            assert bytes(filtered.data) == bytes(incoming.data[16:])
            assert cache['centers'].header.stamp.sec == 101
            capture('frozen')
            future = resume.call_async(Trigger.Request())
            spin_until(future.done, process); assert future.result().success
            spin_until(lambda: cache['centers'].header.stamp.sec == 102, process)
            capture('resumed')
            # 点群消失後のTmap単独セル、および同一セルのラベル・ノード数更新。
            publisher.publish(cloud([], 104))
            topo.header.stamp.sec = 104
            for num_nodes in (3, 1, 0):
                topo.nodes = []
                for idx in range(num_nodes):
                    item = TopologicalNode(); item.pos.x, item.pos.y, item.pos.z = world_points[0]
                    item.label = 255 if idx % 2 else 1
                    topo.nodes.append(item)
                map_pub.publish(topo)
                spin_until(lambda: cache['centers'].header.stamp.sec == 104 and
                    sum(v[0] for v in cells(cache['centers']).values()) == 0 and
                    sum(v[1] for v in cells(cache['centers']).values()) == num_nodes, process)
                assert cache['centers'].width == (1 if num_nodes else 0)
                capture(f'tmap_only_{num_nodes}')
            print('PASS node-only cells, changing labels/counts, removed cells', flush=True)
            topo.nodes = [target]
            topo.header.frame_id = 'wrong_frame'; map_pub.publish(topo)
            spin_until(lambda: all(v[1] == 0 for v in cells(cache['centers']).values()), process)
            publisher.publish(cloud([], 99))
            spin_until(lambda: cache['centers'].header.stamp.sec == 99 and cache['centers'].width == 0, process)
            print('PASS freeze/resume, intensity bytes, frame mismatch, empty input, time rewind', flush=True)
            # 10万点と空フレームの交互入力。旧snapshotの混在・バッファ再利用の回帰。
            large_source = [((idx % 100)*.04-2., ((idx//100) % 100)*.04-2.,
                             (idx//10000)*.15-.6, float(idx)) for idx in range(100000)]
            large_world = [f32(apply(f32(p[:3]), .45, -.2, .3, .6)) for p in large_source]
            large_expected = expected(large_world)
            large = cloud(large_source, 200)
            for idx in range(16):
                msg = large if idx % 2 == 0 else cloud([], 200+idx)
                msg.header.stamp.sec = 200+idx
                publish_until(publisher, msg, 'centers', process)
                actual = {key: value[0] for key, value in cells(cache['centers']).items()}
                assert actual == (large_expected if idx % 2 == 0 else {})
                if idx < 2: capture(f'large_{idx}')
            print('PASS 100000/0 points, 16 alternating frames, no stale counts', flush=True)
            # 全点のセル所属が変化する連続フレーム。旧セルの削除と新セルの追加。
            for idx in range(4):
                shifted = [(p[0]+.08*math.sin(idx), p[1]+.07*math.cos(idx), p[2]+.02*idx, p[3])
                           for p in large_source]
                shifted_world = [f32(apply(f32(p[:3]), .45, -.2, .3, .6)) for p in shifted]
                publish_until(publisher, cloud(shifted, 220+idx), 'centers', process)
                assert {key: value[0] for key, value in cells(cache['centers']).items()} == expected(shifted_world)
                capture(f'moving_{idx}')
            print('PASS moving points, four full-cell comparisons', flush=True)
            publisher.publish(cloud(source, 250, 'missing_sensor_tf'))
            deadline = time.monotonic() + .4
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            assert cache['centers'].header.stamp.sec == 223
            publish_until(publisher, cloud(source, 251), 'centers', process)
            assert {key: value[0] for key, value in cells(cache['centers']).items()} == expected(world_points)
            print('PASS missing TF rejection and valid TF recovery', flush=True)
            # 同一設定のFVGを追加しても、入力購読はworldの1件だけ。
            second_sub = node.create_subscription(PointCloud2, '/shared_test/second_centers',
                lambda msg: cache.__setitem__('second', msg), qos_profile_sensor_data)
            loader = node.create_client(LoadNode, '/shared_world_voxel/_container/load_node')
            assert loader.wait_for_service(timeout_sec=5)
            req = LoadNode.Request()
            req.package_name = 'fuzzy_voxel_grid'; req.plugin_name = 'fuzzy_voxel_grid::VoxelGridNode'
            req.node_name = 'second_fvg'; req.node_namespace = '/shared_second'
            second_settings = dict(settings, shared_point_store='world_points',
                voxel_centers_topic='/shared_test/second_centers', marker_array_topic='/shared_test/second_markers')
            req.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in second_settings.items()]
            future = loader.call_async(req); spin_until(future.done, process)
            assert future.result().success, future.result().error_message
            spin_until(lambda: map_pub.get_subscription_count() == 2, process)
            publish_until(publisher, cloud(source, 260), 'centers', process)
            spin_until(lambda: 'second' in cache and cache['second'].header.stamp.sec == 260, process)
            assert cells(cache['second']) == cells(cache['centers'])
            assert publisher.get_subscription_count() == 1 and direct_pub.get_subscription_count() == 0
            node.destroy_subscription(second_sub); node.destroy_client(loader)
            print('PASS two FVG consumers, same cell output, single input subscription', flush=True)
            stop(process); process = None
        # 少数バケットだけを訪問する狭いFVG範囲での全走査結果との一致。
        narrow = dict(settings)
        for axis in 'xyz': narrow.update({f'range_min_{axis}': -.6, f'range_max_{axis}': .6})
        narrow_path = output / 'narrow.yaml'
        narrow_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': narrow}}))
        command[-1] = 'fvg_params_file:=' + str(narrow_path)
        with (output / 'ros_narrow.log').open('w') as log:
            print('START', command, flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            spin_until(lambda: publisher.get_subscription_count() == 1, process)
            large.header.stamp.sec = 300
            publish_until(publisher, large, 'centers', process)
            assert {key: value[0] for key, value in cells(cache['centers']).items()} == expected(large_world, .6)
            capture('narrow_shared')
            print('PASS narrow FVG bucket query against full-scan reference', flush=True)
            stop(process); process = None
        # 従来の独立点群入力も、共有版と同じ件数・空入力結果。
        command = [os.environ.get('FVG_TEST_PREFIX', '/ros2_ws/install/fuzzy_voxel_grid') + '/lib/fuzzy_voxel_grid/voxel_grid_node',
                   '--ros-args', '--params-file', str(output / 'fvg.yaml')]
        with (output / 'ros_standalone.log').open('w') as log:
            print('START', command, flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            spin_until(lambda: direct_pub.get_subscription_count() == 1, process)
            publish_until(direct_pub, cloud([(*p, 1.) for p in world_points], 103, 'world'), 'centers', process)
            assert {key: value[0] for key, value in cells(cache['centers']).items()} == expected(world_points)
            capture('standalone')
            direct_pub.publish(cloud([], 104, 'world'))
            spin_until(lambda: cache['centers'].header.stamp.sec == 104 and cache['centers'].width == 0, process)
            print('PASS standalone compatibility', flush=True)
            stop(process); process = None
        # FVGなし・共有OFFの既存worldノードによるROI出力の互換性。
        command = ['/ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/world_index_to_voxel_node',
                   '--ros-args', '--params-file', str(output / 'world.yaml'),
                   '-p', 'input_topic:=/shared_test/input', '-p', 'target_frame_id:=roi']
        with (output / 'ros_world_only.log').open('w') as log:
            print('START', command, flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            spin_until(lambda: publisher.get_subscription_count() == 1, process)
            publish_until(publisher, cloud(source, 105), 'roi', process)
            assert set(cache['roi'].data) == packed_ids(local_points, .05)
            print('PASS world-only compatibility, fuzzy disabled', flush=True)
        (output / 'snapshots.json').write_text(json.dumps(snapshots, sort_keys=True))
    finally:
        stop(process)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
