#!/usr/bin/env python3
"""隔離Gazeboでの深度画像・点群・取付TF・首追従・回避入力の有限検証。"""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import traceback
import xml.etree.ElementTree as et

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from check_gazebo_software_stop import owned_launch, port_is_listening, process_snapshot, save_json


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    begin = time.monotonic()
    baseline = process_snapshot()
    launch = owned_launch(args.output, baseline)
    report = {'result': 'failed', 'robot': args.robot, 'checks': {}, 'hardware_output': 'not_connected'}
    save_json(args.output/'baseline_processes.json', list(baseline.values()))
    node = rclpy = None
    cancel_state = {'is_cancelled': False}

    def cancel(_signum, _frame):
        cancel_state['is_cancelled'] = True

    handlers = {item: signal.signal(item, cancel) for item in (signal.SIGINT, signal.SIGTERM)}
    try:
        if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise ValueError('隔離domain96とlocalhost限定設定が必要です')
        if port_is_listening():
            raise RuntimeError('専用Gazebo port11369が使用中です')
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from rclpy.time import Time
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2, JointState
        from std_msgs.msg import String
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from gazebo_msgs.srv import SpawnEntity
        from tf2_ros import Buffer, TransformListener, TransformException
        from voxel_msgs.msg import Voxel

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('head_depth_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        namespace = 'sim_topo_dual_arm_'+args.robot
        frames, positions, gng, voxel_samples = {}, {}, {}, []
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        next_track_sec = 0.0

        def graph_names():
            return sorted(ns.rstrip('/')+'/'+name for name, ns in node.get_node_names_and_namespaces()
                          if name != node.get_name())

        def wait(predicate, max_wait_sec=60):
            nonlocal next_track_sec
            deadline = min(begin+args.timeout_sec, time.monotonic()+max_wait_sec)
            while time.monotonic() < deadline:
                if cancel_state['is_cancelled']:
                    raise RuntimeError('試験中断要求')
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() >= next_track_sec:
                    launch.track()
                    next_track_sec = time.monotonic()+1
                if predicate():
                    return
                if launch.process is not None and launch.process.poll() is not None:
                    raise RuntimeError('Gazebo launchの予期しない終了')
            raise TimeoutError('待機上限: '+report.get('stage', 'preflight'))

        def stage(name):
            report['stage'] = name
            print(json.dumps({'robot': args.robot, 'stage': name}), flush=True)

        def on_frame(kind, message):
            stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            frames.setdefault(stamp, {})[kind] = message
            for old in sorted(frames)[:-12]:
                del frames[old]

        for suffix, kind, msg_type in (('camera/depth/points', 'cloud', PointCloud2),
                                      ('camera/depth/image_raw', 'depth', Image),
                                      ('camera/depth/camera_info', 'info', CameraInfo)):
            node.create_subscription(msg_type, '/'+namespace+'/'+suffix,
                                     lambda msg, key=kind: on_frame(key, msg), qos_profile_sensor_data)
        node.create_subscription(JointState, '/'+namespace+'/joint_states',
                                 lambda msg: positions.update(zip(msg.name, msg.position)), qos_profile_sensor_data)
        node.create_subscription(String, '/'+namespace+'/avoidance/gng_status',
                                 lambda msg: gng.update(json.loads(msg.data)), 10)
        node.create_subscription(Voxel, '/'+namespace+'/self_filter_roi_voxels',
                                 lambda msg: voxel_samples.append((msg.header.frame_id, len(msg.data))), 10)
        command_pub = node.create_publisher(JointTrajectory, '/'+namespace+'/dual_arm_controller/joint_trajectory', 10)
        spawn = node.create_client(SpawnEntity, '/spawn_entity')
        discovery_end = time.monotonic()+1.5
        wait(lambda: time.monotonic() >= discovery_end, 3)
        report['baseline_ros_nodes'] = graph_names()
        if report['baseline_ros_nodes']:
            raise RuntimeError('専用domainに既存ノードあり')
        package = Path(__file__).resolve().parents[1]
        params_file = package/'config'/('topo_dual_arm_'+args.robot+'.yaml')
        params = yaml.safe_load(params_file.read_text())['/**']['ros__parameters']
        root = et.parse(params['urdf_path']).getroot()
        joint_names = [item.get('name') for item in root.findall('joint')
                       if item.get('type') != 'fixed' and item.find('mimic') is None]
        demo = yaml.safe_load((package/'config/dual_arm_gazebo_demo.yaml').read_text())
        demo['dual_arm_gazebo_demo']['enable_viewer'] = False
        demo_path = args.output/'demo.yaml'
        demo_path.write_text(yaml.safe_dump(demo))
        command = ['ros2', 'launch', 'gng_vlut_system', 'dual_arm_gng_lidar_demo.launch.py',
                   'point_cloud_source:=head_depth', 'gui:=false', 'enable_auto_start:=false',
                   'gazebo_master_uri:=http://127.0.0.1:11369', 'params_file:='+str(params_file),
                   'demo_config:='+str(demo_path)]
        report['command'] = command
        launch.start(command)
        selected = {}

        def select_frame():
            for stamp, frame in sorted(frames.items(), reverse=True):
                if stamp <= 0 or not {'cloud', 'depth', 'info'} <= frame.keys():
                    continue
                try:
                    transform = buffer.lookup_transform('world', frame['cloud'].header.frame_id,
                                                        Time.from_msg(frame['cloud'].header.stamp))
                except TransformException:
                    continue
                selected.update(frame, transform=transform, stamp=stamp)
                return True
            return False

        def cloud_array(cloud):
            fields = {item.name: item for item in cloud.fields}
            return np.stack([np.ndarray((cloud.height, cloud.width), dtype='<f4', buffer=cloud.data,
                             offset=fields[name].offset, strides=(cloud.row_step, cloud.point_step))
                             for name in ('x', 'y', 'z')], axis=-1).reshape(-1, 3)

        def rotation_translation(transform):
            q, p = transform.transform.rotation, transform.transform.translation
            return Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])

        stage('depth_and_optical_tf')
        wait(lambda: select_frame() and len(positions) >= 19 and spawn.service_is_ready(), 110)
        frame = dict(selected)
        cloud, depth, info = frame['cloud'], frame['depth'], frame['info']
        assert all(msg.header.frame_id == namespace+'/head_depth_optical_frame' for msg in (cloud, depth, info))
        assert (depth.width, depth.height, depth.encoding) == (320, 240, '32FC1')
        assert (info.width, info.height) == (320, 240) and info.k[0] > 0 and info.k[4] > 0
        points = cloud_array(cloud)
        assert len(points) == depth.width*depth.height
        image = np.ndarray((depth.height, depth.width), dtype='<f4', buffer=depth.data,
                           strides=(depth.step, 4)).reshape(-1)
        is_finite = np.isfinite(points).all(axis=1) & np.isfinite(image)
        assert is_finite.any()
        np.testing.assert_allclose(points[is_finite, 2], image[is_finite], atol=1e-5)
        assert points[is_finite, 2].min() >= 0.1-1e-5 and points[is_finite, 2].max() <= 3+1e-5
        report['checks']['depth_cloud_stamp_and_values'] = True
        report['finite_points'] = int(is_finite.sum())
        before_rotation, before_translation = rotation_translation(frame['transform'])
        local = buffer.lookup_transform(namespace+'/camera_link', cloud.header.frame_id, Time.from_msg(cloud.header.stamp))
        local_rotation, local_translation = rotation_translation(local)
        np.testing.assert_allclose(local_translation, [0, 0, 0], atol=1e-6)
        np.testing.assert_allclose(local_rotation.apply([0, 0, 1]), [1, 0, 0], atol=1e-6)
        report['checks']['mount_and_optical_axes'] = True

        stage('known_object_coordinates')
        box_center = before_translation + before_rotation.apply([0, 0, 0.55])
        request = SpawnEntity.Request()
        request.name = 'head_depth_test_box'
        request.xml = '<sdf version="1.6"><model name="head_depth_test_box"><static>true</static><link name="body"><visual name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></visual><collision name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model></sdf>'
        request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = map(float, box_center)
        request.initial_pose.orientation.w = 1.0
        request.reference_frame = 'world'
        future = spawn.call_async(request)
        wait(future.done, 15)
        assert future.result().success, future.result().status_message

        def sees_box():
            if not select_frame():
                return False
            rotation, translation = rotation_translation(selected['transform'])
            points = cloud_array(selected['cloud'])
            points = points[np.isfinite(points).all(axis=1)]
            world_points = rotation.apply(points)+translation
            num_hits = int(np.all(np.abs(world_points-box_center) <= 0.053, axis=1).sum())
            report['box_points'] = num_hits
            return num_hits >= 20

        wait(sees_box, 25)
        report['checks']['optical_points_in_world'] = True
        stage('neck_motion')
        wait(lambda: command_pub.get_subscription_count() > 0)
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in joint_names]
        target = float(positions['neck_pan_joint'])+0.12
        point.positions[joint_names.index('neck_pan_joint')] = target
        point.time_from_start.sec = 3
        trajectory.points = [point]
        command_pub.publish(trajectory)
        wait(lambda: abs(positions['neck_pan_joint']-target) < 0.025, 35)
        wait(sees_box, 15)
        after_rotation, _ = rotation_translation(selected['transform'])
        change = float((after_rotation*before_rotation.inv()).magnitude())
        assert change > 0.06, change
        report['neck_rotation_rad'] = change
        report['checks']['neck_following_and_world_consistency'] = True
        stage('avoidance_input')
        wait(lambda: gng.get('num_cloud', 0) > 2 and gng.get('num_voxels', 0) > 0 and
             any(frame_id == namespace+'/base_link' and num > 0 for frame_id, num in voxel_samples), 30)
        report['gng_input'] = {key: gng.get(key) for key in ('num_cloud', 'num_voxels', 'cloud_age_sec')}
        report['checks']['self_filter_and_gng_input'] = True
        report['result'] = 'passed'
    except BaseException as error:
        report.update(error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    finally:
        try:
            report['cleanup'] = launch.cleanup()
            if node is not None and launch.process is not None:
                deadline = time.monotonic()+12
                while graph_names() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                report['remaining_ros_nodes'] = graph_names()
                report['cleanup']['is_success'] &= not graph_names() and not port_is_listening()
            if not report['cleanup']['is_success']:
                report.update(result='failed', cleanup_error='所有プロセスの終了未確認')
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            for item, handler in handlers.items():
                signal.signal(item, handler)
            after = process_snapshot()
            save_json(args.output/'after_processes.json', list(after.values()))
            report['wall_sec'] = time.monotonic()-begin
            save_json(args.output/'report.json', report)
            save_json(args.output/'metrics.json', {'is_success': int(report['result'] == 'passed'),
                      'is_cleanup_success': int(report.get('cleanup', {}).get('is_success', False)),
                      'num_checks': len(report['checks']), 'wall_sec': report['wall_sec']})
            print(json.dumps({'result': report['result'], 'error': report.get('error'),
                              'report': str(args.output/'report.json')}), flush=True)
    return int(report['result'] != 'passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['max', 'max_long'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=180)
    args = parser.parse_args()
    args.output = args.output.resolve()
    raise SystemExit(run(args))
