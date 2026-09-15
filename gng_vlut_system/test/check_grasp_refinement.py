"""隔離ドメインでの把持幅・IK・失効と、既存出力への非干渉の有限検証。"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as xml

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry_msgs.msg import TransformStamped
from gng_control_msgs.msg import (
    GraspCandidate, GraspCandidateArray, GraspCandidateMetric,
    GraspCandidateMetricArray, GraspRefinementArray,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross @ cross)


def quaternion(matrix):
    # 回転行列の固有ベクトルによる、符号反転に依存しないテスト姿勢の構築
    values, vectors = np.linalg.eigh(np.array([
        [matrix[0, 0]-matrix[1, 1]-matrix[2, 2], matrix[0, 1]+matrix[1, 0], matrix[0, 2]+matrix[2, 0], matrix[2, 1]-matrix[1, 2]],
        [matrix[0, 1]+matrix[1, 0], matrix[1, 1]-matrix[0, 0]-matrix[2, 2], matrix[1, 2]+matrix[2, 1], matrix[0, 2]-matrix[2, 0]],
        [matrix[0, 2]+matrix[2, 0], matrix[1, 2]+matrix[2, 1], matrix[2, 2]-matrix[0, 0]-matrix[1, 1], matrix[1, 0]-matrix[0, 1]],
        [matrix[2, 1]-matrix[1, 2], matrix[0, 2]-matrix[2, 0], matrix[1, 0]-matrix[0, 1], np.trace(matrix)],
    ]))
    return vectors[:, np.argmax(values)]


def fixture():
    root = xml.parse('/ros2_ws/src/dual_arm_urdf/dual_arm_robot.urdf').getroot()
    parents = {joint.find('child').attrib['link']: joint for joint in root.findall('joint')}
    path, link = [], 'L_tcp'
    while link != 'L_shoulder_mount':
        joint = parents[link]
        path.append(joint)
        link = joint.find('parent').attrib['link']
    values = [.2, .5, -.4, .7, .2, -.6, .1]
    names, idx = [], 0
    target = np.eye(4)
    for joint in reversed(path):
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()]
        angles = [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()]
        local = np.eye(4)
        local[:3, 3] = xyz
        local[:3, :3] = rotation([0, 0, 1], angles[2]) @ rotation([0, 1, 0], angles[1]) @ rotation([1, 0, 0], angles[0])
        target = target @ local
        if joint.attrib['type'] == 'revolute':
            names.append(joint.attrib['name'])
            move = np.eye(4)
            move[:3, :3] = rotation([float(v) for v in joint.find('axis').attrib['xyz'].split()], values[idx])
            target = target @ move
            idx += 1
    assert idx == 7
    points = [target[:3, :3] @ np.array([x*.004, side*.020, .010+z*.004])+target[:3, 3]
              for side in (-1, 1) for x in range(-5, 6) for z in range(8)]
    source = GraspCandidateArray()
    source.header.frame_id = 'refinement_test_root'
    source.tcp_frame = 'L_tcp'
    candidate = GraspCandidate(id=101, state=GraspCandidate.INSIDE, shape_score=.5)
    center = target[:3, 3]+target[:3, 2]*.04
    candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z = map(float, center)
    candidate.pose.orientation.x, candidate.pose.orientation.y, candidate.pose.orientation.z, candidate.pose.orientation.w = map(float, quaternion(target[:3, :3] @ rotation([1, 0, 0], math.pi)))
    source.candidates.append(candidate)
    seeds = GraspCandidateMetricArray()
    seed = GraspCandidateMetric(goal_node_id=123, feasible=True)
    seed.final_joint_state.name = names
    seed.final_joint_state.position = values.copy()
    seed.final_joint_state.position[2] += .025
    seeds.candidates.append(seed)
    return source, seeds, points, target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '218':
        raise RuntimeError('ROS_DOMAIN_ID=218 is required')
    rclpy.init()
    node = rclpy.create_node('grasp_refinement_test_driver')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    prefix = '/grasp_refinement_test'
    source, seeds, points, target = fixture()
    source_pub = node.create_publisher(GraspCandidateArray, prefix+'/source', qos)
    seed_pub = node.create_publisher(GraspCandidateMetricArray, prefix+'/seeds', qos)
    from sensor_msgs.msg import PointCloud2
    cloud_pub = node.create_publisher(PointCloud2, prefix+'/points', 1)
    received, markers, echoed = [], [], []
    node.create_subscription(GraspRefinementArray, prefix+'/result', received.append, qos)
    node.create_subscription(MarkerArray, prefix+'/result/markers', markers.append, qos)
    node.create_subscription(GraspCandidateArray, prefix+'/source', echoed.append, qos)
    static_tf = StaticTransformBroadcaster(node)
    process = None
    with tempfile.TemporaryDirectory(prefix='grasp_refinement_') as temporary:
        params = Path(temporary)/'params.yaml'
        params.write_text('/**:\n  ros__parameters:\n    root_frame: refinement_test_root\n    update_hz: 20.0\n    max_input_age_sec: 0.8\n    max_stamp_diff_sec: 0.25\n')
        command = ['ros2', 'launch', 'gng_vlut_system', 'grasp_candidate_refinement.launch.py',
                   'params_file:='+str(params), 'candidate_topic:='+prefix+'/source',
                   'seed_topic:='+prefix+'/seeds', 'point_cloud_topic:='+prefix+'/points',
                   'output_topic:='+prefix+'/result']
        print('START '+' '.join(command), flush=True)
        with open(args.output+'.log', 'w') as log:
            try:
                process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=source.header.frame_id), points)
                def wait_for_log(text):
                    deadline = time.monotonic()+8
                    while time.monotonic() < deadline:
                        assert process.poll() is None, 'refiner exited'
                        if text in Path(args.output+'.log').read_text():
                            return
                        rclpy.spin_once(node, timeout_sec=.02)
                    raise RuntimeError('missing status log: '+text)

                def wait_for_markers(out):
                    deadline = time.monotonic()+2
                    while time.monotonic() < deadline:
                        for message in reversed(markers):
                            if message.markers and message.markers[0].header == out.header:
                                assert message.markers[0].action == 3
                                assert all(m.type != 9 and not m.text for m in message.markers)
                                return [m for m in message.markers if m.action == 0]
                        rclpy.spin_once(node, timeout_sec=.02)
                    raise RuntimeError('missing geometry markers')

                def check_geometry(out):
                    shapes = wait_for_markers(out)
                    candidate = out.candidates[0]
                    arrow = next(m for m in shapes if m.ns == 'refined_approach')
                    assert arrow.type == 0 and arrow.header == out.header
                    local_arrow = np.array([[p.x, p.y, p.z] for p in arrow.points])
                    np.testing.assert_allclose(local_arrow, [[0, 0, .1483], [0, 0, 0]], atol=1e-9)
                    width = candidate.contact_width if candidate.has_contact_pair else candidate.observed_width
                    if not math.isfinite(width) or width <= 0:
                        assert len(shapes) == 1 and arrow.color.r == arrow.color.g == arrow.color.b
                        q = arrow.pose.orientation
                        original = candidate.source_pose.orientation
                        np.testing.assert_allclose([q.x, q.y, q.z, q.w],
                            [original.w, original.z, -original.y, -original.x], atol=1e-9)
                        return shapes
                    frame = next(m for m in shapes if m.ns == 'refined_gripper')
                    assert frame.type == 5 and frame.pose == arrow.pose
                    assert len(frame.points) == (20 if candidate.has_contact_pair else 120)
                    local = np.array([[p.x, p.y, p.z] for p in frame.points])
                    np.testing.assert_allclose(local.max(axis=0)-local.min(axis=0), [.061, width, .0883], atol=1e-9)
                    for value in (frame.pose, arrow.pose):
                        np.testing.assert_allclose([value.position.x, value.position.y, value.position.z],
                            [candidate.refined_pose.position.x, candidate.refined_pose.position.y, candidate.refined_pose.position.z], atol=1e-9)
                        q = value.orientation
                        expected = candidate.refined_pose.orientation
                        np.testing.assert_allclose([q.x, q.y, q.z, q.w], [expected.x, expected.y, expected.z, expected.w], atol=1e-9)
                    return shapes

                def wait_for(predicate, label, publish=True, max_sec=8, has_cloud=True):
                    deadline = time.monotonic()+max_sec
                    next_publish = 0
                    while time.monotonic() < deadline:
                        assert process.poll() is None, 'refiner exited: '+label
                        if publish and time.monotonic() >= next_publish:
                            stamp = node.get_clock().now().to_msg()
                            source.header.stamp = cloud.header.stamp = stamp
                            source_pub.publish(source); seed_pub.publish(seeds)
                            if has_cloud:
                                cloud_pub.publish(cloud)
                            next_publish = time.monotonic()+.12
                        rclpy.spin_once(node, timeout_sec=.02)
                        if received and predicate(received[-1]):
                            return received[-1]
                    detail = [(c.reason, c.contact_width, c.position_error) for c in received[-1].candidates] if received else []
                    raise RuntimeError(label+': '+repr(detail))

                wait_for_log('waiting_for_candidates')
                assert not received
                no_cloud = wait_for(lambda out: out.candidates[0].reason == 'no_point_cloud', 'missing cloud', has_cloud=False)
                check_geometry(no_cloud)
                source.update_id = 1
                first = wait_for(lambda out: out.source_update_id == 1 and out.candidates[0].has_joint_solution, 'contact and IK')
                result = first.candidates[0]
                assert result.source_candidate_id == 101 and result.seed_node_id == 123
                assert result.has_contact_pair and result.has_gripper_check and not result.has_observed_collision
                first_shapes = check_geometry(first)
                assert abs(result.contact_width-.04) < 1e-5 and abs(result.opening_width-.046) < 1e-5
                assert result.position_error <= .002 and result.orientation_error_deg <= 3
                assert not result.has_arm_path_check and result.reason == 'refined_arm_path_unchecked'
                assert abs(result.observed_width-.04) < 1e-5
                achieved = np.array([result.joint_pose.position.x, result.joint_pose.position.y, result.joint_pose.position.z])
                assert abs(np.linalg.norm(achieved-target[:3, 3])-result.position_error) < 1e-6
                joints = dict(zip(result.joint_state.name, result.joint_state.position))
                assert abs(joints['L_gripper_joint']-.023) < 1e-5
                assert abs(joints['L_gripper_mimic']+.023) < 1e-5
                assert max(abs(joints[name]-value) for name, value in zip(seeds.candidates[0].final_joint_state.name, seeds.candidates[0].final_joint_state.position)) > 1e-4
                assert np.linalg.norm(np.array([result.refined_pose.position.x, result.refined_pose.position.y, result.refined_pose.position.z])-target[:3, 3]) < 1e-5
                assert echoed and serialize_message(echoed[-1]) == serialize_message(source)
                publishers = node.get_publisher_names_and_types_by_node('grasp_candidate_refiner', '/')
                assert {name for name, _ in publishers} <= {prefix+'/result', prefix+'/result/markers', '/rosout', '/parameter_events'}
                assert node.count_publishers(prefix+'/source') == 1
                source.update_id = 2
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=source.header.frame_id), points[:len(points)//2]+[points[-1]])
                missing = wait_for(lambda out: out.source_update_id == 2 and not out.candidates[0].has_contact_pair, 'missing side')
                assert math.isnan(missing.candidates[0].contact_width)
                assert math.isfinite(missing.candidates[0].observed_width)
                check_geometry(missing)
                source.update_id = 3
                obstacle = target[:3, :3] @ np.array([0, 0, .10])+target[:3, 3]
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=source.header.frame_id), points+[obstacle])
                blocked = wait_for(lambda out: out.source_update_id == 3 and out.candidates[0].has_observed_collision, 'base collision')
                assert not blocked.candidates[0].has_joint_solution
                blocked_shapes = check_geometry(blocked)
                assert all(m.color.r > m.color.g for m in blocked_shapes)
                source.update_id = 4
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='refinement_camera'), points)
                wait_for(lambda out: out.source_update_id == 4 and out.candidates[0].reason == 'cloud_transform_unavailable', 'missing TF')
                transform = TransformStamped()
                transform.header.frame_id = source.header.frame_id
                transform.child_frame_id = cloud.header.frame_id
                transform.transform.rotation.w = 1.0
                static_tf.sendTransform(transform)
                recovered = wait_for(lambda out: out.source_update_id == 4 and out.candidates[0].has_joint_solution, 'TF recovery')
                stale = wait_for(lambda out: out.candidates[0].reason == 'stale_input', 'stale invalidation', publish=False)
                assert not stale.candidates[0].has_joint_solution and not stale.candidates[0].has_contact_pair
                source.update_id = 5
                source.candidates.append(copy.deepcopy(source.candidates[0]))
                duplicate = wait_for(lambda out: out.source_update_id == 5 and all(c.reason == 'duplicate_candidate_id' for c in out.candidates), 'duplicate IDs')
                assert len(duplicate.candidates) == 2
                source.update_id = 6
                source.candidates = source.candidates[:1]
                orientation = copy.deepcopy(source.candidates[0].pose.orientation)
                for value in (0.0, 1e308):
                    source.candidates[0].pose.orientation.x = value
                    source.candidates[0].pose.orientation.y = value
                    source.candidates[0].pose.orientation.z = value
                    source.candidates[0].pose.orientation.w = value
                    invalid = wait_for(lambda out: out.source_update_id == source.update_id and out.candidates[0].reason == 'invalid_pose', 'invalid quaternion')
                    assert not wait_for_markers(invalid)
                    source.update_id += 1
                source.candidates[0].pose.orientation = orientation
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=source.header.frame_id), points)
                # 行末paddingを含む二行の組織化点群による、幅・IKの一致確認
                cloud.height = 2
                cloud.width //= 2
                row_bytes = cloud.width*cloud.point_step
                data = bytes(cloud.data)
                cloud.row_step = row_bytes+16
                cloud.data = data[:row_bytes]+bytes(16)+data[row_bytes:]+bytes(16)
                padded = wait_for(lambda out: out.source_update_id == source.update_id and out.candidates[0].has_joint_solution, 'row padding')
                assert abs(padded.candidates[0].contact_width-.04) < 1e-5
                source.update_id += 1
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=source.header.frame_id), points*200)
                budget = wait_for(lambda out: out.source_update_id == source.update_id and
                                  out.candidates[0].reason == 'local_point_budget', 'local point budget')
                assert math.isnan(budget.candidates[0].observed_width)
                assert not budget.candidates[0].has_contact_pair and not budget.candidates[0].has_joint_solution
                check_geometry(budget)
                source.update_id += 1
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=source.header.frame_id), points)
                seeds.candidates = []
                unseeded = wait_for(lambda out: out.source_update_id == source.update_id and out.candidates[0].reason == 'no_joint_seeds', 'missing seeds')
                assert unseeded.candidates[0].has_contact_pair and not unseeded.candidates[0].has_joint_solution
                source.update_id += 1
                source.candidates = []
                wait_for(lambda out: out.source_update_id == source.update_id and not out.candidates, 'empty invalidation')
                deadline = time.monotonic()+.25
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                assert markers and all(m.action != 0 for m in markers[-1].markers)
                wait_for_log('no_candidates')
                log_text = Path(args.output+'.log').read_text()
                log_times = [float(v) for v in re.findall(r'\[INFO\] \[(\d+\.\d+)\].*Refine: ', log_text)]
                assert len(log_times) >= 2 and all(b-a >= 4.9 for a, b in zip(log_times, log_times[1:]))
                payload = {'result': 'passed', 'first': message_to_ordereddict(first),
                           'recovered': message_to_ordereddict(recovered),
                           'first_markers': [message_to_ordereddict(m) for m in first_shapes],
                           'checks': ['40/46 mm widths', 'IK correction', 'gripper joints', 'no old publishers',
                                      'input unchanged', 'occlusion', 'collision', 'TF loss/recovery', 'stale', 'duplicates',
                                      'invalid quaternion', 'row padding', 'missing seeds', 'empty',
                                      'waiting log', 'uncomputed direction only', 'measured gripper dimensions',
                                      'refined orientation', 'dashed observation', 'collision color', 'no text', 'status log rate']}
                Path(args.output).write_text(json.dumps(payload, indent=2))
                print(json.dumps({'result': 'passed', 'first_ms': first.update_ms, 'checks': payload['checks']}), flush=True)
            finally:
                if process:
                    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                        if process.poll() is not None:
                            break
                        os.killpg(process.pid, sig)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            pass
                    assert process.poll() is not None
                    print(f'STOPPED pid={process.pid} returncode={process.poll()}', flush=True)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
