"""隔離ROSドメインでの把持重点入力・TF・失効・既定OFFの検証。"""

import os
import signal
import struct
import subprocess
import tempfile
import time


def main():
    os.environ.update(ROS_DOMAIN_ID='219', ROS_LOCALHOST_ONLY='1')
    import rclpy
    from ais_gng_msgs.msg import TopologicalCluster, TopologicalMap, TopologicalNode
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('grasp_attention_test')
    cloud_pub = node.create_publisher(PointCloud2, '/attention_test_points', qos_profile_sensor_data)
    candidate_pub = node.create_publisher(TopologicalMap, '/grasp_pose_cands/Tmap', 1)
    received = []
    sub = node.create_subscription(TopologicalMap, '/topological_map', received.append, qos_profile_sensor_data)
    sampled = []
    sampled_sub = None
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'attention_sensor'
    transform.child_frame_id = 'attention_candidates'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    def spin(sec):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait(predicate, sec=25):
        deadline = time.monotonic() + sec
        while not predicate():
            assert time.monotonic() < deadline, 'ROS応答の時間超過'
            spin(0.05)

    def stop(process):
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

    def make_candidate(frame='attention_sensor', x=0.7, stamp=None):
        message = TopologicalMap()
        message.header.frame_id = frame
        message.header.stamp = stamp or node.get_clock().now().to_msg()
        vertex = TopologicalNode()
        vertex.pos.x, vertex.pos.y, vertex.pos.z = float(x), 0.0, 0.2
        message.nodes = [vertex]
        message.clusters = [TopologicalCluster(nodes=[vertex.id])]
        return message

    def publish_cloud(stamp=None):
        cloud = PointCloud2()
        cloud.header.frame_id = 'attention_sensor'
        cloud.header.stamp = stamp or node.get_clock().now().to_msg()
        cloud.height, cloud.width = 1, 400
        cloud.fields = [PointField(name=axis, offset=idx * 4, datatype=7, count=1)
                        for idx, axis in enumerate('xyz')]
        cloud.point_step, cloud.row_step = 12, 4800
        cloud.data = b''.join(struct.pack('<fff', 0.6 + x * 0.01, -0.1 + y * 0.01, 0.2)
                              for x in range(20) for y in range(20))
        num_before = len(received)
        cloud_pub.publish(cloud)
        wait(lambda: len(received) > num_before)
        return cloud

    process = None
    with tempfile.TemporaryFile() as log:
        def output():
            return os.pread(log.fileno(), 16 * 1024 * 1024, 0).decode(errors='replace')

        def start(enable_focus):
            command = ['/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu', '--ros-args',
                       '--params-file', '/ros2_ws/src/ais_gng_cpu/src/ais_gng/config/gng_cpu/graspnet.yaml',
                       '--log-level', 'ais_gng_node:=debug']
            params = {'enable_grasp_attention': str(enable_focus).lower(), 'node.num_max': '256',
                      'grasp_attention.margin': '0.035',
                      'node.learning_num': '200', 'input.point_cloud_num': '5000',
                      'input.topic_names': '[/attention_test_points]',
                      'classify.human': 'false', 'classify.car': 'false',
                      'node.enable_observation_support': 'false'}
            for name, value in params.items():
                command += ['-p', f'{name}:={value}']
            return subprocess.Popen(command, stdout=log, stderr=log)

        def case(candidate, has_focus, stamp=None):
            before = output().count('Grasp attention selected')
            num_before = len(sampled)
            candidate_pub.publish(candidate)
            spin(0.08)
            cloud = publish_cloud(stamp)
            spin(0.02)
            assert (output().count('Grasp attention selected') > before) == has_focus
            if sampled_sub is not None:
                wait(lambda: len(sampled) > num_before)
                result = sampled[-1]
                assert result.header == cloud.header
                assert result.height == 1 and result.point_step == 12
                assert result.row_step == result.width * 12 == len(result.data)
                assert not result.is_bigendian
                assert [(field.name, field.offset, field.datatype, field.count) for field in result.fields] == [
                    (axis, idx * 4, PointField.FLOAT32, 1) for idx, axis in enumerate('xyz')]
                expected = []
                if has_focus:
                    # 候補座標のTF適用とクラスタ別AABBによる期待点群。
                    nodes = {vertex.id: (vertex.pos.x + (1.0 if candidate.header.frame_id == 'attention_candidates' else 0.0),
                                          vertex.pos.y, vertex.pos.z) for vertex in candidate.nodes}
                    for point in struct.iter_unpack('<fff', bytes(cloud.data)):
                        if any(all(min(nodes[idx][axis] for idx in cluster.nodes) - 0.035 <= point[axis] <=
                                   max(nodes[idx][axis] for idx in cluster.nodes) + 0.035 for axis in range(3))
                               for cluster in candidate.clusters):
                            expected.append(point)
                assert list(struct.iter_unpack('<fff', bytes(result.data))) == expected
                assert (result.width > 0) == has_focus

        try:
            process = start(False)
            wait(lambda: cloud_pub.get_subscription_count() > 0 or process.poll() is not None)
            assert process.poll() is None
            assert candidate_pub.get_subscription_count() == 0
            assert node.count_publishers('/downsampling/grasp') == 0
            case(make_candidate(), False)
            stop(process)
            wait(lambda: cloud_pub.get_subscription_count() == 0)
            print('PASS: OFF時の候補購読なし、通常Graph配信')

            process = start(True)
            wait(lambda: (cloud_pub.get_subscription_count() > 0 and candidate_pub.get_subscription_count() > 0)
                 or process.poll() is not None)
            assert process.poll() is None
            spin(0.5)
            for _ in range(3):
                case(make_candidate(), True)
            print('PASS: 可視化購読なしでの重点学習継続')
            sampled_sub = node.create_subscription(
                PointCloud2, '/downsampling/grasp', sampled.append, qos_profile_sensor_data)
            wait(lambda: node.count_publishers('/downsampling/grasp') == 1)
            spin(0.5)
            case(make_candidate(), True)
            case(make_candidate('attention_candidates', -0.3), True)
            print('PASS: 選択点群のXYZ・点数・時刻・座標系、異なる座標系からのTF適用')

            wide = make_candidate(x=0.4)
            other = TopologicalNode(id=99)
            other.pos.x, other.pos.y, other.pos.z = 1.0, 0.0, 0.2
            wide.nodes.append(other)
            wide.clusters[0].nodes = [0, 99]
            case(wide, True)
            wide.clusters = [TopologicalCluster(nodes=[0]), TopologicalCluster(nodes=[99])]
            wide.header.stamp = node.get_clock().now().to_msg()
            case(wide, False)
            wide.clusters = []
            wide.header.stamp = node.get_clock().now().to_msg()
            case(wide, False)
            print('PASS: ノードから離れた候補内部点の採用、候補間の非結合、所属なしの通常学習復帰')

            empty = make_candidate()
            empty.nodes = []
            case(empty, False)
            old = node.get_clock().now().to_msg()
            old.sec -= 2
            case(make_candidate(stamp=old), False)
            future = node.get_clock().now().to_msg()
            future.sec += 2
            case(make_candidate(stamp=future), False)
            case(make_candidate('missing_frame'), False)
            case(make_candidate(x=100), False)

            candidate = make_candidate()
            candidate_pub.publish(candidate)
            spin(0.7)
            before = output().count('Grasp attention selected')
            num_before = len(sampled)
            publish_cloud(candidate.header.stamp)
            assert output().count('Grasp attention selected') == before
            wait(lambda: len(sampled) > num_before)
            assert sampled[-1].width == 0 and not sampled[-1].data
            print('PASS: 空候補・古い/未来の時刻・TFなし・該当点なし・受信停止時の通常学習継続')
            print('PASS: 重点解除時の空点群配信')
        except BaseException:
            print(output()[-12000:])
            raise
        finally:
            stop(process)
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
