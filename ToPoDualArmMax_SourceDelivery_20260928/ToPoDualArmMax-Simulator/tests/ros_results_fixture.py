"""表示ゲートウェイ・ROS入力の隔離試験。標準入力の終了または期限到達で全プロセスを停止。"""
import argparse
import json
import signal
import subprocess
import tempfile
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--origin', required=True)
    args = parser.parse_args()
    import rclpy
    from ais_gng_msgs.msg import TopologicalMap, TopologicalNode, TopologicalCluster, PlaneClusterArray, PlaneCluster
    from std_msgs.msg import UInt32MultiArray, String
    from visualization_msgs.msg import Marker, MarkerArray
    from sensor_msgs.msg import PointCloud2, PointField
    from voxel_msgs.msg import Voxel
    import struct
    import copy
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped, Point, PoseArray, Pose
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node('simulator_results_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    graph_pub = node.create_publisher(TopologicalMap, '/topological_map', qos)
    plane_pub = node.create_publisher(PlaneClusterArray, '/plane_clusters', qos)
    nonplane_pub = node.create_publisher(UInt32MultiArray, '/nonplane_components', qos)
    tf_pub = node.create_publisher(TFMessage, '/tf_static', qos)
    marker_pub = node.create_publisher(MarkerArray, '/test/markers', qos)
    pose_pub = node.create_publisher(PoseArray, '/test/poses', qos)
    pc_pub = node.create_publisher(PointCloud2, '/test/points', 1)
    voxel_pub = node.create_publisher(Voxel, '/test/voxels', qos)
    robot_pub = node.create_publisher(String, '/viewer/internal/stream/robot/description', qos)
    robot_pose_pub = node.create_publisher(String, '/viewer/internal/stream/robot/pose', qos)
    candidate_pub = node.create_publisher(TopologicalMap, '/grasp_pose_cands/Tmap', qos)
    robot_map_pub = node.create_publisher(TopologicalMap, '/test/Tmap_robot', qos)
    remote_map_pub = node.create_publisher(TopologicalMap, '/test/Tmap_remote', qos)
    stop_event = threading.Event()
    def await_end():
        import sys
        sys.stdin.read()
        stop_event.set()
    threading.Thread(target=await_end, daemon=True).start()
    def on_signal(_signum, _frame):
        stop_event.set()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    graph = TopologicalMap()
    graph.header.frame_id = 'sensor'
    graph.nodes = [TopologicalNode(id=idx * 2, label=1) for idx in range(128)]
    for idx, vertex in enumerate(graph.nodes):
        vertex.pos.x = .1 + (idx % 16) * .01
        vertex.pos.y = .1 + (idx // 16) * .01
        vertex.pos.z = .2
        vertex.normal.z = 1.
    graph.edges = [value for idx in range(127) for value in (idx, idx + 1)]
    graph.clusters = [TopologicalCluster(id=7, nodes=list(range(64)))]
    planes = PlaneClusterArray()
    planes.header.frame_id = 'sensor'
    planes.clusters = [PlaneCluster(id=3, node_indices=list(range(64)))]
    transform = TransformStamped()
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'sensor'
    transform.transform.translation.x = .25
    transform.transform.rotation.w = 1.
    markers = MarkerArray()
    for idx, kind in enumerate([Marker.ARROW, Marker.CUBE, Marker.SPHERE, Marker.CYLINDER,
                               Marker.LINE_STRIP, Marker.LINE_LIST, Marker.CUBE_LIST, Marker.SPHERE_LIST,
                               Marker.TEXT_VIEW_FACING]):
        marker = Marker()
        marker.header.frame_id = 'sensor'
        marker.ns = 'test'; marker.id = idx; marker.type = kind
        marker.pose.position.x = -.2 + idx * .04; marker.pose.position.z = .25
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = .02
        marker.color.r = .2; marker.color.g = .9; marker.color.a = 1.
        marker.text = 'ROS結果'
        if kind in [Marker.LINE_LIST, Marker.LINE_STRIP, Marker.CUBE_LIST, Marker.SPHERE_LIST]:
            marker.points = [Point(x=0., y=0., z=0.), Point(x=.02, y=.02, z=.02)]
        markers.markers.append(marker)
    poses = PoseArray(); poses.header.frame_id = 'sensor'
    pose = Pose(); pose.position.y = -.1; pose.position.z = .2; pose.orientation.w = 1.
    poses.poses = [pose]
    pc = PointCloud2(); pc.header.frame_id = 'sensor'; pc.height = 1; pc.width = 128
    pc.point_step = 12; pc.row_step = pc.point_step * pc.width; pc.is_dense = True
    pc.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1) for idx, name in enumerate(['x','y','z'])]
    pc.data = b''.join(struct.pack('<fff', -.2 + (idx%16)*.01, .1+(idx//16)*.01, .3) for idx in range(pc.width))
    voxels = Voxel(); voxels.header.frame_id = 'sensor'; voxels.voxel_size = .025
    voxels.x_shift = 20; voxels.y_shift = 10; voxels.z_shift = 0; voxels.offset = 512
    voxel_ids = [((512+idx)<<20) | (512<<10) | 512 for idx in range(4)]
    voxels.data = voxel_ids; voxels.labels = [1,2,3,4]
    urdf = '<robot name="fixture"><link name="base"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link></robot>'
    robot = {'frameId':'sensor','urdf':urdf,'jointNames':[],'jointValues':[], 'positions':[], 'orientations':[],
             'basePosition':[-.1,-.15,.3], 'manipValid':True,'manipScale':[.06,.03,.02],
             'manipCenter':[0,0,.2],'manipOrientation':[0,0,0,1]}
    def publish():
        graph.frame_number += 1
        planes.frame_number = graph.frame_number
        graph.header.stamp = planes.header.stamp = node.get_clock().now().to_msg()
        graph_pub.publish(graph)
        plane_pub.publish(planes)
        nonplane_pub.publish(UInt32MultiArray(data=[graph.frame_number, 1, 9, 64, *range(64, 128)]))
        tf_pub.publish(TFMessage(transforms=[transform]))
        marker_pub.publish(markers); pose_pub.publish(poses); pc_pub.publish(pc)
        voxels.data = voxel_ids[:3] if graph.frame_number % 2 else voxel_ids
        voxels.labels = [1,2,3,4][:len(voxels.data)]; voxel_pub.publish(voxels)
        robot_pub.publish(String(data=json.dumps({'type':'stream.robot.description','tag':'test_robot','robot':robot})))
        robot_pose_pub.publish(String(data=json.dumps({'type':'stream.robot.pose','tag':'test_robot','robot':robot})))
        candidate = copy.deepcopy(graph)
        candidate.clusters[0].nodes = [idx * 2 for idx in range(64)]
        candidate_pub.publish(candidate)
        robot_map = copy.deepcopy(graph)
        robot_map.header.frame_id = 'topo_dual_arm_max_long/base_link'
        robot_map.clusters = []
        robot_map_pub.publish(robot_map)
        # TFのない遠方地図。全体表示・スタジオの霧・視点距離制限の回帰確認用。
        remote_map = copy.deepcopy(graph)
        remote_map.header.frame_id = 'map'
        remote_map.clusters = []
        for idx, vertex in enumerate(remote_map.nodes):
            vertex.pos.x = 100. + (idx % 16) * 10.
            vertex.pos.y = (idx // 16) * 10.
            vertex.pos.z = 5.
        remote_map_pub.publish(remote_map)
    node.create_timer(.05, publish)
    command = ['/ros2_ws/src/ToPoFuzzy-Viewer/backend/build/topo_fuzzy_viewer/viewer_ws_gateway_node',
               '--ros-args', '-p', f'port:={args.port}', '-p', f'allowed_origins:=["{args.origin}"]']
    processes = []
    with tempfile.TemporaryFile() as log:
        try:
            commands = [command,
                ['/ros2_ws/src/ToPoFuzzy-Viewer/backend/build/topo_fuzzy_viewer/viewer_edit_node'],
                ['python3', '/ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/scripts/viewer_vehicle_registration_node.py']]
            for command in commands:
                process = subprocess.Popen(command, stdout=log, stderr=log)
                processes.append(process)
                print(json.dumps({'command': command, 'pid': process.pid}), flush=True)
            end = time.monotonic() + 110
            while not stop_event.is_set() and time.monotonic() < end and all(process.poll() is None for process in processes):
                rclpy.spin_once(node, timeout_sec=.1)
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait(timeout=5)
            if any(process.returncode not in (0, -2) for process in processes):
                log.seek(0); print(log.read().decode(errors='replace'), flush=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print('STOPPED: gateway and test publisher', flush=True)


if __name__ == '__main__':
    main()
