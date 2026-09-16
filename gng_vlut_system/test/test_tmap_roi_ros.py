"""TmapのBBox、余白、TF、複数ROIの隔離ROS回帰検証。"""

import json
import math
import os
from pathlib import Path
import runpy
import signal
import struct
import subprocess
import tempfile
import time


def verify_launch_arguments():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    package = Path(__file__).resolve().parents[1]
    environment = runpy.run_path(str(package / "launch/environment_to_vlut.launch.py"))
    params_file = str(package / "config/ToPoDualArm.yaml")
    context = LaunchContext()
    context.launch_configurations.update(params_file=params_file, robot_name="")
    include = next(action for action in environment["_launch_setup"](context)
                   if isinstance(action, IncludeLaunchDescription))
    expected = "/ToPoDualArm/Tmap_static"
    assert dict(include.launch_arguments)["reachability_map_topic"] == expected
    assert environment["_reachability_map_topic"]({"reachability_map_topic": ""}, "R") == ""
    assert environment["_reachability_map_topic"]({"reachability_map_topic": "/custom/Tmap"}, "R") == "/custom/Tmap"

    for topic in (expected, ""):
        for filename in ("point_to_vlut.launch.py", "point_to_voxel.launch.py",
                         "world_index_to_voxel.launch.py"):
            module = runpy.run_path(str(package / "launch" / filename))
            for enable_build in ("true", "false"):
                context = LaunchContext()
                context.launch_configurations.update(
                    reachability_map_topic=topic, world_index_enable=enable_build)
                actions = module["generate_launch_description"]().entities
                for action in actions:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                if "_launch_setup" in module:
                    actions = module["_launch_setup"](context)
                for action in actions:
                    if isinstance(action, IncludeLaunchDescription):
                        value = dict(action.launch_arguments)["reachability_map_topic"]
                        assert perform_substitutions(
                            context, normalize_to_list_of_substitutions(value)) == topic
                    elif isinstance(action, Node):
                        values = evaluate_parameters(context, action._Node__parameters)[0]
                        if "enable_reachability_filter" in values:
                            assert values["reachability_map_topic"] == topic

    consumers = [{"params_file": params_file, "robot_name": name,
                  "voxel_topic": f"/{name}/voxels"} for name in ("R1", "R2")]
    actions = environment["_shared_world_index_actions"](
        str(package), params_file, {},
        {"enable_build": True, "enable_roi_query": True, "consumers": consumers})
    values = evaluate_parameters(LaunchContext(), actions[0]._Node__parameters)[0]
    assert values["reachability_map_topic"] == "/R1/Tmap_static"
    assert json.loads(values["additional_consumers_json"])[0]["reachability_map_topic"] == "/R2/Tmap_static"
    print("PASS launch arguments: single/shared, direct/index, map/manual", flush=True)


def main():
    os.environ.update(ROS_DOMAIN_ID="224", ROS_LOCALHOST_ONLY="1")
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster
    from voxel_msgs.msg import Voxel

    verify_launch_arguments()
    rclpy.init()
    node = rclpy.create_node("tmap_roi_test")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    broadcaster = StaticTransformBroadcaster(node)
    executable = (get_package_prefix("gng_vlut_system") +
                  "/lib/gng_vlut_system/world_index_to_voxel_node")

    def transform(parent, child, xyz, yaw=0.0):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp = node.get_clock().now().to_msg()
        translation = result.transform.translation
        translation.x, translation.y, translation.z = map(float, xyz)
        result.transform.rotation.z = math.sin(yaw / 2)
        result.transform.rotation.w = math.cos(yaw / 2)
        return result

    def make_map(frame, points):
        result = TopologicalMap()
        result.header.frame_id = frame
        for idx, point in enumerate(points):
            vertex = TopologicalNode(id=idx)
            vertex.pos.x, vertex.pos.y, vertex.pos.z = map(float, point)
            result.nodes.append(vertex)
        return result

    def make_cloud(points):
        result = PointCloud2()
        result.header.frame_id = "roi_world"
        result.height, result.width = 1, len(points)
        result.fields = [PointField(name=axis, offset=4 * idx, datatype=7, count=1)
                         for idx, axis in enumerate("xyz")]
        result.point_step, result.row_step = 12, 12 * len(points)
        result.data = b"".join(struct.pack("<fff", *point) for point in points)
        return result

    def voxel_ids(points):
        return {sum((math.floor(value / 0.1) + 1000000) << shift
                    for value, shift in zip(point, (42, 21, 0))) for point in points}

    # ROI座標の点群。BBox内部、各面の余白、範囲外の混在
    inside = [(0.05, 2.05, 3.05), (0.95, 3.95, 3.95),
              (-0.05, 1.85, 2.75), (1.05, 4.15, 4.25)]
    outside = [(1.15, 3.05, 3.55), (0.55, 4.25, 3.55), (0.55, 3.05, 4.35),
               (-0.15, 3.05, 3.55), (0.55, 1.75, 3.55), (0.55, 3.05, 2.65)]
    map_points = [(0, 0, 0), (2, 1, 1), (float("nan"), 0, 0)]
    robot_tf = transform("roi_world", "roi_robot", (5, -2, 1), -0.3)
    map_tf = transform("roi_robot", "roi_map", (1, 2, 3), math.pi / 2)
    extra_tf = transform("roi_world", "roi_extra", (-2, 1, 0))
    broadcaster.sendTransform([robot_tf, map_tf, extra_tf])

    def world_point(point):
        return (5 + math.cos(-0.3) * point[0] - math.sin(-0.3) * point[1],
                -2 + math.sin(-0.3) * point[0] + math.cos(-0.3) * point[1],
                1 + point[2])

    extra_point = (-1.95, 1.05, 0.05)
    cloud = make_cloud([world_point(point) for point in inside + outside] + [extra_point])
    try:
        for case_idx, (enable_build, enable_query) in enumerate(
                [(False, False), (True, False), (True, True)]):
            prefix = f"/tmap_roi_test_{case_idx}"
            cloud_pub = node.create_publisher(PointCloud2, prefix + "/points", 1)
            map_pub = node.create_publisher(TopologicalMap, prefix + "/Tmap_static", qos)
            extra_pub = node.create_publisher(TopologicalMap, prefix + "/extra_Tmap_static", qos)
            received = {"primary": [], "extra": []}
            subscriptions = [node.create_subscription(
                Voxel, prefix + "/" + key, lambda msg, key=key: received[key].append(msg), qos)
                for key in received]
            params = {
                "input_topic": prefix + "/points", "output_topic": prefix + "/primary",
                "world_frame_id": "roi_world", "target_frame_id": "roi_robot",
                "reachability_map_topic": prefix + "/Tmap_static", "voxel_size": 0.1,
                "reachability_margin_x": 0.1, "reachability_margin_y": 0.2,
                "reachability_margin_z": 0.3, "enable_world_index": enable_build,
                "enable_roi_query": enable_query, "enable_world_bucket_publish": False,
                "parallel_thread_num": 2,
                "additional_consumers_json": json.dumps([{
                    "name": "extra", "target_frame_id": "roi_extra",
                    "output_topic": prefix + "/extra", "voxel_size": 0.1,
                    "reachability_map_topic": prefix + "/extra_Tmap_static",
                    "reachability_margin_x": 0.1, "reachability_margin_y": 0.1,
                    "reachability_margin_z": 0.1}]),
            }
            command = [executable, "--ros-args"]
            for key, value in params.items():
                command.extend(["-p", f"{key}:={json.dumps(value)}"])
            if enable_query:
                command = ["ros2", "launch", "gng_vlut_system", "world_index_to_voxel.launch.py"]
                command.extend(f"{key}:={value if isinstance(value, str) else json.dumps(value)}"
                               for key, value in params.items())
            # 主ROIのマップはノード起動前に一度だけ配信。追加ROIは遅着
            graph = make_map("roi_map", map_points)
            map_pub.publish(graph)
            with tempfile.TemporaryFile(mode="w+") as log:
                process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                print("START", " ".join(command), flush=True)

                def pump(duration=0.4):
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        assert process.poll() is None, "ROIノード異常終了"
                        cloud_pub.publish(cloud)
                        rclpy.spin_once(node, timeout_sec=0.02)

                def expect(key, expected, frame):
                    received[key].clear()
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        pump(0.1)
                        if received[key] and set(received[key][-1].data) == expected:
                            assert received[key][-1].header.frame_id == frame
                            assert abs(received[key][-1].voxel_size - 0.1) < 1e-7
                            return
                    actual = set(received[key][-1].data) if received[key] else None
                    raise AssertionError((case_idx, key, actual, expected))

                try:
                    expect("primary", voxel_ids(inside), "roi_robot")
                    assert not received["extra"], "マップ未取得の固定範囲へのフォールバック"
                    extra_pub.publish(make_map("roi_extra", [(0, 0, 0)]))
                    expect("extra", voxel_ids([(0.05, 0.05, 0.05)]), "roi_extra")

                    # 状態更新のみではBBox再構築なし
                    graph.nodes[0].label = 4
                    map_pub.publish(graph)
                    expect("primary", voxel_ids(inside), "roi_robot")
                    pump()
                    log.flush()
                    log.seek(0)
                    assert log.read().count("Tmap BBoxからROI更新") == 2

                    # マップ変更による縮小と拡大
                    map_pub.publish(make_map("roi_map", [(0, 0, 0)]))
                    expect("primary", set(), "roi_robot")
                    map_pub.publish(graph)
                    expect("primary", voxel_ids(inside), "roi_robot")

                    # 未接続TF・空マップ・無効点だけのマップでは主ROI配信停止
                    for invalid_map in [make_map("missing_frame", [(0, 0, 0)]),
                                        make_map("roi_map", []),
                                        make_map("roi_map", [(float("nan"), 0, 0)])]:
                        map_pub.publish(invalid_map)
                        pump()
                        received["primary"].clear()
                        pump()
                        assert not received["primary"]
                        expect("extra", voxel_ids([(0.05, 0.05, 0.05)]), "roi_extra")
                    map_pub.publish(graph)
                    expect("primary", voxel_ids(inside), "roi_robot")

                    # マップ再配信なしでのTF変更反映
                    map_tf.transform.translation.x = 11.0
                    broadcaster.sendTransform([robot_tf, map_tf, extra_tf])
                    expect("primary", set(), "roi_robot")
                    map_tf.transform.translation.x = 1.0
                    broadcaster.sendTransform([robot_tf, map_tf, extra_tf])
                    expect("primary", voxel_ids(inside), "roi_robot")
                    print(f"PASS build={enable_build} query={enable_query}", flush=True)
                except BaseException:
                    log.flush()
                    log.seek(0)
                    print(log.read(), flush=True)
                    raise
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
                    print("STOPPED", process.pid, flush=True)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            for publisher in (cloud_pub, map_pub, extra_pub):
                node.destroy_publisher(publisher)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
