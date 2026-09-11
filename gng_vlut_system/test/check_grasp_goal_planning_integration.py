"""実GNGと隔離ROSドメインによる把持候補保持・計画スキップの結合確認。"""

import copy
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from ais_gng_msgs.msg import TopologicalMap
from gng_control_msgs.msg import GraspCandidate, GraspCandidateArray, GraspCandidateMetricArray


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "218":
        raise RuntimeError("結合確認にはROS_DOMAIN_ID=218の指定が必要です")
    params = "/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml"
    model_dir = "/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000"
    commands = [
        ["ros2", "run", "gng_vlut_system", "safety_monitor_node", "--ros-args",
         "--params-file", params, "-p", f"gng_model_path:={model_dir}/gng.bin",
         "-p", f"vlut_path:={model_dir}/vlut.bin", "-p", "base_frame:=ToPoDualArm/base_link",
         "-r", "topological_map:=/ToPoDualArm/topological_map_static"],
        ["ros2", "launch", "gng_vlut_system", "grasp_goal_planning.launch.py",
         f"params_file:={params}", "enable_motion:=false"],
    ]
    processes = []
    logs = []
    log_dir = tempfile.TemporaryDirectory(prefix="grasp_candidate_integration_")
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node("grasp_reachability_integration_probe")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        received = {}
        for topic, msg_type, key in [
            ("/ToPoDualArm/topological_map_static", TopologicalMap, "map"),
            ("/grasp_pose_cands", GraspCandidateArray, "candidates"),
            ("/selected_goal_candidate_ids", Int32MultiArray, "goals"),
            ("/ToPoDualArm/grasp_candidate_metrics", GraspCandidateMetricArray, "metrics"),
            ("/ToPoDualArm/plan_topological_map", TopologicalMap, "plan"),
            ("/ToPoDualArm/cand_topological_map", TopologicalMap, "candidate_plan"),
        ]:
            node.create_subscription(msg_type, topic, lambda msg, key=key: received.update({key: msg}), qos)
        publisher = node.create_publisher(GraspCandidateArray, "/grasp_pose_cands", qos)

        def wait_for(predicate, label, max_sec=30.0):
            end = time.monotonic() + max_sec
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                if predicate():
                    return
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError(f"検証対象プロセスの早期終了: {label}")
            raise AssertionError(f"待機時間超過: {label}; received={list(received)}")

        for idx, command in enumerate(commands):
            log = (Path(log_dir.name) / f"process_{idx}.log").open("w")
            logs.append(log)
            print("起動:", " ".join(command), flush=True)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        wait_for(lambda: "map" in received and received["map"].nodes, "実GNG入力")
        map_msg = received["map"]
        selected_node = next(item for item in map_msg.nodes if item.label == 1)
        inside = Pose()
        inside.position.x = float(selected_node.pos.x)
        inside.position.y = float(selected_node.pos.y)
        inside.position.z = float(selected_node.pos.z)
        inside.orientation.w = 1.0
        outside = copy.deepcopy(inside)
        outside.position.x += 100.0
        source = GraspCandidateArray(header=copy.deepcopy(map_msg.header), update_id=1,
            evaluation_header=copy.deepcopy(map_msg.header), voxel_size=0.05, candidates=[
                GraspCandidate(id=42, pose=outside, state=GraspCandidate.OUTSIDE),
                GraspCandidate(id=7, pose=inside, state=GraspCandidate.INSIDE)])
        publisher.publish(source)
        wait_for(lambda: "goals" in received and received["goals"].data, "領域内候補の目標選択")
        allowed_ids = set(received["goals"].data)
        assert selected_node.id in allowed_ids
        wait_for(lambda: "metrics" in received and received["metrics"].candidates, "領域内候補の計画評価")
        assert all(item.goal_node_id in allowed_ids for item in received["metrics"].candidates)
        print("混在入力: 全候補保持・領域内目標だけの計画評価を確認", flush=True)

        source.update_id += 1
        source.candidates = [GraspCandidate(id=42, pose=outside, state=GraspCandidate.OUTSIDE)]
        publisher.publish(source)
        wait_for(lambda: "goals" in received and not received["goals"].data
                 and "candidates" in received and received["candidates"].update_id == source.update_id
                 and "metrics" in received and not received["metrics"].candidates
                 and "plan" in received and not received["plan"].edges
                 and all(item.id == 65535 for item in received["plan"].nodes)
                 and "candidate_plan" in received and not received["candidate_plan"].nodes,
                 "全領域外での旧計画失効")
        assert received["candidates"].candidates[0].pose == outside
        print("全領域外: 候補保持・目標ID/旧経路/旧評価のクリアを確認", flush=True)

        source.update_id += 1
        source.candidates = [GraspCandidate(id=7, pose=inside, state=GraspCandidate.INSIDE)]
        received.pop("metrics", None)
        publisher.publish(source)
        wait_for(lambda: "metrics" in received and received["metrics"].candidates, "領域内復帰後の再計画")
        print("領域内復帰: 再計画を確認", flush=True)
        source.update_id += 1
        source.candidates = []
        publisher.publish(source)
        wait_for(lambda: not received["candidates"].candidates and not received["goals"].data
                 and not received["metrics"].candidates, "空入力")
        assert node.count_publishers("/ToPoDualArm/target_joint_states") == 0
        assert node.count_publishers("/grasp_pose_cands") == 1
        assert node.count_publishers("/grasp_pose_cands/reachability") == 0
        assert node.count_publishers("/grasp_pose_cands/reachability_markers") == 0
        print("空入力のクリア・関節目標配信なしを確認", flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()
        log_dir.cleanup()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        print("検証用プロセスはすべて停止済み", flush=True)


if __name__ == "__main__":
    main()
