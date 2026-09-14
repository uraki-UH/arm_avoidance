"""実GNGと隔離ROSドメインによる候補経路生成・実行系分離の結合確認。"""

import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger
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
         "-r", "topological_map:=/ToPoDualArm/Tmap_static"],
        ["ros2", "launch", "gng_vlut_system", "grasp_joint_candidates.launch.py",
         f"params_file:={params}"],
    ]
    processes = []
    logs = []
    log_dir = tempfile.TemporaryDirectory(prefix="grasp_candidate_integration_")
    os.environ["ROS_LOG_DIR"] = log_dir.name
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node("grasp_reachability_integration_probe")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        received = {}
        for topic, msg_type, key in [
            ("/ToPoDualArm/Tmap_static", TopologicalMap, "map"),
            ("/selected_Tmap", TopologicalMap, "selected_map"),
            ("/grasp_pose_cands", GraspCandidateArray, "candidates"),
            ("/selected_goal_candidate_ids", Int32MultiArray, "goals"),
            ("/ToPoDualArm/grasp_candidate_metrics", GraspCandidateMetricArray, "metrics"),
            ("/ToPoDualArm/plan_Tmap", TopologicalMap, "plan"),
            ("/ToPoDualArm/cand_Tmap", TopologicalMap, "candidate_plan"),
            ("/viewer/internal/stream/robot/description", String, "candidate_robot_description"),
            ("/viewer/internal/stream/robot/pose", String, "candidate_robot_pose"),
        ]:
            node.create_subscription(msg_type, topic, lambda msg, key=key: received.update({key: msg}), qos)
        publisher = node.create_publisher(GraspCandidateArray, "/grasp_pose_cands", qos)
        joint_publisher = node.create_publisher(JointState, "/ToPoDualArm/joint_states", qos)
        update_client = node.create_client(Trigger, "/ToPoDualArm/request_trajectory_update")

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
        wait_for(lambda: "topological_map_path_planner_node" in node.get_node_names(),
                 "経路生成専用ノードの起動")
        assert "topological_map_avoidance_node" not in node.get_node_names()
        assert not any(name.endswith("/request_trial_goal_advance")
                       for name, _ in node.get_service_names_and_types())
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
        wait_for(lambda: "selected_map" in received and received["selected_map"].nodes,
                 "短縮名の選定マップ出力")
        wait_for(
            lambda: "metrics" in received and received["metrics"].candidates and
            "candidate_robot_description" in received and
            "candidate_robot_pose" in received,
            "領域内候補の計画評価と候補ロボット召喚")
        # 通常ロボット・GNGと候補プレビューの基準フレームの一致
        for key in ("candidate_robot_description", "candidate_robot_pose"):
            robot = json.loads(received[key].data)["robot"]
            assert robot["frameId"] == map_msg.header.frame_id, robot["frameId"]
            assert robot["instances"]
            assert all(item["frameId"] == map_msg.header.frame_id
                       for item in robot["instances"])
        print("候補ロボット: YAMLの基準フレームと全インスタンスの一致を確認", flush=True)
        assert all(item.goal_node_id in allowed_ids for item in received["metrics"].candidates)
        assert any(item.feasible and item.path_node_ids for item in received["metrics"].candidates)
        assert all(
            item.final_joint_state.name and
            len(item.final_joint_state.name) == len(item.final_joint_state.position)
            for item in received["metrics"].candidates)
        print("混在入力: 全候補保持・領域内目標だけの計画評価を確認", flush=True)

        stamp = copy.deepcopy(received["metrics"].header.stamp)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert received["metrics"].header.stamp == stamp
        print("静止入力: 周期的な再探索・評価再配信なしを確認", flush=True)

        wait_for(update_client.service_is_ready, "明示更新サービス")
        future = update_client.call_async(Trigger.Request())
        wait_for(lambda: future.done() and received["metrics"].header.stamp != stamp,
                 "明示要求による再計画")
        assert future.result().success
        stamp = copy.deepcopy(received["metrics"].header.stamp)
        joint_publisher.publish(received["metrics"].candidates[0].final_joint_state)
        wait_for(lambda: received["metrics"].header.stamp != stamp, "現在関節姿勢の変更による再計画")
        print("明示要求・現在関節姿勢変更による再計画を確認", flush=True)

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
        assert node.count_publishers("/ToPoDualArm/control_claims") == 0
        assert node.count_publishers("/grasp_pose_cands") == 1
        assert node.count_publishers("/grasp_pose_markers") == 0
        assert node.count_publishers("/grasp_pose_cands/reachability") == 0
        assert node.count_publishers("/grasp_pose_cands/reachability_markers") == 0
        print("空入力のクリア・関節目標配信なしを確認", flush=True)
    except BaseException:
        for log in logs:
            log.flush()
            print(Path(log.name).read_text()[-12000:], flush=True)
        raise
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
