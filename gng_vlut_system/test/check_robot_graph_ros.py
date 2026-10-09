"""隔離ROS環境での姿勢グラフ更新と取消要求の通信試験。"""
import json
from pathlib import Path
import sys
import tempfile
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
workspace = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(workspace / 'gng_vlut_system/scripts'))
from task_executor import task_executor
from task_program import run_state
from task_components import planning_request

with tempfile.TemporaryDirectory() as temporary:
    config = Path(temporary) / 'task.yaml'
    config.write_text(yaml.safe_dump(dict(version=1,
        inputs={'robot_graph': {'robot_id': 'topo_dual_arm_max_long', 'joint_names': ['L_joint4']}},
        methods={'move': {'graph_move': dict(targets='direct', route='robot_graph', trajectory='quintic')}},
        defaults={'move': 'graph_move'}, poses={'goal': {'L_joint4': .2}}, tasks=[dict(kind='move', target='goal')])))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/sim_graph_check', '-p', 'use_sim_time:=true',
        '-p', 'task_file:='+str(config), '-p', 'urdf:='+str(workspace / 'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf')])
    executor = SingleThreadedExecutor()
    node = task_executor()
    driver = Node('graph_test_publisher')
    executor.add_node(node)
    executor.add_node(driver)
    publisher = driver.create_publisher(String, '/sim_graph_check/robot_Tmap_updates', 10)
    for timer in node.timers:
        timer.cancel()
    def wait_for(condition):
        deadline = time.monotonic() + 5
        while not condition():
            if time.monotonic() > deadline:
                raise RuntimeError('ROS受信の期限切れ')
            executor.spin_once(timeout_sec=.05)
    try:
        wait_for(lambda: publisher.get_subscription_count() == 1)
        packet = dict(kind='snapshot', robot_id='topo_dual_arm_max_long', graph_id='test', revision=1,
            stamp_sec=0.0, joint_names=['L_joint4'], nodes=[dict(id=10, positions=[0.], can_traverse=True),
            dict(id=20, positions=[.2], can_traverse=True)], edges=[dict(nodes=[10,20], can_traverse=True)])
        publisher.publish(String(data=json.dumps(packet)))
        source, = node.program.input_sources
        wait_for(lambda: source.snapshot is not None)
        start = tuple(max(bound.min_position, min(0., bound.max_position)) for bound in node.program.bounds)
        goal = list(start)
        goal[node.program.joint_names.index('L_joint4')] = .2
        source(planning_request(start, tuple(goal), node.program.bounds, node.program.limits, node.program.joint_names))
        node.runner.state = run_state.running
        node.backend.is_busy = True
        publisher.publish(String(data=json.dumps(dict(kind='delta', robot_id='topo_dual_arm_max_long',
            graph_id='test', base_revision=1, revision=2, stamp_sec=0.0,
            edges=[dict(nodes=[10,20], can_traverse=False)]))))
        wait_for(lambda: node.runner.state == run_state.stopping)
        assert node.backend.has_cancel_request
        print('ROS受信→経路生成→辺更新→Action取消要求: PASS（実ロボット指令なし）')
    finally:
        executor.shutdown()
        node.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
