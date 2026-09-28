#!/usr/bin/env python3
"""有限トルクモータの読込完了後だけのGazebo物理開始。"""
import time
import rclpy
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty


def main():
    rclpy.init()
    node = rclpy.create_node('start_gazebo_physics')
    try:
        node.declare_parameter('controller_manager', '')
        manager = node.get_parameter('controller_manager').value
        if not manager.startswith('/sim_'):
            raise ValueError('シミュレーション用controller_managerが必要です')
        ready = node.create_client(ListControllers, manager+'/list_controllers')
        unpause = node.create_client(Empty, '/unpause_physics')
        deadline = time.monotonic()+90
        while rclpy.ok() and time.monotonic() < deadline:
            if ready.service_is_ready() and unpause.service_is_ready():
                future = unpause.call_async(Empty.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                if not future.done() or future.exception():
                    raise RuntimeError('Gazebo物理開始の失敗')
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        raise RuntimeError('有限トルクモータ読込の待機上限')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
