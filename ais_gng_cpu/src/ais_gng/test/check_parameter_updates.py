import os
import signal
import subprocess

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically


def main():
    # 既存ROSから隔離した有限時間の設定変更検証。
    os.environ["ROS_DOMAIN_ID"] = "226"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    cmd = ["/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu", "--ros-args",
           "-p", "input.topic_names:=[/parameter_test/points]",
           "-p", "input.voxel_grid_unit:=0.0", "-p", "node.grid:=0.5",
           "-p", "node.num_max:=128", "-p", "classify.human:=false",
           "-p", "classify.car:=false", "-p", "plane_cluster.direct_enabled:=false",
           "-p", "nonplane_component.direct_enabled:=false"]
    with open("/tmp/gng_parameter_updates_node.log", "w") as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=log, start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = rclpy.create_node("parameter_update_test")
            setter = node.create_client(SetParametersAtomically, "/ais_gng_node/set_parameters_atomically")
            getter = node.create_client(GetParameters, "/ais_gng_node/get_parameters")
            assert setter.wait_for_service(timeout_sec=20), "parameter service unavailable"
            assert getter.wait_for_service(timeout_sec=5)

            def call(client, request):
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done(), "service timeout"
                return future.result()

            def set_values(values, is_expected):
                request = SetParametersAtomically.Request()
                request.parameters = [Parameter(name, value=value).to_parameter_msg()
                                      for name, value in values.items()]
                result = call(setter, request).result
                print(values, result.successful, result.reason, flush=True)
                assert result.successful == is_expected

            set_values({"node.learning_num": 12}, True)
            set_values({"node.interval": [0.1, 0.2, 0.3, 0.4]}, True)
            set_values({"node.learning_num": 30, "input.x_max": 100.0}, False)
            set_values({"node.learning_num": -1}, False)
            set_values({"node.learning_num": 30, "node.interval": [0.1, -1.0, 0.3, 0.4]}, False)
            set_values({"node.interval": [0.1]}, False)
            set_values({"input.voxel_grid_unit": 0.2}, False)
            set_values({"node.num_max": 256}, False)
            set_values({"plane_cluster.direct_enabled": True}, False)
            request = GetParameters.Request()
            request.names = ["node.learning_num", "node.interval", "input.voxel_grid_unit", "node.num_max"]
            values = call(getter, request).values
            assert values[0].integer_value == 12
            assert list(values[1].double_array_value) == [0.1, 0.2, 0.3, 0.4]
            assert values[2].double_value == 0
            assert values[3].integer_value == 128
            assert process.poll() is None
            print("parameter_update_ros_test=passed", flush=True)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            print("test process stopped", process.returncode, flush=True)


if __name__ == "__main__":
    main()
