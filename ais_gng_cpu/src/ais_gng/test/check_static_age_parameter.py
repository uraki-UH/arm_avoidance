import os
import signal
import subprocess
import tempfile

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    # 既存ROSから隔離した寿命パラメータの検証。ログも終了時に削除。
    os.environ["ROS_DOMAIN_ID"] = "227"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    cmd = ["/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu", "--ros-args",
           "-p", "input.topic_names:=[/static_age_test/points]",
           "-p", "node.num_max:=128", "-p", "node.grid:=0.5",
           "-p", "classify.human:=false", "-p", "classify.car:=false",
           "-p", "plane_cluster.direct_enabled:=false",
           "-p", "nonplane_component.direct_enabled:=false",
           "-p", "node.static.s1_age_max:=7"]
    with tempfile.TemporaryDirectory(prefix="gng-static-age-ros-") as log_dir:
        os.environ["ROS_LOG_DIR"] = log_dir
        with tempfile.TemporaryFile(mode="w+") as log:
            print("start:", " ".join(cmd), flush=True)
            process = subprocess.Popen(cmd, stdout=log, stderr=log, start_new_session=True)
            node = None
            try:
                rclpy.init()
                node = rclpy.create_node("static_age_parameter_test")
                getter = node.create_client(GetParameters, "/ais_gng_node/get_parameters")
                setter = node.create_client(SetParametersAtomically, "/ais_gng_node/set_parameters_atomically")
                assert getter.wait_for_service(timeout_sec=20)
                assert setter.wait_for_service(timeout_sec=5)

                def call(client, request):
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    assert future.done(), "service timeout"
                    return future.result()

                def read_age():
                    req = GetParameters.Request()
                    req.names = ["node.static.s1_age_max"]
                    return call(getter, req).values[0].integer_value

                assert read_age() == 7
                for value, is_expected in [(3, True), (0, False), (-1, False),
                                           (1.5, False), (16777217, False),
                                           (2147483647, False), (7, True)]:
                    req = SetParametersAtomically.Request()
                    req.parameters = [Parameter("node.static.s1_age_max", value=value).to_parameter_msg()]
                    before = read_age()
                    result = call(setter, req).result
                    assert result.successful == is_expected, result.reason
                    assert read_age() == (value if is_expected else before)
                assert process.poll() is None
                print("static_age_ros_parameter_test=passed", flush=True)
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                stop_process(process)
                log.seek(0)
                print(log.read(), flush=True)
                print("test process stopped", process.returncode, flush=True)

        # 起動時の不正値を既定値へ置き換えず、起動失敗として扱うことの確認。
        with tempfile.TemporaryFile(mode="w+") as log:
            cmd[-1] = "node.static.s1_age_max:=0"
            print("start:", " ".join(cmd), flush=True)
            process = subprocess.Popen(cmd, stdout=log, stderr=log, start_new_session=True)
            try:
                assert process.wait(timeout=20) != 0
                log.seek(0)
                assert "node.static.s1_age_max" in log.read()
                print("static_age_invalid_startup_test=passed", flush=True)
            finally:
                stop_process(process)
                print("test process stopped", process.returncode, flush=True)


if __name__ == "__main__":
    main()
