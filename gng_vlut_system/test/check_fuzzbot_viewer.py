"""FuzzBotの表示専用launch・初回姿勢・Viewer配信の隔離検証。"""
import json
import math
import os
from pathlib import Path
import runpy
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as et

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import xacro


def check_backend_selection(root):
    module = runpy.run_path(str(root/'launch/gng_viewer_bridge.launch.py'))
    for params_name, override, has_control in (
        ('fuzzbot.yaml', '', False), ('fuzzbot.yaml', 'viewer', True),
        ('ToPoDualArm.yaml', '', False), ('fuzzbot.yaml', 'invalid', None),
    ):
        context = LaunchContext()
        for action in module['generate_launch_description']().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations.update(
            params_file=str(root/'config'/params_name), joint_control_backend=override)
        if params_name == 'ToPoDualArm.yaml':
            # 既定viewerの確認用。実測表示による制御経路の省略を無効化。
            context.launch_configurations['enable_dynamixel_current_pose'] = 'false'
            has_control = True
        try:
            actions = module['launch_setup'](context)
        except ValueError as error:
            assert has_control is None and 'joint_control_backend' in str(error), error
            continue
        assert has_control is not None
        controls = [action for action in actions if isinstance(action, IncludeLaunchDescription)
                    and 'backend' in dict(action.launch_arguments)]
        assert len(controls) == 1
        assert controls[0].condition.evaluate(context) == has_control
        assert dict(controls[0].launch_arguments)['backend'] == (
            override or ('external' if params_name == 'fuzzbot.yaml' else 'viewer'))


def main():
    os.environ.update(ROS_DOMAIN_ID='228', ROS_LOCALHOST_ONLY='1')
    root = Path(__file__).resolve().parents[1]
    check_backend_selection(root)
    source = Path(get_package_share_directory('fuzzbot_description'))/'urdf/fuzzbot_pro_normal.urdf.xacro'
    expected_links = {item.attrib['name'] for item in
                      et.fromstring(xacro.process_file(str(source)).toxml()).findall('link')}
    command = ['ros2', 'launch', 'gng_vlut_system', 'gng_viewer_bridge.launch.py',
               'params_file:='+str(root/'config/fuzzbot.yaml')]
    print('launch_command:', ' '.join(command), flush=True)
    rclpy.init()
    node = rclpy.create_node('check_fuzzbot_viewer')
    received, transforms = {}, set()
    def save(kind, message):
        payload = json.loads(message.data)
        if payload.get('tag') == 'fuzzbot':
            received[kind] = payload['robot']
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/viewer/internal/stream/robot/description',
                             lambda message: save('description', message), qos)
    node.create_subscription(String, '/viewer/internal/stream/robot/pose',
                             lambda message: save('pose', message), qos_profile_sensor_data)
    node.create_subscription(JointState, '/fuzzbot/viewer_joint_states',
                             lambda message: received.update(joints=message), qos)
    node.create_subscription(TFMessage, '/tf',
                             lambda message: transforms.update(item.child_frame_id for item in message.transforms),
                             qos_profile_sensor_data)
    process = None
    try:
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                deadline = time.monotonic()+35
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                    if set(received) == {'description', 'pose', 'joints'} and {
                        'fuzzbot/wheel_left_link', 'fuzzbot/wheel_right_link'} <= transforms:
                        break
                    assert process.poll() is None, 'launchの早期終了'
                assert set(received) == {'description', 'pose', 'joints'}, list(received)
                assert {'fuzzbot/wheel_left_link', 'fuzzbot/wheel_right_link'} <= transforms, transforms
                expected = {'wheel_left_joint', 'wheel_right_joint'}
                assert set(received['joints'].name) == expected
                assert all(value == 0 for value in received['joints'].position)
                robot = received['description']
                assert robot['frameId'] == 'world' and robot['basePosition'] == [0, 0, 0]
                assert robot['baseOrientation'] == [0, 0, 0, 1]
                assert set(robot['jointNames']) == expected
                assert all(math.isfinite(value) for point in received['pose']['positions'] for value in point)
                description = et.fromstring(robot['urdf'])
                assert {item.attrib['name'] for item in description.findall('link')} == expected_links
                assert all('${' not in value for item in description.iter() for value in item.attrib.values())
                meshes = {item.attrib['filename'] for item in description.findall('.//mesh')}
                for uri in meshes:
                    package, path = uri.removeprefix('package://').split('/', 1)
                    assert (Path(get_package_share_directory(package))/path).is_file(), uri
                nodes = {name for name, namespace in node.get_node_names_and_namespaces()
                         if namespace == '/fuzzbot' and not name.startswith('transform_listener_impl_')}
                assert nodes == {'initial_joint_state_publisher_node', 'robot_state_publisher',
                                 'robot_description_player', 'robot_viewer_bridge_node'}, nodes
                print(json.dumps({'result': 'pass', 'links': len(description.findall('link')),
                                  'meshes': sorted(meshes), 'nodes': sorted(nodes)}, ensure_ascii=False))
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
                log.seek(0)
                print(log.read()[-12000:])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('所有試験launch: 終了済み', flush=True)


if __name__ == '__main__':
    main()
