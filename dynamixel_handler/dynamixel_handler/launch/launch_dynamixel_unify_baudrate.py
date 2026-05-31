from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_name = 'dynamixel_handler'

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'config_dynamixel_unify_baudrate.yaml'
    )

    node = Node(
        package=pkg_name,
        executable='dynamixel_unify_baudrate',
        name='dynamixel_unify_baudrate',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    ld.add_action(node)
# 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld
