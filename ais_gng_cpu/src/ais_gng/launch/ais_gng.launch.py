import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

package_dir = get_package_share_directory("ais_gng")

def generate_launch_description():
    declar_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='femto.yaml',
        description='config'
    )
    declar_backend = DeclareLaunchArgument(
        'backend',
        default_value='cpu',
        description='backend to launch: cpu or gpu'
    )
    declar_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='',
        description='入力PointCloud2トピックの上書き'
    )
    def launch_setup(context, *args, **kwargs):
        backend = LaunchConfiguration('backend').perform(context)
        if backend not in ('cpu', 'gpu'):
            raise RuntimeError(f"Unsupported backend: {backend}")

        lidar = LaunchConfiguration('lidar').perform(context)
        gng_config_path = os.path.join(package_dir, 'config', f'gng_{backend}', lidar)
        if not os.path.exists(gng_config_path):
            raise RuntimeError(f"Config file not found: {gng_config_path}")

        executable_path = os.path.join(
            get_package_prefix("ais_gng"),
            "lib",
            "ais_gng",
            f"ais_gng_{backend}",
        )
        if not os.path.exists(executable_path):
            raise RuntimeError(f"Backend executable not found: {executable_path}")

        parameters = [gng_config_path]
        input_topic = LaunchConfiguration('input_topic').perform(context)
        if input_topic:
            parameters.append({'input.topic_names': [input_topic]})

        return [
            Node(
                package="ais_gng",
                executable=f"ais_gng_{backend}",
                parameters=parameters,
                output='screen',
                # prefix='gdb -ex run -ex bt --args', # for debugging
                # arguments=['--ros-args', '--log-level', 'WARN'] # no screen log
            ),
            Node(
                package='ais_gng',
                executable='object_gng_dataset_exporter_node',
                output='screen',
            ),
        ]

    return LaunchDescription([
        declar_lidar,
        declar_backend,
        declar_input_topic,
        OpaqueFunction(function=launch_setup),
    ])
