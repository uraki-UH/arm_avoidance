from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('fuzzy_voxel_grid'), 'config', 'voxel_grid.yaml']
    )

    return LaunchDescription([
        Node(
        package="fuzzy_voxel_grid",
        executable="topo_visualizar",
        output="screen"
        ),
        Node(
            package='fuzzy_voxel_grid',
            executable='voxel_grid_node',
            name='voxel_grid_node',
            output='screen',
            parameters=[params_file],
        )
    ])