"""world索引・ROIとFVGによる、一つの点群実体の同一プロセス内共有。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    value = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/dataset/points'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('target_frame', default_value='world'),
        DeclareLaunchArgument('world_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('fuzzy_voxel_grid'), 'config', 'shared_world.yaml'])),
        DeclareLaunchArgument('fvg_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('fuzzy_voxel_grid'), 'config', 'voxel_grid.yaml'])),
        ComposableNodeContainer(
            package='rclcpp_components', executable='component_container_mt',
            name='shared_world_voxel', namespace='', output='screen',
            composable_node_descriptions=[
                ComposableNode(package='gng_vlut_system',
                    plugin='robot_sim::bridge::WorldIndexToVoxelNode',
                    name='world_index_to_voxel_node',
                    parameters=[value('world_params_file'), {
                        'input_topic': value('input_topic'),
                        'world_frame_id': value('world_frame'),
                        'target_frame_id': value('target_frame'),
                        'shared_point_store': 'world_points',
                        'enable_world_index': True}]),
                ComposableNode(package='fuzzy_voxel_grid',
                    plugin='fuzzy_voxel_grid::VoxelGridNode', name='voxel_grid_node',
                    parameters=[value('fvg_params_file'), {'shared_point_store': 'world_points'}]),
            ]),
    ])
