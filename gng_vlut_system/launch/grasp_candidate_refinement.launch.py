from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 既存の候補生成・計画launchから独立した、購読と補正結果の出力
    names = {
        'candidate_topic': '/grasp_pose_cands',
        'point_cloud_topic': '/camera/camera/depth/color/points',
        'seed_topic': '/ToPoDualArm/grasp_candidate_metrics',
        'output_topic': '/grasp_pose_refined',
    }
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('gng_vlut_system'), 'config', 'grasp_candidate_refinement.yaml'])),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in names.items()],
        DeclareLaunchArgument('enable_ik', default_value='true'),
        Node(package='gng_vlut_system', executable='grasp_candidate_refiner_node',
             name='grasp_candidate_refiner', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     **{name: LaunchConfiguration(name) for name in names},
                     'enable_ik': ParameterValue(LaunchConfiguration('enable_ik'), value_type=bool),
                 }]),
    ])
