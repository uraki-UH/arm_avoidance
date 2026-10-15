"""ブラウザからの汎用launch起動確認用。独立した表示TFのみ。"""
from pathlib import Path
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    model = LaunchConfiguration('model').perform(context)
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', namespace) or model not in ('long', 'standard'):
        raise ValueError('表示TFの名前空間・モデルが不正です')
    app = Path(__file__).resolve().parents[4]/'app'
    path = app/'source.urdf' if model == 'long' else app/'models/standard/source.urdf'
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=namespace, name='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': path.read_text(), 'frame_prefix': namespace+'/'}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('namespace', default_value='browser_preview'),
                              DeclareLaunchArgument('model', default_value='long'), OpaqueFunction(function=setup)])
