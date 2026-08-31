import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_dataset_path(dataset_dir, dataset_file, dataset_id):
    dataset_key = dataset_file or dataset_id
    if not dataset_key:
        raise RuntimeError('dataset_file またはdataset_idの指定が必要です。')

    if os.path.isabs(dataset_key):
        candidates = [dataset_key]
    elif dataset_key.endswith(('.json', '.json.gz')):
        candidates = [os.path.join(dataset_dir, dataset_key)]
    else:
        candidates = [
            os.path.join(dataset_dir, f'{dataset_key}_gng_template_v1.json.gz'),
            os.path.join(dataset_dir, f'{dataset_key}_object_surface_dataset_v1.json'),
            os.path.join(dataset_dir, f'{dataset_key}_gng_template_v1.json'),
        ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    if not os.path.isabs(dataset_key) and not dataset_key.endswith(('.json', '.json.gz')):
        matched_paths = sorted(set(
            glob.glob(os.path.join(dataset_dir, f'{dataset_key}*_gng_template_v1.json.gz')) +
            glob.glob(os.path.join(dataset_dir, f'{dataset_key}*_object_surface_dataset_v1.json')) +
            glob.glob(os.path.join(dataset_dir, f'{dataset_key}*_gng_template_v1.json'))
        ))
        if len(matched_paths) == 1:
            return matched_paths[0]
        if len(matched_paths) > 1:
            raise RuntimeError(
                f'dataset_fileに一致するデータセットが複数あります: {dataset_key}')

    raise RuntimeError(f'データセットを見つけられません: {dataset_key}')


def create_publisher_node(context):
    dataset_dir = LaunchConfiguration("dataset_dir").perform(context).rstrip("/")
    dataset_file = LaunchConfiguration("dataset_file").perform(context)
    dataset_id = LaunchConfiguration("dataset_id").perform(context)
    dataset_path = resolve_dataset_path(dataset_dir, dataset_file, dataset_id)
    return [Node(
        package="gng_vlut_system",
        executable="object_template_map_publisher_node",
        name="object_template_map_publisher_node",
        output="screen",
        parameters=[{
            "dataset_path": dataset_path,
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_hz": LaunchConfiguration("publish_hz"),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dataset_dir", default_value="/datasets"),
        DeclareLaunchArgument("dataset_file", default_value=""),
        DeclareLaunchArgument("dataset_id", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="object_template"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        OpaqueFunction(function=create_publisher_node),
    ])
