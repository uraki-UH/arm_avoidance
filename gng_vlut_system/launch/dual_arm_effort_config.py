"""シミュレータ共通の関節選択・トルク制限・標準effort制御設定。"""
import math
import re
import xml.etree.ElementTree as et
from pathlib import Path


def default_urdf(model='topo_dual_arm_max_long'):
    """インストール済み資産を優先した機体URDFの解決。ソース直接起動にも対応。"""
    if model not in ('topo_dual_arm_max', 'topo_dual_arm_max_long'):
        raise ValueError('未対応の機体です: ' + model)
    package_dir = Path(__file__).resolve().parents[1]
    for root in (package_dir, package_dir.parent):
        path = root / 'urdf' / model / 'topo_dual_arm_max.urdf'
        if path.is_file():
            return path
    raise FileNotFoundError('URDF資産が見つかりません: ' + model)


def validate_namespace(namespace):
    """ROS名前空間と機体名の検査。"""
    if not re.fullmatch(r'sim_[a-zA-Z0-9_]+', namespace):
        raise ValueError('sim_で始まる単一の名前空間が必要です')
    return namespace


def load_model(urdf_path):
    """元ファイルを保持したURDF読込と相対メッシュパスの解決。"""
    urdf_path = Path(urdf_path).resolve()
    root = et.parse(urdf_path).getroot()
    for mesh in root.findall('.//mesh'):
        source = mesh.get('filename', '')
        if not source:
            raise ValueError('メッシュパスが必要です')
        if not source.startswith(('file://', 'package://')):
            source = str((urdf_path.parent/source).resolve())
            if not Path(source).is_file():
                raise FileNotFoundError(source)
            mesh.set('filename', 'file://'+source)
    return root


def effort_joints(root, tuning):
    """独立関節のPIDとURDF由来の出力制限。mimicは指令対象外。"""
    gains = {}
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed' or joint.find('mimic') is not None:
            continue
        name = joint.get('name')
        limit = joint.find('limit')
        effort = float(limit.get('effort')) if limit is not None else math.nan
        if not math.isfinite(effort) or effort <= 0:
            raise ValueError(f'有限の正のトルク上限が必要です: {name}')
        item = dict(tuning['default_gains'], **tuning.get('joint_gains', {}).get(name, {}))
        for key, value in item.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f'有限の非負ゲインが必要です: {name}/{key}')
        item.update(u_clamp_min=-effort, u_clamp_max=effort, ff_velocity_scale=0.0)
        gains[name] = item
    if not gains:
        raise ValueError('独立可動関節が必要です')
    return gains


def controller_parameters(gains, namespace=''):
    """標準Controller ManagerとJointTrajectoryControllerの設定。"""
    prefix = '/'+validate_namespace(namespace)+'/' if namespace else ''
    return {
        prefix+'controller_manager': {'ros__parameters': {
            'update_rate': 1000, 'use_sim_time': True,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'dual_arm_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        prefix+'dual_arm_controller': {'ros__parameters': {
            'joints': list(gains), 'command_interfaces': ['effort'], 'state_interfaces': ['position', 'velocity'],
            'gains': gains, 'state_publish_rate': 200.0, 'action_monitor_rate': 50.0,
            'allow_partial_joints_goal': False, 'open_loop_control': False,
            'constraints': {'goal_time': 0.0, 'stopped_velocity_tolerance': .02}}}}
