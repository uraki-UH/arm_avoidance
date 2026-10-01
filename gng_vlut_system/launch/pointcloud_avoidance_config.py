"""機体設定と共通点群回避設定の合成・起動前検査。"""
from copy import deepcopy
from pathlib import Path
import math
import re
import struct
import xml.etree.ElementTree as et

import yaml
import numpy as np
from scipy.spatial.transform import Rotation


def merge_config(base, override):
    result = deepcopy(base)
    for key, value in override.items():
        result[key] = merge_config(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else deepcopy(value)
    return result


def gng_angle_num(path):
    with Path(path).open('rb') as stream:
        data = stream.read(40)
    if len(data) < 32:
        raise ValueError('GNGヘッダーの欠損')
    version = struct.unpack_from('<I', data)[0]
    if version not in range(1, 10):
        raise ValueError(f'未対応のGNGバージョン: {version}')
    offset = 8 if version >= 6 else 4
    if len(data) < offset + 32:
        raise ValueError('GNG角度配列ヘッダーの欠損')
    if struct.unpack_from('<i', data, offset)[0] <= 0:
        raise ValueError('GNGノードの欠損')
    # ノード数・ID・角度誤差・座標誤差の直後にあるEigen配列サイズ
    rows, cols = struct.unpack_from('<qq', data, offset + 16)
    if not 0 < rows < 1000 or cols != 1:
        raise ValueError('GNG角度配列の次元が不正です')
    return rows


def resolve_clearance_margins(config):
    """一箇所の距離設定から各判定への展開。旧共通設定のみの読込みは従来互換。"""
    if 'clearance_margins' not in config:
        return
    margins = config['clearance_margins']
    keys = {'min_clearance_th', 'min_internal_clearance_th', 'min_planning_clearance_th'}
    if not isinstance(margins, dict) or set(margins) != keys:
        raise ValueError('clearance_marginsには点群・内部停止・内部計画の3項目が必要です')
    if any(key in config for key in keys | {'min_cloud_clearance_th'}):
        raise ValueError('距離余裕の重複設定です。旧トップレベル項目をclearance_marginsへ移してください')
    for key, value in margins.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'clearance_margins.{key}には有限の正数が必要です')
    if margins['min_planning_clearance_th'] < margins['min_internal_clearance_th']:
        raise ValueError('計画時の内部形状余裕は内部停止余裕を確保する値が必要です')
    if margins['min_clearance_th'] >= config['target_clearance']:
        raise ValueError('停止距離は目標余裕より小さい値が必要です')
    config.update(margins)
    # 点群の経路下限と開始・停止距離の共通化
    config['min_cloud_clearance_th'] = margins['min_clearance_th']


def load_config(robot_config, input_config=None, camera_pose=None):
    path = Path(robot_config).resolve()
    robot = yaml.safe_load(path.read_text())
    common_path = path.parent / robot.get('common_config', 'pointcloud_avoidance_common.yaml')
    config = merge_config(yaml.safe_load(common_path.read_text()), robot.get('overrides', {}))
    if input_config:
        config = merge_config(config, yaml.safe_load(Path(input_config).read_text()))
    resolve_clearance_margins(config)
    return_clear_sec = config.get('return_clear_sec', .5)
    if (isinstance(return_clear_sec, bool) or not isinstance(return_clear_sec, (int, float))
            or not math.isfinite(return_clear_sec) or return_clear_sec < 0):
        raise ValueError('return_clear_secには有限の非負数が必要です')
    if camera_pose is not None:
        config['pipeline']['external_cloud']['camera_pose'] = camera_pose
    params_path = (path.parent / robot['params_file']).resolve()
    params = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']
    if re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', params['robot_name']) is None:
        raise ValueError('robot_nameは英字始まりの英数字・下線が必要です')
    root = et.parse(params['urdf_path']).getroot()
    links = {item.get('name') for item in root.findall('link')}
    joints = {item.get('name'): item for item in root.findall('joint')}
    roots = links - {item.find('child').get('link') for item in joints.values()}
    if len(roots) != 1:
        raise ValueError('URDFのルートリンクが一意ではありません')
    root_link = roots.pop()
    groups = robot['planning_groups']
    names = [name for group in groups for name in group['joint_names']]
    group_names = [group['name'] for group in groups]
    if not names or len(names) != len(set(names)) or len(group_names) != len(set(group_names)):
        raise ValueError('計画関節またはグループ名の空配列・重複')
    for group in groups:
        if not group['joint_names'] or not group['link_names'] or not set(group['link_names']) <= links:
            raise ValueError('計画グループの関節・リンク設定が不正です')
        for name in group['joint_names']:
            if name not in joints or joints[name].get('type') == 'fixed' or joints[name].find('mimic') is not None:
                raise ValueError(f'独立可動関節ではありません: {name}')
        # 指定関節の可動側に属する外装・指・固定リンクの監視漏れ防止
        monitored = set(group['link_names']) | {joints[name].find('child').get('link') for name in group['joint_names']}
        while True:
            descendants = {item.find('child').get('link') for item in joints.values()
                           if item.find('parent').get('link') in monitored}
            if descendants <= monitored:
                break
            monitored.update(descendants)
        group['link_names'] = sorted(monitored)
    monitored_names = [name for group in groups for name in group['link_names']]
    if len(monitored_names) != len(set(monitored_names)):
        raise ValueError('計画グループ間で監視リンクが重複しています')
    # 角度配列の順序は学習時の関節順を明示。URDFの列挙順による推定なし
    result_dir = Path(params['gng']['data_directory']) / params['gng']['experiment_id']
    for key, default in [('gng_model_filename', 'gng.bin'), ('vlut_filename', 'vlut.bin')]:
        if not (result_dir / params['gng'].get(key, default)).is_file():
            raise FileNotFoundError(result_dir / params['gng'].get(key, default))
    if gng_angle_num(result_dir / params['gng'].get('gng_model_filename', 'gng.bin')) != len(names):
        raise ValueError('GNG関節数とplanning_groupsの関節数が不一致です')
    pipeline = config['pipeline']
    pipeline['base_frame'] = root_link
    if not pipeline.get('enable_self_filter', True):
        raise ValueError('共通回避では自己除去の省略はできません')
    if 'external_environment' in pipeline:
        external = pipeline['external_environment']
        if pipeline['enable_lidar'] or 'external_cloud' in pipeline or not config.get('enable_live_obstacles', False):
            raise ValueError('既存環境入力にはLiDAR無効・継続回避モードと単一入力が必要です')
        max_age = external.get('max_input_age_sec', 1.0)
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('環境入力期限には有限の正数が必要です')
        source_namespace = '/' + external.get('source_namespace', params['robot_name']).strip('/')
        source_link = params.get('frame_id', root_link).strip('/')
        # 実機ルートとGazeboルートを同じ配置として扱うための固定リンク変換
        transform = np.eye(4)
        child_joints = {item.find('child').get('link'): item for item in joints.values()}
        link = source_link
        visited = set()
        while link != root_link:
            if link in visited or link not in child_joints or child_joints[link].get('type') != 'fixed':
                raise ValueError('環境frame_idにはURDFルートまたは固定リンクだけで接続されたリンクが必要です')
            visited.add(link)
            joint = child_joints[link]
            origin = joint.find('origin')
            step = np.eye(4)
            if origin is not None:
                step[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
                step[:3, :3] = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
            transform = step @ transform
            link = joint.find('parent').get('link')
        if not np.isfinite(transform).all():
            raise ValueError('環境座標系の固定変換に非有限値があります')
        external.update(source_namespace=source_namespace, source_frame=source_namespace.strip('/')+'/'+source_link,
                        root_from_source=transform.tolist(), max_input_age_sec=float(max_age))
        external.setdefault('points_topic', params.get('environment_voxelization', {}).get('input_topic', ''))
        if not isinstance(external['points_topic'], str) or not external['points_topic'].startswith('/'):
            raise ValueError('既存環境の元点群には絶対トピック名が必要です')
        # 任意の未処理ROIへの差替えを避ける固定の自己除去出力名
        for key, suffix in [('voxel_topic', 'self_filter_roi_voxels'), ('graph_topic', 'Tmap_static'),
                            ('state_topic', 'gng_node_states_stamped'),
                            ('feature_topic', 'topological_node_features'), ('joint_topic', 'joint_states')]:
            external[key] = source_namespace+'/'+suffix
    if 'external_cloud' in pipeline:
        external = pipeline['external_cloud']
        if pipeline['enable_lidar'] or not config.get('enable_live_obstacles', False):
            raise ValueError('実点群にはLiDAR無効・継続回避モードが必要です')
        if not all(isinstance(external.get(key), str) and external[key] for key in
                   ('input_topic', 'source_frame', 'camera_frame', 'real_joint_topic', 'robot_camera_link')):
            raise ValueError('実点群の入力トピック・座標系が必要です')
        if external['robot_camera_link'] not in links:
            raise ValueError('実カメラ取付リンクがURDFにありません')
        pose = external.get('camera_pose', [])
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise ValueError('camera_poseに仮想空間でのカメラxyz [m]・rpy [rad]の6値が必要です')
        external['camera_pose'] = [float(value) for value in pose]
        if not math.isfinite(external['max_input_age_sec']) or external['max_input_age_sec'] <= 0:
            raise ValueError('点群入力期限には有限の正数が必要です')
    sensor = pipeline['lidar']
    if len(sensor['pose']) != 6 or not all(math.isfinite(value) for value in sensor['pose']):
        raise ValueError('LiDAR poseには有限のxyz/rpyが必要です')
    for key in ('voxel_size', 'publish_hz'):
        if not math.isfinite(pipeline[key]) or pipeline[key] <= 0:
            raise ValueError(f'{key}には有限の正数が必要です')
    if len(pipeline['roi_min']) != 3 or len(pipeline['roi_max']) != 3 or not all(
            math.isfinite(a) and math.isfinite(b) and a < b for a, b in zip(pipeline['roi_min'], pipeline['roi_max'])):
        raise ValueError('ROIの範囲が不正です')
    for key in ('update_hz', 'min_range', 'max_range', 'range_resolution', 'horizontal_angle', 'vertical_angle'):
        if not math.isfinite(sensor[key]) or sensor[key] <= 0:
            raise ValueError(f'LiDAR {key}には有限の正数が必要です')
    if sensor['min_range'] >= sensor['max_range']:
        raise ValueError('LiDAR距離範囲が不正です')
    for key in ('horizontal_samples', 'vertical_samples'):
        if type(sensor[key]) is not int or sensor[key] < 2:
            raise ValueError(f'LiDAR {key}にはサンプル数が必要です')
    config.update(planning_groups=groups, root_link=root_link,
                  trail_links=[joints[group['joint_names'][-1]].find('child').get('link') for group in groups])
    config['enable_gng_vlut'] = True
    return params_path, params, config
