"""追跡球の保存結果から表示専用JSONへの変換。ROS・物理計算・実機出力なし。"""
import argparse
import hashlib
import json
import tempfile
import time
from pathlib import Path


simulator_dir = Path(__file__).resolve().parents[2]
workspace_dir = simulator_dir.parents[1]
artifact_dir = workspace_dir/'log/artifacts/chasing_sphere'


def read_json(path):
    if path.stat().st_size > 16*1024*1024:
        raise ValueError('記録ファイルの容量超過')
    return json.loads(path.read_text())


def recording_metadata(details):
    """逐次表示と保存再生に共通の表示メタデータ。"""
    config = details['config']
    result = dict(format='topo-avoidance-replay/1',model='long',
        policy=config['policy'],candidate_metric=config.get('candidate_metric','joint'),seed=config['seed'],
        target_link=config.get('target_link','L_tcp'),obstacle_shape=config.get('obstacle_shape','sphere'),
        has_legacy_planning_domain='joint_position_limits' not in details and not details.get('enable_periodic_angles',config.get('enable_periodic_angles',False)),
        has_unbounded_joint_limits='joint_position_limits' not in details and details.get('enable_periodic_angles',config.get('enable_periodic_angles',False)),
        joint_position_limits=details.get('joint_position_limits',{}),
        is_valid_scenario=details.get('is_valid_scenario',True),failure_details=details.get('failure_details',{}),
        failure_reason=details.get('failure_reason',''),metrics=details.get('metrics',{}))
    if details.get('contact_rule')=='target_tip':
        result.update(contact_rule='target_tip',outcome=details.get('outcome','unfinished'),
            enable_proximal_guard=config.get('enable_proximal_guard',True),target_contact_links=details['target_contact_links'])
    return result


def export_recording(case_dir, output_dir):
    details = read_json(case_dir/'details.json')
    trajectory = case_dir/'trajectory.jsonl'
    if trajectory.stat().st_size > 16*1024*1024:
        raise ValueError('軌跡ファイルの容量超過')
    frames = [json.loads(line) for line in trajectory.read_text().splitlines() if line.strip()]
    if not 2 <= len(frames) <= 20000:
        raise ValueError('記録フレーム数の範囲外')
    keys = ('time_sec','joints','eef','sphere','sphere_radius_m','phase','has_contact','proxy_gap_m')
    frames = [{**{key:frame[key] for key in keys},**{key:frame[key] for key in ('target','obstacle_segments','obstacle_phase','obstacle_speed_mps') if key in frame}} for frame in frames]
    result = dict(recording_metadata(details),frames=frames)
    body = json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n'
    name = 'avoidance-'+hashlib.sha256(body.encode()).hexdigest()[:20]+'.json'
    path = output_dir/name
    try:
        if not (output_dir/'.avoidance-trash'/(name+'.entry')).exists() and not (output_dir/'.avoidance-deleted'/name).exists():
            with path.open('x') as stream:
                stream.write(body)
            path.chmod(0o644)
    except FileExistsError:
        if path.read_text() != body:
            raise ValueError('表示記録の内容ハッシュ不一致')
    source_case_dir = str(case_dir.resolve().relative_to(workspace_dir)) if case_dir.resolve().is_relative_to(workspace_dir) else None
    return dict(url='/exports/'+name,label=case_dir.name,source_case_dir=source_case_dir,policy=result['policy'],
                candidate_metric=result['candidate_metric'],seed=result['seed'],
                target_link=result['target_link'],obstacle_shape=result['obstacle_shape'],
                has_legacy_planning_domain=result['has_legacy_planning_domain'],
                has_unbounded_joint_limits=result['has_unbounded_joint_limits'],
                failure_reason=result['failure_reason'],duration_sec=frames[-1]['time_sec'])


def export_catalog(case_dir='latest', output_dir=None):
    """一覧更新とGUI削除の排他。ごみ箱の記録の再掲載防止。"""
    output_dir = Path(output_dir) if output_dir is not None else simulator_dir/'app/exports'
    output_dir.mkdir(parents=True,exist_ok=True)
    lock = output_dir/'.avoidance-catalog.lock'
    end_sec = time.monotonic()+2.
    while True:
        try:
            lock.mkdir()
            break
        except FileExistsError:
            if time.monotonic()>=end_sec:
                raise ValueError('実験一覧を更新中です。再度操作してください')
            time.sleep(.02)
    try:
        return export_unlocked_catalog(case_dir,output_dir)
    finally:
        lock.rmdir()


def export_unlocked_catalog(case_dir, output_dir):
    """正式な保持条件・手動試験のカタログ。評価修正前の参考データの自動掲載なし。"""
    output_dir = Path(output_dir) if output_dir is not None else simulator_dir/'app/exports'
    output_dir.mkdir(parents=True,exist_ok=True)
    candidates = [path.parent for path in artifact_dir.glob('holdout/*/details.json')]
    candidates += [path.parent for path in artifact_dir.glob('manual_*/details.json')]
    for pattern in ('final_holdout/*/details.json','manual_*/details.json'):
        candidates += [path.parent for path in (artifact_dir.parent/'pursuit_v2').glob(pattern)]
    candidates += [path.parent for path in (artifact_dir.parent/'pursuit_fixes').glob('verified/*/details.json')]
    candidates = [path for path in candidates if (path/'trajectory.jsonl').is_file()]
    if str(case_dir) == 'latest':
        if not candidates:
            raise ValueError('追跡球の記録なし。先にシミュレーションの実行が必要')
        selected = max(candidates,key=lambda path:(path/'details.json').stat().st_mtime_ns)
    else:
        selected = Path(case_dir).resolve()
        if not (selected/'details.json').is_file() or not (selected/'trajectory.jsonl').is_file():
            raise ValueError('details.jsonとtrajectory.jsonlのある試験ディレクトリが必要')
        candidates.append(selected)
    candidates = sorted(set(candidates),key=lambda path:(path/'details.json').stat().st_mtime_ns,reverse=True)
    candidates = [selected]+[path for path in candidates if path != selected][:255]
    selected_url = None
    runs = []
    for path in candidates[:256]:
        # 無関係な破損記録の明示スキップ。指定した試験の欠損はエラー。
        try:
            entry = export_recording(path,output_dir)
        except (OSError,ValueError,KeyError,TypeError):
            if path == selected:
                raise
            continue
        if (output_dir/'.avoidance-trash'/(Path(entry['url']).name+'.entry')).exists() or (output_dir/'.avoidance-deleted'/Path(entry['url']).name).exists():
            # ごみ箱に退避済みの元ログからの再掲載の回避。元ログ自体は保持。
            continue
        runs.append(entry)
        if path == selected:
            selected_url = entry['url']
    if selected_url is None and runs:
        selected_url = runs[0]['url']
    catalog = dict(format='topo-avoidance-catalog/1',selected=selected_url,runs=runs)
    # 所有する最新カタログ参照だけの置換。内容ハッシュ付きの記録本体は保持。
    filename = output_dir/'avoidance-replays.json'
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w',dir=output_dir,prefix='avoidance-replays-',suffix='.tmp',delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(json.dumps(catalog,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
        temporary.chmod(0o644)
        temporary.replace(filename)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return '/exports/'+filename.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-dir',default='latest')
    args = parser.parse_args()
    try:
        print(export_catalog(args.case_dir))
    except (OSError,ValueError,KeyError,TypeError) as error:
        parser.exit(2,str(error)+'\n')


if __name__ == '__main__':
    main()
