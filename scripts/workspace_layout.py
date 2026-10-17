"""共有ソース・任意機能・独立プロジェクトの構成検証とROS探索パスの解決。"""
import argparse
from pathlib import Path
import re
import subprocess

import yaml


workspace_dir = Path(__file__).resolve().parents[1]
layout_path = workspace_dir/'workspace.yaml'


def relative_path(value):
    """ソースルート内の正規化済み相対パス。上位参照・外部リンクの除外。"""
    if (not isinstance(value, str) or not value or any(char in value for char in ('\x00', '\n', '\r')) or Path(value).is_absolute()
            or '..' in Path(value).parts or str(Path(value)) != value or value == '.'):
        raise ValueError('構成パスには正規化した相対パスが必要です: '+str(value))
    if not (workspace_dir/value).resolve().is_relative_to(workspace_dir):
        raise ValueError('構成パスがソースルート外です: '+value)
    return value


def path_list(values):
    if not isinstance(values, list) or len(values) != len(set(map(str, values))):
        raise ValueError('構成パスの配列・重複の不正')
    return [relative_path(value) for value in values]


def load_layout(path=layout_path):
    layout = yaml.safe_load(path.read_text())
    if not isinstance(layout, dict) or set(layout) != {'version', 'components', 'local_projects', 'external_projects'} or type(layout['version']) is not int or layout['version'] != 2:
        raise ValueError('workspace.yamlの形式不正')
    components = layout['components']
    if not isinstance(components, dict) or not components:
        raise ValueError('構成部品の定義不足')
    roots = []
    for name, component in components.items():
        if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', name) or not isinstance(component, dict):
            raise ValueError('構成部品名・定義の不正')
        if (set(component)-{'label', 'kind', 'paths', 'ros_base_paths', 'repositories'}
                or not isinstance(component.get('label'), str) or component.get('kind') not in ('core', 'optional')):
            raise ValueError('構成部品の項目不正: '+name)
        paths = path_list(component.get('paths'))
        if not paths:
            raise ValueError('構成部品の配置不足: '+name)
        roots.extend(paths)
        for value in path_list(component.get('ros_base_paths', [])):
            if not any(Path(value).is_relative_to(Path(root)) for root in paths):
                raise ValueError('ROS探索パスが部品の配置外です: '+value)
        if 'repositories' in component:
            value = relative_path(component['repositories'])
            if component['kind'] != 'optional' or not any(Path(value).is_relative_to(Path(root)) for root in paths):
                raise ValueError('外部取得定義は任意部品内への配置が必要です')
    roots.extend(path_list(layout['local_projects']))
    external = layout['external_projects']
    if not isinstance(external, dict):
        raise ValueError('外部プロジェクトの定義不正')
    targets = []
    for name, value in external.items():
        if (not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_-]*', name)
                or not isinstance(value, str) or not value or Path(value).is_absolute()
                or any(char in value for char in ('\x00', '\n', '\r')) or str(Path(value)) != value):
            raise ValueError('外部プロジェクトの名前・配置不正')
        target = (workspace_dir/value).resolve()
        if target.is_relative_to(workspace_dir) or workspace_dir.is_relative_to(target):
            raise ValueError('外部プロジェクトは本体外への配置が必要です: '+name)
        targets.append(target)
    for idx, target in enumerate(targets):
        if any(target.is_relative_to(other) or other.is_relative_to(target) for other in targets[idx+1:]):
            raise ValueError('外部プロジェクトの配置重複')
    roots.extend(external)
    for idx, root in enumerate(roots):
        if any(Path(root).is_relative_to(Path(other)) or Path(other).is_relative_to(Path(root)) for other in roots[idx+1:]):
            raise ValueError('構成部品・独立プロジェクトの配置重複: '+root)
    return layout


def ros_base_paths(names, layout=None):
    """明示した部品だけのROS探索範囲。任意部品の自動追加なし。"""
    layout = load_layout() if layout is None else layout
    if not isinstance(names, list) or not names or any(not isinstance(name, str) for name in names) or len(names) != len(set(names)):
        raise ValueError('構築部品の配列・重複の不正')
    result = []
    for name in names:
        component = layout['components'].get(name)
        if component is None or not component.get('ros_base_paths'):
            raise ValueError('ROS探索対象のない構築部品: '+name)
        for value in component['ros_base_paths']:
            if not (workspace_dir/value).is_dir():
                raise ValueError('ROS探索先の不足: '+value)
            if value not in result:
                result.append(value)
    return result


def check_layout(layout=None, enable_git_check=True):
    layout = load_layout() if layout is None else layout
    for component in layout['components'].values():
        if component['kind'] == 'optional' and not any((workspace_dir/value).exists() for value in component['paths']):
            continue
        for value in component['paths']:
            if not (workspace_dir/value).is_dir():
                raise ValueError('構成部品の配置不足: '+value)
        if 'repositories' not in component:
            continue
        manifest = yaml.safe_load((workspace_dir/component['repositories']).read_text())
        if not isinstance(manifest, dict) or set(manifest) != {'repositories'} or not isinstance(manifest['repositories'], dict) or not manifest['repositories']:
            raise ValueError('外部取得定義の形式不正')
        for name, spec in manifest['repositories'].items():
            relative_path(name)
            if (not isinstance(spec, dict) or set(spec) != {'type', 'url', 'version'} or spec['type'] != 'git'
                    or not re.fullmatch(r'https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+\.git', spec['url'])
                    or not re.fullmatch(r'[0-9a-f]{40}', spec['version'])):
                raise ValueError('外部取得定義にはGit URLと固定commitが必要です: '+name)
    if enable_git_check and (workspace_dir/'.git').exists():
        tracked = subprocess.run(['git', '-c', 'safe.directory='+str(workspace_dir), '-C', str(workspace_dir), 'ls-files', '-z', '--', *layout['local_projects'], *layout['external_projects']],
                                 capture_output=True, check=True).stdout
        if tracked:
            raise ValueError('独立プロジェクトに本リポジトリの追跡対象が存在します')
    return layout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('list', 'check', 'ros-paths'))
    parser.add_argument('components', nargs='*', help='ROS探索対象の部品名。省略時は中核のみ')
    args = parser.parse_args()
    if args.components and args.operation != 'ros-paths':
        parser.error('部品名はros-pathsでのみ指定できます')
    try:
        layout = check_layout() if args.operation == 'check' else load_layout()
        if args.operation == 'ros-paths':
            names = args.components or [name for name, component in layout['components'].items() if component['kind'] == 'core' and component.get('ros_base_paths')]
            print('\n'.join(str(workspace_dir/value) for value in ros_base_paths(names, layout)))
        elif args.operation == 'check':
            print('ワークスペース構成: OK')
        else:
            for kind, label in (('core', '中核'), ('optional', '任意機能')):
                print(label)
                for name, component in layout['components'].items():
                    if component['kind'] == kind:
                        print('  '+name+': '+', '.join(component['paths']))
            print('独立プロジェクト（共有対象外）: '+', '.join(layout['local_projects']))
            for name, value in layout['external_projects'].items():
                print('外部プロジェクト: '+name+' → '+value)
    except (ValueError, TypeError, OSError, yaml.YAMLError, subprocess.SubprocessError) as error:
        parser.exit(1, 'ワークスペース構成の不正: '+str(error)+'\n')


if __name__ == '__main__':
    main()
