"""CPUコンテナ内の共通処理。Compose YAMLによる機能・ビルド対象・機体設定の選択。"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import yaml


workspace_dir = Path(__file__).resolve().parents[1]
simulator_dir = workspace_dir/'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator'
config_path = workspace_dir/'docker/compose.simulator_cpu.yaml'


def source_path(value):
    """ソースルート内の相対パス。PC間で共通の設定表現。"""
    if not isinstance(value, str) or not value or Path(value).is_absolute() or '..' in Path(value).parts:
        raise ValueError('パスにはソースルート内の相対パスが必要です')
    result = (workspace_dir/value).resolve()
    if not result.is_relative_to(workspace_dir):
        raise ValueError('ソースルート外のパスは未対応です')
    return result


def load_config(path=config_path):
    config = yaml.safe_load(path.read_text())
    config = config.get('x-cpu-workflow') if isinstance(config, dict) else None
    if not isinstance(config, dict) or set(config) != {'enable_gng', 'model', 'params_file', 'build', 'check', 'demo'}:
        raise ValueError('x-cpu-workflowの設定項目が不正です')
    if type(config['enable_gng']) is not bool or config['model'] not in ('standard', 'long'):
        raise ValueError('GNG有効設定または機体名が不正です')
    source_path(config['params_file'])
    if not isinstance(config['demo'], dict) or set(config['demo']) != {'scenario_file'}:
        raise ValueError('demoの設定項目が不正です')
    source_path(config['demo']['scenario_file'])
    build, check = config['build'], config['check']
    if not isinstance(build, dict) or set(build) != {'build_type', 'executor', 'base_paths', 'base_packages', 'gng_packages'}:
        raise ValueError('buildの設定項目が不正です')
    if build['build_type'] not in ('Release', 'RelWithDebInfo', 'Debug', 'MinSizeRel') or build['executor'] not in ('sequential', 'parallel'):
        raise ValueError('ビルド方式が不正です')
    for key in ('base_paths', 'base_packages', 'gng_packages'):
        values = build[key]
        if not isinstance(values, list) or not values or any(not isinstance(value, str) or not value for value in values):
            raise ValueError('ビルド対象の配列が不正です: '+key)
        if len(values) != len(set(values)):
            raise ValueError('ビルド対象の重複: '+key)
        for value in values:
            if key == 'base_paths':
                source_path(value)
            elif re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value) is None:
                raise ValueError('パッケージ名が不正です: '+value)
    if not isinstance(check, dict) or set(check) != {'duration_sec'}:
        raise ValueError('checkの設定項目が不正です')
    duration = check['duration_sec']
    if type(duration) not in (int, float) or not math.isfinite(duration) or not 0 < duration <= 60:
        raise ValueError('確認時間には有限の正数が必要です（上限60 sec）')
    return config


def build_command(config):
    build = config['build']
    packages = build['base_packages']+(build['gng_packages'] if config['enable_gng'] else [])
    return ['colcon', 'build', '--symlink-install', '--executor', build['executor'],
            '--base-paths', *(str(source_path(value)) for value in build['base_paths']),
            '--packages-up-to', *dict.fromkeys(packages), '--cmake-args',
            '-DCMAKE_BUILD_TYPE='+build['build_type'], '-DBUILD_TESTING=OFF']


def option_present(arguments, option):
    return any(value == option or value.startswith(option+'=') for value in arguments)


def simulation_command(config, arguments):
    defaults = dict(model=config['model'], params_file=str(source_path(config['params_file'])),
                    avoidance='gng' if config['enable_gng'] or '--nonstop-simulation' in arguments else 'none')
    command = ['bash', str(simulator_dir/'start_ros.sh'), '--simulate']
    for key, value in defaults.items():
        option = '--'+key.replace('_', '-')
        if not option_present(arguments, option):
            command.extend((option, value))
    return command+list(arguments)


def execute(command, **options):
    return subprocess.run(command, check=True, **options)


def build_workspace(config):
    execute(build_command(config), cwd=workspace_dir.parent)
    execute(['bash', str(simulator_dir/'integrations/ros2/prepare_ui.sh')], cwd=simulator_dir)


def check_workspace(config):
    execute([sys.executable, '-c',
             'import rclpy, tornado, scipy, mujoco, osqp; '
             'import ais_gng_msgs.msg, ais_gng_feature_msgs.msg, gng_control_msgs.srv, voxel_msgs.msg; '
             'print("ROS・CPU依存: OK")'], timeout=30)
    for path in ('app/generated/ros-results.js', 'app/vendor/three/build/three.module.js'):
        if not (simulator_dir/path).is_file():
            raise ValueError('UI構築が必要です: '+path)
    execute([sys.executable, str(simulator_dir/'integrations/ros2/native_points.py')], timeout=60)
    execute(simulation_command(config, ['--avoidance', 'none', '--duration', str(config['check']['duration_sec'])]),
            timeout=90)
    if not config['enable_gng']:
        return
    for package in config['build']['gng_packages']:
        execute(['ros2', 'pkg', 'prefix', package], timeout=15)
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(simulator_dir/'integrations/ros2')+os.pathsep+environment.get('PYTHONPATH', '')
    execute([sys.executable, '-c',
             'import sys; from gng_inputs import load_models; '
             'params, models = load_models(sys.argv[1]); print("GNG学習済みデータ: OK", list(models))',
             str(source_path(config['params_file']))], env=environment, timeout=30)


def demo_workspace(config):
    """同梱設定による有限回避。正常終了とは別の接触・実行時間・両者の移動確認。"""
    if config['model'] != 'long':
        raise ValueError('同梱CPU回避デモの対象はlongです')
    scenario = source_path(config['demo']['scenario_file'])
    settings = json.loads(scenario.read_text())
    destination = workspace_dir/'log/artifacts/cpu_demo'/str(time.time_ns())
    destination.mkdir(parents=True, exist_ok=False)
    execute(simulation_command(config, ['--scenario', 'chasing_sphere', '--avoidance', 'gng', '--scenario-file', str(scenario),
                                      '--case-dir', str(destination)]), timeout=180)
    result = json.loads((destination/'details.json').read_text())
    metrics = result['metrics']
    if (result['failure_reason'] or not result['is_valid_scenario'] or metrics['contact_sec'] != 0
            or metrics['evaluated_sec'] < settings['duration_sec']-.02
            or metrics['max_ball_speed_mps'] <= 0 or metrics['max_eef_speed_mps'] <= 0
            or result['phase_sec'].get('avoiding', 0) <= 0):
        raise ValueError('CPU回避デモの確認失敗。結果: '+str(destination/'details.json'))
    print('CPU回避デモ: OK / 結果: '+str(destination/'details.json'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('build', 'check', 'demo', 'simulate'))
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.operation != 'simulate' and args.arguments:
        parser.error('追加引数はsimulateでのみ指定できます')
    try:
        config = load_config()
        if args.operation == 'simulate':
            command = simulation_command(config, args.arguments)
            os.execvp(command[0], command)
        dict(build=build_workspace, check=check_workspace, demo=demo_workspace)[args.operation](config)
    except (ValueError, yaml.YAMLError, OSError, subprocess.SubprocessError) as error:
        print('CPU構成の実行失敗: '+str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
