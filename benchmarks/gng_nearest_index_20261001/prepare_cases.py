"""同一入力・同一seedでの実学習比較用manifestと入力ハッシュの作成。"""
import argparse
import hashlib
import json
from pathlib import Path

import yaml


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(data)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--binary', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--datasets', nargs='+', choices=('max_10k', 'long_10k', 'max_final', 'long_final'),
                        default=['max_final', 'long_final'])
    parser.add_argument('--profiles', nargs='+', choices=('standard', 'churn', 'churn_delete'), default=['standard'])
    parser.add_argument('--num-angle-iter', type=int, default=2000)
    parser.add_argument('--num-coord-iter', type=int, default=5000)
    parser.add_argument('--num-samples', type=int, default=4096)
    args = parser.parse_args()
    root = args.root.resolve()
    provenance_path = args.output.with_suffix('.inputs.json')
    assert not args.output.exists() and not provenance_path.exists()
    standard = {'n_best_candidates': 4, 'ais_threshold': 10.0, 'max_edge_age': 2000,
                'beta': .0005, 'lambda': 200, 'learn_rate_s1': .08, 'learn_rate_s2': .008, 'alpha': .5}
    provenance = {'binary': args.binary, 'datasets': {}, 'standard_config': standard,
                  'churn_overrides': {'max_edge_age': 2, 'lambda': 25, 'ais_threshold': .05},
                  'churn_delete_overrides': {'max_edge_age': 0, 'lambda': 25, 'ais_threshold': .05}}
    cases = []
    for dataset in args.datasets:
        model, scale = dataset.rsplit('_', 1)
        robot = 'topo_dual_arm_max' + ('_long' if model == 'long' else '')
        folder = root / f'urdf/{robot}'
        config = root / f'gng_vlut_system/config/{robot}.yaml'
        params = yaml.safe_load(config.read_text())['/**']['ros__parameters']['gng_params']
        extracted = {name: params[name] for name in standard}
        assert extracted == standard, '通常学習設定とベンチ定数の不一致'
        source = (root / f'gng_vlut_system/gng_results/{robot}/gng.bin' if scale == '10k'
                  else root / f'artifacts/gng_self_collision_fix_20261001/{model}/model/gng.bin')
        urdf = folder / 'topo_dual_arm_max.urdf'
        inputs = [source, config, urdf]
        provenance['datasets'][dataset] = {'input_sha256': {str(path): digest(path) for path in inputs},
                                            'extracted_config': extracted}
        mapped = lambda path: str(Path('/ros2_ws/src') / path.relative_to(root))
        for profile in args.profiles:
            for flag, name in [('0', 'linear'), ('1', 'indexed')]:
                argv = ['taskset', '-c', '0', args.binary, '--input', mapped(source),
                        '--urdf', mapped(urdf), '--resource-root', mapped(folder),
                        '--mesh-root', mapped(folder / 'meshes'), '--enable-nearest-index', flag,
                        '--profile', profile, '--seed', '@seed@', '--num-angle-iter', str(args.num_angle_iter),
                        '--num-coord-iter', str(args.num_coord_iter), '--num-samples', str(args.num_samples),
                        '--output', '@case_dir@/result']
                cases.append({'name': f'{dataset}_{profile}_{name}', 'argv': argv,
                              'metrics': 'result/metrics.numeric.json',
                              'env': {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}})
    with provenance_path.open('x') as stream:
        json.dump(provenance, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    with args.output.open('x') as stream:
        json.dump({'cases': cases, 'input_provenance': str(provenance_path)}, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps({'manifest': str(args.output), 'num_cases': len(cases),
                      'provenance': str(provenance_path)}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
