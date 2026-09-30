"""ローカルボクセル出力時のURDF・メッシュ・生成コードの指紋記録。"""
import argparse
import hashlib
import json
from pathlib import Path


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    data = json.loads(args.input.read_text())
    if 'geometry_sha256' in data:
        raise ValueError('指紋記録済み出力への上書き拒否')
    urdf_relative = Path(data['urdf_path']).relative_to('/ros2_ws/src')
    geometry = {}
    for value in data['geometry_files']:
        value = value.removeprefix('file://')
        path = Path(value)
        if path.is_absolute():
            path = root / path.relative_to('/ros2_ws/src')
        else:
            path = root / urdf_relative.parent / path
        geometry[str(path.relative_to(root))] = sha256(path)
    data['geometry_sha256'] = geometry
    sources = ['gng_vlut_system/src/core/robot_model/robot_voxelizer.hpp',
               'gng_vlut_system/src/core/common/voxelizer_engine.hpp',
               'gng_vlut_system/src/core/safety_engine/recognition/urdf_geometry_simplifier.hpp']
    data['voxelizer_source_sha256'] = {name: sha256(root / name) for name in sources}
    args.input.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'num_geometry_files': len(geometry), 'output': str(args.input)}), flush=True)


if __name__ == '__main__':
    main()
