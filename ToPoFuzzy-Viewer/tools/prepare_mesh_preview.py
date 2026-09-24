#!/usr/bin/env python3
"""原本を保持したOBJの表示用GLB生成。座標の変更は明示倍率のみ。"""

import argparse
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np
from PIL import Image
import pymeshlab
import trimesh


def file_hash(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def mesh_bounds(mesh):
    box = mesh.bounding_box()
    return np.array([box.min(), box.max()])


def prepare_preview(source, output_dir, *, num_faces=300000, max_texture_size=4096,
                    unit_scale=1.0, attribution_file=None):
    source = Path(source).resolve(strict=True)
    output_dir = Path(output_dir).absolute()
    if source.suffix.lower() != '.obj':
        raise ValueError('入力形式はOBJです。')
    if output_dir.exists():
        raise FileExistsError(f'既存の出力先への上書きは禁止です: {output_dir}')
    if num_faces < 4 or max_texture_size < 1 or not math.isfinite(unit_scale) or unit_scale <= 0:
        raise ValueError('面数・テクスチャ寸法・倍率を確認してください。')
    started = time.monotonic()
    print(f'原本読込: {source}', flush=True)
    source_hash = file_hash(source)
    meshes = pymeshlab.MeshSet()
    meshes.load_new_mesh(str(source))
    mesh = meshes.current_mesh()
    source_bounds = mesh_bounds(mesh)
    if not np.isfinite(source_bounds).all() or mesh.face_number() == 0:
        raise ValueError('有限座標と面を持つメッシュが必要です。')
    source_faces = mesh.face_number()
    source_vertices = mesh.vertex_number()
    texture_paths = {name: (source.parent / name).resolve(strict=True) for name in mesh.textures()}
    source_files = [source, *source.parent.glob('*.mtl'), *texture_paths.values()]
    source_hashes = {str(path): file_hash(path) for path in source_files}
    if source_hashes[str(source)] != source_hash:
        raise RuntimeError('読込中に原本が変更されました。')

    # 面積比例の表面サンプルによる近似誤差の検証。原本全体の最大誤差ではない値
    num_samples = 20000
    meshes.generate_sampling_montecarlo(samplenum=num_samples)
    sample_id = meshes.current_mesh_id()
    meshes.set_current_mesh(0)
    print(f'軽量化: {source_faces:,} → 目標 {num_faces:,} 三角形', flush=True)
    if source_faces > num_faces:
        filter_name = ('meshing_decimation_quadric_edge_collapse_with_texture'
                       if mesh.has_wedge_tex_coord() else 'meshing_decimation_quadric_edge_collapse')
        meshes.apply_filter(filter_name, targetfacenum=num_faces, preserveboundary=True,
                            preservenormal=True, optimalplacement=True, planarquadric=True)
    mesh = meshes.current_mesh()
    mesh.compact()
    mesh.update_bounding_box()
    preview_bounds = mesh_bounds(mesh) * unit_scale
    reduced_faces = mesh.face_number()
    print(f'軽量化完了: {reduced_faces:,} 三角形、誤差確認', flush=True)
    error = meshes.get_hausdorff_distance(sampledmesh=sample_id, targetmesh=0,
                                         samplevert=True, sampleedge=False, sampleface=False,
                                         maxdist=pymeshlab.PercentageValue(100))
    error_m = {key: float(error[key]) * unit_scale for key in ('min', 'max', 'mean', 'RMS')}
    meshes.set_current_mesh(0)

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.mesh-preview-', dir=output_dir.parent) as temporary:
        temporary = Path(temporary)
        obj_path = temporary / 'reduced.obj'
        meshes.save_current_mesh(str(obj_path), save_textures=False)
        texture_info = []
        for name, path in texture_paths.items():
            target = (temporary / name).resolve()
            if not target.is_relative_to(temporary):
                raise ValueError('テクスチャの絶対パス・親ディレクトリ参照は未対応です。')
            target.parent.mkdir(parents=True, exist_ok=True)
            with Image.open(path) as original:
                resized = original.copy()
                resized.thumbnail((max_texture_size, max_texture_size), Image.Resampling.LANCZOS)
                resized.save(target)
                texture_info.append({'source': str(path), 'source_size': list(original.size),
                                     'preview_size': list(resized.size)})

        print('GLB生成: 材質・UV・画像の内包', flush=True)
        scene = trimesh.load_scene(obj_path, process=False)
        scene.apply_scale(unit_scale)
        for geometry in scene.geometry.values():
            if not np.isfinite(geometry.vertices).all():
                raise ValueError('軽量化後の非有限座標です。')
        num_textured_parts = sum(geometry.visual.kind == 'texture' and
                                 getattr(geometry.visual.material, 'image', None) is not None
                                 for geometry in scene.geometry.values())
        if texture_paths and not num_textured_parts:
            raise ValueError('テクスチャを引き継げませんでした。')
        scene.metadata['preview'] = {'source_sha256': source_hash, 'unit_scale': unit_scale,
                                     'axis_policy': 'preserve_source_axes', 'is_display_only': True}
        result = temporary / 'result'
        result.mkdir()
        glb_path = result / f'{source.stem}_preview.glb'
        glb_path.write_bytes(scene.export(file_type='glb'))
        if glb_path.stat().st_size > 256 * 1024 * 1024:
            raise ValueError('表示用GLBがViewerの256 MiB上限を超えています。')

        # GLB再読込による座標範囲・三角形数・テクスチャ保持の確認
        check = trimesh.load_scene(glb_path, process=False)
        num_exported_faces = sum(len(geometry.faces) for geometry in check.geometry.values())
        if num_exported_faces != reduced_faces:
            raise ValueError('GLBへの変換時に面数が変化しました。')
        if not np.allclose(check.bounds, preview_bounds, atol=1e-5):
            raise ValueError('GLBへの変換時に座標範囲が変化しました。')
        for path, expected in source_hashes.items():
            if file_hash(Path(path)) != expected:
                raise RuntimeError(f'変換中に原本が変更されました: {path}')
        matrix = np.diag([unit_scale, unit_scale, unit_scale, 1.0])
        manifest = {
            'source': str(source), 'source_sha256': source_hash,
            'source_files_sha256': source_hashes, 'source_size_bytes': source.stat().st_size,
            'source_num_vertices': source_vertices, 'source_num_faces': source_faces,
            'source_bounds': source_bounds.tolist(), 'target_num_faces': num_faces,
            'preview_file': glb_path.name, 'preview_sha256': file_hash(glb_path),
            'preview_size_bytes': glb_path.stat().st_size, 'preview_num_faces': num_exported_faces,
            'preview_num_vertices': sum(len(geometry.vertices) for geometry in check.geometry.values()),
            'preview_bounds': check.bounds.tolist(), 'source_to_preview_matrix': matrix.tolist(),
            'preview_to_source_matrix': np.linalg.inv(matrix).tolist(),
            'unit_scale': unit_scale, 'axis_policy': '原本の軸と原点を保持。回転・中心合わせなし。',
            'textures': texture_info, 'num_textured_parts': num_textured_parts,
            'source_surface_to_preview_error': {'num_samples': num_samples,
                'method': '原本表面の面積比例サンプルから軽量メッシュ表面への片方向距離',
                'unit': '明示倍率適用後の座標単位', 'distance': error_m,
                'limitation': '全表面の最大誤差・対称Hausdorff距離・登録精度の保証ではない値'},
            'elapsed_sec': time.monotonic() - started,
            'versions': {name: version(name) for name in ('pymeshlab', 'trimesh', 'numpy', 'pillow')},
            'modifications': '表示用の面数削減、テクスチャ縮小、明示倍率の適用。原本の変更なし。',
            'is_display_only': True,
        }
        if attribution_file:
            attribution_file = Path(attribution_file).resolve(strict=True)
            manifest['attribution'] = json.loads(attribution_file.read_text())
            shutil.copyfile(attribution_file, result / 'SOURCE.json')
        license_file = source.parent / 'license.txt'
        if license_file.is_file():
            shutil.copyfile(license_file, result / 'license.txt')
        (result / 'preview.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        result.rename(output_dir)
    print(json.dumps(manifest, ensure_ascii=False, indent=2), flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--num_faces', type=int, default=300000)
    parser.add_argument('--max_texture_size', type=int, default=4096)
    parser.add_argument('--unit_scale', type=float, required=True, help='原本座標から表示座標への明示倍率')
    parser.add_argument('--attribution_file', type=Path)
    args = parser.parse_args()
    prepare_preview(**vars(args))


if __name__ == '__main__':
    main()
