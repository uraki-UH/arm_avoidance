"""Hesai公式JT128 STEPから表示用STLへの変換。原本の保持・描画用の軽量化。"""
import hashlib
import json
from pathlib import Path
import struct
import tempfile

import fast_simplification
import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer


asset_dir = Path(__file__).resolve().parents[1] / 'app/assets/jt128'
source = asset_dir / 'jt128-side-connector.stp'
source_sha256 = 'ba95712cb637f912fae8360fc90862f7718ef3fe8c613865e5cd779e030acef8'
mesh_dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


def main():
    if hashlib.sha256(source.read_bytes()).hexdigest() != source_sha256:
        raise ValueError('公式原本の変更。部品構成の再確認が必要')
    reader = STEPControl_Reader()
    reader.ReadFile(str(source))
    reader.TransferRoots()
    explorer = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    # 部品別の描画色・面数。CADの内部面色ではなく製品外観用
    settings = [
        ('top_disc', '#21252a', .15, .3, 400),
        ('optical_cap', '#282c31', .1, .25, 4000),
        ('cap_seal', '#202327', .1, .3, 500),
        ('optical_window', '#25292e', .15, .28, 4000),
        None,
        ('base_housing', '#42464a', .55, .38, 10000),
        ('connector_mount', '#4b4f53', .55, .35, 1000),
        ('connector_seal', '#25282b', .1, .45, 800),
        ('side_connector', '#aeb3b7', .8, .27, 4000),
    ]
    parts = []
    excluded_parts = []
    with tempfile.TemporaryDirectory(prefix='jt128-mesh-') as directory:
        part_idx = 0
        while explorer.More():
            shape = explorer.Current()
            setting = settings[part_idx]
            if setting is None:
                # 原本内の外形外ソリッド。幅485 mm・高さ246 mm、公式外形との不整合
                excluded_parts.append({'source_part_idx': part_idx, 'reason': '公称外形外のソリッド。原本は改変せず保持'})
            else:
                name, color, metalness, roughness, num_target_triangles = setting
                BRepMesh_IncrementalMesh(shape, .06, False, .16, True).Perform()
                raw = Path(directory) / 'raw.stl'
                if not writer.Write(shape, str(raw)):
                    raise ValueError('STL出力失敗: ' + name)
                data = raw.read_bytes()
                faces = np.frombuffer(data, mesh_dtype, offset=84).copy()
                num_source_triangles = len(faces)
                if len(faces) > num_target_triangles:
                    source_vertices = faces['vertices'].reshape(-1, 3)
                    min_bounds, max_bounds = source_vertices.min(axis=0), source_vertices.max(axis=0)
                    # STL境界の丸め差の接合。0.0001 mmの頂点座標精度
                    points, idx = np.unique(np.round(source_vertices, 4), axis=0, return_inverse=True)
                    points, triangles = fast_simplification.simplify(points, idx.reshape(-1, 3).astype(np.int32),
                        target_count=num_target_triangles, preserve_border=True, agg=4)
                    faces = np.zeros(len(triangles), mesh_dtype)
                    # 簡略化後の外形外への頂点移動の抑制
                    faces['vertices'] = np.clip(points[triangles], min_bounds, max_bounds)
                    normals = np.cross(faces['vertices'][:, 1] - faces['vertices'][:, 0],
                                       faces['vertices'][:, 2] - faces['vertices'][:, 0])
                    lengths = np.linalg.norm(normals, axis=1)
                    faces['normal'] = normals / np.maximum(lengths[:, None], 1e-20)
                vertices = faces['vertices'].reshape(-1, 3)
                bounds = np.concatenate([vertices.min(axis=0), vertices.max(axis=0)]).tolist()
                if np.max(np.abs(vertices[:, :2])) > 40 or vertices[:, 2].min() < -.01 or vertices[:, 2].max() > 73.1:
                    raise ValueError('表示部品の外形不整合: ' + name)
                filename = name + '.stl'
                with (asset_dir / filename).open('wb') as stream:
                    stream.write(b'Hesai JT128 official CAD, display mesh, millimetres'.ljust(80, b'\0'))
                    stream.write(struct.pack('<I', len(faces)))
                    stream.write(faces.tobytes())
                parts.append({'file': filename, 'name': name, 'source_part_idx': part_idx,
                              'color': color, 'metalness': metalness, 'roughness': roughness,
                              'num_source_triangles': num_source_triangles, 'num_triangles': len(faces),
                              'bounds_mm': bounds})
            explorer.Next()
            part_idx += 1
        if part_idx != len(settings):
            raise ValueError('部品数の変更。再確認が必要')
    metadata = {
        'source': 'https://www.hesaitech.com/wp-content/uploads/2025/09/JT128-3D-Model.zip',
        'source_sha256': source_sha256, 'variant': 'side_connector', 'unit': 'mm',
        'chord_tolerance_mm': .06, 'angular_tolerance_rad': .16, 'vertex_merge_dist_th_mm': .0001,
        'lidar_origin_height_mm': 47.84, 'cad_to_sensor_rot_z_deg': -90,
        'origin_source': 'JT128 J01-en-260330 Figure 5・6',
        'parts': parts, 'excluded_parts': excluded_parts,
    }
    (asset_dir / 'model.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'num_parts': len(parts), 'num_triangles': sum(part['num_triangles'] for part in parts)}))


if __name__ == '__main__':
    main()
