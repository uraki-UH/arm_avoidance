"""STEP球面の再パラメータ化による胸部LiDARカバーの再生成。OCP・NumPyが必要。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

mesh_dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
default_metadata = Path(__file__).resolve().parents[1] / 'urdf/topo_dual_arm_max_long/meshes/chest_lidar.json'


def read_mesh(path):
    data = path.read_bytes()
    num = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * num:
        raise ValueError(f'STLサイズ不整合: {path}')
    return np.frombuffer(data, mesh_dtype, num, 84).copy()


def write_mesh(path, faces):
    with path.open('xb') as stream:
        stream.write(b'ToPo chest LiDAR, STEP tessellation, millimetres'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(faces)))
        stream.write(faces.tobytes())
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_blue_faces(source):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf, XCAFDoc_ColorGen
    from OCP.Quantity import Quantity_ColorRGBA
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS_Iterator, TopoDS

    document = TDocStd_Document(TCollection_ExtendedString('cad'))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.ReadFile(str(source))
    if not reader.Transfer(document):
        raise ValueError('STEP読込失敗')
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(document.Main())

    def named_child(parent, expected):
        children = TDF_LabelSequence()
        shapes.GetComponents_s(parent, children)
        for idx in range(1, children.Length() + 1):
            instance = children.Value(idx)
            actual = instance
            if shapes.IsReference_s(instance):
                actual = TDF_Label()
                shapes.GetReferredShape_s(instance, actual)
            name = TDataStd_Name()
            if actual.FindAttribute(TDataStd_Name.GetID_s(), name) and name.Get().ToExtString() == expected:
                return actual
        raise ValueError(f'STEP部品なし: {expected}')

    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    sensor = named_child(roots.Value(1), 'Mid-360')
    part = named_child(sensor, 'MID-360_4_1')
    faces = []

    def visit(shape, inherited=None):
        color = inherited
        value = Quantity_ColorRGBA()
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if colors.GetColor(shape, kind, value):
                rgb = value.GetRGB()
                color = (rgb.Red(), rgb.Green(), rgb.Blue())
                break
        if shape.ShapeType() == TopAbs_FACE:
            if color and np.allclose(color, [0.005605392, 0.000303527, 0.309468925], atol=1e-8):
                faces.append(TopoDS.Face_s(shape))
            return
        children = TopoDS_Iterator(shape)
        while children.More():
            visit(children.Value(), color)
            children.Next()

    visit(shapes.GetShape_s(part))
    if len(faces) != 4:
        raise ValueError('青色面の構成変更。再確認が必要')
    return faces


def repair_cover(faces, metadata, output):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.BRep import BRep_Tool
    from OCP.BRepTools import BRepTools
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GeomAbs import GeomAbs_Sphere
    from OCP.TopAbs import TopAbs_VERTEX
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.StlAPI import StlAPI_Writer
    from OCP.gp import gp_Sphere, gp_Ax3, gp_Dir

    spheres = [face for face in faces if BRepAdaptor_Surface(face).GetType() == GeomAbs_Sphere]
    if len(spheres) != 1:
        raise ValueError('球面の構成変更。再確認が必要')
    original = spheres[0]
    sphere = BRepAdaptor_Surface(original).Sphere()
    rotation = np.array(metadata['sensor_cad_to_link_rotation'])
    center = rotation @ np.array(sphere.Location().Coord())
    vertices = []
    explorer = TopExp_Explorer(original, TopAbs_VERTEX)
    while explorer.More():
        vertices.append(rotation @ np.array(BRep_Tool.Pnt_s(TopoDS.Vertex_s(explorer.Current())).Coord()))
        explorer.Next()
    z = np.array(vertices)[:, 2]
    merge_tolerance_mm = 0.0001
    # 円形の上下境界を持つ球面帯のみ対象。別形状への推測補修は禁止
    min_z = float(np.median(z[np.abs(z - z.min()) < merge_tolerance_mm]))
    max_z = float(np.median(z[np.abs(z - z.max()) < merge_tolerance_mm]))
    if not np.all(np.minimum(np.abs(z - min_z), np.abs(z - max_z)) < merge_tolerance_mm):
        raise ValueError('球面帯以外の境界')
    radius = sphere.Radius()
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(original, props)
    expected_area = 2 * math.pi * radius * (max_z - min_z)
    if abs(props.Mass() - expected_area) > 0.01:
        raise ValueError('元STEPと再パラメータ化球面の面積不一致')
    axis = rotation.T @ np.array([0, 0, 1])
    aligned = gp_Sphere(gp_Ax3(sphere.Location(), gp_Dir(*axis)), radius)
    replacement = BRepBuilderAPI_MakeFace(aligned, 0, 2 * math.pi,
        math.asin((min_z - center[2]) / radius), math.asin((max_z - center[2]) / radius)).Face()
    sewing = BRepBuilderAPI_Sewing(merge_tolerance_mm)
    for face in faces:
        sewing.Add(replacement if face.IsSame(original) else face)
    sewing.Perform()
    if sewing.NbFreeEdges() != 1:
        raise ValueError('底部の筐体接続辺以外に未接合辺')
    shape = sewing.SewedShape()
    BRepTools.Clean_s(shape)
    mesher = BRepMesh_IncrementalMesh(shape, metadata['chord_tolerance_mm'], False,
                                    metadata['angular_tolerance_rad'], False)
    if mesher.GetStatusFlags():
        raise ValueError(f'三角形化失敗: {mesher.GetStatusFlags()}')
    raw = output / 'cover_cad_frame.stl'
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, str(raw)):
        raise ValueError('STL出力失敗')
    triangles = read_mesh(raw)
    raw.unlink()
    triangles['vertices'] = triangles['vertices'] @ rotation.T
    triangles['normal'] = triangles['normal'] @ rotation.T
    metadata['sensor_cover_repair'] = {
        'method': '元球面の半径・中心・上下境界を保持した軸合わせと面接合',
        'source_part': 'MID-360_4_1', 'radius_mm': radius,
        'center_mm': center.tolist(), 'min_z_mm': min_z, 'max_z_mm': max_z,
        'merge_tolerance_mm': merge_tolerance_mm,
        'generator': 'scripts/rebuild_chest_lidar_cover.py',
    }
    return triangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata', type=Path, default=default_metadata)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    metadata = json.loads(args.metadata.read_text())
    package = args.metadata.resolve().parents[1]
    preset = metadata['variants']['45']
    source = package.parent / preset['source']
    if hashlib.sha256(source.read_bytes()).hexdigest() != preset['source_sha256']:
        parser.error('STEP原本のハッシュ不一致')
    # 入出力の分離。既存ファイルの上書きなし
    args.output_dir.mkdir(parents=True, exist_ok=False)
    cover = repair_cover(load_blue_faces(source), metadata, args.output_dir)
    blue = metadata['sensor_visuals'][0]
    blue['sha256'] = write_mesh(args.output_dir / Path(blue['file']).name, cover)
    blue['triangles'] = len(cover)
    combined = np.concatenate([cover, *[read_mesh(package / item['file']) for item in metadata['sensor_visuals'][1:]]])
    merged = metadata['sensor_mesh']
    merged['sha256'] = write_mesh(args.output_dir / Path(merged['file']).name, combined)
    merged['triangles'] = len(combined)
    (args.output_dir / 'chest_lidar.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'blue': blue, 'merged': merged, 'repair': metadata['sensor_cover_repair']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
