"""双腕 URDF の外装 collision 欠落と形状参照の回帰検証。"""

from functools import cache
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as element_tree

import numpy as np
import pytest


repo_root = Path(__file__).resolve().parents[2]
cover_names = (
    'waist_cover_link', 'L_shoulder_cover_link', 'R_shoulder_cover_link',
    'neck_tilt_cover_link', 'realsense_mount_link',
    'L_XM540_cover_link', 'L_link4_cover_link', 'L_link6_cover_link',
    'R_XM540_cover_link', 'R_link4_cover_link', 'R_link6_cover_link',
)


@pytest.fixture(params=('topo_dual_arm_max', 'topo_dual_arm_max_long'))
def robot_description(request):
    path = repo_root / 'urdf' / request.param / 'topo_dual_arm_max.urdf'
    return path, element_tree.parse(path).getroot()


def geometry_signature(element):
    """空白と属性順に依存しない形状定義の比較用表現。"""
    if element is None:
        return None
    return (element.tag, tuple(sorted(element.attrib.items())),
            tuple(geometry_signature(child) for child in element))


@cache
def read_mesh_num_triangles(path):
    """参照 STL 全三角形のサイズ・座標有限性の確認。"""
    data = path.read_bytes()
    assert len(data) >= 84, path
    num_triangles = struct.unpack_from('<I', data, 80)[0]
    assert num_triangles > 0, path
    assert len(data) == 84 + num_triangles * 50, path
    for triangle in struct.iter_unpack('<12fH', data[84:]):
        assert all(math.isfinite(value) for value in triangle[3:12]), path
    return num_triangles


def test_visual_links_have_collision(robot_description):
    _, robot = robot_description
    missing = [link.get('name') for link in robot.findall('link')
               if link.find('visual') is not None and link.find('collision') is None]
    assert missing == []


def test_cover_collision_matches_visual(robot_description):
    path, robot = robot_description
    names = cover_names
    if path.parent.name == 'topo_dual_arm_max_long':
        names += ('L_joint7_cover_link', 'R_joint7_cover_link')
    joints = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    for name in names:
        link = robot.find(f"link[@name='{name}']")
        assert link is not None, name
        visuals, collisions = link.findall('visual'), link.findall('collision')
        assert len(visuals) == len(collisions) == 1, name
        assert geometry_signature(visuals[0].find('origin')) == geometry_signature(collisions[0].find('origin')), name
        assert geometry_signature(visuals[0].find('geometry')) == geometry_signature(collisions[0].find('geometry')), name
        assert joints[name].get('type') == 'fixed', name


def test_collision_geometry_assets_parse(robot_description):
    path, robot = robot_description
    num_shapes = 0
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            assert geometry is not None and len(geometry) == 1, link.get('name')
            shape = geometry[0]
            if shape.tag == 'mesh':
                mesh_path = (path.parent / shape.get('filename')).resolve()
                assert mesh_path.is_relative_to(path.parent.resolve()), mesh_path
                scale = [float(value) for value in shape.get('scale', '1 1 1').split()]
                assert len(scale) == 3 and all(math.isfinite(value) and value > 0 for value in scale), link.get('name')
                assert read_mesh_num_triangles(mesh_path) > 0
            else:
                assert shape.tag == 'box', link.get('name')
                size = [float(value) for value in shape.get('size').split()]
                assert len(size) == 3 and all(math.isfinite(value) and value > 0 for value in size), link.get('name')
            origin = collision.find('origin')
            if origin is not None:
                for field in ('xyz', 'rpy'):
                    values = [float(value) for value in origin.get(field, '0 0 0').split()]
                    assert len(values) == 3 and all(math.isfinite(value) for value in values), link.get('name')
            num_shapes += 1
    assert num_shapes > 0


def origin_matrix(element):
    """URDF origin の回転・並進行列。"""
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
    roll_cos, pitch_cos, yaw_cos = map(math.cos, (roll, pitch, yaw))
    roll_sin, pitch_sin, yaw_sin = map(math.sin, (roll, pitch, yaw))
    result[:3, :3] = [
        [yaw_cos * pitch_cos, yaw_cos * pitch_sin * roll_sin - yaw_sin * roll_cos,
         yaw_cos * pitch_sin * roll_cos + yaw_sin * roll_sin],
        [yaw_sin * pitch_cos, yaw_sin * pitch_sin * roll_sin + yaw_cos * roll_cos,
         yaw_sin * pitch_sin * roll_cos - yaw_cos * roll_sin],
        [-pitch_sin, pitch_cos * roll_sin, pitch_cos * roll_cos],
    ]
    return result


def test_camera_box_contains_all_visual_vertices(robot_description):
    path, robot = robot_description
    camera = robot.find("link[@name='camera_link']")
    visual, collision = camera.find('visual'), camera.find('collision')
    mesh, box = visual.find('geometry/mesh'), collision.find('geometry/box')
    assert mesh is not None and box is not None
    mesh_path = path.parent / mesh.get('filename')
    num_triangles = read_mesh_num_triangles(mesh_path)
    data_type = np.dtype([('normal', '<f4', (3,)),
                          ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.frombuffer(mesh_path.read_bytes(), dtype=data_type, count=num_triangles,
                             offset=84)['vertices'].reshape(-1, 3).astype(float)
    vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
    visual_origin = origin_matrix(visual.find('origin'))
    box_origin = origin_matrix(collision.find('origin'))
    points_in_link = vertices @ visual_origin[:3, :3].T + visual_origin[:3, 3]
    points_in_box = (points_in_link - box_origin[:3, 3]) @ box_origin[:3, :3]
    size = np.fromstring(box.get('size'), sep=' ')
    assert np.all(np.abs(points_in_box) <= size * 0.5 + 1e-12)
    assert np.allclose(points_in_box.max(axis=0) - points_in_box.min(axis=0), size,
                       rtol=0, atol=1e-12)
