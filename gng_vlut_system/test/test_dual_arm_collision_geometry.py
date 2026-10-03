"""双腕URDFの衝突形状・外接球の包囲・自己干渉・退避余裕の検証。"""
from functools import cache
from itertools import product
from pathlib import Path
import math
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as element_tree

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from dual_arm_avoidance_geometry import build_collision_pairs, compile_link_operations, is_nearby_link_pair, mesh_vertices, read_joint_geometry, read_link_spheres, robot_geometry, shape_vertices


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


def test_topodualarm_collision_geometry():
    """標準ToPoDualArmの衝突形状欠落とカメラ包囲の確認。"""
    path = repo_root / 'urdf/dual_arm_urdf/dual_arm_robot.urdf'
    description = path, element_tree.parse(path).getroot()
    test_visual_links_have_collision(description)
    test_collision_geometry_assets_parse(description)
    test_camera_box_contains_all_visual_vertices(description)


class geometry_test(unittest.TestCase):
    def test_primitive_vertices_and_invalid_dimensions(self):
        """基本形状の外接箱と不正寸法の拒否。"""
        corners = np.asarray(list(product((-1, 1), repeat=3)))
        for primitive, half in (
                ('<box size="0.04 0.08 0.12"/>', [.02, .04, .06]),
                ('<sphere radius="0.03"/>', [.03, .03, .03]),
                ('<cylinder radius="0.02" length="0.10"/>', [.02, .02, .05])):
            with self.subTest(primitive=primitive):
                shape = element_tree.fromstring('<collision><geometry>'+primitive+'</geometry></collision>')
                np.testing.assert_array_equal(shape_vertices(shape, Path('.'), 'body'), corners*half)
        for primitive, message in (
                ('<sphere radius="0"/>', '球寸法'),
                ('<sphere radius="nan"/>', '球寸法'),
                ('<cylinder radius="-1" length="1"/>', '円柱寸法'),
                ('<cylinder radius="1" length="inf"/>', '円柱寸法'),
                ('<capsule radius="1" length="1"/>', '未対応の形状')):
            with self.subTest(primitive=primitive):
                shape = element_tree.fromstring('<collision><geometry>'+primitive+'</geometry></collision>')
                with self.assertRaisesRegex(ValueError, message):
                    shape_vertices(shape, Path('.'), 'body')

    def test_collision_priority_and_visual_fallback(self):
        """衝突形状優先と外装代用のリンク順・球列座標。"""
        root = element_tree.fromstring('''<robot name="shapes">
          <link name="body"><visual><geometry><box size="1 1 1"/></geometry></visual>
            <collision><geometry><box size="0.07 0.02 0.02"/></geometry></collision></link>
          <link name="cover"><visual><origin xyz="0 0 0.5"/>
            <geometry><box size="0.01 0.01 0.01"/></geometry></visual></link>
          </robot>''')
        spheres = read_link_spheres(root, Path('.'))
        self.assertEqual([name for name, _, _ in spheres], ['body', 'body', 'cover'])
        np.testing.assert_allclose([point for _, point, _ in spheres],
                                   [[-.0175, 0, 0], [.0175, 0, 0], [0, 0, .5]], atol=1e-15)
        np.testing.assert_allclose([radius for _, _, radius in spheres],
                                   [np.sqrt(.00050625)]*2+[np.sqrt(.000075)], atol=1e-15)

    def test_reverse_joint_order_and_mimic_transforms(self):
        """記載順と接続順の分離、連続回転・直動・mimic係数の維持。"""
        text = '''<robot name="ordered"><link name="base"/><link name="L_arm"/>
          <link name="R_arm"/><link name="finger"/>
          <link name="tip"><collision><geometry><sphere radius="0.01"/></geometry></collision></link>
          <joint name="tip_fixed" type="fixed"><parent link="finger"/><child link="tip"/>
            <origin xyz="1 0 0"/></joint>
          <joint name="finger_mimic" type="revolute"><parent link="L_arm"/><child link="finger"/>
            <axis xyz="0 0 1"/><mimic joint="L_joint1" multiplier="-0.5" offset="0.2"/></joint>
          <joint name="R_joint1" type="prismatic"><parent link="base"/><child link="R_arm"/>
            <axis xyz="0 0 2"/><limit lower="0" upper="0.4"/></joint>
          <joint name="L_joint1" type="continuous"><parent link="base"/><child link="L_arm"/>
            <axis xyz="0 0 1"/><origin xyz="0 1 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'ordered.urdf'
            path.write_text(text)
            geometry = robot_geometry(path)
        self.assertEqual(geometry.joint_names, ['R_joint1', 'L_joint1'])
        self.assertEqual(geometry.link_indices, {'base': 0, 'R_arm': 1, 'L_arm': 2, 'finger': 3, 'tip': 4})
        np.testing.assert_array_equal(geometry.limits, [[0, .4], [-np.pi, np.pi]])
        positions = np.array([.3, .6])
        transforms = geometry.link_transforms(positions)
        np.testing.assert_allclose(transforms[1, :3, 3], [0, 0, .3], atol=1e-15)
        np.testing.assert_allclose(transforms[2, :3, :3],
                                   [[np.cos(.6), -np.sin(.6), 0],
                                    [np.sin(.6), np.cos(.6), 0], [0, 0, 1]], atol=1e-15)
        np.testing.assert_allclose(transforms[4, :3, 3], [np.cos(.5), 1+np.sin(.5), 0], atol=1e-15)
        np.testing.assert_allclose(geometry.centers(positions), [transforms[4, :3, 3]], atol=1e-15)

    def test_invalid_link_connections_rejected(self):
        """非一意ルート・未接続循環・未知mimic参照の拒否。"""
        for content, message in (
                ('<link name="a"/><link name="b"/>', 'ルートリンク'),
                ('''<link name="root"/><link name="a"/><link name="b"/>
                    <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
                    <joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint>''', '接続'),
                ('''<link name="a"/><link name="b"/>
                    <joint name="ab" type="revolute"><parent link="a"/><child link="b"/>
                      <mimic joint="unknown"/></joint>''', 'unknown')):
            with self.subTest(content=content):
                root = element_tree.fromstring('<robot name="invalid">'+content+'</robot>')
                joints, names, _, parents = read_joint_geometry(root)
                with self.assertRaisesRegex(ValueError, message):
                    compile_link_operations(root, joints, names, parents)

    def test_collision_pair_order_and_group_selection(self):
        """球順・初期重複除外・腕グループ指定による監視対象の維持。"""
        centers = np.array([[0, 0, 1], [.1, 0, 1], [.2, 0, 1], [.205, 0, 1]])
        names = ['L_a', 'R_b', 'L_c', 'body']
        radii = np.full(4, .01)
        spheres = list(zip(names, centers, radii))
        is_arm = np.array([True, True, True, False])
        pairs, inter_arm_pairs = build_collision_pairs(spheres, centers, radii, is_arm, {}, None)
        np.testing.assert_array_equal(pairs, [[1, 0], [2, 0], [2, 1], [3, 0], [3, 1]])
        np.testing.assert_array_equal(inter_arm_pairs, [[1, 0], [2, 1]])
        link_groups = {'L_a': 'first', 'R_b': 'first', 'L_c': 'second'}
        grouped_pairs, grouped_inter_arm_pairs = build_collision_pairs(
            spheres, centers, radii, is_arm, {}, link_groups)
        np.testing.assert_array_equal(grouped_pairs, pairs)
        np.testing.assert_array_equal(grouped_inter_arm_pairs, [[2, 0], [2, 1]])
        parents = {'L_a': 'root', 'R_b': 'L_a', 'L_c': 'R_b', 'body': 'L_c'}
        self.assertTrue(is_nearby_link_pair('body', 'L_a', parents))
        empty_pairs, empty_inter_arm_pairs = build_collision_pairs(
            spheres, centers, radii, is_arm, parents, None)
        self.assertEqual(empty_pairs.shape, (0, 2))
        self.assertEqual(empty_inter_arm_pairs.shape, (0, 2))

    def test_rotated_box_enclosure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'box.urdf'
            path.write_text('''<robot name="box"><link name="base_footprint"/>
              <link name="camera_link"><collision><origin xyz="0.1 0.2 0.3" rpy="0 0 0.7"/>
                <geometry><box size="0.04 0.09 0.025"/></geometry></collision></link>
              <joint name="camera_fixed" type="fixed"><parent link="base_footprint"/>
                <child link="camera_link"/></joint></robot>''')
            geometry = robot_geometry(path)
            points = np.asarray(list(product(np.linspace(-1, 1, 11), repeat=3))) * [.02, .045, .0125]
            matrix = np.array([[np.cos(.7), -np.sin(.7), 0], [np.sin(.7), np.cos(.7), 0], [0, 0, 1]])
            points = points @ matrix.T + [.1, .2, .3]
            gaps = np.asarray([np.linalg.norm(points-point, axis=1)-radius
                               for _, point, radius in geometry.spheres])
            self.assertLessEqual(float(gaps.min(axis=0).max()), 1e-10)

    def test_invalid_box_dimensions_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'box.urdf'
            for size in ('0 1 1', '-1 1 1', 'nan 1 1', 'inf 1 1', '1 1', '1 1 1 1'):
                with self.subTest(size=size):
                    path.write_text('<robot name="box"><link name="base_footprint"><collision>'
                                    '<geometry><box size="'+size+'"/></geometry></collision></link></robot>')
                    with self.assertRaisesRegex(ValueError, '直方体寸法'):
                        robot_geometry(path)

    def test_mesh_enclosure_and_retreat(self):
        root = Path(__file__).resolve().parents[2]
        for model in ('topo_dual_arm_max', 'topo_dual_arm_max_long'):
            with self.subTest(model=model):
                path = root/'urdf'/model/'topo_dual_arm_max.urdf'
                geometry = robot_geometry(path)
                home = np.zeros(len(geometry.joint_names))
                self.assertEqual(len(geometry.joint_names), 19)
                self.assertTrue(geometry.has_internal_clearance(geometry.centers(home)))
                # ローカル座標の衝突メッシュを覆う球列の確認
                vertices = mesh_vertices(path.parent/'meshes/L_link4.stl')*.001
                spheres = [(point, radius) for name, point, radius in geometry.spheres if name == 'L_link4']
                gaps = np.array([np.linalg.norm(vertices-point, axis=1)-radius for point, radius in spheres])
                self.assertLessEqual(float(gaps.min(axis=0).max()), 1e-8)
                for sign in (1, -1):
                    positions = home.copy()
                    min_gap = float('inf')
                    for hand_x in np.linspace(.45, .03, 161):
                        hand = np.array([hand_x, sign*.36, .38])
                        elbow = hand+np.array([.35, 0, 0])
                        previous = positions.copy()
                        positions, has_candidate = geometry.choose_step(positions, home, hand, elbow, .045, .12, .035)
                        self.assertTrue(has_candidate)
                        self.assertLessEqual(float(np.max(np.abs(positions-previous))), .035+1e-10)
                        gap, centers, _, _ = geometry.clearance(positions, hand, elbow, .045)
                        self.assertTrue(geometry.has_internal_clearance(centers))
                        min_gap = min(min_gap, gap)
                    self.assertGreater(min_gap, .07)
                    self.assertLess(geometry.clearance(home, hand, elbow, .045)[0], 0)
                    self.assertGreater(float(np.max(np.abs(positions))), .2)
