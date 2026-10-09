"""胸部LiDARのCAD座標・切替・メッシュ参照の回帰確認。"""
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import tempfile
import unittest
import xml.etree.ElementTree as et

import numpy as np

from chest_lidar_urdf import build_variant, default_urdf, main


class chest_lidar_tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = default_urdf
        cls.metadata = json.loads((cls.source.parent / 'meshes/chest_lidar.json').read_text())

    def test_default_and_link_tree(self):
        root = et.parse(self.source).getroot()
        links = {link.get('name') for link in root.findall('link')}
        children = [joint.find('child').get('link') for joint in root.findall('joint')]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(links - set(children), {'base_footprint'})
        for joint in root.findall('joint'):
            self.assertIn(joint.find('parent').get('link'), links)
        mesh = root.find("./link[@name='chest_lidar_mount_link']/visual/geometry/mesh")
        self.assertEqual(mesh.get('filename'), self.metadata['variants']['45']['mount_mesh']['file'])
        for name in ('chest_lidar_mount_fixed', 'chest_lidar_fixed'):
            self.assertEqual(root.find(f"./joint[@name='{name}']").get('type'), 'fixed')
            expected = et.fromstring(build_variant(self.source, 45)).find(f"./joint[@name='{name}']/origin")
            actual = root.find(f"./joint[@name='{name}']/origin")
            for key in ('xyz', 'rpy'):
                np.testing.assert_allclose(np.fromstring(actual.get(key), sep=' '),
                                           np.fromstring(expected.get(key), sep=' '), atol=1e-11)

    def test_variants_preserve_other_elements(self):
        before = self.source.read_bytes()
        root = et.fromstring(before)
        can_change = {'chest_lidar_mount_fixed', 'chest_lidar_fixed', 'chest_lidar_mount_link'}
        for angle in (45, 60):
            variant = et.fromstring(build_variant(self.source, angle))
            # 相対パスから絶対URIへの変換を除いた既存定義の比較
            for old, new in zip(root.findall('.//mesh'), variant.findall('.//mesh')):
                new.set('filename', old.get('filename'))
            self.assertEqual(len(root), len(variant))
            for old, new in zip(root, variant):
                if old.get('name') not in can_change:
                    self.assertEqual(et.tostring(old), et.tostring(new))
        self.assertEqual(self.source.read_bytes(), before)

    def test_cad_transforms(self):
        rotation = np.array(self.metadata['sensor_cad_to_link_rotation'])
        offset = np.array(self.metadata['cad_to_torso_translation_mm'])
        for angle in (45, 60):
            preset = self.metadata['variants'][str(angle)]
            root = et.fromstring(build_variant(self.source, angle))
            mount = root.find("./joint[@name='chest_lidar_mount_fixed']/origin")
            sensor = root.find("./joint[@name='chest_lidar_fixed']/origin")
            mount_xyz = np.fromstring(mount.get('xyz'), sep=' ')
            sensor_xyz = np.fromstring(sensor.get('xyz'), sep=' ')
            cad_mount = np.array(preset['cad_mount_matrix_mm'])
            cad_sensor = np.array(preset['cad_sensor_matrix_mm'])
            np.testing.assert_allclose(mount_xyz * 1000, cad_mount[:, 3] + offset, atol=1e-8)
            np.testing.assert_allclose((mount_xyz + sensor_xyz) * 1000, cad_sensor[:, 3] + offset, atol=1e-8)
            roll, pitch, yaw = np.fromstring(sensor.get('rpy'), sep=' ')
            self.assertEqual((roll, yaw), (0, 0))
            actual = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
            np.testing.assert_allclose(actual @ rotation, cad_sensor[:, :3], atol=1e-10)

    def test_mesh_files_units_and_collision_enclosure(self):
        for angle in (45, 60):
            root = et.fromstring(build_variant(self.source, angle))
            for mesh in root.findall('.//mesh'):
                self.assertTrue(Path(mesh.get('filename').removeprefix('file://')).is_file())
            for name in ('chest_lidar_mount_link', 'chest_lidar_link'):
                for mesh in root.findall(f"./link[@name='{name}']//mesh"):
                    self.assertEqual(mesh.get('scale'), '0.001 0.001 0.001')
            bracket = root.find("./link[@name='chest_lidar_mount_link']/visual/geometry/mesh")
            self.assertTrue(bracket.get('filename').endswith(f'chest_lidar_mount_{angle}.stl'))
        data = (self.source.parent / self.metadata['sensor_mesh']['file']).read_bytes()
        num_faces = struct.unpack_from('<I', data, 80)[0]
        self.assertEqual(len(data), 84 + 50 * num_faces)
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        vertices = np.frombuffer(data, dtype, num_faces, 84)['vertices'].reshape(-1, 3) * .001
        root = et.parse(self.source).getroot()
        collision = root.find("./link[@name='chest_lidar_link']/collision")
        center = np.fromstring(collision.find('origin').get('xyz'), sep=' ')
        size = np.fromstring(collision.find('geometry/box').get('size'), sep=' ')
        self.assertTrue(np.isfinite(vertices).all())
        self.assertTrue(np.all(vertices.min(0) >= center - size / 2))
        self.assertTrue(np.all(vertices.max(0) <= center + size / 2))

    def test_source_and_asset_hashes(self):
        items = [self.metadata['sensor_mesh'], *self.metadata['sensor_visuals']]
        for preset in self.metadata['variants'].values():
            source = self.source.parent.parent / preset['source']
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), preset['source_sha256'])
            items.append(preset['mount_mesh'])
        for item in items:
            path = self.source.parent / item['file']
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item['sha256'])

    def test_visual_colors_for_both_angles(self):
        expected = {item['file']: item for item in self.metadata['sensor_visuals']}
        for angle in (45, 60):
            root = et.fromstring(build_variant(self.source, angle))
            visuals = root.findall("./link[@name='chest_lidar_link']/visual")
            self.assertEqual(len(visuals), 4)
            for visual in visuals:
                mesh = visual.find('geometry/mesh')
                item = expected['meshes/' + Path(mesh.get('filename')).name]
                color = visual.find('material/color')
                np.testing.assert_array_equal(np.fromstring(color.get('rgba'), sep=' '), item['rgba'])
                self.assertEqual(visual.find('origin').get('xyz'), '0 0 0')
                self.assertEqual(visual.find('origin').get('rpy'), '0 0 0')

    def test_visual_partition_preserves_geometry(self):
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])

        def triangles(item):
            data = (self.source.parent / item['file']).read_bytes()
            num_faces = struct.unpack_from('<I', data, 80)[0]
            self.assertEqual(num_faces, item['triangles'])
            self.assertEqual(len(data), 84 + 50 * num_faces)
            return np.frombuffer(data, dtype, num_faces, 84)['vertices']

        def ordered(vertices):
            # 色グループ分割による面順序の相違を除いた0.00001 mm単位の照合
            rows = np.round(vertices, 5).reshape(-1, 9)
            return rows[np.lexsort(rows.T[::-1])]

        original = triangles(self.metadata['sensor_mesh'])
        colored = np.concatenate([triangles(item) for item in self.metadata['sensor_visuals']])
        np.testing.assert_array_equal(ordered(original), ordered(colored))

    def test_color_space_matches_step(self):
        source = self.source.parent.parent / self.metadata['variants']['45']['source']
        text = source.read_text()
        colors = []
        for match in re.finditer(r"COLOUR_RGB\('.*?',\s*([0-9.Ee+\-]+),\s*([0-9.Ee+\-]+),\s*([0-9.Ee+\-]+)\)", text, re.S):
            colors.append([float(value) for value in match.groups()])
        for item in self.metadata['sensor_visuals']:
            source_rgb = np.array(item['source_rgb_u8']) / 255
            self.assertTrue(any(np.allclose(source_rgb, color, atol=1e-10) for color in colors))
            linear_rgb = np.where(source_rgb <= .04045, source_rgb / 12.92,
                                  ((source_rgb + .055) / 1.055) ** 2.4)
            np.testing.assert_allclose(item['rgba'][:3], linear_rgb, atol=1e-7)

    def test_blue_cover_has_no_angular_gaps(self):
        # 青色球面の全周交差。頂点・辺の一致を避けた方位角と複数断面
        path = self.source.parent / self.metadata['sensor_visuals'][0]['file']
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        triangles = np.frombuffer(path.read_bytes(), dtype, offset=84)['vertices'].astype(float)
        edge_1, edge_2 = triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
        for z_mm in (18, 24, 30):
            origin = np.array([0, 0, z_mm])
            for deg in np.arange(0.37, 360, 10):
                direction = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0])
                cross = np.cross(direction, edge_2)
                determinant = np.einsum('ij,ij->i', edge_1, cross)
                mask = np.abs(determinant) > 1e-10
                inverse = 1 / determinant[mask]
                delta = origin - triangles[mask, 0]
                u = np.einsum('ij,ij->i', delta, cross[mask]) * inverse
                q = np.cross(delta, edge_1[mask])
                v = q @ direction * inverse
                dist = np.einsum('ij,ij->i', edge_2[mask], q) * inverse
                hits = (u >= 0) & (v >= 0) & (u + v <= 1) & (dist > 0)
                self.assertEqual(int(hits.sum()), 1, f'青色曲面の欠損または重複: Z={z_mm} mm, 方位={deg} deg')
                repair = self.metadata['sensor_cover_repair']
                expected = math.sqrt(repair['radius_mm'] ** 2 - (z_mm - repair['center_mm'][2]) ** 2)
                self.assertLessEqual(abs(float(dist[hits][0]) - expected), self.metadata['chord_tolerance_mm'])

    def test_blue_cover_boundary_only_at_base(self):
        # 青色面間の接合。筐体側の下端以外は全辺を2三角形で共有
        path = self.source.parent / self.metadata['sensor_visuals'][0]['file']
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        triangles = np.frombuffer(path.read_bytes(), dtype, offset=84)['vertices']
        vertices, inverse = np.unique(np.round(triangles.reshape(-1, 3), 4), axis=0, return_inverse=True)
        indices = inverse.reshape(-1, 3)
        edges = np.sort(np.concatenate([indices[:, [0, 1]], indices[:, [1, 2]], indices[:, [2, 0]]]), axis=1)
        unique, counts = np.unique(edges, axis=0, return_counts=True)
        self.assertTrue(np.all((counts == 1) | (counts == 2)))
        boundary = vertices[unique[counts == 1]]
        self.assertGreater(len(boundary), 0)
        np.testing.assert_allclose(boundary[:, :, 2], triangles[:, :, 2].min(), atol=.0001)

    def test_simulator_cover_matches_urdf(self):
        # 単独Simulatorの同梱STL・描画キャッシュとROS用形状の一致
        app = self.source.parents[2] / 'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/app'
        manifest = json.loads((app / 'assets.json').read_text())
        item = self.metadata['sensor_visuals'][0]
        data = (app / item['file']).read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(), item['sha256'])
        info = manifest['meshes'][item['file']]
        self.assertEqual(info['source_sha256'], item['sha256'])
        cache = (app / info['file']).read_bytes()
        num_vertices, num_indices = struct.unpack_from('<II', cache)
        self.assertEqual(num_indices, item['triangles'] * 3)
        vertices = np.frombuffer(cache, '<f4', num_vertices * 6, 8).reshape(-1, 6)[:, :3]
        indices = np.frombuffer(cache, '<u4', num_indices, 8 + num_vertices * 24)
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        expected = np.frombuffer(data, dtype, offset=84)['vertices']
        np.testing.assert_allclose(vertices[indices].reshape(-1, 3, 3), expected, atol=.00001, rtol=0)

    def test_invalid_angle(self):
        with self.assertRaises(ValueError):
            build_variant(self.source, 50)

    def test_output_does_not_overwrite(self):
        with tempfile.TemporaryDirectory(prefix='chest-lidar-test-') as folder:
            output = Path(folder) / 'robot.urdf'
            main(['--angle', '60', '--output', str(output)])
            before = output.read_bytes()
            with self.assertRaises(SystemExit):
                main(['--angle', '45', '--output', str(output)])
            self.assertEqual(output.read_bytes(), before)
        with self.assertRaises(SystemExit):
            main(['--angle', '45', '--output', str(self.source)])


if __name__ == '__main__':
    unittest.main()
