"""軽量GLBの面数・材質・座標倍率・原本保護の回帰テスト。"""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
import trimesh

from prepare_mesh_preview import file_hash, prepare_preview


class MeshPreviewTest(unittest.TestCase):
    def test_textured_grid_and_source_protection(self):
        with tempfile.TemporaryDirectory(prefix='mesh-preview-test-') as temporary:
            root = Path(temporary)
            source = root / 'grid.obj'
            texture = root / 'color.png'
            Image.new('RGB', (64, 32), (240, 20, 10)).save(texture)
            (root / 'grid.mtl').write_text('newmtl paint\nKd 1 1 1\nmap_Kd color.png\n')
            lines = ['mtllib grid.mtl', 'usemtl paint']
            side = 16
            for y in range(side):
                for x in range(side):
                    lines.extend([f'v {x * 100} {y * 100} 50', f'vt {x / (side - 1)} {y / (side - 1)}'])
            for y in range(side - 1):
                for x in range(side - 1):
                    a = y * side + x + 1
                    b, c, d = a + 1, a + side, a + side + 1
                    lines.extend([f'f {a}/{a} {b}/{b} {c}/{c}', f'f {b}/{b} {d}/{d} {c}/{c}'])
            source.write_text('\n'.join(lines) + '\n')
            before = file_hash(source)
            result = prepare_preview(source, root / 'preview', num_faces=100,
                                     max_texture_size=16, unit_scale=0.001)
            self.assertEqual(result['source_num_faces'], 450)
            self.assertLessEqual(result['preview_num_faces'], 100)
            self.assertGreater(result['num_textured_parts'], 0)
            self.assertEqual(file_hash(source), before)
            self.assertEqual(result['textures'][0]['preview_size'], [16, 8])
            np.testing.assert_allclose(result['preview_bounds'], [[0, 0, 0.05], [1.5, 1.5, 0.05]])
            np.testing.assert_allclose(np.array(result['source_to_preview_matrix']) @
                                       result['preview_to_source_matrix'], np.eye(4))
            self.assertLess(result['source_surface_to_preview_error']['distance']['max'], 1e-6)
            with self.assertRaises(FileExistsError):
                prepare_preview(source, root / 'preview')
            self.assertEqual(json.loads((root / 'preview/preview.json').read_text())['source_sha256'], before)

    def test_untextured_mesh_and_invalid_scale(self):
        with tempfile.TemporaryDirectory(prefix='mesh-preview-test-') as temporary:
            root = Path(temporary)
            source = root / 'box.obj'
            trimesh.creation.box().export(source)
            result = prepare_preview(source, root / 'preview', num_faces=20, unit_scale=2)
            self.assertEqual(result['preview_num_faces'], 12)
            np.testing.assert_allclose(result['preview_bounds'], [[-1, -1, -1], [1, 1, 1]])
            with self.assertRaises(ValueError):
                prepare_preview(source, root / 'invalid', unit_scale=float('nan'))
            self.assertFalse((root / 'invalid').exists())


if __name__ == '__main__':
    unittest.main()
