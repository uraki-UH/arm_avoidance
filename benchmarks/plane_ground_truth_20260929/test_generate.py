#!/usr/bin/env python3
"""固定シード再現性、観測点ID、解析形状との整合性の検証。"""

import csv
import json
import math
from pathlib import Path
import tempfile
import unittest

import generate


class test_generate(unittest.TestCase):
    def test_fixed_seed_bytes_and_changed_seed(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            output = Path(temporary_dir)
            first = output / "first"
            second = output / "second"
            changed = output / "changed"
            generate.generate_dataset(first, seed=17, frames=6, points=120)
            generate.generate_dataset(second, seed=17, frames=6, points=120)
            generate.generate_dataset(changed, seed=18, frames=6, points=120)
            for file in first.iterdir():
                self.assertEqual(file.read_bytes(), (second / file.name).read_bytes())
                self.assertNotEqual(file.read_bytes(), (changed / file.name).read_bytes())

    def test_csv_count_identity_and_analytic_geometry(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            output = Path(temporary_dir)
            manifest = generate.generate_dataset(output, seed=20260929, frames=9, points=600)
            self.assertEqual(manifest, json.loads((output / "manifest.json").read_text(encoding="utf-8")))
            self.assertEqual(manifest["schema_version"], "1.0")
            self.assertEqual(len(manifest["scenes"]), 4)
            self.assertEqual(tuple(scene["name"] for scene in manifest["scenes"]), generate.scene_names)
            for scene in manifest["scenes"]:
                with self.subTest(scene=scene["name"]):
                    planes = {plane["gt_label"]: plane for plane in scene["planes"]}
                    allowed_labels = set(planes) | {surface["gt_label"] for surface in scene["nonplane"]}
                    seen_ids = set()
                    frame_labels = {}
                    with (output / scene["file"]).open(encoding="utf-8", newline="") as stream:
                        reader = csv.DictReader(stream)
                        self.assertEqual(reader.fieldnames, list(generate.csv_fields))
                        for row in reader:
                            frame_idx = int(row["frame_idx"])
                            point_idx = int(row["point_idx"])
                            self.assertEqual((frame_idx, point_idx), divmod(len(seen_ids), 600))
                            seen_ids.add((frame_idx, point_idx))
                            gt_label = int(row["gt_label"])
                            self.assertIn(gt_label, allowed_labels)
                            frame_labels.setdefault(frame_idx, set()).add(gt_label)
                            x, y, z = (float(row[axis]) for axis in ("x", "y", "z"))
                            self.assertTrue(all(math.isfinite(value) and abs(value) < 3.0 for value in (x, y, z)))
                            if gt_label > 0:
                                plane = planes[gt_label]
                                residual_m = sum(value * normal for value, normal in zip((x, y, z), plane["normal"])) + plane["offset_m"]
                                if scene["name"] in ("coplanar_gap", "noisy_floor"):
                                    noise_std_m = scene["sampling"]["noise_std_m_by_third"][min(2, frame_idx * 3 // 9)]
                                    self.assertGreaterEqual(abs(x), 0.15)
                                else:
                                    noise_std_m = scene["sampling"]["noise_std_m"]
                                self.assertLess(abs(residual_m), 6.0 * noise_std_m)
                            else:
                                cylinder = scene["nonplane"][0]
                                center_x, center_y = cylinder["center_xy_m"]
                                radius_m = math.hypot(x - center_x, y - center_y)
                                self.assertLess(abs(radius_m - cylinder["radius_m"]), 6.0 * generate.base_noise_std_m)
                    self.assertEqual(len(seen_ids), 9 * 600)
                    self.assertEqual(scene["num_points"], len(seen_ids))
                    self.assertEqual(set(frame_labels), set(range(9)))
                    self.assertTrue(all(labels == allowed_labels for labels in frame_labels.values()))

    def test_physical_labels_and_occlusion(self):
        scenes = {scene["name"]: scene for scene in generate.scene_definitions(frames=9)}
        gap = scenes["coplanar_gap"]
        self.assertEqual(len(gap["planes"]), 1)
        before_gap = generate.sample_frame(gap, 0, 9, 600, 11)
        during_gap = generate.sample_frame(gap, 4, 9, 600, 11)
        self.assertTrue(all(row[3] == 1 for row in before_gap + during_gap))
        self.assertGreater(sum(row[0] > 0.0 for row in before_gap), sum(row[0] > 0.0 for row in during_gap))
        self.assertTrue(all(y <= 0.0 for x, y, z, gt_label in during_gap if x > 0.0))

        noisy_floor = scenes["noisy_floor"]
        self.assertEqual(noisy_floor["planes"], gap["planes"])
        self.assertEqual(noisy_floor["sampling"]["occlusion"], gap["sampling"]["occlusion"])
        self.assertEqual(noisy_floor["sampling"]["noise_std_m_by_third"], [0.003, 0.015, 0.03])
        self.assertEqual(gap["sampling"]["noise_std_m_by_third"], [0.0015, 0.003, 0.0045])
        for frame_idx in (0, 4, 8):
            noisy_points = generate.sample_frame(noisy_floor, frame_idx, 9, 600, 11)
            self.assertTrue(all(gt_label == 1 for x, y, z, gt_label in noisy_points))
            if frame_idx == 4:
                self.assertTrue(all(y <= 0.0 for x, y, z, gt_label in noisy_points if x > 0.0))

        step = scenes["step_and_wall"]
        self.assertEqual([plane["gt_label"] for plane in step["planes"]], [1, 2, 3])
        self.assertEqual(step["planes"][1]["offset_m"], -0.10)
        during_step = generate.sample_frame(step, 4, 9, 600, 11)
        self.assertTrue(all(y <= 0.0 for x, y, z, gt_label in during_step if gt_label == 2))

        curved = scenes["curved_object"]
        self.assertEqual(curved["nonplane"][0]["gt_label"], -1)
        before_curved = generate.sample_frame(curved, 0, 9, 600, 11)
        during_curved = generate.sample_frame(curved, 4, 9, 600, 11)
        self.assertTrue(any(y < 0.0 for x, y, z, gt_label in before_curved if gt_label == -1))
        self.assertTrue(all(y >= 0.0 for x, y, z, gt_label in during_curved if gt_label == -1))

    def test_point_allocation_and_invalid_sizes(self):
        for points in (3, 4, 10, 2000):
            counts = generate.allocate_points(points, [0.68, 0.12, 0.20])
            self.assertEqual(sum(counts), points)
            self.assertTrue(all(count > 0 for count in counts))
        with tempfile.TemporaryDirectory() as temporary_dir:
            with self.assertRaises(ValueError):
                generate.generate_dataset(Path(temporary_dir), 0, frames=0)
            with self.assertRaises(ValueError):
                generate.generate_dataset(Path(temporary_dir), 0, points=2)


if __name__ == "__main__":
    unittest.main()
