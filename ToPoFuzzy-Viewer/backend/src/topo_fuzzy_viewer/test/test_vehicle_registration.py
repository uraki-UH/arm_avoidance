"""欠損観測、外れ値、分類保留、入力検証の回帰。"""
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from vehicle_registration import load_models, register_vehicle, validate_observation, yaw_rotation


def snapshot(points):
    return {'frame_id': 'map', 'source_id': '/test', 'selection': {'kind': 'cluster', 'id': 1},
            'graph': {'timestamp': 42, 'nodes': [dict(x=x, y=y, z=z) for x, y, z in points]}}


class VehicleRegistrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = load_models()
        cls.rng = np.random.default_rng(87)

    def test_partial_sedan_pose_and_class(self):
        source = self.models[0]['points']
        source = source[(source[:, 1] < -.4) & (source[:, 2] > .25)][::3]
        points = source @ yaw_rotation(.7).T + [8, -4, 1]
        points += self.rng.normal(0, .015, points.shape)
        before = points.copy()
        result = register_vehicle(snapshot(points), self.models)
        best = result['candidates'][0]
        self.assertEqual(best['model_id'], 'drivaer_sedan')
        self.assertEqual(result['state'], 'class_candidate')
        self.assertLess(abs(best['yaw_deg'] - math.degrees(.7)), 3)
        self.assertLess(np.linalg.norm(np.array(best['translation']) - [8,-4,1]), .1)
        self.assertGreater(best['match_ratio'], .95)
        self.assertGreater(best['unmatched_ratio'], .4)
        self.assertLess(best['inlier_rms_m'], .06)
        np.testing.assert_array_equal(points, before)
        json.dumps(result, allow_nan=False)
        for candidate in result['candidates']:
            self.assertEqual(len(candidate['matched_positions']) + len(candidate['unmatched_positions']), 3500*3)

    def test_unknown_object_is_not_forced_into_a_class(self):
        # 平面の一部だけなら、箱型車体への高い一致率でも車両判定を保留
        points = np.array([[x,y,0.] for x in np.linspace(-.6,.6,8) for y in np.linspace(-.4,.4,6)])
        result = register_vehicle(snapshot(points), self.models)
        self.assertEqual(result['state'], 'insufficient')

    def test_truck_complete_observation(self):
        source = self.models[3]['points'][::7]
        points = source @ yaw_rotation(-1.1).T + [-5, 12, -2]
        result = register_vehicle(snapshot(points), self.models)
        self.assertEqual(result['candidates'][0]['model_id'], 'box_truck')
        self.assertGreater(result['candidates'][0]['match_ratio'], .95)
        self.assertGreater(result['candidates'][0]['support_ratio'], .8)

    def test_outliers_remain_in_final_metrics(self):
        source = self.models[0]['points'][::8] @ yaw_rotation(.3).T
        points = np.vstack((source, self.rng.uniform([6,6,6],[7,7,7],(15,3))))
        result = register_vehicle(snapshot(points), self.models[:1])
        candidate = result['candidates'][0]
        self.assertEqual(candidate['num_observed'], len(points))
        self.assertGreaterEqual(len(candidate['outlier_positions'])//3, 15)
        self.assertLess(candidate['match_ratio'], 1)
        self.assertGreater(candidate['match_ratio'], .85)

    def test_invalid_input(self):
        for points in [np.zeros((1,3)), np.zeros((12,3)), np.full((12,3), np.nan)]:
            with self.assertRaises(ValueError): validate_observation(snapshot(points))
        with self.assertRaises(ValueError): validate_observation({'graph': {'nodes': []}})
        wrong_frame = snapshot(self.models[0]['points'][::100])
        wrong_frame['graph']['frameId'] = 'lidar'
        with self.assertRaises(ValueError): validate_observation(wrong_frame)
        wrong_selection = snapshot(self.models[0]['points'][::100])
        wrong_selection['selection']['kind'] = 'node'
        with self.assertRaises(ValueError): validate_observation(wrong_selection)
        points = self.models[0]['points'][::100]
        for value in [0, 2, float('nan'), True, '0.25']:
            with self.assertRaises(ValueError): register_vehicle(snapshot(points), self.models, value)


if __name__ == '__main__':
    unittest.main()
