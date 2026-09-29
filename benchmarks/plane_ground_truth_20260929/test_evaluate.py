import unittest
import csv
import tempfile
from pathlib import Path
from evaluate import evaluate, score_observations


class evaluation_test(unittest.TestCase):
    def test_perfect_and_permuted_ids(self):
        result = score_observations([(1, 91)]*10 + [(2, 7)]*3 + [(-1, -1)]*2)
        for key in ("pair_precision", "pair_recall", "macro_best_iou"):
            self.assertEqual(result[key], 1.0)
        self.assertEqual(result["planar_miss_ratio"], 0.0)

    def test_merge_penalty(self):
        result = score_observations([(1, 7)]*10 + [(2, 7)]*10)
        self.assertLess(result["pair_precision"], 1.0)
        self.assertEqual(result["pair_recall"], 1.0)
        self.assertEqual(result["macro_best_iou"], 0.5)

    def test_split_penalty(self):
        result = score_observations([(1, 7)]*5 + [(1, 8)]*5)
        self.assertEqual(result["pair_precision"], 1.0)
        self.assertLess(result["pair_recall"], 1.0)
        self.assertEqual(result["macro_best_iou"], 0.5)

    def test_missing_and_ignored(self):
        result = score_observations([(1, -1)]*10 + [(0, 99)]*100)
        self.assertIsNone(result["pair_precision"])
        self.assertEqual(result["pair_recall"], 0.0)
        self.assertEqual(result["planar_miss_ratio"], 1.0)

    def test_nonplane_is_not_correct_plane_pair(self):
        result = score_observations([(-1, 5)]*10)
        self.assertEqual(result["pair_precision"], 0.0)
        self.assertEqual(result["nonplanar_absorption_ratio"], 1.0)

    def test_small_plane_macro_protection(self):
        result = score_observations([(1, 7)]*1000 + [(2, -1)]*10)
        self.assertEqual(result["macro_best_iou"], 0.5)

    def test_reused_node_generation_has_no_vote_inheritance(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            fixtures = {
                "input.csv": [["frame_idx", "point_idx", "x", "y", "z", "gt_label"], [0, 0, 0, 0, 0, 1], [0, 1, 0, 0, 0, 1]],
                "votes.csv": [["frame_idx", "point_idx", "gt_label", "winner_id", "winner_frame"], [0, 0, 1, 7, 1], [0, 1, 1, 7, 1]],
                "assignments.csv": [["frame_idx", "node_id", "node_frame", "x", "y", "z", "pred_cluster"], [0, 7, 2, 0, 0, 0, 99]],
            }
            for name, rows in fixtures.items():
                with (path / name).open("w") as stream:
                    csv.writer(stream).writerows(rows)
            result = evaluate(path / "input.csv", path / "votes.csv", path / "assignments.csv", path, warm_frames=0)
            self.assertEqual(result["all_planar_miss_ratio"], 1.0)
            self.assertEqual(result["all_no_vote_node_ratio"], 1.0)
            self.assertEqual(result["all_unmatched_winner_ratio"], 1.0)

    def test_duplicate_vote_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            fixtures = {
                "input.csv": [["frame_idx", "point_idx", "x", "y", "z", "gt_label"], [0, 0, 0, 0, 0, 1]],
                "votes.csv": [["frame_idx", "point_idx", "gt_label", "winner_id", "winner_frame"], [0, 0, 1, 7, 1], [0, 0, 1, 7, 1]],
                "assignments.csv": [["frame_idx", "node_id", "node_frame", "x", "y", "z", "pred_cluster"], [0, 7, 1, 0, 0, 0, 99]],
            }
            for name, rows in fixtures.items():
                with (path / name).open("w") as stream:
                    csv.writer(stream).writerows(rows)
            with self.assertRaises(ValueError):
                evaluate(path / "input.csv", path / "votes.csv", path / "assignments.csv", path, warm_frames=0)


if __name__ == "__main__":
    unittest.main()
