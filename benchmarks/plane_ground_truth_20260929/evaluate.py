"""正解付き元点の勝者投票からの評価。クラスタ番号の一致そのものには非依存。"""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def num_pairs(num):
    return num * (num - 1) // 2


def score_observations(observations):
    # 未所属は一つの予測クラスタとして扱わず、正解0は評価対象外。
    observations = [(gt, pred) for gt, pred in observations if gt != 0]
    gt_counts = Counter(gt for gt, _ in observations if gt > 0)
    pred_counts = Counter(pred for _, pred in observations if pred >= 0)
    joint = Counter((gt, pred) for gt, pred in observations if gt > 0 and pred >= 0)
    true_pairs = sum(num_pairs(count) for count in joint.values())
    predicted_pairs = sum(num_pairs(count) for count in pred_counts.values())
    positive_pairs = sum(num_pairs(count) for count in gt_counts.values())
    num_planar = sum(gt_counts.values())
    num_nonplanar = sum(gt < 0 for gt, _ in observations)
    plane_ious = []
    for gt, count in gt_counts.items():
        plane_ious.append(max((joint[gt, pred] / (count + pred_count - joint[gt, pred])
                              for pred, pred_count in pred_counts.items()), default=0.0))
    return {
        "pair_precision": true_pairs / predicted_pairs if predicted_pairs else None,
        "pair_recall": true_pairs / positive_pairs if positive_pairs else None,
        "macro_best_iou": sum(plane_ious) / len(plane_ious) if plane_ious else None,
        "planar_miss_ratio": sum(gt > 0 and pred < 0 for gt, pred in observations) / num_planar if num_planar else None,
        "nonplanar_absorption_ratio": sum(gt < 0 and pred >= 0 for gt, pred in observations) / num_nonplanar if num_nonplanar else None,
        "num_planar_points": num_planar, "num_nonplanar_points": num_nonplanar,
    }


def load_frames(path):
    frames = defaultdict(list)
    with Path(path).open() as stream:
        for row in csv.DictReader(stream):
            frames[int(row["frame_idx"])].append(row)
    return frames


def evaluate(dataset, votes, assignments, destination, warm_frames=10, coverage_radius=0.2):
    inputs, winners, graphs = map(load_frames, (dataset, votes, assignments))
    if set(inputs) != set(winners):
        raise ValueError("元点と勝者記録のフレーム集合不一致")
    if set(inputs) != set(graphs):
        raise ValueError("各入力フレームに対応する非空グラフが必要")
    rows, distributions = [], []
    for frame_idx, points in sorted(inputs.items()):
        raw = {int(row["point_idx"]): row for row in points}
        if len(raw) != len(points):
            raise ValueError("元点IDの重複")
        graph = {(int(row["node_id"]), int(row["node_frame"])): row for row in graphs[frame_idx]}
        if len(graph) != len(graphs[frame_idx]):
            raise ValueError("ノード世代キーの重複")
        observed, node_votes, seen = [], defaultdict(Counter), set()
        num_unmatched = 0
        for vote in winners[frame_idx]:
            point_idx, gt = int(vote["point_idx"]), int(vote["gt_label"])
            if point_idx in seen or point_idx not in raw or gt != int(raw[point_idx]["gt_label"]):
                raise ValueError("投票の重複、点番号不一致または正解ラベル不一致")
            seen.add(point_idx)
            key = (int(vote["winner_id"]), int(vote["winner_frame"]))
            node = graph.get(key)
            pred = -1 if node is None else int(node["pred_cluster"])
            num_unmatched += node is None
            observed.append((gt, pred))
            if node is not None and gt != 0:
                node_votes[key][gt] += 1
        if seen != set(raw):
            raise ValueError("投票記録の入力点欠落")
        result = score_observations(observed)
        for (node_id, node_frame), counts in node_votes.items():
            total_votes = sum(counts.values())
            for gt, num_votes in sorted(counts.items()):
                distributions.append((frame_idx, node_id, node_frame, gt, num_votes, num_votes / total_votes))
        result.update(frame_idx=frame_idx, num_nodes=len(graph), num_points=len(points),
                      unmatched_winner_ratio=num_unmatched / len(points),
                      no_vote_node_ratio=(len(graph)-len(node_votes))/len(graph) if graph else 1.0,
                      mixed_vote_node_ratio=sum(len(counts) > 1 for counts in node_votes.values()) / len(node_votes) if node_votes else None)
        # 生点群の被覆は評価専用の別探索。投票生成や学習・平面判定への非入力。
        coordinates = np.array([[float(row[k]) for k in ("x", "y", "z")] for row in points])
        node_coordinates = np.array([[float(row[k]) for k in ("x", "y", "z")] for row in graph.values()])
        num_covered = 0
        if len(node_coordinates):
            for begin in range(0, len(coordinates), 128):
                squared = ((coordinates[begin:begin+128, None] - node_coordinates[None])**2).sum(axis=2)
                num_covered += int((squared.min(axis=1) <= coverage_radius**2).sum())
        result["raw_coverage_ratio"] = num_covered / len(points)
        rows.append(result)
    destination = Path(destination)
    with (destination / "node_gt_votes.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(("frame_idx", "node_id", "node_frame", "gt_label", "num_votes", "vote_ratio"))
        writer.writerows(distributions)
    with (destination / "quality.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    summary = {"num_frames": len(rows), "coverage_radius_m": coverage_radius}
    for prefix, subset in (("all", rows), ("steady", [row for row in rows if row["frame_idx"] >= warm_frames])):
        if not subset:
            raise ValueError("評価対象フレームなし")
        for key in rows[0]:
            if key == "frame_idx":
                continue
            values = [row[key] for row in subset if row[key] is not None]
            if values:
                summary[prefix + "_" + key] = float(np.mean(values))
    (destination / "quality.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset"); parser.add_argument("votes"); parser.add_argument("assignments")
    parser.add_argument("destination")
    args = parser.parse_args()
    evaluate(args.dataset, args.votes, args.assignments, args.destination)
