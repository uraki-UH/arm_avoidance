#!/usr/bin/env python3
"""同一固定入力に対する曲面所属の比較。品質指標と実行成功の分離。"""
import argparse
import json
from pathlib import Path
import statistics

parser = argparse.ArgumentParser()
parser.add_argument("batch", type=Path)
args = parser.parse_args()
groups = {}
for case_dir in sorted(args.batch.iterdir()):
    if not case_dir.is_dir() or not (case_dir / "quality.jsonl").is_file():
        continue
    trial, condition = case_dir.name.split("_", 1)
    name, mode = condition.rsplit("_", 1)
    groups.setdefault((trial, name), {})[mode] = case_dir
def temporal_ids(rows):
    previous = []
    ids = set()
    num_changes = 0
    num_retained = 0
    num_regions = 0
    for row in rows:
        for region in row["regions"]:
            ids.add(region["id"])
            num_regions += 1
            num_retained += bool(region["retained"])
            members = set(region["nodes"])
            matches = [(len(members & set(old["nodes"])) /
                max(1, len(members | set(old["nodes"]))), old["id"])
                for old in previous if old["type"] == region["type"]]
            overlap, old_id = max(matches, default=(0.0, region["id"]))
            if overlap >= 0.5 and old_id != region["id"]:
                num_changes += 1
        previous = row["regions"]
    return {"num_track_ids": len(ids), "num_id_changes_at_half_overlap": num_changes,
            "retained_region_fraction": num_retained / max(1, num_regions)}

summary = []
for (trial, name), modes in groups.items():
    if set(modes) != {"before", "on", "off"}:
        raise RuntimeError("incomplete modes: " + repr((trial, name, modes)))
    rows = {mode: [json.loads(line) for line in (path / "quality.jsonl").read_text().splitlines()]
            for mode, path in modes.items()}
    baseline = rows["before"]
    for mode in ("off", "on"):
        variant = rows[mode]
        if [row["frame"] for row in variant] != [row["frame"] for row in baseline]:
            raise RuntimeError("frame mismatch")
        union_ious, best_region_ious, num_changed = [], [], 0
        for old, new in zip(baseline, variant):
            old_nodes, new_nodes = set(old["curved_nodes"]), set(new["curved_nodes"])
            union_ious.append(len(old_nodes & new_nodes) / max(1, len(old_nodes | new_nodes))
                              if old_nodes or new_nodes else 1.0)
            for region in old["regions"]:
                old_members = set(region["nodes"])
                best_region_ious.append(max((len(old_members & set(other["nodes"])) /
                    max(1, len(old_members | set(other["nodes"]))) for other in new["regions"]), default=0.0))
            num_changed += old != new
        summary.append({"trial": trial, "input": name, "mode": mode,
            "num_frames": len(baseline), "num_changed_frames": num_changed,
            "curved_union_iou_mean": statistics.mean(union_ious),
            "curved_union_iou_min": min(union_ious),
            "best_old_region_iou_mean": statistics.mean(best_region_ious) if best_region_ious else None,
            "baseline_ids": temporal_ids(baseline), "variant_ids": temporal_ids(variant)})
        if mode == "off" and num_changed:
            raise RuntimeError("history-off outputs differ from before")
output = args.batch / "quality_comparison.json"
output.write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
