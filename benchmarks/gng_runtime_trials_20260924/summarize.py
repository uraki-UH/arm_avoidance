from pathlib import Path
from statistics import mean
import json

root = Path(__file__).resolve().parents[2]
artifacts = root / "artifacts/gng_runtime_trials_20260924"
variants = ["tree_raw","grid_raw","tree_voxel_0_1","grid_voxel_0_1","tree_voxel_0_5","grid_voxel_0_5"]
methods = {
    "baseline": variants, "hint": ["tree_raw"], "heap": ["tree_raw"],
    "lto": ["tree_raw","tree_voxel_0_1","tree_voxel_0_5"],
    "fused": ["tree_voxel_0_1","tree_voxel_0_5"], "combined": variants,
}
summary = {"short": {}, "invariants": {}, "extended": {}}
num_checked_frames = 0
for method, targets in methods.items():
    summary["short"][method] = {}
    for variant in targets:
        trials = [json.loads((artifacts / method / f"{variant}_{trial}.json").read_text()) for trial in (1,2,3)]
        baseline = json.loads((artifacts / "baseline" / f"{variant}_1.json").read_text())
        reference = [r["graph_sha256"] for r in baseline["records"]]
        matched = 0
        for trial in trials:
            assert trial["accepted"] == baseline["accepted"]
            assert len(trial["records"]) == len(reference) == 50
            for record, expected in zip(trial["records"], baseline["records"]):
                stats = record["statistics"]
                assert stats["num_nearest_queries"] == 4000
                for key in ["num_input_points","num_training_points","num_zero_samples","num_added_nodes","num_deleted_nodes","num_tree_moves"]:
                    assert stats[key] == expected["statistics"][key], (method,variant,record["frame"],key)
                assert record["graph_sha256"] == expected["graph_sha256"], (method,variant,record["frame"],"graph")
                matched += 1
        num_checked_frames += matched
        item = {
            "exec_mean_ms": mean(t["summary"]["exec_ms"]["mean"] for t in trials),
            "trial_exec_mean_ms": [t["summary"]["exec_ms"]["mean"] for t in trials],
            "total_mean_ms": mean(t["summary"]["total_ms"]["mean"] for t in trials),
            "num_nodes": baseline["summary"]["nodes"]["mean"],
            "quality": trials[0]["quality_summary"],
            "matched_frames": matched,
            "statistics": {key: mean(t["summary"]["statistics"][key] for t in trials)
                for key in trials[0]["summary"]["statistics"]},
        }
        summary["short"][method][variant] = item
        print(method, variant, round(item["exec_mean_ms"],3), "ms")
for method, targets in summary["short"].items():
    for variant, item in targets.items():
        item["reduction_percent"] = (1-item["exec_mean_ms"]/summary["short"]["baseline"][variant]["exec_mean_ms"])*100
        print("reduction",method,variant,round(item["reduction_percent"],1))
summary["invariants"]["short_checked_frames"] = num_checked_frames
# 未初期化修正前の記録との比較。速度比較の基準は修正をそろえたbaseline。
previous = root / "artifacts/gng_minimal_comparison_20260924"
summary["invariants"]["previous_comparison"] = {}
for variant in variants:
    if not (previous / f"{variant}_1.json").exists(): continue
    old = json.loads((previous / f"{variant}_1.json").read_text())
    new = json.loads((artifacts / "baseline" / f"{variant}_1.json").read_text())
    summary["invariants"]["previous_comparison"][variant] = {
        "matched_frames": sum(a["graph_sha256"] == b["graph_sha256"] for a,b in zip(old["records"],new["records"])),
        "num_frames": len(new["records"]),
    }
for variant in variants:
    paths = [artifacts / method / f"{variant}_extended.json" for method in ("baseline","combined")]
    if not all(p.exists() for p in paths): continue
    old,new = [json.loads(p.read_text()) for p in paths]
    assert old["accepted"] == new["accepted"]
    assert len(old["records"]) == len(new["records"]) == 300
    for a,b in zip(old["records"],new["records"]):
        assert a["graph_sha256"] == b["graph_sha256"],(variant,a["frame"],"extended graph")
        assert a["statistics"]["num_nearest_queries"] == b["statistics"]["num_nearest_queries"] == 4000
        for key in ["num_input_points","num_training_points","num_zero_samples","num_added_nodes","num_deleted_nodes","num_tree_moves"]:
            assert a["statistics"][key] == b["statistics"][key],(variant,a["frame"],key)
    summary["extended"][variant] = {
        "baseline": old["summary"], "combined": new["summary"], "quality": new["quality_summary"],
        "matched_frames": 300,
        "reduction_percent": (1-new["summary"]["exec_ms"]["mean"]/old["summary"]["exec_ms"]["mean"])*100,
    }
(artifacts / "summary.json").write_text(json.dumps(summary,indent=2)+"\n")
print("Verified",num_checked_frames,"short frames and",300*len(summary["extended"]),"extended pairs")
print("Previous graphs",summary["invariants"]["previous_comparison"])
