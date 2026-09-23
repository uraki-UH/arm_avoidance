from pathlib import Path
from statistics import mean
import json

root = Path(__file__).resolve().parents[2]
artifacts = root / "artifacts/gng_runtime_trials_20260924"
variants = ["tree_raw","grid_raw","tree_voxel_0_1","grid_voxel_0_1","tree_voxel_0_5","grid_voxel_0_5"]
summary = {"short": {}, "extended": {}}
for method in ("combined","native","radix"):
    summary["short"][method] = {}
    for variant in variants:
        if method == "radix" and "voxel" not in variant: continue
        trials = [json.loads((artifacts / method / f"{variant}_extra_{trial}.json").read_text()) for trial in (1,2,3)]
        baseline = json.loads((artifacts / "baseline" / f"{variant}_1.json").read_text())
        matched = 0
        for trial in trials:
            assert trial["accepted"] == baseline["accepted"]
            for record, expected in zip(trial["records"], baseline["records"]):
                assert record["statistics"]["num_nearest_queries"] == 4000
                for key in ("num_input_points","num_training_points","num_zero_samples"):
                    assert record["statistics"][key] == expected["statistics"][key],(method,variant,key)
                matched += record["graph_sha256"] == expected["graph_sha256"]
        if method != "radix": assert matched == 150,(method,variant)
        first = [r["graph_sha256"] for r in trials[0]["records"]]
        assert all([r["graph_sha256"] for r in t["records"]] == first for t in trials)
        summary["short"][method][variant] = {
            "exec_mean_ms": mean(t["summary"]["exec_ms"]["mean"] for t in trials),
            "trial_exec_mean_ms": [t["summary"]["exec_ms"]["mean"] for t in trials],
            "matched_graph_frames": matched, "num_graph_frames": 150,
            "num_nodes": trials[0]["summary"]["nodes"]["mean"],
            "quality": trials[0]["quality_summary"],
            "statistics": {key: mean(t["summary"]["statistics"][key] for t in trials) for key in trials[0]["summary"]["statistics"]},
        }
for variant in variants:
    path = artifacts / "radix" / f"{variant}_extended.json"
    if not path.exists(): continue
    original = json.loads((artifacts / "baseline" / f"{variant}_extended.json").read_text())
    candidate = json.loads(path.read_text())
    assert original["accepted"] == candidate["accepted"]
    for a,b in zip(original["records"],candidate["records"]):
        assert a["statistics"]["num_nearest_queries"] == b["statistics"]["num_nearest_queries"] == 4000
        for key in ("num_input_points","num_training_points","num_zero_samples"):
            assert a["statistics"][key] == b["statistics"][key]
    summary["extended"][variant] = {
        "baseline": original["summary"], "radix": candidate["summary"],
        "baseline_quality": original["quality_summary"], "radix_quality": candidate["quality_summary"],
        "matched_topology_frames": sum(a["topology_sha256"] == b["topology_sha256"] for a,b in zip(original["records"],candidate["records"])),
        "matched_graph_frames": sum(a["graph_sha256"] == b["graph_sha256"] for a,b in zip(original["records"],candidate["records"])),
        "num_frames": len(candidate["records"]),
    }
for method,targets in summary["short"].items():
    for variant,item in targets.items():
        reference = summary["short"]["combined"][variant]["exec_mean_ms"]
        item["reduction_vs_combined_percent"] = (1-item["exec_mean_ms"]/reference)*100
        print(method,variant,round(item["exec_mean_ms"],3),"ms",round(item["reduction_vs_combined_percent"],1),"%")
(artifacts / "extra_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
print("Extended radix",[(k,v["matched_topology_frames"],v["matched_graph_frames"]) for k,v in summary["extended"].items()])
