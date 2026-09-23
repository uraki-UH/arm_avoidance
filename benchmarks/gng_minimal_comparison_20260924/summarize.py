from pathlib import Path
import json
from statistics import mean

workspace = Path(__file__).resolve().parents[2]
artifacts = workspace / "artifacts/gng_minimal_comparison_20260924"
summary = {"conditions": {}, "frames": 50, "warmup": 10, "bag_frames": 30}
for variant in ("tree_raw", "grid_raw", "tree_voxel_0_1", "grid_voxel_0_1", "tree_voxel_0_5", "grid_voxel_0_5"):
    trials = [json.loads((artifacts / f"{variant}_{idx}.json").read_text()) for idx in (1, 2, 3)]
    reference = [record["graph_sha256"] for record in trials[0]["records"]]
    matched = sum(record["graph_sha256"] == reference[idx]
        for trial in trials for idx, record in enumerate(trial["records"]))
    assert matched == 150
    for trial in trials:
        assert trial["frames"] == 50 and trial["warmup"] == 10 and trial["bag_frames"] == 30
        assert trial["accepted"]["node.learning_num"] == [4000]
        assert ("input.voxel_grid_unit" in trial["accepted"]) == ("voxel" in variant)
        assert ("node.grid" in trial["accepted"]) == variant.startswith("grid")
        for record in trial["records"]:
            stats = record["statistics"]
            assert stats["num_nearest_queries"] == 4000
            assert stats["attention_ms"] == stats["cluster_ms"] == stats["num_probe_points"] == 0
            assert (stats["num_input_points"] > stats["num_training_points"]) == ("voxel" in variant)
    item = {
        "exec_mean_ms": mean(trial["summary"]["exec_ms"]["mean"] for trial in trials),
        "trial_exec_mean_ms": [trial["summary"]["exec_ms"]["mean"] for trial in trials],
        "total_mean_ms": mean(trial["summary"]["total_ms"]["mean"] for trial in trials),
        "num_nodes": trials[0]["summary"]["nodes"]["mean"],
        "num_edges": trials[0]["summary"]["edges"]["mean"],
        "quality": trials[0]["quality_summary"],
        "matched_frames": matched,
        "max_rss_mib": mean(trial["max_rss_kib"] / 1024 for trial in trials),
        "statistics": {name: mean(trial["summary"]["statistics"][name] for trial in trials)
            for name in trials[0]["summary"]["statistics"]},
        "library_sha256": trials[0]["library_sha256"],
    }
    summary["conditions"][variant] = item
# コピー元との比較はグラフの一致のみ。過去の実行時間との混合集計なし。
baseline = workspace / "artifacts/gng_bsp3d_minimal_20260923/minimal_1.json"
if baseline.exists():
    old = json.loads(baseline.read_text())
    new = json.loads((artifacts / "tree_raw_1.json").read_text())
    matched = sum(a["graph_sha256"] == b["graph_sha256"] for a, b in zip(old["records"], new["records"]))
    assert matched == len(old["records"]) == len(new["records"])
    summary["baseline_tree_raw_matched_frames"] = matched
(artifacts / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
for name, item in summary["conditions"].items():
    print(name, round(item["exec_mean_ms"], 3), "ms",
        "prepare", round(item["statistics"]["input_prepare_ms"], 3),
        "learn", round(item["statistics"]["learn_ms"], 3),
        "nodes", round(item["num_nodes"]),
        "coverage", round(item["quality"]["nonzero_coverage_0_2m"] * 100, 2),
        round(item["quality"]["nonzero_coverage_0_4m"] * 100, 2))
print("baseline", summary.get("baseline_tree_raw_matched_frames"))
