from pathlib import Path
import json
from statistics import mean

workspace = Path(__file__).resolve().parents[2]
artifacts = workspace / "artifacts/gng_bsp3d_minimal_20260923"
summary = {"short": {}}
for variant in ("grid", "sampled", "minimal"):
    trials = [json.loads((artifacts / f"{variant}_{idx}.json").read_text()) for idx in (1, 2, 3)]
    reference = [record["graph_sha256"] for record in trials[0]["records"]]
    matched_frames = sum(record["graph_sha256"] == reference[idx]
        for trial in trials for idx, record in enumerate(trial["records"]))
    assert matched_frames == 150
    item = {
        "exec_mean_ms": mean(trial["summary"]["exec_ms"]["mean"] for trial in trials),
        "trial_exec_mean_ms": [trial["summary"]["exec_ms"]["mean"] for trial in trials],
        "num_nodes": trials[0]["summary"]["nodes"]["mean"],
        "quality": trials[0]["quality_summary"],
        "matched_frames": matched_frames,
        "library_sha256": trials[0]["library_sha256"],
    }
    if "statistics" in trials[0]["summary"]:
        item["statistics"] = {name: mean(trial["summary"]["statistics"][name] for trial in trials)
            for name in trials[0]["summary"]["statistics"]}
    summary["short"][variant] = item
    if variant == "minimal":
        minimal_records = [record for trial in trials for record in trial["records"]]
alternative = json.loads((artifacts / "minimal_0_5.json").read_text())
reference = json.loads((artifacts / "minimal_1.json").read_text())
matched = sum(left["graph_sha256"] == right["graph_sha256"]
    for left, right in zip(reference["records"], alternative["records"]))
assert matched == len(reference["records"]) == len(alternative["records"])
summary["voxel_setting_invariance"] = {"matched_frames": matched, "num_frames": len(reference["records"])}
extended = json.loads((artifacts / "extended_minimal.json").read_text())
summary["extended_minimal"] = {"frames": extended["frames"], "warmup": extended["warmup"],
    "summary": extended["summary"], "quality": extended["quality_summary"]}
all_minimal_records = minimal_records + alternative["records"] + extended["records"]
assert len(all_minimal_records) == 500
for record in all_minimal_records:
    stats = record["statistics"]
    assert stats["num_nearest_queries"] == 4000
    assert stats["voxel_ms"] == stats["attention_ms"] == stats["cluster_ms"] == stats["num_probe_points"] == 0
(artifacts / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
for name, item in summary["short"].items():
    print(name, round(item["exec_mean_ms"], 3), round(item["num_nodes"], 1),
        round(item["quality"]["nonzero_coverage_0_2m"] * 100, 2))
print("voxel_setting_invariance", summary["voxel_setting_invariance"])
print("extended_minimal", summary["extended_minimal"]["summary"]["exec_ms"])
