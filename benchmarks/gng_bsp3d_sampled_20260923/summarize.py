from pathlib import Path
import json
import statistics

workspace = Path(__file__).resolve().parents[2]
artifacts = workspace / "artifacts/gng_bsp3d_sampled_20260923"
def normalize_quality(values):
    return {("max_component_ratio" if key == "largest_component_ratio" else key):value for key,value in values.items()}

summary = {"short": {}, "extended": {}, "grid_invariance": {}}
for unit in ("0_1", "0_5"):
    summary["short"][unit] = {}
    for variant in ("grid", "bsp3d", "pure", "sampled"):
        reports = [json.loads((artifacts / f"{variant}_{unit}_{trial}.json").read_text()) for trial in (1,2,3)]
        records = [entry for report in reports for entry in report["records"][report["warmup"]:]]
        reference = [entry["graph_sha256"] for entry in reports[0]["records"]]
        matches = sum(entry["graph_sha256"] == reference[idx] for report in reports for idx,entry in enumerate(report["records"]))
        assert matches == len(reference)*len(reports), (unit, variant, matches)
        hashes = {report["library_sha256"] for report in reports}
        assert len(hashes) == 1, (unit,variant,hashes)
        output = dict(exec_mean_ms=statistics.mean(entry["exec_ms"] for entry in records),
            total_mean_ms=statistics.mean(entry["total_ms"] for entry in records),
            trial_exec_mean_ms=[report["summary"]["exec_ms"]["mean"] for report in reports],
            matched_graph_frames=matches,num_frames=len(reference)*len(reports),
            quality=normalize_quality(reports[0]["quality_summary"]),library_sha256=next(iter(hashes)),
            max_rss_kib=[report["max_rss_kib"] for report in reports])
        if "statistics" in reports[0]["summary"]:
            output["statistics"] = {name:statistics.mean(entry["statistics"][name] for entry in records)
                for name in reports[0]["summary"]["statistics"]}
        summary["short"][unit][variant] = output
for variant in ("grid","sampled"):
    report = json.loads((artifacts/f"extended_{variant}.json").read_text())
    summary["extended"][variant] = dict(frames=report["frames"],warmup=report["warmup"],
        bag_frames=report["bag_frames"],summary=report["summary"],quality=normalize_quality(report["quality_summary"]))
for variant in ("pure","sampled"):
    reference = json.loads((artifacts/f"{variant}_0_1_1.json").read_text())["records"]
    for unit in ("0_001","1_0"):
        report = json.loads((artifacts/f"grid_invariance_{variant}_{unit}.json").read_text())
        matches = sum(first["graph_sha256"] == second["graph_sha256"] for first,second in zip(reference,report["records"]))
        assert matches == len(reference) == len(report["records"]), (variant,unit,matches)
        summary["grid_invariance"][f"{variant}_{unit}"] = dict(matched_frames=matches,num_frames=len(reference),
            node_grid=report["accepted"]["node.grid"][0])
(artifacts/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
for unit,variants in summary["short"].items():
    for variant,entry in variants.items():
        print(unit,variant,round(entry["exec_mean_ms"],2),round(entry["quality"]["nonzero_coverage_0_2m"]*100,2),
            round(entry["quality"]["nonzero_coverage_0_4m"]*100,2))
print("extended",{variant:round(entry["summary"]["exec_ms"]["mean"],2) for variant,entry in summary["extended"].items()})
print("grid_invariance",summary["grid_invariance"])
