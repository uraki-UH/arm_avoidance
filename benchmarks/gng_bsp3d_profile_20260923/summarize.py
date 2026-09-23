import json
from pathlib import Path
from statistics import mean

script_dir = Path(__file__).resolve().parent
# 生データの入力先と再集計結果の出力先。保存済みの集計結果は変更対象外。
root = script_dir.parent.parent / "artifacts" / script_dir.name
schema = json.loads((script_dir / "schema.json").read_text())
original = json.loads((root.parent / "gng_bsp3d_20260923/bsp3d_1.json").read_text())
summary = {}
for mode in ("coarse", "detail", "sampled", "control"):
    report = json.loads((root / (mode + ".json")).read_text())
    item = {"api_summary": report["summary"],
            "num_matching_frames": sum(first["graph_sha256"] == second["graph_sha256"]
                for first, second in zip(report["records"], original["records"]))}
    if mode != "control":
        rows = [json.loads(line.removeprefix("BSP_PROFILE "))
                for line in (root / (mode + ".profile")).read_text().splitlines()
                if line.startswith("BSP_PROFILE ")]
        assert len(rows) == report["frames"]
        rows = rows[report["warmup"]:]
        item["measured_frames"] = len(rows)
        item["stages_ms"] = {name: mean(row["stage_ms"][idx] for row in rows)
                             for idx, name in enumerate(schema["stages"])}
        item["functions_ms"] = {name: mean(row["time_ms"][idx] for row in rows)
                                for idx, name in enumerate(schema["functions"])}
        item["calls"] = {name: mean(row["num_calls"][idx] for row in rows)
                         for idx, name in enumerate(schema["functions"])}
        if mode == "sampled":
            item["samples"] = {name: mean(row["num_samples"][idx] for row in rows)
                               for idx, name in enumerate(schema["functions"])}
        for name in ("num_input", "num_voxel", "num_attention", "num_node_adds", "num_free_slot_checks", "num_add_reject_cap"):
            item[name] = mean(row[name] for row in rows)
    summary[mode] = item
(root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
