import json
from pathlib import Path
from statistics import mean

script_dir = Path(__file__).resolve().parent
# 生データの入力先と再集計結果の出力先。保存済みの集計結果は変更対象外。
root = script_dir.parent.parent / "artifacts" / script_dir.name
names = ["learn_search", "attention_search", "query_box", "tree_aabb",
         "candidate_sort", "candidate_evaluate", "tree_add", "tree_move", "tree_remove"]
stages = ["voxel", "attention", "learn", "label", "maintenance", "cluster"]
summary = {}
for variant in ["grid", "spatial"]:
    rows = [json.loads(line.removeprefix("GNG_PROFILE "))
            for line in (root / (variant + ".profile")).read_text().splitlines()
            if line.startswith("GNG_PROFILE ")]
    report = json.loads((root / (variant + ".json")).read_text())
    assert len(rows) == report["frames"]
    rows = rows[report["warmup"]:]
    item = dict(measured_frames=len(rows),
                stages_ms={name: mean(row["stage_ms"][idx] for row in rows)
                           for idx, name in enumerate(stages)},
                functions_ms={name: mean(row["time_ms"][idx] for row in rows)
                              for idx, name in enumerate(names)},
                calls={name: mean(row["num_calls"][idx] for row in rows)
                       for idx, name in enumerate(names)},
                num_candidates=mean(row["num_candidates"] for row in rows),
                api_exec_ms=report["summary"]["exec_ms"]["mean"])
    original = json.loads((root.parent / "gng_spatial_tree_20260923" / (variant + "_1.json")).read_text())
    item["num_matching_frames"] = sum(a["graph_sha256"] == b["graph_sha256"]
                                      for a, b in zip(report["records"], original["records"]))
    summary[variant] = item
(root / "summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
