from pathlib import Path
from statistics import mean
import json
root = Path(__file__).resolve().parents[2]
artifacts = root / "artifacts/gng_runtime_trials_20260924"
summary = {"cpu": int((artifacts / "pinned_cpu.txt").read_text()), "conditions": {}}
for method, variants in [("baseline",("tree_raw","tree_voxel")),("combined",("tree_raw","tree_voxel")),("radix",("tree_voxel",))]:
    for variant in variants:
        trials = [json.loads((artifacts / method / f"{variant}_pinned_{idx}.json").read_text()) for idx in (1,2,3)]
        summary["conditions"][method+"/"+variant] = {
            "mean_ms": mean(t["summary"]["exec_ms"]["mean"] for t in trials),
            "trial_means_ms": [t["summary"]["exec_ms"]["mean"] for t in trials],
        }
for name,item in summary["conditions"].items():
    variant = name.split("/")[1]
    item["reduction_percent"] = (1-item["mean_ms"]/summary["conditions"]["baseline/"+variant]["mean_ms"])*100
(artifacts / "pinned_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
print(json.dumps(summary,indent=2))
