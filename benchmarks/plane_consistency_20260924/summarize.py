"""共通入力・交互3試行の非時間出力照合と計測集計。"""

import argparse
import csv
import hashlib
import json
import statistics
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--prefix", choices=("", "scale_", "contact_", "fragment_", "direction_", "absorption_"), default="")
args = parser.parse_args()
trial_root = Path(__file__).resolve().parents[2] / "artifacts/plane_consistency_20260924"
result = {"frames": 150, "warmup_frames": 50, "trials": 3,
          "input_sha256": hashlib.sha256((trial_root / "frames.bin").read_bytes()).hexdigest()}
for method in ("before", "after"):
    trials = []
    expected = None
    for trial in range(1, 4):
        with (trial_root / f"{args.prefix}{method}" / f"final_{trial}.csv").open() as source:
            rows = list(csv.DictReader(source))
        assert len(rows) == result["frames"]
        comparable = [{k: v for k, v in row.items() if k not in ("cpu_ms", "wall_ms")}
                      for row in rows]
        assert expected is None or comparable == expected
        expected = comparable
        rows = rows[result["warmup_frames"]:]
        trials.append({key: statistics.fmean(float(row[key]) for row in rows)
                       for key in rows[0] if key != "frame"})
    result[method] = {key: statistics.median(trial[key] for trial in trials) for key in trials[0]}
    result[method]["trial_cpu_mean_ms"] = [trial["cpu_ms"] for trial in trials]
    result[method]["unassigned"] = result[method]["nodes"] - result[method]["assigned"]
    result[method]["assigned_percent"] = 100 * result[method]["assigned"] / result[method]["nodes"]
result["cpu_reduction_percent"] = 100 * (1 - result["after"]["cpu_ms"] / result["before"]["cpu_ms"])
Path(__file__).with_name(f"{args.prefix}summary.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
