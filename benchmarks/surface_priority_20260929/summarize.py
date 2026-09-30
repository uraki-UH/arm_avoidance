#!/usr/bin/env python3
"""有限反復の条件別中央値と品質比較の統合。"""
import argparse
import csv
import json
from pathlib import Path
import statistics

parser = argparse.ArgumentParser()
parser.add_argument("batch", type=Path)
args = parser.parse_args()
report = json.loads((args.batch / "report.json").read_text())
if report["status"] != "completed" or any(not row["cleanup_ok"] for row in report["records"]):
    raise RuntimeError("batch incomplete or cleanup failed")
grouped = {}
for row in report["records"]:
    grouped.setdefault(row["name"], []).append(row["metrics"])
summary = {}
for name, rows in grouped.items():
    keys = set.intersection(*(set(row) for row in rows))
    summary[name] = {key: statistics.median(row[key] for row in rows) for key in sorted(keys)}
    summary[name]["num_trials"] = len(rows)
output = {"aggregation": "median of per-trial metrics", "status": report["status"],
          "completed": report["completed"], "conditions": summary}
(args.batch / "summary.json").write_text(json.dumps(output, indent=2, allow_nan=False))
fields = ["cpu_ms_mean", "cpu_ms_p95", "wall_ms_mean", "curvature_ms_mean",
          "num_curved_nodes_mean", "mean_residual_mean", "max_residual_max",
          "num_curvature_fits_mean", "num_curvature_reused_mean", "num_curvature_deferred_mean"]
with (args.batch / "summary.csv").open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["condition"] + fields)
    for name, values in sorted(summary.items()):
        writer.writerow([name] + [values.get(key, "") for key in fields])
print(json.dumps(output, indent=2))
