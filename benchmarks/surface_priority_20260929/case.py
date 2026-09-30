#!/usr/bin/env python3
"""固定入力1条件の実行と有限の品質・時間指標。"""
import argparse
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=("before", "on", "off"), required=True)
parser.add_argument("--input", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
workspace = Path(__file__).resolve().parents[2]
root = workspace / "artifacts/surface_priority_20260929"
args.output.mkdir(parents=True, exist_ok=True)
variant = "before" if args.mode == "before" else "after"
command = [str(root / variant / "bin/measure"), str(root / "inputs" / (args.input + ".cbor")),
    str(args.output / "frames.jsonl"), str(args.output / "quality.jsonl"), "100", args.mode]
(args.output / "command.json").write_text(json.dumps(command, indent=2))
begin = time.monotonic()
subprocess.run(command, check=True, timeout=80)
elapsed = time.monotonic() - begin
rows = [json.loads(line) for line in (args.output / "frames.jsonl").read_text().splitlines()]
if not rows:
    raise RuntimeError("no measured frames")
metrics = {"num_frames": len(rows), "process_wall_sec": elapsed}
for name in rows[0]:
    values = [row[name] for row in rows if row[name] is not None]
    if not values:
        continue
    if not all(isinstance(value, (float, int)) and math.isfinite(value) for value in values):
        raise RuntimeError("nonfinite metric " + name)
    metrics[name + "_mean"] = statistics.mean(values)
    metrics[name + "_max"] = max(values)
    if name in ("mean_residual", "max_residual"):
        metrics[name + "_num_valid_frames"] = len(values)
    if name.endswith("_ms"):
        metrics[name + "_p95"] = sorted(values)[int((len(values) - 1) * 0.95)]
        metrics[name + "_steady_mean"] = statistics.mean(values[min(5, len(values) - 1):])
if args.input in ("large_curve", "micro_curve"):
    quality = [json.loads(line) for line in (args.output / "quality.jsonl").read_text().splitlines()]
    coverage, pollution = [], []
    for row in quality:
        curve_nodes = set(row["curved_nodes"])
        coverage.append(len(curve_nodes & set(range(288))) / 288)
        pollution.append(len(curve_nodes - set(range(288))) / max(1, len(curve_nodes)))
    metrics["curve_coverage_mean"] = statistics.mean(coverage)
    metrics["curve_coverage_last"] = coverage[-1]
    metrics["curve_pollution_mean"] = statistics.mean(pollution)
    if min(coverage[5:]) < 0.95 or max(pollution) > 0.0:
        raise RuntimeError("synthetic curved support lost or far-plane mixed")
(args.output / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False))
print(json.dumps(metrics), flush=True)
