"""正解付き点群から実GNGを経た、同一グラフによる平面検出の前後比較。"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from evaluate import evaluate


def invoke(argv):
    subprocess.run([str(value) for value in argv], check=True, timeout=180,
                   env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})


def timing(path, column):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    values = [float(row[column]) for row in rows if int(row["frame_idx"]) >= 10]
    if not values:
        raise ValueError("定常時間の計測行なし")
    return {"mean": float(np.mean(values)), "p95": float(np.percentile(values, 95))}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(args):
    source = Path(__file__).resolve().parent
    workspace = source.parents[1]
    binaries = workspace / "artifacts/plane_ground_truth_20260929"
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    dataset = output / "dataset"
    begin = time.monotonic()
    invoke([sys.executable, source / "generate.py", "--output", dataset,
            "--seed", args.seed, "--frames", args.frames, "--points", args.points])
    cloud = dataset / (args.scene + ".csv")
    learned = output / "learned"
    learned.mkdir()
    invoke([binaries / "learn", cloud, learned, 1])
    metrics = {}
    gng_time = timing(learned / "stats.csv", "gng_ms")
    metrics.update(gng_mean_ms=gng_time["mean"], gng_p95_ms=gng_time["p95"])
    for variant in ("before", "after"):
        destination = output / variant
        destination.mkdir()
        invoke([binaries / ("planes_" + variant), learned / "graphs.bin",
                workspace / "benchmarks/plane_merge_simplification_20260929/plane_parameters.txt", destination])
        quality = evaluate(cloud, learned / "votes.csv", destination / "assignments.csv", destination)
        metrics.update({variant + "_" + name: value for name, value in quality.items()})
        plane_time = timing(destination / "plane_timings.csv", "plane_ms")
        metrics.update({variant + "_plane_mean_ms": plane_time["mean"], variant + "_plane_p95_ms": plane_time["p95"]})
    # 最初の一条件で投票の副作用と正解ラベルの学習への非混入を照合。
    if args.seed == 1 and args.scene == "coplanar_gap":
        for mode in ("capture_off", "labels_changed"):
            destination = output / mode
            destination.mkdir()
            source_cloud = cloud
            capture = 0
            if mode == "labels_changed":
                source_cloud = destination / "input.csv"
                with cloud.open() as stream, source_cloud.open("w") as sink:
                    reader = csv.DictReader(stream)
                    writer = csv.DictWriter(sink, fieldnames=reader.fieldnames)
                    writer.writeheader()
                    for row in reader:
                        row["gt_label"] = 123
                        writer.writerow(row)
                capture = 1
            invoke([binaries / "learn", source_cloud, destination, capture])
            has_same_graph = digest(learned / "graphs.bin") == digest(destination / "graphs.bin")
            if not has_same_graph:
                raise AssertionError(mode + ": 計測や正解ラベルによるグラフ変化")
            metrics[mode + "_same_graph"] = 1
    metrics["total_case_sec"] = time.monotonic() - begin
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"scene": args.scene, "seed": args.seed, "total_sec": metrics["total_case_sec"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--points", type=int, default=2000)
    run(parser.parse_args())
