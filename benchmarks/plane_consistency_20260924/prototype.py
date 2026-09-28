#!/usr/bin/env python3
"""平面差分試作の独立比較。試験用コピーへの機械的な設定・計測挿入。"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys

import profile as plane_profile


root = Path(__file__).resolve().parents[2]
modes = ("before", "control", "delta", "block", "both")


def prepare(output, baseline):
    output.mkdir(parents=True, exist_ok=False)
    flags = plane_profile.compile_flags()
    source = root / "ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp"
    for name, path in (("before", baseline), ("current", source)):
        subprocess.run(flags + ["-c", str(path), "-o", str(output / (name + ".o"))],
                       check=True, timeout=90)
    replay = (Path(__file__).parent / "replay.cpp").read_text()
    replay = replay.replace("declareClusterOptions(parameters)", 'declareClusterOptions(parameters, "", true)')
    replay = '#include <cstdlib>\n#include "output_fingerprint.hpp"\n' + replay
    marker = "  fuzzrobo::topological_plane::incremental::Clusterizer clusterizer(options);"
    assert replay.count(marker) == 1
    replay = replay.replace(marker, '''
  options.enable_delta_statistics = std::getenv("PLANE_DELTA") != nullptr;
  options.enable_block_retention = std::getenv("PLANE_BLOCK") != nullptr;
''' + marker)
    marker = "    const auto &s = result.statistics;"
    assert replay.count(marker) == 1
    replay = replay.replace(marker, '''
    auto membership = result.clusters;
    std::cerr << "GEOMETRY," << frame << std::setprecision(17);
    for (auto &cluster : membership.clusters) {
      const auto point = [](const auto &p) {std::cerr << ',' << p.x << ',' << p.y << ',' << p.z;};
      point(cluster.centroid); point(cluster.normal); point(cluster.tangent_u); point(cluster.tangent_v);
      for (const auto value : cluster.position_covariance) {std::cerr << ',' << value;}
      for (const auto &p : cluster.boundary) {point(p);}
      std::cerr << ',' << cluster.area << ',' << cluster.extent_u << ',' << cluster.extent_v
                << ',' << cluster.local_spacing << ',' << cluster.planarity << ',' << cluster.residual_ratio;
      const auto id = cluster.id;
      const auto source_label = cluster.source_label;
      auto indices = std::move(cluster.node_indices);
      auto edges = std::move(cluster.support_edges);
      cluster = ais_gng_msgs::msg::PlaneCluster{};
      cluster.id = id; cluster.source_label = source_label;
      cluster.node_indices = std::move(indices); cluster.support_edges = std::move(edges);
    }
    std::cerr << "\\nMEMBERS," << frame << ',' << output_fingerprint(membership)
              << "\\nFULL," << frame << ',' << output_fingerprint(result.clusters)
              << "\\nREUSED," << frame << ',' << result.statistics.num_retention_reused_nodes << '\\n';
''' + marker)
    (output / "replay.cpp").write_text(replay)
    for name in ("before", "current"):
        subprocess.run(flags + [str(output / "replay.cpp"), str(output / (name + ".o")),
                               "-o", str(output / name)], check=True, timeout=90)
    test = root / "ais_gng_cpu/src/ais_gng/test/test_plane_cluster_incremental.cpp"
    subprocess.run(flags + [str(test), str(output / "current.o"), "-lgtest_main", "-lgtest",
                           "-pthread", "-o", str(output / "tests")], check=True, timeout=90)
    with (output / "tests.log").open("w") as log:
        subprocess.run([str(output / "tests")], stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=60)
    manifest = {"cases": [{"name": mode, "argv": [sys.executable, str(Path(__file__).resolve()),
                 "run", str(output), "--mode", mode, "--case-dir", "@case_dir@"]}
                         for mode in modes]}
    (output / "cases.json").write_text(json.dumps(manifest, indent=2))


def run(output, mode, case_dir):
    env = dict(os.environ)
    env.pop("PLANE_DELTA", None)
    env.pop("PLANE_BLOCK", None)
    if mode in ("delta", "both"):
        env["PLANE_DELTA"] = "1"
    if mode in ("block", "both"):
        env["PLANE_BLOCK"] = "1"
    with (case_dir / "stdout.log").open("w") as stdout, (case_dir / "stderr.log").open("w") as stderr:
        subprocess.run([str(output / ("before" if mode == "before" else "current")),
                        str(root / "artifacts/plane_consistency_20260924/frames.bin"), "--diagnose"],
                       env=env, stdout=stdout, stderr=stderr, check=True, timeout=30)


def summarize(output):
    report = json.loads((output / "batch/report.json").read_text())
    assert report["status"] == "completed" and all(r["cleanup_ok"] for r in report["records"])
    records = []
    for path in sorted((output / "batch").rglob("stdout.log")):
        rows = list(csv.DictReader(path.open()))
        if not rows:
            continue
        lines = path.with_name("stderr.log").read_text().splitlines()
        records.append({"case": str(path.parent.relative_to(output / "batch")),
                        "rows": rows, "lines": lines})
    ref = next(r for r in records if "before" in r["case"])
    def selected(record, prefix):
        return [line for line in record["lines"] if line.startswith(prefix + ",")]
    summary = []
    for record in records:
        assert len(record["rows"]) == len(ref["rows"]) == 150
        geometry = [[float(v) for v in line.split(",")[2:]] for line in selected(record, "GEOMETRY")]
        reference = [[float(v) for v in line.split(",")[2:]] for line in selected(ref, "GEOMETRY")]
        assert all(math.isfinite(value) for row in geometry for value in row), record["case"]
        has_same_members = selected(record, "MEMBERS") == selected(ref, "MEMBERS")
        has_same_shape = len(geometry) == len(reference) and all(len(a) == len(b) for a, b in zip(geometry, reference))
        max_error = max((abs(a - b) for va, vb in zip(geometry, reference)
                         for a, b in zip(va, vb)), default=0) if has_same_members and has_same_shape else None
        item = {"case": record["case"], "cpu_ms": statistics.fmean(float(r["cpu_ms"]) for r in record["rows"][50:]),
                "wall_ms": statistics.fmean(float(r["wall_ms"]) for r in record["rows"][50:]),
                "is_membership_and_edges_equal": has_same_members,
                "is_full_output_equal": selected(record, "FULL") == selected(ref, "FULL"),
                "max_geometry_abs_error": max_error,
                "mean_reused_nodes": statistics.fmean(float(line.split(",")[2]) for line in selected(record, "REUSED")[50:])}
        summary.append(item)
    assert len(summary) == report["total"]
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    assert all(item["is_membership_and_edges_equal"] and
               item["max_geometry_abs_error"] is not None and
               item["max_geometry_abs_error"] <= 1.e-5 for item in summary)
    assert all(item["is_full_output_equal"] for item in summary
               if item["case"].endswith(("before", "control", "block")))


def validate(output):
    flags = plane_profile.compile_flags()
    source = root / "ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp"
    current = source.read_text()
    marker = "  : options(std::move(input_options))\n  {"
    assert current.count(marker) == 1
    # 既存55件全体への試作経路の適用。本番のコンストラクタへの変更なし。
    current = '#include <cstdlib>\n' + current.replace(marker, marker + '''
    options.enable_delta_statistics = std::getenv("PLANE_DELTA") != nullptr;
    options.enable_block_retention = std::getenv("PLANE_BLOCK") != nullptr;
''')
    (output / "forced.cpp").write_text(current)
    subprocess.run(flags + ["-c", str(output / "forced.cpp"), "-o", str(output / "forced.o")],
                   check=True, timeout=90)
    subprocess.run(flags + [str(root / "ais_gng_cpu/src/ais_gng/test/test_plane_cluster_incremental.cpp"),
                           str(output / "forced.o"), "-lgtest_main", "-lgtest", "-pthread",
                           "-o", str(output / "forced_tests")], check=True, timeout=90)
    for mode in ("delta", "block", "both"):
        env = dict(os.environ)
        env.pop("PLANE_DELTA", None)
        env.pop("PLANE_BLOCK", None)
        if mode in ("delta", "both"):
            env["PLANE_DELTA"] = "1"
        if mode in ("block", "both"):
            env["PLANE_BLOCK"] = "1"
        with (output / (mode + "_tests.log")).open("w") as log:
            subprocess.run([str(output / "forced_tests"), "--gtest_filter=PlaneClusterIncremental.*"],
                           stdout=log, stderr=subprocess.STDOUT, env=env, check=True, timeout=30)
    print("existing_suite: 55 tests x 3 modes passed")
    benchmark = (root / "ais_gng_cpu/src/ais_gng/test/benchmark_plane_cluster_incremental.cpp").read_text()
    benchmark = '#include "output_fingerprint.hpp"\n' + benchmark
    marker = "      map = make_dynamic_map(source, iter);"
    assert benchmark.count(marker) == 1
    benchmark = benchmark.replace(marker, '''
      map = source;
      // 総ノード数に依存しない32点の移動。入力生成は計測区間外。
      for (std::size_t idx = 0U; idx < std::min<std::size_t>(32U, map.nodes.size()); ++idx) {
        map.nodes[idx].pos.z += static_cast<float>(0.02 * std::sin(iter * 0.1));
      }
''')
    marker = "      ais_gng_msgs::msg::to_block_style_yaml(result.clusters, trace);"
    assert benchmark.count(marker) == 1
    benchmark = benchmark.replace(marker, '''
      auto membership = result.clusters;
      for (auto &cluster : membership.clusters) {
        const auto id = cluster.id;
        const auto source_label = cluster.source_label;
        auto indices = std::move(cluster.node_indices);
        auto edges = std::move(cluster.support_edges);
        cluster = ais_gng_msgs::msg::PlaneCluster{};
        cluster.id = id; cluster.source_label = source_label;
        cluster.node_indices = std::move(indices); cluster.support_edges = std::move(edges);
      }
      trace << output_fingerprint(membership) << '\\n';
''')
    (output / "scaling.cpp").write_text(benchmark)
    subprocess.run(flags + [str(output / "scaling.cpp"), str(output / "forced.o"),
                           "-o", str(output / "scaling")], check=True, timeout=90)
    cases = []
    for data in ("steady", "dynamic"):
        for size in (2, 8, 18):
            for mode in ("control", "delta", "both"):
                env = {} if mode == "control" else {"PLANE_DELTA": "1"}
                if mode == "both":
                    env["PLANE_BLOCK"] = "1"
                cases.append({"name": f"{data}_{size}_{mode}", "argv": [str(output / "scaling"),
                              data, str(size), "50", "@case_dir@/membership.trace"], "env": env})
    (output / "scaling_cases.json").write_text(json.dumps({"cases": cases}, indent=2))


def summarize_scaling(output):
    report = json.loads((output / "scaling_batch/report.json").read_text())
    assert report["status"] == "completed" and all(r["cleanup_ok"] for r in report["records"])
    results = []
    traces = {}
    for record in report["records"]:
        path = Path(record["log"])
        values = dict(item.split("=", 1) for item in path.read_text().strip().split())
        data, size, mode = record["name"].split("_")
        trace = path.with_name("membership.trace").read_bytes()
        key = (data, size)
        if mode == "control":
            traces[key] = trace
        values["is_membership_and_edges_equal"] = trace == traces[key]
        values["variant"] = mode
        results.append(values)
    (output / "scaling_summary.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    assert all(item["is_membership_and_edges_equal"] for item in results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("prepare", "run", "summarize", "validate", "summarize-scaling"))
    parser.add_argument("output", type=Path)
    parser.add_argument("--baseline-source", type=Path)
    parser.add_argument("--mode", choices=modes)
    parser.add_argument("--case-dir", type=Path)
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(args.output.resolve(), args.baseline_source.resolve())
    elif args.action == "run":
        run(args.output.resolve(), args.mode, args.case_dir)
    elif args.action == "validate":
        validate(args.output.resolve())
    elif args.action == "summarize-scaling":
        summarize_scaling(args.output.resolve())
    else:
        summarize(args.output.resolve())
