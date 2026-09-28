#!/usr/bin/env python3
"""保存済み実GNG入力による段階別CPU時間の計測。本番ソース・配布先への変更なし。"""

import argparse
import csv
import io
import json
from pathlib import Path
import statistics
import subprocess
import sys


root = Path(__file__).resolve().parents[2]
stages = ["prepareFrame", "carryOverLabels", "refitClusters", "releaseOutliers",
          "maintenancePass", "birthClusters", "splitClusters", "mergeClusters",
          "cullClusters", "buildOutput"]


def compile_flags():
    flags = ["g++", "-std=c++17", "-O3", "-DNDEBUG",
             "-I" + str(Path(__file__).resolve().parent),
             "-I" + str(root / "ais_gng_cpu/src/ais_gng/include"), "-I/usr/include/eigen3",
             "-I" + str(root / "ais_gng_cpu/src/gng_cpu/include"),
             "-I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs"]
    for package in ("geometry_msgs", "std_msgs", "builtin_interfaces", "rosidl_runtime_cpp",
                    "rosidl_runtime_c", "rosidl_typesupport_interface", "rcutils"):
        flags.append("-I/opt/ros/humble/include/" + package)
    return flags


def prepare(output, baseline_source=None, enable_phase_comparison=False, enable_visual_comparison=False,
            enable_logic_comparison=False, enable_direct_comparison=False):
    output.mkdir(parents=True, exist_ok=False)
    source = root / "ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp"
    original = source.read_text()
    # コピーに限る機械的計測挿入。関数境界の変更検出付き。
    helper = '''
#include <array>
#include <ctime>
#include <iostream>
namespace {
double profile_cpu_ms() {
  timespec value{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value);
  return value.tv_sec * 1000.0 + value.tv_nsec * 1.e-6;
}
std::array<double, 10> profile_times{};
std::array<unsigned, 10> profile_counts{};
std::array<double, 5> profile_prepare_times{};
struct profile_scope {
  unsigned idx;
  double start = profile_cpu_ms();
  ~profile_scope() {
    profile_times[idx] += profile_cpu_ms() - start;
    ++profile_counts[idx];
  }
};
struct profile_frame {
  profile_frame() { profile_times.fill(0); profile_counts.fill(0); profile_prepare_times.fill(0); }
  ~profile_frame() {
    std::cerr << "PROFILE";
    for (auto value : profile_times) { std::cerr << ',' << value; }
    for (auto value : profile_counts) { std::cerr << ',' << value; }
    std::cerr << '\\n';
    std::cerr << "PREP";
    for (auto value : profile_prepare_times) { std::cerr << ',' << value; }
    std::cerr << '\\n';
  }
};
}
'''
    instrumented = helper + original
    for idx, name in enumerate(stages):
        signature = next(line for line in original.splitlines()
                         if line.startswith("  ") and name + "(" in line
                         and line.strip().startswith(("void ", "bool ", "std::size_t ")))
        marker = signature + "\n  {"
        assert instrumented.count(marker) == 1, name
        instrumented = instrumented.replace(marker, marker + f"\n    profile_scope stage_timer{{{idx}}};")
    marker = "ClusterResult Clusterizer::update(const ais_gng_msgs::msg::TopologicalMap &map)\n{"
    direct_marker = ("ClusterResult Clusterizer::update(const graph_view &map, const std_msgs::msg::Header &header,\n"
                     "  const std::uint32_t frame_number)\n{")
    if direct_marker in instrumented:
        marker = direct_marker
    assert instrumented.count(marker) == 1
    instrumented = instrumented.replace(marker, marker + "\n  profile_frame frame_timer;")
    marker = "profile_scope stage_timer{0};"
    instrumented = instrumented.replace(marker, marker + "\n    double prepare_started = profile_cpu_ms();")
    # 入力コピー、CSR、局所量、EMA、種スコアの区間計測。生産ソースへの計測挿入なし。
    for idx, marker in enumerate(("    // 次数を数えて", "    // 局所間隔は", "    // 法線へEMA", "    // 新規クラスタの種は")):
        assert instrumented.count(marker) == 1
        instrumented = instrumented.replace(marker,
            f"    profile_prepare_times[{idx}] = profile_cpu_ms() - prepare_started;\n"
            "    prepare_started = profile_cpu_ms();\n" + marker)
    marker = "  }\n\n  // 前フレームの所属をGNGノードIDで引き継ぐ。"
    assert instrumented.count(marker) == 1
    instrumented = instrumented.replace(marker,
        "    profile_prepare_times[4] = profile_cpu_ms() - prepare_started;\n" + marker)
    (output / "profile.cpp").write_text(instrumented)
    # CPU直結経路のrho再利用を有効化。既存replayの既定falseとの差の明示。
    replay = (root / "benchmarks/plane_consistency_20260924/replay.cpp").read_text()
    replay = replay.replace("declareClusterOptions(parameters)", 'declareClusterOptions(parameters, "", true)')
    replay = '#include <cstdlib>\n#include "output_fingerprint.hpp"\n' + replay
    marker = '  fuzzrobo::topological_plane::incremental::Clusterizer clusterizer(options);'
    assert replay.count(marker) == 1
    replay = replay.replace(marker, '''
  if (const auto *phases = std::getenv("PLANE_ACQUISITION_PHASES")) {
    options.num_acquisition_phases = static_cast<std::size_t>(std::stoul(phases));
  }
  if (std::getenv("PLANE_FORCE_DELTA")) {options.enable_delta_statistics = true;}
  if (const auto *value = std::getenv("PLANE_TEMPORAL")) {
    options.enable_temporal_update = std::string(value) == "1";
  }
  if (const auto *value = std::getenv("PLANE_SUPPORT_EDGES")) {
    options.enable_support_edges = std::string(value) == "1";
  }
''' + marker)
    marker = '    const auto &s = result.statistics;'
    assert replay.count(marker) == 1
    replay = replay.replace(marker, '''
    std::cerr << "OUTPUT," << frame << "," << output_fingerprint(result.clusters) << "\\n";
    std::cerr << "CORE," << frame << "," << output_fingerprint(result.clusters, false) << "\\n";
    std::cerr << "CONNECTIVITY," << frame << ',' << result.statistics.num_connectivity_reused_clusters << "\\n";
    std::size_t num_edge_points = 0U;
    for (const auto &cluster : result.clusters.clusters) {num_edge_points += cluster.support_edges.size();}
    std::cerr << "EDGES," << frame << ',' << num_edge_points << "\\n";
    double weighted_residual = 0.0, max_residual = 0.0;
    for (const auto &cluster : result.clusters.clusters) {
      weighted_residual += cluster.node_indices.size() * cluster.residual_ratio;
      max_residual = std::max(max_residual, static_cast<double>(cluster.residual_ratio));
    }
    std::cerr << "QUALITY," << frame << ',' << weighted_residual << ',' << max_residual << '\\n';
''' + marker)
    if enable_direct_comparison:
        replay = '#include <fuzzrobo/libgng/api.h>\n' + replay
        marker = "    const auto start = std::chrono::steady_clock::now();"
        assert replay.count(marker) == 1
        replay = replay.replace(marker, '''
    // 既存GNG配列相当の入力準備。全方式で同じ準備を計測外に配置。
    std::vector<::TopologicalNode> raw_nodes(map.nodes.size());
    for (std::size_t idx = 0; idx < raw_nodes.size(); ++idx) {
      const auto &node = map.nodes[idx];
      auto &raw = raw_nodes[idx];
      raw.id = node.id; raw.label = node.label; raw.rho = node.rho;
      raw.pos = {node.pos.x, node.pos.y, node.pos.z};
      raw.normal = {node.normal.x, node.normal.y, node.normal.z};
    }
''' + marker)
        marker = '    const auto result = clusterizer.update(map);'
        assert replay.count(marker) == 1
        replay = replay.replace(marker, '''
#ifdef PLANE_DIRECT_REPLAY
    const auto result = std::getenv("PLANE_DIRECT_INPUT") ? clusterizer.update(
      fuzzrobo::topological_plane::incremental::make_graph_view(
        raw_nodes.data(), raw_nodes.size(), map.edges.data(), map.edges.size()),
      map.header, map.frame_number) : clusterizer.update(map);
#else
''' + marker + '\n#endif')
    (output / "replay.cpp").write_text(replay)
    flags = compile_flags()
    builds = [("baseline", baseline_source or source), ("profile", output / "profile.cpp")]
    if baseline_source:
        builds.append(("after", source))
    for name, path in builds:
        extra = ["-DPLANE_DIRECT_REPLAY"] if enable_direct_comparison and name != "baseline" else []
        subprocess.run(flags + extra + [str(path), str(output / "replay.cpp"), "-o", str(output / name)],
                       check=True, timeout=90)
    manifest = {"cases": [{"name": name, "argv": [sys.executable, str(Path(__file__).resolve()),
                 "run", str(output), "--mode", name, "--case-dir", "@case_dir@"]}
                          for name, _ in builds]}
    if enable_phase_comparison:
        assert baseline_source
        manifest = {"cases": [{"name": f"{mode}_{phases}", "env": {
                    "PLANE_FORCE_DELTA": "1", "PLANE_ACQUISITION_PHASES": str(phases)},
                    "argv": [sys.executable, str(Path(__file__).resolve()), "run", str(output),
                             "--mode", mode, "--case-dir", "@case_dir@"]}
                    for mode, phases in (("baseline", 1), ("after", 1), ("after", 4),
                                         ("after", 8), ("profile", 4))]}
    if enable_visual_comparison:
        assert baseline_source and not enable_phase_comparison
        manifest = {"cases": [{"name": f"{mode}_{phases}_{edges}", "env": {
                    "PLANE_FORCE_DELTA": "1", "PLANE_ACQUISITION_PHASES": str(phases),
                    "PLANE_SUPPORT_EDGES": str(edges)},
                    "argv": [sys.executable, str(Path(__file__).resolve()), "run", str(output),
                             "--mode", mode, "--case-dir", "@case_dir@"]}
                    for phases in (4, 10)
                    for mode, edges in (("baseline", 1), ("after", 0), ("after", 1), ("profile", 0))]}
    if enable_logic_comparison:
        assert baseline_source
        manifest = {"cases": [{"name": name, "env": {
                    "PLANE_FORCE_DELTA": "1", "PLANE_ACQUISITION_PHASES": "5",
                    "PLANE_TEMPORAL": str(temporal)},
                    "argv": [sys.executable, str(Path(__file__).resolve()), "run", str(output),
                             "--mode", mode, "--case-dir", "@case_dir@"]}
                    for name, mode, temporal in (
                        ("baseline", "baseline", 0), ("off", "after", 0),
                        ("temporal", "after", 1), ("profile", "profile", 1))]}
    if enable_direct_comparison:
        assert baseline_source
        manifest = {"cases": [{"name": name,
                    "env": {"PLANE_ACQUISITION_PHASES": "5", "PLANE_TEMPORAL": "0",
                            **({"PLANE_DIRECT_INPUT": "1"} if name in ("raw", "profile") else {})},
                    "argv": [sys.executable, str(Path(__file__).resolve()), "run", str(output),
                             "--mode", mode, "--case-dir", "@case_dir@"]}
                    for name, mode in (("baseline", "baseline"), ("ros", "after"),
                                       ("raw", "after"), ("profile", "profile"))]}
    (output / "cases.json").write_text(json.dumps(manifest, indent=2))


def run(output, mode, case_dir):
    with (case_dir / "stdout.log").open("w") as stdout, (case_dir / "stderr.log").open("w") as stderr:
        subprocess.run([str(output / mode),
                        str(root / "artifacts/plane_consistency_20260924/frames.bin"), "--diagnose"],
                       check=True, timeout=25, stdout=stdout, stderr=stderr)


def validate(output, baseline_source):
    output.mkdir(parents=True, exist_ok=False)
    test_root = root / "ais_gng_cpu/src/ais_gng/test"
    benchmark = (test_root / "benchmark_plane_cluster_incremental.cpp").read_text()
    benchmark = '#include "output_fingerprint.hpp"\n' + benchmark
    marker = 'ais_gng_msgs::msg::to_block_style_yaml(result.clusters, trace);'
    assert benchmark.count(marker) == 1
    benchmark = benchmark.replace(marker, 'trace << output_fingerprint(result.clusters) << "\\n";')
    (output / "synthetic.cpp").write_text(benchmark)
    flags = compile_flags()
    for name, source in (("before", baseline_source), ("after", root /
                         "ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp")):
        subprocess.run(flags + ["-c", str(source), "-o", str(output / (name + ".o"))],
                       check=True, timeout=90)
        subprocess.run(flags + [str(output / "synthetic.cpp"), str(output / (name + ".o")),
                                "-o", str(output / name)], check=True, timeout=90)
    subprocess.run(flags + [str(test_root / "test_plane_cluster_incremental.cpp"),
                            str(output / "after.o"), "-lgtest_main", "-lgtest", "-pthread",
                            "-o", str(output / "tests")], check=True, timeout=90)
    with (output / "tests.log").open("w") as log:
        subprocess.run([str(output / "tests")], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=30)
    results = []
    for mode, size in (("steady", 8), ("dynamic", 8), ("birth", 8),
                       ("merge", 4), ("reject", 4), ("chain", 20)):
        traces = []
        for name in ("before", "after"):
            trace = output / (mode + "_" + name + ".trace")
            result = subprocess.run([str(output / name), mode, str(size), "50", str(trace)],
                                    check=True, timeout=30, capture_output=True, text=True)
            (output / (mode + "_" + name + ".log")).write_text(result.stdout + result.stderr)
            traces.append(trace.read_bytes())
        assert traces[0] == traces[1], mode
        results.append({"mode": mode, "size": size, "frames": 70, "is_full_output_equal": True})
    (output / "validation.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


def summarize(output):
    report = json.loads((output / "batch/report.json").read_text())
    assert report["status"] == "completed"
    assert all(record["cleanup_ok"] for record in report["records"])
    # runner保存ログの場所に依存しないケース別集計。
    results = []
    signatures = []
    final_outputs = []
    fingerprints = []
    for path in sorted((output / "batch").rglob("stdout.log")):
        rows = list(csv.DictReader(io.StringIO(path.read_text())))
        if not rows:
            continue
        signatures.append([{key: val for key, val in row.items()
                            if key not in ("cpu_ms", "wall_ms")} for row in rows])
        stderr = path.with_name("stderr.log").read_text().splitlines()
        fingerprints.append([line for line in stderr if line.startswith("OUTPUT,")])
        final_outputs.append([line for line in stderr if line.startswith('{"clusters":')])
        timings = [[float(v) for v in line.split(",")[1:]]
                   for line in stderr if line.startswith("PROFILE,")]
        result = {"case": str(path.parent.relative_to(output / "batch")), "frames": len(rows),
                  "cpu_ms": statistics.fmean(float(row["cpu_ms"]) for row in rows[50:]),
                  "wall_ms": statistics.fmean(float(row["wall_ms"]) for row in rows[50:]),
                  "nodes": statistics.fmean(float(row["nodes"]) for row in rows[50:])}
        cpu_values = sorted(float(row["cpu_ms"]) for row in rows[50:])
        result["p95_cpu_ms"] = cpu_values[int(len(cpu_values) * 0.95) - 1]
        result["max_cpu_ms"] = cpu_values[-1]
        if timings:
            assert len(timings) == len(rows)
            result["stages_ms"] = {stage: statistics.fmean(row[idx] for row in timings[50:])
                                   for idx, stage in enumerate(stages)}
            result["calls"] = {stage: statistics.fmean(row[idx + 10] for row in timings[50:])
                               for idx, stage in enumerate(stages)}
        prepare_rows = [[float(v) for v in line.split(",")[1:]]
                        for line in stderr if line.startswith("PREP,")]
        if prepare_rows:
            result["prepare_ms"] = {name: statistics.fmean(row[idx] for row in prepare_rows[50:])
                for idx, name in enumerate(("input", "csr", "spacing_normal", "ema", "seed"))}
        reused = [int(line.split(",")[2]) for line in stderr if line.startswith("CONNECTIVITY,")]
        if reused:
            result["num_reused_connectivity_clusters"] = statistics.fmean(reused[50:])
        results.append(result)
    assert len(results) == report["total"] and results, len(results)
    assert all(signature == signatures[0] for signature in signatures)
    assert all(value == final_outputs[0] and len(value) == 1 for value in final_outputs)
    has_fingerprints = any(fingerprints)
    if has_fingerprints:
        assert all(value == fingerprints[0] and len(value) == len(signatures[0]) for value in fingerprints)
    summary = {"is_aggregate_equal": True, "is_final_membership_equal": True,
               "is_full_output_equal": True if has_fingerprints else None, "results": results}
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


def summarize_phases(output, enable_visual_comparison=False, enable_logic_comparison=False):
    report = json.loads((output / "batch/report.json").read_text())
    assert report["status"] == "completed" and all(r["cleanup_ok"] for r in report["records"])
    signatures = {}
    core_signatures = {}
    results = []
    for record in report["records"]:
        directory = Path(record["log"]).parent
        rows = list(csv.DictReader((directory / "stdout.log").open()))
        assert len(rows) == 150
        lines = (directory / "stderr.log").read_text().splitlines()
        signatures[record["name"], record["trial"]] = [line for line in lines if line.startswith("OUTPUT,")]
        core_signatures[record["name"], record["trial"]] = [line for line in lines if line.startswith("CORE,")]
        quality = [[float(v) for v in line.split(",")[2:]] for line in lines if line.startswith("QUALITY,")]
        timings = [[float(v) for v in line.split(",")[1:]] for line in lines if line.startswith("PROFILE,")]
        values = sorted(float(r["cpu_ms"]) for r in rows[50:])
        item = {"case": record["name"], "trial": record["trial"],
                "cpu_ms": statistics.fmean(values), "p95_cpu_ms": values[94], "max_cpu_ms": values[-1],
                "clusters": statistics.fmean(float(r["clusters"]) for r in rows[50:]),
                "assigned": statistics.fmean(float(r["assigned"]) for r in rows[50:]),
                "released": statistics.fmean(float(r["released"]) for r in rows[50:]),
                "weighted_residual_ratio": sum(q[0] for q in quality[50:]) /
                    sum(float(r["assigned"]) for r in rows[50:]),
                "max_cluster_residual_ratio": max(q[1] for q in quality[50:])}
        if enable_visual_comparison:
            edges = [int(line.split(",")[2]) for line in lines if line.startswith("EDGES,")]
            assert len(edges) == 150
            item["num_edge_points"] = statistics.fmean(edges[50:])
            if record["name"].endswith("_0"):
                assert all(value == 0 for value in edges)
        if timings:
            item["stages_ms"] = {stage: statistics.fmean(row[idx] for row in timings[50:])
                                 for idx, stage in enumerate(stages)}
        results.append(item)
    for trial in range(1, report["repeats"] + 1):
        if enable_logic_comparison:
            assert signatures["baseline", trial] == signatures["off", trial]
            if ("precheck", trial) in signatures:
                assert signatures["baseline", trial] == signatures["precheck", trial]
                assert signatures["profile", trial] == signatures["both", trial]
            else:
                assert signatures["profile", trial] == signatures["temporal", trial]
        elif enable_visual_comparison:
            for phases in (4, 10):
                reference = core_signatures[f"baseline_{phases}_1", trial]
                assert len(reference) == 150
                for mode, edges in (("after", 0), ("after", 1), ("profile", 0)):
                    assert core_signatures[f"{mode}_{phases}_{edges}", trial] == reference
                assert signatures[f"baseline_{phases}_1", trial] == signatures[f"after_{phases}_1", trial]
        else:
            assert signatures["baseline_1", trial] == signatures["after_1", trial]
            assert signatures["profile_4", trial] == signatures["after_4", trial]
    filename = "logic_summary.json" if enable_logic_comparison else (
        "visual_summary.json" if enable_visual_comparison else "phase_summary.json")
    (output / filename).write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("prepare", "summarize", "run", "validate", "summarize-phases", "summarize-visual", "summarize-logic"))
    parser.add_argument("output", type=Path)
    parser.add_argument("--mode", choices=("baseline", "profile", "after"))
    parser.add_argument("--baseline-source", type=Path)
    parser.add_argument("--case-dir", type=Path)
    parser.add_argument("--phase-comparison", action="store_true")
    parser.add_argument("--visual-comparison", action="store_true")
    parser.add_argument("--logic-comparison", action="store_true")
    parser.add_argument("--direct-comparison", action="store_true")
    args = parser.parse_args()
    if args.action == "run":
        run(args.output.resolve(), args.mode, args.case_dir)
    elif args.action == "prepare":
        prepare(args.output.resolve(), args.baseline_source, args.phase_comparison, args.visual_comparison,
                args.logic_comparison, args.direct_comparison)
    elif args.action == "validate":
        if not args.baseline_source:
            parser.error("validate requires --baseline-source")
        validate(args.output.resolve(), args.baseline_source)
    elif args.action == "summarize-logic":
        summarize_phases(args.output.resolve(), enable_logic_comparison=True)
    elif args.action == "summarize-phases":
        summarize_phases(args.output.resolve())
    elif args.action == "summarize-visual":
        summarize_phases(args.output.resolve(), True)
    else:
        summarize(args.output.resolve())
