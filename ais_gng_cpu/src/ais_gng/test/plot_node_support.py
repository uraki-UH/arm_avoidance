"""支持領域実験のCSVからの図・被覆監査・自己完結HTML生成。"""
import argparse
import base64
import json
import os
import re
import struct
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/node_support_matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Ellipse
import numpy as np
from scipy.spatial import cKDTree
from scipy.signal import lfilter
from numpy.lib.recfunctions import append_fields


def read_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def ratio(a, b):
    return np.divide(a, b, out=np.full_like(a, np.nan, dtype=float), where=b > 0)


def read_inputs(path):
    frames = []
    with open(path, "rb") as stream:
        while header := stream.read(12):
            stamp, point_num = struct.unpack("<dI", header)
            points = np.frombuffer(stream.read(12 * point_num), dtype="<f4").reshape(-1, 3).copy()
            frames.append((stamp, points))
    return frames


def matrices(rows):
    return np.column_stack([rows[f"c{row}{column}"] for row in range(3) for column in range(3)]).reshape(-1, 3, 3)


def coverage(points, centers, covariance, scales):
    # 包絡球による候補検索後の厳密な楕円体内判定。全入力点を対象とする和集合。
    covered = np.zeros(len(points), dtype=bool)
    tree = cKDTree(points)
    for center, matrix, scale in zip(centers, covariance, scales):
        values, vectors = np.linalg.eigh(matrix)
        values = np.maximum(values, 1e-12)
        indices = np.asarray(tree.query_ball_point(center, scale * np.sqrt(values[-1])), dtype=int)
        if not len(indices):
            continue
        local = (points[indices] - center) @ vectors
        covered[indices] |= np.sum(local * local / values, axis=1) <= scale * scale + 1e-9
    return covered


def residual_statistics(nodes, events, sample_alpha=0.01):
    # ノード世代・勝者順位ごとの入力列から、図に必要な平均とRMSだけの再計算。
    names = ["sample_num"] + [f"{prefix}_{axis}" for prefix in ("raw", "filtered") for axis in ("x", "y", "z", "rms")]
    nodes = append_fields(nodes, names, [np.zeros(len(nodes)) for _ in names], usemask=False)
    if not len(events):
        return nodes
    def node_keys(rows):
        return rows["generation"].astype(np.int64) * 131072 + rows["id"].astype(np.int64) * 2 + rows["rank"].astype(np.int64) - 1
    keys = node_keys(nodes)
    event_keys = node_keys(events)
    order = np.argsort(event_keys, kind="stable")
    starts = np.r_[0, np.flatnonzero(np.diff(event_keys[order])) + 1, len(order)]
    for start, end in zip(starts[:-1], starts[1:]):
        selected_rows = np.flatnonzero(keys == event_keys[order[start]])
        selected = events[order[start:end]]
        frames = nodes["frame"][selected_rows]
        has_finite_values = np.isfinite(np.column_stack([selected[name] for name in ("rx", "ry", "rz")])).all(axis=1)
        selected = selected[np.isin(selected["frame"], frames) & has_finite_values]
        if not len(selected):
            continue
        residual = np.column_stack([selected[f"r{axis}"] for axis in ("x", "y", "z")])
        values = np.column_stack([residual, np.sum(residual ** 2, axis=1)])
        filtered, _ = lfilter([sample_alpha], [1, -(1 - sample_alpha)], values,
                              axis=0, zi=(1 - sample_alpha) * values[:1])
        batch_frames, first, counts = np.unique(selected["frame"], return_index=True, return_counts=True)
        batch = np.add.reduceat(values, first, axis=0) / counts[:, None]
        batch_rows = selected_rows[np.searchsorted(frames, batch_frames)]
        nodes["sample_num"][batch_rows] = counts
        last = np.searchsorted(selected["frame"], frames, side="right") - 1
        has_sample = last >= 0
        for axis_idx, axis in enumerate(("x", "y", "z", "rms")):
            raw = batch[:, axis_idx]
            history = filtered[last[has_sample], axis_idx]
            if axis == "rms":
                raw, history = np.sqrt(np.maximum(raw, 0)), np.sqrt(np.maximum(history, 0))
            nodes[f"raw_{axis}"][batch_rows] = raw
            nodes[f"filtered_{axis}"][selected_rows[has_sample]] = history
    return nodes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    args = parser.parse_args()
    directory = Path(args.directory)
    font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font_path.exists():
        plt.rcParams["font.family"] = FontProperties(fname=str(font_path)).get_name()
        from matplotlib import font_manager
        font_manager.fontManager.addfont(str(font_path))
    plt.rcParams.update({"axes.grid": True, "grid.alpha": 0.2, "font.size": 10})
    figures = []

    def save(figure, name):
        figure.tight_layout()
        figure.savefig(directory / f"{name}.png", dpi=160, bbox_inches="tight")
        figure.savefig(directory / f"{name}.pdf", bbox_inches="tight")
        plt.close(figure)
        figures.append(name)

    baseline = [read_csv(directory / f"baseline_{idx}.csv") for idx in range(5)]
    support = [read_csv(directory / f"support_{idx}.csv") for idx in range(5)]
    modes = {name: read_csv(directory / f"{name}.csv") for name in ("capture2", "cumulative", "first", "diagnostic")}
    nodes = read_csv(directory / "diagnostic/nodes.csv")
    events = read_csv(directory / "diagnostic/events.csv")
    if "sample_num" not in nodes.dtype.names:
        nodes = residual_statistics(nodes, events)
    legacy = read_csv(directory / "diagnostic/legacy_nodes.csv")
    inputs = read_inputs(directory / "input.bin")
    summary = {"warmup_frames": 30, "frames": len(inputs), "repeats": 5, "timing": {}, "coverage": []}
    for name, runs in (("baseline", baseline), ("support", support)):
        data = np.concatenate([run[30:] for run in runs])
        summary["timing"][name] = {column: {"mean": float(np.mean(data[column])),
            "p50": float(np.quantile(data[column], 0.5)), "p95": float(np.quantile(data[column], 0.95)),
            "p99": float(np.quantile(data[column], 0.99))} for column in ("gng_ms", "legacy_ms", "support_ms", "total_ms")}
    summary["is_legacy_identical"] = bool(all(np.array_equal(baseline[0]["checksum"], run["checksum"])
        and np.array_equal(baseline[0]["nodes"], run["nodes"]) for run in baseline + support + list(modes.values())))
    summary["diagnostic_mean_ms"] = float(np.mean(modes["diagnostic"]["diagnostic_ms"][30:]))
    figure, axes = plt.subplots(2, 2, figsize=(13, 8))
    axes[0, 0].plot(baseline[0]["stamp_sec"], baseline[0]["total_ms"], label="従来")
    axes[0, 0].plot(support[0]["stamp_sec"], support[0]["total_ms"], label="支持領域あり")
    axes[0, 0].set(title="実点群入力の処理時間（代表1回）", xlabel="時間 [s]", ylabel="時間 [ms]")
    axes[0, 0].legend()
    axes[0, 1].boxplot([np.concatenate([run[30:]["total_ms"] for run in runs]) for runs in (baseline, support)], labels=["従来", "支持領域あり"], showfliers=False)
    axes[0, 1].set(title="交互5回・最初の30フレーム除外", ylabel="時間 [ms]")
    axes[1, 0].plot(support[0]["stamp_sec"], support[0]["support_ms"])
    axes[1, 0].set(title="楕円体の描画用変換時間（記録OFF）", xlabel="時間 [s]", ylabel="時間 [ms]")
    axes[1, 1].plot(support[0]["stamp_sec"], support[0]["nodes"])
    axes[1, 1].set(title="GNGノード数", xlabel="時間 [s]", ylabel="ノード数")
    save(figure, "overview")

    # 世代付きIDの長期観測ノードを選択し、ID再利用による時系列の連結を回避。
    keys = nodes["generation"].astype(np.int64) * 65536 + nodes["id"].astype(np.int64)
    valid = (nodes["rank"] == 1) & (nodes["sample_num"] > 0)
    unique, counts = np.unique(keys[valid], return_counts=True)
    key = unique[np.argmax(counts)]
    summary["tracked_node"] = {"id": int(key % 65536), "generation": int(key // 65536)}
    figure, axes = plt.subplots(4, 2, figsize=(14, 12), sharex=True)
    for rank in (1, 2):
        selected = nodes[(keys == key) & (nodes["rank"] == rank) & (nodes["sample_num"] > 0)]
        sample_positions = np.cumsum(selected["sample_num"])
        for row, name in enumerate(("x", "y", "z", "rms")):
            axis = axes[row, rank - 1]
            axis.plot(sample_positions, selected[f"raw_{name}"] * 1000, alpha=0.5, label="フレーム内残差統計")
            axis.plot(sample_positions, selected[f"filtered_{name}"] * 1000, label="入力更新率0.01")
            axis.set(ylabel=f"残差 {name} [mm]", title=f"第{rank}勝者 / {name}")
            axis.legend(fontsize=8)
    axes[3, 0].set_xlabel("累積第1勝者入力数"); axes[3, 1].set_xlabel("累積第2勝者入力数")
    figure.suptitle(f"ノード {int(key % 65536)}・生成フレーム {int(key // 65536)} の入力残差", y=1.005)
    save(figure, "residual_timeseries")

    figure, axes = plt.subplots(1, 3, figsize=(15, 4))
    for rank in (1, 2):
        selected = events[events["rank"] == rank]
        lengths = np.sqrt(selected["rx"] ** 2 + selected["ry"] ** 2 + selected["rz"] ** 2)
        axes[0].hist(lengths * 1000, bins=np.linspace(0, 150, 100), density=True, histtype="step", label=f"第{rank}勝者")
        mean_squared = np.bincount(selected["frame"].astype(int), weights=lengths ** 2)
        count = np.bincount(selected["frame"].astype(int))
        axes[1].plot(np.arange(len(count)), 1000 * np.sqrt(mean_squared / count), label=f"第{rank}勝者")
        subset = selected[(selected["id"] == key % 65536) & (selected["generation"] == key // 65536)][::4]
        axes[2].scatter(subset["rx"] * 1000, subset["ry"] * 1000, s=2, alpha=0.15, label=f"第{rank}勝者")
        summary[f"rank{rank}_residual_mm"] = {"rms": float(1000 * np.sqrt(np.mean(lengths ** 2))), "p95": float(1000 * np.quantile(lengths, 0.95))}
    axes[0].set(title="全学習イベントの残差長分布", xlabel="残差長 [mm]", ylabel="密度")
    axes[1].set(title="全ノードの残差RMS", xlabel="フレーム", ylabel="RMS [mm]")
    axes[2].set(title="追跡ノードの残差XY", xlabel="X [mm]", ylabel="Y [mm]")
    for axis in axes:
        axis.legend()
    save(figure, "residual_distribution")
    del events

    figure, axes = plt.subplots(1, 3, figsize=(15, 4))
    data = modes["diagnostic"]
    axes[0].plot(data["stamp_sec"], data["mean_scale"])
    axes[0].set(title="支持領域の平均半軸倍率", xlabel="時間 [s]", ylabel="倍率")
    axes[1].plot(data["stamp_sec"], data["nodes"])
    axes[1].set(title="GNGノード数", xlabel="時間 [s]", ylabel="ノード数")
    selected = nodes[(keys == key) & (nodes["rank"] == 1)]
    for axis_idx in range(3):
        axes[2].plot(selected["stamp_sec"], selected[f"axis{axis_idx}"] * selected["scale"] * 1000, label=f"半軸{axis_idx}")
    axes[2].set(title="追跡ノードの支持領域半軸", xlabel="時間 [s]", ylabel="長さ [mm]")
    axes[2].legend()
    save(figure, "support_scale")

    volume_series = []
    for frame in range(len(inputs)):
        selected = nodes[(nodes["frame"] == frame) & (nodes["rank"] == 1) & (nodes["has_support"] > 0)]
        old = legacy[legacy["frame"] == frame]
        volumes = []
        for rows, scales in ((old, np.full(len(old), 2.0)), (selected, selected["scale"])):
            volumes.append(float(np.sum(np.sqrt(np.maximum(np.linalg.det(matrices(rows)), 0)) * scales ** 3) * 4 * np.pi / 3))
        volume_series.append(volumes)
    volume_series = np.asarray(volume_series)
    figure, axis = plt.subplots(figsize=(10, 4))
    for idx, label in enumerate(("従来・第1勝者2倍", "両勝者・固定2倍")):
        axis.plot(np.arange(len(inputs)), volume_series[:, idx], label=label)
    axis.set(title="広がりすぎの確認：楕円体体積の合計（重複部分も加算）", xlabel="フレーム", ylabel="合計体積 [m³]")
    axis.legend()
    save(figure, "support_volume")
    summary["last_volume_sum_ratio"] = float(volume_series[-1, 1] / volume_series[-1, 0])
    summary["ros_reference"] = {}
    if (directory / "smoke/results.json").exists():
        for name in ("baseline", "support"):
            values = []
            for line in (directory / f"smoke/{name}.log").read_text().splitlines():
                match = re.search(r"processing=([0-9.]+) ms .*?gng=([0-9.]+) convert=([0-9.]+).*?nodes=(\d+)", line)
                if match:
                    values.append([float(value) for value in match.groups()])
            if len(values) > 30:
                data = np.asarray(values)[30:]
                summary["ros_reference"][name] = {"frames": len(values), "mean_processing_ms": float(data[:, 0].mean()),
                    "p95_processing_ms": float(np.quantile(data[:, 0], 0.95)), "mean_node_num": float(data[:, 3].mean())}
    snapshot_names = []
    for frame in (59, 119, 179, len(inputs) - 1):
        points = inputs[frame][1]
        selected = nodes[(nodes["frame"] == frame) & (nodes["rank"] == 1) & (nodes["has_support"] > 0)]
        old = legacy[legacy["frame"] == frame]
        setups = [(old, np.full(len(old), 2.0), "従来の第1勝者・2倍"),
                  (selected, selected["scale"], "両勝者・固定2倍")]
        figure, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
        record = {"frame": frame, "stamp_sec": inputs[frame][0]}
        for setup_idx, (rows, scales, title) in enumerate(setups):
            centers = np.column_stack([rows[name] for name in ("px", "py", "pz")])
            covariance = matrices(rows)
            covered = coverage(points, centers, covariance, scales)
            record[("legacy", "pooled_fixed")[setup_idx]] = float(np.mean(covered))
            axis = axes[setup_idx]
            axis.scatter(points[::4, 0], points[::4, 1], c=np.where(covered[::4], "#718096", "#e53e3e"), s=1, alpha=0.5)
            for center, matrix, scale in zip(centers, covariance, scales):
                values, vectors = np.linalg.eigh(matrix[:2, :2])
                values = np.maximum(values, 1e-12)
                angle = np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1]))
                axis.add_patch(Ellipse(center[:2], 2 * scale * np.sqrt(values[1]), 2 * scale * np.sqrt(values[0]),
                    angle=angle, fill=False, linewidth=0.45, alpha=0.5, color="#3182ce"))
            axis.set(title=f"{title}\n全入力点の3D被覆率 {100 * covered.mean():.1f}%", xlabel="X [m]", ylabel="Y [m]")
            axis.set_aspect("equal")
        figure.suptitle(f"フレーム {frame}：青は楕円体のXY投影、赤は3Dで未被覆の入力点", y=1.01)
        name = f"coverage_{frame}"
        save(figure, name)
        snapshot_names.append(name)
        summary["coverage"].append(record)
    (directory / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    images = {name: "data:image/png;base64," + base64.b64encode((directory / f"{name}.png").read_bytes()).decode() for name in figures}
    before = summary["timing"]["baseline"]["total_ms"]["mean"]
    after = summary["timing"]["support"]["total_ms"]["mean"]
    html = f'''<!doctype html><html lang="ja"><meta charset="utf-8"><title>GNG支持領域の比較</title>
<style>body{{font-family:sans-serif;max-width:1250px;margin:35px auto;padding:0 20px;background:#f8fafc;color:#182536}}img{{width:100%;background:white}}p{{line-height:1.8}}section{{margin:32px 0}}input{{width:80%}}</style>
<h1>第1・第2勝者による支持領域の検証</h1>
<p>実深度点群240フレーム、各20,000点、学習5,000回/フレーム。固定乱数列による交互5回比較。最初の30フレームを時間集計から除外。ボクセル幅0.02 m、ノード探索グリッド幅0.5 m。通常のROS配信・分類・平面処理を含まないGNG API経路の測定。</p>
<p>平均処理時間：従来 {before:.3f} ms → 追加後 {after:.3f} ms（{after-before:+.3f} ms、{100*(after/before-1):+.1f}%）。従来統計の一致：{summary['is_legacy_identical']}。</p>
<p>全イベント・ノードCSV記録は別計測で平均 {summary['diagnostic_mean_ms']:.2f} ms/フレーム。残差グラフの平均・RMSは記録後にPythonで計算。通常のROSノードはイベント記録を無効化。統計更新はGNG内部、描画用の軸・姿勢への変換は出力側で実施。</p>
<p>第1勝者の共分散を保持し、両勝者の入力座標を第1:第2=1:0.5で更新。入力サンプル当たり更新率0.01。入力のないノードの統計は保持し、ノード削除・世代変更時に破棄。半軸倍率は固定2倍。被覆に応じた倍率調整なし。</p>
<p>全入力点の楕円体和集合被覆は、独立の空間索引を使ってオフラインで評価。実行時の被覆計算なし。面全体や未観測領域の被覆保証、誤被覆率の評価ではありません。</p>'''
    if summary["ros_reference"]:
        old = summary["ros_reference"]["baseline"]
        new = summary["ros_reference"]["support"]
        html += f'<p>ROS配信を含む参考測定（各80フレーム・各1回、先頭30除外）：平均 {old["mean_processing_ms"]:.2f} → {new["mean_processing_ms"]:.2f} ms。平均ノード数 {old["mean_node_num"]:.0f} / {new["mean_node_num"]:.0f}。ノード列が完全一致する比較ではなく、Python観測側のペーシングも含むため周期Hzは性能値として不使用。検証ノードは停止済み。</p>'
    html += f'<p>最終フレームの楕円体体積合計は従来の {summary["last_volume_sum_ratio"]:.2f} 倍。被覆改善と過剰な広がりのトレードオフがあり、平面法線・曲率の推定用共分散への置換は行っていません。</p>'
    for name in figures[:5]:
        html += f'<section><img src="{images[name]}" alt="{name}"></section>'
    html += '<section><h2>被覆の時間変化</h2><p>スライダーでフレームを切り替えます。表示はXY投影、被覆判定は3Dです。</p><input id="frame" type="range" min="0" max="3" value="3"><img id="snapshot"></section>'
    html += '<script>const snapshots=' + json.dumps([images[name] for name in snapshot_names]) + ';const slider=document.getElementById("frame");const update=()=>document.getElementById("snapshot").src=snapshots[Number(slider.value)];slider.addEventListener("input",update);update();</script></html>'
    (directory / "report.html").write_text(html)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
