#!/usr/bin/env python3
"""物体点群とGNGの平面内相対変位ヒストグラム出力。"""

import argparse
import gzip
import json
import math
import struct
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DATASET_NAMES = ("basket", "basket_occ", "cup", "cup_occ")


def read_lzf(data, expected_size):
    output = bytearray()
    source_idx = 0
    while source_idx < len(data):
        control = data[source_idx]
        source_idx += 1
        if control < 32:
            literal_size = control + 1
            literal_end = source_idx + literal_size
            if literal_end > len(data):
                raise ValueError("LZFのリテラル長が不正です")
            output.extend(data[source_idx:literal_end])
            source_idx = literal_end
            continue

        copy_size = control >> 5
        copy_idx = len(output) - ((control & 0x1F) << 8) - 1
        if copy_size == 7:
            if source_idx >= len(data):
                raise ValueError("LZFの複写長が不正です")
            copy_size += data[source_idx]
            source_idx += 1
        if source_idx >= len(data):
            raise ValueError("LZFの複写参照が不正です")
        copy_idx -= data[source_idx]
        source_idx += 1
        copy_size += 2
        if copy_idx < 0:
            raise ValueError("LZFの複写開始位置が不正です")
        for _ in range(copy_size):
            if copy_idx >= len(output):
                raise ValueError("LZFの複写範囲が不正です")
            output.append(output[copy_idx])
            copy_idx += 1

    if len(output) != expected_size:
        raise ValueError("LZF展開サイズがPCDヘッダと一致しません")
    return bytes(output)


def read_pcd_points(path):
    with path.open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCDヘッダを読み取れません: {path}")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key.lower()] = values
            if key.lower() == "data":
                break
        payload = stream.read()

    if header.get("data") != ["binary_compressed"]:
        raise ValueError(f"binary_compressed以外のPCDは未対応です: {path}")
    field_names = header["fields"]
    field_sizes = [int(value) for value in header["size"]]
    field_types = header["type"]
    field_counts = [int(value) for value in header.get("count", ["1"] * len(field_names))]
    point_num = int(header["points"][0])
    compressed_size, uncompressed_size = struct.unpack_from("<II", payload, 0)
    compressed = payload[8:8 + compressed_size]
    if len(compressed) != compressed_size:
        raise ValueError(f"PCD圧縮データが途中で終わっています: {path}")
    raw = read_lzf(compressed, uncompressed_size)

    columns = {}
    offset = 0
    for field_name, field_size, field_type, field_count in zip(
            field_names, field_sizes, field_types, field_counts):
        value_num = point_num * field_count
        byte_num = value_num * field_size
        field_data = raw[offset:offset + byte_num]
        if len(field_data) != byte_num:
            raise ValueError(f"PCDフィールドが途中で終わっています: {path}")
        if field_type == "F" and field_size == 4:
            values = np.frombuffer(field_data, dtype="<f4", count=value_num)
        elif field_type == "F" and field_size == 8:
            values = np.frombuffer(field_data, dtype="<f8", count=value_num)
        else:
            offset += byte_num
            continue
        columns[field_name] = values.reshape(point_num, field_count)
        offset += byte_num

    if not all(name in columns for name in ("x", "y", "z")):
        raise ValueError(f"PCDにx, y, zがありません: {path}")
    points = np.column_stack((columns["x"][:, 0], columns["y"][:, 0], columns["z"][:, 0]))
    return points[np.isfinite(points).all(axis=1)]


def read_template(path):
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        dataset = json.load(stream)
    gng = dataset["gng"]
    points = np.asarray(
        [[node["x"], node["y"], node["z"]] for node in gng["nodes"]], dtype=np.float64)
    edges = [tuple(edge) for edge in gng["edges"]]
    plane_node_ids = set()
    for cluster in gng.get("plane_clusters", []):
        plane_node_ids.update(cluster.get("idx", []))
    return points, edges, plane_node_ids


def sample_points(points, voxel_size, max_points):
    lower = np.min(points, axis=0)
    voxel_ids = np.floor((points - lower) / voxel_size).astype(np.int64)
    _, unique_ids = np.unique(voxel_ids, axis=0, return_index=True)
    selected = points[np.sort(unique_ids)]
    if len(selected) <= max_points:
        return selected

    selected_indices = [int(np.argmin(np.sum((selected - np.mean(selected, axis=0)) ** 2, axis=1)))]
    min_dist_sq = np.full(len(selected), np.inf)
    while len(selected_indices) < max_points:
        latest = selected[selected_indices[-1]]
        min_dist_sq = np.minimum(min_dist_sq, np.sum((selected - latest) ** 2, axis=1))
        selected_indices.append(int(np.argmax(min_dist_sq)))
    return selected[np.asarray(selected_indices)]


def make_point_pairs(points):
    delta = points[None, :, :] - points[:, None, :]
    point_num = len(points)
    mask = ~np.eye(point_num, dtype=bool)
    return delta[mask], np.ones(np.count_nonzero(mask), dtype=np.float64)


def make_graph_pairs(points, edges, allowed_ids):
    adjacency = {idx: set() for idx in allowed_ids}
    for first, second in edges:
        if first in adjacency and second in adjacency:
            adjacency[first].add(second)
            adjacency[second].add(first)

    pair_weights = {}
    for first, neighbors in adjacency.items():
        for second in neighbors:
            pair_weights[(first, second)] = 1.0
        for middle in neighbors:
            for second in adjacency[middle]:
                if second != first and second not in neighbors:
                    pair_weights[(first, second)] = max(pair_weights.get((first, second), 0.0), 0.5)

    if not pair_weights:
        return np.empty((0, 3), dtype=np.float64), np.empty(0, dtype=np.float64)
    pairs = np.asarray([points[second] - points[first] for first, second in pair_weights], dtype=np.float64)
    weights = np.asarray(list(pair_weights.values()), dtype=np.float64)
    return pairs, weights


def make_histogram(pairs, weights, radius_edges, angle_edges):
    if len(pairs) == 0:
        return np.zeros((len(radius_edges) - 1, len(angle_edges) - 1), dtype=np.float64)
    radius = np.linalg.norm(pairs[:, :2], axis=1)
    angle = np.arctan2(pairs[:, 1], pairs[:, 0])
    valid = (radius > 1e-6) & (radius < radius_edges[-1])
    histogram, _, _ = np.histogram2d(
        radius[valid], angle[valid], bins=(radius_edges, angle_edges), weights=weights[valid])
    total = np.sum(histogram)
    return histogram / total if total > 0.0 else histogram


def shifted_intersection(template_histogram, observed_histogram):
    scores = []
    for shift in range(template_histogram.shape[1]):
        shifted = np.roll(template_histogram, shift, axis=1)
        scores.append(float(np.minimum(shifted, observed_histogram).sum()))
    return np.asarray(scores)


def plot_histogram(ax, histogram, title, pair_num, nonzero_num):
    shown = np.sqrt(histogram / max(float(np.max(histogram)), 1e-12))
    image = ax.imshow(
        shown,
        origin="lower",
        aspect="auto",
        extent=(-180.0, 180.0, 0.0, 220.0),
        cmap="magma",
        vmin=0.0,
        vmax=1.0,
    )
    ax.set_title(f"{title}\npairs={pair_num}, bins={nonzero_num}")
    ax.set_xlabel("planar displacement angle [deg]")
    ax.set_ylabel("planar distance [mm]")
    return image


def save_histogram_figure(records, output_path):
    figure, axes = plt.subplots(len(records), 4, figsize=(21.0, 11.5), constrained_layout=True)
    for row, record in enumerate(records):
        histograms = (
            (record["point_histogram"], "Point cloud"),
            (record["graph_histogram"], "GNG paths: all nodes"),
            (record["nonplane_histogram"], "GNG paths: nonplane"),
            (record["nonplane_node_pair_histogram"], "GNG nodes: nonplane all pairs"),
        )
        for column, (histogram, title) in enumerate(histograms):
            image = plot_histogram(
                axes[row, column], histogram, f"{record['name']}: {title}",
                record["pair_counts"][column], int(np.count_nonzero(histogram)))
            figure.colorbar(image, ax=axes[row, column], fraction=0.046, pad=0.04, label="sqrt(normalized bin weight)")
    figure.suptitle("Translation-invariant planar displacement histograms", fontsize=15)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def save_comparison_figure(records, output_path, angle_bins):
    basket = records[0]
    yaw_deg = np.arange(angle_bins, dtype=np.float64) * 360.0 / angle_bins
    figure, axes = plt.subplots(4, 1, figsize=(12.0, 12.5), sharex=True, constrained_layout=True)
    modes = (
        ("point_histogram", "Point cloud histogram overlap"),
        ("graph_histogram", "All-node GNG histogram overlap"),
        ("nonplane_histogram", "Nonplane GNG histogram overlap"),
        ("nonplane_node_pair_histogram", "Nonplane GNG node-pair histogram overlap"),
    )
    summary = {}
    for axis, (key, title) in zip(axes, modes):
        axis.set_title(title + " against basket template")
        axis.set_ylabel("histogram intersection")
        for record in records:
            scores = shifted_intersection(basket[key], record[key])
            best_idx = int(np.argmax(scores))
            best_yaw_deg = float(yaw_deg[best_idx])
            axis.plot(yaw_deg, scores, label=f"{record['name']}: max={scores[best_idx]:.3f} at {best_yaw_deg:.0f} deg")
            summary.setdefault(record["name"], {})[key] = {
                "max_overlap": float(scores[best_idx]),
                "best_yaw_deg": best_yaw_deg,
            }
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("yaw shift of basket template [deg]")
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return summary


def make_record(dataset_dir, name, radius_edges, angle_edges, voxel_size, max_points):
    source_points = read_pcd_points(dataset_dir / f"{name}_source.pcd")
    sampled_points = sample_points(source_points, voxel_size, max_points)
    graph_points, graph_edges, plane_node_ids = read_template(dataset_dir / f"{name}_gng_template.json.gz")
    point_pairs, point_weights = make_point_pairs(sampled_points)
    all_ids = set(range(len(graph_points)))
    graph_pairs, graph_weights = make_graph_pairs(graph_points, graph_edges, all_ids)
    nonplane_ids = all_ids - plane_node_ids
    nonplane_pairs, nonplane_weights = make_graph_pairs(graph_points, graph_edges, nonplane_ids)
    nonplane_node_pairs, nonplane_node_weights = make_point_pairs(
        graph_points[np.asarray(sorted(nonplane_ids))])
    return {
        "name": name,
        "source_point_num": int(len(source_points)),
        "sampled_point_num": int(len(sampled_points)),
        "graph_node_num": int(len(graph_points)),
        "nonplane_node_num": int(len(nonplane_ids)),
        "pair_counts": (
            int(len(point_pairs)), int(len(graph_pairs)), int(len(nonplane_pairs)),
            int(len(nonplane_node_pairs))),
        "point_histogram": make_histogram(point_pairs, point_weights, radius_edges, angle_edges),
        "graph_histogram": make_histogram(graph_pairs, graph_weights, radius_edges, angle_edges),
        "nonplane_histogram": make_histogram(nonplane_pairs, nonplane_weights, radius_edges, angle_edges),
        "nonplane_node_pair_histogram": make_histogram(
            nonplane_node_pairs, nonplane_node_weights, radius_edges, angle_edges),
    }


def main():
    parser = argparse.ArgumentParser(description="平面内相対変位ヒストグラムの比較")
    parser.add_argument("--dataset-dir", type=Path, default=Path("/home/fuzzrobo/datasets"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--voxel-size", type=float, default=0.003)
    parser.add_argument("--max-points", type=int, default=360)
    parser.add_argument("--radius-bin-num", type=int, default=22)
    parser.add_argument("--angle-bin-num", type=int, default=36)
    args = parser.parse_args()

    if args.voxel_size <= 0.0 or args.max_points < 2:
        raise ValueError("voxel_sizeとmax_pointsの指定が不正です")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    radius_edges = np.linspace(0.0, 0.22, args.radius_bin_num + 1)
    angle_edges = np.linspace(-math.pi, math.pi, args.angle_bin_num + 1)
    records = [make_record(
        args.dataset_dir, name, radius_edges, angle_edges, args.voxel_size, args.max_points)
        for name in DATASET_NAMES]

    histogram_path = args.output_dir / "xy_displacement_histograms.png"
    comparison_path = args.output_dir / "basket_xy_displacement_overlap.png"
    summary_path = args.output_dir / "xy_displacement_summary.json"
    save_histogram_figure(records, histogram_path)
    summary = save_comparison_figure(records, comparison_path, args.angle_bin_num)
    summary["dataset_stats"] = {
        record["name"]: {
            "source_point_num": record["source_point_num"],
            "sampled_point_num": record["sampled_point_num"],
            "graph_node_num": record["graph_node_num"],
            "nonplane_node_num": record["nonplane_node_num"],
            "point_pair_num": record["pair_counts"][0],
            "graph_pair_num": record["pair_counts"][1],
            "nonplane_graph_pair_num": record["pair_counts"][2],
            "nonplane_node_pair_num": record["pair_counts"][3],
        }
        for record in records
    }
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"ヒストグラム: {histogram_path}")
    print(f"比較曲線: {comparison_path}")
    print(f"集計値: {summary_path}")
    for name in DATASET_NAMES:
        point = summary[name]["point_histogram"]
        all_graph = summary[name]["graph_histogram"]
        nonplane_graph = summary[name]["nonplane_histogram"]
        nonplane_node_pair = summary[name]["nonplane_node_pair_histogram"]
        print(
            f"{name}: point={point['max_overlap']:.3f}@{point['best_yaw_deg']:.0f}deg, "
            f"all_graph={all_graph['max_overlap']:.3f}@{all_graph['best_yaw_deg']:.0f}deg, "
            f"nonplane_graph={nonplane_graph['max_overlap']:.3f}@"
            f"{nonplane_graph['best_yaw_deg']:.0f}deg, "
            f"nonplane_node_pair={nonplane_node_pair['max_overlap']:.3f}@"
            f"{nonplane_node_pair['best_yaw_deg']:.0f}deg")


if __name__ == "__main__":
    main()
