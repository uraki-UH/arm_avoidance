"""最終出力の小平面対の再照合。統合途中の順序とは独立したオフライン診断。"""

import argparse
import itertools
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import yaml


def fit_plane(points):
    center = points.mean(axis=0)
    centered = points - center
    values, vectors = np.linalg.eigh(centered.T @ centered / len(points))
    return center, vectors[:, 0], np.sqrt(np.maximum(values, 0.0))


def rms_to_plane(points, fit):
    return float(np.sqrt(np.mean(((points - fit[0]) @ fit[1]) ** 2)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--max-fragment-nodes", type=int, default=30)
    args = parser.parse_args()
    data = json.loads(args.snapshot.read_text())
    points = np.asarray(data["nodes"], dtype=float)
    clusters = data["clusters"]
    config = yaml.safe_load((Path(__file__).resolve().parents[2] /
        "ais_gng_cpu/src/ais_gng/config/plane_cluster_incremental.yaml").read_text())
    params = config["plane_cluster_incremental_node"]["ros__parameters"]
    edges = np.asarray(data["edges"], dtype=int).reshape(-1, 2)
    edges = edges[(edges[:, 0] != edges[:, 1]) & np.all(edges < len(points), axis=1)]
    lengths = np.linalg.norm(points[edges[:, 0]] - points[edges[:, 1]], axis=1)
    local_lengths = [[] for _ in points]
    owner = np.full(len(points), -1, dtype=int)
    fits = []
    for idx, cluster in enumerate(clusters):
        owner[cluster["nodes"]] = idx
        fits.append(fit_plane(points[cluster["nodes"]]))
    direct = defaultdict(lambda: [[], []])
    unassigned_contacts = defaultdict(lambda: defaultdict(list))
    for (first, second), length in zip(edges, lengths):
        if length > 1.0e-9:
            local_lengths[first].append(length)
            local_lengths[second].append(length)
        first_owner, second_owner = owner[first], owner[second]
        if first_owner >= 0 and second_owner >= 0 and first_owner != second_owner:
            if first_owner > second_owner:
                first, second = second, first
                first_owner, second_owner = second_owner, first_owner
            direct[first_owner, second_owner][0].append(first)
            direct[first_owner, second_owner][1].append(second)
        elif first_owner >= 0 and second_owner < 0:
            unassigned_contacts[second][first_owner].append(first)
        elif second_owner >= 0 and first_owner < 0:
            unassigned_contacts[first][second_owner].append(second)
    spacings = np.array([sorted(lengths)[(len(lengths) - 1) // 2] if lengths else 1.0e-9
                         for lengths in local_lengths])
    for cluster in clusters:
        assert np.isclose(spacings[cluster["nodes"]].mean(), cluster["spacing"], rtol=1.0e-5)

    def effective_spacing(value):
        cap = params["max_effective_spacing"]
        return min(value, cap) if cap > 0 else value

    def evaluate_pair(pair, contacts):
        first, second = pair
        sides = [clusters[idx] for idx in pair]
        side_points = [points[side["nodes"]] for side in sides]
        union_points = np.concatenate(side_points)
        union_fit = fit_plane(union_points)
        side_spacings = [effective_spacing(side["spacing"]) for side in sides]
        union_spacing = sum(side["spacing"] * len(side["nodes"]) for side in sides) / len(union_points)
        values = union_fit[2]
        growth_th = max(params["merge_residual_growth_min_th"], params["merge_residual_growth_ratio"] *
                        max(fits[idx][2][0] / spacing for idx, spacing in zip(pair, side_spacings)))
        ratios = {
            "absolute_residual": float(values[0] / effective_spacing(union_spacing) /
                                       params["max_normalized_cluster_residual"]),
            "residual_growth": float(values[0] / effective_spacing(union_spacing) / growth_th),
            "side_residual": max(rms_to_plane(side, union_fit) / spacing
                                 for side, spacing in zip(side_points, side_spacings)) /
                                 params["max_merge_side_residual_ratio"],
            "contact_residual": max(rms_to_plane(points[indices], fits[other]) /
                                    effective_spacing(spacings[indices].mean())
                                    for indices, other in zip(contacts, (second, first))) /
                                    params["max_merge_side_residual_ratio"],
        }
        failed = []
        if not (values[1] / max(values[2], 1.0e-9) >= params["merge_min_planarity"] or
                values[1] >= params["min_plane_width_ratio"] * union_spacing):
            failed.append("plane_extent")
        failed.extend(key for key, value in ratios.items() if value > 1.0)
        return {"ids": [side["id"] for side in sides], "sizes": [len(side["nodes"]) for side in sides],
                "failed_geometry": failed, "ratios_to_limit": {key: round(value, 4) for key, value in ratios.items()}}

    small = {idx for idx, cluster in enumerate(clusters)
             if len(cluster["nodes"]) <= args.max_fragment_nodes}
    pairs = []
    for pair, contacts in direct.items():
        if not small.intersection(pair):
            continue
        value = evaluate_pair(pair, contacts)
        value["num_edges"] = len(contacts[0])
        contact_lengths = np.linalg.norm(points[contacts[0]] - points[contacts[1]], axis=1)
        value["max_contact_length_ratio"] = float(np.max(
            contact_lengths / np.minimum(spacings[contacts[0]], spacings[contacts[1]])))
        ratios = value["ratios_to_limit"]
        # 最終出力だけから判断可能な救済条件。連続フレーム数は履歴不足のため対象外。
        value["has_fragment_geometry"] = bool(
            params.get("enable_fragment_merge", False) and params["merge_connection_requirement"] == 2 and
            value["num_edges"] == 1 and not value["failed_geometry"] and
            min(value["sizes"]) <= params["max_fragment_nodes"] and
            value["max_contact_length_ratio"] <= params["max_fragment_edge_ratio_th"] and
            max(ratios["absolute_residual"] * params["max_normalized_cluster_residual"],
                max(ratios["side_residual"], ratios["contact_residual"]) * params["max_merge_side_residual_ratio"])
            <= params["max_fragment_residual_ratio_th"])
        pairs.append(value)

    # 未所属点を1個挟む接続の幾何確認。法線・時間安定性未評価のため統合可否とは区別。
    bridge_candidates = []
    for node_idx, contacts in unassigned_contacts.items():
        for pair in itertools.combinations(sorted(contacts), 2):
            if not small.intersection(pair) or pair in direct:
                continue
            value = evaluate_pair(pair, [contacts[pair[0]], contacts[pair[1]]])
            bridge_ratio = max(rms_to_plane(points[[node_idx]], fits[idx]) /
                               effective_spacing(spacings[node_idx]) for idx in pair)
            if not value["failed_geometry"] and bridge_ratio <= params["growth_residual_ratio"]:
                value["bridge_node_idx"] = int(node_idx)
                bridge_candidates.append(value)
    adjacent_small = set().union(*(set(pair).intersection(small) for pair in direct))
    result = {
        "max_fragment_nodes": args.max_fragment_nodes,
        "num_clusters": len(clusters), "num_small_clusters": len(small),
        "num_small_without_direct_plane_contact": len(small - adjacent_small),
        "num_pairs_with_small_cluster": len(pairs),
        "geometry_failures_overlapping": dict(Counter(reason for pair in pairs for reason in pair["failed_geometry"])),
        "geometry_pass_but_insufficient_edges": [value for value in pairs if not value["failed_geometry"] and
                                                value["num_edges"] < params["merge_connection_requirement"]],
        "all_conditions_pass_at_final_output": [value for value in pairs if not value["failed_geometry"] and
                                               value["num_edges"] >= params["merge_connection_requirement"]],
        "one_unassigned_node_bridge_candidates": bridge_candidates,
        "pairs": pairs,
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
