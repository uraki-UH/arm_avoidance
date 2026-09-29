#!/usr/bin/env python3
"""検出器から独立した解析形状による正解ラベル付き点群の生成。"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from pathlib import Path
import random


schema_version = "1.0"
scene_names = ("coplanar_gap", "step_and_wall", "curved_object", "noisy_floor")
csv_fields = ("frame_idx", "point_idx", "x", "y", "z", "gt_label")
base_noise_std_m = 0.003


def scene_definitions(frames: int) -> list[dict]:
    """観測条件と、ラベル割り当ての根拠となる解析形状。"""
    occlusion = {
        "start_frame_idx": frames // 3,
        "end_frame_idx_exclusive": 2 * frames // 3,
    }
    scenes = [
        {
            "name": "coplanar_gap",
            "file": "coplanar_gap.csv",
            "description": "観測パッチ間の空白を持つ単一の水平床面",
            "planes": [
                {
                    "gt_label": 1,
                    "normal": [0.0, 0.0, 1.0],
                    "offset_m": 0.0,
                    "support": {
                        "patches": [
                            {"x_m": [-2.8, -0.15], "y_m": [-2.2, 2.2]},
                            {"x_m": [0.15, 2.8], "y_m": [-2.2, 2.2]},
                        ],
                        "gap_interpretation": "同一の連続床面に対する観測欠落",
                    },
                }
            ],
            "nonplane": [],
            "sampling": {
                "patch_weights": [0.65, 0.35],
                "occluded_patch_weights": [0.85, 0.15],
                "occlusion": dict(occlusion, right_patch_y_m=[-2.2, 0.0]),
                "noise_std_m_by_third": [0.0015, 0.003, 0.0045],
                "noise_phase": "min(2, frame_idx * 3 // frames)",
            },
        },
        {
            "name": "step_and_wall",
            "file": "step_and_wall.csv",
            "description": "床面、小面積の高さ0.10 mの段差上面、垂直壁面",
            "planes": [
                {
                    "gt_label": 1,
                    "normal": [0.0, 0.0, 1.0],
                    "offset_m": 0.0,
                    "support": {
                        "x_m": [-2.8, 2.8],
                        "y_m": [-2.2, 2.17],
                        "excluded_rectangle": {"x_m": [0.23, 1.82], "y_m": [-0.92, 0.92]},
                    },
                },
                {
                    "gt_label": 2,
                    "normal": [0.0, 0.0, 1.0],
                    "offset_m": -0.1,
                    "support": {"x_m": [0.27, 1.78], "y_m": [-0.88, 0.88]},
                },
                {
                    "gt_label": 3,
                    "normal": [0.0, 1.0, 0.0],
                    "offset_m": -2.2,
                    "support": {"x_m": [-2.8, 2.8], "z_m": [0.03, 1.8]},
                },
            ],
            "nonplane": [],
            "sampling": {
                "surface_weights": [0.68, 0.12, 0.20],
                "occluded_surface_weights": [0.72, 0.08, 0.20],
                "occlusion": dict(occlusion, step_y_m=[-0.88, 0.0]),
                "noise_std_m": base_noise_std_m,
                "unsampled_surfaces": "段差の側面および各面の交線付近",
            },
        },
        {
            "name": "curved_object",
            "file": "curved_object.csv",
            "description": "水平床面と、非平面として定義した滑らかな円柱側面",
            "planes": [
                {
                    "gt_label": 1,
                    "normal": [0.0, 0.0, 1.0],
                    "offset_m": 0.0,
                    "support": {
                        "x_m": [-2.8, 2.8],
                        "y_m": [-2.2, 2.2],
                        "excluded_disk": {"center_xy_m": [0.8, 0.0], "radius_m": 0.62},
                    },
                }
            ],
            "nonplane": [
                {
                    "gt_label": -1,
                    "shape": "cylinder_side",
                    "center_xy_m": [0.8, 0.0],
                    "radius_m": 0.6,
                    "z_m": [0.03, 1.6],
                    "angle_rad": [0.0, 2.0 * math.pi],
                    "local_planarity_note": "局所的な平面近似の可否によらず円柱側面全体を非平面とする正解定義",
                }
            ],
            "sampling": {
                "surface_weights": [0.70, 0.30],
                "occluded_surface_weights": [0.85, 0.15],
                "occlusion": dict(occlusion, cylinder_angle_rad=[0.0, math.pi]),
                "noise_std_m": base_noise_std_m,
                "unsampled_surfaces": "円柱の上下端面および床面との交線付近",
            },
        },
    ]
    noisy_floor = copy.deepcopy(scenes[0])
    noisy_floor["name"] = "noisy_floor"
    noisy_floor["file"] = "noisy_floor.csv"
    noisy_floor["description"] = "観測ノイズの標準偏差を0.003、0.015、0.030 mへ増大した単一の水平床面"
    noisy_floor["sampling"]["noise_std_m_by_third"] = [0.003, 0.015, 0.03]
    scenes.append(noisy_floor)
    return scenes


def allocate_points(points: int, weights: list[float]) -> list[int]:
    """各観測面への最低1点と重み付き剰余配分による点数割り当て。"""
    if points < len(weights):
        raise ValueError("各観測面に1点を割り当て可能な点数が必要")
    remaining_points = points - len(weights)
    weighted_points = [remaining_points * weight / sum(weights) for weight in weights]
    counts = [1 + int(value) for value in weighted_points]
    order = sorted(range(len(weights)), key=lambda idx: (-(weighted_points[idx] % 1), idx))
    for idx in order[: points - sum(counts)]:
        counts[idx] += 1
    return counts


def sample_rectangle(rng: random.Random, support: dict) -> tuple[float, float]:
    """床面の観測範囲から、物体の占有領域を除いた座標の抽出。"""
    while True:
        x = rng.uniform(*support["x_m"])
        y = rng.uniform(*support["y_m"])
        rectangle = support.get("excluded_rectangle")
        if rectangle is not None:
            if rectangle["x_m"][0] <= x <= rectangle["x_m"][1] and rectangle["y_m"][0] <= y <= rectangle["y_m"][1]:
                continue
        disk = support.get("excluded_disk")
        if disk is not None:
            center_x, center_y = disk["center_xy_m"]
            if (x - center_x) ** 2 + (y - center_y) ** 2 < disk["radius_m"] ** 2:
                continue
        return x, y


def sample_frame(scene: dict, frame_idx: int, frames: int, points: int, seed: int) -> list[tuple]:
    """幾何面の選択時に固定したラベルと、その後の観測ノイズ付き座標。"""
    rng = random.Random(f"plane_ground_truth_{schema_version}:{seed}:{scene['name']}:{frame_idx}")
    sampling = scene["sampling"]
    occlusion = sampling["occlusion"]
    is_occluded = occlusion["start_frame_idx"] <= frame_idx < occlusion["end_frame_idx_exclusive"]
    rows = []

    if scene["name"] in ("coplanar_gap", "noisy_floor"):
        weights = sampling["occluded_patch_weights"] if is_occluded else sampling["patch_weights"]
        counts = allocate_points(points, weights)
        noise_std_m = sampling["noise_std_m_by_third"][min(2, frame_idx * 3 // frames)]
        plane = scene["planes"][0]
        for patch_idx, (patch, count) in enumerate(zip(plane["support"]["patches"], counts)):
            support = dict(patch)
            if is_occluded and patch_idx == 1:
                support["y_m"] = occlusion["right_patch_y_m"]
            for _ in range(count):
                x, y = sample_rectangle(rng, support)
                rows.append((x, y, rng.gauss(0.0, noise_std_m), plane["gt_label"]))
    else:
        weights = sampling["occluded_surface_weights"] if is_occluded else sampling["surface_weights"]
        counts = allocate_points(points, weights)
        noise_std_m = sampling["noise_std_m"]
        for surface_idx, count in enumerate(counts):
            if scene["name"] == "step_and_wall" or surface_idx == 0:
                plane = scene["planes"][surface_idx]
                support = dict(plane["support"])
                if scene["name"] == "step_and_wall" and surface_idx == 1 and is_occluded:
                    support["y_m"] = occlusion["step_y_m"]
                for _ in range(count):
                    noise_m = rng.gauss(0.0, noise_std_m)
                    if plane["normal"] == [0.0, 1.0, 0.0]:
                        x = rng.uniform(*support["x_m"])
                        y = -plane["offset_m"] + noise_m
                        z = rng.uniform(*support["z_m"])
                    else:
                        x, y = sample_rectangle(rng, support)
                        z = -plane["offset_m"] + noise_m
                    rows.append((x, y, z, plane["gt_label"]))
            else:
                cylinder = scene["nonplane"][0]
                angle_range = occlusion["cylinder_angle_rad"] if is_occluded else cylinder["angle_rad"]
                center_x, center_y = cylinder["center_xy_m"]
                for _ in range(count):
                    angle_rad = rng.uniform(*angle_range)
                    radius_m = cylinder["radius_m"] + rng.gauss(0.0, noise_std_m)
                    x = center_x + radius_m * math.cos(angle_rad)
                    y = center_y + radius_m * math.sin(angle_rad)
                    z = rng.uniform(*cylinder["z_m"])
                    rows.append((x, y, z, cylinder["gt_label"]))

    rng.shuffle(rows)
    return rows


def generate_dataset(output: Path, seed: int, frames: int = 40, points: int = 2000) -> dict:
    """CSV点群と再現条件・解析形状を含む版管理済みメタデータの保存。"""
    if frames < 1:
        raise ValueError("framesには正の整数が必要")
    if points < 3:
        raise ValueError("pointsには3以上の整数が必要")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    scenes = scene_definitions(frames)
    for scene in scenes:
        scene["num_points"] = frames * points
        with (output / scene["file"]).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(csv_fields)
            for frame_idx in range(frames):
                rows = sample_frame(scene, frame_idx, frames, points, seed)
                for point_idx, (x, y, z, gt_label) in enumerate(rows):
                    writer.writerow((frame_idx, point_idx, f"{x:.9f}", f"{y:.9f}", f"{z:.9f}", gt_label))
    manifest = {
        "schema_version": schema_version,
        "generator": "plane_ground_truth_20260929/generate.py",
        "seed": seed,
        "frames": frames,
        "points_per_frame": points,
        "coordinate_unit": "m",
        "csv_fields": list(csv_fields),
        "plane_equation": "normal[0] * x + normal[1] * y + normal[2] * z + offset_m = 0",
        "gt_labels": {
            "positive": "シーン内で時刻によらず固定した解析平面のインスタンスID",
            "-1": "解析形状として非平面に属する点",
            "0": "評価対象外の境界点用予約値。本生成器では交線を避けた観測点のみ",
        },
        "label_source": "ノイズ付与前の解析形状の所属。検出器の出力・パラメータへの非依存",
        "point_identity": "frame_idxとpoint_idxの組による一意な観測点ID。時系列の対応点IDではない",
        "point_order": "固定シードによるフレーム内の観測点順序のシャッフル",
        "noise_model": "平均0のGaussian法線方向ノイズ。円柱側面は半径方向への付与",
        "temporal_geometry": "固定形状からフレームごとに独立抽出した観測点と中盤の部分遮蔽",
        "determinism": "同一Python実行環境と同一引数によるCSVおよびJSONのバイト一致",
        "scenes": scenes,
    }
    with (output / "manifest.json").open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="CSVとmanifest.jsonの保存先")
    parser.add_argument("--seed", type=int, default=20260929, help="疑似乱数の固定シード")
    parser.add_argument("--frames", type=int, default=40, help="シーンごとのフレーム数")
    parser.add_argument("--points", type=int, default=2000, help="1フレームの観測点数")
    args = parser.parse_args()
    if args.frames < 1 or args.points < 3:
        parser.error("--framesは正の整数、--pointsは3以上の整数が必要")
    manifest = generate_dataset(args.output, args.seed, args.frames, args.points)
    print(json.dumps({"output": str(args.output), "scenes": len(manifest["scenes"]), "frames": args.frames, "points_per_frame": args.points}, ensure_ascii=False))


if __name__ == "__main__":
    main()
