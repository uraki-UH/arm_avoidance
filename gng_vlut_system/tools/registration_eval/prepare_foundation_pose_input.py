#!/usr/bin/env python3
"""GNGデータセットをFoundationPoseの単一フレーム入力へ変換する。"""

import argparse
import gzip
import json
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


class InputError(RuntimeError):
    """入力データセットの不足・不整合。"""


def load_dataset(dataset_path: Path) -> dict:
    try:
        with gzip.open(dataset_path, "rt", encoding="utf-8") as input_file:
            return json.load(input_file)
    except (OSError, json.JSONDecodeError) as error:
        raise InputError(f"GNGデータセットを読み込めません: {dataset_path}: {error}") from error


def get_rgbd_storage(dataset: dict) -> dict:
    source = dataset.get("source")
    if not isinstance(source, dict):
        raise InputError("sourceが存在しません")
    point_cloud = source.get("point_cloud")
    if not isinstance(point_cloud, dict):
        raise InputError("source.point_cloudが存在しません")
    if point_cloud.get("format") != "rgbd_png":
        raise InputError(
            "rgbd_png保存物ではありません。organized RGB-D点群とCameraInfoを入力して"
            "ObjectGngDatasetExporterから保存してください")
    return point_cloud


def resolve_dataset_file(dataset_path: Path, file_name: object, file_label: str) -> Path:
    if not isinstance(file_name, str) or not file_name:
        raise InputError(f"{file_label}のファイル名がありません")
    relative_path = Path(file_name)
    if relative_path.is_absolute() or ".." in relative_path.parts:
        raise InputError(f"{file_label}のファイル名はデータセット直下の相対パスだけ指定可能です")
    file_path = dataset_path.parent / relative_path
    if not file_path.is_file():
        raise InputError(f"{file_label}が見つかりません: {file_path}")
    return file_path


def load_camera_matrix(point_cloud: dict) -> np.ndarray:
    camera = point_cloud.get("camera")
    if not isinstance(camera, dict):
        raise InputError("camera情報がありません")
    camera_k = camera.get("k")
    if not isinstance(camera_k, list) or len(camera_k) != 9:
        raise InputError("camera.kは9要素の配列で指定してください")
    try:
        matrix = np.asarray(camera_k, dtype=np.float64).reshape(3, 3)
    except ValueError as error:
        raise InputError("camera.kを3x3行列へ変換できません") from error
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise InputError("camera.kの焦点距離が不正です")
    return matrix


def load_depth_image(depth_path: Path) -> np.ndarray:
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16 or depth.ndim != 2:
        raise InputError("depth画像は16-bit単一チャンネルPNGが必要です")
    if not np.any(depth > 0):
        raise InputError("depth画像に有効画素がありません")
    return depth


def validate_color_image(color_path: Path, image_shape: tuple[int, int]) -> None:
    color = cv2.imread(str(color_path), cv2.IMREAD_UNCHANGED)
    if color is None or color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] not in (3, 4):
        raise InputError("color画像は8-bit RGBまたはRGBA PNGが必要です")
    if color.shape[:2] != image_shape:
        raise InputError("color画像とdepth画像のサイズが一致しません")


def load_mask_image(mask_path: Path, image_shape: tuple[int, int]) -> np.ndarray:
    mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    if mask is None:
        raise InputError(f"mask画像を読み込めません: {mask_path}")
    if mask.shape[:2] != image_shape:
        raise InputError("mask画像とdepth画像のサイズが一致しません")
    if mask.ndim == 3:
        mask = np.any(mask > 0, axis=2)
    else:
        mask = mask > 0
    if not np.any(mask):
        raise InputError("mask画像に対象画素がありません")
    return mask.astype(np.uint8) * 255


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GNGデータセットをFoundationPose入力へ変換")
    parser.add_argument("dataset", type=Path, help="*_gng_template.json.gz")
    parser.add_argument("--mesh-file", required=True, type=Path, help="登録物体のCAD mesh")
    parser.add_argument("--output-dir", required=True, type=Path, help="空のFoundationPose入力出力先")
    parser.add_argument("--mask-file", type=Path, help="対象物の二値マスクPNG")
    parser.add_argument(
        "--allow-depth-mask",
        action="store_true",
        help="mask未指定時にdepth有効画素を暫定マスクとして使用",
    )
    parser.add_argument(
        "--foundation-pose-dir",
        type=Path,
        help="FoundationPose取得先。指定時はrun_demo.pyの存在を検査",
    )
    parser.add_argument("--frame-id", default="000000", help="出力フレーム識別子")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dataset_path = args.dataset.resolve()
    mesh_path = args.mesh_file.resolve()
    output_dir = args.output_dir.resolve()
    if not dataset_path.is_file():
        raise InputError(f"GNGデータセットが見つかりません: {dataset_path}")
    if not mesh_path.is_file():
        raise InputError(f"CAD meshが見つかりません: {mesh_path}")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.frame_id):
        raise InputError("frame_idは英数字、_、-だけで指定してください")
    if output_dir.exists():
        if not output_dir.is_dir():
            raise InputError(f"output_dirはディレクトリにしてください: {output_dir}")
        if any(output_dir.iterdir()):
            raise InputError(f"output_dirは空ディレクトリにしてください: {output_dir}")
    if args.foundation_pose_dir is not None:
        run_demo_path = args.foundation_pose_dir.resolve() / "run_demo.py"
        if not run_demo_path.is_file():
            raise InputError(f"FoundationPoseのrun_demo.pyが見つかりません: {run_demo_path}")

    dataset = load_dataset(dataset_path)
    point_cloud = get_rgbd_storage(dataset)
    if point_cloud.get("depth_unit_m") != 0.001:
        raise InputError("FoundationPose入力ではdepth_unit_m=0.001のdepth PNGが必要です")
    depth_path = resolve_dataset_file(dataset_path, point_cloud.get("depth_file_name"), "depth画像")
    color_path = resolve_dataset_file(dataset_path, point_cloud.get("color_file_name"), "color画像")
    camera_matrix = load_camera_matrix(point_cloud)
    depth = load_depth_image(depth_path)
    validate_color_image(color_path, depth.shape)

    if args.mask_file is not None:
        mask = load_mask_image(args.mask_file.resolve(), depth.shape)
        mask_source = str(args.mask_file.resolve())
    elif args.allow_depth_mask:
        mask = (depth > 0).astype(np.uint8) * 255
        mask_source = "depth有効画素"
    else:
        raise InputError("mask-fileを指定してください。暫定検証だけは--allow-depth-maskを指定できます")

    output_dir.mkdir(parents=True, exist_ok=True)
    rgb_dir = output_dir / "rgb"
    depth_dir = output_dir / "depth"
    mask_dir = output_dir / "masks"
    for directory in (rgb_dir, depth_dir, mask_dir):
        directory.mkdir(exist_ok=True)
    shutil.copy2(color_path, rgb_dir / f"{args.frame_id}.png")
    shutil.copy2(depth_path, depth_dir / f"{args.frame_id}.png")
    if not cv2.imwrite(str(mask_dir / f"{args.frame_id}.png"), mask):
        raise InputError("mask画像を書き込めません")
    np.savetxt(output_dir / "cam_K.txt", camera_matrix, fmt="%.12g")
    manifest = {
        "template_id": dataset.get("template_id"),
        "source_dataset": str(dataset_path),
        "mesh_file": str(mesh_path),
        "frame_id": args.frame_id,
        "mask_source": mask_source,
        "depth_unit_m": point_cloud["depth_unit_m"],
    }
    with (output_dir / "foundation_pose_input.json").open("w", encoding="utf-8") as output_file:
        json.dump(manifest, output_file, ensure_ascii=False, indent=2)
        output_file.write("\n")

    foundation_pose_dir = args.foundation_pose_dir.resolve() if args.foundation_pose_dir else Path("/path/to/FoundationPose")
    print(f"FoundationPose入力を作成しました: {output_dir}")
    print("実行例:")
    print(
        f"python3 {foundation_pose_dir / 'run_demo.py'} --mesh_file {mesh_path} "
        f"--test_scene_dir {output_dir} --debug 0 --debug_dir {output_dir / 'result'}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except InputError as error:
        print(f"FoundationPose入力準備失敗: {error}", file=sys.stderr)
        raise SystemExit(2)
