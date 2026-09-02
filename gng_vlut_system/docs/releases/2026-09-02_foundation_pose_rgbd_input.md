# 2026-09-02 - FoundationPose用RGB-D入力準備

## 概要

`rgbd_png`形式で保存した物体GNGデータセットを、FoundationPoseの単一フレーム入力構成へ変換する補助ツールを追加。

## 追加

- `prepare_foundation_pose_input.py`によるcolor、depth、mask、カメラ内部パラメータの入力検査
- FoundationPose互換の`rgb/`、`depth/`、`masks/`、`cam_K.txt`出力
- 入力由来、CAD mesh、マスク由来を記録する`foundation_pose_input.json`
- `--allow-depth-mask`による単体撮影向け暫定マスク生成

## 挙動

`pcd_binary_compressed`形式やcolor、depth、CameraInfoが欠けるGNGデータセットは変換しない。実環境では`--mask-file`でセグメンテーション器などの物体マスクを与える。depth有効画素マスクは背景を含まない変換経路確認だけに限定。

FoundationPoseの6D姿勢は候補仮説として扱う。テンプレート召喚前に、既存GNGの平面、非平面成分、共分散評価による検証が必要。

## API影響

ROS topic、launch引数、message、serviceの変更なし。

## 検証

- `python3 -c 'source=open("gng_vlut_system/tools/registration_eval/prepare_foundation_pose_input.py", encoding="utf-8").read(); compile(source, "prepare_foundation_pose_input.py", "exec")'`
- `pcd_binary_compressed`形式のbasketテンプレートで安全な変換拒否を確認
- 最小`rgbd_png` fixtureでFoundationPose入力一式の出力を確認

## 注意

FoundationPose推論にはCUDA対応GPU、PyTorch、PyTorch3D、NVDiffRast、学習済み重み、CAD meshが別途必要。
