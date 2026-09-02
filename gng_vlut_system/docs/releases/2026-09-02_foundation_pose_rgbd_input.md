# 2026-09-02 - FoundationPose用RGB-D入力準備

## 概要

`rgbd_png`形式で保存した物体GNGデータセットを、FoundationPoseの単一フレーム入力構成へ変換する補助ツールを追加。

## 追加

- `prepare_foundation_pose_input.py`によるcolor、depth、mask、カメラ内部パラメータの入力検査
- FoundationPose互換の`rgb/`、`depth/`、`masks/`、`cam_K.txt`出力
- 入力由来、CAD mesh、マスク由来を記録する`foundation_pose_input.json`
- `--allow-depth-mask`による単体撮影向け暫定マスク生成
- ToPo-FUZZY深度カメラからの16-bit depth、semantic色RGB、物体mask、`cam_K.txt`のZIP出力
- ToPo-FUZZYからrosbridge経由で送る`/foundation_pose/input/{color,depth,mask,camera_info}`の単発配信

## 挙動

`pcd_binary_compressed`形式やcolor、depth、CameraInfoが欠けるGNGデータセットは変換しない。実環境では`--mask-file`でセグメンテーション器などの物体マスクを与える。depth有効画素マスクは背景を含まない変換経路確認だけに限定。

FoundationPoseの6D姿勢は候補仮説として扱う。テンプレート召喚前に、既存GNGの平面、非平面成分、共分散評価による検証が必要。

HTML出力のRGBはsemantic色であり、実写RGBによる推論精度の評価には使わない。HTML出力は入出力接続と遮蔽条件の検証用途とする。

HTMLのROS配信は、設定可能なプレフィックス配下へ`rgb8` color、ミリメートル単位の`16UC1` depth、`mono8` mask、CameraInfoを同一stampで送る。`frame_id`は`foundation_pose_camera`で固定。

## API影響

ROSのlaunch引数、message、serviceの変更なし。HTMLのrosbridge経由で、設定したプレフィックス配下にRGB-D入力4トピックを追加。

## 検証

- `python3 -c 'source=open("gng_vlut_system/tools/registration_eval/prepare_foundation_pose_input.py", encoding="utf-8").read(); compile(source, "prepare_foundation_pose_input.py", "exec")'`
- `pcd_binary_compressed`形式のbasketテンプレートで安全な変換拒否を確認
- 最小`rgbd_png` fixtureでFoundationPose入力一式の出力を確認
- ToPo-FUZZYの合成点群からZIPを生成し、展開後のdepth PNGが16-bitであることを確認
- ToPo-FUZZYのRGB-D ROSメッセージ構成の静的検査

## 注意

FoundationPose推論にはCUDA対応GPU、PyTorch、PyTorch3D、NVDiffRast、学習済み重み、CAD meshが別途必要。
