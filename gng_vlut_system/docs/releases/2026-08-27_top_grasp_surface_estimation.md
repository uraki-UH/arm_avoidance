# 2026-08-27 - top-only surface grasp estimation

## Summary

各平面クラスタのXY-OBBと隣接平面からの突出距離を判定し、上面把持TCP Poseを生成するC++経路を追加。

## Changed

- GNG edgeから平面クラスタ間の隣接graphを生成
- 法線では候補を除外せず、各領域の所属ノードをXY平面へ投影して2次元OBBを生成
- 各クラスタを独立候補とし、隣接クラスタを候補OBBへ統合しない
- 候補重心から各隣接クラスタ平面までの絶対距離を求め、最小突出距離未満の候補を棄却
- 隣接クラスタがない候補は突出距離条件を適用せず、把持面積内なら採用
- 採用OBBの主軸へyawを合わせ、ローカルZ軸を下向きにしたTCP Poseをpublish
- 上面把持Markerの赤い主矢印を、汎用bridge既定のローカル`-X`から進入方向のローカル`+Z`へ変更
- 平面クラスタ、候補Pose、面積占有率、JSON summary、Markerを1 launchで起動

## Behavior Impact

従来の把持ボクセル総当たりとは独立して、上面把持に限定した軽量な対象推定を比較利用できる。物体ボクセルとグリッパ体積graphは不要。

## Topics / Params / Messages

- `/topological_map`: GNGノードとedge
- `/topological_planar_clusters_incremental`: 把持対象領域と隣接関係の単位となる平面クラスタ
- `/top_grasp_pose_cands`: 採用したTCP Pose群
- `/top_grasp_pose_cand_scores`: 把持可能面積に対する対象OBB面積比
- `/top_grasp_pose_cands/summary`: 平面領域数、棄却理由、処理時間
- `grasp_size_x`, `grasp_size_y`: グリッパ把持面の外寸
- `footprint_margin`, `footprint_padding`: 内側安全余白とGNG点間補正
- `minimum_protrusion_distance`: 隣接平面から必要な候補重心の最小突出距離
- `minimum_region_nodes`: OBB判定に必要な最小所属ノード数

## Verification

- Releaseビルド成功
- `grasping_system` 4テスト、0 failure
- 合成データで壁に隣接する出っ張りを採用し、同一平面上の小領域を突出距離で棄却
- GraspNet scene 3で16〜17平面領域、隣接14〜17組から1〜2候補、突出不足棄却0〜1件、対象推定0.06〜0.07 ms

## Risk / Notes

- 候補数は平面クラスタの分割・併合とクラスタ間edgeに追従するため、実センサで突出距離閾値を調整する必要がある
- 現段階では把持面積への包含判定まで。指接触、基部衝突、ロボット到達性は後段評価が必要
