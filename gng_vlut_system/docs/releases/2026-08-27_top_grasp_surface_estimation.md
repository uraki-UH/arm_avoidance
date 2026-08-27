# 2026-08-27 - top-only surface grasp estimation

## Summary

隣接平面クラスタの上側領域を統合したXY-OBBがグリッパ把持面積へ収まるかを判定し、上面把持TCP Poseを生成するC++経路を追加。

## Changed

- GNG edgeから平面クラスタ間の隣接graphを生成
- 法線では候補を除外せず、各領域の所属ノードをXY平面へ投影して2次元OBBを生成
- 単体OBBが収まる領域から、重心Zが高い隣接領域を再帰的に統合
- 同じ最上位クラスタへ到達する領域群をまとめ、統合OBBをグリッパ内寸へ90度の2方向で照合
- 側面・段差だけなら収まっても、上側隣接領域を含めると収まらない候補を棄却
- 採用OBBの主軸へyawを合わせ、ローカルZ軸を下向きにしたTCP Poseをpublish
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
- `higher_neighbor_z_tolerance`: 上側隣接領域と判定する重心Z差の余裕
- `minimum_region_nodes`: OBB判定に必要な最小所属ノード数

## Verification

- Releaseビルド成功
- `grasping_system` 4テスト、0 failure
- 合成データで垂直側面と上面を1候補へ統合し、統合すると把持面積を超える段差候補を棄却
- GraspNet scene 3で16〜17平面領域、隣接18〜22組から1〜4候補、対象推定0.07〜0.12 ms

## Risk / Notes

- 候補数は平面クラスタの分割・併合とクラスタ間edgeに追従するため、実センサ検証後に時間方向の確認条件を追加する余地がある
- 現段階では把持面積への包含判定まで。指接触、基部衝突、ロボット到達性は後段評価が必要
