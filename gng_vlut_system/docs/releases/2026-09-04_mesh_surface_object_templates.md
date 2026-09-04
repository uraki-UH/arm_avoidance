# 2026-09-04 - 完全表面mesh物体GNGテンプレート

## Summary

OBJ/STL meshから内部点なしの完全表面点群を生成し、`ais_gng` CPU backendの学習結果を
`object_surface_dataset`へ統合する経路を追加。

## Added

- 面積比例の三角形表面サンプラ
- 完全表面JSONから`PointCloud2`を10 Hzで配信するROS 2補助コマンド
- `ais_gng`保存済みgzip GNG JSONを完全表面JSONへ統合する補助コマンド
- 軽自動車、普通自動車、トラック、バイク、自転車、船のテンプレートcatalog
- `mesh_surface.yaml`のCPU GNG設定
- Dockerでの変換、学習、保存、統合、静的召喚の手順書

## Behavior Impact

テンプレート生成時はmesh表面だけを使用し、内部点、オクルージョン、観測ノイズを追加しない。
既定の座標正規化はXY中心を原点、最下部をZ=0とする。環境側のdepth観測GNGは別途欠損を含むまま照合する。

## Topics / Params / Messages

- Topic: `/mesh_template_surface` (`sensor_msgs/msg/PointCloud2`)
- GNG設定: `ais_gng/config/gng_cpu/mesh_surface.yaml`
- 既存service: `/save_gng_data` (`ais_gng_msgs/srv/SaveObjectGngDataset`)
- 静的召喚topic: `/<template_id>/topological_map_static`
- 基幹GNGバイナリ、既存message定義、既存topicの変更なし

## Verification

- `python3 -m py_compile gng_vlut_system/tools/mesh_surface_template.py`
- 最小OBJからの完全表面JSON、PCD、GNG統合JSONの生成確認
- Docker内の`colcon build --packages-select ais_gng`
- Docker隔離ROS domainでの`PointCloud2`配信・受信確認
- Docker隔離ROS domainでの完全表面400点からCPU GNG学習、464 node・365 edgeの保存、
  `object_surface_dataset`への統合確認

## Risk / Notes

車両meshはライセンス確認が必要なため同梱しない。catalogの`mesh_path`へ取得済みmeshを設定して使用する。
