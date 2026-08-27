# 2026-08-27 - HTML object GNG template

## Summary

`ToPo-FUZZY_Manipulation_v1.html`へ、完全形の物体GNGをテンプレートJSONとして保存し、
環境GNGと独立して読み込み・重ね描画する基盤を追加した。

## Added

- `schema_version: 1`、`kind: object_template`のテンプレートJSON。
- `template_id`、表示名、`canonical_yaw_deg`、GNG node・edge・付随属性の保存。
- 勝者点群数と勝者点群共分散を含むnode情報の保存。
- テンプレートの保存、読込、解除、重ね描画のUI。
- 完全表面点群と、条件付きの学習済みGNGを含む`object_surface_dataset` JSONの保存。
- `object_template_map_publisher_node`と専用launchによる`/<template_id>/topological_map_static`配信。

## Behavior Impact

- テンプレートは環境側の`state.nodes`、`state.edges`と別の`state.objectTemplate`へ保持する。
- テンプレート操作は、環境点群、GNG学習状態、クラスタ、ROS入力を変更しない。
- `canonical_yaw_deg`は保存するが、現時点では回転・平行移動の照合は実行しない。
- 深度由来の環境GNGは完全表面データセットへ混在させず、`object_surface`入力で学習したGNGだけを同梱する。
- 物体テンプレートtopicはロボット関節空間の`/ToPoDualArm/topological_map_static`と分離する。

## Topics / Params / Messages

- ROS topic、parameter、message型、基幹GNGバイナリ形式の変更なし。
- 単体HTML内のJSON入出力のみ追加。
- Topic: `/<template_id>/topological_map_static` (`ais_gng_msgs/msg/TopologicalMap`、`transient_local`)
- Launch: `object_template_map_publisher.launch.py`、必須引数`dataset_id`、任意引数`dataset_dir`、`frame_id`、`publish_hz`
- 読込先: `/datasets/<dataset_id>_object_surface_dataset_v1.json`

## Verification

- `git diff --check`
- Node.jsによる`ToPo-FUZZY_Manipulation_v1.html`のinline script構文検査
- Docker内の`colcon build --packages-select gng_vlut_system`
- `mug_template_test`データセットから`/mug_template_test/topological_map_static`の3 node、3 edge、1 cluster配信

## Risk / Notes

- 欠損を許容するyaw照合の評価式と対応候補の可視化は未実装。
