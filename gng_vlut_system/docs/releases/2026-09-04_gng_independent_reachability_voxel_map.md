# GNG非依存到達可能ボクセルmap

## 追加内容

- URDFの関節範囲を低差異列で直接サンプルする`reachability_voxel_builder`
- FKと自己衝突判定を通過した手先位置だけを規則ボクセルへ保存する`reachability_voxel_map.bin`
- 位置、法線、代表関節角、空間隣接edgeだけを含む軽量`VIZGST1`出力
- `visualization_gng_static_node`によるGNG非依存の静的topic配信
- 到達可能ボクセルmapを入力にする`reachability_voxel_visualization_gng_trainer`

## 挙動

- 元GNG、VLUT、元ノード対応表、遷移列を読み込まない構成
- 登録済みセルは少なくとも1個の自己衝突なし代表関節角を持つ構成
- 未登録セルは指定サンプル数で未確認の状態。到達不能の確定ではない
- 空間隣接edgeは可視化用。関節空間での無衝突遷移は表さない
- 可視化GNGも元GNGを読まず、ボクセルmapの代表関節角と空間隣接候補edgeを使う構成
- 可視化edgeはURDF FK補間で途中nodeを経由した候補だけを残す構成

## 検証

- `gng_cpu_container`で`colcon build --packages-select gng_vlut_system --symlink-install`成功
- `ToPoDualArm.yaml`、`left_arm`、`0.2 m`、500サンプルで300対象セル中40セルを登録
- 隔離ROS domainで40ノード、74エッジの`TopologicalMap`を受信
- 42ボクセルから15ノード、50エッジの可視化GNGを生成・再読込
