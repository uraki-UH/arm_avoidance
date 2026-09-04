# 可視化GNGの状態空間保持と空間被覆

## 変更内容

- 可視化GNGの学習特徴量を手先位置だけから、関節移動時間と手先位置の複合特徴量へ変更
- 関節移動時間をURDFの関節速度上限により`max_i(|dq_i| / max_velocity_i)`へ正規化
- 手先位置を時間相当へ換算する`workspace_motion_sec_per_m`を追加
- 手先空間セルを均等に選ぶ`workspace_sample_resolution`を追加
- 可視化ノード位置を所属元ノードの重心から、代表元ノードの実在する手先位置へ変更
- 元angle edgeごとの補間列と移動時間をすべて保存
- FK補間軌跡から局所KNN距離の接続半径内にある可視化nodeだけを選ぶedge生成へ変更
- FK軌跡で裏付けられた候補edgeを各可視化nodeの最短6本へ制限
- 代表元の法線、状態label、関節角と補間軌跡の接続可否を保存する`VIZGNG5`へ更新
- 元ノード対応表と遷移列を除いた`VIZGST1`を追加
- `VIZGST1`だけを読む`visualization_gng_static_node`を追加

## 挙動影響

- 手先位置が近くても、関節移動時間が大きい畳み姿勢は別の可視化状態として残りやすい構成
- 入力点数の多い手先領域だけに可視化ノードが集中しにくい構成
- `VIZGNG2`から`VIZGNG4`は読み込まないため、`visualization_gng_trainer`で再生成が必要
- static nodeは保存時点のlabelを使い、動的safe / danger更新および既存軌道変換は行わない

## 検証

- `gng_cpu_container`で`colcon build --packages-select gng_vlut_system --symlink-install`成功
- `ToPoDualArm10000/gng.bin`の10,801元ノードから150ノードの`VIZGNG5`を生成・再読込成功
- 572可視化edge、234,920元angle edge遷移、代表元ノードID、関節角、法線、状態label、移動時間、接続可否の保存を確認
