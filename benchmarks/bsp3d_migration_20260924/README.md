# bsp3dへの共通索引移行（2026-09-24）

> 同日追記: 本文は移行直後の比較。現在は目標選択の全体走査削減と位置索引の差分更新を実装済み。[新しい仕様と比較](../goal_selection_efficiency_20260924/README.md)を参照。座標変更時の全再構築はbsp3dの必須動作ではなく、当時の呼出し側実装。

目標選択の静的索引をdoubleのbsp3dへ移し、旧GNG実験版の8分木方式・共通SpatialTreeを削除した比較記録。元CPU GNGの学習実装・YAML・起動先の変更なし。

## 機能の対応

| 旧SpatialTree側の仕組み | bsp3d側の対応 | 今回の判断 |
| --- | --- | --- |
| AABB範囲内の要素列挙 | 不足 | 閉区間の`query_aabb`を追加。空部分木の省略・分割軸による枝刈り・葉の座標キャッシュを利用 |
| double座標 | テンプレート本体で対応済み | `bsp3d::Index<T, double>`・`point3<double>`・`index_params<double>`の公開指定を追加 |
| 削除時のハンドル、部分木の件数 | 既存実装あり | 既存実装の利用 |
| 葉内移動の判定、祖先を利用した再配置 | 既存実装あり。境界を管理する祖先への直接移動もあり | 既存実装の利用 |
| 8子セルの連続配置・固定順探索 | bsp3dは二分木。2近傍専用処理とSoA座標キャッシュあり | 8分木の構造を追加する速度上の根拠はなし |
| 座標が同じ場合の静的索引の再利用 | 目標選択側の機能 | ラベル・ID・TF変更での再利用を維持 |

共通の仕組みが多い一方、木の分割数と構築費は異なるため、全用途でbsp3dが速いという意味ではない。

## 条件

- 比較元: `base_revision.txt`のコミット。旧ライブラリと目標選択ヘッダをGitから保存コピー。通常ソースに旧依存を復活させる操作なし。
- コンテナ: `gng_cpu_container`、GCC 11.4、Release相当`-O3 -DNDEBUG`、double。CPU 0固定、前後を交互に3試行。既存ROS・bagと同居し、CPU周波数や競合負荷の固定なし。
- `points.txt`: 保存したGNGノード18,729点。出典は`artifacts/gng_radix_production_20260924/boost/voxel_0.5_1_frame_299.npz`。2026-09-15の静的マップ10,801点とは別の入力。
- `planar.txt`: 合成の薄い平面20,000点、原点重複500点、seed 20260924。正確な入力ファイルのSHA-256は`summary.json`。
- 単体索引: 1試行12反復、先頭2反復を除いた平均値の3試行中央値。構築は1回分、AABB・8近傍はそれぞれ1024問い合わせの合計。目標選択で8近傍探索は不使用。
- `before`: 旧8分木。`bsp32`は既定の葉32・bboxなし。`bsp16`・`bsp8`は葉の容量のみ変更。`bsp32_bbox`は点群の実境界箱による追加枝刈り。

## 単体索引の結果

実GNG座標:

| 方式 | 構築 ms | AABB 1024回 ms | 8近傍 1024回 ms |
| --- | ---: | ---: | ---: |
| before | 1.371 | 0.664 | 2.451 |
| bsp32 | 4.420 | 0.461 | 1.397 |
| bsp16 | 4.967 | 0.401 | 1.402 |
| bsp8 | 5.624 | 0.391 | 1.618 |
| bsp32_bbox | 5.098 | 0.446 | 1.507 |

合成平面:

| 方式 | 構築 ms | AABB 1024回 ms | 8近傍 1024回 ms |
| --- | ---: | ---: | ---: |
| before | 1.399 | 2.017 | 3.439 |
| bsp32 | 4.330 | 1.932 | 1.921 |
| bsp16 | 5.134 | 1.747 | 2.105 |
| bsp8 | 6.228 | 1.981 | 2.479 |
| bsp32_bbox | 4.999 | 1.796 | 2.081 |

採用構成は`bsp32`。実GNG座標ではAABBが約30.5%短縮、構築は約3.2倍。葉をさらに小さくすると構築負担が増え、bboxにも維持費があるため、既定値を保持。範囲検索が必要なのは目標選択であり、GNGの2近傍学習に範囲検索を導入した変更ではない。

## 目標選択全体の結果

候補数1・20・100、固定の回転TF、壁ラベル・可操作性付きマップで全走査版と比較。1試行220反復、先頭20を除く200反復の中央値をさらに3試行で中央値化。下表は索引を利用した`select_goal_nodes`全体で、索引構築・ROS受信・publish・viewerは対象外。

| 入力 | 候補数 | 旧SpatialTree ms | bsp3d ms |
| --- | ---: | ---: | ---: |
| points | 1 | 1.779 | 1.895 |
| points | 20 | 2.013 | 2.089 |
| points | 100 | 2.158 | 2.233 |
| planar | 1 | 1.694 | 1.820 |
| planar | 20 | 1.857 | 1.953 |
| planar | 100 | 2.210 | 2.270 |

実GNG座標では約0.075〜0.116 ms増加。単体AABBの短縮だけでは選択全体の改善には至らず、全体高速化の主張なし。メッセージの結果構築や特徴量の処理、同居負荷を含む値であり、差の原因を単一処理へ断定しない。旧・新の選択IDと出力mapは、それぞれ同じ全走査基準と7,920組すべて一致。

マップ座標・配列順・件数変更時は索引を再構築。同じ座標の再受信やラベル・ID・法線・TF変更では再利用を維持。今回の初期構築増加は静的索引の再利用で毎回は発生しないが、座標が毎回変わる運用では負担になる。

## 不採用の一括構築

空のrootへ全点を登録して既存の分割処理を再帰適用する試作。`bulk32`は葉32・中央値付近の分割、`bulk16`は葉16、`bulk32_midpoint`は広がりの中点による分割。旧・採用・試作を同じ追加比較内で交互に測定。

実GNG座標:

| 方式 | 構築 ms | AABB 1024回 ms | 8近傍 1024回 ms |
| --- | ---: | ---: | ---: |
| before | 1.371 | 0.663 | 2.478 |
| bsp32 | 4.583 | 0.467 | 1.445 |
| bulk32 | 9.881 | 0.462 | 1.423 |
| bulk16 | 11.144 | 0.471 | 1.524 |
| bulk32_midpoint | 6.528 | 0.479 | 1.328 |

合成平面:

| 方式 | 構築 ms | AABB 1024回 ms | 8近傍 1024回 ms |
| --- | ---: | ---: | ---: |
| before | 1.381 | 1.975 | 3.347 |
| bsp32 | 4.334 | 1.911 | 1.908 |
| bulk32 | 12.828 | 1.901 | 1.944 |
| bulk16 | 13.778 | 1.880 | 2.115 |
| bulk32_midpoint | 8.510 | 1.833 | 1.997 |

全点から既存の分割処理を反復する案では初期構築が遅くなり不採用。現在の実装に`build` APIは追加していない。試作は`bulk_prototype.patch`に保存し、移行前bsp3dヘッダに適用して再現可能。これは一括構築一般が遅いという結果ではなく、今回の具体的な実装の判断。

## 検証

- AABB: float/double、bbox有無、ヒステリシス有無、追加・移動・削除・空・重複・上下限境界・nextafter・不正範囲、37,600比較成功。ASan/UBSanでも成功。
- bsp3d既存テスト: 動的40,000操作と全走査照合に成功。CMakeのCTest 2/2成功。
- 目標選択: 通常パッケージのReleaseビルド成功、`test_grasp_candidate_reachability`の12テスト成功。回転・境界・TF・メタデータ変更と索引再利用を検証。
- GNG独立比較版: 共通SpatialTree削除後にReleaseビルド・CTest 18/18成功。
- 一括構築試作: 動的更新を含む75,216比較成功。結果一致を確認したうえで時間増加を理由に不採用。
- 実ROSの目標選択ノードを新規起動したbag再生試験は未実施。通常ビルドのインストール先は更新済み。

## 再現手順

コンテナ内で実行。標準の`gng_vlut_system`がビルド済みで、ROS依存と同じGTestライブラリが必要。`prepare.py`は固定Gitコミットから比較元と試作ソースを生成し、ローカルの入力点ファイルは生成しない。生ログ・入力座標はGit管理外の`artifacts/bsp3d_migration_20260924/`。入力がないcheckoutでは同条件での再計測は不可。

```bash
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/benchmarks/bsp3d_migration_20260924/prepare.py
timeout -s INT -k 5 600 python3 /ros2_ws/src/benchmarks/bsp3d_migration_20260924/build_compare.py
timeout -s INT -k 5 600 python3 /ros2_ws/src/benchmarks/bsp3d_migration_20260924/run_compare.py
timeout -s INT -k 5 300 python3 /ros2_ws/src/benchmarks/bsp3d_migration_20260924/run_bulk.py
```

今回起動したビルド・試験コマンド（コンテナ内）:

```bash
cmake -S /ros2_ws/src/bsp3d -B /tmp/bsp3d_migration_20260924/bsp_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/bsp3d_migration_20260924/bsp_build -j2
ctest --test-dir /tmp/bsp3d_migration_20260924/bsp_build --output-on-failure --timeout 60
cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree -B /tmp/bsp3d_migration_20260924/gng_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/bsp3d_migration_20260924/gng_build -j2
ctest --test-dir /tmp/bsp3d_migration_20260924/gng_build --output-on-failure --timeout 30
c++ -std=c++17 -O1 -g0 -fsanitize=address,undefined -fno-omit-frame-pointer -I/ros2_ws/src/bsp3d/include /ros2_ws/src/bsp3d/examples/test_aabb.cpp -o /tmp/bsp3d_migration_20260924/aabb_sanitized
/tmp/bsp3d_migration_20260924/aabb_sanitized
cd /ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select gng_vlut_system --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
ctest --test-dir /ros2_ws/build/gng_vlut_system -R test_grasp_candidate_reachability --output-on-failure --timeout 90
```

実際の実行時はコンパイル・ビルド・CTestにも120〜600秒の`timeout -s INT -k 5`を付与。コマンド詳細は`build_commands.txt`、出力は各`*_ctest.log`・`goal_build.log`・比較ログへ保存。

初回colconの探索で計測用`setup.py`の誤実行による既存成果物ディレクトリの`FileExistsError`を検出。`benchmarks/COLCON_IGNORE`を追加して通常ビルドから除外。旧fixture不在とGTestヘッダ・ライブラリ不一致の初回失敗は、出典を明記した保存入力とワークスペースのGTestへ修正済み。

終了確認: 起動した比較・ビルド・試験プロセスは全終了。ホスト・コンテナの今回専用`/tmp/bsp3d_migration_20260924`を削除。既存ROS・bagの13プロセスはPID・PPID・コマンド一致、3コンテナのIDと起動状態を維持。確認結果は`verification.json`、生記録は`runtime_before.json`・`runtime_after.json`・`runtime_diff.json`。
