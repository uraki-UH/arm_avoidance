# 2026-09-15 - モデル当てはめなしの連続面クラスタリング

## Summary

既存GNGの位置・法線・エッジだけで連続面を抽出する `smooth_graph` を比較用に追加。
実入力21フレームで曲面処理は7.407 msから1.159 msへ短縮した一方、従来別々だった領域の大きな統合を確認。
既定値は従来の `model` を維持。連続面の抽出であり、物体境界や曲面種別の識別ではない。

## Changed / Added

既存 `surface_model_tracking.cpp` 内に接続判定と差分更新を追加。新規の実装ファイル・ライブラリ・メッセージ定義なし。
位置差を `delta`、単位法線を `n_a, n_b` として、次の条件を満たす実GNGエッジを採用。

```text
norm(delta) <= max_link_length
abs(dot(n_a, n_b)) >= cos(max_link_normal_deg)
max(abs(dot(n_a, delta)), abs(dot(n_b, delta)))
  <= norm(delta) * sin(max_link_tangent_deg)
```

無効法線・非有限距離のエッジは不採用。法線の正負反転に対して不変。
採用エッジの連結成分を抽出し、`min_fit_nodes` を満たす成分を `smooth_surface`、小成分を `unknown` として保持。
平面も連続面に含み、球・円柱・平面などへの分類や、曲率フィット、空隙の補完はなし。
GNG学習、入力点数、既存の平面クラスタリングには変更なし。

- 位置・正規化法線・判定設定の変化した端点を持つエッジだけ幾何条件を再評価。
- 接続採否の変化、採用エッジの追加・削除、ノード消失・ID位置飛びの影響成分だけ所属を再探索。
- 分割時は大きい子、統合時は旧所属の多い成分へIDを継承。座標系変更、フレーム番号・時刻の巻戻り、重複IDで履歴を解除。
- `extract()` は履歴なし、`tracker::update()` は履歴あり。同じ設定なら所属と採用エッジが一致。
- JSON、Marker、任意のTopologicalMap出力に共通の所属を使用。棄却エッジは迂回路で同一領域になっても描画・グラフ出力から除外。

全ノード・エッジの変化確認、キーのソート、出力構築は毎回必要。現実装の全更新コストは
ノード数N・エッジ数Eに対して `O(E log E + N log N)` が上限の目安、保持メモリは `O(N+E)`。
切断時には旧成分全体の探索が必要な場合があり、変更端点だけで完結する方式ではない。

## Topics / Params / Messages

| 設定 | 既定値 | 用途 |
| --- | --- | --- |
| launch `surface_method` | `auto` | YAML設定／`model`／`smooth_graph` の選択 |
| `surface_model.method` | `model` | 起動時の方式選択 |
| `surface_model.max_link_length` | 0.08 m | 採用エッジ長 |
| `surface_model.max_link_normal_deg` | 45 deg | 両端法線の接続角 |
| `surface_model.max_link_tangent_deg` | 30 deg | smooth_graphのエッジと両端接平面の角度 |
| `surface_model.min_fit_nodes` | 12 | smooth_graphでは連続面の所属ノード数 |
| `surface_model.retention.enable` | true | smooth_graphでは判定・所属・IDの履歴利用。falseで毎回全判定 |
| `surface_model.retention.max_node_displacement` | 0.05 m | ID継承対象の位置変化 |
| `surface_model.min_display_plane_patches` | 2 | 既存表示フィルタ。0で元平面数による制限なし |

新規設定は `method`、`max_link_tangent_deg`、launchの `surface_method`。
起動時に読込み、実行中のROSパラメータ変更による切替は未対応。
smooth_graphではモデル誤差・半径・フィット予算・曲率品質・平面保護・支持領域補完と、上表以外のretention設定は不使用。
平面入力は現在フレームのパッチ対応と表示条件に使用。非平面ノードだけの面を表示する場合は `min_display_plane_patches: 0` が必要。

トピック名・ROSメッセージ型の変更なし。`/curved_surface_clusters/models` のJSONへ `method` を追加。
smooth_graphでは `link_check_num`、`connectivity_node_num` を追加し、形状種別は `smooth_surface` または `unknown`。
`fit` は出力せず、パッチの `curvature.method: none`、`valid: false`、フィット数・曲率時間は0。
`is_retained` は領域IDの継承で、モデル適合の保証ではない。Markerラベルは領域種別・IDのみ。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml surface_method:=smooth_graph
```

従来方式へ戻す場合は `surface_method:=model` を指定。省略時も現行YAMLは `model`。

## Verification

Releaseビルド。保存済みの `/camera/camera/depth/color/points` 由来のGNGと平面入力21フレームを再評価。
各フレーム1,545ノード・10平面で一致。実行順を入れ替えた5回について、各回21フレーム平均の中央値を比較。

| 処理 | 平均時間の中央値 |
| --- | ---: |
| 従来model、履歴あり | 7.407 ms |
| smooth_graph、履歴なし | 1.117 ms |
| smooth_graph、差分・ID保持あり | 1.159 ms |

従来比84.4%短縮、約6.4倍の処理速度。ただしクラスタリング結果は異なる。
計測は曲面抽出と内部出力データ構築の範囲で、JSON化・Marker生成・ROS転送・描画・GNG学習を含まない。
実入力では全エッジの幾何条件を再評価し、平均1,514.8ノード、全体の98.0%を所属再探索。
差分管理ありは履歴なしより約3.7%遅く、今回の実入力に対する差分化単独の利益は未確認。
同一実フレームを反復した別検証では、初回を除く20回の判定・所属再探索がともに0、
履歴なし1.124 msに対して履歴あり0.681 ms。こちらは静止入力の単一実行平均。

新方式の最大領域は平均1,420.4ノード、範囲1,357〜1,472ノード。
最初のフレームでは従来644ノードに対して1,403ノード。
表示対象も平均95.7から1,426.5ノードへ増加。背景平面と小領域を大きく統合する挙動が、既定置換を見送った根拠。
物体分割の正解ラベルはなく、精度の数値評価ではない。

- C++テスト74件成功。既存66件と、全周円柱、角・平行段差、分割・統合、ノード移動・並べ替え・削除・ID再利用・重複、無効入力、方式切替、棄却近道の非表示を検証。
- ランダム変更80フレームと実入力21フレームで差分・履歴なしの所属と採用エッジが完全一致。変更前バイナリの保存結果と現在model方式の所属・種別・IDも21フレームで一致。
- domain 218のROS検証でJSON・Graph・Markerの一致、フィットなし、静止時の省略、全エッジ削除時の分割、空入力時の消去を確認。
- ROS単発計測は初回1.889 ms、同一入力0.705 ms、全エッジ切断2.566 ms。全面変化の一例であり最大時間の保証ではない。
- launch引数・Python構文・`git diff --check` 成功。通常のGNG launchやbag再生は新規起動せず、保存実入力で検証。

コンテナ内の検証コマンド:

```bash
source /ros2_ws/install/setup.bash
cmake --build /ros2_ws/build/ais_gng --target test_surface_model plane_cluster_incremental_node replay_surface_models benchmark_surface_merge -j2
/ros2_ws/build/ais_gng/test_surface_model --gtest_color=no
python3 /ros2_ws/src/tmp/surface_graph_20260915/compare.py
python3 /ros2_ws/src/tmp/surface_graph_20260915/ros_check.py
ros2 launch ais_gng ais_gng.launch.py --show-args
```

ROS検証スクリプト内の起動コマンド:

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 /ros2_ws/build/ais_gng/plane_cluster_incremental_node --ros-args \
  -r __node:=smooth_graph_check \
  -p input_topic:=/smooth_graph_check/map -p clusters_input_topic:=/smooth_graph_check/planes \
  -p surface_model.output_topic:=/smooth_graph_check/result -p surface_model.method:=smooth_graph \
  -p surface_model.hz:=50.0 -p surface_model.enable_graph:=true -p surface_model.enable_markers:=true \
  -p enable_plane_markers:=false -p enable_nonplane_markers:=false
```

検証ROSノードはSIGINTで正常停止、driver・ビルド・テスト・再評価・描画も全終了。既存プロセスへの停止・再起動操作なし。
ログ・比較図・検証スクリプトは無視対象 `tmp/surface_graph_20260915/`、入力は `tmp/surface_incremental_20260915/observed.json`。
比較図の再生成はworkspaceで `python3 tmp/surface_graph_20260915/plot.py`。一時成果物はGit管理外。

## Risk / Notes

- 滑らかな局所接続の連鎖で、遠方の異なる面まで同一成分になる。連続面の定義に一致しても、物体や曲率領域としての適切な分割とは別問題。
- 欠損で実エッジがなくなると分割。元のGNG法線の誤りは過統合・過分割の原因。長時間・多シーンの分割品質は未評価。
- 改善候補は元平面の位置支持による背景平面保護、細い橋となる接続の検証、モデル当てはめなしの法線変化の境界判定。今回未実装で、速度・分割品質の両方の検証が必要。
- C++構造体のABI変更に伴い、リンクする実行ファイルをまとめて再ビルド済み。旧ライブラリと新ヘッダで作った実行ファイルの混用は不可。
