# 2026-09-23 - Spatial Tree版GNGの範囲検索・候補ソート廃止

## 1. 要約

独立コピー`ais_gng_cpu/experimental/gng_spatial_tree`の近傍取得を、木全体の厳密な最近傍2ノード探索へ変更。範囲検索、候補の列挙・ソート、子セルの並べ替えを廃止。Releaseの同一bag比較で旧版617.50 msから112.51 msへ短縮。グリッド版104.26 msに対しては約7.9%長い結果。

- 木の構築・分割・統合・動的更新は共通`SpatialTree::AdaptiveTree`を使用。GNG独立コピー内の`find_spatial_nearest`で3次元・2近傍の探索を実装。
- 入力点側の子セルを先行し、残りはセル番号の差分マスク1〜7の固定順。分割面までの距離から求めた下限と、現在の第2近傍距離による枝刈り。
- 順序は距離順ではないため、枝刈り対象のセルを飛ばした後も後続セルを評価。固定探索半径や近傍数の近似なし。
- 警戒領域・寿命リセット・重点学習ラベル判定は最近傍2ノードに限定。距離パラメータの判定は維持。

`gng_spatial_nearest_test`で全ノード走査との照合を追加。空、1ノード、遠方、境界、根領域外、同距離、移動、削除、ID再利用、セル統合・再分割、再初期化、寿命・ラベル判定を検証。

旧27セル相当のAABB候補収集と、グリッド順の候補ソートの反復コストを除去。旧探索範囲外でも最近傍が存在する場合は取得。

**削除**

固定範囲の候補配列・`query_aabb`呼び出し・候補ソート。共通木の汎用`findNBest`にある子セルの並べ替えも、最終版の探索経路では不使用。

## 2. 条件・検証

- 寿命・ラベル・警戒領域判定に3番目以降のノードを使用せず、遠方の最近傍も取得するため、旧グリッド／AABB版とのグラフ完全一致保証なし。
- 同距離候補の選択は固定の木探索順に依存。
- YAML入力範囲・ボクセル処理・ノード上限・既存のセル当たり10ノードの管理を維持。入力点の追加除外なし。固定グリッド管理と密なエッジ行列のメモリは残存。
- 元の`ais_gng_cpu/src/gng_cpu`や共通SpatialTreeへの今回の変更なし。通常のROS launchへの組込み・ライブラリ置換なし。

追加・変更なし。距離判定には`node.interval`・`node.s1_reset_range`・`ds.range_max`を継続使用。

`CMAKE_BUILD_TYPE=Release`、実コンパイルコマンドの`-O3 -DNDEBUG`を確認。APIテスト16件と内部最近傍テスト1件がすべて成功。

同じbagの先頭30フレームを繰り返し50回入力。先頭10回を除いた40回を集計。グリッド版と最終木版は交互に3試行。学習乱数とLPFの時間刻み0.1秒を固定し、適用済みYAMLパラメータの一致を確認。

| 方式 | 試行数 | GNG本体平均 ms | 入出力込み平均 ms | 最終ノード数 | 最終エッジ数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 旧AABB・候補ソート版 | 1 | 617.50 | 620.21 | 19,925 | 51,660 |
| 汎用2近傍版（子セルソートあり） | 3 | 196.56 | 199.19 | 19,901 | 53,566 |
| 最終2近傍版（子セル固定順） | 3 | 112.51 | 115.12 | 19,901 | 53,566 |
| 比較グリッド版 | 3 | 104.26 | 106.85 | 19,925 | 51,660 |

最終版の本体平均は各試行114.44／111.65／111.43 ms、各試行p95は117.44／114.76／114.27 ms。旧AABB版の約5.49倍の処理速度。子セル固定順版と汎用2近傍版のグラフハッシュは150/150フレーム一致。各方式の3試行間も全フレーム一致。旧AABB版とグリッド版の対照は50/50フレーム一致。

設定・起動スクリプト・集計値・SHA-256は[benchmarks/gng_spatial_nearest_20260923](../../../benchmarks/gng_spatial_nearest_20260923/)へ移管。生JSON・ビルド条件の生成ファイル・テストログ・共有ライブラリはGit管理外の`artifacts/gng_spatial_nearest_20260923/`にローカル保管。最終版ライブラリは`libgng_spatial_fixed_order.so`、同ディレクトリの`libgng_spatial.so`は比較用の汎用2近傍版。過去版との比較には該当バイナリまたは対応ソースが必要。[保存方針と前提条件](../../../benchmarks/README.md)。

実行した起動コマンド（コンテナ内）:

```bash
cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree \
  -B /tmp/gng_spatial_nearest_build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build /tmp/gng_spatial_nearest_build -j2
ctest --test-dir /tmp/gng_spatial_nearest_build --output-on-failure --timeout 30
bash /ros2_ws/src/benchmarks/gng_spatial_nearest_20260923/run_benchmarks.sh
bash /ros2_ws/src/benchmarks/gng_spatial_nearest_20260923/run_fixed_order_benchmarks.sh
```

各計測スクリプト内のPython実行は`timeout -s INT -k 5 120`つき。実行セッションはすべて終了。新規ROSノードの起動なし、既存プロセスへの停止・再起動操作なし。初回計測スクリプトはROS環境読み込み中の未定義変数で終了し、`set -u`を環境読み込み後へ移して再実行。

**制約**

GNGコアの直接API計測であり、ROS変換・TF・配信・viewer・外部分類器の時間は対象外。センサー座標のまま入力し、bag／YAML／学習条件を固定。CPU固定なし、既存処理との競合を含む測定であり、全ROS系のリアルタイム性能は未検証。

木探索の変更による実bagの視覚的品質評価は未実施。YAML値や探索の意味を変えずにグリッド版と同じグラフを生成する最適化ではない。

既定の`GNG_DETERMINISTIC_BENCHMARK=ON`は比較用。実時間動作用にはOFFで再ビルド。最新仕様は[独立版README](../../../ais_gng_cpu/experimental/gng_spatial_tree/README.md)。
