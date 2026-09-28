# 2026-09-28 - 平面統計・時系列分散・可視化出力の削減

## 1. 要約

平面統計の旧寄与除去＋新寄与追加、ブロック単位の保持証明を追加。
初回試作では両方OFF。今回の作業開始時点の差分統計ONを維持し、保持証明はOFF。
境界候補の限定、成立不能な生成種の除外、入力読出しの共通化を適用。
判定条件・平面ID・ROSメッセージ定義は維持。保持ブロックを別クラスタとして出力する変更はなし。
先行の可視化削減では平面エッジ座標複製・MarkerArrayを既定OFF。続くトピック生成の依頼によりMarkerArrayのみ既定ONへ復帰。
所属・重心・法線・共分散・接平面基底・広がり・残差は保持。
連結証明の参照を配列添字からノードIDへ変更。無関係な点の増減・並べ替えによる全平面失効を除去。
所属集合・直前フレーム・親エッジの確認後に再利用し、保証不能な面は既存BFSへ復帰。
所属変更面の親エッジ検査、次数4までの汎用中央値選択、ROSノード列の重複読出しも削減。
続報では次数5〜8の中央値も固定比較網へ変更し、距離の値・下側中央値の定義を維持。
履歴用の座標・間隔・所属・IDは更新末尾で作業配列と交換し、全量コピーを省略。
CPU経路はGNGの既存ノード・接続配列を借用して平面処理へ入力。単独ROS入力も同じ処理本体を使用。
借用ビューの寿命は同期update呼出し終了まで。結果は自己所有、接続値はIDでなく入力配列添字。
他の分類・出力処理が使うROSグラフの作成、平面側の作業配列・CSR・全入力走査は残存。
このAPI変更単独の高速化は未確認。今回の改善をROS変換の撤去や完全差分処理とは扱わない。

CPU直結経路で試す場合、使用中のYAMLの`ros__parameters`に起動時設定：

```yaml
plane_cluster.enable_delta_statistics: true
plane_cluster.enable_block_retention: false
plane_cluster.num_acquisition_phases: 4 # 移籍・新規生成候補の分散周期[入力フレーム]
plane_cluster.enable_support_edges: false # エッジ座標複製の省略
plane_cluster.enable_temporal_update: false # 比較用の所属1パス方式
```

単独の`plane_cluster_incremental_node`では`plane_cluster.`接頭辞を除去。
設定後は対象ノードの再起動が必要。動的なパラメータ切替は非対応。
差分OFFで従来の再累積経路へ復帰。ブロックONは比較実験用で、本番既定ONは見送り。
`enable_temporal_update: true`では、そのフレーム先頭の再推定済み平面を固定して所属更新を1回。
所属変更後に平面を再推定し、保持条件の逸脱を再確認。分断・統合・法線EMAは従来どおり。
ON時は`maintenance_iter`不使用。獲得途中の残りは次入力へ継続し、従来との同一所属・ID系列は非保証。
安全確認付きの時間短縮は小さく、面数も増えたため既定OFF。比較用途の手動ONのみ。

周期は初回実装時の既定1、先行作業時点は手元の10、最新作業開始時点は5。その変更を上書きせず維持。上記4は比較用の設定例。
所属済みノードの移籍と新規平面の種選択のみ分散。成長先の点数制限はなし。
未所属点の既存面への取り込み、逸脱解放、統計更新、統合・分断、法線EMAは毎入力。
継続候補の選択機会は周期内に到来するが、クラスタ完成までの時間や数学的収束の保証ではない。
逸脱条件・EMAは従来どおり。分散による解放遅延は追加しないが、生の法線変化への即応保証ではない。

共通YAML・単独ノード既定は`enable_support_edges: false`、`enable_plane_markers: true`。
通常launchのCPU経路では既存平面を受信するmarkers-onlyモードで、平面判定の再計算なし。
`/plane_clusters/markers/hull`・`normal`・`nodes`のpublisherを作成し、同期グラフと平面からMarkerArrayを生成。
停止する場合は共通YAMLの`plane_cluster_incremental_node.ros__parameters.enable_plane_markers: false`。適用にはlaunchの再起動が必要。
非平面・曲面の機能は変更なし。平面トピックは維持するが`support_edges`配列は空。
エッジに依存する外部処理は`enable_support_edges: true`で互換出力を明示指定。
特に把持照合はエッジなしの場合、同一frame_id・frame_numberのTopologicalMapが必要。
同期元グラフを用意できない構成ではエッジ出力をONにすること。把持処理全体の実機確認は未実施。
`start_plane_cluster:=false`では可視化ノードも停止。CPU GNG単体の直接実行にはMarker生成ノードの起動が別途必要。
報告済み6.278 msは平面処理単体で、復帰したMarker生成・publish・通信・描画の時間を含まない。

## 2. 条件・検証

- Marker復帰は通常Release・install、関連147件成功。既定ONのhull／normal／nodesで非空データ受信、CPU出力を使うmarkers-onlyで平面publisherの非重複を確認。明示OFFでは3トピック不在、CPU／独立ROSの30入力完全一致（28入力非空）。試験6プロセスは全停止、追加デーモンなし。既存GNG・平面・bagのPID変更を作業中に観測したが起動停止操作は未実施、Viewer・3コンテナは維持。
- 復帰試験ログはコンテナ内`/tmp/plane_markers_{on,off}_20260928.log`、ビルドは`/tmp/plane_markers_build_20260928.log`、関連試験は`/tmp/plane_markers_ctest_20260928.log`。Marker込みの交差点全体時間は今回未計測。
- 最新の直接入力・固定中央値比較・履歴交換は旧版6.701→6.278 ms、p95中央値7.115→6.855 ms（5周期・交互5試行）。全150実入力・合成6条件×70入力の全出力一致。通常Release関連147件、ROS実点群30入力の独立経路一致を確認。5 ms目標は未達。
- 可視化エッジ削減の同条件比較は4周期8.171→7.262／10周期7.671→7.256 ms。エッジ以外の全フィールド一致。5 ms目標・総ノード数非依存は未達。[先行結果・条件・コマンド](../../../benchmarks/plane_consistency_20260924/PROFILE.md)。
- 連結再利用は旧新版5周期固定・交互5試行で7.055→6.777 ms、実150入力・合成6条件×70入力の全出力一致。p95改善は不安定であり、常時短縮の保証なし。
- 最新の1パス試作は旧版6.816／新版OFF7.075／ON6.644 ms（交互5試行）。OFFは全実入力一致、ONは平均所属0.30%減・面数64.58→66.65。分断事前判定は追加走査で遅くなり撤去。正解付き認識品質は未検証。
- 追加・削除・順序変更・所属移動・分断・統合・クラスタ添字圧縮・resetを検証。1パスONの合成6条件×70入力は旧版と全出力一致、実入力は上記の所属差あり。
- 最新の通常Release関連144件、分離ROSの1パスOFF／ON各15チェック成功。先行の省略／互換出力各15チェックでもエッジ有無による所属・幾何不変と平面Marker publisherの不在を確認。起動した試験ノードは全停止。
- `ClusterOptions`・`ClusterStatistics`のC++構造体を拡張。外部C++利用側も再ビルドが必要。ROSメッセージの再定義はなし。
- 入力差分探索・隣接構築・新規生成・出力に全体走査が残存。16bitノードID照合は従来どおりで、世代付き変更通知への接続や表示データ共有は未実装。
- 系列・座標系の切替では明示的resetが必要。任意の巨大座標・長期間運用・ライブGNG全体時間と認識品質は未検証。

Marker復帰の起動コマンド（`gng_cpu_container`内、ROSとinstallをsource後、`/ros2_ws/src`）：

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 PLANE_SMOKE_DELTA=1 PLANE_SMOKE_PHASES=5 PLANE_SMOKE_MARKERS_ONLY=1 PLANE_SMOKE_OUTPUT_DIR=/tmp/plane_markers_on_20260928 timeout -s INT -k 15 130 python3 benchmarks/plane_consistency_20260924/smoke.py
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 PLANE_SMOKE_DELTA=1 PLANE_SMOKE_PHASES=5 PLANE_SMOKE_MARKERS=0 PLANE_SMOKE_OUTPUT_DIR=/tmp/plane_markers_off_20260928 timeout -s INT -k 15 130 python3 benchmarks/plane_consistency_20260924/smoke.py
```
