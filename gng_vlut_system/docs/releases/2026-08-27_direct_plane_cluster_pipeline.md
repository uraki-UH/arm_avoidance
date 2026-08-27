# CPU GNG・平面クラスタ直接接続

## 変更内容

- CPU版`ais_gng`の`gng_exec()`後に、同一プロセス・同一コールバック内で`Clusterizer::update()`を実行する経路を追加した。
- 平面クラスタ処理のための`/topological_map` publish・DDS serialize・subscribe・deserialize・別executor起床を削除した。`/topological_map`自体は既存利用者向けに従来どおりpublishする。
- `plane_cluster_incremental_node`へ`clusters_input_topic`を追加した。指定時はクラスタ計算を行わず、マップと既に計算済みのクラスタからhull・normal・nodeマーカーだけを生成する。
- `ais_gng.launch.py`で平面マーカー変換を既定起動する。CPU版は内蔵クラスタ出力をmarkers-onlyモードで描画用トピックへ変換し、GPU版は独立ノードでクラスタ生成とマーカー変換を行う。
- `top_grasp_surface_estimator.launch.py`の表示ノードは既定OFFとし、`ais_gng.launch.py`と同時起動した際の二重化を防いだ。`ais_gng.launch.py`を使わない場合は`start_plane_cluster:=true`で従来経路を起動できる。
- 平面クラスタパラメータ宣言を共通化し、直接経路は`plane_cluster.*`、単独ノードは従来名で同じ設定を利用する。
- backend用コンパイル定義の大文字・小文字不一致を修正し、CPU専用処理が実際に有効になるようにした。
- 既存の`I: ..., Nodes: ..., Clusters: ...`要約行へ`GNG: ... ms, Plane: ... ms`を追加した。直接平面処理が無効な場合は`Plane: off`と表示する。
- 平面クラスタの所属変更を追跡し、解放・取り込み・移動・分割・併合のいずれも発生していない定常フレームで、全ノードからの平面再フィットを重複実行しないようにした。判定条件としきい値は変更していない。
- CPU GNGが計算済みの`rho=acos(mean(abs(n_i・n_j)))`を新規平面クラスタの種順序に再利用し、平面クラスタ側の重複した全edge法線内積走査を削除した。`rho`は所属・保持・分割・併合のハード判定には使わない。

## パラメータ

- `plane_cluster.direct_enabled` (CPU版既定: `true`)
- `plane_cluster.output_topic` (既定: `/topological_planar_clusters_incremental`)
- `plane_cluster.use_node_rho_for_seed_order` (CPU直結経路の既定: `true`)。独立ノードの`use_node_rho_for_seed_order`は古いbagとの互換性のため既定`false`。`ais_gng.launch.py`では`use_node_rho_for_seed_order:=true|false`で起動時に上書き、`auto`でYAMLまたはCPU直結側の既定値を使う。
- 実行中は`ros2 param set /ais_gng_node plane_cluster.use_node_rho_for_seed_order true|false`で、既存のクラスタ所属とIDをリセットせず次フレームから切り替えられる。
- その他の平面クラスタ設定は`plane_cluster.min_cluster_nodes`など、既存名へ`plane_cluster.`を付けてCPU GNG側で指定する。
- 従来経路へ戻す場合はCPU GNGで`plane_cluster.direct_enabled:=false`とし、`ais_gng.launch.py`で`plane_clusters_input_topic:=''`を指定する。
- `ais_gng.launch.py`:`start_plane_cluster` (既定: `true`)。平面クラスタの描画が不要な場合は`false`にする。
- `ais_gng.launch.py`:`plane_clusters_input_topic` (既定: `auto`)。CPU版では内蔵出力を使い、GPU版では空文字として独立計算する。

## 検証

- Releaseビルド: `ais_gng`、`grasping_system`成功。
- 機能テスト: `test_plane_cluster_incremental` 9件、`grasping_system` 4件を含め成功。
- 合成Releaseベンチマーク: 2760 nodes / 5344 edges / 2000 iterationsで平均`0.294 ms`、p95 `0.314 ms`、p99 `0.389 ms`。
- 不要な再フィット削減の同一セッション前後比較: 変更前平均`0.338 ms`、p50 `0.316 ms`、p95 `0.461 ms`。変更後3回の平均は`0.290 ms`、p50 `0.271 ms`、p95 `0.391 ms`で、それぞれ約14%、14%、15%低下した。
- `rho`再利用後3回の平均は`0.250 ms`、p50 `0.242 ms`、p95 `0.298 ms`。不要な再フィット削減後の`0.290 ms`からさらに約14%、当初の`0.338 ms`から合計約26%低下した。
- 実点群20 Hz・20000点入力で、直接経路は約1480 nodes / 4000 edges、全処理`4.1～4.7 ms`、うち平面クラスタ`0.38～0.49 ms`。
- 従来ROS経路はGNG側`3.6～4.1 ms`に、別ノードの平面更新`0.60～0.66 ms`とクラスタ・マーカーpublish`0.43～0.50 ms`が加わった。直接経路ではクラスタ確定までのDDS配送・別executor待ちがなくなる。
- パッケージ全体lintは既存のcopyright・書式違反により失敗。今回の機能テスト失敗はなし。
