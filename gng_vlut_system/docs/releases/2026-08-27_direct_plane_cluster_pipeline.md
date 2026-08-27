# CPU GNG・平面クラスタ直接接続

## 変更内容

- CPU版`ais_gng`の`gng_exec()`後に、同一プロセス・同一コールバック内で`Clusterizer::update()`を実行する経路を追加した。
- 平面クラスタ処理のための`/topological_map` publish・DDS serialize・subscribe・deserialize・別executor起床を削除した。`/topological_map`自体は既存利用者向けに従来どおりpublishする。
- `plane_cluster_incremental_node`へ`clusters_input_topic`を追加した。指定時はクラスタ計算を行わず、マップと既に計算済みのクラスタからhull・normal・nodeマーカーだけを生成する。
- `top_grasp_surface_estimator.launch.py`は表示ノードを既定でmarkers-onlyモードにし、CPU GNG側とのクラスタ計算の二重化を解消した。
- 平面クラスタパラメータ宣言を共通化し、直接経路は`plane_cluster.*`、単独ノードは従来名で同じ設定を利用する。
- backend用コンパイル定義の大文字・小文字不一致を修正し、CPU専用処理が実際に有効になるようにした。

## パラメータ

- `plane_cluster.direct_enabled` (CPU版既定: `true`)
- `plane_cluster.output_topic` (既定: `/topological_planar_clusters_incremental`)
- その他の平面クラスタ設定は`plane_cluster.min_cluster_nodes`など、既存名へ`plane_cluster.`を付けてCPU GNG側で指定する。
- 従来経路へ戻す場合はCPU GNGで`plane_cluster.direct_enabled:=false`とし、把持launchで`plane_clusters_input_topic:=''`を指定する。

## 検証

- Releaseビルド: `ais_gng`、`grasping_system`成功。
- 機能テスト: `test_plane_cluster_incremental` 8件、`grasping_system` 4件を含め成功。
- 合成Releaseベンチマーク: 2760 nodes / 5344 edges / 2000 iterationsで平均`0.294 ms`、p95 `0.314 ms`、p99 `0.389 ms`。
- 実点群20 Hz・20000点入力で、直接経路は約1480 nodes / 4000 edges、全処理`4.1～4.7 ms`、うち平面クラスタ`0.38～0.49 ms`。
- 従来ROS経路はGNG側`3.6～4.1 ms`に、別ノードの平面更新`0.60～0.66 ms`とクラスタ・マーカーpublish`0.43～0.50 ms`が加わった。直接経路ではクラスタ確定までのDDS配送・別executor待ちがなくなる。
- パッケージ全体lintは既存のcopyright・書式違反により失敗。今回の機能テスト失敗はなし。
