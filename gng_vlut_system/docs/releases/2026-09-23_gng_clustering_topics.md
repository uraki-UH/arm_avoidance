# 2026-09-23 - クラスタリングOFF時の通信口抑止

## 1. 要約

平面・曲面クラスタリングのOFFを計算だけでなく関連ノード・通信口へ反映。通常構成の両方OFFで、平面・非平面・曲面時間の空トピックを未生成。`/topological_map`は継続更新。

- `ais_gng.launch.py`の自動平面入力で平面OFF時に可視化・曲面ノードを起動せず、保存ノードの平面購読も無効化。
- CPUの非平面Publisherを平面計算ONに連動。曲面時間のSubscriberは`surface_model.enable`が有効な場合だけ生成。launchでは曲面ノード未起動時も時間購読を無効化。
- Viewerの補助平面購読を発行元の存在に連動。既存の1秒周期確認で接続・解除し、停止時の平面キャッシュも破棄。

- launch条件の回帰テストを15件へ拡張。OFF/ON、設定優先順位、外部入力、独立再計算、GPU起動条件の検証。
- 実ROSのOFF→平面ON→曲面ON→OFFテスト。各構成で非空`/topological_map`の3フレーム以上の更新を必須化。Viewer・保存ノードを含む通信口と停止後のトピック消滅の確認。

- 平面計算OFFでも可視化Publisher・非平面Publisher・保存用購読が残る不整合。
- 曲面OFFでも計算時間の購読が残る不整合。
- 今回の作業途中で発生した、未生成の曲面時間SubscriberへのNULL参照。通常GNGの起動ログでexit code -11を確認し、ログ処理にも未生成判定を追加。修正後は点群入力と継続マップ更新で検証。

**削除**

機能・トピック名・メッセージ定義の削除なし。無効機能の不要な通信口だけを抑止。

## 2. 条件・検証

- 設定変更は従来どおりGNG launch再起動後に適用。既存のViewerプロセスは補助購読を常設しているため、今回の更新にはViewerの再起動も必要。
- `start_plane_cluster:=false`の追加は通常の両方OFFでは不要。平面ONで可視化・曲面だけを停止する用途は維持。
- `plane_clusters_input_topic`を明示した外部平面入力・独立再計算は維持。外部発行元・別ノードの購読まで停止する処理はなし。
- Viewerの選択済みStreams項目の保持・復帰時の再購読は従来仕様。ROSトピック消滅とUIの選択項目保持は別。

`/plane_clusters`、`/plane_clusters/markers/*`、`/nonplane_components`、`/curved_surface_clusters/update_ms`などの生成条件だけの変更。既存の`plane_clustering`・`curve_clustering`を使用し、設定値やしきい値の編集なし。WS v2のフィールド・バイナリ形式の変更なし。

- 変更前の追加launchテストは、両方OFFでも3ノード起動する条件で失敗。変更後の全15件が成功。
- GNG CPUコンポーネントのReleaseビルド、Viewerバックエンドのcolconビルド、Frontendのlint・buildが成功。検証ビルドは途中で並列数を1へ抑えて再実行。既存の警告とFrontendのチャンクサイズ警告あり。
- Viewer CTest 3件が成功。非平面の全6到着順・元ノードID・エッジ・再接続・空成分・購読解除の実ROS/WSテストが成功。既存テストの旧84-byte前提を現行TMG1 version 2の96-byteへ修正。
- 曲面時間ログの`off`→`--`→計算時間→`off`が成功。曲面OFF・人/車推論ONの所属回帰テストでも5フレームの出力を確認。
- 隔離ビルドと通常インストール先の両方でOFF/ONの4構成が成功。通常CPUコンポーネントとViewer実行ファイルを更新済み。
- 通常環境の復旧確認時点ではGNG発行元が0だったため、実bagの再起動後確認とは区別。利用者の既存プロセスの停止・再起動操作なし。

主な検証用起動コマンド（コンテナ内、ROSと通常workspaceのsetup読込後）：

```bash
timeout --signal=INT --kill-after=5 30 python3 -B \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_clustering_yaml_launch.py
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  timeout --signal=INT --kill-after=15 150 python3 -B \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_clustering_topics_ros.py
VIEWER_GATEWAY_EXECUTABLE=/tmp/gng-clustering-topics-OkY7fC/viewer_build/topo_fuzzy_viewer/viewer_ws_gateway_node \
  timeout --signal=INT --kill-after=10 60 python3 -B \
  /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_nonplane_stream.py
ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1 \
  LD_LIBRARY_PATH=/tmp/gng-clustering-topics-OkY7fC/ais:$LD_LIBRARY_PATH \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 timeout --signal=INT --kill-after=10 80 python3 -B \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/surface_timing_ros_test.py \
  --executable /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu
```

統合テストが起動するGNGは`ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=<一時YAML>`、Viewerは`viewer_ws_gateway_node --ros-args -p port:=<空きポート>`。検証プロセスは全終了。一時ビルド成果物61 MBは削除済み（再生成可能）。

**制約**

GPU実行、実ブラウザ表示、実bagによる長時間検証は未実施。人検出のフィードバック・確定条件は今回の修正対象外。一般の外部Subscriberが残る場合は、同名トピックがROSグラフに存在する可能性あり。
