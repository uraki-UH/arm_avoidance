# 2026-09-15 - 把持重点入力候補の点群配信

## 1. 要約

把持アテンションの選択点を`/downsampling/grasp`で可視化可能。[現行仕様](../../../ais_gng_cpu/docs/grasp_attention.md#選択点群の可視化)を更新。

CPUの重点学習ON時のみPointCloud2 Publisherを生成。購読時だけ既存の選択添字とXYZ変換処理を再利用して点群化。追加の検索・点群ボクセル登録・設定項目なし。

## 2. 条件・検証

GNGコア・学習配分・選択条件は変更なし。入力処理ごとの配信で、候補なし・失効・TF失敗・対象点なしは空点群。入力停止時の独立した消去タイマーなし。購読者なしではXYZメッセージ生成・配信なし。

- 新規出力: `/downsampling/grasp`、`sensor_msgs/msg/PointCloud2`、XYZのみ。
- QoS: best_effort、volatile、depth 1。名前空間内の相対トピック名。
- 座標系・時刻: `/topological_map`と共通のヘッダ。
- 有効化: 既存の`enable_grasp_attention: true`とGNG再起動。新パラメータなし。

Dockerビルド、AABB単体テスト3件、ROS domain 219の隔離テストに成功。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 20s 600s colcon build --packages-select ais_gng --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON && timeout -s INT -k 5s 60s /ros2_ws/build/ais_gng/test_grasp_attention && timeout -s INT -k 15s 120s python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_grasp_attention_ros.py'
```

検証スクリプト内のCPU起動コマンド（`enable_grasp_attention`をfalse/trueで各1回）:

```bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 \
  /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu --ros-args \
  --params-file /ros2_ws/src/ais_gng_cpu/src/ais_gng/config/gng_cpu/graspnet.yaml \
  --log-level ais_gng_node:=debug \
  -p enable_grasp_attention:=true -p grasp_attention.margin:=0.035 \
  -p node.num_max:=256 -p node.learning_num:=200 \
  -p input.point_cloud_num:=5000 -p 'input.topic_names:=[/attention_test_points]' \
  -p classify.human:=false -p classify.car:=false \
  -p node.enable_observation_support:=false
```

OFF時のPublisherなし、購読なしの重点学習継続、選択点のXYZ・点数・データ配列・時刻・座標系、候補TF変換、別候補間の非採用、失効・空候補・TF失敗時の空点群を確認。既存AABB抽出テストも成功。実Viewerの目視・配信負荷の定量測定は未実施。

検証スクリプトはfinallyで専用CPUを停止し、Python ROSノードも終了。終了後のプロセス一覧で検証ノード・追加ROSデーモンの残留なし、コンテナ起動状態と既存CPU PID 1061427・Viewer PID 419809の維持を確認。既存計画系PID 497596/497597/497599の終了を観測したが、本作業からの停止・再起動操作なし。

**制約**

配信点はAABB内の重点入力候補であり、GNG内部の入力範囲フィルタ適用後の最終点集合や各反復の抽選履歴ではない。周辺の床等を含む可能性は既存AABB方式と同じ。稼働中GNGへの自動反映なし。
