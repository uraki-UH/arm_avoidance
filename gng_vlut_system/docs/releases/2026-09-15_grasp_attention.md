# 2026-09-15 - CPU GNGの把持候補近傍重点学習

後続変更: ノード半径方式を[物体候補AABB＋余白方式](2026-09-15_grasp_attention_aabb.md)へ置換。以下の半径・kd-treeの記載は初期実装の記録。

## 1. 要約

把持候補ノード近傍の実測点へ学習回数を配分する、既定OFFのCPUオプションを追加。[現行仕様・設定・起動方法](../../../ais_gng_cpu/docs/grasp_attention.md)を正本とする。

総学習回数を固定したまま、通常学習と重点更新を混合。通常枠内の既存混合比、勝者選択、実エッジ更新、学習係数は維持。

- launch引数`enable_grasp_attention`。`auto`はYAML、`true`/`false`は明示上書き。
- CPUライブラリの`gng_set_priority_input`。次の学習1回だけの実測点添字指定。
- 候補中心のkd-treeによる厳密半径検索。点群の追加ボクセル化・ROS再送なし。
- 候補の空入力・受信停止・時刻不一致・TF失敗・該当点なしの場合の通常学習復帰。

重点再学習を独立観測として数えないよう、重点分を共分散・支持統計・観測方向件数・統計用勝者イベントから除外。通常学習枠の既存統計は維持。

**削除**

既存オプション・APIの削除なし。

## 2. 条件・検証

既定OFFでは候補購読・追加TF購読・近傍検索なし。ON時のみ学習分布と通常統計の更新回数が変化。初期値は半径0.03 m、重点比率0.5、有効期間0.5秒。全項目は起動時設定。

入力は既存`/grasp_pose_cands/Tmap`（`ais_gng_msgs/msg/TopologicalMap`）。新ROSトピック・メッセージなし。新パラメータは仕様書の表を参照。既存ライブラリ構造体の変更なし、新CPU ROSノードには更新したライブラリが必要。

<a id="verification"></a>

Dockerで以下を実行。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 20s 600s colcon build --packages-select gng_cpu ais_gng --symlink-install --parallel-workers 1 --cmake-args -DGNG_BUILD_BENCHMARKS=ON -DBUILD_TESTING=ON && timeout -s INT -k 10s 120s ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure -R "gng_priority_input_api_test|gng_observation_api_test" && timeout -s INT -k 10s 120s ctest --test-dir /ros2_ws/build/ais_gng --output-on-failure -R "^test_grasp_attention$" && timeout -s INT -k 15s 120s python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_grasp_attention_ros.py'
```

- `gng_cpu`・`ais_gng`のReleaseビルド成功。既存のPCL/Torch/CMake警告あり。
- CTest 3対象に成功。重点比率0.5時、通常勝者イベントが1000から500へ減少。共分散件数・観測方向件数に重点分の追加なし。入力置換・実行完了・不正指定後の重点状態失効を確認。
- ROS domain 219の合成データで、OFF時の候補購読なし、ON時の抽出、異なるframeからのTF、空候補・古い/未来時刻・TFなし・該当点なし・受信停止時の通常Graph配信を確認。
- 初回ROSビルドのTFローカル変数参照、APIテストの初期化後学習回数設定を修正し、再検証成功。
- 5000候補中心・10万点の合成入力で索引構築と抽出を測定。空間ハッシュ版43.8318 ms、採用したkd-tree版20.3293 ms。両者42178点を抽出。総当たりとの照合、重複中心・空入力・不正値も検証。単回の測定であり実環境性能保証なし。

テスト専用CPUの起動コマンド（`enable_grasp_attention`をfalse/trueで各実行）:

```bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --ros-args --params-file /ros2_ws/src/ais_gng_cpu/src/ais_gng/config/gng_cpu/graspnet.yaml \
  --log-level ais_gng_node:=debug \
  -p enable_grasp_attention:=true -p node.num_max:=256 -p node.learning_num:=200 \
  -p input.point_cloud_num:=5000 -p 'input.topic_names:=[/attention_test_points]' \
  -p classify.human:=false -p classify.car:=false -p node.enable_observation_support:=false
```

同じPythonスクリプト内に`grasp_attention_test`とTF broadcasterを作成。finallyで専用CPUとPythonノードを終了。終了後のプロセス一覧で残留なしを確認。既存CPU PID 492092・Viewer PID 419809を維持。把持推定器の外部再起動を観測したが、本作業からの停止・再起動操作なし。

launch引数のインストール先での確認（ノード起動なし）:

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 30s ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml enable_grasp_attention:=true --show-args'
```

**制約**

実物の把持成功率・最適パラメータは未検証。近傍検索分の追加コストあり。現在は単一入力点群専用、複数入力は通常学習へ復帰。候補の運動予測・未観測形状生成・ノード密度の強制増加なし。詳細は仕様書を参照。
