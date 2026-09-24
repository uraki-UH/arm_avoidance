# 2026-09-15 - 把持物体候補AABBによる重点学習

## 1. 要約

ノード半径方式を、物体候補ごとのAABB＋余白方式へ置換。[現行仕様](../../../ais_gng_cpu/docs/grasp_attention.md)を更新。

`/grasp_pose_cands/Tmap`の`clusters[].nodes`を元ノードIDとして解決し、TF変換後の所属ノードからAABBを構築。各方向へ`grasp_attention.margin`を加算し、いずれかの範囲内の実測点を重点対象とする。

候補内部のノードから遠い点、別候補間の空間、未所属・不正所属・重複・余白0の回帰テスト。

近傍ノードとの距離によらず候補物体内部を重点対象へ包含。別候補をまとめた巨大なAABBの生成なし。

**削除**

ノード単位のkd-treeと`grasp_attention.radius`。旧YAMLの同キーは`grasp_attention.margin`へ置換が必要。

## 2. 条件・検証

- 既定OFF・重点比率0.5・有効期間0.5秒を維持。余白は0.03 m、0も許可。
- 未所属ノードは範囲に含めず、クラスタなしは通常学習。クラスタ内の参照先不明・不正座標は該当クラスタを除外。Graph内の重複IDは全範囲を解除。
- 総学習回数、重点分の観測統計への非計上、候補失効・TF失敗の処理は維持。GNGコアの変更なし。
- 実物形状に沿う切り抜きではないため、候補AABB内の床や別物体の点も対象になり得る。

新ROSトピック・メッセージなし。パラメータ変更は`grasp_attention.radius`から`grasp_attention.margin`への置換のみ。設定反映にはGNG再起動が必要。

Docker内のReleaseビルド、AABBテスト3件、既存GNG APIのCTest 2対象、隔離ROSテストに成功。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 20s 600s colcon build --packages-select ais_gng --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON && timeout -s INT -k 5s 60s /ros2_ws/build/ais_gng/test_grasp_attention && timeout -s INT -k 5s 60s ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure -R "gng_priority_input_api_test|gng_observation_api_test" && timeout -s INT -k 15s 120s python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_grasp_attention_ros.py'
```

合成データの10候補・5000ノード・10万入力点で、AABB構築＋点抽出1.42386 ms、選択279点。単回測定であり、旧kd-treeテストとは入力分布・選択範囲が違うため直接の速度比較ではない。ROS側では遠いノード間の候補内部点の採用、別候補間の非採用、クラスタなし・空候補・時刻不整合・TFなし・受信停止時の通常出力を確認。

検証スクリプトが起動するCPUノードのコマンドは[初期実装の検証記録](2026-09-15_grasp_attention.md#verification)と同じ。ROS domain 219、入力`/attention_test_points`、`enable_grasp_attention`のfalse/trueで実行。finallyでCPU・Python検証ノードを停止。終了後のプロセス一覧で残留なしを確認。既存CPU PID 814657・Viewer PID 419809は維持。

**制約**

既存プロセスの停止・再起動なし。実物の把持成功率、実環境での最適余白、Viewer目視は未検証。未所属点群から物体領域を推測する代替処理なし。
