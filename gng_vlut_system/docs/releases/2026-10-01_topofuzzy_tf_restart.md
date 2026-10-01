# 2026-10-01 - TopoFuzzy配信のTF時刻後退警告

変更:

- 対象: `topofuzzy_bridge_node`。名前空間解決後の`frame_id == source_frame_id`ではTF Buffer・Listenerの生成なし。
- 原因: 座標変換不要でも共有TFを購読し、別系統のGazebo再起動による古い時刻のTFを蓄積済みBufferへ投入する構成。
- 維持: フレームの付替え・時刻の上書き・警告の全体抑制なし。異なる座標系のTF変換と既存の時刻後退時Bufferクリアは継続。

結果:

- 修正前: 同一座標・実時間で`TF_OLD_DATA` 20件、動的TF購読1件。
- 修正後: 同一座標・実時間／同一座標・シミュレーション時間／座標変換あり・シミュレーション時間の3条件成功。各条件で100→1秒、120→2秒の2回の時刻後退、警告0件。
- グラフ: 10,801ノードのID・接続・座標を検査。同一座標出力の不変、変換ありの場合の指定並進量を確認。同一座標時の動的・静的TF購読はともに0件。
- ビルド: Docker内Release対象ビルド成功。既存installのsymlink経由で新実行ファイルへ反映。
- 試験終了: 所有bridge・試験ドライバ・runnerの終了確認。既存実機・Gazeboへの指令・停止操作なし。試験中の既存GazeboのPID変更は本作業の操作外。
- 未検証: 修正後の実Gazebo再起動とViewerの通し操作。任意の異種時計TFを必要とする構成の分離は対象外。

反映: 追加引数なし。稼働中の旧`gng_viewer_bridge.launch.py`は自動更新されないため再起動が必要。Dynamixelドライバの再起動は不要。[現行仕様](../dynamixel_sim_control.md)。

検証コマンド（コンテナ内、ROSとworkspaceのsource後）:

```bash
cd /ros2_ws/src
python3 -B skills/run-benchmark-batch/scripts/run_batch.py artifacts/topofuzzy_tf_restart_20261001/baseline.json --output artifacts/topofuzzy_tf_restart_20261001/before --repeats 1 --timeout-sec 60 --max-total-sec 65 --estimate-sec 15
timeout 240s cmake --build /ros2_ws/build/gng_vlut_system --target topofuzzy_bridge_node -j2
python3 -B skills/run-benchmark-batch/scripts/run_batch.py artifacts/topofuzzy_tf_restart_20261001/fixed.json --output artifacts/topofuzzy_tf_restart_20261001/after --repeats 1 --timeout-sec 60 --max-total-sec 185 --estimate-sec 10
```

前提: `baseline.json`は修正前バイナリ専用。再実行時は未使用の出力ディレクトリ指定。各試験は`ROS_DOMAIN_ID=216`、bridge直接起動の実引数は各`output.log`に記録。[試験コード](../../test/check_topofuzzy_tf_restart.py)。
