# 2026-09-28 - 人の前腕接近に対する双腕退避デモ

## 1. 要約

Gazeboで人の前腕を模したカプセルが左右の腕へ順番に接近し、距離に応じて退避・復帰するデモを追加。
このlaunchは実測関節角とURDF外接球による局所探索。[点群・GNG/VLUT接続版](2026-09-28_dual_arm_gng_lidar.md)を別launchで追加。実カメラの人検出との接続は未実装。
通常の関節動作デモとは同時実行せず、専用launchで起動。

```bash
ros2 launch gng_vlut_system dual_arm_avoidance_demo.launch.py
```

起動後に自動開始し、左→右を1巡。`gui:=false`でGazebo画面なし。
longは`params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml`を追加。
自動開始を止める場合は`enable_auto_start:=false`。

```bash
ros2 service call /sim_topo_dual_arm_max/avoidance/start std_srvs/srv/Trigger '{}'
ros2 service call /sim_topo_dual_arm_max/avoidance/stop std_srvs/srv/Trigger '{}'
```

Viewerは`sim_<機種名>`のロボットと`/sim_<機種名>/avoidance/markers`を表示ON。
橙色が前腕、緑・紫が左右手首の実測軌跡、距離線と文字が外接形状間の余裕・動作状態。
距離不足や情報失効時は停止し、自動再開なし。状態詳細は`/sim_<機種名>/avoidance/status`。

## 2. 条件・検証

| 項目 | 内容 |
| --- | --- |
| 設定 | [回避YAML](../../config/dual_arm_avoidance_demo.yaml)。接近位置・速さ・半径・余裕を変更可能 |
| 接近条件 | 前腕長0.35m、半径0.045m。手位置x=0.45→0.03m、y=±0.36m、z=0.38m |
| 時間 | 各側：接近24s・近接保持3s・後退12s・復帰待機12s。シミュレーション時刻 |
| 距離 | 目標`target_clearance=0.12`m、停止下限`min_clearance_th=0.035`m。目標は軟らかい評価項 |
| 指令 | 19関節の位置補間。探索は腕14関節、移動幅を制限。外接球・床・固定作業台で候補と中間姿勢を確認 |
| 入力 | Gazeboのjoint_statesとmodel_states。更新失効1sで障害物更新・関節軌道を停止 |
| 可視化 | Marker 7個、各手首軌跡250点まで。実測姿勢を既存Viewer bridgeから配信 |
| 幾何試験 | max / long × 左右で退避成立・包囲・関節移動幅・内部余裕を検証 |
| max実Gazebo | 左右退避・復帰、状態失効停止、停止サービス成功。最小推定余裕0.0803m、初期姿勢の反実仮想−0.06129m |
| max処理時間 | 状態更新1回の平均14.02ms / 最大75.07ms |
| long実Gazebo | 左右退避・復帰、状態失効停止、再開後fault維持、停止サービス成功。最小推定余裕0.0950m |
| long反実仮想 | 同じ障害物位置と初期姿勢の外接距離は最小−0.06295m。退避なしでは外接形状が重なる条件 |
| long処理時間 | 状態更新1回の平均13.89ms / 最大68.63ms。待機周期0.15sとは別 |
| Viewer試験 | maxの姿勢・前腕・距離、longの7マーカーと手首軌跡をWebSocketで実受信。実GUIの目視確認は未実施 |

距離は外接球とカプセルの間の推定値。三角形メッシュの接触力・人体への衝撃の評価ではない。
自己干渉監視は隣接リンクと初期姿勢ですでに外接球が重なる組を除外した近似。連続軌道の安全保証なし。
モデルが変わる場合は固定作業台の形状と外接球の適用範囲も確認が必要。
トルクPID・質量・慣性・摩擦の扱いは[物理設定メモ](../dual_arm_simulation.md)を参照。

再現：コンテナでROS環境をsourceし、`ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1`を設定。

```bash
python3 /ros2_ws/src/gng_vlut_system/test/check_dual_arm_avoidance_demo.py \
  --output /ros2_ws/src/artifacts/dual_arm_avoidance_20260928/max_final
```

longの試験は`--params-file /ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml --namespace sim_topo_dual_arm_max_long`を追加。
試験は専用Gazeboポート11359、headlessで起動し、finallyで終了。
Viewer配信試験は起動中に`test/check_dual_arm_avoidance_viewer.py --output <保存先>`を実行。専用gatewayを起動・終了。
幾何試験は`python3 -m unittest discover -s gng_vlut_system/test -p test_dual_arm_avoidance_geometry.py`。

結果はGit除外の`artifacts/dual_arm_avoidance_20260928/{max_final,long}/report.json`と`viewer_long_retry/viewer_report.json`。
初回はサービス取得の失効と実時間／ROS時刻の補間更新不整合で停止。model_states購読・ROS時刻での更新へ修正。
試験側の停止応答後の旧fault受信、Viewerの検出前購読も修正後に成功。既存通常デモの単体3件と幾何試験も成功。
全試験launch・gateway終了、専用ポート11359の解放を確認。既存ROSプロセス・3コンテナの停止操作なし。
