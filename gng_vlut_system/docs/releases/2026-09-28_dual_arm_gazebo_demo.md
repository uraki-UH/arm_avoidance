# 2026-09-28 - 双腕の本番学習とGazebo関節動作デモ

## 1. 要約

`topo_dual_arm_max`のGNG・VLUT学習が完了。Gazebo Classic + ros2_controlによる双腕デモを追加した。

- 学習は既定設定の100万回、追加学習10万回、1000ノード。バックグラウンドで約438秒後に正常終了。
- 保存先は`gng_results/topo_dual_arm_max/{gng.bin,vlut.bin}`。VLUT幅0.02 m。衝突検査による削除は0ノード・0エッジ。
- デモは左腕→ホーム→右腕→ホーム→両腕→ホーム→グリッパー開→ホームの1巡。既定では開始待ち。
- 腕14関節と腰・首・グリッパー5関節を位置制御。腰・首はゼロ姿勢を目標とし、指のmimicはGazebo側で追従。
- 操作は`sim_<機種名>`内の`demo/start`・`demo/stop`・`demo/status`。実機ドライバへの接続なし。
- Gazebo実測joint_statesをToPoFuzzy Viewerへ配信。外部指令との競合を避け、仮想関節ドライバは不使用。
- 指令時の現在角から補間し、URDF範囲外・非有限角・不明関節を拒否。状態失効時は軌道取消とfault表示。再開は明示操作。
- GNGによる経路生成・自動回避・物体把持はデモへ未接続。慣性・摩擦・トルクと実機の一致は未検証。

Docker内で起動（自動で1巡する場合）：

```bash
ros2 launch gng_vlut_system dual_arm_gazebo_demo.launch.py enable_auto_start:=true
```

`gui:=false`でGazebo画面なし。longは`params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml`を追加。通常のROSドメインでViewer backendが起動済みなら`sim_<機種名>`のロボットが配信される。

開始・停止（別ターミナル）：

```bash
ros2 service call /sim_topo_dual_arm_max/demo/start std_srvs/srv/Trigger '{}'
ros2 service call /sim_topo_dual_arm_max/demo/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /sim_topo_dual_arm_max/demo/status
```

[デモ設定](../../config/dual_arm_gazebo_demo.yaml)で姿勢、補間時間、指令速度、繰り返し、GUI・Viewer表示を調整。実機用の非常停止とは別の、シミュレーション軌道取消。

## 2. 条件・検証

| 検証 | 結果 |
| --- | --- |
| maxの途中停止・再開・全8姿勢 | 成功。3925関節状態受信、復帰誤差0.000150 rad |
| longの同操作と状態配信失効 | 成功。4211関節状態受信、復帰誤差0.000150 rad |
| long停止後 / Gazebo再開後の角度変化 | 最大0.000150 / 0.000150 rad。fault維持、動作の自動再開なし |
| 実測最大速度 | max 0.182、long 0.192 rad/s。補間指令上限0.15 rad/sは実測値の保証ではない |
| URDF姿勢検査 | 3テスト成功。両モデル対応、関節制限・不正値・空姿勢の拒否 |
| 学習済みViewer配信 | 1000ノードのTmap、URDF、21関節姿勢を実受信 |

実GUIの描画確認は未実施。Gazebo実動作検証はheadless、ROS_DOMAIN_ID=98、GAZEBO_MASTER_URI=http://127.0.0.1:11359。Viewer検証はdomain 99、学習はdomain 97。全起動プロセス終了。既存コンテナ・Viewerの停止・再起動操作なし。作業中にfrontendの起動時刻変化を観測。

実行コマンド（`ROS_LOCALHOST_ONLY=1`、ROS環境をsource済み）：

```bash
ROS_DOMAIN_ID=97 ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml
ROS_DOMAIN_ID=98 python3 /ros2_ws/src/gng_vlut_system/test/check_dual_arm_gazebo_demo.py \
  --output /ros2_ws/src/artifacts/dual_arm_demo_20260928/integration
ROS_DOMAIN_ID=98 python3 /ros2_ws/src/gng_vlut_system/test/check_dual_arm_gazebo_demo.py \
  --params-file /ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml \
  --namespace sim_topo_dual_arm_max_long --output /ros2_ws/src/artifacts/dual_arm_demo_20260928/integration_long
```

試験スクリプトが`ros2 launch gng_vlut_system dual_arm_gazebo_demo.launch.py gui:=false enable_auto_start:=false gazebo_master_uri:=http://127.0.0.1:11359`を指定機種で起動し、終了時に停止。max試験後の取消結果判定補強はlong試験で検証。Viewerは[モデル設定文書](2026-09-28_dual_arm_models.md)の起動コマンドを一時実行して停止した。

根拠はGit除外の`artifacts/dual_arm_demo_20260928/`内の`training_status.json`・`training.log`・各`report.json`・`trained_viewer.log`。依存4パッケージをコンテナへ導入しDockerfileにも反映。旧URDFディレクトリ不在による初回install失敗は、存在時だけインストールする形へ修正し再実行成功。生成installマニフェストをGit管理から除外。

物理値・位置指令とトルクPIDの違いは[物理設定メモ](../dual_arm_simulation.md)、人腕接近の退避は[専用デモ](2026-09-28_dual_arm_avoidance_demo.md)を参照。

制御接続は[Gazebo ros2_control公式仕様](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)、軌道指令は[JointTrajectoryController公式仕様](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)に準拠。
