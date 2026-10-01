# 2026-10-01 - 左腕前方伸展の開始姿勢とGazebo自己ボクセル

## 追記: 前方斜め下45°への変更

- 変更: Gazebo専用設定の左肩`L_joint1`を−90°から−45°（−0.7853981633974483 rad）へ変更。ほかの関節・速度・回避距離は保持。復帰目標は従来どおり回避開始時の実測姿勢。
- 幾何検証: 肘からグリッパ基部への向きは水平から下45°、手先位置はルート基準`(0.241477, 0.138750, 0.251943) m`。内部干渉検査成功。
- 単体検証: コンテナで11 / 11件成功。ホストでの初回実行は`control_msgs`不足による収集失敗。新姿勢でのGazebo通し動作は未検証。
- 反映: 次回launch起動時、既存のsymlink-installでは再ビルド不要。既存Gazebo・実機への操作なし。従来の水平伸展試験結果は以下に保持。
- 試験コマンド: コンテナ内でROS・workspaceをsource後、`PYTHONDONTWRITEBYTECODE=1 timeout 60s python3 -B -m pytest -q -p no:cacheprovider gng_vlut_system/test/test_topodualarm_launch.py gng_vlut_system/test/test_dual_arm_avoidance_geometry.py`。終了済み、新規常駐ノードなし。

## 旧設定: 水平前方伸展

変更:

- 開始姿勢: `dynamixel_sim_control.launch.py`のGazebo既定を左肩−90°・他関節0へ変更。旧ゼロ姿勢YAMLは保持。開始姿勢の指定・復帰先・旧版選択は[操作仕様](../dynamixel_sim_control.md)。
- 初期化: URDFの可動域・有限値・mimic検査後、モデル生成中の一時停止時だけ関節位置を設定。物理開始後の追従は有限トルクモータ。
- 保持: 同一位置目標の繰返し送信を抑止。回避軌道受理・停止時に保持目標の送信履歴を解除。停止・実測失効の監視期限は従来値。
- 自己形状: 実環境Tmap利用時もGazebo実測関節から`/sim_ToPoDualArm/self_voxel`を生成。旧仮想カメラ入力の実機マスクを`/sim_ToPoDualArm/real/self_voxel`へ分離。[入力と表示の区別](../realsense_gazebo.md)。

検証:

- Releaseビルド成功、関連単体164件成功。既存CMake・SciPy/NumPy警告あり。
- 前方伸展のURDF手先位置: ルート基準`(0.326, 0.13875, 0.487) m`。初期姿勢の自己干渉検査成功。
- 初回3試行: 前方伸展の生成成功、回避切替の静止確認は失敗。切替待ち中の手首速度最大0.038828 rad/sを確認。同一保持目標の補間再開始を抑止後、[trial4](../../../artifacts/left_forward_gazebo_20261001_trial4/report.json)でA開始・Space実測停止・B解除成功。開始実測`L_joint1 = −1.569545 rad`、左腕変化0.005303 rad時点で停止。全軌道完走ではない開始試験。
- 前方伸展の最終試験: 左腕実測変化0.101515 radまで回避を継続後、Space実測停止・B解除に成功。右腕変化0.000281 rad。[trial5](../../../artifacts/left_forward_gazebo_20261001_trial5/report.json)。実機出力OFF、試験ノード・ポート残留なし。
- 初回試行の終了処理中に`dual_arm_control.py`のROSメッセージ変換例外を観測。試験プロセス・ノード・ポートの残留なし。
- 自己形状の隔離Gazebo試験: 実環境入力経路で自己ボクセル1,749件、開始時との差分最大1,151セル、固定した実機側マスクの差分0セル。退避・復帰、実機関節欠測・点群欠測の停止、Viewer実測一致1,235件を確認。[結果](../../../artifacts/gazebo_self_voxel_20261001_trial1/report.json)。この試験のGazebo開始姿勢は旧ゼロ姿勢。

試験コマンド（コンテナ内、ROSとworkspaceをsource後）:

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_dynamixel_gazebo_start.py \
  /ros2_ws/src/artifacts/left_forward_gazebo_20261001_trial5
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 ROS2CLI_NO_DAEMON=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_viewer_environment_gazebo.py \
  --output /ros2_ws/src/artifacts/gazebo_self_voxel_20261001_trial1
```

試験での実際のlaunch: [command.json](../../../artifacts/left_forward_gazebo_20261001_trial5/command.json)。実機指令publisherは0、所有試験停止済み。既存ユーザーGazeboへの変更は次回起動から適用。
