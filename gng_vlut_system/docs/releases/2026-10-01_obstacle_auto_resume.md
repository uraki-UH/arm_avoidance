# 2026-10-01 - 点群接近の保持と自動再開

- 要求: 障害物が離れたらBを押さずにGazebo回避へ復帰。Spaceなどの手動停止とは別の扱い。
- 観測: 点群余裕15.4 mmで`software_stop`、現設定の停止基準20 mm。再読取り時の余裕42.2〜53.7 mmでも停止ラッチを維持。[読取り結果](../../../artifacts/software_stop_recovery_20261001/live.json)。20 mmは作業開始時点の既存設定、今回の変更対象外。
- 変更: 共通設定へ`enable_obstacle_auto_resume: false`と`resume_clear_sec: 0.3`を追加。実環境入力設定では自動再開ON。実行中の点群接近は`state=running, phase=obstacle_wait`で接近時の姿勢を保持。目標余裕50 mmとGNG最寄り・直接隣接の安全継続後に同じ実行を再開、開始姿勢と実行世代を保持。
- 表示: `回避=障害物待ち（離れたら自動再開）`。通常端末の状態行・操作案内の2行構成を維持。
- 保持指令: 待機突入時の固定姿勢を周期送信。QPでの退避目標への差替えなし。新鮮な回避指令の監視は継続。
- 適用範囲: 点群近接のみ。Space・入力失効・自己干渉・関節異常は停止ラッチ継続。開始距離不足による拒否・空ROIの扱いは維持。既存停止ラッチを自動解除する変更なし。
- 単体: 171件成功。保持姿勢固定・同じ実行世代での再開・安全継続時間のリセット・隣接危険・手動停止・入力欠測・自己干渉・機能OFFと表示を検証。
- 隔離Gazebo: 接近→保持→点群除去→同世代で監視へ再開、待機中Space停止後のラッチ維持、待機中の点群入力欠測停止に成功。保持1秒間の最大関節変化0.000233 rad。水平開始姿勢・合成点群の入力切替による状態遷移試験、動く実障害物への回避性能は対象外。[結果](../../../artifacts/software_stop_recovery_20261001/gazebo/report.json)。
- 終了: 試験19.6秒、所有Gazebo・ROSノード残留なし、端末復元成功。既存launchの停止・解除・実機指令なし。反映は次回Gazebo launch起動時。

所有プロセスの起動コマンド（コンテナ内、ROSとworkspaceをsource後、すべて終了済み）:

```bash
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1 timeout -k 3 20 python3 /ros2_ws/src/artifacts/avoidance_live_response_20261001/probe.py
cd /ros2_ws/src/gng_vlut_system
PYTHONDONTWRITEBYTECODE=1 timeout -k 3 60 python3 -m pytest -q test/test_obstacle_auto_resume.py test/test_dual_arm_limits.py test/test_avoidance_timing.py test/test_gng_lidar_path.py test/test_dual_arm_control.py test/test_dynamixel_sim_keyboard.py test/test_pointcloud_avoidance.py test/test_topodualarm_launch.py
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1
timeout -s INT -k 30 180 python3 /ros2_ws/src/gng_vlut_system/test/check_obstacle_auto_resume.py --output /ros2_ws/src/artifacts/software_stop_recovery_20261001/gazebo
```


## 続報: 停止距離マージンの設定集約

- 変更: `pointcloud_avoidance_common.yaml`の`clearance_margins`へ距離余裕を集約。点群の開始・停止・待機・経路・QP下限を同じ`min_clearance_th`から導出。内部停止のコード固定値も`min_internal_clearance_th`から指定、計画内部余裕は`min_planning_clearance_th`。
- 現在値: 点群10 mm、内部停止5 mm、内部計画10 mm。作業開始時点の値を保持。退避・再開目標の`target_clearance`は別設定。
- 移行: 機体・入力ファイルの上書きも同じ辞書内へ移動。旧トップレベル距離項目の併記・不正値・計画余裕不足を起動時に拒否。共通launch以外の旧デモ設定は従来互換。[仕様](../pointcloud_avoidance.md)。
- 検証: 関連単体113件成功。設定上書き後の単一値反映、旧項目の競合拒否、内部停止距離の変更による状態遷移を確認。インストール先から前方伸展・旧ゼロ姿勢の両設定を読込み、10・5・10 mmへの展開を確認。既存NumPy/SciPy互換・非推奨警告3件。
- 反映・制限: インストール済みファイルはソースへのリンク、再ビルド不要。Gazebo launch再起動後の反映。今回はGazebo通し試験・実機動作なし、既存プロセスへの操作なし、有限の検証プロセスは終了済み。

検証コマンド（コンテナ内、ROSとworkspaceをsource後、終了済み）:

```bash
cd /ros2_ws/src/gng_vlut_system
PYTHONDONTWRITEBYTECODE=1 timeout -k 5 90 python3 -m pytest -q -p no:cacheprovider \
  test/test_pointcloud_avoidance.py test/test_obstacle_auto_resume.py \
  test/test_dual_arm_limits.py test/test_gng_lidar_path.py test/test_local_qp.py \
  test/test_viewer_environment.py test/test_topodualarm_launch.py
```
