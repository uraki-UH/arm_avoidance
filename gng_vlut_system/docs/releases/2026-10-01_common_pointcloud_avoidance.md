# 2026-10-01 - 機体設定による共通Gazebo点群回避

変更:

- 起動入口: `pointcloud_avoidance.launch.py`。機体YAMLによるURDF・GNG/VLUT・任意名の計画関節グループ・LiDAR・ROIの選択。
- 対応設定: ToPoDualArm左腕7関節、max / max_long双腕14関節。ルートリンクの自動取得、外装・指の監視対象追加、GNG関節数の検査。
- 既存入口: `dual_arm_control.launch.py robot:=topodualarm`の幾何回避は維持。共通構成への移行はlaunch名変更。

結果:

- Releaseビルド・install: 成功。Python回帰: 399 / 399 件成功。既存環境のSciPy / NumPyバージョン警告あり。
- ToPoDualArm通し試験: 成功。仮想LiDAR点群・自己除去・VLUT危険判定・GNG経路利用・退避復帰・LiDAR欠測時停止。実機出力なし。
- 最小推定クリアランス: 0.052301 m。最大関節変位: 1.312871 rad。GNG経路由来の指令選択: 85 回。局所補正: 53 回。通し試験壁時間: 78.39 s。
- 初回失敗: ROI外の追加余白0.2 mによる床ボクセル混入。指先の点群クリアランス約-0.008 mで開始時停止。共通構成だけ追加余白を0に設定後、通し試験成功。床の形状監視は維持。
- 試験終了: 全3回の所有プロセス回収・専用ROSドメイン空・端末復元に成功。後続のユーザー表示確認用デモは明示依頼により起動継続。既存Viewerを利用。
- 表示確認: Gazebo GUIウィンドウ生成、既存ViewerのWebSocketで`sim_ToPoDualArm`のdescription・pose受信に成功。ブラウザは`http://localhost:5173`を表示。

制限: 固定基台・同梱world・初期関節角0。ToPoDualArmの検証は左腕のみ。max系はこのPCの学習データ未配置で通し試験未実施。任意機体の動作保証なし。

根拠: `artifacts/common_pointcloud_20261001/{first,diagnostic,roi}/report.json`。成功試験の完全なlaunch引数は`roi/command.json`。
詳細: [共通設定・機体追加・試験起動コマンド](../pointcloud_avoidance.md)。

表示確認用起動（コンテナ内、ROS環境読込み後）:

```bash
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 ROS2CLI_NO_DAEMON=1 \
  ros2 launch gng_vlut_system pointcloud_avoidance.launch.py gui:=true enable_viewer:=true
```

表示ログ: `artifacts/common_pointcloud_20261001/interactive/ros_logs`。既存Viewerのdomain 25に整合。初回domain 0での配信待ち失敗後、表示デモだけ再起動。操作用ターミナルでA開始、Space停止、Ctrl+C終了。Viewer: `http://localhost:5173`、機体名`sim_ToPoDualArm`。
