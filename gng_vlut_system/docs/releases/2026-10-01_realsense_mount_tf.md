# 2026-10-01 - RealSenseの頭部取付TF

- 追加: `realsense_mount_tf.launch.py`と`realsense_mount.yaml`。`ToPoDualArm/camera_link → camera_link`の固定TFと、親子frame・取付xyz/rpyの上書き。
- 同時起動: `ToPoDualArm.yaml`の`enable_realsense_mount_tf: true`を`gng_viewer_bridge.launch.py`で読込み、通常のViewerコマンドから取付TFを追加。`realsense_mount_config`で取付設定を選択。同時起動の無効化・未指定機体の既定OFFに対応。
- 配置根拠: URDFのカメラと同じ位置・向きというユーザー確認。取付補正ゼロ。RealSense内部の光学TFは既存ドライバに委任。
- 検証: Releaseビルド・install成功。隔離domain 96でlaunchから位置0・単位回転のTFを実受信。既存CMakeのPkgConfig・PCL_ROOT警告あり。
- 同時起動検査: ToPoDualArmでON、明示falseでOFF、max既定OFFの3条件と取付launch読込みに成功。初回2回は確認スクリプトのlaunchパス取得方法の誤りで失敗、取得方法修正後に成功。同時起動変更後のbuild/install成功、ROSノードの追加起動なし。
- 終了確認: 初回はSIGINT終了直後のDDS残留により未確認判定。再確認でlaunch PID 62597・配信PID 62598の消滅とdomain 96の他ノードなしを確認。所有試験プロセス終了済み。
- 適用範囲: 実機上の点群位置精度は未検証。後続の[通常Viewerへの実測表示組込み](2026-10-01_dynamixel_ids_31_52.md#通常viewer起動への組込み)で首ピッチの表示上限への丸めを回避。腰は後続のユーザー確認に基づく[固定0°の継続配信](2026-10-01_viewer_roi.md)へ対応。各軸の校正は未完了。

試験起動: コンテナ内`ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ros2 launch gng_vlut_system realsense_mount_tf.launch.py`。購読・終了管理は`timeout 30s python3 -`による有限診断、終了再確認は`timeout 15s python3 -`。結果は`artifacts/realsense_mount_20261001/report.json`と`cleanup_followup.json`。

利用方法: [実機頭部へのRealSense取付TF](../realsense_gazebo.md#実機頭部へのrealsense取付tf)。
