# 2026-10-01 - ブラウザとROSのロボット状態・軌道往復

## 要約

- 追加: world内のロボット配置、関節状態、URDF各リンクTF、モデルURDFのROS配信。
- 配信時刻: 点群・深度・関節・TFで共通。取得時の姿勢を保存し、XYZをbase_footprintへ変換。
- 追加: 点群に依存しない姿勢配信、モデル別JointTrajectoryのブラウザ再生、停止操作。
- 制限: 位置のみの相対軌道、線形補間、関節範囲・速度検査。実機制御・動力学・衝突検査は対象外。
- 正本: [操作・トピック・再現コマンド](../../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md)。

## 条件・検証

- 条件: 標準モデル、独立Chrome、Docker内Humble、ROS_DOMAIN_ID=178、HTTP 18879。
- 配置: XYZ=(0.3,-0.2,0.1) m、yaw=25 deg。
- 点群・TF照合: 有効114,472点、TF 43変換、光学座標からbase_footprintへの全点照合成功。
- 往復: ROS軌道→ブラウザ首Yaw 0.1 rad→ROS関節状態の到達確認成功。
- 回帰: 形式・深度・状態・軌道のPython 15件成功。
- ブラウザ検査: 未知関節・過速度・範囲外の3条件拒否、停止操作成功、ページエラー0件。
- 初回失敗: 深度生成の共有ロック待ちで姿勢送信がタイムアウト、軌道受信停止。生成をロック外へ移動し、通信待ち上限を10秒へ変更後に再試験成功。
- 未検証: Longの実往復、持続レート、GNG/VLUTの回避計画との接続、実機。
- 試験起動: `node /tmp/topo_robot_browser_test.mjs`。
- ROS試験起動: `docker exec gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/topo_robot_ros_test.py'`。
- 試験プロセス: 終了済み。既存サーバー・既存ROSは維持。
