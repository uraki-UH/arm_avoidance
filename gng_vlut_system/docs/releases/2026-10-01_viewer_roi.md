# 2026-10-01 - 通常ViewerのROI生成と固定腰TF

- 原因の実測: 6秒間でRealSense点群90件・自己マスク182件を受信。`/ToPoDualArm/roi_voxels`のpublisherは0。新規TF購読で`base_link → torso_link`が欠落し、光学frameからbase_linkへの変換失敗。腕・首の動的TFとカメラ取付TFは存在。
- ROI追加: `enable_environment_voxelization`をToPoDualArmで有効化。通常Viewerから`world_index_to_voxel_node`を起動し、機体設定の点群入力・GNG範囲・余白・VLUTセル幅・ボクセルID形式を利用。ロボット基準への直接変換、未接続TFの座標読み替えなし。自己除去前後トピックを分離し、自己認識・自己除去が無効なら起動拒否。
- 固定腰: ユーザー確認は正面向き・固定0°。ブリッジの`fixed_joint_names`・`fixed_joint_positions`へ明示し、全サーボ入力が揃う周期だけ関節値に追加。固定位置は回転rad・直動m。配列長不一致・非有限値・名前重複は拒否。旧設定の既定は固定関節なし。
- 検証: 2パッケージのbuild/install成功。隔離domain 96の通常Viewer起動で、光学frameの試験点群から自己除去前後各3ボクセルを受信。12秒経過後もカメラTFの最終更新から約0.010秒、腰0を含む21関節・Viewer姿勢の一致を確認。試験所有プロセス終了・残留ノードなし。
- 回帰: 疑似18台入力で21関節の変換・符号付き位置・欠測停止と復帰・Viewer姿勢JSON一致、読取り以外の通信命令なし。旧ID設定の起動・終了成功。試験プロセス終了済み。既存コンパイラ・CMake警告あり。
- 制限: 実機の稼働中Viewerには再起動時から反映。実点群の最終ROI件数・実物との位置精度は修正後未検証。各軸の符号・ゼロ点・グリッパー換算の校正は別途必要。Gazeboの回避開始とは別の表示経路。

試験コマンド（コンテナ内、終了済み）:

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  timeout 65s python3 /ros2_ws/src/artifacts/viewer_roi_20261001/check.py
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  timeout 60s python3 /ros2_ws/src/gng_vlut_system/test/check_dynamixel_current_pose.py --viewer
```

実機の購読診断はコンテナ内`timeout 14s python3 -`・`timeout 12s python3 -`、旧設定の起動確認はdomain 98の`timeout 15s python3 -`、いずれも終了済み。
根拠: `artifacts/viewer_roi_20261001/report.json`・`launch.log`・`legacy_config.json`。仕様・起動: [通常ViewerのROI表示](../realsense_gazebo.md#通常viewerのroi表示)。

## Tmap_staticの環境状態更新への接続

- 再起動後の実測: 6秒間で自己除去後ROI91件、最終650ボクセル。`occupied_voxels`・`danger_voxels`のpublisherなし、`Tmap_static`の新規更新受信なし。
- 修正: 通常Viewerに`voxel_to_vlut_node`を追加。自己除去後ROI → 占有・危険ボクセル → `topofuzzy_bridge_node`の状態更新へ接続。危険判定方式・膨張幅・セル幅は既存機体設定に追従。
- 通し試験: 光学frameの障害物追加・除去で、全10,801ノード安全 → 衝突7,560・危険3,241 → 全安全への復帰を確認。占有2,937・危険29,406ボクセル。全ノード位置は不変、ラベルだけ変化。自己除去を含む通常Viewer起動で検証、build/install成功。
- 実点群確認: domain 25に変換ノードだけを6秒追加し、`Tmap_static`更新33件を受信。最終占有611・危険3,457ボクセル、安全10,574・危険208・衝突19ノード。試験用変換ノードは終了済み、継続利用にはViewer再起動が必要。既存RealSense・handler・Viewerの停止なし。
- 終了確認: 動作検査と残留ノードなしの確認に成功。SIGINT終了処理中に既存`static_transform_publisher`のexit −11を記録。所有プロセスは全終了、実機ノードの停止なし。

試験コマンド（コンテナ内、終了済み）:

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  timeout 90s python3 /ros2_ws/src/artifacts/viewer_tmap_20261001/check.py
```

根拠: `artifacts/viewer_tmap_20261001/report.json`・`launch.log`。6秒の実機受信診断は`timeout 14s python3 -`、終了済み。
実点群確認の起動・終了管理はコンテナ内`timeout 22s python3 -`。追加ノードの全起動引数・結果は`artifacts/viewer_tmap_20261001/live_report.json`、ログは`live_bridge.log`。
