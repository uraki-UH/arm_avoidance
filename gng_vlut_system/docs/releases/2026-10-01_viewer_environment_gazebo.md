# 2026-10-01 - 更新中の実環境TmapによるGazebo回避と実測姿勢表示

変更:

- 入力: 通常Viewer側の自己除去後ROI・Tmap_static・関節特徴と、元点群・実機全関節の新鮮さ検査。
- 座標・時刻: URDF固定リンクによる環境→ルート変換、実時間stampの期限検査と継続更新。Gazebo側の自己マスクによる実環境の再処理なし。
- Viewer姿勢: 既存のGazebo実測関節→`sim_ToPoDualArm`配信経路の利用。実機関節は自己除去の根拠として保持。
- 起動設定・制限: [RealSense/Gazebo手順](../realsense_gazebo.md#更新中の環境tmapをgazeboへ接続)。現在の学習データは左腕7関節。旧仮想カメラ配置入力も保持。

検証:

- ビルド: `gng_vlut_system` Release・install成功。既存CMake警告あり。
- Python回帰: 55 / 55 件成功。既存SciPy/NumPy版数警告あり。
- 隔離Gazebo最終版: 通常Viewerの点群→自己除去→Tmap更新→退避・復帰、実機関節欠測・点群欠測の実測停止に成功。自己除去26セル、最小推定余裕0.068742 m、GNG経路選択115回、Viewer実測一致1,274件。[試験結果](../../../artifacts/viewer_environment_gazebo_20261001_trial4/report.json)。
- 現在の実RealSense入力: 約12秒の回避動作、関節変化最大0.356093 rad、最小推定余裕0.070780 m、GNG経路選択33回。ViewerのGazebo実測一致496件・最大差0 rad、Space停止後の速度0.0000083 rad/s。[実入力結果](../../../artifacts/viewer_environment_gazebo_live_20261001/report.json)。
- 初回失敗: 固定56秒待ちの終了時点で復帰途中（最大残角0.179142 rad）。復帰完了の実測条件と80秒上限へ試験を修正後に成功。[初回結果](../../../artifacts/viewer_environment_gazebo_20261001_trial1/report.json)。
- 再検証での別停止: [trial3](../../../artifacts/viewer_environment_gazebo_20261001_trial3/report.json)では実機関節欠測の失効前に回避軌道の可動域保護が発動。停止だけを判定した旧試験の合格表記は、欠測停止の根拠には不採用。停止理由の検査を追加したtrial4で入力失効による停止を確認。可動域保護の閾値変更なし、先行した可動域逸脱の原因特定は未実施。
- 終了処理: 所有試験ノード・Gazebo・追加DDSノードの残留なし、既存RealSense・Dynamixel・Viewer維持。実入力試験のSIGINT終了時に`gzclient` exit -11を観測。動作中の回避結果とは別のGUI終了異常。
- 未検証: 実機軸・絶対角の校正、右腕の学習経路、長時間の実環境変動。点群からGazebo衝突物体を生成する機能なし。

試験起動コマンド（コンテナ内、source済み）:

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_viewer_environment_gazebo.py \
  --output /ros2_ws/src/artifacts/viewer_environment_gazebo_20261001_trial4
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 ROS2CLI_NO_DAEMON=1 \
  python3 /ros2_ws/src/artifacts/viewer_environment_gazebo_live_20261001/check.py
```

試験出力先は再実行時に未使用のディレクトリを指定。GUI付き実入力確認の実際のlaunch引数は[command.json](../../../artifacts/viewer_environment_gazebo_live_20261001/command.json)。
