# 2026-10-01 - ToPoDualArmの統合回避デモ

- 追加: `dual_arm_control.launch.py robot:=topodualarm`。名前空間`sim_ToPoDualArm`、機体寸法に対応する左右接近、直動グリッパーの専用姿勢設定。
- 回避方式: URDF外接形状による幾何探索。左腕用の既存GNG/VLUTは不使用。max系の既定GNG/VLUT構成は維持。
- 修正: 直動関節の軸方向力 [N] の取得、回転／直動の停止速度診断の分離。計画時の内部形状余裕0.01 m、実測監視の停止判定0.005 mは維持。
- 環境: 起動済みHumbleコンテナへDockerfile記載のGazebo制御依存を追加導入。`gng_vlut_system`のReleaseビルド・install成功。
- 単体検証: 操作・設定等のPython回帰155件、計画余裕変更後の幾何・GNG等36件成功（7件重複）。C++停止ラッチ10件成功。
- Gazebo検証: 左右の接近・退避・復帰、回避中L拒否、Aでホールド、Space実測停止・0.5秒保持に成功。最小推定余裕0.052258 m、最大回転変位0.496979 rad。GUI無効、実機出力なし、実行時間109.39秒。[通し結果](../../../artifacts/topodualarm_20261001/planning_margin/report.json)。
- Viewer配信: 機種別既定設定で起動、`sim_ToPoDualArm`の姿勢67メッセージ受信。実行時間4.78秒、所有プロセス・専用ROSノード残存0。[結果](../../../artifacts/topodualarm_20261001/viewer/report.json)。画面の目視は未実施。
- 修正前の失敗: 操作ノードの名前空間制限、ODE worldの計算エラー・速度超過、試験側の切替途中キー送信、右腕と胴体の余裕4.998 mmによる停止。名前空間対応、quick選択、試験の切替完了待ち、計画余裕の追加後に上記通し成功。
- 後始末: 成功試験の所有プロセス残存0、専用ROSドメインの残存ノード0、試験端末復元。初回急終了時のDDS残存判定は、後続起動前の専用ドメイン空を確認済み。既存ROS・コンテナは維持。
- 制限: 実機回避・Dynamixel接続・GUI画面の目視は未検証。ToPoDualArmの実機UDPは起動時拒否。GNG/VLUTによる双腕回避への対応は別作業。
- 起動・試験コマンド: [機種別手順](../dual_arm_simulation.md#topodualarmの統合回避デモ)。今回の試験出力先は`artifacts/topodualarm_20261001/planning_margin`。
- Viewer試験コマンド: 同じ試験スクリプトへ`--check-viewer-startup`を追加、出力先`artifacts/topodualarm_20261001/viewer`。全試験終了済み。作業中に外部で起動されたViewer・frontendは維持。
