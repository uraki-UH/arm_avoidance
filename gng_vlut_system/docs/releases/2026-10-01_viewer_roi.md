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

## 周期ログの集約

- 形式: `PCl: 100 (0.10 ms) | Vxl: Env 36 Self 20000 Dup 4 (0.04 ms) | Emap: Safe 10800 Coll 1 Dang 0 (0.15 ms)`。数値は書式例。表示名のみ`PC`から`PCl`へ変更、内部トピック`pc`は保持。
- 表記変更の検証: Viewer実行形式・コンポーネントの再ビルド、`test_viewer_status`の4件成功・終了済み。通し試験の期待文字列も更新、表記変更後のROS通し試験は未実施。既存ノードの再起動なし。
- 頻度: 既存`robot_viewer_bridge_node`から1 Hz。プロセス接頭辞・長い時刻の重複なし。起動説明・警告・エラーは別行、従来の段階別周期ログ・接続数はDEBUGへ移行。
- PCl: 処理対象入力の`width × height`、無効点も含む受信点数。msは点群コールバック内の座標変換・ROIボクセル化などの経過時間。
- Vxl: `Env`は自己除去前の環境、`Self`は膨張込みの自己除去マスク、`Dup`はそのマスクに該当して実際に除去した環境ボクセル数。msはマスク参照・検査・自己除去・出力配信、自己形状の生成・膨張時間は対象外。
- Emap: 有効GNGノードの安全・衝突・注意の排他的分類。msはVLUT照合から把持形状の追加判定・ノード状態更新まで。グラフ配信・描画時間は対象外。
- 時間条件: 各段階の直近1回、steady clockの経過ms、小数2桁。ROSトピックの転送待ち時間・段階間のフレーム同期・時間合算なし。未受信・3秒超の更新停止は件数とmsを`--`表示、0とは区別。
- 内部転送: 各機体名前空間の`viewer_status/{pc,vxl,emap}`、`std_msgs/msg/Float64MultiArray`、最大5 Hz・best effort。配列は順に`[PCl,ms]`・`[Env,Self,Dup,ms]`・`[Safe,Coll,Dang,ms]`。点群・グラフ本体の追加購読や追加ノードなし。
- 適用: 通常の`gng_viewer_bridge.launch.py`で自動有効化、追加引数不要。個別ノードの`enable_viewer_status`既定値はfalse。稼働中プロセスは再起動時から反映。

有限検証コマンド（コンテナ内）:

```bash
/ros2_ws/build/gng_vlut_system/test_viewer_status
timeout --signal=INT --kill-after=15s 70s python3 /ros2_ws/src/gng_vlut_system/test/check_viewer_status.py
```

通し試験の起動対象: `ros2 launch /ros2_ws/src/gng_vlut_system/launch/gng_viewer_bridge.launch.py params_file:=<一時試験YAML> joint_control_backend:=external`。domain 229、試験点群・関節値のみ、実機ドライバ起動なし。

- 検証: 関連ノード8ターゲットのビルド、単体4件、通常launch経由の通し試験に成功。100点入力・自己除去前後の差分とDupの一致・Emap更新・実測ms・旧INFO周期ログなし・入力停止後の失効表示を確認。
- 試験実測例: `PC: 100 (0.02 ms) | Vxl: Env 36 Self 1884 Dup 0 (0.02 ms) | Emap: Safe 10721 Coll 0 Dang 80 (0.18 ms)`。実環境の性能測定ではなく、100点の機能試験。内部転送の追加負荷は未測定。
- 初回失敗・修正: colconの複数`--cmake-target`指定拒否後、CMakeの複数ターゲット指定へ変更。試験終了時のSIGINT二重配送によるexit −2を、親launchだけへの送信へ変更。既存の二種類の関節購読に由来するDURABILITY警告は保持。
- 終了状態: 所有試験launch終了、隔離ドメインの残留ノードなし。既存プロセスの停止操作なし。最終試験ログはコンテナ内`/tmp/viewer_status_check_9noikzsc/launch.log`。
