# 2026-10-01 - 左腕Gazebo回避とDynamixel試験用キー操作

変更:

- 対象: ユーザー指定のToPoDualArm左腕ID41〜47。点群・GNG回避と実機転送の操作端末を追加。旧起動構成は保持。
- 操作: H有効化、J/K単関節小動作、F Gazebo補間目標の転送、A回避、Space両系統停止、R実機解除、B Gazebo解除。起動時の実機指令なし、実測姿勢での保持目標と低速profileの読返し後に対象IDのtorque ON。
- 実測: ドライバに`fresh_joint_states`を追加。同じ通信で位置・速度を取得できたIDのみ、通信開始の実時間stamp付きで配信。旧キャッシュ済み配信による停止確認なし。
- 仕様・移行: [起動手順と制限](../dynamixel_sim_control.md)。初期値は低速・小範囲の試験用。Spaceは保持指令とラッチによるソフト停止、独立した非常停止ではない。

検証:

- ビルド: `dynamixel_handler`・`gng_vlut_system` Release/install成功。エージェントによる既存実機ドライバの再起動なし。
- 単体: 65 / 65 件成功。左腕7軸の部分欠測、古い／未来／重複stamp、frame不一致、非有限速度、重複ID、動作軸の混入、速度0でも位置変化がある場合の停止確認取消と、既存キー・関節変換・ToPoDualArm launchの回帰。
- 隔離模擬モータ: ID47の+1°（モータ座標−1°）、保持目標→torque ON順序、Space後の非再開、開始姿勢差拒否、模擬JTC目標追従、Gazebo stamp停止、実測欠測、端末停止、実測再送、±5°範囲超過の停止に成功。[最終結果](../../../artifacts/dynamixel_sim_control_20261001_trial9/report.json)。実機・実USBへの指令なし。
- 初期試験の失敗: 実行権限不足、ROS引数の未除去を修正。試験側ではROS配列型の比較、処理中キーの連打、端末復帰前入力、実測静止成立前の有効化で失敗。条件を待つ試験へ修正し、出力側の開始条件は維持。[trial1〜9](../../../artifacts/)。
- Gazebo起動試験の初回失敗: 試験側のポート確認API呼出し不一致。続く試験では実測停止成立前のBキーを拒否。実測停止を待つ試験へ修正後、旧右手首選択版の起動・Space・Bに成功。[trial3](../../../artifacts/dynamixel_gazebo_start_20261001_trial3/report.json)。
- 左腕GNG回避の最終起動試験: Aによる回避開始、左腕実測変化0.006877 radの時点でSpace停止、実測停止確認後のB解除に成功。右腕実測変化0.000286 rad、controller実測591件。実機指令publisherは0。[最終結果](../../../artifacts/dynamixel_gazebo_start_20261001_trial5/report.json)。回避全軌道完走ではなく、開始と途中停止の検証。
- 左腕起動試験の途中失敗: [trial4](../../../artifacts/dynamixel_gazebo_start_20261001_trial4/report.json)はGNG回避ノードの起動前にAを操作してサービス未接続の停止。新端末に回避準備表示・準備前Aの拒否を追加し、trial5で成功。
- 実USBの読取り専用確認: 作業途中で既存ドライバのPID変更を検出後、domain25で新しい`fresh_joint_states`を3秒購読。91メッセージ、18 ID（左腕41〜47を含む）、観測stamp経過時間の最大0.941 ms、frame=`dynamixel_motor`。新しい実測配信を確認、動作指令・USB再接続なし。故意の実USB切断による欠測除外は未検証。
- 未検証: 実機小動作・停止、左腕全7軸の実機校正、実機への回避軌道転送、PC／USB故障時の独立停止。実機試験は支持条件・電源停止手段と開始姿勢の照合待ち。新しい実測配信は稼働中のため、追加ドライバ再起動は不要。
- 後始末: 全所有試験・読取り専用プローブの終了、試験ノード・Gazeboポートの残留なし、試験端末設定の復元。既存RealSense・Viewer・GNG学習・Dynamixelへの停止／再起動操作なし。作業途中の既存プロセスPID変更は、エージェントによる操作とは別。

試験コマンド（コンテナ内、source済み）:

```bash
colcon build --packages-select dynamixel_handler gng_vlut_system \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd /ros2_ws/src/gng_vlut_system
python3 -m pytest -q test/test_dynamixel_sim_output.py \
  test/test_dual_arm_control_keyboard.py test/test_joint_command_model.py \
  test/test_topodualarm_launch.py
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 \
  python3 test/check_dynamixel_sim_control.py ../artifacts/dynamixel_sim_control_20261001_trial9
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 \
  python3 test/check_dynamixel_gazebo_start.py ../artifacts/dynamixel_gazebo_start_20261001_trial5
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 \
  python3 /ros2_ws/src/artifacts/dynamixel_fresh_readonly_check_20261001.py
```

出力先は再実行時に未使用ディレクトリを指定。各試験のlaunch実引数は出力先の`command.json`。試験ログはworkspace直下の`artifacts`へ集約。読取り専用プローブは実行した標準入力コードをファイル化した再現用。

## トルクOFF停止と電流上限の追加

- 背景: Spaceは位置保持の停止でトルクを維持。速度制限だけでは保持時の電流・力の制限なし。
- 読取り専用の実機調査: 左腕41〜43はXM540-W270（model 1120）、44〜47はXM430-W350（1020）。全軸`cur_position`、読取り時の報告はtorque OFF・電流0 mA。Goal CurrentとCurrent Limitは41〜43が5,506.43 mA、44が3,209.17 mA、45〜47が3,206.48 mA、Goal PWMとPWM Limitは100%。設定上限とその時点の電流実測は別物。瞬間的な過大トルクの発生は未計測。
- Eキー: 選択IDのtorque OFF要求、位置指令の完全遮断、再開禁止ラッチ、Gazebo停止。実測欠測時も送信可能。Spaceへの切替による保持指令再開なし。出力許可false時は未送信を表示。
- 再開: 実測静止・OFF報告後のR、改めてH。Rだけではtorque ONなし。OFF報告はキャッシュを含む既存statusに基づくため、最新のtorqueレジスタ読取り成功や電源遮断の保証なし。
- 電流上限: `max_current_ma`の既定0は未設定・H拒否。通常動作は上記2機種の電流ベース位置制御だけを許可、2.69 mA単位で上限内の非ゼロ値へ変換。Goal Current読返し後にtorque ON、運転中の上限逸脱／読返し失効・電流制御モード変更はtorque OFF。EEPROM上限の変更・制御モード変更なし。模擬試験の100 mAは実機の推奨値ではない。
- 根拠: ドライバのmodel対応表・電流換算と、[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#goal-current102)・[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102)のGoal Current仕様。電流ベース位置制御での電流目標による制限。
- 単体検証: 69 / 69 件成功。実測なしのOFF・7 ID限定・error解除／mode変更の混入なし・保持再開阻止・OFF報告前の解除拒否・電流上限未設定での無送信を含む回帰。
- 模擬検証: EによるOFF、実測欠測中の未停止表示、Spaceによる保持再開なし、R→Hだけでの再有効化、電流読返し上限逸脱のOFFに成功。[電流制限版trial2](../../../artifacts/dynamixel_torque_off_20261001_trial2/report.json)。
- 最終版: 上記に電流ベース位置制御からのモード変更時のOFFを加えて成功。[trial3](../../../artifacts/dynamixel_torque_off_20261001_trial3/report.json)。試験launch・ノード残留なし、端末設定復元済み、6秒の読取りプローブも終了済み。
- 実機への操作: 状態の6秒購読だけ。駆動・torque切替・電流/PWM設定の書込みなし。実機での停止確認・荷重支持に必要な電流値の決定は未実施。

追加試験の起動コマンド（コンテナ内、source済み）:

```bash
cd /ros2_ws/src/gng_vlut_system
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 \
  python3 test/check_dynamixel_sim_control.py ../artifacts/dynamixel_torque_off_20261001_trial3
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 \
  python3 /ros2_ws/src/artifacts/dynamixel_torque_limits_readonly_20261001.py
```

単体コマンドは上段と共通。起動出力先は未使用ディレクトリを指定。
