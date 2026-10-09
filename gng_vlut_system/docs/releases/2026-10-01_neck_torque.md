# 2026-10-01 - ID51・52専用の設定式電流抵抗

変更:

- 追加: `dynamixel_neck_torque.launch.py`・専用ノード・YAML。対象ID固定、電流上限`max_current_ma`と抵抗係数`damping_gain`は軸別設定。
- 起動: 出力許可・設定値・機種・currentモード・開始時OFF・新鮮な実測・ゼロ電流読返しを確認。モード・PID変更なし。
- 終了: ゼロ電流・トルクOFF要求、10 Hz再送、最大3秒の報告待ち。Ctrl+C・異常・親launch消失に対応。位置目標・旧角度への復帰なし。
- 設定: 初版の出力許可false・電流上限／係数0から、ユーザー承認により最小電流試験用へ変更。当時のYAMLは出力許可true、各軸上限2.69 mA、係数26.9 mA/(rad/s)。その後のユーザー調整値は維持、現行仕様は下記続報を参照。起動中変更は拒否、再起動で反映。実機推奨値・安全性の実証ではない。

結果:

- 単体: 新規46件と既存31件、計77件成功。CMake登録テストも成功。
- 続報・最小電流設定: YAMLの5条件を追加、計82件成功（0.39秒）。当該設定の隔離模擬ROS3条件成功（9.04秒、見込み12秒）。Ctrl+C時のOFF報告、実測途絶時のOFF要求と未確認表示、cur_position時の指令0件を確認。記録先: `artifacts/neck_torque_20261001/min_current/`。
- 結合: 隔離ROS_DOMAIN_ID=217・模擬ドライバ8条件成功。Ctrl+C、親launchのSIGTERM・SIGKILL、実測失効、上限未設定、出力未許可、誤モード、OFF報告なしを確認。
- 初回失敗: 親launchへのSIGTERM時に子ノードの継続を検出。試験の子終了待機とLinux親終了通知を追加。再送回数の監査で終了処理の過剰送信も検出し、10 Hzへ制限。最終試験は各条件100指令未満を確認。
- 配備: Docker内CMake configure・install成功。Pythonノード・launch・YAMLのsymlink反映。
- 実機: トルク指令なし。適正電流・抵抗係数・実機安定性・重力落下対策は未検証。
- 終了: 所有試験launch・模擬ドライバ・runner停止済み。既存Dynamixel・Gazebo・Viewerへの指令／停止操作なし。

制限: OFF報告はhandlerのキャッシュを含み、実レジスタ再読取り保証なし。制御ノード自身へのSIGKILL・PC停止・USB断でのOFF送信保証なし。重力補償の実機校正・角度保持・独立非常停止は未実施。[設定・起動・前提条件](../dynamixel_neck_torque.md)。

試験コマンド（コンテナ内、ROSとworkspaceのsource後）:

```bash
cd /ros2_ws/src
PYTHONDONTWRITEBYTECODE=1 timeout 40s python3 -B -m pytest -q -p no:cacheprovider gng_vlut_system/test/test_dynamixel_neck_torque.py gng_vlut_system/test/test_dynamixel_sim_output.py gng_vlut_system/test/test_dynamixel_sim_keyboard.py
python3 -B skills/run-benchmark-batch/scripts/run_batch.py artifacts/neck_torque_20261001/final_cases.json --output artifacts/neck_torque_20261001/final_named --repeats 1 --timeout-sec 40 --max-total-sec 330 --estimate-sec 3
python3 -B skills/run-benchmark-batch/scripts/run_batch.py artifacts/neck_torque_20261001/min_current_cases.json --output artifacts/neck_torque_20261001/min_current --repeats 1 --timeout-sec 40 --max-total-sec 130 --estimate-sec 4
```

再実行: 未使用の出力ディレクトリ指定。単独再現は`ROS_DOMAIN_ID=217 python3 -B gng_vlut_system/test/check_dynamixel_neck_torque.py --case sigint --output /tmp/neck_torque_check`。実際のlaunch引数と結果は各試験の`output.log`・`probe/launch.log`・`probe/ros/`に記録。試験値20・30 mAと係数10・20は模擬入力専用、実機推奨値ではない。

## 続報: 目標角度なしの重力補償と整数設定

- 変更: モータ実測角度のsin/cos支持電流と減衰を合成、合成後の電流上限・量子化・補償立上げを追加。目標位置・自動増量・モード変更なし。
- 設定: 既存の上限200 mA・減衰係数50を保持。実機係数未測定につき`enable_gravity_compensation: false`、係数0。係数0または振幅超過での有効化は拒否。静止保持の調整完了ではない。
- 修正: 整数配列の`damping_gain: [50, 50]`等を受付、停止理由をROS loggerとlaunchログへ保存。mode・error・pingの現状値を診断へ追加。
- 単体: 関連94件成功、0.22秒。静止時の補償・角度による符号変化・立上げ・合成電流上限・整数／不正値・既存停止条件を確認。
- 模擬: ROS_DOMAIN_ID=217の13条件成功、27.60秒（再試行時見込み39秒）。静止補償・補償中欠測・未設定／振幅超過の拒否・実際の整数YAMLを追加。機械運動のシミュレーションではなく通信・電流計算の検証。
- 初回失敗: 終了時の再送数判定に補償立上げ中の通常指令を含めた試験側の誤り。最初のOFF以降だけを判定対象へ修正。再検証前に権限審査タイムアウト1回、同一コマンドの許可された再試行で完了。
- 記録: `artifacts/neck_torque_20261001/gravity/`（初回）・`gravity_final/`（最終）。所有試験・模擬ノードは全停止、既存handler・デーモン継続。実機への指令・係数校正なし。

起動コマンド（コンテナ内、ROSとworkspaceのsource後）:

```bash
cd /ros2_ws/src
python3 -B skills/run-benchmark-batch/scripts/run_batch.py artifacts/neck_torque_20261001/gravity_cases.json --output artifacts/neck_torque_20261001/gravity_final --repeats 1 --timeout-sec 40 --max-total-sec 530 --estimate-sec 3
```
