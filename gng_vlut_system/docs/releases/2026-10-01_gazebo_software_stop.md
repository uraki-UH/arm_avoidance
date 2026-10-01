# 2026-10-01 - Gazeboソフト停止と保持検証

変更:

- 停止API: `safety/stop`・`safety/reset`の追加
- 保持: 駆動層での位置指令遮断・有限トルク保持
- 再開: controller停止・実測停止確認・明示解除後の新規指令
- デモ連動: 自動開始の無効化・旧GNG経路の破棄
- 統合launch: `dual_arm_control.launch.py`。同じ端末のAで回避／保持、Lで追従／保持、停止中のLで解除後保持。Spaceで停止、Ctrl+Cで全終了。R・Q・Sキー廃止
- 専用停止ツール: 既存launch用の別端末操作。解除キーなし
- UDP出力: 補間済みGazebo目標の19関節CSV化、Hで許可、Space・異常で遮断。既定OFF、L再開後の自動再送なし

ホールド経由への限定:

- 単体成功数: 350 / 350 件。[結果](../../../artifacts/dual_arm_hold_gate_20261001/unit_final.xml)・[実行コマンド](../dual_arm_simulation.md#切替条件の単体試験)。試験プロセス終了済み、既存ROS停止なし
- 開始元: ホールドのみ。追従中のA・回避中のLは拒否、現在動作の継続。各モードの同じキーでホールド復帰
- 停止後のL: 条件付き解除→ホールドのみ。実測静止・controller状態確認後、改めてA/Lで開始。UDP自動再送なし
- 検証範囲: ROSサービス窓口の相互切替拒否・処理中の開始拒否・拒否要求の後追い実行なし・停止解除全遷移。Gazebo追加試験・実機送信なし

キー削減時点の検証（現行URDF、localhost、各1回）:

- 単体成功数: 333 / 333 件。キー入力・解除の全遷移・中断条件・UDP再送禁止・形状包囲。実ROS/Gazeboとの接続なし
- 当時の再開条件: 実測停止・新鮮なリーダー入力・未完了要求なし。現行のホールド経由へ変更済み
- 起動互換: camera_linkの衝突用boxを外接球列へ包含。元URDFの変更なし
- 初回成功数: 0 / 2 回。両機種ともbox未対応で回避ノード終了、修正済み。188.31 s（推定180 s）
- 修正後成功数: 0 / 2 回。max_longはSpace後の実測停止未確認、maxは起動中の状態更新失効。161.95 s（推定180 s）
- 到達範囲: max_longの起動・H許可・追従・UDP送信遮断。L再開・追従OFF保持のGazebo通し確認は未到達
- 後始末: 全4試行で所有プロセス・ROS node・portの残存なし、端末復元済み。既存ROS・コンテナ維持、実機送信なし

[単体](../../../artifacts/dual_arm_keys_20261001/unit_final.xml)・[初回](../../../artifacts/dual_arm_keys_20261001/udp_first/report.json)・[box対応後](../../../artifacts/dual_arm_keys_20261001/udp_box_retry/report.json)・[新キー・試験コマンド](../dual_arm_simulation.md#gazebo目標のudp出力)。停止・失効判定の緩和なし。物理停止未確認の原因は未確定、ODE警告も観測。

UDP追加時の検証（旧キー・当時のURDF、localhost、各1回）:

- 回帰成功数: 308 / 308 件
- Gazebo→UDP成功数: max 1 / 1 回、max_long 1 / 1 回
- 目標角一致数: 206 / 206 パケット
- 停止パケット受信後の角度送信数: 0 パケット
- 実測途絶から停止パケット受信: max 0.507 s、max_long 0.545 s。実機の物理停止時間ではない
- 試験時間: 123.95 s（事前推定180 s）

初回: 両機種とも追従切替中のUDP目標失効。統合時の時刻・controller状態配信を100 Hzへ変更後に成功、監視期限0.5 sは維持。
初回終了処理はDDS情報残存で確認失敗（所有PID残存なし）。Q経由の正常終了優先後は端末・ROS graph・portの復元確認済み。
[初回記録](../../../artifacts/dual_arm_udp_20261001/first/report.json)・[再試験記録](../../../artifacts/dual_arm_udp_20261001/clock_retry/report.json)・[設定・起動コマンド](../dual_arm_simulation.md#gazebo目標のudp出力)。

統合操作追加時の検証（旧キー・当時のURDF、各1回）:

- 回帰成功数: 155 / 155 件
- Gazebo成功数: max 1 / 1 回、max_long 1 / 1 回
- 確認範囲: 模擬leaderへの移動・全関節停止・入力継続中の解除後保持・実機ON拒否・端末復元・全終了
- AのON/OFF成功数: 2 / 2 回。接近から退避までの通し動作は未検証
- 試験時間: 144.96 s（事前推定180 s）

元workspace・Docker installへ反映。所有試験全終了、既存ROS・コンテナ維持。[操作・試験コマンド](../dual_arm_simulation.md#1つのlaunchでの統合操作)・[統合試験記録](../../../artifacts/dual_arm_control_20261001/first/report.json)。

結果（最新の直接指令試験、各2回）:

- 単体試験成功数: 35 / 35 件
- max成功数: 1 / 2 回
- max_long成功数: 2 / 2 回
- 停止要求後の停止到達: 4 / 4 回
- 試験時間: 283.56 s（事前推定340 s）
- 所有試験の終了確認: 4 / 4 回。既存ROS・コンテナ維持、専用domain空・ポート閉鎖

キーボード追加検証: 単体23/23件、疑似端末・模擬ROSサービス5/5件成功。要求受付と実測表示の分離、拒否・不達・端末復元の確認。試験プロセス終了済み、Gazebo物理試験の追加なし。[操作・試験コマンド](../dual_arm_simulation.md#キーボード停止)・[入力層の試験記録](../../../artifacts/gazebo_stop_keyboard_20261001/pty/report.json)。

制限:

- max: 停止ラッチ中の物理再開で0.176 rad/s・0.000176 radの左右肩変動。保持判定失敗、原因未確定。ODE計算エラーも観測、停止閾値の緩和なし
- 通常回避デモ: 両機種とも開始後の入力鮮度喪失で停止試験へ未到達。初回未受信ではなく、失効入力の特定は未完了
- 初回直接指令試験: maxの再activate後にも0.0906 rad/sの変動。両機種の終了検出は試験側の二重SIGINTで失敗、修正後の終了確認は成功
- 事前検査の初回失敗: 連続回転関節の上下限なしへの試験側対応漏れ、修正済み
- 未検証: action軌道・実機・人が接近し続ける条件。物理非常停止の代替不可
- 実機送信: 未実施。実機宛先は明示許可・機種別校正・受信側停止/watchdog仕様の確認が必要。設定例は模擬専用、旧14項目のUDP設定は流用不可。USBドライバの統合起動なし

条件・起動コマンド: [仕様・再現手順](../dual_arm_simulation.md#ソフト停止の再現試験)。機種別結果: [最新試行](../../../artifacts/gazebo_software_stop_20261001/direct_diagnostics/report.json)、[初回直接指令](../../../artifacts/gazebo_software_stop_20261001/direct/report.json)、[通常回避デモ](../../../artifacts/gazebo_software_stop_20261001/demo_retry/report.json)、[事前検査](../../../artifacts/gazebo_software_stop_20261001/demo/report.json)。
