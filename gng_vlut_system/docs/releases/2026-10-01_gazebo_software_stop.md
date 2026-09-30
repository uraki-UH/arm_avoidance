# 2026-10-01 - Gazeboソフト停止と保持検証

変更:

- 停止API: `safety/stop`・`safety/reset`の追加
- 保持: 駆動層での位置指令遮断・有限トルク保持
- 再開: controller停止・実測停止確認・明示解除後の新規指令
- デモ連動: 自動開始の無効化・旧GNG経路の破棄
- 統合launch: `dual_arm_control.launch.py`。同じ端末のAで回避、Lで追従、Space / Sで停止、Rで解除後の保持、Qで全終了
- 専用停止ツール: 既存launch用の別端末操作。解除キーなし

統合操作の検証（各1回）:

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
- 実機送信: 無効固定。校正・実測鮮度・実機停止処理の未確認によりHのON要求を拒否。USBドライバの統合起動なし

条件・起動コマンド: [仕様・再現手順](../dual_arm_simulation.md#ソフト停止の再現試験)。機種別結果: [最新試行](../../../artifacts/gazebo_software_stop_20261001/direct_diagnostics/report.json)、[初回直接指令](../../../artifacts/gazebo_software_stop_20261001/direct/report.json)、[通常回避デモ](../../../artifacts/gazebo_software_stop_20261001/demo_retry/report.json)、[事前検査](../../../artifacts/gazebo_software_stop_20261001/demo/report.json)。
