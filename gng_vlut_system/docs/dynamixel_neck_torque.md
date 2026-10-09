# パンチルトID51・52の重力補償・減衰

対象: XM430-W350のID51・52固定。目標角度を持たない重力補償と粘性抵抗。重力係数は実機未校正、既定の補償は無効。チルトの自重落下に備えた機体の支持と独立した電源停止手段が必要。手を離した位置での静止・抵抗感・安定性・適正電流の実機検証は未実施。

## 設定と起動

設定先: [dynamixel_neck_torque.yaml](../config/dynamixel_neck_torque.yaml)。配列の順序は`[ID51, ID52]`。

| 項目 | 意味 | 設定 |
| --- | --- | --- |
| `allow_hardware_output` | 実機への指令送信許可 | `true`・試験用 |
| `max_current_ma` | 補償・減衰の合成電流指令絶対値上限 [mA] | YAMLのユーザー調整値 |
| `damping_gain` | 各軸の速度に対する抵抗電流係数 [mA/(rad/s)] | YAMLのユーザー調整値 |
| `driver_namespace` | 既存handlerのROS名前空間 | `/dynamixel` |
| `enable_gravity_compensation` | 校正済み重力補償の有効化 | `false` |
| `gravity_cos_ma` | モータ角度cos項の支持電流係数 [mA] | `[0.0, 0.0]`・未校正 |
| `gravity_sin_ma` | モータ角度sin項の支持電流係数 [mA] | `[0.0, 0.0]`・未校正 |
| `gravity_ramp_sec` | ON報告後の補償立上げ時間 [sec] | `1.0` |

既存調整値: 編集時点のユーザー値（上限200 mA・減衰係数50）を保持。実機推奨値・安全性確認済みの値ではない。初回試験値2.69 mA・係数26.9からの変更はユーザー調整。自動増量なし。

未完了: 支持電流係数の実機校正。必要情報はモータ座標の角度と、その姿勢で重力につり合う符号付き電流の組。角度の異なる複数点による係数同定と別角度での確認が必要。手で支えた状態の電流を、そのまま重力支持電流と見なすことは不可。URDFの質量・重心からの自動設定、ストール電流からの係数決定なし。係数0のまま有効化した場合は起動拒否。

起動時: 条件成立後にトルクON要求。設定済みでも、実機のモード変更・ON操作を今回の設定作業で実施した意味ではない。出力禁止へ戻す場合は`allow_hardware_output: false`。

上限・減衰係数の未設定、非有限値、要素数不一致は起動拒否。電流上限は機器の指令分解能2.69 mAを考慮した値、減衰係数は正の値が必要。数値配列は整数・小数どちらも受付（各配列内の型は統一）。重力係数は符号付き、各軸の補償振幅`sqrt(cos係数²+sin係数²)`が電流上限を超える場合も起動拒否。設定は起動時固定、変更はCtrl+Cによる終了・OFF報告確認後の再起動で反映。

事前条件:

- 51・52のトルクOFF、Operating Modeが`0: Current Control`（ROS表記`current`）。`cur_position`では起動拒否。モード・PID・EEPROM設定の自動変更なし。
- `Torque On by Goal Update`と`Reverse Mode`が無効。Reverse時の電流・速度符号は今回の確認対象外。
- 既存`dynamixel_handler`から`fresh_joint_states`と`state/status`・`state/goal`・`state/extra`の継続受信。他の指令送信元・同時USBアクセスなし。
- モード変更が必要な場合は、首を支持してOFFを確認後、Wizard等で事前設定。Wizardとhandlerによる同一USBの同時使用は不可。モード変更に伴うゲイン等の初期化は[機器仕様](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11)を確認。

ROSとworkspaceのsource後、既存handlerを稼働したまま起動:

```bash
ros2 launch gng_vlut_system dynamixel_neck_torque.launch.py
```

別の設定ファイルを使う場合のみ`config_file:=/absolute/path/settings.yaml`を追加。Gazebo・handler・Viewerの追加起動なし。条件成立後のゼロ電流目標送信・読返し、ON要求、ON報告待ちを経て抵抗制御へ移行。

終了: 同じ端末の**Ctrl+C**。ゼロ電流と51・52のトルクOFFを10 Hzで再送し、最大3秒の状態報告待ち。`OFF報告あり（キャッシュを含む）`または`OFF未確認`を表示。後者は成功扱いせず、独立停止手段による確認が必要。首以外のIDへの指令なし。

## 制御と制限

電流指令: `I = clip(r * (a*cos(q) + b*sin(q)) - d*v, -上限, +上限)`を2.69 mA単位で絶対値切捨て。`a=gravity_cos_ma`、`b=gravity_sin_ma`、`d=damping_gain`、`q`はmotor座標の実測角度[rad]、`v`は実測速度[rad/s]。`r`はON報告後に0から1へ`gravity_ramp_sec`で増加。補償無効時は重力項0、静止時の電流指令0 mA。補償有効時は静止中も角度に応じた支持電流。ただし重力トルク0の姿勢や指令分解能未満では0 mA。

座標・モデル: 基台が固定・直立したパンチルトの各軸独立なsin/cos近似。係数はURDF関節角度ではなくmotor座標、符号・角度原点を含めて校正。基台の傾き・積載物・ケーブル荷重の変化は再校正対象。機械摩擦による抵抗は別。位置・速度・PWM目標、旧角度への復帰指令、誤差積分なし。補償誤差や電流量子化による姿勢ドリフトは残存。

監視: 実測取得時刻から0.2秒、状態・電流目標受信から1.5秒、機種・Drive Mode受信から10秒の期限。欠測・モード逸脱・電流目標の上限逸脱・他の指令publisher・運転中OFFの検出時は終了時OFF処理へ移行、自動再開なし。位置・速度には通信成功分だけの`fresh_joint_states`を使用。status・goal・extraにはhandlerのキャッシュを含み、OFF報告は新しいtorqueレジスタ読取りを保証する表示ではない。

終了経路: Ctrl+C、ノードへのSIGTERM/SIGHUP、例外、Linuxの親プロセス終了通知。親launchだけの終了時も子ノードに通知。制御ノード自身へのSIGKILL、PC停止、USB断ではOFF送信保証なし。[機器側Bus Watchdog](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#bus-watchdog98)は全Instruction Packetが対象であり、handlerの読取りが続く状態では制御ノード消失の代用監視にならない。

安全上の範囲: ソフトウェアの電流**指令**上限であり、実電流・関節トルク・接触力の保証なし。角度可動域の能動保持、独立非常停止、通信遅延を含む実機の安定性保証は対象外。補償有効化だけによる静止保証なし。既存位置保持launchの停止仕様は変更なし。

診断: 起動時の表示は`重力補償＋減衰`または`減衰のみ（静止時0 mA）`。停止理由はROS loggerとlaunchログへ保存。モード不一致時は実際のmode・error・pingを併記。

検証: [模擬試験・未検証範囲](releases/2026-10-01_neck_torque.md)。
