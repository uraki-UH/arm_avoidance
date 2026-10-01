# パンチルトID51・52の電流抵抗

対象: XM430-W350のID51・52固定。目標角度を持たない粘性抵抗、位置保持・重力補償なし。チルトの自重落下に備えた機体の支持と独立した電源停止手段が必要。実機での抵抗感・安定性・適正電流の検証は未実施。

## 設定と起動

設定先: [dynamixel_neck_torque.yaml](../config/dynamixel_neck_torque.yaml)。配列の順序は`[ID51, ID52]`。

| 項目 | 意味 | 初期値 |
| --- | --- | --- |
| `allow_hardware_output` | 実機への指令送信許可 | `true`・試験用 |
| `max_current_ma` | 各軸の電流指令絶対値上限 [mA] | `[2.69, 2.69]` |
| `damping_gain` | 各軸の速度に対する抵抗電流係数 [mA/(rad/s)] | `[26.9, 26.9]` |
| `driver_namespace` | 既存handlerのROS名前空間 | `/dynamixel` |

現行値: ユーザー承認による最小電流の試験設定。上限は指令1刻み、係数は速度0.1 rad/s付近で1刻みとなる値。静止時・速度絶対値0.1 rad/s未満では指令0 mA、浮動小数点丸めにより切替点付近も0の場合あり。抵抗感の保証・実機推奨値・安全性の実証ではなく、首を支持した段階試験の開始値。自動増量なし。

起動時: 条件成立後にトルクON要求。設定済みでも、実機のモード変更・ON操作を今回の設定作業で実施した意味ではない。出力禁止へ戻す場合は`allow_hardware_output: false`。

上限・係数の未設定、非有限値、要素数不一致は起動拒否。電流上限は機器の指令分解能2.69 mAを考慮した値、係数は正の値が必要。YAMLの配列要素は小数表記。設定は起動時固定、変更はCtrl+Cによる終了・OFF報告確認後の再起動で反映。

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

電流指令: `I = clip(-抵抗係数 × モータ速度, -電流上限, +電流上限)`を2.69 mA単位で絶対値切捨て。速度はmotor座標のrad/s。位置・速度・PWM目標、旧角度への復帰指令なし。静止時の指令電流0 mA。機械摩擦による抵抗は別。

監視: 実測取得時刻から0.2秒、状態・電流目標受信から1.5秒、機種・Drive Mode受信から10秒の期限。欠測・モード逸脱・電流目標の上限逸脱・他の指令publisher・運転中OFFの検出時は終了時OFF処理へ移行、自動再開なし。位置・速度には通信成功分だけの`fresh_joint_states`を使用。status・goal・extraにはhandlerのキャッシュを含み、OFF報告は新しいtorqueレジスタ読取りを保証する表示ではない。

終了経路: Ctrl+C、ノードへのSIGTERM/SIGHUP、例外、Linuxの親プロセス終了通知。親launchだけの終了時も子ノードに通知。制御ノード自身へのSIGKILL、PC停止、USB断ではOFF送信保証なし。[機器側Bus Watchdog](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#bus-watchdog98)は全Instruction Packetが対象であり、handlerの読取りが続く状態では制御ノード消失の代用監視にならない。

安全上の範囲: ソフトウェアの電流**指令**上限であり、実電流・関節トルク・接触力の保証なし。重力補償、角度可動域の能動保持、独立非常停止、通信遅延を含む実機の安定性保証は対象外。既存位置保持launchの停止仕様は変更なし。

検証: [模擬試験・未検証範囲](releases/2026-10-01_neck_torque.md)。
