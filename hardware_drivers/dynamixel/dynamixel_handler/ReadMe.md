# DynamixelHandler-ros2

Dynamixel X・P・ProシリーズのROS 2通信ノード。Humble／Jazzy対応。
上流: [ROBOTIS-JAPAN-GIT/DynamixelHandler-ros2](https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros2)。

## How to install

workspaceルートで:

```bash
git submodule update --init --recursive
colcon build --symlink-install --packages-up-to dynamixel_handler
source install/setup.bash
```

## How to use

USB接続、ID重複なし、baudrate統一が前提。
設定は[config_dynamixel_handler.yaml](dynamixel_handler/config/config_dynamixel_handler.yaml)を参照。

```bash
ros2 launch dynamixel_handler dynamixel_handler_launch.xml
```

起動時のモード変更・トルクON・終了時OFFは設定依存。読取りだけの用途でも初期化時の書込み有無を確認。
制御入力でモードが変わるため、別指令元との競合は避けること。

## Topics

| 入出力 | 主なトピック |
| --- | --- |
| 一括状態 | /dynamixel/states |
| 系列別一括指令 | /dynamixel/commands/{x,p,pro,all} |
| 個別状態 | /dynamixel/state/{status,present,goal,gain,limit,error,extra} |
| 個別指令 | /dynamixel/command/... |
| 操作・診断 | /dynamixel/shortcut、/dynamixel/debug |

型は[メッセージ定義](dynamixel_handler_msgs/msg/)、接続例は[examples](dynamixel_handler_examples/ReadMe.md)を参照。
角度deg、速度deg/s、電流mA。ROS標準JointStateのSI単位とは区別。
キャッシュ値を含む配信と、同時刻の実測は別。fresh_joint_statesの用途は[実機制御手順](../../../gng_vlut_system/docs/dynamixel_sim_control.md)を参照。

## Parameters

設定は[config](dynamixel_handler/config/)を参照。ROS型の確認はros2 interface show、稼働設定の確認はros2 param dumpを使用。

## Optional機能

Dummy Servo・自動エラー解除・OpenCR IMU・外部ポート等。設定は[config](dynamixel_handler/config/)を参照。
Dummy Servo指定IDは実機より優先。表示された状態が実機由来かは別確認。

## 各種情報の分類と Control Table との対応

[メッセージ](dynamixel_handler_msgs/ReadMe.md)・[アドレス定義](dynamixel_handler/src/internal/mylib_dynamixel/src/dynamixel_parameters.h)を参照。
EEPROM領域への変更はトルクOFFを伴う。制御モード・機種による対応差あり。

## Baudrateの一括変更

設定は[config_dynamixel_unify_baudrate.yaml](dynamixel_handler/config/config_dynamixel_unify_baudrate.yaml)を参照。

```bash
ros2 launch dynamixel_handler dynamixel_unify_baudrate_launch.xml
```

接続モータの通信設定を書き換える操作。pybind11の_mylib_dynamixelが必要。

## Latency Timer

USB側と設定YAMLのlatency_timerを一致させること。
確認先は /sys/bus/usb-serial/devices/<デバイス>/latency_timer。

## 速度に関してメモ

Fast Sync Read・分割読取り／書込みの選択はconfigを参照。性能はbaudrate・USB latency・読取り項目・モータ数に依存。

## Trouble Shooting

| 症状 | 確認先 |
| --- | --- |
| パッケージ・型が見つからない | build後のsetup.bash |
| 内部ライブラリ不足 | submodule更新と再ビルド |
| モータ未検出 | 電源・USB権限・ID範囲・baudrate・機種 |
| 初期化が完了しない | 接続ID数、応答・エラー、初期化設定 |
| 低い通信成功率 | 配線・電源・USB latency・baudrate |
| publish失敗 | ros2 daemon、domain、DDS設定 |

停止・Fast Sync Readの既知制限は[開発メモ](dynamixel_handler/ReadMe.md#既知の制限)を参照。
トルクOFF・通信停止だけで全機種の旧目標が解除される保証はない。

## Launchファイルと設定（yaml）

[launch](dynamixel_handler/launch/)・[config](dynamixel_handler/config/)を参照。
