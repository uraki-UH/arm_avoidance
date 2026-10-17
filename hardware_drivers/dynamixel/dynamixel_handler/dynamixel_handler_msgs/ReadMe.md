# Dynamixelメッセージ

定義は[msg](msg/)、生成対象は[CMakeLists.txt](CMakeLists.txt)を参照。

## Topic list

| 用途 | 型・トピック |
| --- | --- |
| 一括状態 | DxlStates → /dynamixel/states |
| 一括指令 | DxlCommandsX／P／Pro／All → /dynamixel/commands/... |
| 個別状態・指令 | DynamixelStatus／Goal／Present／Gain／Limit／Error／Extra |
| 操作・診断 | DynamixelShortcut／DynamixelDebug |
| 外部ポート | DxlExternalPort |

## How to use

```bash
ros2 interface show dynamixel_handler_msgs/msg/DynamixelPresent
ros2 topic echo /dynamixel/state/present
```

プログラム例は[examples](../dynamixel_handler_examples/ReadMe.md)を参照。

## Topic type detail

field・型・単位は[msg](msg/)を参照。角度deg、電流mAのメッセージをROSのrad・Aと混同しないこと。
配列はid_listとの対応を維持。空配列は未取得／未指定であり、ゼロ値ではない。
状態の各要素は異なる読取り周期を持ち、presentとgoalの同時取得保証なし。

## External port に関して

対応機種と設定は[外部ポート実装](../dynamixel_handler/src/optional_function/external_port.cpp)を参照。
