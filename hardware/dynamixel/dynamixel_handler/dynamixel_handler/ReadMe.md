# DynamixelHandler開発

## プログラムの構成

| 場所 | 責務 |
| --- | --- |
| src/main.cpp | ノード生成・executor |
| src/dynamixel_handler.cpp・hpp | 状態・設定・周期処理 |
| src/dynamixel_handler_ros_* | ROS入出力 |
| src/dynamixel_handler_dyn_* | モータ通信 |
| src/optional_function/ | 任意機能 |
| src/internal/mylib_dynamixel/ | 通信ライブラリ |
| src/sub_node/ | 補助ノード |

[実装](src/)・[設定](config/)を参照。baudrate変更はPythonとpybind11の_mylib_dynamixelを使用。

## 既知の制限

- 通信断時の停止は機種・モード・watchdogに依存。特にXC330、位置制御とhoming_offsetの組合せは要確認。
- トルクの瞬間的なON/OFFだけでは旧目標の解除を保証しない。
- Fast Sync Readで先頭IDが断線した後、後続Pingも失敗する条件を観測。機種・順序・再試行条件の切分けが必要。
- Yシリーズの制御表・Indirect Address・状態構造の対応は未完了。

## optional_function 実装ルール（開発者向け）

追加先は[src/optional_function](src/optional_function/)。既存のexternal_port・imu_opencrを実装例として使用。
有効化フラグ・設定読込み・初期化・周期処理を共通ノードへ接続し、ROS入出力と通信処理を分離。
設定は[config](config/)へ集約。必須値不足・初期化失敗を黙って無視しないこと。
新機能は無効時に通信・周期負荷を追加しない構成。ビルド登録は[CMakeLists.txt](CMakeLists.txt)を参照。
