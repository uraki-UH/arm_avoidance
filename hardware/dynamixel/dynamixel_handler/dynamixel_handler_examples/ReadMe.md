# dynamixel_handler_examples

起動済みdynamixel_handlerへ接続するROS 2サンプル。

## Build

```bash
colcon build --symlink-install --packages-up-to dynamixel_handler_examples
source install/setup.bash
```

## Abstract of each example

| 例 | 用途 |
| --- | --- |
| example0 | JointStateとの相互変換。position／velocityはrad系、effortはA |
| example1 | 関数形式の一括指令・状態購読 |
| example2 | example1のクラス形式 |
| example3 | 速度制御・速度上限 |
| example4 | 系列共通の目標・モード指定 |

```bash
ros2 launch dynamixel_handler_examples example1.xml
```

例は[src](src/)、設定・起動は[launch](launch/)を参照。
モード・トルク変更を含むため、実機への接続前に対象IDと値を確認。
