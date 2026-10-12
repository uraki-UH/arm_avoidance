# 実機接続環境

機器ごとのドライバ・接続設定を管理するディレクトリ。リポジトリのcloneで各環境を取得可能。

| 配置 | 内容 |
| --- | --- |
| [mid360](mid360/README.md) | MID-360専用Docker環境・固定IP設定・ROS 2点群入力 |
| [dynamixel/dynamixel_handler](dynamixel/dynamixel_handler/ReadMe.md) | Dynamixelドライバ・メッセージ定義・サンプル |
| dynamixel/dynamixel_joint_state_bridge | Dynamixel実測値からJointStateへの変換・機体別対応表 |

MID-360の起動はリポジトリルートで`python3 hardware/mid360/start.py`。Dockerのみの起動は`docker compose up mid360`。

DynamixelのROSパッケージ名・launch名は従来どおり。ソース移動前にビルド済みの場合は、ワークスペースルートで対象パッケージの再ビルドが必要。

```bash
colcon build --packages-select dynamixel_handler_msgs dynamixel_handler dynamixel_handler_examples dynamixel_joint_state_bridge --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

ビルドはドライバの起動・既存実機ノードの再起動を含まない。
