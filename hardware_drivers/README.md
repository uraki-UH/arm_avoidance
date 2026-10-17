# 共通機器ドライバ

共通のセンサ・モータドライバと接続設定。機体固有ソースは`robots/`配下。

| 配置 | 内容 |
| --- | --- |
| [mid360](mid360/README.md) | MID-360専用Docker環境・固定IP設定・ROS 2点群入力 |
| [dynamixel/dynamixel_handler](dynamixel/dynamixel_handler/ReadMe.md) | Dynamixelドライバ・メッセージ定義・サンプル |
| dynamixel/dynamixel_joint_state_bridge | Dynamixel実測値からJointStateへの変換・機体別対応表 |

MID-360の起動はリポジトリルートで`python3 hardware_drivers/mid360/start.py`。Dockerのみの起動は`docker compose up mid360`。

DynamixelのROSパッケージ名・launch名は従来どおり。ソース移動前にビルド済みの場合は、ワークスペースルートで対象パッケージの再ビルドが必要。

```bash
colcon build --packages-select dynamixel_handler_msgs dynamixel_handler dynamixel_handler_examples dynamixel_joint_state_bridge --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

ビルドはドライバの起動・既存実機ノードの再起動を含まない。

## ToPoArm

ToPoDualArmのソースはワークスペース内の`robots/topo_dual_arm/`で別Git管理。本体のcloneには未収録。
Dockerの参照先を変更する場合は`TOPOARM_SOURCE_DIR`を指定。
実機ドライバ環境の構築は本体ルートで`docker compose --profile hardware up -d --build topoarm_hardware`。
別Gitの`lib_dynamixel` submoduleの初期化が必要。コンテナの既定動作は構築後の待機のみで、実機駆動の開始なし。

`ToPoDualArm3.yaml`の表示には、利用するROS環境へ`topoarm_description`を事前構築・インストール。
URDF・メッシュはROSパッケージ経由。旧ソースを参照するインストール済み環境は再構築が必要。
