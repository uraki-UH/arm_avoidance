# lib_dynamixel

Dynamixel通信ライブラリ。上流: [SHINOBI-organization/lib_dynamixel](https://github.com/SHINOBI-organization/lib_dynamixel)。
ROS組込みは[親パッケージ](../../../ReadMe.md)を参照。単独ビルドは未検証。

## API List

API宣言は[dynamixel_communicator.h](src/dynamixel_communicator.h)を参照。
接続・設定、Ping／Reboot／FactoryReset、Read／Write、Sync／Fast Sync、再試行付き操作を提供。

## Address Map

機種別のアドレス・物理量換算は[dynamixel_parameters.h](src/dynamixel_parameters.h)・[実装](src/dynamixel_parameters.cpp)を参照。

## API の 詳細説明

引数・戻り値・対応型は[src](src/)を参照。
設定変更・Reboot・FactoryResetは実機状態を変更する操作。通信成功と物理停止は別判定。
