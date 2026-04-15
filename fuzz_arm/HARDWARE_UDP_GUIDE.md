# UDP通信フレームワーク利用・開発ガイド

本プロジェクトにおける実機アームとのUDP通信（姿勢受信・指令送信）の枠組みと、その利用方法についてまとめます。

## 1. 構成要素の概要

通信システムは以下の3層構造になっています。

1.  **Low Level (UDPComm)**: 生のソケット通信（送信・受信）。
2.  **Middle Level (TwinManager)**: ロボットの関節状態（`JointState`）とUDPパケット（`ReceptorData`/`CommandData`）の相互変換。
3.  **App Level (UDPController / OnlineIntegration)**: 制御アルゴリズムに基づいた指令値の決定。

## 2. 通信設定 (Configuration)

実機と通信する際のネットワーク設定は `env_config.txt` で変更可能です。

```ini
# env_config.txt (例)
robot_ip = 192.168.1.100  # ロボット（コントローラ）のIP
robot_port = 8080         # ロボット側の受付ポート
pc_port = 9090            # PC側の受信待機ポート
```

## 3. 実機連携の作業ステップ

### ステップ1：Pythonによる疎通確認
C++プログラムを動かす前に、ネットワーク設定が正しいか確認します。

```bash
# 実機からの受信テスト
python3 scripts/test_udp_sync.py

# 実機への送信テスト
python3 scripts/test_udp_control.py
```

### ステップ2：TwinManager の統合
独自の制御プログラムを作成する場合、`TwinManager` をインスタンス化してメインループで回します。

```cpp
#include "hardware/twin_manager.hpp"

auto &twin = hardware::TwinManager::Instance();
twin.Initialize(); // 設定ファイルからIP/Portを読み込み

while (running) {
    // 1. 実機の最新姿勢を受信
    twin.Update(); 
    auto current_q = twin.GetRealJointState();

    // 2. 制御計算 (GNGなどのアルゴリズム)
    auto target_q = MyAlgorithm(current_q);

    // 3. 指令値をセット（次回の Update() で送信される）
    twin.SetTargetJointState(target_q);
}
```

### ステップ3：ROS 2 トピックへの変換 (Bridge Node)
Ubuntu 環境で ROS 2 トピックとして実機を扱いたい場合、以下のノードを作成します。

*   **Node Name**: `udp_bridge_node`
*   **Publish**: `/joint_states` (sensor_msgs/JointState)
    *   `twin.GetRealJointState()` の内容を変換して配信。
*   **Subscribe**: `/joint_commands` (std_msgs/Float64MultiArray 等)
    *   受信した値を `twin.SetTargetJointState()` にセット。

## 4. 注意事項

*   **パケット構造**: `src/hardware/data_utils.hpp` で定義されている構造体の並び順（パディング）が、マイコン側（ESP32やSTM32等）と一致している必要があります。
*   **リアルタイム性**: UDPはパケットロスが発生するため、受信側で「最後に受信した時刻」をチェックし、古いデータで制御を続けないように `TwinManager` 内で保護ロジックを検討してください。
*   **ビルド**: 通信パフォーマンスを確保するため、必ず **Releaseモード** でビルドしてください（`RECONSTRUCT_VLUT.md` 参照）。
