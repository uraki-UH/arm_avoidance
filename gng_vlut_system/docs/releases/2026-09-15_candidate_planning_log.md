# 2026-09-15 - 候補軌道計画の計算時間ログ

## 1. 要約

`grasp_joint_candidates.launch.py`の計画更新ごとに、計算時間と結果を英語のINFOの1行で表示。

```text
dof=7 Plan: 34.35 ms Count: goal=2 reach=2
```

- `std::chrono::steady_clock`で目標・開始候補選定から経路探索・選択までを計測。入力待ち・mutex待ち・配信用データ生成・配信は対象外。CPU占有時間ではなく経過時間。
- `goal`は目標GNGノードの候補数、`reach`は経路の得られた目標数。把持姿勢の入力件数とは別。
- 起動時の関節一覧・トピック一覧・安全ノード数・準備完了と候補受信ログはDEBUGのみ。共通実装の実行系ノードにも、このログ整理を適用。

## 2. 条件・検証

- 時間ログは計画更新時だけ。入力不変のタイマー周期では出力なし。候補消失によるクリア時は`goal=0 reach=0`。
- 計画結果・更新条件・警告・エラーへの変更なし。

変更なし。指定済みのlaunchコマンドで表示。

- コンテナ内の古いビルド設定を再構成し、Releaseビルド・インストールに成功。
- 保存済み10,000ノードGNGと`ToPoDualArm.yaml`を使用した既存ROS結合テストに成功。静止入力時の再探索なし、明示要求・関節変更時の再計画、候補消失時のクリアと復帰、関節目標配信なしを確認。
- 同テストを`runpy`で起動し、後片付け前のlaunchログを回収。時間ログ7件、非負の時間・候補数と到達数の整合、長いINFOログの非表示を確認。到達2件の計画時間は34.35 / 33.41 / 10.28 / 10.98 ms。姿勢が途中で変化する結合確認の実測値であり、性能比較ではない。
- Subsequent log-format changes, including the final single-line format above, passed Release rebuilds with `cmake --build /ros2_ws/build/gng_vlut_system --target topological_map_planning -j2`. The ROS integration results above precede these wording changes.

ビルドコマンド（コンテナ内、`/ros2_ws`）:

```bash
source /ros2_ws/install/setup.bash
MAKEFLAGS=-j2 timeout -s INT -k 20s 600s colcon build --packages-select gng_vlut_system --symlink-install --executor sequential --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release
```

結合テストの入口:

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py
```

検証中に起動したROSコマンド（同じ隔離ドメイン）:

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin -p base_frame:=ToPoDualArm/base_link -r topological_map:=/ToPoDualArm/Tmap_static
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

検証用launch・子ノードはすべて停止済み。

**制約**

1回の計画更新に時計参照2回とINFO出力1回を追加。出力自身の時間は計測対象外。
