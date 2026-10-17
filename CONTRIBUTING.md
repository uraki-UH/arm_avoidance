# 共同開発

対象: ROS 2 Humble。環境構築は[RUN_GUIDE](MD/RUN_GUIDE.md)、記述規約は[AGENTS.md](AGENTS.md)。
通常の作業は`main`からの作業ブランチ、1 PRは1目的。公開済み履歴・タグの書換え禁止。

```bash
git switch -c uraki_your_topic
```

## 変更対象と入口

| 対象 | 主な変更先 | 仕様・接続契約 |
| --- | --- | --- |
| 姿勢GNG・VLUT | `gng_vlut_system/include/`・`src/` | [実装仕様](gng_vlut_system/docs/TECHNICAL_SPEC.md) |
| 点群学習・登録 | `ais_gng_cpu/`・`pointcloud_sampling/` | 各パッケージの`package.xml`・README |
| 回避・経路生成 | Simulatorの`integrations/ros2/gng_avoidance.py`・`pursuit_cbf.py`・`shuttle_trajectory.py` | [共通ROS回避](gng_vlut_system/docs/dual_arm_simulation.md#センシング--ros回避--実行先) |
| 把持候補・保持物体 | `grasping_system/` | [責務・入出力](grasping_system/README.md) |
| Viewer・描画 | `ToPoFuzzy-Viewer/frontend/`・`libs/ros_visualization_web/` | [Simulator構成](ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/docs/ARCHITECTURE.md) |
| 物理・センサ・接続 | Simulatorの`integrations/ros2/`・`app/` | [開発ガイド](ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/docs/DEVELOPMENT.md) |
| 関節・形状・起動設定 | `urdf/`・対象パッケージの`config/`・`launch/` | 対象機体のYAML・URDF |
| 3D位置合わせの比較 | `extensions/registration/` | [任意ツール](extensions/registration/README.md) |
| クラスタ再分類 | `extensions/fuzzy_classifier/` | [任意パッケージ](extensions/fuzzy_classifier/README.md)。標準起動には不使用 |

Simulatorは`ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/`。
方式追加は既存の登録JSONとアダプタ境界を優先。[拡張契約](ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/docs/ARCHITECTURE.md#ros連携の拡張通信契約)を参照。

## 依存と境界

- 中核・任意部品の配置とROS探索範囲は[workspace.yaml](workspace.yaml)。CPUの構築部品・対象パッケージは[Compose YAML](docker/compose.simulator_cpu.yaml)。開発コンテナの`cb`・`cbd`も中核だけの探索。
- 同じ変更で整合が必要な実装・メッセージは本体で管理。単独利用・異なる依存環境・独立リリースの機能は分離候補。任意実装は`extensions/`、機体別ソースは`robots/`。別Gitは`local_projects`・`external_projects`で共有範囲から分離し、接続アダプタだけを利用側へ配置。
- `external_projects`は本体外の別Git、`local_projects`は本体内のローカル専用配置。各履歴・未コミット変更を個別管理し、本体への一括追加・本体からの相対importは禁止。接続はROS topic等の契約経由。
- ROS依存は対象パッケージの`package.xml`、ビルド・インストールは`ament`／`colcon`。通常構築への任意環境・学習済み重みの必須化なし。
- CPUイメージのOS・Python依存は[Dockerfile](Dockerfile)、UI依存はSimulatorの`package.json`・ロックファイル。新規依存は両方の構築経路への反映確認。
- センサ入力・関節実測・駆動指令は別経路。ROSメッセージ契約の変更は`MSG/`と送受信側を同じPRへ包含。長さはm、角度はrad。
- UIへ回避アルゴリズムを複製せず、既存ROSノード・Pythonアダプタへ接続。GNG以外の方式へGNG学習ファイルの必須化なし。
- 外部コード・素材の条件は既存の個別ライセンス・`THIRD_PARTY_NOTICES.md`を保持。個人用の絶対パス、bag、実機設定・認証情報の追加禁止。

## 変更確認とPR

ホストのリポジトリルートで:

```bash
bash scripts/dual_arm_cpu.sh build
bash scripts/dual_arm_cpu.sh check
bash scripts/dual_arm_cpu.sh demo
```

共通CIも同じ3操作。`check`は依存・点群・短時間物理、`demo`は既知条件の接触なし実行と移動の確認。回避性能比較とは別。
UI変更時はブラウザで変更箇所も確認。GNG/VLUT・把持・実機の変更は対象機能の追加検証が必要。
自作ローカルテストは既存方針どおりGit管理外。未配布テストや`colcon test`の検出0件を、共有版の検証済み根拠として使用しないこと。

PRには目的、変更対象、実行コマンド・結果、未検証範囲、入出力・依存の変更だけを記載。試行錯誤のcommitは取り込み時にsquash。
不具合報告はcommit、実行環境、コマンド、最小設定、期待／実際の動作。生ログ・大容量データはGitへ追加せず、共有先を別途案内。

## 参考と適用範囲

[ROS 2 Developer Guide](https://github.com/ros2/ros2_documentation/blob/humble/source/The-ROS2-Project/Contributing/Developer-Guide.rst)のパッケージ境界・README／CONTRIBUTING・PR前検証を採用。
[rosdep公式手順](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Rosdep.rst)の依存宣言、[ros-tooling/action-ros-ci](https://github.com/ros-tooling/action-ros-ci)のソース構築・CI確認を参考。
現構成は既存Docker入口の共用。全パッケージのrosdep自動解決・網羅的テスト・ROS品質レベルの認定は対象外。
