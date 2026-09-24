# 計測・再現資料

通常のビルド・ROS実行には不要な、GNG実験の再現資料と保存済み集計結果。

## 保存先の区分

- `benchmarks/<実験名>/`: 再現・集計スクリプト、設定スナップショット、計測用ヘッダー・スキーマ、保存済み集計、当時計測対象のSHA-256。Git管理対象。
- `artifacts/<実験名>/`: 共有ライブラリ、CMake生成物、生ログ、フレーム単位のJSON、再集計結果。ローカル保管のみでGit管理対象外。
- 詳しい条件・結果・限界: `gng_vlut_system/docs/designs/` と `releases/` の各記録。

`artifacts/COLCON_IGNORE`をGit管理し、保存した比較用パッケージも通常のcolcon探索から除外。`.gitignore`だけではROSパッケージ探索の除外にはならない。

生成物のGit管理除外による実ファイルの削除なし。SHA-256資料内のパスは計測当時の記録であり、移動前の `artifacts/` 表記を維持。

## 計測資料

| ディレクトリ | 内容 |
| --- | --- |
| [gng_followup_efficiency_20260924](gng_followup_efficiency_20260924/) | 探索・重点候補・クラスタ・疎エッジの4候補比較と本番3変更の採用 |
| [gng_normal_efficiency_20260924](gng_normal_efficiency_20260924/) | 法線・曲率の連続配列参照と差分再利用、全出力一致の検証 |
| [gng_radix_multiseed_20260924](gng_radix_multiseed_20260924/) | 6シードの品質・観測代表点比較と本番基数ソート採用 |
| [gng_radix_production_20260924](gng_radix_production_20260924/) | 本番機能を保持した基数ソートの時間・選択学習点・被覆の比較 |
| [gng_production_efficiency_20260924](gng_production_efficiency_20260924/) | 本番CPUの結果一致を維持した空きID管理・重心走査統合と実験版の基数ソート標準化 |
| [gng_runtime_trials_20260924](gng_runtime_trials_20260924/) | 学習量を維持した空き番号管理・LTO・入力ソートの実装比較 |
| [gng_minimal_comparison_20260924](gng_minimal_comparison_20260924/) | 最小構成で入力voxel・node.grid・treeの6条件比較 |
| [gng_bsp3d_minimal_20260923](gng_bsp3d_minimal_20260923/) | 入力ボクセル化なし・元点直接学習の最小tree版 |
| [gng_bsp3d_sampled_20260923](gng_bsp3d_sampled_20260923/) | 固定ノードグリッド撤去・照合回数制限の比較 |
| [gng_bsp3d_20260923](gng_bsp3d_20260923/) | グリッド・固定順8分木・bsp3dの比較 |
| [gng_bsp3d_profile_20260923](gng_bsp3d_profile_20260923/) | bsp3dの段階・詳細・標本計測 |
| [gng_spatial_nearest_20260923](gng_spatial_nearest_20260923/) | 旧AABB・汎用2近傍・固定順2近傍の比較 |
| [gng_spatial_profile_20260923](gng_spatial_profile_20260923/) | 旧AABB版の処理別計測 |
| [gng_spatial_tree_20260923](gng_spatial_tree_20260923/) | 旧AABB版の設定と集計 |
| [gng_coverage_20260923](gng_coverage_20260923/) | ノード生成範囲の確認 |

## 実行前提

既存スクリプトはコンテナ内の `/ros2_ws/src`、ROS Humble、当時計測に使った `/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3` を前提。ライブラリのビルド方法は[独立版README](../ais_gng_cpu/experimental/gng_spatial_tree/README.md)を参照。

比較スクリプトは `artifacts/<実験名>/` のライブラリを使用。新しい環境では再ビルドした対応ライブラリの配置が必要。旧AABB・汎用2近傍などの過去版は現行ソースと異なるため、現行版のビルドだけで過去の比較条件を再現した扱いにはしない。旧AABB向け `gng_spatial_profile_20260923/instrument.py` も対応する旧版ソースが必要。

プロファイルの `summarize.py` は同名の `artifacts/<実験名>/` にある生データを入力とし、比較元の `artifacts/gng_bsp3d_20260923/` または `artifacts/gng_spatial_tree_20260923/` のフレームJSONも必要。再集計の出力は `artifacts/<実験名>/summary.json`。保存済みの `benchmarks/<実験名>/summary.json`への自動反映なし。

```bash
python3 /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/summarize.py
python3 /ros2_ws/src/benchmarks/gng_spatial_profile_20260923/summarize.py
```

生データのない新規checkoutでは上記の再集計は不可。Git内には集計値・計測条件・再現コードだけを保存し、全生データを同梱した配布ではない。
