# 2026-09-24 - 最小GNGの入力ボクセルとnode.grid比較

## Summary

ユーザー指定の「入力ボクセル化」「ノード探索もnode.grid」の両方を、同じ最小学習処理で比較。生点群はtree 11.30 ms／grid 11.15 ms、入力voxel 0.1 mは26.37／24.98 ms、0.5 mは22.47／21.37 ms。今回の条件では入力ボクセル化の約9〜12 msが主な追加コスト。

## Changed

独立コピー内の入力整理とノード探索をビルド時に切替。共通の4,000回学習・ノード／エッジ更新・法線／ラベル・寿命処理を使用。

## Added

`gng_minimal_comparison`の4ライブラリ、19件のテスト、6条件×3試行の再現スクリプト、時間内訳と被覆率の保存。

## Fixed

比較用のセル番号計算で、有限座標とYAML上端の境界点を明示的に処理。入力重心の丸め誤差を範囲内へ制限。

## Removed

既存実装からの削除なし。

## Behavior Impact

生点群は元点数、入力voxelは占有セル数に応じた学習配分。node.gridは27セル内・1セル10ノードの制約があり、treeの全域厳密2近傍と出力が異なる。全点の観測寿命維持・重点サンプリング・クラスタリングは全条件で除去済み。本番GNGと同じ機能・品質ではない。

## Topics / Params / Messages

ROS topic・launch・messageの変更なし。比較版では`input.voxel_grid_unit`をvoxel版のみ、`node.grid`をgrid版のみで受理。未使用設定は拒否。統計APIに候補数と方式識別を追加。

## Verification

Release `-O3 -DNDEBUG`の4ライブラリ、CTest 19/19件成功。計測全900フレームで最近傍検索4,000回・事前探索0回、方式別の3試行間グラフ900/900一致。tree_rawはコピー元の保存済み50フレームと一致。

本番ソース・コピー元ソースのハッシュ不変。本番ライブラリのSHA-256は`cdb5d7736c23ed07fdc86ca7202abf5c518ca0222067f0550cb0be8f1e8417a4`で不変。既存ROS・bag・デーモン10プロセスのPID・親PID・コマンドが一致。

起動コマンド:

```bash
docker exec gng_cpu_container bash -lc \
  'timeout -s INT -k 10 900 bash /ros2_ws/src/benchmarks/gng_minimal_comparison_20260924/run_benchmarks.sh'
```

ビルド・テストコマンドは[実装README](../../../ais_gng_cpu/experimental/gng_minimal_comparison/README.md)、条件と全結果は[計測README](../../../benchmarks/gng_minimal_comparison_20260924/README.md)。今回起動したビルド・テスト・計測は終了、一時ビルドは削除済み。ROSノードの新規起動なし。

## Risk / Notes

既存ROS稼働中の短時間比較で、ROS配送・Viewer描画は未計測。原点以外の0.2 m被覆率は生点群51.3〜52.3%、voxel 0.1 mは51.8〜53.3%、voxel 0.5 mは41.1〜41.6%。0.5 mでは細部の被覆が低下。原点込みの指標も生JSONへ保存。実験版の本番差替えなし。
