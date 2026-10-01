# SpatialTreeのURDF高次元学習への適用調査

> 2026-10-01追記: 通常GNGの関節角学習と左右TCP辺生成へ厳密近傍索引を組込み。[実装・検証記録](../../gng_vlut_system/docs/releases/2026-10-01_gng_nearest_index.md)。以下は2026-09-30時点の試作結果。

- 結論: `MovingBSPTree`の厳密k近傍探索が導入候補。実データによる探索・位置更新APIの単体比較まで完了、本体への組込みなし。
- 対象版: `~/SpatialTree`、commit `964aab43d7f8681d76cdded385624e1623feb9fd`。
- 保存物: [全比較結果・条件](SUMMARY.md)、[入力ハッシュ・実行環境](metadata.json)、[比較コード](measure.cpp)。学習済みモデル・ライブラリ・実装設定への変更なし。

## 検索の測定結果

各条件1,000検索、3反復の平均。保存点の軸別範囲内の一様クエリ、上位4候補。既存処理相当のEigen動的ベクトル全探索＋partial_sortとの比較。構築・FK・衝突・グラフ更新を含まない時間。

| 次元 | 保存ノード数 | 全探索 [µs/検索] | MovingBSP [µs/検索] | 時間比 |
| --- | ---: | ---: | ---: | ---: |
| 7 | 10,801 | 135.362 | 11.056 | 12.24 |
| 14 | 10,000 | 138.379 | 38.879 | 3.56 |

- 検証: 先頭1,000点の部分集合・保存点周辺クエリも含む24,000検索対の上位4候補ID一致。二乗距離の最大丸め差9.54e-7 rad²。同距離ID順の保証は未検証。
- 更新単体: 10万回×各次元×3反復、7次元0.093–0.102 µs/回、14次元0.114–0.118 µs/回。更新後600検索のID一致、木の不変条件検査成功。
- 制限: ランダム点を目標へ0.008移動した更新単体。実際の勝者・全隣接ノード移動、挿入・削除を含む学習全体の比較ではない。
- 実行: ホストi7-14650HX、GCC 13.3、`-O3 -march=native -DNDEBUG`、fast-mathなし、CPU配置変更なし。Docker内のtrainer実行条件との同一性は未保証。
- バッチ: 静的3試行・更新3試行とも正常終了。初期見積り15秒／6秒、実測2.302秒／0.350秒。各試行60秒・各バッチ180秒の上限。

## 統合候補と互換性

- 差替え位置: [GrowingNeuralGas.cpp](../../gng_vlut_system/src/core/gng/GrowingNeuralGas.cpp)の`one_train_update`。毎回の全探索を`findNBest(..., n_best_candidates)`へ置換する案。衝突条件による上位候補からの勝者選出は既存方式の保持。
- 距離: 現行の関節角度は生のL2²、角度wrap・可動域正規化なし。MovingBSPの距離定義と一致。
- 同期対象: `add_node`、`remove_node`、`update_node_weights`、`load`、`setParams`、フィルタ。近傍ノードを含む全座標変更の`updatePosition`経由化、安定したノードアドレス、同距離ID昇順の互換性が必要。
- 対象集合: `status.active`ではなく現行探索条件の`id != -1`。探索ループ内の100反復ごとの誤差減衰も保持対象。
- 可変次元: 現行は`Eigen::VectorXf`、MovingBSPはコンパイル時次元。7/14次元などの実体化と、それ以外の全探索への復帰経路が必要。
- 木の選択: `AdaptiveTree`は分割ごとに2^D個の子。高次元には二分木の`MovingBSPTree`、最初の比較は`approx_eps=0`・`NoHysteresis`。
- 別候補: Step5の腕ごとの固定3次元座標エッジ探索、可視化GNGの関節次元＋3の特徴探索。今回の性能計測対象外。

## 全体時間との区別

[以前のmax生成記録](../../gng_vlut_system/docs/releases/2026-09-28_effectivity_map_refresh.md)では全体1,656秒、初期学習350.365秒、衝突考慮学習347.755秒。
元ログの工程開始時刻差では中間フィルタ491.516秒、最終フィルタ442.986秒、合計934.502秒。
探索だけの高速化率を、FK・衝突検査・VLUT込みの全体へ適用する根拠なし。過去の別実行であり、今回との直接の性能比較でもない。

## 既存ライブラリ試験の失敗

`moving_bsp_test`はassert有効で2回とも終了コード1。fast-mathなし29.44秒、原CMake相当のfast-mathあり26.17秒。両方同じ2件の失敗。

- `uniform, rebuild disabled`: 併合発生の期待に対し`collapses=0`。境界越えは1,609回。
- `uniform, approx eps=0.5`: 再構築発生の期待に対し`rebuilds=0`。
- 解釈: シナリオの網羅条件の失敗。探索距離・木の不変条件の不一致ログなし。ただし全テスト合格ではなく、期待分岐の網羅確認が未完了。
- 根拠: [通常ログ](library_tests/test.log)、[原CMake相当ログ](library_tests/test_fast_math.log)、[起動コマンド](library_tests/commands.txt)。

## 起動条件と再現

指定launchの既定`config/topoarm_dual.yaml`は現在のsource/installに存在しないため、実行時は`params_file`の明示が必要。
`ToPoDualArm.yaml`の既定profileは左腕7次元、`topo_dual_arm_max.yaml`と`topo_dual_arm_max_long.yaml`は左右14次元。

実施した[比較起動コマンド](commands.txt)と[ライブラリ試験起動コマンド](library_tests/commands.txt)を保存。
元の一時ディレクトリは試験終了後に削除済み。再現時は次の準備後、`commands.txt`のコンパイル・runnerコマンドを使用。
既存結果への上書きを避けるため、runnerの`--output`には新しいディレクトリを指定。

```bash
mkdir -p /tmp/spatialtree_urdf_audit_20260930 /tmp/spatialtree_library_test_20260930
cp /home/uraki/uraki_ws/benchmarks/spatialtree_urdf_20260930/measure.cpp /tmp/spatialtree_urdf_audit_20260930/
cp /home/uraki/uraki_ws/benchmarks/spatialtree_urdf_20260930/cases*.json /tmp/spatialtree_urdf_audit_20260930/
```

- 終了確認: コンパイル・runner・比較6試行・ライブラリ試験2回の全終了、自身の試験プロセス残存なし。ROSノードの新規起動なし、既存ROS・コンテナの停止なし。
