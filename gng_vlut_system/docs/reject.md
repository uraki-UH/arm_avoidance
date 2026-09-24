# 不採用・採用見送りの記録

実装予定ではなく、採用しないと判断した内容の記録。単なる未実装や検証失敗は対象外。
時間・優先順位・依存作業の都合で止めているだけの作業は [pending.md](pending.md) に記録。
記録単位は「日付 / 対象範囲 / 判断 / 判断根拠 / 理由 / 再検討条件 / 関連資料」。恒久的な禁止と一時的な見送りを区別。

## 2026-09-24: bsp3dの既存分割処理による一括初期構築

- 対象・判断: 全点をrootへ登録して既存の分割処理を再帰適用する一括構築案の不採用。比較差分は保存し、通常APIへの追加なし。
- 判断根拠: エージェントによる同じ入力・Release・CPU固定の3試行比較。実GNG18,729点で逐次構築4.583 msに対し一括9.881 ms、中点分割6.528 ms。合成平面でも4.334 msに対し12.828 ms／8.510 ms。結果照合と動的更新試験は成功したが、初期構築の改善にならず。[条件と再現資料](../../benchmarks/bsp3d_migration_20260924/README.md)。
- 再検討条件: 分割時の選択・再配置・境界更新の重複を削減した別の具体的実装で、結果を維持して複数入力の構築・検索合計時間を短縮できる場合。ユーザーの恒久禁止や一括構築一般の否定ではない。

## 2026-09-24: 移動ノード追跡によるエッジ距離の差分更新

- 対象・判断: 今回のCPU GNGで移動・接続変更ノードをビット列に記録し、関係エッジだけ距離を計算する案の本番採用見送り。比較実装は保存。
- 判断根拠: エージェントによる実入力・固定乱数の結果一致と3試行比較。保守区間は2.285→2.249 ms／1.937→1.903 msと短縮が小さく、別途比較した実在エッジプールの直接走査を採用。[全比較・限界](../../benchmarks/gng_incremental_updates_20260924/README.md)。
- 再検討条件: 移動・接続変更が少ない入力で、追跡を含めた学習・保守・全体時間が直接走査より一貫して短くなる場合。ユーザーによる恒久禁止ではない。

## 2026-09-24: 通常LiDARの重点候補XYZコピー削減

- 対象・判断: 元点番号参照と区間参照を独立・合成検証し、本番採用は見送り。比較実装は保存。
- 判断根拠: エージェントによる固定入力・乱数の全出力一致と反復測定。元点番号参照は候補生成0.510→0.344 ms／0.305→0.183 msへ短縮した一方、学習は5.295→5.738 ms／4.442→4.699 ms。合成比較でも生成＋学習の増加が残り、明確な全体改善なし。[詳細](releases/2026-09-24_gng_followup_efficiency.md)。
- 再検討条件: 候補順と学習点を維持し、間接参照の負担を含む全体時間で代表入力に一貫した改善が確認できる場合。ユーザーによる恒久禁止ではない。

## 2026-09-24: 本番CPUの入力ソートを基数ソートへ置換する案

- 対象・判断: 本番CPUの入力ソート置換は今回採用見送り。独立比較版は保存し、最小実験版での基数ソート標準ONは継続。
- 根拠: ユーザーの「学習結果に影響しないような効率化なら」という本番変更条件に対するエージェント判断。同じ全点・セル割当・4,000回学習で速度改善を確認したが、重点候補内の順序変更で選択点も変わり、初回からグラフ構造が異なる。[検証](releases/2026-09-24_gng_radix_production.md)。
- 再検討条件: 実際の学習点変更と出力差を許容する条件の明示、および対象データでの評価。ユーザーによる恒久的な不採用指定ではない。
- 同日再判断: ユーザーが乱数に由来する結果差を許容し追加検証を承認。6シードで明確な品質悪化を認めず、観測代表点のみの変更はグラフに帰還しないことを確認。本番CPUの入力ソートとして採用。[新判断と検証](releases/2026-09-24_gng_radix_multiseed.md)。

## 2026-09-24: 最小GNG高速化実験の標準構成に採用しなかった候補

- 対象: 独立した`gng_runtime_trials`の標準ビルド設定。各候補の比較用実装は保持、本番採用の判断ではない。
- 判断根拠: エージェントによる同一bag・同一学習回数のRelease測定と出力照合。[実測結果](../../benchmarks/gng_runtime_trials_20260924/README.md)。
- 最小ヒープ: 基準12.29 msに対し8.92 ms。探索開始位置保持の8.56 msと比べて利点がなく、標準構成への採用見送り。異なるノード削除・追加パターンで有利となる測定結果が再検討条件。
- CPU向け命令選択: 完全一致版比の短縮率が約−2.4〜＋0.8%で、明確な改善なし。標準構成への採用見送り。対象CPUと代表入力で一貫した改善が確認できる場合に再検討。
- 基数ソート: 全入力と4,000回学習を維持して約10〜11 msへ短縮したが、加算順序の丸め差が長期の接続判断へ波及。出力完全一致を維持する標準設定への採用見送り。微小な座標・構造差を許容する用途では既存の任意オプションとして利用可能。許容差・評価範囲が明示された場合、標準設定への採用を再検討。
- 同日追記: ユーザーの追加承認により、独立実験版の基数ソートを標準ONとして再採用。本番は出力一致を確認した空きID管理・重心走査統合だけを採用。[追加判断と検証](releases/2026-09-24_gng_production_efficiency.md)。

## 2026-09-14: 未確定のROS評価指標への暫定式

- 対象範囲: 計算根拠と用途が未確定で、候補選定・制約判定に使われていないROS評価指標。
- 判断: 暫定式の採用見送り。物理的な成立条件や利用中の順位付けの一括削除は対象外。
- 判断根拠: ユーザーの「未使用な評価指標は一旦計算式はなし」という方針。
- 理由: 仮の数値を計算済みの品質・安全性と誤認することの防止。
- 再検討条件: 測定対象・単位・用途と妥当性確認方法の確定後、個別に実装を判断。
- 今回の適用範囲: 参照調査に基づき、配信用の`joint_limit_margin_min/mean`と`estimated_energy/duration`を未計算化。利用中の面積比順位付けは維持。[変更内容](releases/2026-09-14_provisional_candidate_metrics.md)

## 2026-09-14: 接続条件緩和による平面クラスタ改善案

- 対象範囲: 直前の`merge_connection_requirement`の2本から1本への緩和と、共通YAMLのCPU版への転送を含む[平面クラスタ改善案](releases/2026-09-14_plane_merge_connections.md)。
- 判断: 現案は不採用。ユーザーからのreject記録依頼に基づく判断。
- 理由: ユーザー評価は「性能良くなっていない」「多分あまり良くない感じ」。具体的な悪化箇所・原因・処理時間の変化は未計測であり、悪化の断定や変更項目別の寄与の切り分けは未実施。
- 検証との区別: 回帰テスト成功と比較再生でのクラスタ数減少は実施済みの事実だが、クラスタリング性能の改善や統合の正しさの裏付けとしては不十分。
- 再検討条件: 未確定。検討案は、同一入力で残存分割・誤統合・時系列安定性を比較し、改善の根拠を確認した上での再採用判断。接続条件緩和やYAML転送の恒久禁止という判断ではない。
- 今回の適用範囲: 不採用記録と変更履歴への参照追記のみ。コード・設定の取り消し、ビルド、ROSプロセスの操作なし。
- 2026-09-24の別方式: 通常2本の要求を維持した、小断片・短い接続・厳しい残差・連続確認に限定する救済を実装。全体の要求を1本へ下げる旧案とは別。[仕様・検証](releases/2026-09-24_plane_cluster_consistency.md)。

## 2026-09-15: Surface clustering micro-optimizations and exact patch cache

- Scope/decision: agent assessment for the captured GNG stream; production adoption deferred for three fitting micro-optimizations and an exact unchanged-patch cache.
- Evidence: identical captured inputs and five interleaved Release runs showed only 1.4–2.8% total reductions. No exact unchanged plane patches were observed across 200 patch comparisons. Details and reproduction commands: [investigation](designs/curved_surface_position_fit.md#2026-09-15-live-cost-and-incremental-update-investigation).
- Correctness: the first two prototypes preserved recorded non-timing outputs exactly; fixed Eigen dimensions introduced only tested floating-point rounding differences. All three passed 66 existing tests. The decision is based on limited benefit, not test failure or a user rejection.
- Reconsider when representative inputs show a substantial unchanged-patch fraction, or targeted candidate-fitting work produces a material and repeatable gain. Incremental fitting in general is not rejected. Production code/settings remain unchanged.

## 2026-09-15: smooth_graphによる既定曲面方式の置換

- **対象・判断**: モデル当てはめなしの連続面抽出を既定方式にする案は採用見送り。比較用の起動選択肢としては実装済み。
- **根拠**: 同じ実入力21フレームで7.407 msから1.159 msへ短縮した一方、平均1,420/1,545ノードが一つの成分に統合。背景平面と小領域の境界を維持できず、現在の形状クラスタの同等置換とは扱えない。[仕様・測定記録](releases/2026-09-15_smooth_surface_graph.md)。
- **再検討条件**: モデル当てはめなしで背景平面・細い橋・法線変化の境界を扱い、多シーンで分割品質と実行時間を確認できた場合。差分管理も今回の変動入力では3.7%増であり、常時高速化とはみなさない。
