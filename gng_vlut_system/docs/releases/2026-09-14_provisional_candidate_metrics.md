# 2026-09-14 - 暫定評価指標の未計算化

## Summary

候補選定・制約判定に未使用の配信用暫定指標4項目を未計算化。

## Changed

`joint_limit_margin_min`、`joint_limit_margin_mean`、`estimated_energy`、`estimated_duration`は常にNaN。
汎用`/evaluation_metrics`への変換は既存の有限値判定により`sample_metric_valid=false`。

## Added

上記4指標のNaN・無効フラグと、候補ID・選択状態・姿勢・関節値・経路・可操作性の保持を検証する回帰テスト。

## Fixed

同じノードスコアを関節余裕の最小値・平均値として配信する代用処理を停止。

## Removed

簡易エネルギー・時間推定関数と、それだけに使っていた引数・パラメータ。

## Behavior Impact

配信値だけを変更。実際の候補選定で使用中の関節限界スコアや経路計算は維持。
上方把持方式の`footprint_fill_ratio`は順位付けで使用中のため変更なし。
NaNを低評価や0として使わず、未計算として扱う必要あり。既存HTMLの欠損値処理の変更は対象外。

## Topics / Params / Messages

- トピック名とROSメッセージ定義の変更なし。
- `metrics_max_joint_velocity`を廃止。外部YAMLに指定がある場合は削除。
- メッセージフィールドとschema定義は互換性のため維持。新たな代替式・既定スコアの導入なし。

## Verification

検証結果は [progress.md](../progress.md) の今回の実施記録を参照。

## Risk / Notes

指標の再実装は測定対象・単位・用途・妥当性確認方法の確定後。記録方針は [reject.md](../reject.md) を参照。
過去のbag・保存済みスライド等は変更なし。過去記録の値と現行実装の未計算値を区別。
