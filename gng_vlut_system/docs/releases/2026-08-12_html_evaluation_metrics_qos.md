# 2026-08-12 - HTML evaluation metrics QoS

## Summary

HTMLを`/evaluation_metrics`のPublisherより後に起動しても、保持された最新の評価指標を受信できるようにする。

## Changed

- HTMLの`/evaluation_metrics`購読を`RELIABLE`、`TRANSIENT_LOCAL`に変更した。

## Behavior Impact

- ROS PublisherとHTMLの起動順に依存せず、既存の評価指標schemaを受信できる。
- 新しい評価指標schemaやPublisherは追加しない。

## Verification

- HTML内JavaScriptの構文確認。
- rosbridge経由で既存`EvaluationMetrics`メッセージを受信。
