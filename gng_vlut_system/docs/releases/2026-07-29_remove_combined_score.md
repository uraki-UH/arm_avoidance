# 2026-07-29 - Remove combined score

## Summary

`min_singular_value * joint_limit_score`だけを表していた旧統合スコアを削除した。

## Removed

- GNGノードの内部状態と計算処理
- 把持候補の`metric_names/metric_values`
- `/evaluation_metrics`のschemaとsample
- 新規GNGバイナリへの保存
- 関連スクリプトと評価指標文書

## Compatibility

- GNGバイナリversionを9へ更新する。
- version 4から8の既存ファイルは旧fieldの4バイトを読み捨てて継続利用できる。
- version 9は旧fieldを保存しないため、ノードあたり4バイト小さくなる。
- `/evaluation_metrics`のschema revisionを5へ更新する。

## Verification

- `combined_score`の実コード参照が残っていないことを確認
- `gng_vlut_system`のビルド成功
- 旧version 6 GNGバイナリの読込成功
- 旧version 6をversion 9として別名保存し、5501有効ノードを再読込できることを確認
- build前後でDockerコンテナ内にテストプロセスが残っていないことを確認
