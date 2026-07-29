# 2026-07-29 - Filter evaluation metric candidates

## Summary

ROS側の候補選択結果と実行不能候補をHTML側の評価入力へ混入させないようにした。

## Removed

- `/evaluation_metrics`のschemaとsampleから`selected`を除外する。
- 動的な`metric_names`に`selected`が含まれてもpublishしない。
- 現在姿勢から候補経路を生成できない`feasible=false`候補をsampleから除外する。
- `feasible`を評価指標から除外する。
- 未使用のint、bool、string型sample配列へ追加していたダミー値を削除する。
- スカラー値と配列値のfield名を`sample_metric_ids`との対応が分かる名称へ変更する。
- schemaの`metric_ids/value_types`と重複していた`sample_metric_value_types`を削除する。

## Behavior Impact

HTML側の評価指標一覧に`selected`は表示されない。
HTML側へは`feasible=true`候補の評価sampleだけを送る。
診断用`GraspCandidateMetricArray`の`selected`と`feasible`は変更しない。
`sample_int_values`、`sample_bool_values`、`sample_string_values`は空配列になる。
ROS bridgeの切断後も起動ボタンから再接続でき、`/evaluation_metrics`の購読を張り直す。
ライブ受信した指標は把持推定評価指標とMembership Functionの入力候補へ反映する。
rosbridgeコンテナへworkspaceのinstall volumeを共有し、カスタムメッセージを購読可能にする。

## Topics / Params / Messages

- `/evaluation_metrics`のmessage field名を変更する。
- publishされるmetric定義から`selected`と`feasible`を削除する。
- publishされるsampleは`feasible=true`候補だけになる。
- field名変更と重複field削除に伴いschema revisionを4へ更新する。

| 変更前 | 変更後 |
|---|---|
| `sample_float_values` | `sample_metric_scalar_values` |
| `sample_float_array_offsets` | `sample_metric_array_offsets` |
| `sample_float_array_values` | `sample_metric_array_values` |

## Verification

- `gng_control_msgs`と`gng_vlut_system`のビルド
- ビルド済み`EvaluationMetrics` interfaceに`sample_metric_value_types`が存在しないことを確認
- rosbridgeコンテナ内で`gng_control_msgs/msg/EvaluationMetrics`を解決できることを確認
- WebSocket経由の`/evaluation_metrics`購読でschema revision 4、指標21件、候補sample 8件を受信
- build前後でDockerコンテナ内にテストプロセスが残っていないことを確認

## Risk / Notes

- `/evaluation_metrics`内の`selected`または`feasible`を参照していたconsumerは値を取得できなくなる。
- 旧field名を参照するconsumerは新field名への更新が必要。
- sampleの型は`sample_metric_ids`とschemaの`metric_ids/value_types`を対応させて解決する必要がある。
