# 2026-07-29 - Viewer evaluation metric selector

## Summary

把持推定評価指標をROSまたはrosbagから認識し、ファジィ入力として選択できるGUIを追加した。

## Added

- `EvaluationMetrics` schemaから評価指標チェックボックスを動的生成する。
- 選択した数値指標へ既定のLow、Medium、High Membership Functionを生成する。
- 受信sampleの範囲による0〜1正規化と、候補scope IDによる値参照を追加する。

## Behavior Impact

チェックONの評価指標だけがMembership Functionとルール条件の入力候補になる。
チェックOFFでは定義を保持したままファジィ推論条件を無効化する。
string型は表示のみで選択できない。

## Topics / Params / Messages

- 既存の `gng_control_msgs/EvaluationMetrics` とrosbag bundle JSONを使用する。
- ROS topic名、message定義、parameterの変更はない。

## Verification

- HTML内のJavaScript構文解析
- ヘッドレスChromeで動的チェックボックス生成、OFF時のMF候補除外、ON時の再追加を確認

## Risk / Notes

- float arrayは配列要素の平均値をMembership Function入力として扱う。
