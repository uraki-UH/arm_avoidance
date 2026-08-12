# 2026-08-12 - GraspNet safe node mass deletion

## Summary

GraspNet点群を学習中にsafe nodeを含む多数のGNG nodeが同時に削除される問題を修正した。

## Changed

- `graspnet.yaml`の`edge.age_max`を`1000`から`50000`へ変更した。

## Added

- なし。

## Fixed

- 1 frame内の学習iteration中にedgeが寿命へ達し、孤立したnodeが一斉削除される問題を修正した。

## Removed

- なし。

## Behavior Impact

- GraspNet入力時のGNG nodeとsafe nodeがframe間で安定する。
- 古いedgeが従来より長く残るため、地形が大きく変化するリアルタイム入力では別途調整が必要になる。

## Topics / Params / Messages

- 変更parameter: `edge.age_max=50000`
- 現在の`node.learning_num=4000`に対して十分なedge ageを確保する。
- topicとmessage定義の変更はない。

## Verification

- 実行中の`/topological_map`をlabel別node IDで観測した。
- `edge.age_max=1000`では1 frameで最大322 node、safe node最大312個の消失を確認した。
- `node.learning_num=3000`、`edge.age_max=10000`の比較試験では、85 frame連続でnode ID削除0、総node数497〜502を確認した。
- 現在の設定変更にも余裕を持たせるため、最終的に`edge.age_max=50000`を採用した。
- 診断後に一時parameterを元の`1000`へ復元した。

## Risk / Notes

- `edge.age_max`はframe単位ではなくGNG学習iteration単位で作用する。
