# 2026-08-12 - GraspNet GNG age parameter names

## Summary

`graspnet.yaml`のnode寿命とedge寿命がAiS-GNGへ適用されていなかったparameter名の不一致を修正した。

## Changed

- `node.s1_age`を正式名`node.s1_age_max`へ変更した。
- `node.edge_age_max`を正式名`edge.age_max`へ変更した。
- ユーザーが設定した寿命値、node間隔、入力点数などの調整値は変更していない。

## Added

- なし。

## Fixed

- safe node寿命`1000`とedge寿命`1000`が無視され、それぞれ既定値`6`と`100`で実行される問題を修正した。

## Removed

- AiS-GNGが宣言していない`node.s1_age`と`node.edge_age_max`の使用をやめた。

## Behavior Impact

- `graspnet.yaml`起動時にsafe node寿命とedge寿命の設定値`1000`がGNGへ渡る。
- 設定変更を反映するにはAiS-GNGの再起動が必要である。

## Topics / Params / Messages

- 使用parameter: `node.s1_age_max`, `edge.age_max`
- topicとmessage定義の変更はない。

## Verification

- 隔離した`ROS_DOMAIN_ID`で`ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml`を起動した。
- 起動ログで`node.s1_age_max [6, 1000, 12, 12]`と`edge.age_max 1000`を確認した。
- 検証用プロセスが残っていないことを確認した。

## Risk / Notes

- nodeがsafe以外へ再ラベルされた場合、safe表示からは外れる。今回の修正は寿命とedge切断による削除を対象とする。
