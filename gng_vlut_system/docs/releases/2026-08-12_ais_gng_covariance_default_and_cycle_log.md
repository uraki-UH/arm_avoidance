# 2026-08-12 - AiS-GNG covariance default and cycle log

## Summary

AiS-GNGの共分散楕円用処理を既定で停止し、実行周期と工程別処理時間をROS 2のINFOログへ出すようにした。

## Changed

- `node.covariance_enabled`の既定値を`true`から`false`へ変更した。
- `graspnet.yaml`で共分散処理を明示的に無効化した。
- 共分散無効時は、共分散履歴の更新と前フレームノードのスナップショット生成を呼び出さない。

## Added

- `performance.log_interval_ms`を追加した。既定値は`5000` msで、`0`なら周期ログを無効化する。
- 周期、Hz、全処理時間、工程別時間、入出力規模、共分散状態をINFOログへ出す。

## Fixed

- Viewerで共分散楕円を表示していない場合でも、AiS-GNG側が共分散を計算・保持していた無駄を解消した。

## Removed

- 共分散無効時のノード全走査によるデバッグ統計処理を除外した。

## Behavior Impact

- 既定設定では`TopologicalNode.winner_point_covariance`を更新しない。
- 共分散楕円が必要な場合は`node.covariance_enabled:=true`相当の設定が必要になる。
- GNGの学習回数、入力点数、出力topicは変更しない。

## Topics / Params / Messages

- 追加parameter: `performance.log_interval_ms` (`int`, default `5000`)
- 既定値変更parameter: `node.covariance_enabled` (`bool`, default `false`)
- topic名とmessage定義の変更はない。

## Verification

- Docker内で`colcon build --symlink-install --packages-select ais_gng`成功。
- 隔離した`ROS_DOMAIN_ID`で疑似PointCloud2を2 Hz送信し、`cycle period=... (2.00 Hz)`と`covariance=off`を確認した。
- 検証用launch、publisher、プロセスグループが残っていないことを確認した。

## Risk / Notes

- `graspnet.yaml`の`node.learning_num=3000`と`input.point_cloud_num=100000`は維持している。負荷調整は周期ログの工程別実測を基に行う。
