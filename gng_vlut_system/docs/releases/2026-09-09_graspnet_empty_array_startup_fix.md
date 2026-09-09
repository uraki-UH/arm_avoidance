# 2026-09-09 - Graspnet Empty Array Startup Fix

## Summary

`ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml` の起動直後の異常終了を修正。

## Changed

`config/gng_cpu/graspnet.yaml` の `boundary.lidar_angles_deg` を未指定に変更。

## Added

未指定時のCameraInfo利用と、LiDAR時の6要素指定について設定コメントを追加。

## Fixed

ROS 2 Humbleのパラメータ読込で、明示した空配列 `[]` が値未設定となり、
`InvalidParameterValueException: parameter_value_from failed for parameter 'boundary.lidar_angles_deg': No parameter value set`
でGNG本体がSIGABRT（exit -6）になる問題。

## Removed

YAMLの `boundary.lidar_angles_deg: []`。パラメータ機能自体は存続。

## Behavior Impact

ノード内の型付き空配列の既定値を使用し、従来意図していたCameraInfo経由の視野設定を維持。
GNG学習ロジック・点群入力・曲面モデル・境界証拠の有効設定は変更なし。

## Topics / Params / Messages

トピック・メッセージ・launch引数の変更なし。CameraInfo利用時は当該キーを省略。
LiDAR視野を指定する場合だけ、角度6要素の数値配列を設定。

## Verification

Docker内で同じコマンドを修正前12秒・修正後18秒の有限時間で実行。
修正後は `Initialized successfully`、入力 `I: 100000`、`Nodes: 3000`、平面・非平面・Surface更新ログを確認。
検証に起動したlaunchと3ノードはSIGINT後に正常終了。既存Viewerとbag再生のPIDは維持。

```bash
timeout --signal=INT --kill-after=5s 18s \
  ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml
```

`timeout` の終了コード124は予定した検証停止。各子ノードは正常終了。

## Risk / Notes

現環境のinstall設定はソースへのsymlinkであり、再ビルド不要。
設定をコピー配置した別環境では設定再インストールが必要。
