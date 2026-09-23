# 2026-09-23 - CPU GNGのパラメータ反映修正

## Summary

入力ボクセルサイズ0が無視される不具合と、実行中の学習設定変更が内部コピーへ届かない不具合の修正。

## Fixed

- `input.voxel_grid_unit=0`の受理。YAML範囲内の各点を一対一で保持し、ボクセル集約を省略。元点番号・範囲対応を維持。
- 有効ボクセルの最後のセルが欠落する件数計算と、範囲内点がない場合の残存状態を修正。
- `node.grid`が更新済みでも失敗を返す分岐を修正。
- 初期化後の有効なコア設定変更を学習側のコピーへ同期。学習回数変更に合わせたイベントバッファの再確保。
- CPU ROSコールバックで一括変更の全値を事前検証。実行中の未対応変更・不正値で成功を返さず、ROS値と内部状態の部分更新を防止。

## Behavior Impact

0指定は従来の暗黙0.1 m間引きと異なり、入力点を集約しないため処理量が増える場合あり。追加フィルタなし。入力範囲・点数上限は既存YAMLに従う。CPUのみ。GPUの動的反映方針は今回の対象外。

## Topics / Params / Messages

動的対応一覧は[README](../../../ais_gng_cpu/README.md#cpuのパラメータ反映)。その他のCPU ROS変更は明示的に拒否しYAML編集・再起動を要求。従来その場で変更できた一部プラグイン設定も、この確実に反映可能な集合以外では再起動を要求。

コアAPIでも初期化後の容量・グリッド・入力設定・初期学習係数変更を拒否。ROSメッセージ定義・YAML値への変更なし。未宣言YAMLキー全体の検出・全パラメータの起動時異常値検査は今回の保証対象外。

## Verification

- CTest 7件成功。間引き0で同じセルの点を保持、正値で平均化・末尾保持、範囲外のみ・空入力・1セル入力を確認。
- API回帰テストで初期化後の学習回数0→12→6000→0と記録イベント数の一致を確認。容量・範囲の動的変更拒否を確認。
- 既存イベントAPI・差分APIテスト成功。
- CPUコンポーネントのビルド成功。隔離ROS_DOMAIN_ID=226で成功・拒否・一括変更時のROS値維持を確認。
- CPU共有ライブラリの原子的な置換とコンポーネントの再ビルド済み。実bag連続実行・viewer描画・性能比較は未実施。

起動コマンド（コンテナ内、全終了済み）:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu -B /tmp/gng_params_build -DGNG_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/gng_params_build -j2
ctest --test-dir /tmp/gng_params_build --output-on-failure --timeout 30
/tmp/gng_params_build/gng_training_event_api_test
/tmp/gng_params_build/gng_map_delta_api_test
cmake --build /ros2_ws/build/ais_gng --target ais_gng_component_cpu -j2
timeout --signal=INT --kill-after=15 60 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_parameter_updates.py
```

ROSテストの子ノード起動引数は[検証スクリプト](../../../ais_gng_cpu/src/ais_gng/test/check_parameter_updates.py)を参照。テストはfinallyで自身の子プロセスを停止。既存ROSの停止・再起動なし。一時ビルドと一時ログは検証後に削除し、結果ログはGit管理外の`artifacts/gng_parameter_application_20260923/`にローカル保管。

## Risk / Notes

利用者のlaunch再起動後に適用。独自にCPUライブラリを読み込むアプリケーションも再起動が必要。既存の未コミット変更と作業中のat128.yaml更新は保持。
