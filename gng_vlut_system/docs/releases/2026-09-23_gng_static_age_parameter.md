# 2026-09-23 - CPU GNGの長期記憶寿命の設定化

## Summary

長期記憶ノードの削除寿命を固定値100から`node.static.s1_age_max`へ接続。

## Changed

- YAML読み込み・コア設定・削除判定・実行中のROSパラメータ変更を接続。
- `at128.yaml`の説明を未観測・未選択寿命（GNG更新回数）へ変更。値100と他の設定値は維持。

## Added

既定値・設定値・実行中変更・不正値拒否・観測時の寿命リセットのAPI回帰テストと、隔離ROS検証スクリプト。

## Fixed

YAML項目が未宣言・未接続のため、値を変更しても固定100のままになる挙動。

## Removed

削除判定内の固定値。既定値100は互換性のため維持。

## Behavior Impact

単位は秒や学習反復数ではなくGNG更新回数。近傍入力の観測または学習の勝者選択で寿命カウンタを0へリセット。
削除判定後の加算順序は従来通り。`node.static.age_min: -1`による昇格無効も従来通り。
通常ノード・クラスタ所属ノードの寿命は別設定。今回の対応はCPUのみ。

## Topics / Params / Messages

- `node.static.s1_age_max`: 正の整数、既定100。既存YAML名を維持。内部識別子は`max_static_s1_age`。
- 0、負値、非整数、内部int範囲外、float API経由で別整数へ丸められるROS値は拒否。
- トピック・メッセージ変更なし。新しいバイナリへの切替はlaunch再起動後、その後の値変更は実行中も可能。

## Verification

- CTest 10件成功。寿命100・3・実行中3→7で、未観測後の削除時刻が設定値と一致。観測継続中の保持も確認。
- 追加で寿命1のAPI試験成功。
- 初回CTestはROS環境の`LD_LIBRARY_PATH`が旧インストール済みライブラリを優先し、新パラメータの2件が失敗。参照先を明示して全件成功。更新後のインストール済みライブラリでも全件成功。
- CPUコアと`ais_gng_component_cpu`のビルド成功。共有ライブラリは一時名から原子的に置換し、ビルド成果物とのSHA-256一致を確認。
- ROS_DOMAIN_ID=227で起動値7、実行中7→3→7、不正値拒否と値保持、起動時0による失敗を確認。
- 既存ROSの停止・再起動なし。試験ノード・子プロセスは全終了。実bagでの寿命比較・viewer描画・性能比較は未実施。

実行コマンド（コンテナ内、すべて終了済み。一時ビルドは検証後に削除）:

```bash
source /ros2_ws/install/setup.bash
cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu -B /tmp/gng-static-age-DucHUF/core \
  -DGNG_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_FRAME_LOG=OFF
cmake --build /tmp/gng-static-age-DucHUF/core -j2
LD_LIBRARY_PATH=/tmp/gng-static-age-DucHUF/core:$LD_LIBRARY_PATH \
  ctest --test-dir /tmp/gng-static-age-DucHUF/core --output-on-failure --timeout 30
LD_LIBRARY_PATH=/tmp/gng-static-age-DucHUF/core:$LD_LIBRARY_PATH \
  timeout 30 /tmp/gng-static-age-DucHUF/core/gng_static_age_api_test 1 1
cmake --build /ros2_ws/build/gng_cpu --target gng_cpu -j2
cmake --build /ros2_ws/build/ais_gng --target ais_gng_component_cpu -j2
ulimit -c 0
timeout -s INT -k 15 90 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_static_age_parameter.py
```

子ノードの起動引数・停止処理は[検証スクリプト](../../../ais_gng_cpu/src/ais_gng/test/check_static_age_parameter.py)に記載。試験用ROSログは専用一時ディレクトリ内で削除。

## Risk / Notes

長期記憶が有効な環境で寿命を短縮すると、既に寿命を消費したノードが次回更新で削除される場合あり。
長期記憶への昇格設定・ノード数上限・学習回数の自動変更なし。既存の未コミット差分は保持。
