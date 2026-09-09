# 2026-09-09 - GNG Web Toolsの配置集約

## Summary

HTML用のWASMアダプタとbag変換CLIを `gng_web_tools/` 配下へ集約。
ROSパッケージ名・実行コマンド・学習ロジックは維持。

## Changed

- `gng_wasm_core/` → `gng_web_tools/wasm/`。
- `gng_bundle_exporter/` → `gng_web_tools/exporter/`。
- WASMのCMake・ビルドスクリプトで基幹ソースへの相対パスを更新。
- `ToPo-FUZZY_Manipulation_v1.html` の配布JS読み込み先を更新。
- README・設定ファイル案内・CLIの既定出力先説明を新配置へ更新。
- 既存のresultsディレクトリ、隠しファイル、配布JS/WASMも一括移動。

## Added

- `gng_web_tools/README.md` に役割・ビルド・既存コマンドを整理。
- `tests/wasm_load.cjs`: HTML参照先・WASM ABI 3・500回学習・JSON出力の確認。
- `tests/exporter_load.py`: 新ソース位置・既定出力先・一時PointCloud2 bagのCLI変換確認。

## Fixed

新配置からの基幹ソース参照とHTMLからの配布ファイル参照。

## Removed

旧ルート直下の2ディレクトリ。互換用symlinkは未作成。
機能・ROSパッケージ・実行ファイルの削除なし。

## Behavior Impact

- 親ディレクトリにpackage.xmlを置かず、colconは配下の2パッケージを個別検出。
- WASMとROS依存Pythonのビルドは独立。WASM側へのROS依存追加なし。
- ROSパッケージ名 `gng_wasm_core`、`gng_bundle_exporter` を維持。
- `ros2 run gng_bundle_exporter gng-bundle-export` は変更なし。
- ソースまたはsymlink-install時の既定出力先は `gng_web_tools/exporter/results/`。
- CMakeの旧ソース位置キャッシュは、移行時の `--cmake-clean-cache` 指定で更新。
- 配布JS/WASM、学習アダプタのC++ソース、基幹GNGソースは内容変更なし。
  移動した既存ファイルの内容差分はCMake・ビルドスクリプト・README・CLIヘルプ・gitignoreの6ファイルのみ。
- 新パスでも空resultsディレクトリの保持ファイルを追跡できるよう、gitignoreに例外を追加。

## Topics / Params / Messages

トピック、ROSパラメータ、メッセージ定義、WASM ABIの変更なし。

## Verification

Docker `gng_cpu_container` 内で実行。各ビルド・テストはtimeout付き。

```bash
docker exec -w /ros2_ws gng_cpu_container timeout --kill-after=5s 180s bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  colcon build --base-paths /ros2_ws/src/gng_web_tools \
    --packages-select gng_wasm_core gng_bundle_exporter \
    --symlink-install --cmake-clean-cache \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    --event-handlers console_direct+
'
```

両パッケージのビルド・既存install先への反映成功。新配置からのcolcon検出と
`ros2 run gng_bundle_exporter gng-bundle-export --help` の成功を確認。
nativeテストは `nodes=10 edges=13 iterations=500`。

```bash
docker exec gng_cpu_container timeout --kill-after=5s 30s \
  node /ros2_ws/src/gng_web_tools/tests/wasm_load.cjs
docker exec gng_cpu_container timeout --kill-after=5s 120s \
  bash /ros2_ws/src/gng_web_tools/wasm/scripts/build_wasm.sh /tmp/gng-web-tools.K4DFhB/dist
docker exec gng_cpu_container timeout --kill-after=5s 45s bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  set -e
  node /ros2_ws/src/gng_web_tools/tests/wasm_load.cjs /tmp/gng-web-tools.K4DFhB/dist/gng_wasm_core.js
  python3 /ros2_ws/src/gng_web_tools/tests/exporter_load.py
'
```

- 配布済みWASM: `nodes=9 edges=10 iterations=500`、ABI・JSON確認成功。
- 一時再生成WASM: `nodes=9 edges=8 iterations=500`、ABI・JSON確認成功。
- 既存ROSコマンドで一時bagの1メッセージをJSON出力。新ソース位置・既定出力先も一致。
- 配布済みと再生成WASMのグラフ同一性を保証するテストではなく、読み込み・実行の確認。
- 一時再生成物は配布済みファイルへ上書きせず、検証後に削除。

## Risk / Notes

- ブラウザ画面の操作・描画は未検証。HTMLの実参照先を使ったNode.jsでのWASM実体化を確認。
- colconのexporterパッケージindex markerに関する既存警告、および基幹ソースの既存コンパイル警告は残存。
- 古い配置を参照する外部スクリプト・ブックマークは新パスへの更新が必要。
  過去の作業ログ・リリースノートの旧パス表記は履歴として維持。
- 自分が起動したビルド・テストは全終了。ROSノード・HTTPサーバーの新規起動や既存プロセスへの停止操作なし。
  作業中の他操作によるviewer終了・GNG起動には介入なし。
