# 2026-09-28 - 共有CPUコード変更後のWASMリンク修正

## 1. 要約

全体Releaseビルドが`gng_wasm_core_cpu_kernel_test`のリンクで停止する原因を修正。
共有`cugng.cpp`から追加された`VoxelGrid::has_occupied_cell(unsigned int) const`の呼出しに対し、
WASM側のソース一覧に実装元`voxel_grid.cpp`が含まれていなかった。

`gng_web_tools/wasm/CMakeLists.txt`の通常ライブラリ・Emscripten実行対象と、
`gng_web_tools/wasm/scripts/build_wasm.sh`へ同ファイルを追加。
計算処理・共有CPUコードの変更なし。

## 2. 条件・検証

- 失敗根拠：コンテナ内`/ros2_ws/log/build_2026-09-28_18-45-14/gng_wasm_core/stderr.log`。
- 前回の計画部品整理の検証は`gng_vlut_system`単体。全体ビルド成功の証拠ではない。
- 修正後の`gng_wasm_core`ビルド・既存CTest 1件成功。
- EmscriptenによるWASM生成と既存Nodeロード試験成功。500反復、9ノード・7エッジ。
- WASM生成時は既存のCリンケージ戻り値警告あり。ブラウザ実画面は未検証。
- 生成先は一時ディレクトリ。配布済み`dist`成果物の更新なし。
- 全体検証の初回は並行ビルド中に`ais_gng`で`file truncated`。同一出力先の競合が疑われるため、他方の終了後に再実行。
- 最終Release全体ビルドは29パッケージ成功。起動したビルド・試験は全終了。
- ROSノードの起動・既存プロセスへの停止操作なし。

コマンドは`gng_cpu_container`内、`/ros2_ws`でROS Humbleとinstall環境を読込み後に実行。

```bash
colcon build --packages-select gng_wasm_core --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
ctest --test-dir /ros2_ws/build/gng_wasm_core --output-on-failure
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash /ros2_ws/src/gng_web_tools/wasm/scripts/build_wasm.sh /tmp/wasm_link_fix_artifacts
node /ros2_ws/src/gng_web_tools/tests/wasm_load.cjs /tmp/wasm_link_fix_artifacts/gng_wasm_core.js
```
