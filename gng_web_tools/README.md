# GNG Web Tools

`ToPo-FUZZY_Manipulation_v1.html` 向けのGNG実行・データ変換ツール群。
配置のみを集約し、WASM向けC++とROS依存Pythonのビルド・依存関係は分離。

```text
gng_web_tools/
├── wasm/       # ROSパッケージ名: gng_wasm_core
├── exporter/   # ROSパッケージ名: gng_bundle_exporter
└── tests/      # 配置・WASM読み込みの確認
```

- [wasm](wasm/README.md): 基幹GNGソースからブラウザ実行用WASMを生成するアダプタ。
- [exporter](exporter/README.md): ROS bagをHTML用JSON/gzipへ変換するCLI。
- 親ディレクトリはROSパッケージではなく、colconは配下の2パッケージを検出。
- 基幹GNGソースは従来どおり `ais_gng_cpu/src/gng_cpu` に配置。

## Docker内でのビルド

既存のCMakeキャッシュに旧ソースパスが残るため、移行時は `--cmake-clean-cache` を指定。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  colcon build --base-paths /ros2_ws/src/gng_web_tools \
    --packages-select gng_wasm_core gng_bundle_exporter \
    --symlink-install --cmake-clean-cache
'
```

通常のcolconビルドはnativeアダプタのみを生成。配布WASMの再生成は別操作。

```bash
docker exec gng_cpu_container bash \
  /ros2_ws/src/gng_web_tools/wasm/scripts/build_wasm.sh
```

## 既存コマンド

ROSパッケージ名と実行ファイル名は変更なし。

```bash
ros2 run gng_bundle_exporter gng-bundle-export export \
  --bag /path/to/bag \
  --config /ros2_ws/src/gng_web_tools/exporter/config/export_topics.example.yaml \
  --output /path/to/output.json
```

ソースツリーまたはsymlink-install使用時の既定出力先は `exporter/results/`。
通常インストールではPythonパッケージの配置に依存するため、明示的な `--output` 指定を推奨。
旧ルート直下の2フォルダは廃止。HTMLの読み込み先は
`gng_web_tools/wasm/dist/gng_wasm_core.js`。

## 読み込みテスト

```bash
node gng_web_tools/tests/wasm_load.cjs
```

HTMLの参照先、ABI、入力点群からの学習・JSON出力を確認。
任意の第1引数で、別ディレクトリへ再生成したJSの検証も可能。

エクスポータの確認は、ROS環境をsourceした状態で実行。

```bash
python3 gng_web_tools/tests/exporter_load.py
```

一時bagと出力ファイルはテスト終了時に削除。実トピックへの送信なし。
