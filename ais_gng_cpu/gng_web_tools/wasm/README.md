# gng_wasm_core

`ais_gng_cpu/src/gng_cpu`の学習コアをHTMLから実行するWASMアダプタ。
CUGNGの勝者探索・ノード／エッジ更新・年齢処理を共有。ROS・認証・ボクセル前処理・分類・クラスタリングは対象外。

## ビルド

Emscriptenの`em++`が必要。

```bash
cd ais_gng_cpu/gng_web_tools/wasm
bash scripts/build_wasm.sh
```

出力はWASM内蔵`dist/gng_wasm_core.js`。CMakeのEmscriptenターゲットはJS・WASMの2ファイル。
通常のcolconはnativeアダプタだけを生成。[Dockerビルド・読込検証](../README.md)。

## API

既存の`gng_wasm_*` C API・JSON形式を維持。
入力設定→点群設定→学習→Graph JSON取得。定義は[src](src/)を参照。
