# gng_wasm_core

`ais_gng_cpu/src/gng_cpu` の学習コアを、HTMLからWebAssemblyで実行するためのアダプタです。

## 目的

- 基幹 `CUGNG` と同じ勝者探索、ノード更新、エッジ更新、年齢処理を実行
- HTML 側は bundle JSON や点群 JSON を渡すだけにする
- Emscripten で WASM としてビルドする

## ざっくり API

- `gng_wasm_set_points(float* xyz, uint32_t point_count)`
- `gng_wasm_set_config(...)`
- `gng_wasm_run()`
- `gng_wasm_write_graph_json(char* dst, uint32_t capacity)`

## 実装範囲

- `CUGNG`、`Param`、`Node`、`Vec3f` を基幹ソースから直接コンパイルする構成
- HTMLの既存 `gng_wasm_*` C APIとJSONグラフ形式を維持
- ROS、認証、ボクセル前処理、ラベリング、クラスタリングは含めない構成
- `scripts/build_wasm.sh` はWASM内蔵の `dist/gng_wasm_core.js` を生成
- CMakeのEmscriptenターゲットは `gng_wasm_core.js` と `gng_wasm_core.wasm` の2ファイルを生成

## ビルド手順

### 1. Emscripten を使える状態にする

`em++` が使えることを確認してください。

### 2. wasm を生成する

```bash
cd ~/uraki_ws/gng_wasm_core
bash scripts/build_wasm.sh
```

出力先を変えたい場合:

```bash
bash scripts/build_wasm.sh dist
```

### 3. 生成物

- `scripts/build_wasm.sh` の出力: `dist/gng_wasm_core.js`
- CMake Emscriptenターゲットの出力: `gng_wasm_core.js` と `gng_wasm_core.wasm`

## 補足

- 通常の `colcon build` は基幹GNG学習コアをリンクしたnative C++アダプタを生成
- wasm 用のビルドは `scripts/build_wasm.sh` を使ってください
- もしローカルに Emscripten がなければ、`docker run --rm -v ~/uraki_ws:/work -w /work/gng_wasm_core emscripten/emsdk:latest bash scripts/build_wasm.sh` でも生成できます
