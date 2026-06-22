# gng_wasm_core

`ais_gng_cpu` の GNG 更新処理を、HTML から切り離して使えるようにするための独立コアです。

## 目的

- ROS 依存なしで GNG 更新を実行する
- HTML 側は bundle JSON や点群 JSON を渡すだけにする
- Emscripten で WASM としてビルドする

## ざっくり API

- `gng_wasm_set_points(float* xyz, uint32_t point_count)`
- `gng_wasm_set_config(...)`
- `gng_wasm_run()`
- `gng_wasm_write_graph_json(char* dst, uint32_t capacity)`

## 現状

- HTML の内蔵実装とは別の独立実装として追加しています
- 現段階では標準 GNG の更新コアを C++ で持つ形です
- `em++` で直接ビルドすると `gng_wasm_core.js` と `gng_wasm_core.wasm` を出力できます

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

- `dist/gng_wasm_core.js`
- `dist/gng_wasm_core.wasm`

## 補足

- 通常の `colcon build` は native の C++ コアとしてそのまま使えます
- wasm 用のビルドは `scripts/build_wasm.sh` を使ってください
- もしローカルに Emscripten がなければ、`docker run --rm -v ~/uraki_ws:/work -w /work/gng_wasm_core emscripten/emsdk:latest bash scripts/build_wasm.sh` でも生成できます
