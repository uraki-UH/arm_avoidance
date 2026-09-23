# GNGの学習量を維持した実装比較

前回の`gng_minimal_comparison`からの独立コピー。`COLCON_IGNORE`付きで本番ライブラリ・ROS launchへの組込みなし。

学習回数・入力上限・入力範囲・ボクセル幅・ノード上限・探索精度・ノード／エッジの更新条件を維持した高速化実験。`tree_raw`、`grid_raw`、`tree_voxel`、`grid_voxel`の4ライブラリは前回と同じ構成。

## 実装の切替

| CMake設定 | 内容 | 標準値 |
| --- | --- | --- |
| `GNG_FREE_NODE_MODE` | 0:毎回先頭から探索、1:最小空き番号の開始位置を保持、2:最小ヒープ | 1 |
| `GNG_ENABLE_LTO` | ファイルをまたぐ最適化とインライン化 | ON |
| `GNG_FUSE_VOXEL_REDUCTION` | ソート後の区間検出・重心計算の走査統合 | ON |
| `GNG_RADIX_VOXELS` | 32bitセル番号を8bitずつ安定基数ソート | OFF |
| `GNG_NATIVE_CPU` | 実行CPU向け命令選択、`-ffp-contract=off` | OFF |
| `GNG_DETERMINISTIC_BENCHMARK` | 乱数をフレーム番号、LPFの時間刻みを0.1秒に固定 | ON |
| `GNG_BUILD_VARIANTS` | ビルド対象のライブラリ組合せ | 上記4種 |

標準の組合せ版は最小の空きIDを以前と同じ順序で選択。ノード削除時に探索開始位置を戻す方式。入力重心の加算順序も維持。`-ffast-math`、近似探索、学習点や学習回数の削減なし。

基数ソート版は有効点をすべて保持し、同じセル番号の順に同じ個数の重心を生成。ただし同じセル内の点の順序が変わるため、浮動小数点加算の丸め差あり。通常の組合せ版と分離した任意の実験オプション。

`Vec3f`の既定コンストラクタが一時オブジェクトだけを生成していた既存不具合を、このコピー内で修正。ゼロ長ベクトルの正規化結果をゼロへ初期化。基準版を含む全8構成へ同じ修正を適用。

## ビルド

既存`gng_cpu_container`内:

```bash
cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_runtime_trials \
  -B /tmp/gng_runtime_trials_build/optimized -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/gng_runtime_trials_build/optimized -j 4
ctest --test-dir /tmp/gng_runtime_trials_build/optimized --output-on-failure
```

基数ソート版では別のビルド先と`-DGNG_RADIX_VOXELS=ON`を指定。コンパイラ最適化なし等の比較設定は[再現スクリプト](../../../benchmarks/gng_runtime_trials_20260924/build_trials.sh)を参照。通常のROS実行用ではなく、比較用の時間刻み固定ビルド。

ライブラリ名は前回互換の`libgng_minimal_<variant>.so`。実験ごとに`artifacts/gng_runtime_trials_20260924/<method>/`へ保存し、本番インストール先へのコピーなし。

## 検証範囲と出力

- 既存API、空入力、境界・非有限入力、最近傍、ノード移動・削除・セル上限、ゼロベクトル、重心計算順序の検証。
- 計測JSONは各フレームの入力数・候補数・最近傍検索回数・原点選択回数・ノード更新数と全グラフのハッシュを保存。
- 連続入力の追加計測では、座標の丸め差と接続構造を分離する`topology_sha256`を保存。対象はノードID・生成フレーム・ラベル・エッジ。
- 品質評価とbag読込は時間計測外。GNG本体時間、入力転送、出力変換を分離。

[実測・比較条件](../../../benchmarks/gng_runtime_trials_20260924/README.md)に結果を記録。

全点の観測寿命維持・重点学習・クラスタリングを含まない前回の最小構成を維持。本番GNGとの機能差は前回から継続。エッジ表のメモリ構造も変更なし。
