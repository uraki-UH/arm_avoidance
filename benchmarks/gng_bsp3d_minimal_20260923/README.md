# ボクセル化なし最小tree版の比較

[実装仕様](../../ais_gng_cpu/experimental/gng_bsp3d_minimal/README.md)、[計測結果と制約](../../gng_vlut_system/docs/releases/2026-09-24_gng_bsp3d_minimal.md)。

- `run_benchmarks.sh`: 保存済み元CPU・前回探索枠制限版・最小版の各3試行、未使用voxel設定の比較、最小版の連続300フレーム。
- `summarize.py`: 試行間グラフ、未使用設定による不変性、最近傍回数の照合と再集計。
- `at128_voxel_*.yaml`: 同じ比較設定。最小版が受理しない項目は生JSONの`ignored`に保存。
- `summary.json`: 保存済み集計。再集計による自動上書きなし。
- `sha256.json`: 対象ソース・設定・ライブラリのハッシュ。

実行環境は既存`gng_cpu_container`内のROS Humble、numpy・scipy・PyYAML。bag・トピックは検証記録を参照。追加依存のインストールなし。

[実装README](../../ais_gng_cpu/experimental/gng_bsp3d_minimal/README.md)の手順でビルドし、`libgng_bsp3d_minimal.so`を`artifacts/gng_bsp3d_minimal_20260923/`へ配置。比較元は`artifacts/gng_bsp3d_20260923/libgng_grid.so`と`artifacts/gng_bsp3d_sampled_20260923/libgng_bsp3d_sampled.so`が必要。

コンテナ内:

```bash
bash /ros2_ws/src/benchmarks/gng_bsp3d_minimal_20260923/run_benchmarks.sh
python3 /ros2_ws/src/benchmarks/gng_bsp3d_minimal_20260923/summarize.py
```

出力はGit管理外の`artifacts/gng_bsp3d_minimal_20260923/`。生データのないcheckoutでは再計測が必要。ディレクトリ名の日付は作業開始日の2026-09-23、完了記録は翌2026-09-24。
