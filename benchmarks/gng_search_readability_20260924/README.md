# CPU GNGの探索本体を既存関数へ集約（2026-09-24）

## 1. 要約

`getMinGrid()`と`getDownSamplingGrid()`の中へ探索本体を戻し、
`get_min_grid_impl`・`get_down_sampling_grid_impl`と内部宣言を削除。
呼び出し側の名称・引数を維持し、関数内のラムダに参照先を渡す構成。
探索用の連続配列と通常ノード配列の選択は探索開始時の1回だけ。

| input.voxel_grid_unit | 全体：整理前→後 | 照合・重点候補区間：整理前→後 |
| --- | ---: | ---: |
| 0.1 m | 88.798→88.825 ms | 65.230→65.420 ms |
| 0.5 m | 32.562→31.847 ms | 10.991→10.781 ms |

今回の比較では目立った性能低下なし。読みやすさの整理であり、試行間変動を含む時間差を新たな高速化率とは扱わない。

## 2. 条件・検証

- Macnica交差点bag `/lidar_points`、約16万点、上限2万ノード、学習4,000回、node.grid=0.5 m。
- Release＋LTO、CPU 0固定、各120フレームの［20,120）平均の3試行中央値。第2試行は前後順を反転。既存ROSを維持して測定。
- 入力転送＋gng_exec＋グラフ取得を計測。比較コピーのみ乱数・時間刻みを固定。
- 通常720組、voxel無効30組、観測・統計・重み付き重点入力など90組の計840組で全照合項目一致。
- 本番CTest21件＋API2件、WASM native1件成功。配布版の通常乱数・実時間による実入力30フレームも成功。
- 本番Release版へ原子的に反映済み。既存GNGは現在読み込み中の版を保持し、次回起動から適用。
- 既存ROS11プロセス・3コンテナを維持。検証プロセスは全終了、今回の一時ビルドを削除。
- 環境確認の初回は差替えに伴う`(deleted)`表記を相違として検出。読み込みアドレス・権限・inodeの比較へ修正し、同一ライブラリの保持を再確認。

[集計](report.json)、[反映](install.json)、[環境確認](runtime_verification.json)、[検証スクリプト](verify.py)。
[既存検証手順](../gng_unused_mapping_20260924/README.md)を再利用し、生成スクリプト・生ログ・フレームJSON・ソースコピーは`artifacts/gng_search_readability_20260924/`に保存。
再現には削除前ではなく今回の整理前の`before_source`、同じbag、ROS Humbleが必要。

起動コマンド（すべて終了済み）：

```bash
docker exec gng_cpu_container bash -lc 'timeout --signal=INT --kill-after=10s 1300 python3 /ros2_ws/src/benchmarks/gng_search_readability_20260924/verify.py build'
docker exec gng_cpu_container bash -lc 'timeout --signal=INT --kill-after=10s 1900 python3 /ros2_ws/src/benchmarks/gng_search_readability_20260924/verify.py measure'
docker exec gng_cpu_container bash -lc 'timeout --signal=INT --kill-after=10s 700 python3 /ros2_ws/src/benchmarks/gng_search_readability_20260924/verify.py finish'
docker exec gng_cpu_container bash -lc 'timeout --signal=INT --kill-after=10s 90 python3 /ros2_ws/src/benchmarks/gng_search_readability_20260924/verify.py cleanup'
```

`finish`は検証済みライブラリの配布を含む。最後の`cleanup`は環境確認の修正後の再実行。
