# CPU GNGの未使用対応表削除（2026-09-24）

## 1. 要約

本番CPU版の`voxel2node_ids`の確保・書込み・ノードID順ソートを削除。
入力基数ソート、全点照合、重点学習、観測、クラスタは従来どおり。

| voxel幅 | 全体：削除前→後 | 照合・重点候補区間：削除前→後 |
| --- | ---: | ---: |
| 0.1 m | 93.319→88.383 ms | 69.675→65.054 ms |
| 0.5 m | 32.444→32.460 ms | 11.261→10.929 ms |

0.1 mの全体は5.3%短縮。0.5 mは区間短縮を確認したが、全体の改善は未確認。
従来の対応整列だけの時間を、今回の全体短縮量として扱わない。

## 2. 条件・検証

- Macnica交差点bagの`/lidar_points`。約16万点、上限20万点、上限2万ノード、学習4,000回、node.grid=0.5 m。
- Release（-O3＋LTO）、CPU 0固定。各120フレーム、先頭20フレームを除外した平均の3試行中央値。第2試行は前後順を反転。
- 計測範囲はC API入力転送＋gng_exec＋グラフ取得。bag読込・ハッシュ照合・ROS通信・外部分類器は対象外。既存ROSを動かした環境での測定。
- コピーだけ乱数と時間刻みを固定。各フレームのグラフ・ラベル・クラスタと所属順・入力voxel・重点候補数・学習回数を比較。
- 通常720組、voxel無効30組、観測・統計・重み付き重点入力・学習イベント・地図差分有効90組の計840組で全照合項目一致。
- 本番CTest21件＋API2件、WASM native1件成功。配布版の通常乱数・実時間による実入力30フレームも成功。
- 公開C APIと公開シンボルを維持。本番Releaseライブラリを原子的に反映済み。既存GNGは旧inodeを使用し、次回起動から適用。
- 起動したビルド・比較・試験は全終了、一時ビルド削除済み。既存ROS11プロセスと3コンテナを維持。状態確認の初回は確認コマンド自身を誤検知し、判定を修正して再確認。

[集計](report.json)、[配布確認](install.json)、[環境確認](runtime_verification.json)。
生ログ・フレームJSON・ソースコピーは`artifacts/gng_unused_mapping_20260924/`に保存。
比較元は削除前に保存した`before_source`を使用し、[前](before_source_sha256.json)・[後](after_source_sha256.json)のSHA-256で識別。

実行コマンド（既存の比較用コピー・bag・ROS Humbleが必要）：

```bash
docker exec gng_cpu_container bash -lc 'python3 /ros2_ws/src/benchmarks/gng_unused_mapping_20260924/prepare.py && timeout --signal=INT --kill-after=10s 1200 bash /ros2_ws/src/benchmarks/gng_unused_mapping_20260924/build.sh'
docker exec gng_cpu_container bash -lc 'timeout --signal=INT --kill-after=10s 1800 bash /ros2_ws/src/benchmarks/gng_unused_mapping_20260924/run.sh'
python3 benchmarks/gng_unused_mapping_20260924/report.py
docker exec gng_cpu_container bash -lc 'timeout --signal=INT --kill-after=10s 600 bash /ros2_ws/src/benchmarks/gng_unused_mapping_20260924/validate.sh'
docker exec gng_cpu_container bash -lc 'python3 /ros2_ws/src/benchmarks/gng_unused_mapping_20260924/cleanup.py'
```

`validate.sh`は確認済みライブラリの配布を含むため、単なる再集計は`report.py`のみで実施。
