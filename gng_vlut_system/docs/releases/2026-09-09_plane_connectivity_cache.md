# 2026-09-09 - 平面クラスタの連結証明キャッシュ

## Summary

平面クラスタの分割確認に、前回の全域木による連結証明を導入。
所属集合と木の接続が残るクラスタではBFSを省略し、それ以外だけを再探索。
合成動的入力の探索ノード数は約75〜77%減少。ただし全体時間の改善は小さく、
悪化した測定条件も存在。汎用的な高速化率の確定には至っていない。

## Changed

- 対象: `ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp`。
- ノードID配列の一致、旧所属IDの一致、所属数の一致によるキャッシュ有効性の確認。
- 前回のCSR位置に親エッジが残る場合は直接参照。位置変更時だけ当該行を探索。
- 木エッジの消失・所属変更時は該当クラスタのBFSを実行し、木を更新。
- 入力ノードの個数・ID・並びが変わる場合は全証明を保守的に無効化。
- 全クラスタの連結確認済みフレームでは成分配列生成も省略。
- 分断中のクラスタは証明を保持せず、毎フレームの探索と確認待ちカウンタを維持。

## Added

- `ClusterStatistics::num_connectivity_reused_clusters`: BFS省略クラスタ数。
- `ClusterStatistics::num_connectivity_scanned_nodes`: 実際のBFS訪問ノード数。
- 単体テスト6件。位置更新、非木エッジ変更、木エッジ削除、ノード再配置、
  エッジ再配置、ノードID置換を対象。
- ベンチマークの `dynamic` 入力列と、任意の第4引数による全クラスタ出力のYAML保存。
- ベンチマークのスレッドCPU時間計測。CPU時間にもキャッシュ競合や動作周波数の影響は残存。

## Fixed

判定不具合の変更なし。連結確認の不要な再探索の削減。

## Removed

なし。

## Behavior Impact

- GNG学習、法線・位置の更新、平面フィット、併合条件、分割の確認待ち条件は変更なし。
- 位置や法線が変わっても幾何計算は従来どおり毎フレーム実行。
- 既存GNGエッジだけを使用。証明木をGNGの出力エッジへ追加する処理はなし。
- 全体の入力走査・CSR生成は引き続きO(N+E)。完全な差分グラフ更新ではない。
- キャッシュ用メモリはO(N+C)。Cは内部クラスタ数。
- 初回生成や所属変化の多い入力では、キャッシュ準備の負荷が追加。

## Topics / Params / Messages

トピック・ROSパラメータ・ROSメッセージ定義の変更なし。
上記の統計2項目はC++内部構造体への追加であり、ROS出力への追加ではない。
`ClusterStatistics` のレイアウト変更を伴うため、利用側も同時に再ビルドが必要。

## Verification

### 正確性とビルド

- Docker `gng_cpu_container` 内、g++ 11.4、Eigen3、ROS Humble。
- `-std=c++17 -O3 -DNDEBUG` で変更前後を別オブジェクトへ直接ビルド。
- 変更前ソースSHA256:
  `6854b741564507998e2fb74b761cc4f38597ca1641a27c361f981017a06859d8`。
  直前の逐次併合修正・分割削除早期終了を含む、今回の作業開始時のスナップショット。
- 変更後ソースSHA256:
  `956745dd6c9951008a1d789cdb48e6404668de5d4db02f7da427796c53494d31`。
- 変更前の既存15テスト、変更後の21テストが成功。
- 2,760ノードの動的入力120フレームで、出力全フィールドを17桁精度のYAMLへ書き出し、
  `cmp` による完全一致を確認。先頭20フレームも比較対象。
- 両YAMLのSHA256:
  `0ee748d19c3553aacb59a88d76e60ba0e480699801f0ce8b154e28ef74c7b8ff`。
- 通常のCMake経路でもCPU本体・平面クラスタノード・テスト・ベンチマークのビルド成功。
- install側の実行ファイル・ライブラリはbuild側へのsymlink。
  ビルド済みファイルへ反映済み。既存ROSノードの再起動は未実施。

実行した主なコマンド（一時比較ディレクトリは後片付け対象）:

```bash
docker exec gng_cpu_container timeout --kill-after=5s 180s bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  set -e
  cmake --build /ros2_ws/build/ais_gng --target ais_gng_cpu plane_cluster_incremental_node test_plane_cluster_incremental benchmark_plane_cluster_incremental -j2
  /ros2_ws/build/ais_gng/test_plane_cluster_incremental
'
```

Docker内での出力比較コマンド:

```bash
for version in baseline current; do
  taskset -c 4 /tmp/plane-tree-bench.f5v4Mb/$version dynamic 1 100 /tmp/plane-tree-bench.f5v4Mb/$version.yaml
done
cmp /tmp/plane-tree-bench.f5v4Mb/baseline.yaml /tmp/plane-tree-bench.f5v4Mb/current.yaml
```

### 時間比較

生データ: [経過時間5回比較](2026-09-09_plane_connectivity_cache.csv)。

AMD Ryzen 7 260の論理CPU4に固定。12条件それぞれ変更前後500サンプルを5回実行。
各測定の先頭20サンプルはウォームアップとして除外。
奇数回は変更前→変更後、偶数回は逆順。タイマー対象は `update()` のみ。
入力生成、reset、初期パッチ生成、結果の破棄は対象外。
表は等サンプル数5回の平均。負の増減は短縮を意味し、統計的な優位性の保証ではない。

| 入力 | ノード数 | 変更前平均[ms] | 変更後平均[ms] | 時間増減 |
| --- | ---: | ---: | ---: | ---: |
| steady 1 | 2760 | 0.2368 | 0.2407 | 1.6% |
| steady 4 | 11040 | 1.0312 | 1.0071 | -2.3% |
| birth 1 | 2760 | 0.2119 | 0.2303 | 8.7% |
| birth 4 | 11040 | 0.9521 | 1.0082 | 5.9% |
| merge 9 | 2916 | 0.4298 | 0.4094 | -4.8% |
| merge 17 | 10404 | 1.3488 | 1.3840 | 2.6% |
| reject 9 | 2916 | 0.3396 | 0.3613 | 6.4% |
| reject 17 | 10404 | 1.3773 | 1.6779 | 21.8% |
| chain 81 | 2916 | 0.5437 | 0.5449 | 0.2% |
| chain 289 | 10404 | 2.2403 | 1.5276 | -31.8% |
| dynamic 1 | 2760 | 0.2647 | 0.2504 | -5.4% |
| dynamic 4 | 11040 | 1.1138 | 1.0992 | -1.3% |

全60比較ペアでクラスタ数・併合数・隣接候補数・所属ノード数・最小平面性・checksumが一致。
これらの集約値一致と、上記120フレームの全出力一致は別の検証。

動的入力の平均BFS訪問数:

- 2,760ノード: 679.408ノード/フレーム（入力全ノード比24.6%）。
- 11,040ノード: 2,537.728ノード/フレーム（入力全ノード比23.0%）。
- 定常入力: 全クラスタの証明再利用、BFS訪問0。
- 変更前バイナリの新統計項目は未更新のため0。変更前の探索省略を意味しない。
- `birth` の出力クラスタ数0は出力確認待ちによるもの。内部クラスタ生成・BFSは実行済み。

追加のCPU時間測定は [途中結果](2026-09-09_plane_connectivity_cache_cpu_partial.csv) に分離。
同一条件でも時間が大幅に変動し、比較環境として安定しないため2回目途中で打ち切り。
34行の不完全な結果であり、上表や高速化率の結論には不使用。
自分が起動したtimeoutのPIDを確認して、そのプロセスだけにSIGTERMを送付。

## Risk / Notes

- 動的入力の経過時間は今回の5回平均で1.3〜5.4%短縮だが、他条件では悪化も存在。
  特に大きいchain/rejectの振れが大きく、31.8%短縮などを実用性能の保証として扱わないこと。
- 探索削減の正確性は検証済みだが、実環境の連続GNG入力に対する今回の変更の
  速度・全出力一致は未検証。前回の実入力差分率測定は速度検証の代替ではない。
- 既存のROS・rosbag・viewerプロセスは停止対象外。自分が起動した検証プロセスのみ後片付け。
- 自分のビルド・テスト・ベンチマークの終了と、一時比較ディレクトリ・ソース退避ファイルの削除を確認。
  作業中に他の操作によるviewer停止・frontend再起動・colcon起動を観測したが、介入なし。
