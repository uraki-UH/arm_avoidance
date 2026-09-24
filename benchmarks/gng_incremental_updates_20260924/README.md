# CPU GNGの全体走査削減

基準コミット: `8c16ef895a6e97ca618ca1017585e08fb873bc83`。本番採用は `pooled`。

## 採用内容

- 入力照合と学習の間で探索用座標・ラベル配列を再利用。通常フレームで全ノード同期を2回から1回へ削減。追加・移動時の座標反映を継続し、フレーム外のラベル変更は次の同期で反映。
- 孤立ノード候補を64ビット単位のビット列へ記録。既存の同期走査で初期候補を収集し、新規追加・切断・他ノード削除で候補を追加。ID昇順で確認し、元の削除時点と最少残存数を維持。`eta_decay_rate < 1` の場合は元の全ノード走査と浮動小数点乗算順を維持。
- エッジ両端のIDを実在エッジ用プールに保存。距離更新はプールを一度走査し、未使用スロットを除外。学習で更新済みの小さな座標配列を参照し、小ID側から大ID側への元の差分計算順を維持。
- 単独のCUGNG呼出しは従来の同期・保守経路。公開C API、YAML、フィルタ、学習回数、寿命条件、法線・曲率・ラベルの更新頻度は変更なし。内部C++クラスの配置変更に対して利用パッケージを再ビルド。

ノード寿命の全走査自体は残存。標準的GNGの積算誤差による分割・全誤差減衰は、このCPU実装の学習経路にはなし。設定済みat128の学習係数減衰率は1.0。

## 測定条件

Release＋既存LTO、CPU 0固定、交差点bagの同一先頭フレーム。`node.grid=0.5`、上限20,000ノード、毎フレーム4,000回学習。入力voxel幅0.1／0.5 m、300フレームと100フレームの2追加試行。第2試行の実行順を反転。表は共通区間50〜99フレームの平均についての3試行中央値。単位ms。

比較コピーだけ乱数列と時間刻みを固定。本番は通常の乱数・実時間。既存GNG・Viewer・bag再生の12プロセスを維持した状態での測定のため、負荷変動を含む。

| 入力voxel幅 | 保守区間・変更前 | 保守区間・採用版 | 区間短縮率 | 全体・変更前 | 全体・採用版 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0.1 m | 2.184 | 1.565 | 28.36% | 93.338 | 94.004 |
| 0.5 m | 1.869 | 1.284 | 31.29% | 33.315 | 32.807 |

保守区間は短いエッジの判定、ノード寿命、孤立削除・学習係数減衰、エッジ距離更新の合計。全体は0.5 mで1.53%短縮。0.1 mの全体は0.71%増であり、改善を確認できず。区間短縮率を全体の高速化率として扱わない。0.1 mでは入力照合が約65〜66 msを占有。

初回300フレームの先頭50除外平均も、保守区間2.162→1.505 ms／1.896→1.326 ms。全体93.374→93.488 ms／34.059→33.480 ms。

## 個別候補

以下は最初の比較群。採用版は別に比較元を再測定し、上表の `before_pool` と対にして評価。同じ基準ライブラリを使用し、最初の測定を上書きせず保存。

| 方式 | 全体 0.1 m | 全体 0.5 m | 保守 0.1 m | 保守 0.5 m |
| --- | ---: | ---: | ---: | ---: |
| before | 96.608 | 34.438 | 2.285 | 1.937 |
| sync | 99.490 | 34.640 | 2.382 | 1.980 |
| orphan | 94.837 | 34.059 | 2.094 | 1.785 |
| edges（移動ノードの追跡） | 95.157 | 34.049 | 2.249 | 1.903 |
| combined（上記3件） | 96.392 | 34.217 | 2.184 | 1.821 |

同期再利用の単独効果は小さく、全体短縮は未確認。採用版では距離更新にも同じ座標配列を利用。孤立候補管理には保守区間の短縮あり。移動ノードを記録して距離計算を減らす案は、区間の短縮が小さく、追跡不要のプール直接走査へ変更。移動ノード追跡のコードは本番に不採用。

## 検証結果

- 比較用bag処理7,630フレーム、基準再利用を含む5,450組で全照合項目一致。うち拡張機能450組。
- グラフ、座標、法線、曲率、クラスタ、入力ラベル、voxel・ノード対応、重点候補数、学習4,000回を照合。拡張機能では観測、支持・共分散統計、重み付き重点入力、学習イベント、map deltaも照合。
- 追加回帰テストは固定dense実装と3,696回の比較。追加・移動・削除、ID再利用、隣接順、uint8寿命巻戻り、減衰0.99／1.0、フレーム外の変更、64ビット境界・末尾の余りを確認。全5候補で成功。採用版のASan／UBSanも成功。
- 本番ReleaseのCTest21件、追加API2件、WASM native共有CPU実装テスト1件が成功。
- インストール済みの通常乱数・実時間版で拡張機能つき30フレーム成功。実入力の総実行数7,660フレーム。
- 実際の引数なし`cb`で30パッケージが58.4秒で成功。既存の符号比較等による`gng_wasm_core`のstderrあり、失敗なし。
- 公開22シンボル不変。ライブラリを原子的に差替え、既存GNGの旧inodeへのマッピング維持を確認。次回GNG起動から新実装が有効。
- 既存12プロセスのPID・コマンドが前後一致。検証プロセス全終了、今回の一時ビルド3ディレクトリを削除。比較ソース・結果・ライブラリは保存。

初期の比較コードでVec3fの非constメソッドとの不一致、回帰テスト生成範囲の誤りを修正後に全テスト成功。通常サンドボックスの `mountinfo path is not absolute` は実行環境側の問題で、承認済みの実行経路を使用。

[全条件監査](audit.json)、[時間](report.json)、[配布](install.json)、[終了確認](verification.json)。

## 再現手順

既存の保存先を上書きしないため、準備は新環境または空の実験保存先向け。`setup.py` は基準コミットから展開。最初のコマンドはホストのリポジトリルートで実行。

```bash
python3 benchmarks/gng_incremental_updates_20260924/setup.py
```

以下は既存 `gng_cpu_container` 内の `/ros2_ws/src` で実行。

```bash
python3 benchmarks/gng_incremental_updates_20260924/snapshot_runtime.py
for method in sync orphan edges combined; do
  python3 benchmarks/gng_incremental_updates_20260924/optimize.py artifacts/gng_incremental_updates_20260924/${method}_source --method "$method"
  python3 benchmarks/gng_incremental_updates_20260924/register_test.py artifacts/gng_incremental_updates_20260924/${method}_source
done
python3 benchmarks/gng_incremental_updates_20260924/prepare_pooled.py
python3 benchmarks/gng_incremental_updates_20260924/prepare.py --methods before sync orphan edges combined pooled
for method in before sync orphan edges combined pooled; do
  python3 benchmarks/gng_followup_efficiency_20260924/instrument_attention.py artifacts/gng_incremental_updates_20260924/${method}_deterministic
done
bash benchmarks/gng_incremental_updates_20260924/build.sh before combined sync orphan edges
bash benchmarks/gng_incremental_updates_20260924/run_suite.sh before sync orphan edges combined
bash benchmarks/gng_incremental_updates_20260924/build.sh pooled
bash benchmarks/gng_incremental_updates_20260924/verify_boundaries.sh
mkdir -p artifacts/gng_incremental_updates_20260924/before_pool
cp artifacts/gng_incremental_updates_20260924/before/{libgng_cpu.so,CMakeCache.txt,compile_commands.json} artifacts/gng_incremental_updates_20260924/before_pool/
bash benchmarks/gng_incremental_updates_20260924/run_suite.sh before_pool pooled
python3 benchmarks/gng_incremental_updates_20260924/audit.py --methods sync orphan edges combined pooled
python3 benchmarks/gng_incremental_updates_20260924/report.py
```

実際の本番反映・配布・後片付けで実行したコマンド。基準ソースとの一致を確認するため、適用済みソースへの再実行は拒否。

```bash
python3 benchmarks/gng_incremental_updates_20260924/adopt.py --method pooled
bash benchmarks/gng_incremental_updates_20260924/validate_production.sh
python3 benchmarks/gng_incremental_updates_20260924/finalize.py
```

追加メモリはノード候補の約2.5 KBと、エッジプール1スロットあたり8 bytesの端点ID配列。全ペアの配列は復活させていない。

単一bag・固定乱数列での比較。Emscriptenブラウザ・Jetson実機・別入力での速度は未検証。
