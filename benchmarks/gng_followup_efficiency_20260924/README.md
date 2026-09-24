# CPU GNGの追加4候補の検証

基準コミットは `28ce6c77b0de7801f296c1b14ff183d1cd49fa41`（法線高速化後）。探索用連続配列、重点候補コピー削減、クラスタ管理、疎エッジ保存を独立実装し、合成版も比較。本番採用は `combined_xyz`（探索・クラスタ管理・疎エッジの3件）。

## 条件と結果

Release＋既存LTO、CPU 0固定、交差点bagの先頭300フレームと100フレームの2追加試行。実行順を第2試行で反転。`node.grid=0.5`、ノード上限20,000、毎フレーム4,000回学習。比較コピーだけ乱数とdtを固定。本番の乱数・実時間は維持。

以下は各試行の共通区間50〜99フレームの平均についての3試行中央値。単位ms。個別候補の全体時間には変更対象外の処理の変動も含むため、全差分を純粋な最適化効果とは扱わない。

| 方式 | 全体 0.1 m | 全体 0.5 m | 判断 |
| --- | ---: | ---: | --- |
| 基準 before | 101.209 | 36.050 | 比較元 |
| search | 93.084 | 35.051 | 探索に必要な座標・ラベルの連続配置を採用 |
| attention_ids | 102.433 | 35.723 | 元点番号参照は見送り |
| attention_spans | 103.408 | 35.848 | 区間参照は見送り |
| cluster | 103.863 | 35.758 | コピー・ID管理の削減を採用。単独効果は小さい |
| edges | 101.171 | 33.867 | 疎保存を採用。メモリ削減が大きい |
| combined（4件） | 91.440 | 32.607 | コピー削減を含む比較版 |
| **combined_xyz（3件）** | **91.937** | **32.565** | **本番採用** |

採用版は基準比9.16%／9.67%短縮。初回300フレームの先頭50除外平均も、100.651→91.961 ms／35.707→32.855 ms。

- 探索単独の照合時間：72.012→63.304 ms／11.878→10.732 ms。
- クラスタ管理単独のクラスタ時間：6.028→6.067 ms／6.458→6.258 ms。0.1 mでは明確な短縮なし。
- 疎エッジ単独の保守時間：2.902→2.215 ms／2.331→1.873 ms。クラスタの距離参照にも影響。
- 重点候補の元番号参照は候補生成0.510→0.344 ms／0.305→0.183 ms、一方で学習5.295→5.738 ms／4.442→4.699 ms。合成版でも生成＋学習の時間増加が残り、明確な全体改善を得られず見送り。区間参照も独立に検証済み。

## メモリ

20,000ノード時のエッジ関連配列は論理2,000,000,000 bytesから予約容量1,800,010 bytesへ縮小。別に探索用配列400 KBと入力順の再利用配列を追加。

測定用Pythonプロセスの初期化直後RSSは約1,984.6→78.5 MiB。GNGだけのRSSではない。300フレームを事前に読み込む計測器の最大RSSは約4.37→2.51 GiBであり、初期化直後RSSと混同しない。

## 出力と回帰検証

- 本比較8方式×1,090フレーム＝8,720フレーム。基準再利用の7,630組で全照合項目一致。
- 事前probeは5方式×200＝1,000フレーム、800組一致。合計9,720フレーム・8,430組一致。
- インストール後の通常乱数・実時間版は実入力30フレーム成功。bag実行総数9,750。
- 全比較フレームの学習回数4,000を確認。グラフ、座標、法線、曲率、クラスタ、入力ラベル、voxel・ノード対応、重点候補数を照合。
- 拡張機能630組では観測、支持・共分散統計、重み付き重点入力、学習イベント、map deltaの内容・メタデータも一致。
- 探索回帰は追加・移動・削除・欠番・ID再利用・再初期化を確認。
- クラスタ回帰は同票の最小ID、重複旧IDの先頭一致、ROSID穴埋め、HUMAN/CAR引継ぎ等を変更前後で確認。
- 重点候補回帰はbefore／ids／spans各56ケース。65536点切替、64点ブロック境界、voxel・観測テーブル有無、範囲除外、実学習イベントを確認。
- 疎エッジは従来dense実装と24,096操作を照合。接続順、次数上限、寿命境界、重複隣接、削除とID再利用、delta順を含む。ASan／UBSanも成功。
- 本番CTest20件、追加API2件、共有CPU実装のWASM nativeテスト1件が成功。
- 引数なしの実際の`cb`で30パッケージが56.1秒で成功。`gng_wasm_core`の符号比較等の警告出力あり、失敗なし。

`audit.py`は全方式・全条件の存在、フレーム数、比較ハッシュ、Release＋LTOを必須検査。`summarize.py`の部分集計だけを完了根拠としない。[audit](audit.json)、[時間・メモリ](report.json)、[全比較](summary.json)、[配布](install.json)、[環境確認](verification.json)。

## 実装と互換性

探索はバッチ開始時に座標・ラベルを同期し、ノード追加・移動を即時反映。単発探索は元Node参照を維持。クラスタは作業配列再利用、move／swap、旧ID索引、重複検索削減。エッジは隣接順と共有pool IDを対応させ、寿命・距離だけ実在辺へ保存。

公開C API、YAML、入力フィルタ、学習回数、寿命・クラスタの判定は変更なし。内部C++クラス配置と`getEdgeIndex()`の値の意味は変更。直接利用する`gng_wasm_core`は再ビルド・試験済み。既存のuint8寿命巻戻りの挙動も比較対象として維持。

## 再現手順と起動コマンド

以下は既存`gng_cpu_container`内、作業ディレクトリ`/ros2_ws/src`で実行。保存先の既存コピーを上書きしないため、ソース準備は新環境／空の実験保存先向け。

```bash
python3 benchmarks/gng_followup_efficiency_20260924/prepare_sources.py --methods before attention_ids attention_spans search cluster edges combined combined_xyz
python3 benchmarks/gng_followup_efficiency_20260924/optimize_attention.py artifacts/gng_followup_efficiency_20260924/combined_source --variant ids
for method in combined combined_xyz; do
  for step in search cluster edges; do
    python3 benchmarks/gng_followup_efficiency_20260924/optimize_$step.py artifacts/gng_followup_efficiency_20260924/${method}_source
  done
done
python3 benchmarks/gng_followup_efficiency_20260924/register_attention_test.py artifacts/gng_followup_efficiency_20260924/combined_source --variant ids
python3 benchmarks/gng_followup_efficiency_20260924/register_attention_test.py artifacts/gng_followup_efficiency_20260924/combined_xyz_source --variant before
python3 benchmarks/gng_followup_efficiency_20260924/prepare.py --methods before attention_ids attention_spans search cluster edges combined combined_xyz
for method in before attention_ids attention_spans search cluster edges combined combined_xyz; do
  python3 benchmarks/gng_followup_efficiency_20260924/instrument_attention.py artifacts/gng_followup_efficiency_20260924/${method}_deterministic
done
bash benchmarks/gng_followup_efficiency_20260924/build.sh before attention_ids attention_spans search cluster edges
bash benchmarks/gng_followup_efficiency_20260924/run_probe.sh before attention_ids attention_spans search cluster
bash benchmarks/gng_followup_efficiency_20260924/run_suite.sh before attention_ids attention_spans search cluster edges
bash benchmarks/gng_followup_efficiency_20260924/build.sh combined combined_xyz
bash benchmarks/gng_followup_efficiency_20260924/run_suite.sh combined combined_xyz
python3 benchmarks/gng_followup_efficiency_20260924/audit.py --methods attention_ids attention_spans search cluster edges combined combined_xyz
```

実際の初回probeではattention段階の追加タイマー未導入。その後全方式を再ビルドし、本比較は同一計測器で実行。

本番反映では次を実行。基準ソース・全照合の一致を前提にし、適用済みソースへの再実行は拒否。

```bash
python3 benchmarks/gng_followup_efficiency_20260924/adopt.py --method combined_xyz
bash benchmarks/gng_followup_efficiency_20260924/validate_production.sh
python3 benchmarks/gng_followup_efficiency_20260924/finalize.py
```

追加単体検証では`verify_edges.sh`、各`search_cache_test`／`cluster_bookkeeping_test`／`attention_lookup_test`の有限タイムアウト付きビルドと実行を使用。主な個別コマンド：

```bash
timeout 1200 bash /ros2_ws/src/benchmarks/gng_followup_efficiency_20260924/verify_edges.sh
timeout 600 cmake --build /tmp/gng_followup_search_build --target search_cache_test -j 3
timeout 60 /tmp/gng_followup_search_build/search_cache_test
ctest --test-dir /tmp/gng_followup_cluster_build -R cluster_bookkeeping_test --no-tests=error --output-on-failure
ctest --test-dir /tmp/gng_followup_cluster_reference_build -R cluster_bookkeeping_test --no-tests=error --output-on-failure
cmake --build /tmp/gng_followup_attention_test_build/before --target attention_lookup_test -j 2
```

## 保存と限界

生ログ・ライブラリ・比較ソースは`artifacts/gng_followup_efficiency_20260924/`。通常colcon探索から除外済み。作成した検証プロセスは終了し、所有する一時ビルドだけを片付け、元データと結果は保存。

初回読取ではGNGを含む12プロセスが存在したが、保存した検証前ベースライン時点ではViewer＋bagの9プロセス。終了時も同じPID・コマンドの9プロセスで一致。GNG停止操作は実施しておらず、消失時刻・原因は未特定。新ライブラリは次回GNG起動から有効。

準備時のGit所有者検査エラーは、当該コマンド限定の`-c safe.directory`で解消。終了監査の初回は想定12件と保存記録9件の不一致で停止し、前後の実記録を確認して修正。サンドボックスの`mountinfo path is not absolute`は実行環境側の別問題。

単一bag・同一乱数列と時間刻みの比較。別環境の速度を保証する結果ではない。EmscriptenブラウザとJetson実機は未検証。
