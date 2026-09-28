# 軽量サンプラーと汎用反復実行器の検証

## 1. 要約

2026-09-27確認。初回98＋保持比較49＝147試行・14,700入力。全試行正常終了・子プロセス回収を確認。
この比較時点では本番GNG・YAMLを変更せず、外部登録による試験のみ。その後の保持なし軽量方式の[本番反映](PRODUCTION.md)は別記。
既存の粗いセル統計を省き、既に計算済みの最近傍ノード所属を使う`np_points`が軽量候補。
評価点数ではなく総学習4,000回を固定。学習2,000回を非平面最近傍セルへ点数比例で配分。

条件選択に使っていない形状乱数4〜8の5試行平均。誤差は小さいほど良好：

| 方式 | API全体 ms/入力 | 物体→最近傍ノード平均 m | 同p95 m |
| --- | ---: | ---: | ---: |
| 追加重点なし | 41.074 | 0.102597 | 0.175083 |
| 現行粗いセル重点25% | 47.164 | 0.105598 | 0.175651 |
| 最近傍非平面セル・点数比例50% | 41.954 | 0.101679 | 0.167051 |
| セル点数変化・セル均等25% | 42.284 | 0.101627 | 0.169701 |

最近傍方式は全5種で平均誤差が減少。ただし平均改善0.9%＝0.92 mm、追加0.88 ms/入力。
地面95,000点を毎入力再生成する乱数4〜6でも、平均0.102199→0.101673 m、p95 0.174410→0.166736 m。
そのときの全体40.887→41.796 ms/入力。地面の平均誤差は0.133777→0.136676 mに悪化。
点数変化方式は再生成条件で平均0.103732 m、43.332 ms/入力へ悪化し、観測ゆらぎへの弱さを確認。
スコアの変更だけで大幅な追従改善・追加コストゼロを達成したとはいえない。本番採用は未実施。

同日追加の保持比較（乱数9〜13、5試行平均）。残留は通過済み領域のノード数/入力：

| 過去の保持フレーム | 平均誤差 mm | 残留ノード | API全体平均 ms/入力（試行平均の中央値） |
| --- | ---: | ---: | ---: |
| 0 | 102.026 | 13.729 | 43.333（43.372） |
| 2 | 101.992 | 13.503 | 43.957（43.991） |
| 4 | 102.134 | 13.249 | 45.891（44.056） |

2保持の平均改善は0.034 mmのみ。4保持は1試行53.659 msまで遅く、除外せず平均・中央値を併記。
地面再生成3種の誤差は保持0/2/4で102.763/102.890/102.967 mm、時間43.225/43.948/43.851 ms。
ラベルを5入力中2入力欠落させた3種では、誤差103.196/104.737/104.915 mmと保持で悪化。
欠落条件の重点学習回数は平均1,200→2,000回、保持由来の重み比は約40%。総学習は4,000回で不変。
減衰はセル間の相対重みであり、正の対象が残る限り重点枠2,000回自体は減らない制約。
通常時の保持由来比は2/4保持で1.13/1.68%。保持の明確な効果は未確認、本番への導入なし。

## 2. 条件・検証

- 入力は床95,000点＋円筒5,000点、30静止＋70移動、速度0.06 m/入力。入力voxel 0.1 m。
- ノード上限4,000、学習4,000回。非平面は前回ノードのz>0.08 mという既知形状の代用ラベル。
  実クラスタリング誤り、車・人の認識精度、実bag、Viewerは未検証。
- コピーしたCPUライブラリの乱数20260926・dt=0.1を固定。形状乱数を変えて同じ種同士で比較。
  Docker `gng_cpu_container`、CPU 4固定、既存ユーザーGNG・bag・Viewerは稼働したまま。
- 指標は移動70入力平均。API全体は入力設定・重点設定・GNG実行・結果取得を含む。
  Python側の点生成・所属ID列作成・品質評価、ROS配信は時間外。理論的なゼロ追加コストの測定ではない。
- 保持は固定座標の入力0.1 mセル、過去H入力、重み`max(0,1-age/(H+1))`、再検出時age=0。
  既存セル整列順と期限付き履歴の線形突合せ。未観測セルは期限内メタ情報のみ、点群保存・再学習なし。
  保持0は履歴処理を省略。初回設定で履歴を初期化する試作で、ROSのTF変更・巻戻しへの統合は未実装。
- 残留は円筒中心の通過線分から水平0.35 m以内、現在中心から0.35 mより外、0.1<z<1.9 mのノード数。
  円筒半径0.25 m＋入力voxel幅0.1 mの許容。現形状の支持精度や残留寿命を直接測る指標ではない。
- 保持比較は`batch_retention_20260927`25試行（保持前方式・OFFを含む）、`batch_retention_resample/gap_20260927`各12試行。
  種9〜13／9〜11、後者gapは非平面ID列を周期的に空にする人工条件。実分類器の欠落とは区別。
  `retention_summary_20260927.json`に全指標・平均・中央値・同一種との差を保存。
- 初期66試行では、点数／セル均等、最近傍距離残差、時間方向の点数変化、既存unknown配分を比較。
  配分12.5/25/50/75%と残差開始距離も比較。残差方式は平均改善を再現せず、unknown変更も平均誤差が悪化。
  既存`node.unknown_learning_rate: 0.3`は拒否され、OFF基準は既定の約80% unknown配分。
  比較値0.5/0.7/0.9は内部整数変換のため、そのまま50/70/90%になるとは限らない。
- `batch_screen/tune/change/unknown/holdout/resample_20260927`は順に18/24/12/12/20/12試行。
  生ログ・実行argv・seed・終了コード・cleanup・指標は各`report.json`、全指標集計は`sampling_summary_20260927.json`。
  すべて`artifacts/tracking_attention_20260926/`配下。良い試行だけの選別なし。
- 既存NumPy/SciPy推奨版不一致の警告を保持。runnerの孤児回収競合を初期検証で修正し再検証。
  ホスト・Dockerでrunner回帰10件成功（回数、順序、環境、失敗、中断、時間上限、孤児回収、上書き拒否、指標検査）。
  C++試作品の点数変化単体検証、スキル構造検証も成功。
- 保持0/2/4の減衰・更新・空セル・負座標・世代不一致・未観測期限切れを単体検証。
  ASan/UBSan成功。保持0と従来軽量方式の500入力分の全計時外指標が一致。
- 汎用[`run-benchmark-batch`](../../skills/run-benchmark-batch/SKILL.md)を追加し、ユーザースキル領域へsymlinkで登録。
  予測110秒→実測110.27秒（20試行）、66秒→67.99秒（再生成12試行）。予測は保証値ではない。
  長い処理はツールの完了待ちを利用。終了済み会話の外部再起動・トークン消費ゼロは提供しない。

再現は既存の`before/at128.yaml`と`det_final_after/libgng_cpu.so`が前提（[作成条件](README.md)）。
以下はコンテナ内`source /ros2_ws/install/setup.bash`、`cd /ros2_ws/src`の後。再試行は新しい出力名が必須。

```bash
c++ -std=c++17 -O3 -fPIC -shared -Iais_gng_cpu/src/gng_cpu/include benchmarks/tracking_attention_20260926/experiment_sampler.cpp -o artifacts/tracking_attention_20260926/retention_sampler.so
python3 benchmarks/tracking_attention_20260926/verify.py suite --tag final_ --sampler-library /ros2_ws/src/artifacts/tracking_attention_20260926/retention_sampler.so --cases off np_points:.5 np_hold:.5:.05:0 np_hold:.5:.05:2 np_hold:.5:.05:4 --output artifacts/tracking_attention_20260926/retention_cases_20260927.json
python3 skills/run-benchmark-batch/scripts/run_batch.py artifacts/tracking_attention_20260926/retention_cases_20260927.json --output artifacts/tracking_attention_20260926/batch_retention_20260927 --repeats 5 --start-seed 9 --timeout-sec 150 --max-total-sec 600 --estimate-sec 5.5
```

保持の再生成／欠落はsuiteへ`--resample-ground`／`--label-gap-frames 2 --label-gap-period 5`を追加、np_pointsを除外。
別manifest・出力へ3反復、種9開始、仮予測5.7秒/試行。実測は通常142.39秒、再生成70.33秒、欠落67.88秒。
初回98試行は保存済みmanifestが正本。screen/tune/change/unknownは種1開始・3反復、holdoutは種4開始・5反復、resampleは種4開始・3反復。
初回runner上限150秒/試行・1,200秒/全体、仮予測はscreenのみ12秒、他5.5秒。実行argvは各reportに保存。
全実行は`docker exec gng_cpu_container bash -lc '…'`内でrunnerを起動し、完了まで待機。
集計は`verify.py summarize --reports <各report.json> --output <新規.json>`。runner単体は`python3 skills/run-benchmark-batch/scripts/test_run_batch.py`。
C++単体は上記ビルドの`-O3 -fPIC -shared`を`-O2 -DEXPERIMENT_TEST`へ置換し`retention_sampler_test`として実行。
メモリ検査は`-O1 -g -DEXPERIMENT_TEST -fsanitize=address,undefined -fno-omit-frame-pointer`で`retention_sampler_sanitize_test`を作成・実行。

全benchmark・単体検証プロセスは終了。既存ROSプロセス・コンテナへの停止や再起動操作なし。
初回98試行の開始時GNG PID 398493は終了確認時に退出（理由未調査）。保持比較ではGNG 408534・bag 293745・Viewer 333336と3コンテナを維持。
