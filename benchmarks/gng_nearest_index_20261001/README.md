# GNG近傍索引の実学習比較

標準条件の3反復で、線形走査と索引の学習結果が全6組・各3段階で完全一致。計測区間全体の中央値はmaxで8.604→2.307秒、longで11.238→3.239秒。通常launchも全身衝突判定を有効にして2モデルとも完了。元入力のSHA不変、所有プロセスの終了、実行前後の既存プロセス一致を確認済み。

対象は実ライブラリの `GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>`。14関節の保存GNGを読み込み、`GngParameters.enable_nearest_index` の有無だけを切り替えた比較。同じseed・同じ固定入力列で実際の角度学習と左右TCP辺生成を実行する構成。

## 測定条件

| 項目 | 内容 |
| --- | --- |
| 主入力 | 自己干渉修正済みmax 19,139ノード、long 24,006ノード |
| 対応する追加入力 | 元max/longの各10,000ノード。今回の本測定には未使用 |
| 角度学習 | `gngTrain(samples, 2000)`、`std::srand(seed)`による同一選択列 |
| TCP辺生成 | `trainCoordEdgesOnTheFly(5000, layer)`を左右各層、固定列の再生サンプラと実URDF FK |
| 固定入力 | 各trial 4096姿勢、seed 1・2・3。線形と索引の入力ファイルSHA一致 |
| 実行資源 | 単一CPUへのaffinity、OpenMP・BLAS各1スレッド、各条件別プロセス |
| 反復 | 各モデル・探索方式3回。runnerによる条件順の巡回 |
| 衝突検査 | 比較ベンチでは無効。衝突検査付き通常launchは別の小規模smoke |
| 出力保護 | 元入力の変更なし、既存出力先の拒否、新名バイナリへの固定コピー |

標準条件は通常max/long YAMLと同じ値: 候補4、AiS距離10、辺寿命2000、beta 0.0005、lambda 200、学習率0.08/0.008、alpha 0.5。入力は既存 `collect_joint_limits` が返す関節区間内の一様乱数。有効な上下限のない関節は既存規則どおり±pi。通常trainer全体のTCP受理サンプリングや衝突判定を再現する測定とは区別。

追加・削除用の `churn_delete` は別条件。辺寿命0、lambda 25、AiS距離0.05へ変更し、保存済み64姿勢近傍の固定入力を使用。公開 `NODE_ADDED` 通知による追加集計と「初期数＋追加数−最終数」による削除集計。集計providerはノードを編集しないため、`can_modify_node_positions()=false` の契約。既存 `churn` 条件の辺寿命2と先行結果も保持。

## 計測区間と一致条件

`total_sec` は比較対象GNGのconstructor・設定・loadから、角度学習、左右TCP辺生成、各段階のFK更新と3回の保存まで。索引の構築・更新・再構築を含む区間。入力GNGの事前解析、URDF読込み、固定列の生成は `preparation_sec` に別記録。プロセス起動から終了までの時間はrunnerの `elapsed_sec` に記録。

角度学習後の `angle.gng`、左TCP辺生成後の `coord0.gng`、右TCP辺生成後の `final.gng` を照合。全ノードの生レコードと全層辺の生レコード多重集合が完全一致。関節角度・TCP・ノードID・誤差・状態、辺ID・age・activeを含む条件。辺の保存順序は別扱い。固定入力SHA、設定値、追加・削除・既存移動の集計値も一致。

## 3反復の結果

表の時間は各条件の3反復中央値、速度比は「線形時間の中央値÷索引時間の中央値」。同じtrialの速度比を先に求める集計とは別定義。

| モデル | 入力ノード | 線形走査の全体 | 索引の全体 | 中央値時間の比 |
| --- | ---: | ---: | ---: | ---: |
| max | 19,139 | 8.604秒 | 2.307秒 | 3.729倍 |
| long | 24,006 | 11.238秒 | 3.239秒 | 3.469倍 |

同じtrialごとの速度比の中央値はmax 3.934倍、long 3.462倍。角度学習だけではmax 5.109倍・long 1.914倍、左TCP辺生成は8.691倍・11.387倍、右TCP辺生成は25.325倍・25.252倍。この段落は全てtrialごとの速度比の中央値。

両モデルとも各trialで9ノード追加、削除0。最終max 19,148ノード、long 24,015ノード。既存ノードの角度変更数はmax 5900・5915・5903、long 5912・6040・6026。勝者探索だけの静的測定ではなく、隣接移動・追加を含む実学習の確認。

本測定12/12ケース成功、全6組の3段階すべてで完全一致。runner合計82.49秒。[比較結果](../../artifacts/gng_nearest_index_20261001/training_provider_fixed_comparison.json)、[実行記録](../../artifacts/gng_nearest_index_20261001/training_provider_fixed_batch/report.json)。この速度比は上記区間の測定値で、衝突判定付き通常学習全体の速度比を示すものではない。

## 追加・削除の予備確認

max、seed 1、角度500反復・左右TCP各1000反復の1回測定。両条件とも各段階の入力・全ノード・全層辺が完全一致。

| 条件 | 線形の全体 | 索引の全体 | 速度比 | 追加 / 削除 / 既存移動 |
| --- | ---: | ---: | ---: | ---: |
| 標準 | 2.534秒 | 1.529秒 | 1.657倍 | 2 / 0 / 2186 |
| churn_delete | 2.472秒 | 1.701秒 | 1.453倍 | 283 / 234 / 55 |

4/4ケース成功。[比較結果](../../artifacts/gng_nearest_index_20261001/pilot_provider_fixed_comparison.json)、[実行記録](../../artifacts/gng_nearest_index_20261001/pilot_provider_fixed_batch/report.json)。

## 通常launchの確認

索引ONを明示した通常 `offline_urdf_trainer_dual.launch.py`。max/long各32ノード上限、角度1000反復、refine 0、ROS domain 91・localhost限定。各600秒の上限と専用 `ROS_LOG_DIR`。32は上限で、保存された実ノード数は両方6。

| モデル | 経過時間 | 実ノード | 辺数:角度/左TCP/右TCP | VLUT参照 |
| --- | ---: | ---: | --- | ---: |
| max | 61.639秒 | 6 | 15 / 11 / 14 | 全6ノード、3256関係 |
| long | 80.443秒 | 6 | 15 / 11 / 12 | 全6ノード、3597関係 |

両方で `NearestSearch enable_nearest_index=true`、衝突検査有効、全リンクのmesh surface＋1 mm solid containment、TCP辺構築後の最終StrictFilter、GNG/VLUT保存、ROSプロセス正常終了をログ照合。保存ファイルは既存GNG v9/VLUT v2の独立デコーダで全ノード・辺参照を検査。入力2万ノード規模の全身再学習や保存グラフ全体の再構築は今回未実施。[検証結果](../../artifacts/gng_nearest_index_20261001/training_smoke_verification.json)、[runner記録](../../artifacts/gng_nearest_index_20261001/training_smoke_batch/report.json)。

## 再現入口

root管理のCMake targetは `gng_nearest_index_benchmark`。固定済みバイナリはcontainer内 `/tmp/gng_nearest_index_20261001/gng_nearest_index_benchmark_provider_fixed`、SHA-256 `b7cef1193c500510d36844a335226edce9415f8e1e46404a12b92810f3def7f2`。コンパイルはRelease `-O3 -DNDEBUG`。flags・link・入力・ソースSHAは [provenance](../../artifacts/gng_nearest_index_20261001/benchmark_provider_fixed_provenance.json)。

```bash
python3 benchmarks/gng_nearest_index_20261001/prepare_cases.py \
  --root /home/uraki/uraki_ws \
  --binary /tmp/gng_nearest_index_20261001/gng_nearest_index_benchmark_provider_fixed \
  --output /home/uraki/uraki_ws/artifacts/gng_nearest_index_20261001/reproduction_cases.json \
  --datasets max_final long_final --profiles standard \
  --num-angle-iter 2000 --num-coord-iter 5000 --num-samples 4096

docker exec gng_cpu_container python3 \
  /ros2_ws/src/artifacts/gng_nearest_index_20261001/benchmark_run_batch.py \
  /ros2_ws/src/artifacts/gng_nearest_index_20261001/reproduction_cases.json \
  --output /ros2_ws/src/artifacts/gng_nearest_index_20261001/reproduction_batch \
  --repeats 3 --timeout-sec 180 --max-total-sec 2400 --estimate-sec 10

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  benchmarks/gng_nearest_index_20261001/compare_results.py \
  --root /home/uraki/uraki_ws \
  --reports artifacts/gng_nearest_index_20261001/reproduction_batch/report.json \
  --output artifacts/gng_nearest_index_20261001/reproduction_comparison.json
```

`prepare_cases.py` は標準値と元YAMLの抽出値を照合し、入力GNG・URDF・設定のSHAをmanifest隣接の `.inputs.json` へ保存。通常launchは [専用manifest](../../artifacts/gng_nearest_index_20261001/training_smoke_logged_cases.json)、実際の起動コマンドと各終了記録は [benchmark_commands.jsonl](../../artifacts/gng_nearest_index_20261001/benchmark_commands.jsonl)。既存outputの再利用は禁止で、新しい出力先が必要。

## 先行試験の保持

初回pilotはベンチの入力準備で失敗。URDF loaderが設定しない `has_limits` を必須としていたため、既存 `collect_joint_limits` に統一。旧binary・入力・[失敗記録](../../artifacts/gng_nearest_index_20261001/pilot_batch/report.json)を保持。

次のpilotでは全保存結果が一致し、標準条件が1.493倍速くなった一方、83回追加の `churn` が0.735倍へ低下。status-only providerでも通知後に索引を全破棄する経路があったため、位置編集の有無を示すprovider契約を追加。未知providerは既定で全無効化を維持し、既存status-only providerと集計providerは位置編集なしを明示。その後、新名binaryと新manifestで上記測定を実施。[旧pilot比較](../../artifacts/gng_nearest_index_20261001/pilot_limits_fixed_comparison.json)は変更せず保持。旧churnと新churn_deleteは辺寿命も異なるため、両者の時間だけでprovider修正単体の効果を断定しない取扱い。

## 終了と保全

本担当で起動したcontainer 26 PID・host 6 PIDの消滅を確認。所有PIDがファイル名に含まれる一時URDF 23件だけを清掃。ベンチ・通常launchとも終了済み。コンテナ状態と既存host 38プロセス・container 17プロセスは、PID・起動時刻・PGID・argvを含め前後完全一致。既存プロセスへの停止操作なし。

元入力6件・補助ソース5件・最新固定binaryの計12 SHA不変。[SHA照合](../../artifacts/gng_nearest_index_20261001/benchmark_final_sha_check.json)、[所有終了・清掃](../../artifacts/gng_nearest_index_20261001/benchmark_owned_cleanup.json)、[前後状態差分](../../artifacts/gng_nearest_index_20261001/benchmark_runtime_diff.json)。各失敗記録・旧binary・全保存結果を保持。
