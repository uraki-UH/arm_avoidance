# 2026-10-01 - 通常GNG学習の厳密近傍索引

変更:

- 対象: `offline_urdf_trainer` の関節角学習と左右TCP層の辺生成。対応次元は3 / 7 / 14、その他は全探索。
- 索引: 固定版SpatialTreeの `MovingBSPTree`、double座標、`NoHysteresis`、`approx_eps=0`。候補順位は従来のEigen float距離とID順で確定。同値・丸め境界は候補拡大から全件走査。
- 更新: 勝者と隣接ノードの移動、追加、削除、ID再利用に追従。`id != -1` の全ノードが対象で、`status.active` による候補除外なし。
- 再構築: load・setParams・座標再計算・mutable参照取得・各学習呼出し開始時の無効化。位置変更の可能性があるprovider callback後も、保持参照による別ノード編集を含め索引を無効化。
- provider契約: `can_modify_node_positions()` は既定true。関節角・座標・構成を変更しない既存5providerはfalseを明示し、追加時の増分同期を維持。
- 誤差減衰: 従来と同じ100反復ごとの全ノード更新。近傍探索ループから分離。
- 設定: `gng_params.enable_nearest_index: true` が既定。launch引数 `enable_nearest_index:=false` で全探索、未指定は設定ファイル値。
- 依存: [固定6ヘッダと来歴](../../third_party/spatialtree/README.md)。ホームディレクトリのcloneへのビルド時依存なし。

検証: 新規13件と既存衝突フィルタ3件、計16件成功。動的更新・削除再利用・同距離・丸め境界・外部参照・provider編集・左右TCP層・保存結果の全探索一致。[回帰集約](../../../artifacts/gng_nearest_index_20261001/final_test_summary.json)。予備比較で挿入頻度が高い条件の再構築コストを確認し、status専用providerの契約を追加。provider契約追加後の最終build・16件の回帰も成功。実モデルの3反復比較も全試行で一致。通常launchのmax/longも正常終了。


実学習比較: max 19,139 / long 24,006ノード、角度2,000反復＋左右TCP各5,000反復、各条件3回。下表は計測区間の中央値とその比。索引構築・増分更新・FK更新・3回の保存を含み、URDF読込み・入力作成・衝突検査は対象外。

| モデル | 全探索 | 索引あり | 中央値時間比 |
| --- | ---: | ---: | ---: |
| max | 8.604秒 | 2.307秒 | 3.73倍 |
| long | 11.238秒 | 3.239秒 | 3.47倍 |

- 品質: 全6対・各3段階でノードの生レコードと全層辺の生レコード多重集合が一致。入力・学習パラメータ・追加削除移動件数も一致。
- 追加削除の確認: 別の負荷条件で283追加・234削除が発生。全探索2.472→索引1.701秒、各段階の保存結果も一致。標準条件の速度とは別集計。
- 制限: 同時稼働する既存処理があるローカル計測。上表を、衝突検査・VLUT構築を含む通常学習全体の速度比へ適用する根拠なし。

通常起動: max / longの各32ノード上限・角度1,000反復・refine 0、実出力はいずれも6ノード。起動から終了まで61.64 / 80.44秒。索引ON、衝突検査ON、全身表面交差＋1 mm完全内包判定、Step5後の全層StrictFilter、GNG/VLUT保存を確認。保存後の独立decodeで辺数[15,11,14] / [15,11,12]、VLUT参照3,256 / 3,597と全ノード対応を確認。大規模な通常学習全体の時間比較ではなく、起動経路の小規模確認。[通常起動の検証記録](../../../artifacts/gng_nearest_index_20261001/training_smoke_verification.json)。

先行失敗: provider回帰fixtureでload後の挿入枠が不足。疎IDの入力へ修正し、2挿入と接続先のassertを維持した最終回帰で成功。

適用範囲: 通常学習の自己干渉検査と保存形式は従来どおり。前回の保存グラフ再構築スクリプトは別処理。
終了・保全: 起動したbuild・回帰・ベンチ・通常launchは全終了。所有PIDに対応する一時URDFのみ削除。既存host 38 / container 17プロセスとコンテナ状態が前後一致。元入力と測定binary・補助ソース12件、最終build時の本体・依存24件のSHAも一致。[起動コマンド・終了記録](../../../artifacts/gng_nearest_index_20261001/benchmark_commands.jsonl)、[build・回帰の起動記録](../../../artifacts/gng_nearest_index_20261001/build_provider_batch/report.json)、[前後状態](../../../artifacts/gng_nearest_index_20261001/benchmark_runtime_diff.json)。

詳細: [条件・測定値・再現コマンド](../../../benchmarks/gng_nearest_index_20261001/README.md)。
