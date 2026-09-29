# 正解付き入力点群によるGNG・平面検出の評価

## 評価経路

`generate.py`の解析形状からCSV点群と`manifest.json`を生成。
正解は元点の`gt_label`として保持し、GNGの意味ラベル・法線・学習設定への入力なし。
`learn.cpp`は本体の`CUGNG::learn_normal`を各入力点に1回呼び、既存の学習イベントから勝者を取得。
CSV内の順序は生成器でシャッフル済み。重点・ボクセルサンプリング、物体クラスタリング、平面からの学習フィードバックは今回対象外。
法線・幾何ラベルは本体`Labelling::labelling_fuzzy`を使用。時間依存を除くため、その既存dtクランプを0.5秒へ固定。
最大1,000ノード、その他は`Param`既定値。実際の設定は各試行の`learned/gng_metadata.json`へ保存。

学習直後の同一グラフを`planes_before`と`planes_after`へ入力。両者はGTファイルを読まない構成。
変更前は`artifacts/plane_merge_simplification_20260929/before.cpp`と`before_include`、変更後は現行の平面実装。
比較対象は残差悪化判定の撤去と接続長共通化。GNG自体の変更比較ではない。

## 正解・データ形式

- CSV：`frame_idx,point_idx,x,y,z,gt_label`。位置単位m、フレーム番号は0始まり連続。
- GT：正の整数は平面インスタンス、-1は非平面、0は評価除外用の予約値。
- 同一床の観測欠落パッチは同一ID。段差上面・壁面は別ID。GTは生成時の物理面に固定。
- 曲面シーンの円柱側面は解析形状上の非平面として-1を付与。ユーザー方針は局所平面パッチを許容するため、このラベルは「平面クラスタ所属禁止」の正解ではない。
- 交線の曖昧さを避けた観測範囲、ノイズ量、遮蔽期間をmanifestへ記録。
- `graphs.bin`は同一ビルド環境専用の一時的な構造体列。長期・他環境での正本は版付きCSVとmanifest。

基本3シーンは観測欠落床、0.10 mの段差＋壁、床＋円柱。追加`noisy_floor`は同一床でノイズ標準偏差を0.003／0.015／0.030 mへ変化。
各シーン40フレーム×2,000点、乱数種1～3。中盤のみ部分遮蔽。移動物体・センサー運動・実LiDARの光線モデルは未実装。

## 投票・評価

投票は`votes.csv`へ元点ID・GT・勝者ID・生成フレームを保存。各点1回、フレームをまたぐ票の累積なし。
フレーム末に残ったノードと`(id,生成フレーム)`で照合。削除済み勝者・未確定平面は未所属扱い。
最多ラベルへの丸めなし。`node_gt_votes.csv`に混合した票数・比率を保存。投票なしノードは正解不明として別集計。

- pair precision：同一の予測平面に入った点対のうち、同じ正解平面に属する割合。非平面点は正解対として非計上。
- pair recall：同じ正解平面の点対のうち、同一の予測平面に入った割合。未所属による取りこぼしも低下要因。
- macro best IoU：正解面ごとに最大IoUの予測面を求め、面ごとに等重み平均。1対1対応の指標ではないためpair指標と併記。
- planar miss：正解平面点のうち、勝者が消失／平面未所属の割合。
- nonplanar absorption：解析形状上の非平面点のうち勝者が平面所属の割合。円柱では局所平面パッチ所属率として解釈し、誤検出率としての使用禁止。
- raw coverage：元点からフレーム末の最近傍GNGノードまで0.20 m以内の割合。投票とは別の評価専用探索、学習への非入力。
- no-vote／mixed-vote：投票なしノード率と、複数GTから票を受けたノード率。

分母0の指標は未定義。フレームCSVでは空欄、平均では除外。precision単独での良否判定は禁止。
現在のpair precisionは負のGTを正解対に含めない厳格な平面専用指標。曲面シーンの局所平面パッチ許容を判定する合否指標としては非適用。
曲面への受け渡し品質には、床との混合、局所近似誤差、元ノード・境界の保持、同じ曲面への再統合を別途検証する必要あり。今回の実行範囲は平面検出まで。
全40入力の`all_*`と最初10入力を除いた`steady_*`を保存。後者も遮蔽・ノイズ変化を含み、完全な定常状態を意味しない。
GNG時間は学習・法線／ラベル・後処理・イベント取得を含むwall ms。平面時間はthread CPU ms。CSV I/O・GT採点は両者から除外。
正解は入力点に厳密だが、ノードへの対応は学習時の支持関係。学習後座標の幾何的所属を直接保証する正解ではない。

## 実行

既存`gng_cpu_container`内の`/ros2_ws/src`で実行。`before`ソース・ヘッダーの保存済みsnapshotが必要。
runnerは`/home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py`を`/tmp/plane_ground_truth_run_batch.py`へコピー。

```bash
timeout 360 bash benchmarks/plane_ground_truth_20260929/build.sh
PYTHONDONTWRITEBYTECODE=1 timeout 30 python3 -m unittest discover -s benchmarks/plane_ground_truth_20260929 -p 'test_*.py' -v
python3 /tmp/plane_ground_truth_run_batch.py benchmarks/plane_ground_truth_20260929/cases.json --output artifacts/plane_ground_truth_20260929/batch --repeats 3 --timeout-sec 180 --max-total-sec 600 --estimate-sec 10
python3 /tmp/plane_ground_truth_run_batch.py benchmarks/plane_ground_truth_20260929/noise_cases.json --output artifacts/plane_ground_truth_20260929/noise_batch --repeats 3 --timeout-sec 180 --max-total-sec 600 --estimate-sec 10
```

出力先は再利用不可。再試行では新しいパスを指定。反復回数は`--repeats`、入力フレーム・点数は`run_case.py --frames/--points`で指定可能。
乱数種1の床条件では、capture OFFとGTのみ変更した学習結果のグラフバイト一致も検査。
各試行に`dataset`、`learned`、`before`、`after`、`metrics.json`を保存。予測残時間・終了・後片付けはrunnerのreportへ記録。
