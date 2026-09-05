# 勝者入力による支持領域

CPU版GNGの第一勝者共分散を保持し、第一・第二勝者の入力点から別の支持楕円体を生成する実験実装。
設定は`config/gng_cpu/graspnet.yaml`の`node.support.*`に集約。
通常は無効。同ファイルの`node.enable_support`を`true`に変更して有効化。
直接起動時の一時的な有効化例:

```bash
ros2 run ais_gng ais_gng_cpu --ros-args \
  --params-file <ais_gngのconfig/gng_cpu/graspnet.yaml> \
  -p node.enable_support:=true
```

## 更新式

各ノードの統計はGNG内部の勝者確定時・ノード移動前に直接更新。経過時間、フレーム周期、
入力のない呼出しによる忘却なし。時刻はCSV・図の表示用途のみ。

- 第一勝者の更新率: `sample_alpha`。既定`0.01`。
- 第二勝者の更新率: `1 - (1 - sample_alpha)^second_weight`。既定重み`0.5`。
- `sample_alpha=0`は加重サンプル数による累積平均・母共分散。忘却なし。
- ノード削除・IDの生成フレーム変更時の統計破棄。

入力点`x`、旧平均`μ`、旧共分散`C`、実効更新率`a`について、

```text
δ = x - μ
μ_new = μ + a δ
C_new = (1 - a) (C + a δ δᵀ)
M = C_new + (μ_new - w) (μ_new - w)ᵀ
```

`w`は現在のGNGノード位置。入力座標の統計から現在のノード中心の二次モーメント`M`を導出。
ノード位置が変わった場合の楕円体再中心化は、観測統計の更新とは別。
最小軸標準偏差`0.002 m`で特異行列を正則化。

従来の`winner_point_covariance`は変更前のWelford方式を維持。
更新式と履歴は既存の`gng_cpu/src/utils/node.hpp`のNode内に保持。ノード初期化時に統計もリセット。
`node_support.hpp`は更新済み行列から描画用の軸・姿勢を計算する処理だけ。外側の履歴マップなし。
支持領域有効時は、`node.covariance_winner_rank_max`の値によらず従来フィールドへ第一勝者だけを集計。
支持領域無効時の既存rank設定の挙動は従来どおり。

## 楕円体の大きさ

半軸は`base_scale × sqrt(Mの固有値)`。既定倍率は固定2。
共分散行列の2倍ではなく、半軸長の2倍に相当。
入力分位点・エッジ中点による倍率調整、上限倍率、実行時の被覆評価は削除済み。
第二勝者が異なる面にある場合の混合や、第一勝者だけの場合より広がる性質があるため、
法線・曲率推定への転用は未検証。
現実装は形状推定へ接続せず、支持領域を別に配信。

## 可視化と記録

`node_support_ellipsoids`は`visualization_msgs/MarkerArray`。
RVizやViewerのMarker表示で確認可能。色は淡青。
既存の共分散表示と並べて比較可能。マーカーは直径を設定済みで、追加の2倍拡大は不要。
経過時間によるマーカー消去なし。次の地図配信で所属を更新。

通常のROSノードでは学習イベント記録を無効化。CSV出力・残差グラフ用統計・診断ディレクトリ設定なし。
計測と記録は`benchmark_node_support.cpp`、残差平均・RMSの集計は`plot_node_support.py`に限定。
支持領域の更新式とMarker配信は維持。

| CSV | 内容 |
| --- | --- |
| `diagnostic/events.csv` | 各入力イベントのrank、ノードID・世代、残差XYZ |
| `diagnostic/nodes.csv` | ノード・rank・支持行列・半軸・倍率 |
| `diagnostic/legacy_nodes.csv` | 従来の第一勝者共分散 |
| `diagnostic.csv` | ベンチマーク側の区間時間 |

グラフではノードID・世代・rankごとに入力順を維持して残差平均・RMSを再構成。
そのフレームの出力地図に属さないイベントは除外。削除ノードの履歴の混入なし。
過去に生成した残差統計付きCSVにも対応。
既存のイベントABIは維持。追加していた入力座標の配列・取得APIは削除。
ROSメッセージ出力と描画には`gng_get_node_statistics(node_id)`で更新済み統計を取得。
既存のTopologicalNodeとROSメッセージのレイアウト・残差共分散の定義は維持。

## 再現手順

`export_support_bag.py`で読み取り専用SQLite bagからXYZ列を抽出。
`benchmark_node_support.cpp`で同じ入力列・固定乱数列のGNG学習を比較。
固定乱数はビルドスクリプトが検証用コピーへ適用。本体ソースに検証用の分岐なし。
パラメータ設定後にgng_initを呼び、5,000学習/フレームと入力容量を反映。
以前の比較スクリプトは初期化後の設定だったため、NodeConfigへ反映されない項目あり。過去の時間との単純比較不可。
現行の通常計測はイベント記録なし。CSV出力指定時とcapture2モードのみ記録を有効化。
統計更新時間はgng_msに含み、legacy_msは結果取得、support_msは描画用変換の時間。

コンテナ内の再現コマンド。`run_support_benchmark.sh`でビルドと計測を一括実行:

```bash
bash /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/run_support_benchmark.sh \
  <input.binを配置した新しい出力ディレクトリ>
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/smoke_node_support.py \
  /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  <同じ出力ディレクトリ>/input.bin \
  <同じ出力ディレクトリ>/smoke
```

比較の前提は、`/tmp/node_support_baseline`への基準コミット`d19b3e32`の
`ais_gng_cpu/src/gng_cpu`の展開。現行ソースは`/tmp/node_support_current_source`へコピー。
両コピーの乱数だけを`42 + frame_number`へ置換し、Release・フレームログ無効でビルド。
比較用ビルドの既存install/buildへの反映なし。

ROS検証は既存ROS環境と独立した`ROS_DOMAIN_ID=187`。両モードで80入力ずつ確認。
起動コマンドの全引数と停止結果は`smoke/results.json`へ保存。
スクリプトの`finally`で自身のノードにSIGINTを送り、終了待ち。既存のbag再生は停止対象外。

ホスト側の図生成:

```bash
python3 ais_gng_cpu/src/ais_gng/test/plot_node_support.py artifacts/node_support_internal_20260905
```

NumPy・SciPy・Matplotlibを使用。`report.html`は外部参照のない自己完結ファイル。
PNG/PDF、集計JSON、CSVも同じ実験ディレクトリに保存。
全入力点の被覆監査は、包絡球の空間検索と3D楕円体判定によるオフラインの和集合評価。
体積図は楕円体体積の合計で、重複を除いた和集合体積ではない。

## 検証結果

統計更新をGNG内部へ移した前後の比較記録は、
`artifacts/node_support_internal_20260905/README.md`と同ディレクトリの`report.html`に集約。
同一2万点入力、5,000学習、固定乱数列での交互3回比較。通常のgraspnet設定とは入力・ノード数が異なる条件。

| ビルド | 内部更新への変更前 | 変更後 |
| --- | ---: | ---: |
| Release | 13.973 ms | 13.384 ms |
| Debug | 40.980 ms | 41.622 ms |

従来共分散61,382行と学習イベント2,400,000行は一致。
支持行列123,514行の最大絶対差は`1.47e-9`。機能テスト4組と、ROS配信各80地図の確認済み。
当時の検証ノードの起動コマンドと停止結果は`smoke/results.json`に保存。起動した検証ノードは停止済み。

被覆調整削除時の記録は`artifacts/node_support_fixed_20260905/`、
それ以前の被覆調整ありの実験は`artifacts/node_support_samples_20260905/`に保存。
旧実験は設定の初期化順序も異なるため、上表との時間の直接比較は不可。
