# 2026-09-29 - 平面接続候補に限定した曲面モデル探索

## 1. 要約

球・円柱・楕円柱・二次曲面の既存当てはめを維持し、平面核につながる候補だけを抽出する任意設定を追加。
全ノードのハッシュ表生成も16bitノードIDの直接参照表へ変更。

- 同じ元平面の所属点と、`max_link_length`内の実GNGエッジから到達成分を作成。
- 必要枚数の元平面を含む成分、または検証済み保持曲面を含む成分だけを既存のモデル探索へ入力。
- 候補選別は全入力を一度走査。曲率推定・境界判定・モデル試行の対象点数を削減。
- ノード・元平面の添字と保持曲面IDを元入力へ復元。保持中の平面枚数減少でも候補を保護。
- `surface_model.enable_plane_local_search`の既定値は`false`。`model`方式専用、起動時設定。

有効化は、使用するセンサーYAMLの`ais_gng_node.ros__parameters`へ以下を指定してlaunchを再起動。

```yaml
plane_clustering: true
curve_clustering: true
surface_model.enable_plane_local_search: true
```

ROSノードを直接起動する場合、`curve_clustering`の代わりに`surface_model.enable: true`を指定。
Dockerの通常build/installへ反映済み。追試後は`at128.yaml`だけ局所探索をONへ変更し、提示launchで有効化。共通既定値はOFF。

## 2. 条件・検証

固定入力の`tracker.update`だけを計測。時間単位はms、平均・p95は各3試行の中央値、最大は全3試行中の最大値。
基準版は`batch_local`、最終版は`batch_flat`の別バッチ。全48試行成功、同一入力・同一形状判定設定。

| 入力 | 基準版平均 | 最終版平均 | 最終版p95 | 最終版最大 | 候補ノード数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 交差点100入力、平均19,144点 | 24.333 | 0.950 | 0.982 | 1.102 | 0 |
| 円柱288点＋遠方単一平面19,000点、20入力 | 7.440 | 1.374 | 1.422 | 1.504 | 288 |
| 把持の実入力1,545点、21入力 | 7.546 | 7.556 | 8.689 | 8.855 | 1,524 |

交差点入力は指定bag `/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2` の`/lidar_points`由来の保存GNG150入力から、平面ウォームアップ50入力後の100入力を固定。
旧保存形式にない`boundary_evidence`は0。平面出力も一度生成して固定した比較であり、現在のライブ入力・60msの現象の完全再現ではない。
既存bag・TF・rosbridgeは稼働したまま、CPU固定なし。主担当のビルド・ROS試験と性能計測は時間を分離。
GNG生成、平面クラスタ生成、JSON・Marker・Graph生成、ROS配送、Viewer描画は上表の計測外。

交差点では基準版も曲面0件。空出力だけの高速化とならないよう、非空の大規模合成入力も比較し、円柱1本・288点を20入力すべてで保持。
ほぼ全点が候補になる把持入力では1〜3ms未達。候補内の探索回数・QR/SVD・全点残差評価は継続しており、任意入力に対する3ms上限保証なし。

- 基準版→局所候補版：141入力の表示曲面の所属・形状係数・追跡IDが完全一致。
- 局所候補版→ID直接参照版：141入力の全比較出力が完全一致（時間値は比較外）。
- 通常build・CTest成功、既存74件と追加7件の81件PASS。ID衝突・保持・元添字復元・不正入力等を確認。
- ROS試験：別domain 198、1,888点中288点の円柱を39フレーム配信。JSON・Marker・Graph所属、元添字、保持を確認。
- ROS試験はID直接参照追加前の局所候補版。最終版の差分は上記141入力の一致と通常ビルド・81件で検証。

| 設定・互換性 | 内容 |
| --- | --- |
| 必要平面数 | `surface_model.min_display_plane_patches`を候補選別にも使用。既定2、0で平面枚数による除外なし。保持曲面は別条件で保護 |
| 非平面だけの探索 | `min_plane_usage_ratio=0`に加え、必要に応じて`min_display_plane_patches=0`または局所候補設定OFF |
| 対象外領域 | JSONのpatch/model所属とMarkerから省略。Graphの`TopologicalMap.nodes`は全入力を保持 |
| JSON追加項目 | `has_candidate_filter`、`num_input_nodes`、`num_candidate_nodes`。元の配列添字を維持 |
| 互換性 | ROSメッセージ定義変更なし。C++のoptions/result拡張のためリンク利用側の再ビルドが必要 |
| 判定の制約 | モデル式・誤差閾値は維持。ただし不要候補の除外で最大試行回数の配分が変わるため、任意入力の完全同一結果は保証しない |

指定launchのライブ追試（既存bag継続再生、各40秒、先頭15秒除外、各45曲面更新）：

| 実行時設定 | Curve平均 | p95 | 最大 |
| --- | ---: | ---: | ---: |
| 局所探索OFF | 33.172 ms | 40.637 ms | 47.388 ms |
| 局所探索ON | 1.087 ms | 1.194 ms | 1.255 ms |

実行時パラメータでOFF/ONを確認。両条件で表示曲面0件、ONの候補0点。ライブ2条件は異なる点群フレームのため参考比較。
GNG欄は別処理で40〜60msの行も継続。初回probeはHumble非対応APIのimportで失敗し、修正後2/2成功。
`at128.yaml`を試験済みON設定へ更新し、install参照先との内容一致を確認。[ライブの条件・起動コマンド・終了確認](../../../artifacts/surface_local_20260929/live/README.md)。

小幅なgraph構築・曲率数値計算の試作は今回の本番反映を見送り。[判断](../reject.md)。
起動した試験・ビルド・比較プロセスはすべて終了済み。既存プロセスの停止なし。
外部で作業中に起動されたfrontendはそのまま維持。

根拠：[性能・入力来歴・再現手順](../../../benchmarks/surface_local_20260929/README.md)、[起動コマンド](../../../artifacts/surface_local_20260929/commands.md)、[集計](../../../artifacts/surface_local_20260929/perf/summary.csv)、[ROS試験](../../../artifacts/surface_local_20260929/ros_smoke/result.json)。
