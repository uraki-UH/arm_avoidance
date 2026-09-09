# 2026-09-09 - Curved Surface Cluster Graph

## Summary

曲面モデルを既存のTopologicalMap形式でも出力可能。
表示対象は曲面メッシュではなく、モデルごとに色分けした元GNGノードと実エッジ。
Viewerの既存バイナリ転送・GraphRenderer・ノード/エッジ一括描画を再利用。
現在の既定表示はMarkerのまま。`surface_model.enable_graph=true` 時のみ追加生成。
Marker最適化後の仕様・結果は `2026-09-09_curved_surface_marker_reuse.md` を参照。

## Changed

- 任意のグラフ出力トピックは `/curved_surface_clusters`。`/topological_map` 自体は変更なし。
- モデルごとの所属を `clusters[].nodes` に保持。所属はnode.id、edgesは元のノード配列添字。
- node.id、位置、法線、共分散、通常分類・semantic・境界属性は保持。入力点群所属一覧 `inpcl_ids` のみ可視化用複製から省略。
- 元から存在する同一領域内のエッジだけを保持。上位モデル間を架空のエッジで接続しない。
- クラスタ色はnode/edgeで共通、未確定ノードは灰色。既存の属性ラベルを書き換えない。
- `/models` の形状種別、曲率、残差などの詳細JSONと抽出アルゴリズムは維持。

## Added

- `surface_model.enable_graph` (既定false)、`surface_model.enable_markers` (既定true)。通常はMarkerだけを生成。
- C++のID・属性・エッジ保持テスト、Viewerの所属色・ID参照・通常色復帰テスト。

## Fixed

従来のMarker経路では、受信ごとの材質変更に伴うgeometryの破棄と、球数の変化によるInstancedMesh再生成が発生。
任意のグラフ表示では既存GraphRendererのバッファ再利用を使用。
下記の比較結果はMarker最適化前の記録。現在はMarker側もバッファ再利用へ変更済み。

## Removed

標準起動の `/surface_models` と `/surface_models/markers`。同じ情報を複数の可視化経路へ既定で重複配信しない。

## Behavior Impact

- グラフ出力有効時のみViewerで `/curved_surface_clusters` を追加。通常のGNGレイヤーとして扱う。
- モデルに適合した平面領域も含む上位統合結果。曲面だけの抽出フィルタではない。
- unknownはグラフ上ではクラスタ未所属、詳細は `/models` に保持。
- ノードやクラスタの時系列追跡は追加しない。分割変化で色が変わり得る。
- 既定名および名前空間付きの `*/curved_surface_clusters` で自動色分け。別名ではGraphRendererの `enable_cluster_colors` 指定が必要。
- 新しい出力形式はノード再起動後に適用。フロントエンドは再読み込みで反映。
- エッジは既存GNGの円柱表示。転送・更新負荷の改善とGPU描画時間の改善は別問題。

## Topics / Params / Messages

| トピック | 型・用途 |
| --- | --- |
| `/curved_surface_clusters` | ais_gng_msgs/TopologicalMap、明示的有効時のみ |
| `/curved_surface_clusters/models` | std_msgs/String、形状パラメータJSON |
| `/curved_surface_clusters/markers` | visualization_msgs/MarkerArray、既定の表示 |

通常起動は変更なし。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml
```

別起動用は `ros2 launch ais_gng surface_models.launch.py`。同じ処理を通常起動と重複起動しないこと。
別起動の `enable_graph:=true enable_markers:=false` でグラフ表示へ切替。通常起動の設定は `config/surface_model.yaml`。

## Verification

以下はグラフ有効・Marker無効として比較した際の記録。現在の既定publisher数は `[0,1,1]`。
- ais_gng Releaseビルド、C++テスト14件に成功。
- Viewerバックエンドのcolcon build、frontend lint/build、Nodeテスト2件に成功。
- `ROS_DOMAIN_ID=198` の通常launchで `[graph, models, markers]` publisher数 `[1,1,0]` を確認。
- `check_surface_models.py --template /datasets/mug_gng_template.json.gz --seconds 6 --launch --graph --no-markers` 相当で、10フレームのグラフとモデル所属一致、Marker無効を確認。
- `--markers` の比較出力をChromeの実GraphRenderer/MarkerArrayRendererで描画。Playwright未導入のためCDP使用。
- 1100x800 / 390x844の非空キャンバス・色分け・更新・カメラ移動を確認。contextLost=false。
- 検証launch・ノード、ブラウザ、Viteは全て停止済み。既存ROSプロセスの停止操作なし。

同一mug 502ノード、946エッジ、2モデルの比較:

| 項目 | Marker | TopologicalMap |
| --- | --- | --- |
| 描画呼び出し数 | 5 | 2 |
| 同じ内容を20回再受信した際のGPU buffer生成/破棄 | 380 / 380 | 0 / 0 |
| ROS CDRサイズ | 58,669 bytes | 53,158 bytes |
| WSペイロード | JSON換算 約155,685 bytes | 既存バイナリ形式 47,111 bytes |
| 描画のみの中央値 / p95 (desktop) | 0.5 / 0.8 ms | 1.1 / 1.7 ms |

WSサイズは既存converter相当のJSON生成および固定長プロトコルから算出。通信全体の実測値ではない。
描画時間はソフトウェアWebGLで明示render+finishを計測。React更新・転送・JSON parseは含まない。
GPU描画単体は円柱エッジ化により増加しており、全体FPSの高速化をこの値からは主張しない。
実行中GNGを修正前に6秒観測した際は、約3,000球・5,500〜5,800エッジ、約2 Hz、抽出本体約2〜5 ms。
試作の全ノード再表示が負荷源の一つであり、曲面抽出時間とは分けて扱う。

再起動後の実GNG 3,000ノードでも、6フレームのグラフ・モデル所属一致を確認。
別時刻の入力で、抽出本体は平均18.13 ms、p95 19.02 ms。修正前と同一入力ではなく、抽出速度の比較には使用しない。
このスナップショットは15確定モデル、Marker 32 draw calls、新グラフ5 draw calls。
旧Markerの20更新比較はCDP待ち時間上限に達したため、大規模入力の新旧速度比は未確定。
通常Viewerの起動・ROS接続を伴わない専用ページでも新グラフ単体を再確認。
1100x800 / 390x844の双方で非空表示、3更新のGPU buffer生成/破棄0、contextLost=false。
描画単体中央値はdesktop 1.8 ms、mobile 0.9 ms。少数サンプルかつソフトウェアWebGLであり実機FPSの保証ではない。

## Risk / Notes

全量のグラフを2 Hzで転送する方式。差分配信への変更ではない。
旧Markerと新グラフを両方有効にすると重複描画になる。通常は新グラフだけを選択。
通常の `/topological_map` と同時表示した場合も元ノードが重なる。比較が不要なら元グラフのノード・edge表示を無効化。
描画性能は環境・モデル数・同時表示レイヤーによって変わる。長時間のGPU安定性は未検証。
