# 2026-09-09 - Curved Surface Marker Reuse

## Summary

曲面クラスタの可視化はMarkerArrayを既定として継続。
表示名は `/curved_surface_clusters/markers`、ノードは球、実エッジは線分のまま。
色分けと表示形状を保ち、ViewerでのGPUリソース再生成を抑制。

## Changed

- `surface_model.enable_markers=true`、`surface_model.enable_graph=false` が既定。
- 追加TopologicalMapのpublisher作成・メッセージ生成・配信を既定では省略。
- 球リストのInstancedMesh容量を維持。ノード数増加で容量不足となった場合のみ2の累乗へ拡張。
- 線分リスト・線列も位置バッファを再利用し、drawRangeで実際の要素数を指定。
- 材質を使い回し、色・透明度だけ更新。geometryとmaterialを別々の寿命で解放。
- 実線に不要なcomputeLineDistancesを省略。

## Added

球の直径・位置・色、線分色・要素数、空配列、再増加、GPUオブジェクト同一性、最終解放の回帰テスト。

## Fixed

材質変更に巻き込まれた生存中geometryの破棄、ノード数の揺れによる毎回のInstancedMesh再生成。

## Removed

通常表示でのTopologicalMap重複配信。比較用の実装は `enable_graph=true` の任意機能として維持。

## Behavior Impact

Viewerでは旧 `/surface_models/markers` のレイヤーを外し、`/curved_surface_clusters/markers` を選択。
ROSノードは次回起動から新しい既定値。Viewerはブラウザ再読み込みで最適化を反映。
抽出ロジック、モデル所属、Markerの名前空間/id、球・線分サイズ、配色、更新周波数2 Hzは変更なし。

## Topics / Params / Messages

| トピック | 既定状態 |
| --- | --- |
| `/curved_surface_clusters/markers` | visualization_msgs/MarkerArray、有効 |
| `/curved_surface_clusters/models` | std_msgs/String、形状モデル詳細JSON、有効 |
| `/curved_surface_clusters` | ais_gng_msgs/TopologicalMap、無効 |

通常起動は従来どおり。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml
```

別起動: `ros2 launch ais_gng surface_models.launch.py`。通常起動との重複実行は不要。

## Verification

- ais_gng Release build、C++14件、frontend回帰テスト2件、lint/buildに成功。
- 隔離ROS_DOMAIN_ID=198の通常launchで `[graph, models, markers] = [0,1,1]`。
- `check_surface_models.py --template /datasets/mug_gng_template.json.gz --seconds 6 --launch` で10フレーム取得、配色と所属、グラフ無効を確認。
- `node /tmp/render_surface_model.cjs /tmp/curved_graph_compare.json markers-only` で実MarkerArrayRendererを検証。独立Chrome/CDP、ソフトウェアWebGL、ROS接続なしの専用canvas。
- 1100x800 / 390x844で非空表示、色分け、20更新、カメラ移動の画素変化、contextLost=falseを確認。
- 検証launch、ノード、Chrome、Viteは全て停止済み。既存ROSプロセスは停止操作なし。

同一mug 502ノード・946エッジの20更新で、GPU buffer生成/破棄は旧Markerの380/380から0/0へ。
描画呼び出し5回、三角形84,336枚、色付き画素desktop 30,561 / mobile 14,516は維持。
描画単体の中央値はdesktop 0.7 ms、mobile 0.8 ms。旧測定値との差からFPS改善を主張しない。
この変更の効果は主に更新時のGPUリソース確保・解放の削減。

実GNGから取得済みの3,000ノードでも、`node /tmp/render_surface_model.cjs /tmp/curved_graph_live.json markers-only` で再検証。
desktop/mobileの双方で20更新のGPU buffer生成/破棄0/0、32 draw calls、contextLost=false。
カメラ移動による画素変化も確認。描画単体中央値はdesktop 2.0 ms、mobile 1.5 ms。
これは保存スナップショットの再送であり、動的な再分割や長時間運転の性能を保証するものではない。

## Risk / Notes

全量Marker転送と描画呼び出し数は削減していない。複数レイヤーの重複表示や高密度ノードの描画負荷は別途残る。
容量はMarkerの生存中に縮小せず、削除・アンマウント時に解放。最大容量は過去最大要素数の2倍未満が目安。
長時間のGPU安定性・クラッシュ解消を保証するテストではない。
