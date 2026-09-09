# 2026-09-09 - Curved Surface Sharp Boundaries

## Summary

角付近のGNGノード法線が中間方向へ傾き、直角の平面同士まで曲面へ統合されるケースへの対策。
離れたパッチの代表法線の角度ではなく、接続境界での法線の連続性を追加評価。
GNG学習コアと既存の平面クラスタ抽出は変更なし。

## Changed

- 近接する平面パッチ同士では、GNGノードの法線に加えて、両パッチの境界法線を確認。
- `PlaneCluster.normal` を基準とし、利用できない場合はパッチ内の平均法線、最後に元ノード法線へフォールバック。
- 曲率の品質が0.5を満たす場合、既存Kで両方の法線を同じGNG edge中点へ補正。符号の反転も補正。
- 境界位置の法線角が `max_link_normal_deg` を超える平面ペアは `sharp_edges` に記録し、接続候補から除外。
- 同じ平面ペアに複数edgeがある場合、不連続境界の判定を優先。別の良好なedge1本だけで接続を復活させない。
- 非平面ノード経由の迂回経路があっても、既知のsharpな平面ペアを含む同一モデルの採用を禁止。
- 平面と非平面、非平面同士の接続角判定は従来どおり。元グラフや所属ノード自体は削除しない。

## Added

- `/curved_surface_clusters/models` のJSONに `sharp_edges` を追加。各要素は `patches` の添字2つ。
- 直角の平面でノード法線だけが45度方向に混合したケースと、非平面ノードによる迂回経路の回帰テスト。
- 代表法線が90度異なる局所パッチでも、円柱面として連続していれば統合を維持する肯定テスト。
- ROS検証でsharpな平面ペアを同じモデルに含めていないことを確認。
- 保存テンプレートの再入力テストで、ファイル内の平面法線も保持。

## Fixed

従来は平面クラスタ同士の代表法線を参照せず、境界ノード1組が距離・法線角条件を満たすだけで平面全体を統合候補にできた。
ノード法線の混合や非平面ノードの迂回で候補化された後、残差条件だけでは近退化quadric等への統合を止められない場合があった。

## Removed

なし。入力点群、元GNG edge、平面所属、保存テンプレートの変更なし。

## Behavior Impact

- 曲面領域全体の法線差を45度に制限する変更ではない。円柱の別側面など、離れたパッチ同士の90度の差だけでは棄却しない。
- 既知のsharpなペアを含む候補は数値フィット前に棄却。この棄却は `model_fits` に含めない。
- 可視化は元平面2クラスタを統合した曲面のMarkerが既定のまま。表示条件とは別にフィッティング候補を修正。
- 許容残差を小さくする変更ではない。RMS 4mm、単点残差12mm、接続距離8cm、接続角45度の既定値を維持。

## Topics / Params / Messages

- 追加パラメータ、launch引数、ROS msg定義の変更なし。
- `surface_model.max_link_normal_deg` を境界位置のパッチ法線にも適用。
- `surface_model.yaml` のコメントを更新。
- JSON schemaは `surface_region_graph_v1` のまま、`sharp_edges` の追加のみ。
- Viewerの配色、ノードと実edgeのMarker形式、フロントエンドの変更なし。

## Verification

Releaseビルド成功、C++ 21テスト成功。
混合法線の直角2面は、修正前に1領域へ誤統合される失敗を確認後、修正後に2平面へ分離。
迂回経路の禁止、2平面からの円柱統合、直交方向のパッチを含む連続円柱、回転・並進・密度差を加えた楕円柱、球、接する壁、色・所属・消去を検証。

```bash
colcon build --packages-select ais_gng --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
/ros2_ws/build/ais_gng/test_surface_model
```

`/semantic_points` の1243点を1フレーム保存し、別ROSドメイン197で10Hz再入力、20秒間GNG学習。
起動コマンド: `ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml input_topic:=/semantic_boundary_check/points`。
取得した同一フレームの837ノード・10平面を再学習せず比較。
修正前後で全モデルの種別・所属が一致し、5平面・114ノードの円柱（半径約57.6mm）を維持。
このフレームの統合円柱内で、直接接続した平面の代表法線角は最大約34度。ユーザーが見た直角統合の当該フレームは未取得であり、実シーンでの再現確認とは区別。
修正後の同一入力8フレームでコア平均2.060ms、p95 2.163ms、sharpペア0。

保存mug 502ノード・10平面を、保存済みの平面法線も含めて再入力。
sharpペア20、表示対象は2平面を含む79ノードの1球面モデル。表示対象外のモデルもJSONに保持。
8フレームでコア平均1.250ms、p95 1.478ms。誤統合が減っても、正しい物体部位への分割を保証する評価ではない。

```bash
ROS_DOMAIN_ID=197 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_surface_models.py \
  --launch --template /datasets/mug_gng_template.json.gz --seconds 5 \
  --output /tmp/curved_boundary_mug_plane_normals.json
```

結果はコンテナ内の `/tmp/curved_semantic_before.json`、`/tmp/curved_semantic_after.json`、`/tmp/curved_boundary_mug_plane_normals.json`。
法線省略版mugの参考試験は `/tmp/curved_boundary_mug.json`。こちらは平均法線へのフォールバック条件であり、保存法線版とは入力条件が異なる。
コア時間にはROS転送、JSON化、Marker生成、ブラウザ描画を含まない。

検証用GNG1回と `ros2 launch ais_gng surface_models.launch.py` による再入力3回はすべて停止済み。
GNGの終了コード-2は検証終了時のSIGINTによる停止。停止後に検証プロセスとROSデーモンの残存なしを確認。
既存bag、Viewer、および作業中にユーザーが起動したsemantic_points用GNGは停止していない。

## Risk / Notes

### 実行中の統合・消失の観測

semantic_pointsを入力中のGNGを10秒間購読し、15更新を取得。表示あり2更新、表示なし13更新。
モデル・Markerのpublisherは各1つで、重複配信なし。探索回数は50〜80回で、128回の上限には未到達。

| フレーム | 元平面ID 2と5の接続 | 統合結果と表示 |
| --- | --- | --- |
| 2143 | 平面2 → 非平面ノードID 288 → 平面5 | 97ノードの円柱、2平面を含むため表示 |
| 2150 | 元patch_edgesにも経路なし | 33ノードと62ノードの別円柱、各1平面のため非表示 |
| 2220 | 平面2 → 非平面ノードID 125 → 平面5 | 99ノードのquadric、2平面を含むため再表示 |

元ノードIDによる前後の照合でも分割を確認。2143から2150への消失で、MarkerのDELETEを1件観測。
今回の直接原因はGNG接続の変動。法線ゲート前のpatch_edgesでも経路が消えており、sharp追加や残差上限による棄却とは区別。
曲面推定は毎回の作り直しで、前回モデルの継続判定やヒステリシスは未実装。表示に必要な元平面数2を満たさなくなった時点で削除。
対策候補は、前回モデルを最新ノードで再検証する継続判定。表示だけを残す変更や、明確な形状不一致・sharp境界を無視する保持は未実装。
観測データ: コンテナ内 `/tmp/curved_surface_flicker_trace.json`。実装・パラメータの変更なし。
`docker exec -i gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout --signal=INT --kill-after=5s 18s python3 -'` から、rclpy観測ノード `curved_surface_flicker_probe` を実行。
観測ノードは10秒の購読後に終了。GNG・Viewer・bagへの停止操作なし。

その後の対応: [Curved Surface Retention](2026-09-09_curved_surface_retention.md) で、現在ノードの適合性による継続判定を実装。上記の「未実装」は観測時点の状態。

### 残る制約

- 曲率情報が不十分なパッチは代表法線で判定するため、粗い曲面パッチで保守的に分断する可能性が残る。
- sharp判定は近接する実GNG edgeを持つ平面ペアが対象。直接edgeのない2面について、この判定だけで誤統合を防ぐ保証はない。
- GNGの誤った接続や平面法線の誤推定も影響する。sharpは校正済みの確率判定ではない。
- 観測点のない境界中点への曲率補正は局所近似。曲面の実在や隠れた接続面を証明するものではない。
- 今回はC++とROS出力を検証。ブラウザ上の当該フレームの目視比較は未実施。
