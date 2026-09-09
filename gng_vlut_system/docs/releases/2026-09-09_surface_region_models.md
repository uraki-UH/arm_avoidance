# 2026-09-09 - Surface Region Models and Patch Curvature

## Summary

既存の平面クラスタと非平面ノードをLocalPatchとして再利用し、上位SurfaceModelへ統合するC++試作。
平面判定を最終形状とは扱わず、平面パッチも曲面に所属可能。GNG学習コアと既存PlaneClusterメッセージは変更しない。
現在の表示対象と退化モデル対策は [Curved Surface Merge Display](2026-09-09_curved_surface_merge_display.md) を参照。以下の初期検証時とは異なり、既定Markerは元平面2クラスタを統合した曲面のみを表示。

局所曲率の現行仕様は [位置偏差による曲率推定](../designs/curved_surface_position_fit.md) を参照。以下の法線変化方式は初期実装の記録。

## 現行の曲面表現：局所高さ関数とimplicit quadric

| 対象 | 表現 | 用途 |
| --- | --- | --- |
| 各平面パッチ | `h = au² + buv + cv² + du + ev + f` | ノード位置からの局所曲率・境界接平面推定 |
| 統合候補の領域全体 | `Q(x,y,z) = 0` | 曲面モデルへの適合・所属判定 |

領域全体の共通保存表現は、正規化座標 `(p-origin)/scale` におけるimplicit quadric。

```text
Q(x,y,z) = Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J = 0
```

- 一般二次曲面だけの当てはめではなく、平面・球・円筒・楕円筒・一般二次曲面の候補比較。残差と自由度ペナルティによる選択。
- 形状種別にかかわらず、上記10係数の暗黙式への保存表現の統一。
- 局所高さ関数も `h - f(u,v) = 0` とした場合、制約付きimplicit quadricの一種。ただし基準平面に対して高さが一意な局所範囲用の表現であり、カップ全周を単一の高さ関数で表現する方式とは別物。
- 今回の位置ベース化は局所推定と境界接平面の算出が対象。領域全体の円筒軸推定・法線整合判定などにおける入力ノード法線の使用は維持。
- 比較結果・品質指標・残存課題は [位置偏差による曲率推定](../designs/curved_surface_position_fit.md) に集約。

## Changed

- `ais_gng.launch.py` の可視化ノードに上位モデル出力を追加。既定2 Hz。
- 別起動用 `surface_models.launch.py` は実行中GNGの同一フレームのmapとplane_clustersを購読。
- 以前の `/nonplane_components/markers` は非平面連結成分の表示のまま。今回の曲面モデルとは別物。

## Added

- 平面クラスタ1個を1パッチ、未所属の有限座標ノード1個を1パッチに圧縮。所属添字は保持。
- 元GNG edgeをパッチ間edgeへ縮約。距離と符号に依存しない法線角で成長候補を限定。
- plane / sphere / cylinder / elliptic_cylinder / quadric の最小二乗推定と、自由度ペナルティ付き選択。
- 全所属点と各パッチの残差、法線整合性で採否を判定。係数推定のみ最大256点へ間引く。
- 平面パッチの対称2x2曲率テンソルK、主曲率2値、UV方向、法線、接平面基底、support_cov、normal_scatter、plane_rms、法線予測残差と品質指標。
- `r = -K q` の対称3未知数を直接最小二乗推定。符号不定な入力法線はパッチ内でそろえる。
- 推定Kとモデル微分の差を担当範囲内の法線変化差に換算し、高品質パッチに対して整合判定。
- 大きな平坦パッチの保護と、平面解が非平面モデルの最小ペナルティより良い場合の早期終了。

## Fixed

- 平面パッチを曲面候補から除外してしまう構造を上位層で解消。
- 同じSurfaceModelのノード、実GNG edge、任意の上位パッチグラフを同色に統一。別モデルは別色。現在のMarkerではunknownは非表示。
- Viewer表示は `/curved_surface_clusters/markers` のSPHERE_LISTとLINE_LISTを使用。消えたnamespace/idにはDELETEを送信。

## Removed

なし。保存済みテンプレートの書き換え・再学習は不要。

## Behavior Impact

- 学習コア内の平面抽出とは別に、可視化側で上位モデルを推定する試作。ROSシリアライズを除いたコア処理時間を計測。
- 広い平面の保護条件: 曲率品質 >= 0.8、plane_rmsが閾値以内、接平面位置分散のtraceが統合候補全体の位置分散trace以上、推定曲率による法線変化が許容値以内。
- 物体テンプレートの寸法を使った除外ではなく、現在の統合候補に対する相対的な広さ。大きさだけでは除外しない。
- パッチは分割しない。適合しない原子パッチはunknownとして残る。全有限ノードを重複なく保持。

## Topics / Params / Messages

```bash
# Docker内。既存GNGに後から追加する場合。
source /ros2_ws/install/setup.bash
ros2 launch ais_gng surface_models.launch.py
```

- launch引数: `input_topic` (既定 `/topological_map`)、`plane_clusters_topic` (`/plane_clusters`)、`output_topic` (`/curved_surface_clusters`)、`params_file`、`enable_markers` (既定true)、`enable_graph` (既定false)。
- `/curved_surface_clusters`: ais_gng_msgs/TopologicalMap。比較・外部利用向けの任意出力。既定では生成なし。
- `/curved_surface_clusters/models`: std_msgs/StringのJSON、schema=`surface_region_graph_v1`。
- `/curved_surface_clusters/markers`: visualization_msgs/MarkerArray。Viewerで選択する既定の曲面クラスタ表示。
- データはmapのframe_id、frame_number、stampを保持。node_indicesは配列添字でありnode.idとは区別。
- patchesに原平面ID・所属添字・curvature、modelsに種別・所属patch/node添字・暗黙曲面係数と残差。
- 曲率単位は1/m。主方向はaxis_u/axis_vに対するUVベクトル。主曲率は絶対値の大きい順。
- 法線不足・6点未満・接平面支持がほぼ線状の場合、`curvature.valid=false`。未推定をゼロ曲率とはしない。
- `confidence` は位置支持の条件と法線予測残差によるヒューリスティックで、確率・校正済み信頼区間ではない。
- Quadric座標は `(p-origin)/coordinate_scale`、係数順は `xx,yy,zz,xy,xz,yz,x,y,z,1`。
- 設定: `ais_gng/config/surface_model.yaml` の `surface_model.*`。
- 主要設定: `enable`、`hz`、`max_link_length`、`max_link_normal_deg`、`max_patch_rms`、`max_point_residual`、`max_normal_deg`、`max_curvature_normal_error`、`protect_dominant_flat_patches`、`complexity_penalty`、`max_model_fits`。
- Marker出力有効時は `enable_patch_graph=true` で圧縮パッチグラフを重畳。`enable_labels` は既定false。TEXT_VIEW_FACINGはRViz向けで、現在のViewerには表示されない。
- 通常のais_gng起動でも有効になるため、同じ出力トピックへstandaloneを重複起動しないこと。

## Verification

Releaseビルド、C++テスト13件、既存ViewerのMarkerArrayテスト1件に成功。
円柱への平面/非平面の混合統合、回転・並進・密度差を加えた楕円柱、球、直角境界、接する壁の保護、欠損法線、曲率方向・符号、ノード保存、色分け、消去を検証。

```bash
colcon build --packages-select ais_gng --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
/ros2_ws/build/ais_gng/test_surface_model
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_surface_models.py \
  --template /datasets/mug_gng_template.json.gz --seconds 10 --launch \
  --output /tmp/surface_model_mug_launch_check.json
```

実保存mugを同一データのまま再送して16フレーム計測。入力502ノード・平面10パッチ・非平面45ノード。

| 結果 | 値 |
| --- | --- |
| 統合曲面 | Quadric 2領域: 127ノード / 341ノード |
| 平面と非平面をまたぐ統合 | 2領域とも該当 |
| 未確定 | 34ノード |
| RMS近似表面残差 | 2.50 / 2.46 mm |
| Surface全処理 | 平均2.389 ms、p95 2.726 ms |
| うちパッチ曲率計算 | 平均0.0343 ms、p95 0.0412 ms |

コア時間はパッチ構築・曲率・モデル探索を含み、ROS配送・JSON化・Marker生成・ブラウザ描画は含まない。
同じ保存データの再送であり、動的シーンに対する安定性ベンチマークではない。
検証launchと子ノードはSIGINT後に正常終了を確認。既存ROSプロセスは停止していない。

初期検証時の配色は青127ノード、黄緑341ノード、灰色34ノード。色IDを疎なregion IDから直接生成せず、現在のモデル集合内で連番にして色相の近接を抑制。現在の表示条件では灰色の未確定34ノードを省略。
以下は最適化前のMarker経路の検証記録。現在のMarker再利用は `2026-09-09_curved_surface_marker_reuse.md`、任意のTopologicalMapとの比較は `2026-09-09_curved_surface_graph.md` を参照。
同じROS MarkerArrayを既存ViewerのMarkerArrayRendererで描画し、ChromeのソフトウェアWebGLで1100x800 / 390x844を確認。
両方とも非空の色付き画素、5 draw calls、contextLost=false。ブラウザとテスト用Viteサーバーは終了済み。
画像: `/tmp/surface_model_mug_desktop.png`、`/tmp/surface_model_mug_mobile.png`。
最終配色版の6秒再送でも同じ所属結果。平均2.440 ms、うち曲率0.0378 ms。

## Risk / Notes

- mugは円柱としては採用されず一般Quadricとなった。胴体/取っ手の正しい意味的分離を保証しない。
- mugの全10平面パッチで曲率品質が低く、K整合ゲートは適用されなかった。保存ノード法線のばらつきと局所線形近似の適合性に課題が残る。
- GNG位置・法線を等重みで利用。元入力点群の担当面積重みや4次momentの積算、QuadricStats Hのmergeは未実装。
- 残差 `abs(F)/norm(grad F)` は一次近似で、真の最短距離ではない。正解ラベル付き精度評価は未実施。
- region IDは最小所属ノード添字に由来するフレーム内ID。時系列追跡ではないため、再分割やモデル集合の変化で色が変わる場合がある。
- 128回の領域fit上限は計算量制限であり厳密な時間上限ではない。大規模入力の計測は別途必要。
