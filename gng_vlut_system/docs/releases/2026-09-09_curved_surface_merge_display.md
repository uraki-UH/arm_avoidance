# 2026-09-09 - Curved Surface Merge Display

## Summary

曲面Markerの既定表示を「元平面クラスタを2つ以上統合した非平面モデル」に限定。
2つの平面パッチから円柱等への統合は許可し、入力パッチ数とモデルの退化判定を区別。
一般二次曲面が2平面の積だけで低残差を達成するケースを除外。
後続の境界法線・迂回統合対策は [Curved Surface Sharp Boundaries](2026-09-09_curved_surface_sharp_boundaries.md) を参照。

## Changed

- 平面1パッチと非平面ノードの統合、非平面ノードのみの統合、plane、unknownを曲面Markerの既定表示から除外。
- 表示対象のノード、実GNG edge、任意のラベル・上位パッチグラフのみ同色で出力。対象外へ変わった既存MarkerにはDELETEを送信。
- ログに表示対象モデル数 `shown`、表示対象ノード数 `nodes`、推定回数と上限 `fits=N/128` を追加。

## Added

- `surface_model.min_display_plane_patches`、既定2。
- `/models` JSONの各モデルに `plane_patch_num` と `is_display_candidate`、ルートに `min_display_plane_patches` を追加。
- 有限時間通信テストに `--params-file` と表示対象の所属・数の検証を追加。

## Fixed

- `F=L1*L2=0` は元の2平面の和集合を表し、滑らかな曲面への統合を示さなくても残差を小さくできた。
- 正規化座標で一般quadricの対称4x4行列Qを検査。絶対固有値を昇順に並べた2番目が最大値の1e-6倍以内なら、そのquadric候補を除外。
- 上記は数値的なrank 2以下のモデル除外。入力平面が2パッチという理由で統合を禁止する判定ではない。plane候補は従来どおり別途評価。

## Removed

曲面Markerでの未確定ノードの灰色表示と、単独平面の重複表示。
元GNG、平面クラスタ、非平面ノード、保存テンプレート自体の削除・変更なし。

## Behavior Impact

- フィッティングと表示フィルタは独立。非表示になったモデルの所属や推定結果も `/models` に保持。
- 任意の `enable_graph=true` によるTopologicalMapは従来の全ノード・モデル所属を保持。今回の表示フィルタはMarkerに適用。
- `min_display_plane_patches=1` で平面1パッチを含む曲面、0で非平面ノードのみの曲面も表示可能。planeとunknownは値によらず非表示。
- 学習コアと平面クラスタ抽出は変更なし。許容残差・法線角・推定回数上限の既定値も変更なし。

## Topics / Params / Messages

- 表示: `/curved_surface_clusters/markers`、MarkerArray、既定有効。
- 診断: `/curved_surface_clusters/models`、std_msgs/String、schema=`surface_region_graph_v1`。追加フィールドのみ。
- 設定: `ais_gng_cpu/src/ais_gng/config/surface_model.yaml`。
- 既定: `max_patch_rms=0.004` m、`max_point_residual=0.012` m、`max_normal_deg=35`度、`max_model_fits=128`。
- 通常起動: `ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml`。変更反映には起動し直しが必要。
- 別起動用のlaunch引数に変更なし。同じ出力トピックへの重複起動は避ける。

## Verification

Releaseビルド成功、`test_surface_model` の19テスト成功。
30度の折れ目を持つ2平面は修正前の1領域から2平面へ分離。
局所平面RMS 2mm未満の2パッチから1つの円柱へ統合・表示する肯定テストも成功。
表示対象の色分け、単独平面+非平面の非表示、非平面のみの明示表示、消去、JSON所属保持を検証。

```bash
colcon build --packages-select ais_gng --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
/ros2_ws/build/ais_gng/test_surface_model
ROS_DOMAIN_ID=197 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_surface_models.py \
  --launch --template /datasets/mug_gng_template.json.gz --seconds 5 \
  --output /tmp/curved_display_mug.json
```

保存mug 502ノード・平面10パッチ・非平面45ノードを同一入力のまま再送。
各設定8フレーム、法線角等は固定し、RMSと単点残差を組で変更。

| パッチRMS / 単点残差 | 表示曲面 | 表示ノード | 元平面パッチ数/曲面 | fit回数 | コア平均 / p95 |
| --- | --- | --- | --- | --- | --- |
| 4 / 12 mm（既定） | Quadric 2 | 468 | 2, 8 | 115 | 2.673 / 3.083 ms |
| 3 / 9 mm | Quadric 3 | 413 | 2, 4, 3 | 119 | 2.376 / 2.516 ms |
| 2 / 6 mm | Quadric 1 | 263 | 5 | 93 | 2.001 / 2.175 ms |

誤差を厳しくすると統合範囲は縮むが、正しい胴体・取っ手分割になったことは未検証。
コア時間はROS転送、JSON、Marker生成、ブラウザ描画を含まない。8フレームの参考値であり性能保証ではない。
比較結果はコンテナ内 `/tmp/curved_display_mug{,_3mm,_2mm}.json` に記録。

過去取得済みの3000ノードの1フレームでは2515ノードがunknown、13領域が単独plane、残りのcylinder/quadricも元平面は各1パッチ。
複数平面を統合した曲面は0件だったが、従来Markerでは全3000ノードを表示していた。今回の既定表示条件なら0件。
このフレームのfitは128回の上限に到達。非表示とする変更は、未統合の原因を解消するものではない。

検証スクリプトから一意の出力先で `ros2 launch ais_gng surface_models.launch.py` を起動。
mug検証3回と実入力検証1回のlaunch・子ノードはすべてSIGINTで終了し、残存なしを確認。
実入力検証は7秒間結果を受信できず未完了。終了後のプロセス確認では既存GNGが停止していた。既存GNGへの停止操作なし。
既存bag再生とViewerは継続。ROSデーモンの新規残存なし。

## Risk / Notes

- 2枚の局所平面を1つの滑らかな曲面で説明することは必要。平面式2個の積だけでは滑らかさや同一物体の証拠にならない、という区別。
- rank判定は退化モデルの一部への対策。ノイズを含む近退化モデルや一般quadricの過剰な適合を全面的に解決するものではない。
- mugの2曲面は修正後も採用。残差は一次近似距離であり、正解ラベル付き精度検証や分岐面の連続性検証は未実施。
- 多数の未確定ノードを表示から外しても、フィッティング・全結果JSONの計算量そのものは減らない。
- フロントエンド変更なし。今回の色・所属・消去はC++とROS MarkerArrayで確認し、ブラウザ実画面は再検証していない。
