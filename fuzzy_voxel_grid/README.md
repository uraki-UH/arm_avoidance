# FVGとworldバケットの共通点群経路

## 構成

点群の保持・空間検索・セル別点数集計を`voxel_idx::point_cloud_store`へ分離。
worldバケットにfuzzy属性を必須化せず、FVGが同じ点群スナップショットを参照する構成。

```text
PointCloud2 → world変換・bucket索引（唯一のwriter）
                ├─ worldバケットの可視化
                ├─ 各ROI座標への変換・占有ID → 既存VLUT入力
                └─ 共有セル件数（設定ごとに1回）→ 各FVGのTmap集計・評価
```

- 共通ストア：ROSメッセージ・fuzzy評価・GNG処理に非依存のC++ライブラリ。Eigen使用。
- 同一プロセス内で`shared_point_store`名が一致した利用者間だけの共有。
  別プロセスで同じ名前を設定しても共有不可。二重writerは起動時に拒否。
- world座標XYZの所有元はbucket索引のみ。FVGのセル内XYZ配列は撤去済み。
  FVGは共有セル件数を参照し、表示用件数・Tmapノード数・ラベルだけを保持。
  freeze時だけフィルタ用ノード座標を保存。未使用の履歴・256ラベル集計配列は撤去。
- 元のPointCloud2は共有所有権で保持。intensityなどの追加属性を削除しない。
  索引内XYZと元メッセージは用途の異なる表現であり、元属性は索引の各点への直接アクセスAPIでは未公開。
- 公開後のスナップショットは不変。writerは読者が保持中のバッファを再利用しない。
  通常は2世代の容量を再利用し、遅い読者が旧世代を保持する場合は必要な世代だけ追加確保。
  「全体で物理バッファ1枚」や「全フレーム配送」の保証ではない。
- FVGはタイマー周期で最新1フレームを参照。入力より遅い場合は中間フレームを省略。
  未更新時のXYZ再走査・セル再構築なし。可視化の再配信は従来周期を維持。
- 共通セル集計は連続配列索引と使用済みセルだけの初期化。過大な範囲ではhashへ自動切替。
  bucket検索も共通化し、狭域は候補キー検索、広域は保持bucket走査で空領域の検索を省略。
- Marker生成はlive/freeze共通。入力・Tmap更新時に生成し、未更新時はキャッシュを再配信。
  全セルの診断集計はDEBUG時のみ。判定式・解像度・点数・配信周期は維持。

## 座標・解像度・評価の独立性

| 対象 | 座標・解像度 | 保持内容 |
| --- | --- | --- |
| world索引 | `world_frame`、`bucket_size` | 現フレームのworld XYZ |
| ROI | `target_frame`、`voxel_size` | 範囲内の占有ID。worldと異なるTFも可 |
| FVG | `world_frame`、`voxel_size_x/y/z`、`grid_origin_x/y/z` | 範囲・除外箱を適用した件数、Tmap属性、評価 |

検索bucketと評価セルを同じ幅にする必要はない。異なるセル幅・原点・ROIの集計結果は別管理。
`point_frame_channel::cell_query(point_cell_spec)`で集計を取得。
同じframeスナップショット・セル幅・原点・範囲・除外箱・配列上限の読者間で結果を共有。
異なる設定は別集計。writerのXYZ変換ループへの融合や、別TFのROI占有集計との共用ではない。
world単独ではセル集計の要求もFVG用配列の確保も不要。

`max_dense_voxel_num: 8000000`は連続索引を使う範囲内セル数の上限。`0`で常時hash。
連続索引は1セル4バイト／バッファ。読取中の旧結果は不変とし、解放済みバッファを再利用。
上限は索引方式の選択用であり、出力セル数・点数の切捨てではない。
点群の共有であり、worldの占有判定をfuzzyに置き換える変更ではない。
既存の評価基盤を維持し、新しい非平面・密度・履歴スコアの式やサンプリング規則は追加していない。

共有時のTmapは点群と同じ`frame_id`が必要。
`0 <= 点群stamp - Tmap stamp <= max_tmap_age_sec`のときだけ属性を集計。
既定`max_tmap_age_sec: 1.0`、単位は秒。未来・古過ぎる・別座標のTmapを自動変換して混在させない。
集計は常に現フレームのみ。frame変更・時刻巻戻しで旧セルを混在させない。
TF欠損時は新規worldフレームを作らず待機。入力停止時の保持済み表示には元stampを使用。

freeze中もworld索引は更新可能。FVGの固定状態による除外判定にはworld座標を使用し、
`filtered_new_points_topic`の出力座標・header・intensity等は元入力のまま保持。
このフィルタ出力はworld変換済み点群ではない。

## ビルド・起動

以下は既存ROS依存をビルド済みの`gng_cpu_container`内、ワークスペース`/ros2_ws`の例。
ホストビルドも同じCMake構成で、パスとROS環境を置換。ホストでの実ビルドは未検証。
FVGの`COLCON_IGNORE`は維持。通常のworkspaceビルドにFVGを強制追加しない。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
colcon build --packages-select voxel_idx gng_vlut_system --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cmake -S src/fuzzy_voxel_grid -B build/fvg_shared \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/ros2_ws/install/fuzzy_voxel_grid \
  -Dvoxel_idx_DIR=/ros2_ws/install/voxel_idx/share/voxel_idx/cmake
cmake --build build/fvg_shared -j4
cmake --install build/fvg_shared
```

起動する各シェルで環境を読込後、共有launchを実行。

```bash
source /ros2_ws/install/setup.bash
source /ros2_ws/install/fuzzy_voxel_grid/share/fuzzy_voxel_grid/local_setup.bash
ros2 launch fuzzy_voxel_grid shared_world_voxel.launch.py \
  input_topic:=/dataset/points world_frame:=world target_frame:=world
```

必要なセンサー→world、world→targetのTFを別途用意。
既定出力は`/world_index/buckets`、`/roi_voxels`、`/voxel_markers`。
`/voxel_centers`は従来通り`publish_voxel_centers: true`の場合だけ配信。
`world_params_file:=...`で[world設定](config/shared_world.yaml)、
`fvg_params_file:=...`で[FVG設定](config/voxel_grid.yaml)を変更可能。
ROI範囲はtarget座標、FVG範囲はworld座標。既定範囲はロボット近傍向けのため、屋外では要調整。
launchの`input_topic/world_frame/target_frame`はworld YAMLの対応設定より優先。

このlaunchはworld＋ROI＋FVGのみ。VLUT・GNG・Viewer・TF配信は別途起動。
同じ入力を扱う既存worldノードを重ねて起動すると、共有経路とは別に再処理されるため併用不要。
`environment_to_vlut.launch.py`全体の起動方式は今回変更していない。
既存のworld単独・FVG単独起動はそのまま使用可能。`shared_point_store: ""`が独立動作の既定値。
共有launchは内部で両ノードの名前を`world_points`へ揃え、world索引構築を有効化。

## 検証・対象外

検証日：2026-09-26、ROS 2 HumbleコンテナのReleaseビルド。
共通所有権・別TF・異方性セル・原点・除外箱・Tmap時刻・freeze属性保持・10万点更新・
単独互換性の検証内容は[試験手順](test/README.md)を参照。
GNG内部の学習用索引、persistent depthベンチマーク固有の履歴、別プロセスの共有メモリは対象外。
実bagの認識品質・実Viewer描画・長時間のメモリ上限は未検証。
固定・動的な合成入力のCPU・メモリ・遅延の変更前後比較は
[性能測定](../benchmarks/shared_voxel_cost_20260926/README.md)を参照。広域の集計・Marker通信負荷は残存。
