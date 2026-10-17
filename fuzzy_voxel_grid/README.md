# FVGとworldバケットの共通点群経路

PointCloud2のworld索引・ROI・FVG集計を同一プロセス内で共有。
設定は[shared_world.yaml](config/shared_world.yaml)・[voxel_grid.yaml](config/voxel_grid.yaml)を参照。

## 構成

```text
PointCloud2 → world bucket → ROI占有 → VLUT
                         └→ セル件数 → FVG評価
```

同じshared_point_store名でも別プロセス間では共有不可。writerは1個。
公開スナップショットは不変、利用側は最新1件を参照。遅い読者による世代保持があるため、物理バッファ1枚の保証はない。

## 座標・解像度・評価の独立性

world索引、ROI、FVGはそれぞれのframe・セル幅・原点を持ち、同一設定の場合だけ集計を再利用。
Tmapは点群とframeが一致し、時刻条件を満たす場合だけ評価。TF欠損・時刻巻戻りで旧データを混在させない構成。
freezeのフィルタ出力は元の座標・header・追加属性を維持し、world変換済み点群ではない点に注意。
連続索引の上限は方式選択用で、点数・出力の切捨てではない。

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

センサ→world→targetのTFは別途必要。ROIはtarget座標、FVGはworld座標。
設定は[world](config/shared_world.yaml)・[FVG](config/voxel_grid.yaml)。屋外では範囲の調整が必要。
このlaunchはworld＋ROI＋FVGのみ。VLUT・GNG・Viewerは別途起動し、同じ入力のworld単独ノードを重ねない構成。

## 検証・対象外

検証条件は[試験手順](test/README.md)、性能は[測定結果](../benchmarks/shared_voxel_cost_20260926/README.md)を参照。
別プロセス共有メモリ・実bagの認識品質・実Viewer・長時間メモリ上限は対象外／未検証。
