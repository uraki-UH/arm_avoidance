# 2026-08-25 - Reachability filtered environment voxelization

## Summary

VLUTへ入力する環境占有ボクセルを、ロボット基準座標系のreachability範囲内だけで構築できるようにした。

## Changed

- `point_to_voxel_node`の全量`PointCloud2`コピーと変換済み点vectorを廃止した。
- 点をロボット基準座標系へ逐次変換し、範囲内の点だけを直接voxel IDへ集約する構成へ変更した。
- 非有限値、範囲外点、同一voxelの重複をpublish前に除外する。
- 範囲内voxelの重複除去を再利用可能なdense bitmapで高速化し、過大領域では再利用hashへfallbackする。

## Added

- TF変換後のreachability AABBと軸別marginによる早期除外。
- 入力点数、採用点数、範囲外点数、非有限点数、出力voxel数、callback処理時間のthrottleログ。
- reachability境界と逐次集約処理の単体テスト。

## Behavior Impact

- `point_to_voxel.launch.py`と`point_to_vlut.launch.py`ではreachability filterが既定で有効になる。
- nodeを`ros2 run`で直接起動した場合は後方互換のためfilterが無効になる。
- 範囲内に点がない入力でも空の`voxel_msgs/Voxel`をpublishし、下流の占有を消去できる。

## Topics / Params / Messages

- topic名とmessage型の変更なし。
- `enable_reachability_filter`: reachability filterのON/OFF。
- `min_reachability_x`、`max_reachability_x`: ロボット基準座標系のx範囲、単位m。
- `min_reachability_y`、`max_reachability_y`: ロボット基準座標系のy範囲、単位m。
- `min_reachability_z`、`max_reachability_z`: ロボット基準座標系のz範囲、単位m。
- `reachability_margin_x`、`reachability_margin_y`、`reachability_margin_z`: 把持物の張り出し、位置誤差、移動台車の対象移動範囲を含む軸別margin、単位m、既定値0.2 m。
- `max_dense_voxel_num`: dense bitmapに割り当てる最大voxel数、既定値8,000,000。超過時は再利用hashへ自動fallback。

## Verification

- 一時Dockerコンテナ内で`colcon build --symlink-install --packages-up-to gng_vlut_system --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release`: 成功。
- 一時Dockerコンテナ内で`colcon test --packages-select gng_vlut_system`: 11件のgtest成功。
- rosbag配信中の`/camera/camera/depth/color/points`、`frame_id=base_link`、約18万点、0.02 m voxelで実測。
- filter無効: callback平均7.56 ms、出力平均9,081 voxel、CPU約9.0%、RSS約62 MiB。
- 各軸0.2 m margin付きfilter有効: callback平均6.43 ms、出力平均5,165 voxel、CPU約8.0%、RSS約64 MiB。
- filter有効時は入力点の約85%を保持し、callback時間を約15%、出力voxel数を約43%削減。
- 同一binaryのhash fallback平均6.67 msに対し、dense bitmap平均6.17 msで約7.5%短縮。
- 定常RSS瞬間値はhash fallback約42 MiB、dense bitmap約52 MiB。DDS受信bufferを含む参考値。

## Risk / Notes

- 範囲値は`target_frame_id`の座標系で評価する。launchの既定値は`ToPoDualArm.yaml`のTCPサンプリング範囲を各軸0.2 m拡張した領域に対応する。
- 別ロボットまたは別GNG profileでは、対象configのreachability範囲に合わせてlaunch引数を指定する必要がある。
- 把持物や移動台車の対象移動範囲が0.2 mを超える場合は、該当軸のmarginを拡大する必要がある。
- 移動マニピュレータでは点群時刻のTFにより切り出し領域が現在のrobot baseへ追従し、marginが計画上の先読み移動範囲になる。
- rosbag再生周期は測定中に約6〜16 Hzで変動したため、CPU使用率は実測時の参考値とし、処理比較にはcallback時間を用いる。

## World bucket feasibility benchmark

複数ロボットで点群索引を共有する方式の成立条件確認用として、固定幅world bucketと
`world_bucket_benchmark_node`を追加した。各bucketは元点を保持し、world AABBによる粗抽出後に
各robot座標系へ変換して既存の2 cm voxel IDへ集約する。直接走査とのID比較を各フレームで実施する。

- 入力約18万点、0.2 m bucket、既定reachabilityと各軸0.2 m margin、20フレーム。
- bucket構築p50 3.69 ms、直接走査p50は1台5.51 ms、8台28.46 ms。
- bucket合計p50は1台9.21 ms、8台32.62 ms。ROIが入力点の大半を含む条件では不採用相当。
- `y/z=[-0.5, 0.5] m`、各軸0.1 m marginの感度試験では、8台目のROI採用点が約1.3万点。
- 感度試験の8台合計p50は直接20.35 ms、bucket 17.83 msで約12.4%短縮。
- 全測定で`mismatch_num=0`。robotごとの回転を含めて直接方式とVLUT IDが完全一致。
- 1～4台では索引構築費を回収できず、8台かつ狭いROIでのみ有効性を確認。

固定俯瞰カメラ向けにbucketと点配列の確保容量をフレーム間で再利用する。現時点では比較用実装であり、
`point_to_voxel_node`の既定経路には組み込まない。

広いROIでは索引よりrobot単位の並列化が有効だったため、完全一致のOpenMP比較経路も追加した。

- 8台逐次p50 28.21 msに対し、8 thread p50 5.89 ms。約4.8倍高速、wall time約79%短縮。
- 4 thread p50 10.02 ms。約2.8倍高速、wall time約64%短縮。
- TF変換、reachability判定、2 cm voxel ID集約、sortまでrobot単位に独立実行。
- 全測定で逐次版とのID不一致なし。近似処理なし。
- wall time短縮と引き換えに同時CPU使用量が増えるため、実運用では専用thread数の上限設定が必要。

### ROI sensitivity

同じ実点群で環境AABBと局所ROIの比率を追加測定した。

- 環境点群AABBは約2.26×2.07×1.06 m、入力約18万点。
- 局所chunk ROIはmargin込みで約0.36×0.36×0.36 m、同じ位置の8ロボットとして共有効果を測定。
- ROI採用率p50 11.7%、bucket構築p50 3.74 ms、全条件でVLUT ID不一致なし。
- 1台は直接2.06 ms、bucket 4.50 ms。2台は直接4.14 ms、bucket 5.20 ms。
- 4台は直接8.24 ms、bucket 6.61 msで約19.8%短縮。
- 8台は直接16.44 ms、bucket 9.39 msで約42.9%短縮。
- 4 threadのrobot並列は8台4.50 msで最短。bucketと異なり同時CPU使用量の増加が主な交換条件。

固定bucketの損益分岐は今回のCPUと点分布では、ROI採用率約12%で3～4台の間だった。
全reachabilityを1個のAABBにすると空領域を大量に含むため、実運用ではリンク別swept volume、
予測軌道chunk、把持物拡張chunkの和として問い合わせる構成が適切。

### World index reuse

world indexを更新せず次フレームへ単純再利用する方式と、固定カメラdepthによる差分更新候補率を測定した。

- 2フレーム更新周期の再利用フレームは8台約5.5 msまで短縮。
- 2フレーム更新周期はoccupied ID recall 90.2%、precision 91.6%。安全用途には不採用。
- 4フレーム更新周期はrecall 91.9%、precision 89.6%。更新周期延長でも残留IDが増加。
- 変換済みPointCloud2と元PointCloud2は点数が毎フレーム変わる非organized形式。pixel単位のhandle更新不可。
- raw depth imageは固定解像度でpixel IDを維持可能。
- depth変化率p50は5 mmで30.6%、10 mmで22.5%、20 mmで15.2%、50 mmで7.6%。
- 2 cm VLUT基準では、全点再構築の代わりに約15%のdepth pixelを更新候補にできる見込み。

採用候補はpersistent world indexとdepth差分overlayの組み合わせ。初期prototypeではvalid depthの変化を
即時反映し、depth値が0になったpixelだけをfree-space証拠の連続確認後に削除する。valid depthが奥へ移る
変化も連続確認したい場合は、旧点と新点を併存させる二層overlayの追加が必要となる。camera-to-world TF、
画像寸法、intrinsicsが変化した場合はindex全再構築。定期的な低頻度全再構築も併用する。

### Persistent depth world index prototype

raw depthのpixel indexを安定handleにする`persistent_depth_world_index`と、毎フレーム全depthから
VLUT IDを再構築する基準方式との比較node`depth_world_index_benchmark_node`を追加した。indexは
world座標系の0.2 m固定bucketにpixelごとの3D点を保持する。

比較条件は`config/depth_world_index_benchmark.yaml`へ集約した。

視覚検証用として、world indexの間引きPointCloud2、ROI voxelのCUBE_LIST Marker、間引きなしの
`voxel_msgs/Voxel`をdebug topicへpublishする。raw depth rosbagのidentity座標設定では、各debug messageの
frameは`camera_depth_optical_frame`となる。

- depth値が1 mm以上変化したvalid pixelは即時にworld点を更新する。
- depth値が0のpixelだけは`free_confirmation_num`フレーム連続時に削除する。valid depthが奥へ移る変化は
  現観測を優先して即時更新する。
- camera-to-world transform、画像寸法、intrinsicsの変更時は全点を再構築する。
- 各robotは自身のworld poseから局所AABBを問い合わせ、既存のreachability filterと`VoxelIdCodec`で
  従来どおりVLUT IDを生成する。
- 現時点では比較用nodeのみであり、productionの`point_to_voxel_node`には未統合。

Docker内のRelease buildとgtestは成功し、合計17 testでerror/failureなし。

#### Raw depth実測

`/rosbag/uraki/rosbag2_2026_04_22-19_10_41`の848×480 raw depth、camera_info、0.02 m voxel、
0.2 m bucket、60 frameで測定した。camera-to-worldと8台のrobot poseはidentity、すなわち共有indexの
処理費を分離する比較条件である。

| 条件 | index更新 p50 | VLUT ID | 1台 直接 / persistent total p50 | 4台 直接 / persistent total p50 | 8台 直接 / persistent total p50 |
| --- | ---: | --- | ---: | ---: | ---: |
| 広めROI、1 mm更新、即時削除 | 8.84 ms | recall=1、precision=1、ID不一致0 | 10.83 / 16.46 ms | 44.20 / 37.37 ms | 88.05 / 65.98 ms |
| 広めROI、1 mm更新、3 frame削除確認 | 8.40 ms | recall=1、precision=0.922 | - | - | 90.61 / 68.20 ms |
| 広めROI、20 mm更新、即時削除 | 4.59 ms | recall=0.943、precision=0.850 | 10.76 / 11.69 ms | 43.82 / 32.34 ms | 87.53 / 60.08 ms |
| 狭い局所ROI、1 mm更新、即時削除 | 8.06 ms | recall=1、precision=1、ID不一致0 | 6.09 / 8.79 ms | 24.28 / 10.95 ms | 48.48 / 13.57 ms |

広めROIの完全整合設定では、8台で約25%短縮した。狭い局所ROIでは4台で約55%、8台で約72%短縮した。
1台ではdepth全画素の差分検出費を回収できず、2台前後から共有indexの効果が現れる。

このbagでは1 mm以上変化するpixelがp50約29万、全407,040 pixelの約70%であり、depth差分の候補自体は
少なくなかった。したがって20 mm閾値による候補削減は速度面では有効でも、VLUT occupied IDのfalse negativeを
発生させるため安全経路では不採用とする。実運用の基準は`depth_update_mm_th:=1`、完全整合が必要な経路では
`free_confirmation_num:=1`とする。0深度の瞬断に保守余裕が必要な経路だけ`free_confirmation_num:=3`を使い、
一時的なfalse positiveを上流の安全余裕として受け入れる。
