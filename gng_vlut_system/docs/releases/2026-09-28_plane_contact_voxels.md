# 2026-09-28 - 平面に接する入力占有セルの可視化

## 要約

CPU版GNGに `/plane_contact_voxels`（`visualization_msgs/msg/MarkerArray`）を追加。
GNGが作成済みの入力voxelを再利用し、別グリッド・生点群の再集計なしでCUBE_LISTを出力。

- 緑 `plane_node_cells`：平面所属ノードと同じ入力占有セル。
- 黄 `adjacent_input_cells`：緑の元となるノードセルに面共有で6隣接する入力占有セル。
- 入力点のないセル・斜め隣接は出力対象外。同一セル判定を隣接判定より優先。
- 平面の厳密な幾何交差・点の平面所属・挿入候補そのものではない。黄色の点の自動取り込みなし。
- ノード挿入側は既存の最近傍ノード経由を維持。[セル→平面索引の比較](../../../benchmarks/voxel_plane_lookup_20260928/README.md)は試作に限定。
- 未購読時は抽出・Marker生成を省略。購読時も既定最大2 Hz。OFF時はpublisher自体なし。
- 平面処理OFF時もpublisherなし。空の結果には対象namespaceのDELETEを送信。
- 同日最適化：8 MiB以内のビット表で直接照合、前回使用領域だけのクリア。巨大グリッドは疎索引へ退避。
- 完全な差分更新ではなく、各出力時の平面ノード走査は継続。hull判定・重点抽選・学習条件は変更なし。

at128は確認用にON、その他の設定ファイルに対する宣言既定はOFF。起動時設定：

```yaml
plane_contact.enable_voxels: true
plane_contact.interval_sec: 0.5  # 出力間隔[s]。0は毎入力。
```

`input.voxel_grid_unit`がそのまま表示セル幅。CPU直結の平面計算・voxel ONが必要。
接触表示は `enable_node_insertion` と独立。既存のat128の直接挿入ON設定は維持。
起動済みの旧バイナリには再起動が必要。確認時は稼働中ノードのpublisherも検出済み。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points
```

TopoFuzzyViewerでMarkerArrayトピック `/plane_contact_voxels` を選択。
既存GatewayのCUBE_LIST変換・TSXのワイヤーフレーム表示を利用。Viewer側のコード変更なし。

## 条件・検証

- 同日最適化後：新旧同一入力の抽出API 1.225→0.725 ms（40.8%減）。ROS送信・描画は対象外。
- 60入力×3試行で全セル出力一致、索引切替等6条件一致、製品CTest 23件成功。[比較・再現](../../../benchmarks/voxel_plane_lookup_20260928/README.md)。
- この最適化は別ビルドで検証。稼働installへの反映・既存ノード再起動は未実施。以下は最適化前の導入時記録。
- 通常Releaseのgng_cpu / ais_gngをビルド・installへ反映。CPU26件、ROS関連6件が成功。
- `allow_external_sampler=OFF`の別ビルドも23件成功。任意サンプラーAPIの公開制限は維持。
- 接触APIの未初期化、入力置換後、空入力、voxel無効、古い世代、セル中心・点数、正負の隣接、斜め除外を検証。
- API `gng_get_plane_contact_voxels` は世代付き参照ノードを受け、セル中心・点数・区分を返却。
- 現入力のセルだけが対象。戻り配列は次回同API呼出しまで有効な借用配列。
- 実bag60入力×4条件×3回、全12試行成功、全試行 `cleanup_ok=true`。
- OFFのトピック不在、未購読時のpublisher、2 Hzの受信・間隔、毎入力の60メッセージを確認。
- 毎入力の検証ではframe/stamp・セル幅・格子中心・色・空集合時DELETEも確認。
- 平均表示数は2 Hz条件約11,751セル、毎入力条件約12,086セル。Viewerの実画面操作・描画負荷は未測定。
- 約19,111ノードの独立比較では、参照列作成＋実API抽出が平均1.512 ms/出力、約12,967セル。
- 抽出値は60入力×3試行、先頭20入力除外。ROS送信・シリアライズ・描画は含まない。
- ROS全体平均はOFF 53.53 / 未購読59.92 / 2 Hz 88.86 / 毎入力58.20 ms。
- 同じ順のGNG区間が37.17 / 41.14 / 67.96 / 37.81 msと変動。追加ノード総数も異なり、差を可視化コストと断定不可。
- 全体時間は並行稼働下の参考値。可視化以外の保存済みパラメータは入力namespace以外一致。
- 初回リンク失敗は同ソース再ビルドで解消（原因未特定）。初回のvoxel無効テストは起動後変更禁止を誤ったため別の起動時設定試験へ移動。
- 初回ROS試験はDELETEのサイズまで要求して失敗。描画対象だけの寸法検証へ修正して全条件を再検証。
- 最終ROS反復は予測360秒→実測406.45秒。抽出比較3試行は予測21秒→19.84秒。
- 試験・ビルド・presence確認の全プロセス終了。既存bag・Viewer・3コンテナの継続稼働を確認。
- 作業中に既存GNG・TFのPID変更を観測。エージェントによる既存プロセスの停止・再起動なし。

主な検証コマンド（コンテナ内、通常ビルドは `/ros2_ws`、反復試験は `/ros2_ws/src`）：

```bash
timeout 420 colcon build --packages-select gng_cpu ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure
ctest --test-dir /ros2_ws/build/ais_gng -R 'test_(plane_cluster_incremental|nonplane_component_extractor|node_support|grasp_attention|boundary_attention|spatial_sampling)$' --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py \
  benchmarks/voxel_plane_lookup_20260928/contact_cases.json \
  --output artifacts/voxel_plane_lookup_20260928/ros_batch_verified \
  --repeats 3 --timeout-sec 120 --max-total-sec 1440 --estimate-sec 30
```

起動は `docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && …'`。
ROS試験はdomain 183・CPU 4、実行argv・PID・停止結果は `artifacts/voxel_plane_lookup_20260928/ros_batch_verified/report.json`。
抽出時間は `extract_batch/report.json`。通常・製品のビルド／CTestログも同artifactディレクトリへ保存。
製品確認は別の `product` ビルド先で `-Dallow_external_sampler=OFF -DGNG_BUILD_BENCHMARKS=ON -DGNG_ENABLE_FRAME_LOG=OFF` を指定。
最後の公開確認は2秒間だけの `plane_contact_presence_check`（rclpy）でpublisher一覧を取得、destroy/shutdown済み。
