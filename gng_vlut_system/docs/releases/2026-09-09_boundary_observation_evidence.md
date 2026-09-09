# 2026-09-09 - 境界候補の観測証拠と視野端の分離

## Summary

GNG実行側で低次数の境界候補を実測レイから追加検査。
遮蔽・自由空間・視野端の独立した証拠ビットと、証拠なしの不明状態を配信。
グラフの接続欠けや観測角度min/maxからの自由空間推定なし。

## Changed

- 次数による `is_boundary_candidate` は維持。追加判定は候補ノードのみ。
- 実行側の `boundary_evidence.hpp` にセンサ視野・実測点索引・局所面比較を実装。
- 既存の `Boundary: ... ms` に索引構築と追加判定時間も含む方式。
- ViewerはGNGからの証拠を転送・表示のみ。Labels内の既存定義一覧から追加項目を生成。

## Added

`TopologicalNode.boundary_evidence` のビット：

| 値 | 意味 |
| --- | --- |
| 0 | 証拠なし・原因不明 |
| 1 | 局所面の延長より手前に実測点のあるレイ。遮蔽の証拠 |
| 2 | 局所面の延長位置を通過し、その奥に実測点のあるレイ。自由空間の証拠 |
| 4 | 明示されたセンサ視野の端のセルでの観測 |

ビットの併存が可能。視野端であることと、別方向の遮蔽・自由空間の証拠は排他的ではない。
非候補ノードは0。0は境界不存在や自由空間不存在の証明ではない。

## Fixed

- 欠測セル・走査線の隙間・視野情報のない点群端を、自由空間として扱わない仕様。
- 全周LiDARのyaw継ぎ目を視野端として扱わない周期接続。
- 傾斜面の距離変化と、本当の深度段差の混同を抑える局所接平面補正。
- フレームごとの証拠再計算。古いCameraInfo・原点による前フレームの証拠継承なし。

## Removed

既存トピック・API・低次数判定の削除なし。追加トピック・GNGライブラリ公開関数の追加なし。

## Behavior Impact

処理順序：

1. 同一学習フレームの観測原点と姿勢を確認。情報不足・複数センサ入力は不明。
2. 同時刻CameraInfoの校正済みピンホール視野、または明示的なLiDAR角度範囲を使用。
3. 実入力の変換済みXYZをセンサ座標に戻し、観測セルへ登録。同一セルは最短リターンのみ。
4. 候補と同じセルの実測点が対応距離内にある場合だけ追加検査。
5. 外周セルを視野端として記録。全周yawには外周なし。
6. 周囲8セルに実測点があれば、その実測レイと候補の局所接平面の交点を比較。

接平面は同一セルの実測点 `p` とGNGノード法線 `n`、単位実測レイを `d` として、期待距離 `t=(n・p)/(n・d)`。
実測距離 `r` と `t` の差が誤差余裕を超える場合だけ証拠候補。
法線欠落、掠れ角（`|n・d| < 0.2`）、負の交差距離では反証なし。
周辺に局所面と整合する非平行な2方向の実測支持がない場合も、遮蔽・自由空間は不明。
視野端の証拠には法線は非必須。

Viewerの一覧は「境界候補」1項目。原因別の表示フィルタ・色設定は既定で折りたたみ。
新規設定の原因別フィルタは全ON。既存設定のON/OFFは保持。
境界候補とHANDLEの優先順位は一覧の↑↓で変更。境界候補OFF時は原因別表示も一括無効。
詳細は `2026-09-09_viewer_boundary_label_group.md` を参照。

## Topics / Params / Messages

- ROSの `TopologicalNode` に `uint8 boundary_evidence` と3定数を追加。
- WebSocket JSONは同名属性。TMG1は既存84バイトレコードの予約オフセット6を使用、サイズ・版は維持。
- 旧予約値0・JSON属性なしは原因不明。ROSは送受信側の同一定義での再ビルドが必要。
- 判定パラメータはYAMLによる起動時指定。

| パラメータ | 既定値 | 意味 |
| --- | --- | --- |
| `boundary.enable_evidence` | true | 境界候補の追加判定 |
| `boundary.min_range_gap_th` | 0.03 | 実測距離と期待距離の差の判定用余裕[m] |
| `boundary.max_anchor_dist` | 0.05 | 候補と同じ観測セル内の実測点の対応距離[m] |
| `boundary.lidar_angles_deg` | 未指定（内部は空配列） | `[min_yaw, max_yaw, min_pitch, max_pitch, yaw_step, pitch_step]` [deg]。CameraInfo利用時はYAMLのキーを省略 |

観測設定は既存の `node.enable_observation_support`、`input.observation_sensor_frame`、または固定原点・姿勢を使用。
CameraInfo利用時は `input.observation_camera_info_topic` も必要。観測支持OFF・姿勢不明・視野不明では追加証拠なし。
LiDAR配列が空ならCameraInfo経路、非空ならLiDAR経路を優先。
LiDAR座標軸はx前方・y左方・z上方、カメラはoptical軸。固定姿勢設定の既存名は `input.observation_camera_rotation`。
LiDAR角度範囲は観測セルの下端包含・上端非包含。yawは連続区間（例170〜190度）、360度幅は周期接続。
数値は実センサの視野・走査間隔に合わせること。観測点のmin/maxからの自動設定なし。

## Verification

- C++単体テスト：連続面、遮蔽、自由空間、視野端との併存、欠測、単一走査線、傾斜面、実測点から離れた候補、法線欠落、無効入力、フレーム消去、全周継ぎ目、部分視野の検査成功。
- 合成20,000入力・5,000候補の初回測定は14.91 ms（他ビルドとの競合中）。処理時間上限や数ms以内の保証なし。
- コンテナ内Release版の同一合成負荷は2.24 ms。原点・姿勢変換、CameraInfo表準備、ROS配信を含まない測定。
- ASan・UBSan検査成功。環境のptrace制約によりLeakSanitizerは無効化、リーク検査の成功主張なし。
- gng_cpuの既存3テスト（次数、角度範囲、観測API）も成功。
- フロントエンドの証拠ビット全8組み合わせ・旧データの不明扱い・優先色解決の検査成功。
- `npm run lint`、`npm run build -- --configLoader runner --outDir /tmp/topo-boundary-evidence.jMDYbO` 成功。既存の大きなバンドル警告あり。
- 隔離ROSドメイン193で、平面外周の視野端、深度段差の遮蔽・自由空間、CameraInfo欠落時の不明復帰を各5フレーム確認。
- このROS検査では検出経路の検証用に全ノードを候補化。実運用の次数4判定とは別の設定。
- `ais_gng_msgs`・`gng_cpu`・`ais_gng`・`topo_fuzzy_viewer` の分離Releaseビルド成功。Viewerの既存未使用変数等の警告あり。
- `ctest` による観測画素・境界証拠の2テスト成功。
- 隔離ROSドメイン189で、候補ON（次数4・次数0）とOFFを各ROS 8フレーム・WebSocket 3フレーム確認。入力停止中の学習出力増加なし。
- 転送検査の既存スクリプトにもブラウザと同じ受信完了通知を追加。未知証拠0のROS→WebSocket転送を確認。非ゼロの全組み合わせはフロントエンドの人工パケット検査、非ゼロの判定出力は前記ROS検査で確認。

### 実行コマンドと停止確認

ホストから実行したROS証拠検査：

```bash
docker exec gng_cpu_container bash -lc '
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /tmp/gng-boundary-evidence.42Jm5J/install/ais_gng_msgs/share/ais_gng_msgs/local_setup.bash
source /tmp/gng-boundary-evidence.42Jm5J/install/gng_cpu/share/gng_cpu/local_setup.bash
source /tmp/gng-boundary-evidence.42Jm5J/install/ais_gng/share/ais_gng/local_setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/torch/lib:$LD_LIBRARY_PATH
timeout --signal=INT --kill-after=10s 85s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/boundary_evidence_ros_test.py \
  --executable /tmp/gng-boundary-evidence.42Jm5J/install/ais_gng/lib/ais_gng/ais_gng_cpu
'
```

Viewer転送検査は上記と同じセットアップに次のViewer環境を追加し、Python呼出しを置換：

```bash
source /tmp/gng-boundary-evidence.42Jm5J/install/topo_fuzzy_viewer/share/topo_fuzzy_viewer/local_setup.bash
timeout --signal=INT --kill-after=10s 85s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/boundary_candidates_ros_test.py \
  --executable /tmp/gng-boundary-evidence.42Jm5J/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --gateway-executable /tmp/gng-boundary-evidence.42Jm5J/install/topo_fuzzy_viewer/lib/topo_fuzzy_viewer/viewer_ws_gateway_node
```

起動パラメータ一式は検査スクリプト内の `command`・`gateway_command` に記載、実行時にも標準出力へ記録。
検査用GNG・Viewerは `finally` による停止済み。既存ROSノード・bag再生の停止操作なし。
分離ビルド成果物・ログはコンテナの `/tmp/gng-boundary-evidence.42Jm5J` に保存。

## Risk / Notes

- このビットは候補近傍の局所的な証拠。物体の真の端の確定でも、テンプレート全体の棄却条件でもない。
- 比較方向・交点そのものは今回配信しないため、ビットだけをマッチングの反証に転用することは不可。照合スコアは今回未変更。
- 法線誤差や曲面では局所接平面仮定が崩れる可能性あり。2方向の実測支持は抑制条件であり保証ではない。
- 間引き後の実入力のみを利用。観測セル不足では検出漏れが増えるが、不明セルの補間なし。
- 視野外ノードや、同一セルに実測点のない候補は不明。視野端は外周1セル内での近接証拠。
- 不規則走査・移動歪み・マルチセンサ統合の精密な観測モデルは対象外。単一視点・明示モデルが前提。
- 全入力の索引構築が1回必要。低次数判定だけより計算量・メモリは増加。
- 既存の未コミット変更を保持。通常のinstallや実行プロセスへの直接反映は未実施。
