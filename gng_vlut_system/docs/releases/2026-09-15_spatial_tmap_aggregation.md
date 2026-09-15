# 2026-09-15 - Tmap L0の空間集約修正

## Summary

`Tmap_vis_L0`を元GNGの空間集約として生成。関節状態による所属の偏り、代表姿勢への描画位置の
置換、FK補間によるedge上書きを解消。10,801元ノードから150ノード・740エッジへ再生成済み。

## Changed

- 既定の`joint_motion_weight`を0へ変更。正値を明示した場合だけ関節移動時間も所属判定へ追加。
- 描画位置を所属元ノードの手先位置の重心へ変更。実在する代表関節角・元IDとの分離。
- 元coord-space edgeの所属先間への縮約を維持。自己ループ・重複のみ除外。
- 共有trainerを使う到達可能ボクセル側にも、元ボクセルの空間隣接edgeを入力。

## Added

- 空間所属の関節角非依存性、重心、元接続の縮約一致、FK補間の非干渉、保存再読込、負の重み拒否のテスト。

## Fixed

- 空間集約edgeがFK補間結果で置き換わり、近傍本数制限で元接続が消失する問題。
- 空間的に離れた姿勢が同一グループとなり、1個の代表TCPへ表示が偏る既定動作。

## Removed

- 両trainerの`--edge-max-neighbors`と`max_edge_neighbors`。元接続の本数切り捨てを廃止。

## Behavior Impact

launch・トピック名は変更なし。既存binの自動再学習・自動再読込はないため、別環境では
trainerによる再生成後にbridgeを起動。今回のworkspaceには更新済みbinを配置済み。
保存形式`VIZGNG5` / `VIZGST1`は変更なし。形式が同じ旧binも受理されるため、古い集約結果を
使い続けないよう再生成が必要。元`gng.bin`・`vlut.bin`は変更なし。

```bash
ros2 run gng_vlut_system visualization_gng_trainer \
  --input /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin \
  --target-nodes 150 --iterations 200000 --seed 42 \
  --joint-motion-weight 0 \
  --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

## Topics / Params / Messages

- `/ToPoDualArm/Tmap_static`、`/ToPoDualArm/Tmap_vis_L0`、既存軌道topicは継続。
- `ais_gng_msgs/msg/TopologicalMap`の変更なし。新しいROSノード・メッセージの追加なし。
- `transition_paths`は既存軌道表示用の独立メタ情報として継続。空間edgeの構成には不使用。
- 現行仕様の正本: [TECHNICAL_SPEC.md 13章](../TECHNICAL_SPEC.md#13-可視化専用3次元gng)。

## Verification

Docker Releaseビルド、GTest 5件、同じlaunchによる隔離ROS配信とViewer Gateway受信に成功。
保存した座標・edgeとROS本文の一致、全元ノードの一意所属、frameの一致を確認。
受信した元`Tmap_static`から独立に計算した縮約edgeとの差分は欠落0本・余分0本。
接続集計の初回は検証スクリプトがedgeの配列添字をIDとして扱い失敗。メッセージ仕様に合わせて修正後に再測定。

| 測定対象 | 修正前 | 修正後 |
| --- | ---: | ---: |
| 集約ノード数 | 150 | 150 |
| edge数 | 554 | 740 |
| 元ノードから所属集約点までの距離RMS | 219.2 mm | 46.2 mm |
| 同距離の95 percentile | 370.0 mm | 67.7 mm |
| 連結成分数 / 孤立node数 | 1 / 0 | 1 / 0 |
| edge長中央値 | 87.7 mm | 107.0 mm |
| edge長最大値 | 243.7 mm | 557.4 mm |

元接続を保持するため長いedgeも残存。edge長の短縮を達成したという結果ではない。
今回のtrainer単発経過時間は4.14秒。連続性能ベンチマークではない。
現在のViewer `GraphRenderer`で同じ視点・倍率の前後比較を3方向から描画し、画像を目視確認。
専用Chromeによるbin描画の検証であり、ユーザーの既存ブラウザ画面は操作していない。

結果・画像・再現スクリプト・旧binの保存先: `tmp/tmap_spatial_20260915/`。
`result.json`はROS/WS、`quality.json`は品質比較、`compare_1.png`〜`compare_3.png`は前後比較。
生成binはGit管理外。別checkoutへのコード反映だけではデータ更新にならない。

### 実行したコマンド

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 240s cmake --build /ros2_ws/build/gng_vlut_system --target visualization_gng_trainer topofuzzy_bridge_node visualization_gng_static_node reachability_voxel_visualization_gng_trainer -j2'

docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && cmake -S /ros2_ws/src/gng_vlut_system -B /ros2_ws/build/gng_vlut_system > /ros2_ws/src/tmp/tmap_spatial_20260915/cmake.log 2>&1 && timeout -s INT -k 10s 120s cmake --build /ros2_ws/build/gng_vlut_system --target test_visualization_gng -j2 && timeout -s INT -k 5s 60s ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure -R "^test_visualization_gng$"'

docker exec -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/tmap_spatial_20260915_logs gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 120s nice -n 10 /ros2_ws/build/gng_vlut_system/src/visualization_gng_trainer --input /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin --output-prefix /ros2_ws/src/tmp/tmap_spatial_20260915/generated/vis_gng --target-nodes 150 --iterations 200000 --seed 42 --joint-motion-weight 0 --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml > /ros2_ws/src/tmp/tmap_spatial_20260915/trainer.log 2>&1'

docker exec -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/tmap_spatial_20260915_logs gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 30s 100s python3 /ros2_ws/src/tmp/tmap_spatial_20260915/check_launch.py'

timeout -s INT -k 5s 60s node tmp/tmap_spatial_20260915/compare.mjs
node tmp/tmap_spatial_20260915/quality.mjs
```

検証スクリプト内の常駐プロセス起動コマンド:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node --ros-args -r __node:=tmap_spatial_gateway_check -p port:=19096
/opt/google/chrome/chrome --headless=new --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader --no-first-run --no-default-browser-check --remote-debugging-pipe --user-data-dir=/tmp/tmap-spatial-browser-pvyKvM about:blank
```

検証launch PGID 1782414、Gateway 1782546、Chrome 2944349とその子プロセスは停止済み。
launch停止時の一部Python nodeのexit -2は検証後SIGINTによる終了。専用ROSログ・Chrome profileを削除。
コンテナ内に`ss`がなかったため、socket bindで19096の解放を確認。ROS daemonの新規残留なし。
コンテナ起動状態・既存Chromeは維持。開始時に存在した通常bridge PID 1780469とその子の退出を
作業中に観測したが、その原因は未調査。本作業から既存プロセスの停止・再起動操作は実施していない。

元データSHA256の前後一致:

- `gng.bin`: `cd3e45f0ad019110609c804a0d79e8725456d5a5f27622d2b3b87fbf5fedb53b`
- `vlut.bin`: `35c1c42110301cc1293c124156d92f3ebbc2d31f1895aa3ff8c4c4936dfe3f26`

## Risk / Notes

- 重心は描画用の空間要約であり、代表関節角のFK結果ではない。IK目標・無衝突経路としての利用は別途検証が必要。
- 空間所属集合の内部連結性は強制しない。集約edgeは元接続の存在を表すが、曲面メッシュの滑らかさは保証しない。
- 元angle edgeの補間列は引き続き離散近傍への対応であり、連続FK軌跡の再構成ではない。
- 共有の到達可能ボクセルtrainerはビルド確認のみ。既存の到達可能ボクセルbinの再生成・置換は未実施。
