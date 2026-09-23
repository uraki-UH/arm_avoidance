# 2026-09-16 - L0の元姿勢割合による色表示

## Summary

簡略版L0の色と安全判定用labelを分離。元姿勢の安全・危険・衝突／使用不可の件数割合による混色。

## Changed

- 通常ブリッジの既存状態集約ループで3種類の件数を収集。新しい近傍探索や形状再構築なし。
- Viewerのノード詳細に件数と割合を追加。座標・labelが不変でも件数変更で表示更新。
- 保存形式を`VIZGNG6`・`VIZGST2`へ更新。従来の`VIZGNG5`・`VIZGST1`は件数0で読込可能。
- `ToPoDualArm10000/vis_gng_L0.bin`と`vis_gng_static_L0.bin`を新形式で再生成。

## Added

- `TopologicalNode`の`num_safe_states`・`num_danger_states`・`num_collision_states`。各`uint32`。
- `TMG1`バイナリversion 2。node末尾のoffset 84・88・92に上記件数、record長96バイト。
- 件数集約・旧形式互換・軌道色保持・ROS→WS配信・表示色更新の回帰テスト。

## Fixed

- ほぼ全姿勢が衝突していてもsafeが1件あるだけで純緑に見えるL0表示。判定用labelの変更なし。

## Removed

- トピック・パラメータの削除なし。

## Behavior Impact

- `/ToPoDualArm/Tmap_vis_L0`は元姿勢の現在の状態件数を反映。
- `/ToPoDualArm/Tmap_vis_static_L0`は保存時点の件数。今回の保存データは安全10,801・危険0・衝突0なので静的版は緑のまま。
- safe・danger・collisionのパレット色を線形RGBで混合。単色・目標・semantic・境界などの明示的色指定は優先。
- 件数未収録・不正値・合計0の場合は従来色。元Tmapや入力軌道の判定・配色は維持。
- 件数割合は学習済み姿勢の構成比であり、成功確率・関節空間の体積割合・代表姿勢の安全保証ではない。

## Topics / Params / Messages

既存トピックを継続。追加launch引数・パラメータ・トピックなし。
ROSメッセージ定義変更のため`ais_gng_msgs`と依存パッケージの再ビルド、送受信ノードの再起動が必要。
Viewerも新Frontendへ更新してページを再読み込み。外側のWSプロトコルはv2のまま。
新Frontendは旧TMG1 version 1も読込可能だが、旧Frontendは新version 2に非対応。

## Verification

コンテナ内のビルド・C++テスト:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --packages-above ais_gng_msgs --executor sequential --symlink-install
ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1 /ros2_ws/build/gng_vlut_system/test_visualization_gng
/ros2_ws/build/topo_fuzzy_viewer/test_nonplane_graph
```

初回ビルドは追加テストで`GNG::Status`のヘッダー依存不足により失敗。
明示include追加後、`colcon build --packages-select gng_vlut_system --symlink-install`が成功。
依存する7パッケージすべてのビルド完了。GNG8件、Viewerバックエンド6件のテスト成功。

ホストの`ToPoFuzzy-Viewer/frontend`で実行:

```bash
node --test tests/l0_state_colors.test.mjs tests/cluster_graph.test.mjs tests/boundary_evidence.test.mjs
npm run lint
l0_build_dir=$(mktemp -d /tmp/l0-viewer-build.XXXXXX)
npm run build -- --configLoader runner --outDir "$l0_build_dir"
```

3テスト・lint・本番ビルド成功。通常の`npm run build`は既存`node_modules/.vite-temp`の所有者が
`nobody`で書込不可のため失敗し、所有権を変更せずrunnerと一時出力先で再検証。
safe1・collision99の赤寄り表示、件数反転時の緑寄り更新、旧v1読込、目標単色保持を確認。
実ブラウザ・実GPUでの表示操作は未検証。

### モデル生成とROS検証の起動コマンド

以下は`gng_cpu`コンテナ内で実行。生成・単体テスト・検証スクリプトは終了、起動したROSノードは全停止済み。
既存のユーザーノードの停止／再起動なし。

```bash
ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1 ros2 run gng_vlut_system visualization_gng_trainer \
  --input /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin \
  --output-prefix /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng \
  --layer 0 --target-nodes 150 --iterations 200000 --seed 42 \
  --joint-motion-weight 0 --workspace-motion-sec-per-m 1.0 --workspace-sample-resolution 0.05 \
  --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

python3 /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_l0_state_stream.py \
  /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng_static_L0.bin
```

生成結果は150ノード・740エッジ・連結成分1・孤立0、所要7.44秒。元`gng.bin`・`vlut.bin`のSHA-256不変。
検証スクリプトはドメイン225・ポート19097で次の子プロセスとrclpy検証ノードを起動し、終了時に停止。

```bash
/ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/visualization_gng_static_node \
  --ros-args \
  -p model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng_static_L0.bin \
  -p topic_name:=/l0_test/Tmap_static -p frame_id:=ToPoDualArm/base_link
/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node --ros-args -p port:=19097
```

静的150ノードの件数をROS受信し、WSバイナリで一致を確認。
合成ノードの座標・label・時刻を固定したまま、件数`(1,4,95)`→`(1,59,40)`の更新受信に成功。
動的ブリッジの集約はC++単体テスト、描画色はFrontendテストで別途検証。

## Risk / Notes

- 古いメッセージ定義で起動中のノードには適用されない。再ビルド済みでも再起動が必要。
- 旧モデルは同一フォルダの`vis_gng_L0_v5_20260916.bin`・`vis_gng_static_L0_v1_20260916.bin`へ退避済み。
  以前の`vis_gng_L0_v2_20260916.bin`も保持。
- 新モデルは旧バイナリでは読込不可。コードを戻す場合は対応するモデルもバックアップから復元。
- [現行仕様](../TECHNICAL_SPEC.md#134-rosパラメータとトピック)、[WS形式](../../../ToPoFuzzy-Viewer/common/ws_protocol_v2.md#graph-stream)。
