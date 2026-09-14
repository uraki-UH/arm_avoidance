# 2026-09-14 - 候補のバウンディングボックス選択

## Summary

Topo Fuzzy Viewerの候補選択を、個々のノードの球ではなく物体全体のバウンディングボックスとの交差へ変更。

## Changed

- ホバー・クリックともに初回から箱との交差判定。ノード間の空隙も選択対象。
- 可視ソースの候補範囲を既存バックエンドで一括計算。受信データ更新時だけ再取得し、全体4 Hz・同時1件に制限。
- 描画と当たり判定に同じTF・手動表示変換・余白を適用。重なる箱はカメラから近い候補を優先。

## Added

- 一括範囲取得とAABBホバー・クリックの回帰テスト。
- 枠の境界での一時的な判定外れに350msの解除猶予。

## Fixed

- ノードの隙間や同じ物体内のノード切替による枠の点滅。
- 更新要求中の枠消去と遅延応答による退出後の再表示。
- `grasp_plane`と`grasp_nonplane`の到着順による代表選択の変化。両方存在する場合は平面側の選択に統一。

## Removed

- ホバーの点・球メッシュへのraycast。既存ノード詳細クリックの削除なし。

## Behavior Impact

- キャンバス退出・ドラッグ・編集モード・購読解除では即時解除。5pxを超えるドラッグは候補クリックとして扱わない。
- 明示的なクラスタ・非平面component・SPHERE_LISTを物体単位として利用。所属のない単一ノードからの物体推測なし。
- バックエンドの再起動と更新版frontendが必要。作業中の既存Viewerは再起動せず維持。

## Topics / Params / Messages

- ROS topic・launch引数・ROSメッセージの変更なし。
- 既存`edit.inspect_graph`で`enable_bounds_only=true`かつ`selection`省略時に`{ bounds: [...] }`を返却。選択指定時の既存形式は維持。
- 現行契約は[BACKEND_API](../../../ToPoFuzzy-Viewer/doc/BACKEND_API.md#候補の独立表示)と[WS protocol v2](../../../ToPoFuzzy-Viewer/common/ws_protocol_v2.md#読取専用の候補切り出し)を参照。

## Verification

- Docker Releaseビルド、候補抽出C++テスト7件、frontendのhover・既存Markerテスト、lint、buildに成功。
- 回帰テストで空隙からの初回選択・クリック、近い箱の優先、TFと手動変換、更新中の保持、取得失敗後の再試行、ドラッグと退出を確認。メッシュraycastの呼出しはテスト側で失敗扱い。
- 初回テストの一時出力先`node_modules`は権限不足。テスト配下の一時ディレクトリへ変更後に再実行成功。
- ROS domain 226の内部RPCで候補部品の合算AABBと通常詳細取得の一致を確認。検証用ノード・プローブは終了済み。既存ROSのPID・コンテナ状態を維持、ROS daemonの新規残留なし。

実行コマンド（すべて終了済み）:

```bash
docker exec gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
colcon build --packages-select topo_fuzzy_viewer --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
'
docker exec gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
/ros2_ws/build/topo_fuzzy_viewer/test_graph_inspection
'
cd ToPoFuzzy-Viewer/frontend
npm run test:hover
node tests/candidate_hover_frame.test.mjs
npm run test:markers
npm run lint
npm run build -- --configLoader runner --outDir /tmp/codex-hover-box-dist
```

ROS検証の起動コマンド（一時スクリプトは終了後削除）:

```bash
docker exec -i gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15s 60s python3 -u -
' < /tmp/codex-hover-box-rpc.py
```

スクリプト内で同じ環境の`ros2 run topo_fuzzy_viewer viewer_edit_node`を起動。`finally`で自分のプロセスグループだけにSIGINTを送り、終了を確認。

## Risk / Notes

- ブラウザでの実画面・GPU操作確認と大規模実入力での負荷計測は未実施。frontendテストは実THREE・React Three Fiberと模擬rendererを使用。
- 箱内の空白も選択範囲となるため、凹形状の実表面との厳密な一致は対象外。
- frontend buildには従来の大きいbundleに関する警告あり。生成した一時build出力を削除し、TypeScript生成キャッシュの変更を除去。
