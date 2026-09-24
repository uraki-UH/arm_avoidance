# 2026-09-14 - 候補のバウンディングボックス選択

## 1. 要約

Topo Fuzzy Viewerの候補選択を、個々のノードの球ではなく物体全体のバウンディングボックスとの交差へ変更。

以下は導入時の記録。現在は[独立表示のBbox連動](2026-09-15_inspection_bbox_gate.md)により、直接クリックもBboxに従属し、`/grasp_pose_cands/Tmap`だけ既定ON。

- ホバー・クリックともに初回から箱との交差判定。ノード間の空隙も選択対象。
- 可視ソースの候補範囲を既存バックエンドで一括計算。受信データ更新時だけ再取得し、全体4 Hz・同時1件に制限。
- ホバー枠・枠内クリック・範囲取得は、トピック別Graph表示設定`enable_bounding_box=true`だけを許可。既定はすべてOFF。法線などのMarkerには設定・判定なし。ノード・エッジ描画と既存の直接クリックによる詳細選択は維持。
- 描画と当たり判定に同じTF・手動表示変換・余白を適用。重なる箱はカメラから近い候補を優先。

- 一括範囲取得とAABBホバー・クリックの回帰テスト。
- 枠の境界での一時的な判定外れに350msの解除猶予。
- Graphの設定欄に`Bounding Box`トグル。全OFF時は判定タイマーも停止。

- ノードの隙間や同じ物体内のノード切替による枠の点滅。
- 更新要求中の枠消去と遅延応答による退出後の再表示。
- `grasp_plane`と`grasp_nonplane`の到着順による代表選択の変化。両方存在する場合は平面側の選択に統一。

**削除**

- ホバーの点・球メッシュへのraycast。既存ノード詳細クリックの削除なし。
- トピック名・prefixの許可リスト。把持候補を含め、名前からの自動ONなし。
- Marker側の`Bounding Box`ボタン・設定・ホバー判定経路。Marker本体や法線の描画は維持。

## 2. 条件・検証

- キャンバス退出・ドラッグ・編集モード・購読解除では即時解除。5pxを超えるドラッグは候補クリックとして扱わない。
- 明示的なクラスタ・非平面component・SPHERE_LISTを物体単位として利用。所属のない単一ノードからの物体推測なし。
- `/grasp_pose_cands/Tmap`を使う場合も、そのトピックの`Bounding Box`を明示的にON。非平面成分など任意名のグラフも、必要なものだけ同様にON。OFF直後のクリック・遅延応答では再表示なし。
- トピック別フラグは既存表示設定と同じViewer内の状態。再読み込み後はOFFから開始。
- 初期の一括取得機能導入にはバックエンド更新も必要。今回のフラグ化だけならfrontend更新のみで適用可能。既存Viewerの自動再起動なし。

- ROS topic・launch引数・ROSメッセージの変更なし。
- `enable_bounding_box`: Viewer内のグラフトピック別フラグ。既定false、未指定もOFF。ROSパラメータではない。
- 既存`edit.inspect_graph`で`enable_bounds_only=true`かつ`selection`省略時に`{ bounds: [...] }`を返却。選択指定時の既存形式は維持。
- 現行契約は[BACKEND_API](../../../ToPoFuzzy-Viewer/doc/BACKEND_API.md#候補の独立表示)と[WS protocol v2](../../../ToPoFuzzy-Viewer/common/ws_protocol_v2.md#読取専用の候補切り出し)を参照。

- Docker Releaseビルド、候補抽出C++テスト7件、frontendのhover・既存Markerテスト、lint、buildに成功。
- 回帰テストで空隙からの初回選択・クリック、近い箱の優先、TFと手動変換、更新中の保持、取得失敗後の再試行、ドラッグと退出を確認。メッシュraycastの呼出しはテスト側で失敗扱い。
- 静的ロボットマップ除外後、`npm run test:hover`と`./node_modules/.bin/tsc -p tsconfig.app.json --noEmit --incremental false`を再実行して成功。除外ソースの範囲取得なしと他ソースの既存動作を確認。両コマンド終了済み、ROS起動なし。
- 把持候補配下への対象限定後、`npm run test:hover`、`npm run lint`、`npm run build -- --configLoader runner --outDir /tmp/codex-grasp-only-hover-dist`に成功。対象外だけの表示時に取得・枠・クリック選択が発生しないことを確認。全コマンド終了済み、一時build出力を削除。backendは未変更で、別途進行中の全体colcon buildとの競合回避のため再ビルドを重ねず維持。
- トピック別フラグ化後、`npm run test:hover`の3設定ケース（把持候補Graph、非平面成分Graph、任意名Marker）、`npm run lint`、`npm run build -- --configLoader runner --outDir /tmp/codex-topic-bounding-box-dist`に成功。既定OFF、明示ON、OFF時の解除・遅延応答破棄、再ON、TF・手動変換を確認。Dockerの`colcon build --packages-select topo_fuzzy_viewer --symlink-install --parallel-workers 1`にも成功。コマンドは全終了済み、ROSノードの起動・停止なし。
- Graph限定後、`npm run test:hover`、`npm run test:markers`、`npm run lint`、`npm run build -- --configLoader runner --outDir /tmp/codex-graph-only-bounds-dist`に成功。グラフの明示ON/OFFと既存Marker描画を確認。検証コマンドは全終了済み、ROSノード・サーバーの起動なし。backendは未変更。
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

**制約**

- ブラウザでの実画面・GPU操作確認と大規模実入力での負荷計測は未実施。frontendテストは実THREE・React Three Fiberと模擬rendererを使用。
- 箱内の空白も選択範囲となるため、凹形状の実表面との厳密な一致は対象外。
- frontend buildには従来の大きいbundleに関する警告あり。生成した一時build出力を削除し、TypeScript生成キャッシュの変更を除去。
