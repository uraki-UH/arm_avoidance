# 矢印表示・送信側の共通化

## 変更内容

把持候補、PoseArray、ROS Marker、法線、クラスタ速度、クラスタ詳細の矢印を共通化。
`arrows/geometry.ts`で位置基準・実寸・姿勢・色を解釈し、`ArrowBatch`で円柱と円錐を描画。
姿勢配列・標準Marker・法線は一括描画。既存の独立したArrowHelper・法線専用円錐生成を削除。
追加整理で未使用`NormalVectorRenderer`と`DirectionalArrow`ラッパーも削除。
Viewerの矢印処理は`visualization/arrows/`の5ファイルへ集約。把持専用の姿勢矢印生成を`arrow_visualization::make_pose_arrows`へ移管。
クラスタ速度の二重位置変換も修正。

- 共通設定: 色、状態別配色、不透明度、全長、軸直径、矢先長・直径、前面表示、根元／矢先／中点、主軸、補助2軸。
- 補助軸: 完全な姿勢入力の場合のみ。法線・速度・始点終点Markerから任意の補助軸を生成する処理はなし。
- 設定保存: ブラウザのレイヤー別localStorage。ROSやWS更新ごとの表示設定送信は不要。
- 姿勢配信: 位置・完全なクォータニオン・状態。候補IDとupdate_idを維持。
- 標準Marker配信: 色・scaleの共有辞書。初回・変更時・再接続時だけ辞書を送信。
- ROS生成: 新規ヘッダー専用パッケージ`arrow_visualization`。平面法線・速度・姿勢ブリッジから同じ`make_arrow`を使用。
- スコア: `/grasp_pose_cand_scores`と`score_topic`を廃止し、`GraspCandidateArray.candidates[].shape_score`へ統一。

描画専用のROSメッセージは追加していない。ID・スコア・到達状態を持つ把持候補は`GraspCandidateArray`、
汎用姿勢は`PoseArray`、RViz向け描画は`MarkerArray`を使用。
ROS MarkerのROS区間は標準型の都合で色・寸法を含み、WS区間で重複を圧縮。
通常の把持候補は意味データを直接表示するため、ROS区間にも描画設定を含まない。

操作・座標規約・ワイヤ形式: [矢印共通仕様](../../../ToPoFuzzy-Viewer/common/arrow_visual_spec.md)。
既存の状態判定コスト計測タスクは`TASK_LIST.md`の2.3に維持。

## 検証

以下は実装時の実行記録。ユーザー指示により矢印専用テスト6ファイルと登録設定は削除済み。
記載の削除済みスクリプトは現在の実行手順ではなく、当時の検証記録。

- ROSビルド: `arrow_visualization`、`grasping_system`、`ais_gng`、`gng_vlut_system`。依存の`gng_control_msgs`、`ais_gng_msgs`、`pointcloud_sampling`も更新して成功。
- Viewer backend: `colcon build --packages-select topo_fuzzy_viewer --symlink-install --cmake-args -DBUILD_TESTING=ON`成功。
- Viewer frontend: `npm run lint`、`npm run build`、`npm run test:markers`成功。
- C++テスト: 共通Marker生成3件、矢印プロトコル2件成功。
- 既存テスト: `test_grasp_candidate_publisher`、`test_top_grasp_surface_estimator`実行成功。
- 描画テスト: 基準位置3種、主軸回転・補助軸直交性、180度反転、TF、寸法、色、不透明度、奥行き、空入力、無効姿勢、辞書参照、1,000候補および1,000標準Markerの2メッシュ一括描画、始点終点Markerのpose変換を確認。
- ROS→WS: 候補状態更新、空配列、汎用PoseArray、設定辞書の共有・送信省略・更新・再接続復元、購読解除を確認。
- 把持launch: 既定出力・個別トピック指定・座標系指定、到達状態更新、複数ID、TF欠落時の消去を確認。

ビルドには既存のC++未使用引数・PCL/Torch設定警告とViteチャンクサイズ警告あり。
既存Viewerや既存ROSノードの停止・再起動は未実施。新しいgatewayを通常運用に反映する際は更新後のバイナリで起動が必要。

## CPU描画準備コスト

frontendコンテナ、Node.js v20.20.2。8反復の先頭2反復を除いた6回のソート後中央側値。
`build_arrow_parts`の行列・色生成と配列化を計測。別ビルド等の負荷による変動を含む参考値。

| 候補数 | 主矢印のみ [ms] | 補助2軸付き [ms] |
| --- | ---: | ---: |
| 1,000 | 3.07 | 8.35 |
| 10,000 | 18.00 | 49.22 |

GPU描画時間、ブラウザ全体のフレーム時間、ROS到達性判定時間とは別の計測。
実機GPU計測は`TASK_LIST.md`のFに継続。1バッチは円柱・円錐の2メッシュ、補助2軸付きの場合も同じ構成。
標準Markerと姿勢配列は座標系・設定ごと、法線はレイヤーごとの一括描画。

## 起動した検証プロセスと停止

以下の検証用プロセスはすべて終了済み。結合テストのfinallyで子ノードも停止。

```bash
node ToPoFuzzy-Viewer/frontend/tests/check_pose_array_stream.cjs
```

上記からコンテナ内で起動したコマンド:

```bash
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15 100 python3 /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/pose_array_stream_fixture.py
/ros2_ws/src/ToPoFuzzy-Viewer/backend/install/topo_fuzzy_viewer/lib/topo_fuzzy_viewer/viewer_ws_gateway_node --ros-args -p port:=19091 -r __node:=pose_array_stream_test_gateway
```

候補トピック検証コマンド:

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15 110 python3 /ros2_ws/src/grasping_system/test/check_top_grasp_topic_integration.py
ros2 launch grasping_system top_grasp_surface_estimator.launch.py params_file:=/tmp/top_grasp_topic_contract_<検証ごとのID>/params.yaml
```

個別出力ケースでは上記launchに`candidate_topic:=/topic_contract/grasp_pose_cands`、
`candidate_nodes_topic:=/topic_contract/grasp_pose_cands/nodes`、`summary_topic:=/topic_contract/grasp_pose_cands/summary`を追加。
共通publisherの回帰テストは`/ros2_ws/build/grasping_system/test_grasp_candidate_publisher`から有限時間のノードを起動し、終了済み。
