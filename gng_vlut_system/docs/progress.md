# 実施済み作業の記録

実際に行った変更・調査・検証だけの記録。予定は [TASK_LIST.md](TASK_LIST.md)、方針未確定の課題は [TASK_CANDIDATES.md](TASK_CANDIDATES.md)、不採用判断は [reject.md](reject.md) に分離。
時間・依存作業などによる保留作業の状態は [pending.md](pending.md) に分離。
記録単位は「日付 / 対象 / 実施内容 / 結果・検証範囲 / 根拠へのリンク」。既存履歴の一括転記なし。

## 2026-09-15: 把持アテンションの物体候補AABB化

- ユーザー指定によりノード半径方式をクラスタ別AABB＋余白へ置換。`radius`を`margin`へ変更し、既定OFF・配分率・失効条件・GNGコアを維持。[仕様・実行コマンド](releases/2026-09-15_grasp_attention_aabb.md)を記録。
- Dockerビルド、AABB単体テスト3件、既存GNG APIテスト2対象、隔離ROSの候補内部点・別候補間・所属なし・失効・TF検証に成功。10候補・5000ノード・10万入力点のAABB構築＋抽出は合成データ単回で1.42386 ms。
- 検証用CPU・Pythonノードは終了、既存CPU/ViewerのPIDを維持。実環境での把持成功率・最適余白は未検証。

## 2026-09-15: 把持候補近傍のGNG重点学習

- CPU GNGへ既定OFFの重点入力APIとROS候補購読を追加。総学習回数内での配分、候補失効・TF失敗時の通常処理、重点分の観測統計への非計上を実装。[仕様・起動方法](../../ais_gng_cpu/docs/grasp_attention.md)と[検証コマンド・結果](releases/2026-09-15_grasp_attention.md)を記録。
- Docker Releaseビルド、CTest 3対象、隔離ROSでのON/OFF・TF・時刻・空候補・受信停止の検証に成功。初回ビルドのTF参照範囲とテストの設定順序を修正後に成功。
- 5000候補中心・10万点の近傍検索を合成データで測定。空間ハッシュ43.8318 msからkd-tree20.3293 msへ変更、選択点42178点で一致。重点50%時の通常勝者イベント500回、共分散・観測方向件数への重点分の混入なしを確認。
- 検証用ROS domain 219のCPU・Pythonノードは終了、既存CPU/ViewerのPIDを維持。把持推定器の外部再起動を観測したが本作業からの停止なし。実物把持の成功率改善・実入力での最適設定は未検証。

## 2026-09-15: 非平面成分のViewer Graph化

追記: ユーザー指定により`/nonplane_components`のBbox既定設定を削除し、GUI・独立ビュー選択を対象外へ変更。Graph配信・法線・共分散と把持候補Tmapの既定ONは維持。関連テスト2ファイル・lint・frontend本番ビルド・Docker backendビルドに成功。実ブラウザ操作は未検証。有限コマンドは終了、一時出力は削除済み。ROSノード・サーバーの起動停止なし。

今回の検証コマンド（frontendディレクトリ）:

```bash
node --test tests/inspection_bbox_gate.test.mjs tests/candidate_hover_frame.test.mjs
npm run lint
npm run build -- --configLoader runner --outDir /tmp/nonplane-bbox-hidden-build
```

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 180s colcon build --packages-select topo_fuzzy_viewer --symlink-install --parallel-workers 1'
```

- ROS所属配列を維持し、バックエンドの非平面Marker生成を既存Graphバイナリ配信へ置換。元ID・法線・共分散・成分所属・実エッジを保持し、平面側端点を成分のBboxから除外。[現行仕様・実行コマンド](releases/2026-09-15_nonplane_graph_viewer.md)を記録。
- `/nonplane_components`に既定OFFのBbox切替を追加。他トピックの未指定設定・把持候補の既定ONと既存の並行変更を維持。
- Docker backendビルド・CTest 2対象、隔離ROSでの入力全6到着順・再接続再購読・空成分・購読解除検証、frontend関連5ファイル・lint・本番ビルドに成功。初回のテスト側端点数・再購読漏れ・旧Bbox期待値と一時出力先権限を修正後に再検証成功。
- 実ブラウザの目視と実GNG入力の描画は未検証。専用ROS domain 218・port 19092のノードは終了、一時build出力は削除。既存Viewerの停止・再起動なし。CPU GNGの外部操作によるPID更新を観測したが、本作業からの停止操作なし。

## 2026-09-14: 作業記録の分類

- `progress.md` と `reject.md` を新設し、実施済み作業と不採用判断を分離。
- ローカルスキルと `AGENTS.md`、文書索引を更新。未確定指標への仮の計算式追加を避ける規則を明文化。
- 既存のタスク本文・過去のリリースノートの移動や削除なし。
- `quick_validate.py skills/maintain-project-docs` によるスキル形式検証に成功。

## 2026-09-14: 配信用の暫定評価指標

- リポジトリ内の参照調査で、関節余裕min/meanと推定エネルギー・時間が候補選定に未使用で、配信・表示用であることを確認。
- 4指標の暫定処理と専用パラメータを削除。既存NaN初期値と汎用評価の無効フラグを維持。
- 上方候補の面積比は順位付けに使用中であるため維持。採否条件・軌道選択ロジックの変更なし。
- 変更仕様は [リリースノート](releases/2026-09-14_provisional_candidate_metrics.md) を参照。
- Dockerの`gng_cpu_container`内で対象ノードと追加テストをビルド。テスト用includeパス・静的リンク順の不足を修正後、成功。
- 回帰テスト1件に成功。4指標のNaN・無効フラグ、候補ID・選択状態・姿勢・関節値・経路・位置可操作性の保持を確認。
- 実環境の把持・Viewer画面での検証は未実施。常駐ROSノードの新規起動・既存プロセスの停止なし。ビルド・テストプロセスは終了。

実行コマンド（Docker内、`source /ros2_ws/install/setup.bash` 後）:

```bash
cmake --build /ros2_ws/build/gng_vlut_system --target topological_map_avoidance_node -j2
cmake --build /ros2_ws/build/gng_vlut_system --target test_candidate_metric_availability -j2
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^test_candidate_metric_availability$' --output-on-failure
```

## 2026-09-14: 保留作業の記録先追加

- `pending.md`を新設し、保留理由・完了済み範囲・再開条件・次の一手の記録項目を整備。
- 作業記録スキル、`AGENTS.md`、文書索引と各台帳の案内を更新。保留と不採用の区別、再開時の移管・二重管理防止を明文化。
- 既存タスク本文と既存の不採用判断は維持。保留項目の自動登録・移動なし。ROSコード・設定の変更なし。

## 2026-09-14: ファジィルール実装の所在確認と設計書

- HTMLの既定ルール・所属関数・IF-THEN評価・JSON編集処理と、別用途の`FuzzyClassifier`を確認。
- ROS把持経路では指標配信と固定的な候補選択を確認。汎用IF-THENエンジンの把持候補選択への接続は見つからず。
- [ROSルールエンジンの実装雛形設計](designs/fuzzy_rule_engine_design.md)を追加。責務・入出力・設定形式・欠損値・ID対応・確認項目を記載。
- 実装・数値境界の確定・ROS起動・動作検証は未実施。既存コードとタスク順序の変更なし。

## 2026-09-14: 隣接平面クラスタの統合拒否調査

- 稼働中のCPU GNGから`/topological_map`と`/plane_clusters`を読み取り、同じ`frame_number`の5更新（9236〜9240）について隣接クラスタ対の統合条件を再計算。
- クラスタ7と10は法線差約5.4〜6.5度・接続4本で、面内広がり比・統合後残差上限・少数側残差の条件を通過。正規化残差約0.30に対して増加判定の許容値が約0.20となり、残差増加条件による拒否を確認。
- 根拠は[統合判定](../../ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp#L1248)。出力済みクラスタ対の再評価であり、内部の逐次統合全経路や物理的な同一平面性の検証ではない点に留意。
- [CPU起動処理](../../ais_gng_cpu/src/ais_gng/launch/ais_gng.launch.py#L167)で、`plane_params_file`から非平面成分設定だけを抽出し、平面クラスタの統合設定をCPUノードへ渡していないことを確認。
- 調査用ノード`plane_merge_readonly_probe`は終了し、終了後のプロセス一覧で残存なしを確認。既存のGNG・Viewer・ROS daemonの停止や再起動なし。ROSソースコード・設定変更、ビルド、統合条件変更後の検証は未実施。

調査ノードの実行コマンド（終了済み、調査スクリプトは一時ファイル）:

```bash
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 15s python3 -' < /tmp/plane_merge_probe.py
```

## 2026-09-14: 把持ファジールール設計資料の統合

- 入力設計・実装雛形・145件のルール候補・ROS入力指標候補の4資料を [統合設計書](designs/fuzzy_grasp_design.md) へ移管。旧4ファイルはリンク案内のみ。
- 現行実装調査から未実装の改善提案を統合先へ移管。実装説明・数値再現結果・過去のスライド・既存進捗は維持。
- `/grasp_pose_cands`の型、`/plane_clusters`の名称、独立スコアtopicの廃止、関節余裕・推定時間等の未計算状態をソース確認の範囲で整合。全topicの実受信確認なし。
- README・現状資料・評価メッセージ仕様・スライド生成元の参照先を更新。スライドのPowerPoint・PDF再生成は未実施。
- 移管前後の照合でルール145件の本文、入力表67行、指標・仮説等のID100件、JSON雛形1件の保持を確認。
- 文書リンク88件の解決、コードフェンス対応、スライド生成元のPython構文を検査。既存索引の無関係なリンク切れ1件は変更対象外。`git diff --check`に成功。
- ROSコード・設定・評価式・タスク順序の変更なし。常駐プロセスの新規起動・既存プロセスの停止なし。

## 2026-09-14: 疎な接続による平面分割の改善

- 実入力のクラスタ1と54で、幾何条件を通過する一方、直接接続1本のため統合候補から外れるケースを確認。1と19は少数側残差でも拒否となる別ケース。
- 接続1本のノイズ付き同一平面で回帰テスト失敗を確認後、統合用接続数とYAML転送を修正。変更範囲・互換性は[リリースノート](releases/2026-09-14_plane_merge_connections.md)を参照。
- DockerビルドとC++22件・launch2件のテストに成功。同一実入力列の隔離再生で分割数の減少を確認。全箇所の正解判定や修正後Viewerでの目視確認は未実施。
- 調査・比較ノードはすべて終了。プロセス一覧で既存GNG・Viewer・ROS daemonの維持とテストプロセス残存なしを確認。比較用一時スクリプトは終了後削除。

起動コマンド（すべて終了済み）:
```bash
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 15s python3 -' < /tmp/plane_merge_probe.py
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 120s python3 -' < /tmp/plane_merge_replay.py
```

## 2026-09-14: 不採用案の追加テスト削除

- ユーザー依頼により、[不採用の平面クラスタ改善案](reject.md)に追加したC++テスト1件、launchテスト2件、専用補助コードとCMake登録を削除。既存テスト・処理本体の変更なし。
- 削除対象のソース参照残存なしと`git diff --check`を確認。再ビルド・テスト実行・ROSプロセスの操作は未実施。

## 2026-09-14: 上方把持フローの順序修正

- [システム全体フロー](presentations/README.md#システム全体フロー上方把持の順序修正版)をDOT・SVG・PNGで作成。生成コマンドは同READMEに記載、生成処理は終了済み。
- 把持面の寸法確認と、候補姿勢生成後の付属ノード・対象寸法確認を分離。幾何的候補と関節姿勢探索の出力も分離。
- SVG上の主要接続16本と、未接続ファジー評価への入力矢印がないことを機械確認。PNGで文字と接続を目視確認、`git diff --check`に成功。
- ROSコード・設定変更、常駐プロセスの新規起動、既存プロセスの停止なし。

## 2026-09-14: 全体フローのスライド配置調整

- 全体フローのSVGを1920×1080の固定配置へ変更。上段の環境認識・把持候補生成、下段の照合・関節姿勢評価へ整理し、主要文字を26pxへ拡大。
- 元図の22ノード・22接続の保持とSVG・PNG寸法を機械確認。PNGで文字・矢印・背景区分の重なりを目視確認し、修正。
- SVGを配置の正本、DOTを接続の参照用として案内を更新。[PNG再生成コマンド](presentations/README.md#システム全体フロー上方把持の順序修正版)を実行し、生成処理は終了済み。`git diff --check`に成功。
- ROSコード・設定変更、常駐プロセスの新規起動、既存プロセスの停止なし。

## 2026-09-14: 候補ノードの独立3Dビュー

- Topo Fuzzy Viewerへ候補ノードのクリック選択と独立カメラの詳細表示を追加。切り出しは既存`viewer_edit_node`、描画は既存の一括描画を利用。単体HTML・把持推定処理への変更なし。[仕様・操作方法](releases/2026-09-14_candidate_inspection_view.md)を参照。
- DockerのReleaseビルド、追加C++テスト5件、frontend lint・ホスト側buildに成功。frontendコンテナ内buildは既存MCAP依存不足で失敗し、依存が揃っているホストで検証。依存追加なし。
- ROS domain 225とWS port 19001のダミー候補を専用Chromeで受信。ノードクリックから平面25点・非平面5点の表示、主画面と独立した回転・ズーム・平行移動、全体表示、固定保持、明示更新、ウィンドウ移動、ドラッグ誤選択の防止を操作・画像比較で確認。
- サイドバーを含む画面座標と3D領域座標の混同による初期配置のはみ出しを検出・修正。実ユーザー画面・大規模実環境での性能測定は未実施。
- 検証用ROS・HTTP・Chromeの全プロセス終了と専用3ポートのリスナー消滅を確認。既存ROS・frontendのPID、ROS daemon、コンテナ状態を維持。

検証起動コマンド（すべて終了済み、一時スクリプト使用）:
```bash
docker compose exec -T -w /ros2_ws gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout --signal=INT --kill-after=15s 620s python3 -u -' < /tmp/codex-candidate-inspection-ros.py
node /tmp/codex-inspection-browser.cjs
node /tmp/codex-inspection-ui-test.cjs
```

## 2026-09-14: 候補のホバー枠

- 候補ノード・クラスタへのホバー枠を追加。独立ビューと共通の所属解決・AABBを使用し、TF・手動表示変換を適用。[仕様](releases/2026-09-14_candidate_hover_frame.md)を参照。
- Docker Releaseビルド、候補抽出C++テスト6件、frontend lint・ホスト側buildに成功。検証によるTypeScript生成キャッシュの差分は除去。
- domain 225・WS port 19001と専用Chromeで、TF付き枠の描画位置・候補切替・枠消去・クリックとの両立・ドラッグ中の非表示・購読OFFでの取得停止を検証。範囲取得は2.1秒で6件。大規模実環境での負荷計測は未実施。
- 検証用ROS・HTTP・Chromeの終了、専用3ポートの消滅、既存ROSのPID維持を確認。開始前に停止状態だったfrontend・Viewerの起動なし。コンテナ状態を維持。

検証起動コマンド（すべて終了済み、一時スクリプト使用）:
```bash
docker compose exec -T -w /ros2_ws gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout --signal=INT --kill-after=15s 620s python3 -u -' < /tmp/codex-hover-ros.py
node /tmp/codex-hover-browser.cjs
node /tmp/codex-hover-test.cjs
```

## 2026-09-14: 把持候補経路生成と回避実行の分離

- `grasp_joint_candidates.launch.py`の起動先を経路生成専用ノードへ変更。共通処理の移動による実装共有と、追従・退避・近傍追加ペナルティを通らない計画更新を実装。[仕様・検証コマンド](releases/2026-09-14_candidate_path_planner.md)を記録し、RUN_GUIDEと現行仕様・ソース参照を更新。
- Docker ReleaseビルドとC++・既存到達性テストに成功。追加C++テストの補助グラフのインターフェース不足による初回ビルド失敗は、補完後の再ビルド・再実行で解消。
- domain 218で指定launchと実GNG入力を実行。専用ノードの起動、回避ノードとtrialサービスの不在、候補経路・評価・ロボットプレビューの配信、静止時の再探索抑制、明示要求・関節角度変更での更新、空候補時の旧出力消去、関節指令・control claim配信なしを確認。
- 最終ビルドで結合テストを再実行し成功。検証用プロセスはすべて停止済み。既存ROSのPIDとコンテナ状態を維持し、ROS daemonの新規残留なし。実環境でのCPU負荷測定は未実施。

## 2026-09-14: 付属抽出の不要な切替設定の削除

- 参照平面方式へ一本化し、付属抽出の2つの切替設定と旧成分所有者方式を削除。現行ToPoDualArm設定の動作を維持。[仕様・実行コマンド](releases/2026-09-14_fixed_reference_attachment.md)を記録し、資料・スライドを更新。
- Dockerビルド、単体テスト、domain 117のROS結合テスト3ケースに成功。テスト側の候補ID固定前提と未確定出力取りこぼしを修正し、全ケースを再確認。
- 検証用プロセスはすべて終了済み。既存ROSのPIDを維持し、稼働ノードの再起動なし。実機の把持成功率は未検証。

## 2026-09-14: 候補選択のバウンディングボックス化

- ユーザー指定により、初回ホバーとクリックの当たり判定をノードの球から物体全体のAABBへ変更。候補範囲の一括取得と更新中の保持を実装。[仕様・検証コマンド](releases/2026-09-14_candidate_bbox_picking.md)を記録。
- Docker Releaseビルド、C++テスト7件、frontend hover・Marker回帰テスト、lint・buildに成功。テスト一時出力先の権限不足は書込可能なテスト配下への変更で解消。
- domain 226のROS内部RPCで部品合算AABBと詳細取得の一致を確認。検証ノードは停止済みで既存ROSのPID・コンテナ状態を維持。実画面のGPU操作と大規模実入力の負荷計測は未実施。

## 2026-09-14: グラフトピック名のtmap短縮

- 環境入力`/topological_map`を維持し、関連トピックの既定値・launch・設定例を短縮。Viewerの経路判定を新旧名へ対応。[変更一覧・検証コマンド](releases/2026-09-14_tmap_topics.md)を記録。
- 関連ROSノードのDockerビルド、frontend型チェック・名前判定テストに成功。domain 217のViewer検証で新旧5トピックの検出・購読・バイナリ受信を確認。
- 経路結合テストは並行作業中のC++目標選択ノードが未配置のため停止。変換ノードのビルド対象も既存CMakeに存在せず、成功扱いなし。調整待ちの範囲を[pending.md](pending.md)へ記録。
- 検証用プロセスはすべて終了済み。既存ROSノードのPID維持と未再起動を確認。実画面のGPU操作は未検証。

## 2026-09-14: 計画目標選択のCPU張り付き解消

- 高負荷の対象をPython目標選択ノードへ特定。実入力のprofileでROSメッセージの取出し・Pythonオブジェクト展開が約82%を占有することを確認し、C++へ移管。[変更内容・計測条件・起動コマンド](releases/2026-09-14_goal_selector_cpu.md)を記録。
- Docker Releaseビルド、選択C++テスト9件と既存候補評価テストに成功。初回のメッセージ定数所属・整数型不一致は修正後の再ビルドで解消。
- 同一実入力10,801ノード・8候補で選択結果の完全一致を確認。TF移動・復帰、空候補の失効・復帰を検証。実入力6秒のCPU測定は103.27%から12.17%へ低下し、最終版の再測定は11.33%。
- tmap短縮を保持した最終launchの結合テストをdomain 228で実行し成功。C++既定topicの整合・実行ファイル配置・選定map配信まで確認し、対応する結合確認の保留を解消。変換ノードの別のビルド問題は対象外。
- 計測・再生・launchはすべて停止済み。既存ROSのPID・コンテナ状態を維持。稼働中旧版の再起動と実画面確認は未実施。

## 2026-09-14: Tmapへの表記統一

- ユーザー指定の`Tmap`へROS既定名・設定・Viewer判定を統一。環境入力`/topological_map`は維持。[変更・検証コマンド](releases/2026-09-14_tmap_topics.md)を更新。
- 関連3パッケージのDocker再ビルド、frontend型チェック・名前判定、domain 218の経路結合テストに成功。検証プロセスは終了済み。既存ROSへの停止・再起動操作なし。

## 2026-09-14: 静的ロボットマップのホバー枠除外

- `/ToPoDualArm/Tmap_static`をホバー枠・範囲取得から除外。実装は除外条件1行のみ。[仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- ホバー回帰テストとfrontend型チェックに成功。検証コマンドは終了済み、ROSの起動・再起動なし。実画面確認は未実施。

## 2026-09-14: 把持対象ノードのTopologicalMap配信

- ユーザー指定の`/grasp_pose_cands/Tmap`へ出力を変更。同じ候補内の元GNGエッジだけを収録し、候補IDはclusterへ対応。到達性を既存semantic_labelで配信し、Viewerの共通ラベル設定へ接続。[仕様・検証コマンド](releases/2026-09-14_candidate_topological_map.md)を記録。
- Docker Releaseビルド3パッケージ、把持推定・部分グラフC++テスト、Viewer詳細抽出7件、domain 117のROS結合3ケースに成功。候補間エッジ除外、状態更新、遅延購読、空配信を確認。
- frontendのラベル・ホバー回帰テスト、lint・buildに成功。既存ビルド警告あり、実画面GPU描画と大規模入力の負荷測定は未実施。
- 検証launch・子ノードは全停止済み。既存ROSのPIDとコンテナ状態を維持し、ROSデーモンの新規残留なし。稼働中旧版の自動再起動なし。

## 2026-09-14: バウンディングボックス判定の把持候補限定

- ユーザー指定により、ホバー枠・枠内クリック・範囲取得を`/grasp_pose_cands/`配下だけへ限定。非平面成分などの個別除外ではなく、許可するprefixの判定へ変更。[現行仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- 対象外だけの表示時にも範囲取得・枠・クリック選択が発生しない回帰テスト、frontend lint・buildに成功。実画面操作は未検証。
- 検証コマンドは全終了済み。ROSノード・サーバーの新規起動なし。backendは未変更で、既存の全体ビルドへの干渉なし。

## 2026-09-14: 把持候補矢印の上位N件表示

- 既存Marker設定へ表示数スライダーを追加。受信順の先頭N件だけを描画し、0は全件。ROS・対象ノードグラフ・計画への変更なし。[仕様・検証](releases/2026-09-14_candidate_display_limit.md)を記録。
- 描画回帰テスト、lint、型チェックに成功。コマンドは全終了済み。ROS・サーバー起動なし、実画面操作は未検証。

## 2026-09-14: バウンディングボックスのトピック別フラグ化

- ユーザー指定により、名前による対象制限を削除。Graph/Markerのトピック別`enable_bounding_box`とGUIの`Bounding Box`切替を追加し、全トピックの既定をOFFへ統一。[現行仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- 把持候補Graph・非平面成分Graph・任意名Markerの回帰検証に成功。既定OFF、明示ON、OFF時の枠・クリック解除、遅延応答破棄、再ON、全OFF時のタイマー停止を確認。
- frontend lint・buildとDocker backendビルドに成功。検証コマンドは全終了済み。ROSノードの新規起動・停止操作なし。実画面操作は未検証。

## 2026-09-14: 把持対象グラフの表示既定値

- `/grasp_pose_cands/Tmap`のノードサイズ0.008、エッジ表示OFF、ノード不透明度0.5へ変更。他レイヤーと手動設定は維持。[仕様・検証](releases/2026-09-14_candidate_topological_map.md)を更新。
- 既定値回帰テスト・型チェックに成功。検証コマンドは終了済み、ROS起動なし。実画面操作は未検証。

## 2026-09-14: 把持候補ロボットプレビュー未配信の調査

- 21:32:23のlaunchログで`topological_map_path_planner_node`（PID 51618）の起動直後の終了コード-6を確認。目標選択ノードだけが継続し、計画・プレビュー生成ノードは不在。
- 6秒の購読で静的GNG 10,801ノード、把持候補6件（INSIDE 2件）、選定ID `[3425,3485,4105]`を確認。候補評価トピックのpublisherは0。robot pose購読はQoS不一致があり、未受信を停止の根拠には使用せず。
- domain 218で同一設定を再現すると正常初期化し、10秒の上限まで入力待ちを継続。元の異常終了の例外本文は保存ログになく、詳細原因は未確定。コード変更・既存launchの停止や再起動なし。
- 以下の調査用プロセスはすべて終了し、既存ROSのPID維持を確認。

```bash
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 10s /ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/topological_map_path_planner_node --ros-args -r __node:=topological_map_path_planner_node -r __ns:=/ToPoDualArm --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml --params-file /tmp/launch_params_pihazn67'
docker exec -i gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 15s python3 -' < /tmp/preview_inputs_probe.py
```

## 2026-09-14: Marker側のバウンディングボックス設定の削除

- ユーザー指定により、法線などのMarker側からボタン・設定・判定経路を削除。Graph側の既定OFF・明示ONと、Marker本体の描画は維持。[現行仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- ホバー・既存Marker描画の回帰テスト、frontend lint・buildに成功。検証コマンドは全終了済み。ROS・サーバー起動なし、実画面操作は未検証。

## 2026-09-14: 把持候補ロボットの基準フレーム修正

- 実配信で通常ロボットの `base_link` と候補の `base_footprint` の不一致を確認。計画ノードもYAMLの `frame_id` を使用するよう修正し、重複した初期化を削除。[仕様・検証コマンド](releases/2026-09-14_candidate_robot_frame.md)を記録。
- 追加回帰検証で修正前の失敗を再現。Docker Releaseビルド、domain 218の既存経路結合テストと全候補フレーム一致、フレーム設定5ケースに成功。実画面描画は未確認。
- 購読プローブ・検証launch・子ノードは全停止済み。テスト用一時ログを削除。既存ROSへの停止・再起動操作なし。
- 最終確認時に、こちらの操作によらない既存Viewerコンテナ・関連launchの停止を確認。検証プロセスとROSデーモンの新規残留なし。外部変更を戻すための再起動は未実施。

## 2026-09-14: 候補ロボットと本体のTF不整合調査

- Docker内の6秒の読み取り専用購読で、候補ロボットのframeIdは`ToPoDualArm/base_footprint`、本体・静的GNG・選定GNGは`ToPoDualArm/base_link`と確認。候補配信も確認済み。
- `world -> base_footprint`は原点・無回転、別配信元の`world -> base_link`は位置`(0.15,0,-0.2)`・yaw `3.2 rad`。URDF由来の`base_footprint -> base_link`も同時配信され、`base_link`の親が競合。候補と本体で異なる変換の適用を確認。
- `world -> graspnet_table`は単位変換、把持候補と対象ノードグラフは`world`。今回の購読範囲では物体入力側の非単位変換なし。
- 調査用コマンドは下記のとおり終了コード0で終了し、プローブ残留なし。既存プロセスの停止・再起動操作、ソース・設定変更なし。調査中に外部からの既存launch再起動を観測したため、全PID維持とは記録せず。実画面での修正後確認は未実施。

```bash
docker exec -i gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 15s python3 -' < /tmp/grasp_frame_probe.py
```

## 2026-09-14: 稼働中処理のCPU負荷測定

- 22:00:50から10秒間の`pidstat`測定。100%は論理CPU 1個相当、ホストは16論理CPU。候補経路計画98.5%、Viewer専用Chrome GPUプロセス92.3%、GraspNet再生70.4%、同Chrome renderer32.0%、ais_gng_cpu26.5%、topofuzzy_bridge24.7%、world_index_to_voxel17.3%、目標選択13.5%、voxel_to_vlut10.0%、上方把持推定9.1%、平面クラスタ3.7%、Viewer gateway3.6%。Chromeの値はGPU使用率ではなくCPU使用率。
- 別時点の`docker stats --no-stream`ではgng_cpu_container244.12%、graspnet_player72.95%、rosbridge_container0.32%、frontend0.10%。frontendコンテナ値にブラウザ描画負荷は含まず。
- コード確認では、計画側に入力不変時の早期returnが存在する一方、`planFromStartCandidates`内でゴール候補ごと・開始候補ごとにDijkstra探索を実行。`topofuzzy_bridge`の占有・危険ボクセル受信は入力不変判定なしで安全評価とdirty化。計画が実際に何を契機に再実行したかの内訳は未測定。
- GraspNet再生は20 Hz・フレーム10〜15のループ設定。コンテナ内`graspnet_player_cpp.cpp`の`publish_frame`は各回RGB/depth画像の読み込み、点群生成、PointCloud2生成を実行し、生成済み点群のキャッシュなし。Viewerは既に`frameloop="demand"`、`dpr={1}`。
- 関数別サンプリングは`perf_event_paranoid=4`によって失敗。権限制限の変更なし、関数別CPU割合・改善率は未確定。ソース・設定の変更、既存プロセスの停止・再起動なし。
- 以下の有限計測コマンドは終了済み。対象主要PIDの継続を確認。ROSノード・サーバーの新規起動なし、失敗したperfの空ファイルは削除。

```bash
pidstat -u -p 1845486,1844918,1844966,1844494,1845334,1754609,1845139,1845484,1845141,1845222,1754613,1841990 2 5
timeout -s INT -k 3s 12s perf record -F 49 -g -p 1845486,1844494 -o /tmp/grasp_cpu_20260914.perf -- sleep 5
```

## 2026-09-14: 候補ロボットの表示数制限

- 候補ロボット欄に既存スライダーを追加。配信順の先頭N件、0は全件。ROS側の評価・選定・配信への変更なし。[仕様・検証コマンド](releases/2026-09-14_candidate_robot_display_limit.md)を記録。
- 候補数・順序更新・全件復帰・空配信・TF維持と既存ロボット色・Markerの回帰検証に成功。frontend lint・本番ビルド・backendビルドに成功。通常ビルドの権限不足・DockerのMCAP依存不足を記録し、依存や権限を変えず検証経路を切替。実画面操作は未検証。
- 検証コマンドは全終了、一時ファイルを削除。ROS・サーバーの新規起動や既存プロセスの停止・再起動操作なし。

## 2026-09-14: 候補ロボットの全件復帰・単体選択GUI

- 「全件に戻す」「先頭N件」「1体選択」を追加。番号スライダーで単体候補を指定し、全件復帰時は件数制限と単体選択を解除。Collisionの表示対象も選択候補へ一致。[現行仕様・検証コマンド](releases/2026-09-14_candidate_robot_display_limit.md)を更新。
- GUIコールバック、候補番号変更、候補数減少・空配信、全件復帰、TF維持と既存ロボット色・Markerの回帰検証に成功。frontend lint・型チェック・本番ビルドに成功。実ブラウザ操作は未検証。
- 全検証コマンド終了、一時出力・テスト用ファイルを削除。既存Frontendサーバーを維持。ROS・サーバーの新規起動、既存プロセスの停止・再起動なし。

## 2026-09-14: 候補経路探索の重複削減

- 安全ゴール間での開始候補別Dijkstra共有と探索配列化を実装。終点例外・隣接危険度ペナルティ有効時は個別探索を維持。[仕様・測定条件・全起動コマンド](releases/2026-09-14_candidate_path_batch.md)を記録。
- Dockerビルド、C++テスト6件、domain 218のROS結合テストに成功。実GNGの5開始候補×8ゴールの全経路一致、追加3回の探索時間中央値1,370.62 msから286.57 msを確認。変更後の個別・共有比較であり、旧版バイナリや稼働全体のCPU改善率とは区別。
- 検証launch・子ノードと有限計測は全終了済み、最終プロセス一覧で残留なし。既存ROSへの停止・再起動操作なし。調査中に外部からの候補計画launch起動を観測し、そのプロセスは維持。

## 2026-09-14: 候補経路のエッジコスト共有

- 1回の計画内で開始候補間の有向エッジ判定・基礎コストを再利用する遅延キャッシュを実装。安全制約・終点別ペナルティは維持し、単一開始候補時はキャッシュ生成なし。[現行仕様・全起動コマンド](releases/2026-09-14_candidate_path_batch.md)を更新。
- Dockerビルド・C++7件・domain 218の既存ROS結合テストに成功。実GNGの全40経路一致、キャッシュ寿命と更新後の再評価を確認。3回の同一実行内比較の中央値は共有方式271.269 ms、追加キャッシュ方式107.392 ms。全体CPU改善率は未測定。
- 検証用launch・子ノード・比較計測はすべて終了済み。既存ROSの停止・再起動操作なし。新たな設定・トピック・メッセージ変更なし。

## 2026-09-14: 固定GNG索引・探索木再利用による候補経路高速化

- 候補専用ノードに固定グラフ索引と再開可能な探索木を追加。通過可否変更時の失効、例外終点別の独立木、CPU数を上限としたジョブ並列化を実装。[仕様・全検証コマンド](releases/2026-09-14_candidate_static_path_index.md)を記録。
- Dockerビルド・C++8件・domain 218のROS結合テストに成功。実GNGの5開始候補×8終点で全40経路一致。各5回の初回探索は安全終点2.99〜3.91 ms、危険終点6.33〜8.49 ms。安全状態更新後も各回10 ms未満、再利用は最大0.240 ms。索引準備31〜41 msは別計測。ROS配信・描画込みの時間とは区別。
- 実運用の6候補が全て危険ラベルだった時点を観測。同時刻スナップショットの取得は候補ID配信元不在で失敗し、危険終点の性能検証は保存済みGNGの制御条件として実施。全体CPU・実運用遅延の改善率は未測定。
- プローブ・検証launch・子ノード・有限ベンチマークはすべて終了済み。既存ROSへの停止・再起動操作なし。トピック・パラメータ追加なし。

## 2026-09-15: 候補軌道計画の計算時間ログ

- 計画更新時の計算時間・目標候補数・到達数をINFOの1行に集約し、起動時の詳細一覧と候補受信ログをDEBUGへ移動。[計測範囲・検証・起動コマンド](releases/2026-09-15_candidate_planning_log.md)を記録。
- Releaseビルド・インストールとdomain 218の既存ROS結合テストに成功。計算時間ログ7件、静止入力での再探索なし、通常ログの簡潔化を確認。検証用launch・子ノードはすべて停止済み。
- Applied the requested English format `dof=7 Plan: 34.35 ms Count: goal=2 reach=2` and removed the separate startup INFO log. Release library rebuilds passed; no ROS processes were started for these wording changes.

## 2026-09-15: Surface clustering cost and incremental updates

- Observed the existing stream and captured 21 synchronized frames with 1,545 nodes. Mean surface extraction was 9.196 ms at about 1.9 Hz; all frames exhausted the 128-fit budget. Exact unchanged-patch count was zero across 200 comparisons. [Measurements, limitations, artifacts, and startup commands](designs/curved_surface_position_fit.md#2026-09-15-live-cost-and-incremental-update-investigation).
- Compared three isolated Release prototypes on identical inputs. Total reductions were 1.4–2.8%; each passed 66 existing tests. [Production adoption deferred](reject.md#2026-09-15-surface-clustering-micro-optimizations-and-exact-patch-cache). No production source or setting changes for this investigation.
- Capture/probe/build/replay/test processes all exited. Existing GNG launch and nodes remained running; no user process was stopped or restarted.

## 2026-09-15: モデル当てはめなしの連続面抽出

- `surface_method:=smooth_graph` を追加。実GNGエッジの距離・法線・接平面条件と影響成分の差分再探索を既存ファイル内に実装。[仕様・比較条件・全起動コマンド](releases/2026-09-15_smooth_surface_graph.md)を記録。
- Releaseビルド・C++74件・隔離ROS検証に成功。実入力21フレームで所属と採用エッジが毎回全探索と一致。従来modelの所属・種別・IDも変更前結果と一致。
- 従来7.407 msに対して新方式1.159 ms。ただし最大成分は平均1,420/1,545ノードへ拡大。差分管理単独は実入力で3.7%の増加、固定入力では判定・所属再探索の省略を確認。分割品質を理由に既定値modelを維持。
- 検証ROSノード・driver・再評価・ビルド・描画はすべて終了、プロセス一覧で残留なし。既存プロセスの停止・再起動操作なし。結果・比較図は `tmp/surface_graph_20260915/` に保存。

## 2026-09-15: 手元の候補経路高速化とリモート更新の統合

- ユーザー承認に基づき、手元のステージ済み10ファイルを`a957cdc`へ保存。最新fetch後の`origin/grasp_new4`（`cf17573`、計測ログ・連続面抽出の2コミット）を統合。ソースは自動マージ、進捗文書の末尾追記競合は双方の記録を保持して解消。
- Docker内で候補計画・曲面抽出のビルド、C++8件と74件、domain 218のROS結合テストに成功。候補ロボットの基準座標、混在入力、静止時の再計画抑止、関節変更・明示要求、空入力のクリアと関節目標配信なしを確認。
- 下記の有限コマンドと検証用launch・子ノードはすべて終了済み。既存ROSへの停止・再起動操作なし。確認中に別の既存シェルからの候補計画launch起動を観測し、そのプロセスは維持。リモートへのpushなし。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/gng_vlut_system --target test_candidate_metric_availability topological_map_planning -j2 && timeout -s INT -k 5s 120s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/ais_gng --target test_surface_model plane_cluster_incremental_node -j2 && timeout -s INT -k 5s 120s /ros2_ws/build/ais_gng/test_surface_model --gtest_color=no'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
```

結合テスト内の`ros2 run gng_vlut_system safety_monitor_node`と`ros2 launch gng_vlut_system grasp_joint_candidates.launch.py`の全引数は[検証起動コマンド](releases/2026-09-14_candidate_static_path_index.md#verification)と同一。

## 2026-09-15: 把持幅・TCP姿勢・関節姿勢の独立した追加評価

- 既存候補を購読する別ノードと専用出力を追加。局所実点群による幅・接触・グリッパ掃引評価、既存関節候補を初期値にしたIKを実装。[仕様・制限・全起動コマンド](releases/2026-09-15_grasp_candidate_refinement.md)を記録。
- Releaseビルド、新規8件・既存16件のC++検証と隔離ROS結合検証に成功。合成40 mm対象・実URDFで40/46 mmの接触幅・開口とIK補正、既存出力への非干渉、欠損・失効・不正入力時の無効化を確認。
- 指定bagの実点群20フレームを既存GNG・上方候補生成へ投入。追加評価は48更新・最大14候補で平均11.569 ms、最大18.701 ms（IKなし）。観測幅457件、確定接触対0件。実点群での把持成立、補正した腕の衝突・経路は未検証。
- 検証driver・launch・子ノード・有限計測は全終了済み、開始前後のプロセス一覧で残留なし。既存プロセスの停止・再起動操作なし。結果は `tmp/grasp_refinement_20260915/` に保存。

## 2026-09-15: 追加pushされた把持候補補正の再統合

- 最新fetch後の`origin/grasp_new4`（`fd44919`）を前回の統合結果`1a314f7`へ統合。前回の高速化・計測ログ・曲面抽出を保持。ソース・仕様書は自動マージ、進捗文書の追記競合は双方を保持して解消。
- Dockerで専用メッセージ生成、新規ノードと既存計画のReleaseビルド・インストールに成功。C++は新規補正8件、到達性9件、経路計画8件の計25件成功。domain 218で新規補正と既存候補計画のROS結合テストも成功。
- 合成対象の接触幅40 mm・開口46 mm・IK補正、既存トピックへの非干渉、欠損・衝突・TF失効復帰・入力停止・不正入力・空入力の処理を確認。実点群での把持成立は今回の検証対象外。
- 下記コマンドと検証用launch・子ノードはすべて終了済み。一時結果・ROSログの専用ディレクトリを削除し、テスト残留なしを確認。既存ROSの停止・再起動操作、pushなし。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 240s colcon build --packages-select gng_control_msgs --symlink-install --executor sequential && source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 120s cmake -S /ros2_ws/src/gng_vlut_system -B /ros2_ws/build/gng_vlut_system && timeout -s INT -k 15s 480s cmake --build /ros2_ws/build/gng_vlut_system --target grasp_candidate_refiner_node test_grasp_refinement test_grasp_candidate_reachability test_candidate_metric_availability topological_map_planning -j2 && timeout -s INT -k 5s 120s ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure -R "^(test_grasp_refinement|test_grasp_candidate_reachability|test_candidate_metric_availability)$"'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/grasp_merge_check_Fs4LBp/ros gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 120s cmake --install /ros2_ws/build/gng_vlut_system && timeout -s INT -k 20s 120s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_refinement.py --output /tmp/grasp_merge_check_Fs4LBp/result.json && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
```

新規検証の子launchは`ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py params_file:=/tmp/grasp_refinement_0vwzlmg0/params.yaml candidate_topic:=/grasp_refinement_test/source seed_topic:=/grasp_refinement_test/seeds point_cloud_topic:=/grasp_refinement_test/points output_topic:=/grasp_refinement_test/result`。既存候補計画テストの子launchは[前回の起動コマンド](releases/2026-09-14_candidate_static_path_index.md#verification)と同一。

## 2026-09-15: 候補独立ビューのXYZ軸切替

- 独立3DビューのXYZ軸を既定OFFとし、チェックボックスを追加。主画面・候補選択・XYZ寸法表示は維持。[仕様・検証コマンド](releases/2026-09-15_inspection_axes_toggle.md)を記録。
- frontendのlint・本番ビルドに成功。ブラウザの実操作は未検証。有限コマンドは終了、一時出力は削除済み。ROS・開発サーバーへの起動停止操作なし。

## 2026-09-15: 候補独立ビューの情報整理

- フッターをノード数・エッジ数・XYZ寸法の1行へ集約し、座標系の説明を削除。`nonplane_components`の表示だけを`nonplane_`へ短縮。[仕様・検証コマンド](releases/2026-09-15_inspection_info_layout.md)を記録。
- HTML出力テスト・lint・本番ビルドに成功。元source不変も確認。実ブラウザ描画は未検証。有限コマンドは終了、一時出力は削除済み。ROS・開発サーバーへの起動停止操作なし。

## 2026-09-15: 独立ビューのBbox切替

- 独立ビューへ既定OFFのBbox表示を追加。受信済みの範囲から線枠を描画し、主画面設定は維持。[仕様・検証コマンド](releases/2026-09-15_inspection_bbox_toggle.md)を記録。
- HTML出力テスト・lint・本番ビルドに成功。実ブラウザの切替・描画は未検証。有限コマンドは終了、一時出力は削除済み。ROS・開発サーバーへの起動停止操作なし。

## 2026-09-15: 主画面Bboxと独立表示の連動

- Bbox OFF時にも残っていたノード・クラスタ・Marker直接選択を、トピック別フラグへ連動。詳細取得の開始・応答でも確認し、OFF後の遅延表示とエラーを抑止。`/grasp_pose_cands/Tmap`だけ既定ONへ変更。[仕様・全検証コマンド](releases/2026-09-15_inspection_bbox_gate.md)を記録。
- 関連frontendテスト5ファイル、lint・本番ビルド、Dockerのbackendビルドに成功。Appの実接続条件と非同期処理、明示OFFの保持、既存ホバー・Marker・独立ビュー内Bbox既定OFFを確認。実ブラウザ操作は未検証。
- 検証コマンドは全終了、一時テスト・build出力を削除。ROSノード・開発サーバーの新規起動や既存プロセスの停止・再起動操作なし。

## 2026-09-15: 接続小平面の合算と未指定Bbox GUIの非表示

- 非平面経由の接続を再利用し、起点の最低ノード数・傾斜角と合算対象の寸法判定を分離。小平面・側面を含む候補Tmapを検証。[仕様・Docker実行コマンド](releases/2026-09-15_grasp_small_plane_membership.md)を記録。
- Bounding Box未指定のGraphではGUIなし、明示falseでは再ON可能とし、通常`/topological_map`を対象外へ変更。[仕様・frontend実行コマンド](releases/2026-09-15_bbox_explicit_controls.md)を記録。
- Docker Releaseビルド・C++回帰検証・CTest、frontend関連4ファイルのテスト・lint・本番ビルドに成功。初回ホバーテストの旧false期待値を更新して再検証成功。実入力・実ブラウザ・実機動作は未検証。
- 有限コマンドは全終了、一時出力は削除済み。既存ROS PIDを維持し、検証プロセスの残留なし。frontendコンテナの稼働期間更新を観測したが、本作業からの起動停止操作なし。

## 2026-09-15: Viewerの配信元停止・再起動追従

- 停止時の旧レイヤー・送信待ち・描画完了待ちを消去し、Streams選択を維持した同名配信元の自動再購読を実装。[変更内容・検証コマンド](releases/2026-09-15_viewer_stream_restart.md)を記録。
- Gatewayビルド、別ドメインでの配信プロセス停止・再起動、未ACKグラフ・点群・QoS変更Marker・即時再起動の復帰検証に成功。Reactフックの状態消去・復帰テストとlintにも成功。
- 通常frontendビルドの依存不足・設定キャッシュ権限による失敗を記録。ホストのTypeScript検査と、設定を直接読み込むViteでの一時ディレクトリへの本番アセット生成に成功。実ブラウザ描画は未検証。
- 検証用Gateway・配信元は終了し、既存Gateway PID 96433を維持。frontendの外部操作による停止・再起動を観測したが、本作業からの起動停止操作なし。

## 2026-09-15: 配信元再起動時の可視化設定保持

- 点群の表示設定を受信バッファと分離し、旧データ削除後の同名トピック復帰時に再適用。グラフの受信ごとの色変更・属性欠落による表示設定の自動OFFを廃止。[仕様・検証コマンド](releases/2026-09-15_viewer_stream_restart.md#表示設定保持の追加検証)を更新。
- 再起動回帰2件、Bbox回帰3件、lint、TypeScript検査、本番アセット生成に成功。非表示・透明度0・手動変換・グラフ設定の保持、新点群バッファへの置換を確認。実ブラウザ描画は未検証。
- 有限テスト・ビルドは終了、一時出力は削除済み。ROS・サーバーの新規起動や既存プロセスの停止操作なし。

## 2026-09-15: 上方把持候補の凸包・回転包含判定

- 平面PCAと複合候補の固定軸を廃止し、2D凸包・支持点切替区間による開口包含判定へ置換。[仕様・検証コマンド](releases/2026-09-15_convex_grasp_footprint.md)を記録。
- Docker Releaseビルド・CTest・AddressSanitizer/UndefinedBehaviorSanitizer検査に成功。回転形状、合算時の再回転、重複点不変性、退化形状、ランダム120件と独立角度走査の整合を確認。
- 32平面・3,072ノードの合成入力で推定時間の中央値0.125 ms（内点あり）・0.620 ms（全点が凸包頂点）を測定。実入力・Viewer・実機把持は未検証。
- 検証プロセスは全終了、一時出力は削除済み。既存プロセスへの起動停止操作なし。外部再起動後の推定器が今回のビルド結果を使用していることを確認。

## 2026-09-15: HTML・Viewerの把持ラベル統合

- HTMLの`/handle_points`専用配信を削除し、把持部位生成を残して`/semantic_points`へ集約。HTMLの出力ラベルを0/1に揃え、縁・蓋・机等と到達性2〜4の衝突を解消。[仕様・検証コマンド](releases/2026-09-15_grasp_label_unification.md)を記録。
- Viewerの「把持ラベル」配下に把持部位・未評価・到達範囲内・到達範囲外を統合。個別色・表示状態と旧設定の移管を確認。
- 実HTML関数の点群生成、ラベル解決・移管、モーダル構造、実Chromeの操作の4検証とlint・型検査・本番アセット生成に成功。HTML→ROS→GNGの実通信は今回未検証。
- `node tests/label_priority_browser.test.mjs`で起動したChrome PID 1060265は停止済み。専用プロファイル・一時ビルド出力を削除。既存ブラウザ・ROSノードの起動停止操作なし。

## 2026-09-15: SpatialTree比較とTmap目標選択索引

- 通常版とSpatialTree2を実Tmapの10,801ノードで比較。同一数値型でのバイナリ一致・検索不一致0を確認し、既存通常版doubleを採用。[仕様・比較結果・全検証コマンド](releases/2026-09-15_static_spatial_index.md)を記録。
- 静的索引の再利用と回転セルの外接箱検索を実装し、既存の選定条件・結果を保持。実Tmap座標と合成候補20件の目標選択中央値は1.351 msから0.485 msへ短縮。
- Docker Releaseビルド、12件の回帰検証、AddressSanitizer/UndefinedBehaviorSanitizer、隔離ROSでの実GNG候補・経路出力確認に成功。実画面・実機把持は未検証。
- 検証プロセスは全終了、一時コピー・実行ファイル・ROSログを削除し、再現用の座標・結果ログを保持。既存目標選択ノードPID 497597を維持。AIS・frontendの外部再起動を観測したが、本作業からの起動停止操作なし。
