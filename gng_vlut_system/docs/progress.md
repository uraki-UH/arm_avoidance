# 実施済み作業の記録

実際に行った変更・調査・検証だけの記録。予定は [TASK_LIST.md](TASK_LIST.md)、方針未確定の課題は [TASK_CANDIDATES.md](TASK_CANDIDATES.md)、不採用判断は [reject.md](reject.md) に分離。
時間・依存作業などによる保留作業の状態は [pending.md](pending.md) に分離。
記録単位は「日付 / 対象 / 実施内容 / 結果・検証範囲 / 根拠へのリンク」。既存履歴の一括転記なし。

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
