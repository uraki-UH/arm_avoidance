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
