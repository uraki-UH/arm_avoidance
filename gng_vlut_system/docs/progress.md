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
