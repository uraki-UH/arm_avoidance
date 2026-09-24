# 2026-09-14 - 候補ロボットの表示数・単体選択

## 1. 要約

`ToPoDualArm/candidate_goal_preview` のロボット欄で「全件に戻す」「先頭N件」「1体選択」を切替。

既存の候補ロボット判定とControlSliderを使用。`max_visible_candidates` の既定0は全件。`selected_candidate_idx` がnull・未指定なら件数制限、数値ならその候補1体だけのVisual・可操作性楕円体を描画。GUIの番号は1始まり、保持する添字は0始まり。現在の配信順は候補ID順であり、評価ランキングではない。

- 「全件に戻す」は単体選択と件数制限を解除。色・TF・可視状態は維持し、候補未受信時も操作可能。
- 「1体選択」は候補番号スライダーで切替。「先頭N件」へ戻す際は以前の件数制限を復帰し、全件だった場合は1件から開始。
- 候補数減少時は現在の範囲へ表示添字を補正。配列内の位置の選択であり、同一候補IDの追跡ではない。

実GUIコールバックで全件復帰・単体選択・モード切替・空入力時の有効状態を検証。実React Three Fiber・模擬URDF/rendererで表示数、選択番号の切替、範囲補正、全件復帰、候補更新、TF維持を検証。

入力停止中の表示数変更も再描画へ反映。空のinstancesから先頭候補の通常描画へ戻る挙動を解消。
Collisionは既存の1体表示を維持し、単体選択時は選択候補の関節値へ一致。空のinstancesはCollisionも非表示。

## 2. 条件・検証

候補データの受信更新後も表示数設定を保持。リロード時は全件へ復帰。ROS配信数・評価式・候補選択・TF・関節値・通常ロボットの設定UIへの変更なし。ROS側の既存プレビュー上限8件は維持。

ROS・WebSocketプロトコルの変更なし。追加はFrontendのロボット表示設定のみ。

以下に成功。全コマンド終了済み。一時出力・テスト用ファイルは削除済み。ROSノード・サーバーの新規起動、既存プロセスの停止・再起動なし。

```bash
docker exec frontend timeout 60s node --test tests/robot_candidate_limit.test.mjs tests/robot_link_appearance.test.mjs tests/marker_array_renderer.test.mjs
# frontendディレクトリでの実行
timeout 120s npm run lint
timeout 120s npm run build -- --configLoader runner --outDir /tmp/codex-candidate-selection-build.eJGCEk
# 表示数追加時のbackend検証。単体選択追加ではbackend未変更
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && cd /ros2_ws && timeout -s INT -k 10s 120s colcon build --packages-select topo_fuzzy_viewer --symlink-install --event-handlers console_direct+'
```

**制約**

実ブラウザでのスライダー操作・GPU描画は未検証。ホストの通常ビルドと既存色テストはnode_modulesへの書き込み権限で失敗し、Dockerの通常ビルドは既存のMCAP依存不足で失敗。依存や権限は変更せず、テストはDocker、型チェック・本番ビルドはホストのrunner方式と一時出力先で完了。既存の大きいbundle警告あり。
