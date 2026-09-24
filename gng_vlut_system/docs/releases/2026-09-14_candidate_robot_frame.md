# 2026-09-14 - 候補ロボットの表示基準フレーム

## 1. 要約

候補ロボットと通常ロボットの基準フレームをYAML設定で統一。

- `robot_base_frame` が空の場合は共通の `frame_id` を使用。両方が空の場合のみ従来のURDFルートへフォールバック。
- `frame_id` とURDFルートの単独リンク名にはノード名前空間を補完。`world` と名前空間付きフレームはそのまま使用。

- ToPoDualArmの通常表示が `base_link`、候補だけが `base_footprint` を参照していた位置・向きの不一致。
- 後から上書きされる重複したフレーム初期化を削除。

## 2. 条件・検証

- 候補配信・現在手先Pose・評価内のFK基準に適用。候補関節角度・URDF・Viewer描画処理への変更なし。
- 稼働中の `grasp_joint_candidates.launch.py` は再起動後に反映。既存プロセスへの停止・再起動操作なし。

- 既存YAMLの `frame_id` を計画ノードでも読込。`robot_base_frame` の明示指定は従来どおり優先。
- topic名・メッセージ形式・launch引数への変更なし。

- 稼働中ROSを12秒購読。通常ロボットの `ToPoDualArm/base_link` と、候補の `ToPoDualArm/base_footprint` を確認。前者のworld変換は位置 `[0.15, 0, -0.2]`・yaw `3.2`、後者は単位変換。
- 修正前に追加したフレーム一致検証で失敗を再現。修正後は全候補のdescription/poseとGNGフレームの一致、経路生成・実行指令非配信の結合テストに成功。
- YAML既定、`world`、名前空間付き指定、空指定のURDFフォールバック、`robot_base_frame` 明示優先の5ケースに成功。

Docker内、`source /ros2_ws/install/setup.bash` 後の実行コマンド:

```bash
colcon build --packages-select gng_vlut_system --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1 --event-handlers console_direct+
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 25s 120s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py
```

結合テスト内部で起動した `safety_monitor_node` と `grasp_joint_candidates.launch.py`、購読用 `python3 -`、5ケース用の次のノードはすべて終了済み。5ケースの追加パラメータは順に「追加なし」「`-p frame_id:=world`」「`-p frame_id:=ToPoDualArm/base_link`」「`-p 'frame_id:=""'`」「`-p frame_id:=world -p robot_base_frame:=override_base`」。

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 /ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/topological_map_path_planner_node --ros-args -r __node:=codex_candidate_frame_case -r __ns:=/ToPoDualArm --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p goal_candidate_ids_topic:=/codex_candidate_frame_goals
```

**制約**

- 実画面の描画位置は未確認。ROS配信フレームと既存ViewerのTF適用経路を検証。
- 実行環境では `base_link` に `world` と `base_footprint` の両方からTF配信あり。TFツリーの親重複は本変更とは別の設定上の問題であり、既存TF発行元への変更なし。
- Releaseビルドに既存のCMake CMP0074警告あり。コンパイルエラーなし。
