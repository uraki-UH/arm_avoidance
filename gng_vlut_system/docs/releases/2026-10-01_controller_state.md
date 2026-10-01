# 2026-10-01 - Gazeboコントローラの非推奨購読先の移行

- 原因: `dual_arm_controller/state`の購読による非推奨警告。
- 変更: UDP出力・Dynamixel連携・関連試験の購読先を`dual_arm_controller/controller_state`へ統一。目標値は`reference`、実測値は`feedback`へ移行。ログレベルの一括抑制なし。
- 検証: 単体145 / 145件成功。旧フィールドが空の新メッセージからの目標取得、実測値の誤転送防止を含む回帰。`git diff --check`成功。
- 実環境: domain25で4秒の読取り専用購読は0件。確認中に既存Gazebo終了、警告解消の実動作確認は未完了。エージェントによる既存プロセスの停止・再起動なし、実機指令なし。
- 反映: インストール済みPython実行ファイルはソースへのリンク。次回の対象launch起動から適用。稼働中プロセスへの自動適用なし。
- 試験終了: 単体・標準入力の読取りプローブとも終了済み。試験由来の常駐ノードなし。

単体コマンド（コンテナ内、ROS・workspaceのsource済み）:

```bash
cd /ros2_ws/src
PYTHONDONTWRITEBYTECODE=1 timeout 60s python3 -B -m pytest -q -p no:cacheprovider \
  gng_vlut_system/test/test_dual_arm_control.py \
  gng_vlut_system/test/test_dual_arm_control_udp.py \
  gng_vlut_system/test/test_dynamixel_sim_output.py
```

読取りプローブ: `docker exec -i gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 timeout --signal=INT --kill-after=2s 10s python3 -B -'`。標準入力コードによる`controller_state_migration_probe`、新トピックの購読と旧購読者一覧の参照のみ。
