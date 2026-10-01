# 2026-10-01 - Gazebo操作ログの短縮

- 変更: `gazebo | mode=avoid`形式、回避状態の日本語表示、状態行直下への切替・解除・停止・終了キー併記。制御処理・ROS診断値の変更なし。
- 停止表示: 当初の`停止ロック`を`停止解除待ち(B)`へ変更。解除済み時の停止・静止未確認表示は省略。`回避=停止`・`準備OK`を`回避=開始待ち`へ変更。静止の確認は停止指令保持中のみ表示、診断失効・停止理由表示は維持。
- 検証: 続報の表示変更を含む単体75 / 75件成功。実行中launchの再起動なし、実機指令なし。Gazebo通し試験は未実施。
- 試験コマンド: `PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider gng_vlut_system/test/test_dynamixel_sim_keyboard.py gng_vlut_system/test/test_gazebo_stop_keyboard.py gng_vlut_system/test/test_dual_arm_control_keyboard.py`、終了済み。
- 反映: 次回の操作端末起動時。[操作・表示仕様](../dynamixel_sim_control.md#起動と操作)。

## 続報: 回避ノードの反復ログ削除

- 変更: `Trajectory blocked: ...`と`Avoidance: path latched ...`の出力処理を削除。DEBUGを含む全ログレベルで出力なし。共有実装の`path latched`行が対象。再計画・経路採用処理は保持。
- 検証: `topological_map_planning`共有ライブラリの再ビルド成功、差分検査成功。install側がビルド成果物へのsymlinkであることを確認。ROSノードの新規起動・既存launchの再起動なし。
- ビルドコマンド: コンテナ内でROSとworkspaceをsource後、`timeout -k 10 240 cmake --build /ros2_ws/build/gng_vlut_system --target topological_map_planning -j2`。終了済み。
- 反映: 同じlaunchコマンドでの次回起動時。稼働中プロセスは旧ライブラリを継続使用。

## 続報: 操作端末をGazebo状態2行へ限定

- 変更: `dynamixel_sim_control.launch.py`の起動後の画面ログを抑止、子プロセスの出力をログファイルへ集約。操作端末も実機状態・操作結果・初期案内をログ側へ移し、状態変化時の`gazebo | ...`と操作キーの2行のみを専用TTYへ出力。
- 検証: 表示・操作・停止の既存72件成功。有限の子プロセスと仮想端末で、launch画面出力0文字・専用TTYは指定2行・子の標準出力と標準エラーはログ保存を確認。ROSノード起動なし、既存launch未再起動。
- 試験コマンド: コンテナ内でsource後、`PYTHONDONTWRITEBYTECODE=1 timeout -k 3 20 python3 /ros2_ws/src/artifacts/gazebo_terminal_20261001/check_output.py`。親・子とも終了済み、仮想端末を解放。
- 反映: 次回起動時。`ros2 launch`自身がlaunchファイル読込み前に出す起動案内まで非表示にする場合は、コマンド末尾へ`>/dev/null 2>&1`を指定。操作画面は専用TTYへの直接出力のため継続表示。[起動例](../dynamixel_sim_control.md#起動と操作)。

## 続報: A操作の開始拒否理由

- 調査: Aは`control/avoidance`への開始要求。初回4秒読取りで回避開始世代0、点群余裕31.511 mm、稼働設定の開始条件35 mm超を確認。この距離では開始拒否となり、制御FSMの切替拒否処理で停止ラッチへ遷移。実際のA操作時のサービス応答本文は旧実装で非保存のため、過去の拒否理由の完全な再現は不可。
- 続測: 点群余裕78.114 mm・制御hold・停止ラッチOFF。関節とボクセルの別受信からの再計算では左指`L_finger_left`、余裕73.903 mm。非同期受信のためstatusとは同一標本ではない。[読取り結果](../../../artifacts/gazebo_start_rejection_20261001/live.json)。途中の空ボクセル受信で診断スクリプトが失敗し、空配列の記録へ対応して再実行。
- 修正: 開始拒否メッセージへ実測余裕と必要距離を追加、制御FSMでサービス応答理由を保持、停止時の既存状態行へ`理由=`を追加。2論理行の表示を維持。開始条件・停止条件・キー割当の変更なし。反映は次回launch起動時。
- 検証: 関連169件成功。拒否時の停止・指令非送信、余裕の数値表示、停止理由の単一行表示・診断失効時の非表示を確認。既存launch未再起動、停止解除・回避開始・実機への操作なし。
- 読取り起動コマンド: コンテナ内でsource後、`ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1 timeout -k 3 15 python3 /ros2_ws/src/artifacts/gazebo_start_rejection_20261001/probe.py`。初回は同じ4秒のstatus購読部分を`python3 -`で実行。全読取りノード終了済み。
- 単体コマンド: コンテナ内パッケージルートで`PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test/test_dual_arm_control.py test/test_dual_arm_limits.py test/test_dynamixel_sim_keyboard.py test/test_dual_arm_control_keyboard.py test/test_gazebo_stop_keyboard.py`。終了済み。


## 続報: Bの停止解除拒否

- 調査: 稼働キーボードの`sim_reset`ログに複数の静止未確認による拒否と、その後の受付を確認。初回6秒の読取りでは停止保持適用済み・静止未確認、腰最大0.03980 rad/s。静止条件は回転0.01 rad/s・直動0.001 m/sを0.25秒継続。解除条件の迂回なし。
- 別の停止原因: 初回読取りの直近理由は自己除去後の点群が空（`has_cloud_tree: false`）による入力異常。停止解除拒否とは別事象。
- 変更: 拒否理由を処理待ち・操作失効・Gazebo静止未確認・制御側静止未確認などへ分離し、状態行の`B=`に送信待ち／受付／拒否を表示。元の停止理由は保持。GazeboのA・Bを実機診断の鮮度チェックから分離。
- 検証: 状態表示・停止解除FSMの単体104件成功。初回診断は`timeout -k 2 15 python3 -`への標準入力スクリプトで6秒購読、正常終了。再読取りはhold・ラッチOFF、腰最大0.04490 rad/s。[再読取り](../../../artifacts/gazebo_reset_feedback_20261001/report.json)。この読取りの`num_confirmed: 0`はラッチOFF時であり、停止不成立の判定には非使用。
- 制限: 腰の動きの物理的・数値的原因は未確定、今回の変更で解消したとの確認なし。静止未確認の場合のB拒否は維持。反映は操作端末・制御ノードを含むlaunch再起動後。既存プロセスの停止・再起動・解除要求・実機指令なし。読取り・テストは終了済み。

検証コマンド（コンテナ内、ROSとworkspaceをsource後、終了済み）:

```bash
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1 timeout -k 2 15 python3 /ros2_ws/src/artifacts/gazebo_reset_feedback_20261001/probe.py
cd /ros2_ws/src/gng_vlut_system
PYTHONDONTWRITEBYTECODE=1 timeout -k 5 60 python3 -m pytest -q -p no:cacheprovider test/test_dual_arm_control.py test/test_dynamixel_sim_keyboard.py
```
