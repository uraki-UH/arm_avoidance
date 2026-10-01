# 2026-10-01 - 終了したシミュレーションロボットのViewer表示削除

- 原因: 共通descriptionトピックの全配信元消失だけを削除条件としていたため、実機側の配信継続中は終了したシミュレーション表示も残存。
- 変更: descriptionの`tag`と配信元GIDを対応付け、1秒周期で配信元ごとに消失判定。該当キャッシュ・描画データ・未描画姿勢のみ削除、他ロボットと表示設定は維持。
- 仕様: [Robot Lifecycle](../../../ToPoFuzzy-Viewer/common/ws_protocol_v2.md#robot-lifecycle)。配信停止ではなくROS配信元消失の判定、ROS発見情報の反映遅延あり。
- 検証: backend Releaseビルド、frontend lint・build、frontend削除ハンドラ試験1件に成功。隔離ROS domain84・WS19094で、実機役を残したSIGINT終了・再接続時のキャッシュ削除・同名再起動・全終了に成功。
- 初回失敗: ホストbuildは既存tsbuildinfoの書込み権限、frontendコンテナbuildは既存MCAP依存不足で失敗。依存追加なし、共有workspaceのgngコンテナbuildで成功。既存コンパイラ警告・bundleサイズ警告あり。
- 未検証: 実ブラウザの目視と実Gazebo launch全体のCtrl+C。描画ハンドラとROS→WebSocket経路は個別検証済み。
- 反映: Viewerバックエンド再起動・画面再読込みが必要。既存Gatewayは未再起動、実機指令なし。
- 後片付け: 模擬配信・試験Gateway・ビルド全終了、既存Gateway PID31942維持。既存Gazeboは確認中にPID変更あり、エージェントによる停止・起動なし。

試験コマンド（gngコンテナ内、ROS・workspaceのsource済み）:

```bash
timeout --signal=INT --kill-after=12s 75s python3 -B /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_robot_lifecycle.py
cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend
node --test tests/robot_lifecycle.test.mjs
npm run lint
npm run build
```
