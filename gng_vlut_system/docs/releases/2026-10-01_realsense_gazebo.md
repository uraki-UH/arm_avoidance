# 2026-10-01 - RealSense実点群によるGazebo継続回避

変更:

- 入力: `realsense_gazebo_input.yaml`と`external_pointcloud_bridge.py`追加。実時間stampの新鮮さ検査、仮想カメラ配置、受信ごとのGazebo時刻付与。
- 回避: 模擬前腕・LiDARなしの継続回避、開始姿勢への復帰、欠測停止。`roi_voxels` → 実機姿勢による自己除去 → `self_filter_roi_voxels` → VLUT/GNGの常設経路。
- 自己形状: 実機全関節のFKとカメラ取付補正による生成。Gazeboの回避姿勢から独立。欠損・失効・重複関節値、TF取得失敗時のマスク更新抑止。自己除去を省略する設定の起動拒否。
- 操作: 共通launchの`input_config`・`camera_pose`追加。A開始／保持、Space停止、Ctrl+C終了。実機出力なし。

結果:

- ビルド・install: 成功。回帰: 410 / 410 件成功。既存SciPy / NumPyバージョン警告あり。
- Gazebo通し試験: 試験点群・実機役の非ゼロ首腰関節値による自己除去・退避・復帰に成功。除去数最大24ボクセル、最小推定点群余裕0.094394 m、最大関節変位1.313007 rad、GNG指令選択89回。
- 欠測試験: 点群配信を継続したまま実機役関節入力だけを停止した場合と、点群入力を停止した場合の両方でGazebo実測停止に成功。試験所有プロセス終了・端末復元・専用domain 96空を確認。
- 実機の未検証範囲: 腰を含む全実測関節値が未接続。実カメラ取付校正と、実測姿勢による自己除去を含む実点群回避は未検証。腰のDynamixel IDは確認待ち。
- 修正前の実入力診断: 10秒で変換済点群83フレーム受信、最終110,717点。仮想配置`[0.08, 0, 0.55, 0, 0, 0]`は未校正。診断ノード終了済み。自己除去常設化後の通し検証とは別。
- 初回失敗: 新規ブリッジの実行権限不足。権限修正後成功。初回cleanupは残留DDS検出で未確認判定、その後プロセス消滅・再試験開始時domain 96空を確認。成功試験は所有プロセス終了・端末復元・domain空を確認。
- 初期実装の訂正: 実機と仮想機体の姿勢差を理由に自己除去を省略していた経路を撤去。修正前の`fixture_second`・`roi_topic`・`interactive`の成功記録は、常設自己除去の検証根拠には該当せず。
- 表示用デモ: 修正前に203点群・882ボクセル・GNG危険415ノードを確認後、ユーザー操作で終了。今回の試験で既存RealSense・Viewerの停止なし。確認用5秒ROSノード・WebSocket読取りは終了済み。

根拠: `artifacts/realsense_gazebo_20261001/real_self_filter/report.json`。初期実装の記録: 同ディレクトリの`real_input_probe.json`、`fixture_first/report.json`、`fixture_second/report.json`、`interactive/status.json`。
起動コマンド・座標設定・制限: [RealSense実点群を使うGazebo回避](../realsense_gazebo.md)。実カメラ診断はコンテナ内`python3 -`による`external_pointcloud_bridge`と`real_cloud_output_probe`の10秒受信。

最新の試験起動コマンド（コンテナ内、ROS環境読込み後、終了済み）:

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_external_pointcloud.py \
  --output /ros2_ws/src/artifacts/realsense_gazebo_20261001/real_self_filter
```
