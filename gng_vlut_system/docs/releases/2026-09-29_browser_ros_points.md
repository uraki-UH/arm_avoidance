# 2026-09-29 - SourceDeliveryブラウザのROS点群送信・物体編集

## 1. 要約

SourceDeliveryのブラウザに「ROS 2送信」タブを追加。
RGB-D全体、選択物体の完全表面、選択物体の遮蔽付きRGB-Dを独立ブリッジへ送信可能。
GNG・FVGの結果待ちから独立し、送信中の取得・通信は1件に制限。

- 完全表面は指定点数（既定10,000点）の面積比例サンプリング。同じ形状・姿勢・点数では再利用。
- 遮蔽付きは全シーンと対象だけの深度を照合。視野外・全面遮蔽では0点を送信。
- 物体・車両の追加前に、テーブル／base_footprint基準のXYZ［mm］を指定可能。
- 既存の追加後位置編集を維持。3D上の右クリックから物体の削除が可能。

[起動・トピック・制限の正本](../../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md#独立した点群送信)。
[物体操作](../../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/README.md#ros-2への点群送信物体編集)。

2026-10-01追加: 深度画素位置を維持した画像・CameraInfo・PointCloud2の同時出力。
共通stamp・カメラ光学座標、32FC1［m］、無効深度0／XYZはNaN。既存XYZ送信は維持。
詳細・再起動手順は上記の正本に集約。

## 2. 条件・検証

| 項目 | 条件・結果 |
| --- | --- |
| ブラウザ | 標準モデル、既存8877サーバー、独立Chromeプロファイル、ページエラー0件 |
| ROS | Docker内Humble、ROS_DOMAIN_ID=178、HTTP 18879、既存ノードと分離 |
| 実受信 | シーンRGB-D 127,350点、完全表面10,000点、対象可視点3,348点 |
| 出力整合 | 3トピックでbase_footprint、点数×12バイトのXYZデータを購読側で確認 |
| 表面 | 立方体10,000点の全点が境界面上、6面全てに分布、平行移動後の範囲を確認 |
| 遮蔽 | 96×64深度画像、遮蔽なし780点／全面0点／半分390点 |
| GPU読み出し | 同期・非同期の両経路で上記遮蔽結果が一致、遮蔽物の表示状態を復元 |
| 編集 | world指定(800,200,300) mmの追加・削除、実マウス右クリックのメニュー表示・削除成功 |
| 回帰 | Python形式検証5件成功、変更JavaScriptの構文検査成功 |
| 差分 | 既存CRLFを維持。cr-at-eol指定でdiff --check成功 |

可視対象が画角外の条件では0点のROS受信も確認。
完全表面には裏面・内部面を含み、複数メッシュの外皮を取り出す集合演算は未実装。
2026-10-01追記: RGB-D全体で深度画像・CameraInfo・画素対応XYZを同時送信可能。
RGB画像・色フィールド、時計同期、TF配信は対象外。
GNG学習との統合運転、Longモデルの実送信、大規模車両メッシュの性能、実機動作は未検証。

試験起動コマンド（いずれも終了済み）：

```bash
node /tmp/topo_points_browser_test.mjs
node /tmp/topo_points_context_test.mjs
node /tmp/topo_points_visible_test.mjs
docker exec gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/topo_points_ros_test.py'
```

一時試験スクリプトは独立Chromeを起動し、終了時に自身のプロセス群だけ停止。
ROS試験は `pointcloud_bridge.py --port 18879` と購読ノードを起動し、終了時に停止。
試験ChromeとROS試験プロセスの残存なしを確認。既存サーバー・ユーザーのブラウザ・既存ROSは維持。

2026-10-01検証: 標準モデル、Humble、ROS_DOMAIN_ID=178、HTTP 18879。
848×480全画素の深度・XYZ対応、共通時刻、既存点群の有効点数一致を確認。
有効124,002画素／無効283,038画素。形式・メッセージ回帰9件成功。
試験起動: `node /tmp/topo_depth_browser_test.mjs`、
`docker exec gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/topo_depth_ros_test.py'`。
自身のChrome・購読ノード・18879ブリッジは停止済み。既存プロセスは維持。
持続送信レート・Longモデル・実RealSense購読アプリとの互換性は未検証。
