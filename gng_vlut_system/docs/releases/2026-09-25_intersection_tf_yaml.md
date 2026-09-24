# 2026-09-25 - 交差点の位置・姿勢をYAMLで指定

## 1. 要約

`intersection_tf.launch.py`を追加し、[intersection_tf.yaml](../../../ais_gng_cpu/src/ais_gng/config/intersection_tf.yaml)から`world → map → hesai_lidar`の静的TFを配信。
`pos: [0, 0, 6]`、`rot_deg: [3, 13, 3]`を初期値とし、Viewerと同じEuler XYZ順でクォータニオンへ変換。目視による暫定補正。

TF配信後にGNGを再起動し、Viewerの点群・GNG両方の手動変換をリセット。[起動・編集手順](../../../ais_gng_cpu/README.md#交差点bagの位置姿勢をyamlで補正)。

## 2. 条件・検証

| 項目 | 内容 |
| --- | --- |
| 条件 | Humble、追加したYAML・launchをコンテナのsymlink-install先へ登録 |
| 変換 | world→mapの恒等変換、map/world→hesai_lidarの並進・回転を受信。独立計算のRx Ry Rz行列との最大差2.23e-16未満 |
| 入力検査 | 非有限値・要素数・型などの不正ベクトル5件、不正フレーム3件の拒否 |
| 試験環境 | ドメイン213の初回2試行はTF未受信。プロセス起動前からドメイン93を設定した試行で成功。初回の通信不成立原因は未特定 |
| 後片付け | 全試験TF・launch終了。最終試験前後の既存GNG・bag・Viewerプロセス維持 |
| 未検証 | 稼働中GNGへの補正適用とViewerの実描画。既存プロセスの停止操作なし |

最終試験の起動コマンド：`ROS_DOMAIN_ID=93 ros2 launch ais_gng intersection_tf.launch.py`。受信検証後に停止済み。
