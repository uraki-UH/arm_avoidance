# 大容量点群再生の検証（2026-09-05）

対象: `/rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed`

環境: ROS 2 Humble、Fast DDS 2.6.10、rmw_fastrtps_cpp 6.2.9、
rosbag2_transport 0.15.15、`gng_cpu_container`（host network / host IPC）。
点群は約3.46 MiB、記録周期は約30 Hz、bag長は約25.17秒。

## 測定結果

| 条件 | 平均Hz | 最大間隔 | 備考 |
| --- | ---: | ---: | --- |
| UDPバッファ拡張後・共有メモリ設定前 | 6.77 | 1.460秒 | 12秒のraw C++受信 |
| 32 MiB共有メモリ・先読み2,000件 | 29.98 | 1.007秒 | 55秒、100 ms超2回、5 ms未満60回、点群等の選択再生 |
| 32 MiB共有メモリ・先読み50件 | 29.99 | 0.141秒 | 55秒、100 ms超2回、5 ms未満4回、全トピック再生 |
| 32 MiB共有メモリ・先読み10件・GNG稼働 | 29.99 | 0.142秒 | 55秒、ループごとのキュー枯渇警告、全トピック再生 |
| 32 MiB共有メモリ・先読み50件・GNG稼働（採用） | 29.99 | 0.143秒 | 55秒、100 ms超2回、5 ms未満4回、99パーセンタイル46.1 ms |

後半4条件は起動後3秒を除外し、steady clockでraw受信の到着時刻を測定。
受信器はSensorDataQoS、SingleThreadedExecutorによる待機式処理。
初期12秒試験とは測定時間・受信器の待機方式・再生トピックが異なるため、
数値の差だけを単一要因の効果とは扱わない。

先読み2,000件では平均Hzが正常でも、停止後の追いつき配送が残った。
50件への削減で秒単位の停止は解消。10件では追加改善がなく、通常値は50件を採用。
完全な33 ms等間隔ではなく、ループ境界付近に約140 msの間隔が残存。

GNG入力の内部ログでは初期100件除外後1,830件で平均21.83 Hz、最大61.23 ms、
100 ms超0回。GNG計算は約45 ms/回であり、点群配送の30 Hzとは別の処理上限。
再生の切替操作を含む区間は、このGNG集計の対象外。
採用する50件の設定でも、直近625件のGNG内部周期は平均21.73 Hz、最大58.26 ms、100 ms超0回。

## 採用設定と適用方法

`fastdds_shm_large_pointcloud.xml`の共有メモリ領域32 MiB、
UDP送受信バッファ16 MiBを使用。`/dev/shm`に約33.6 MBの領域作成を確認済み。
`gng_cpu`のCompose環境変数に設定済み。
現在のコンテナとDockerfileにはBash起動時の自動設定も導入済み。
新しいターミナルではexportと先読み引数の指定は不要。
以下は検証条件を明示した起動例。

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/docker/fastdds_shm_large_pointcloud.xml
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed \
  --loop --read-ahead-queue-size 50
```

GNG検証時の起動コマンド（Humbleとworkspaceのsetup.bash読み込み後）:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/docker/fastdds_shm_large_pointcloud.xml
ros2 run ais_gng ais_gng_cpu --ros-args \
  --params-file /ros2_ws/src/ais_gng_cpu/src/ais_gng/config/gng_cpu/topo_points.yaml \
  -p 'input.topic_names:=[/camera/camera/depth/color/points]' \
  -p performance.log_interval_ms:=1
```

測定器の起動コマンド（一時ビルド）:

```bash
timeout 65 /tmp/generic_rate_probe/install/generic_rate_probe/lib/generic_rate_probe/generic_rate_probe
```

検証用playerには`--disable-keyboard-controls`も付与。
検証用のbag player・GNG・周期測定器は作業終了時に停止済み。
途中の比較では`FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`または`SHM`を指定したが、
環境変数だけでは転送経路の変更を立証できないため、比較値を実際のTCP/SHM専用転送の証拠とはしない。
採用設定はXMLと実際の共有メモリサイズで確認済み。

## 参考

- [Humble・約3 MB点群のrosbag再生欠落報告](https://github.com/ros2/rosbag2/issues/1152)
- [Fast DDS共有メモリ領域の仕様とサイズ不足の警告](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)
- [先読みキュー不足による再生遅延報告](https://github.com/ros2/rosbag2/issues/1963)

先読み増量はキュー不足への対策だが、本bagのループ再生では過大なキューを避ける必要がある。
