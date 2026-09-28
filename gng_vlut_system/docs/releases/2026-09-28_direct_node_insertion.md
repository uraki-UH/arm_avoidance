# 2026-09-28 - 入力占有による直接追加と未観測ノードの寿命

## 要約

続報：入力を既存警戒領域で受け持つ最近傍の通常寿命を更新し、追加削除の往復を抑制。
通常gng_cpuへ反映、既存ROS再起動なし。診断は通常ビルドから除外。
[比較条件・結果・全起動コマンド](../../../benchmarks/voxel_plane_lookup_20260928/churn.md)。

入力voxelの先頭実測点へ代表点を統一。重心経路と補助挿入経路の二重化を撤去。
既存警戒領域外・近傍の有限平面で説明不能な入力代表点を、学習抽選前に追加。
入力占有なし・平面で説明不能なノードには連続未観測寿命を適用。
再観測時のリセット、削除IDの再利用、孤立ノードの即時削除との競合を修正。
オクルージョン判定・別占有グリッド・26近傍処理は追加なし。
仕様・設定・ABI互換性の正本：[CPUサンプリング](../../../ais_gng_cpu/docs/sampling.md#未カバー占有セルの直接挿入)。

続報の接続修正は通常gng_cpuを再ビルドしinstallへ反映。稼働中ノードの再起動・ROS起動検証は未実施。
追加直後のノードから既存探索の最近傍1ノードへ即時接続。第2近傍への追加接続なし。
先の接続全撤去は訂正。入力最近傍対の接続と既存次数上限を維持し、候補なしの場合だけ遠方接続を回避。
探索は周辺セル限定。無制限の遠方探索という説明は訂正。新しい距離閾値・追加制限なし。
処理費用は増加しており、軽量化・実物体の追従品質改善は未確認。

## 条件・検証

ID再利用の続報：平面側へ生成フレームを追加し、再利用IDの所属・法線EMA・孤立猶予・連結履歴を失効。
同じID・同じ世代の通常追従は維持。旧統計の除去は既存差分集計を再利用、全クラスタのリセットなし。
修正前は生成フレームを更新した同一IDの別ノードへ、平面所属・法線履歴を誤継承。
6×8点の平面の半分を20 m離して再生成し、両側間のエッジを除去。再利用IDでは5入力連続で1面・最大幅21.4 m。
同じ位置・エッジで新IDにすると最大幅1 m、3入力目から2面。生成フレームがgraph_viewで欠落することも確認。
修正後は平面テスト68件成功。6×6点の回帰例でROS/直接入力×差分集計ON/OFFの4構成を検証。
遠方再生成後10入力で新ID対照と所属・幾何が一致し、最大幅1 m未満・最終2面。実bagの改善は未確認。
元の6×8点再現例も修正後は最大幅1 m・3入力目から2面へ一致。関連CTest 2対象成功。
通常ais_gngの全利用側を再ビルド・install反映。`/ros2_ws`でROS環境読込み後に
`CMAKE_BUILD_PARALLEL_LEVEL=2 timeout 300 colcon build --packages-select ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+`を実行。
全ビルド・試験終了、エージェントによる既存ROS停止・再起動なし。ID修正単独の処理時間増分は未測定。
node_inputの構造変更に伴う利用側再ビルドが必要。生成フレーム未提供の独自入力ではID再利用の識別不可。
起動はコンテナ内のROS環境読込み後、`timeout 180 cmake --build /ros2_ws/build/ais_gng --target test_plane_cluster_incremental -j2`と
`timeout 60 /ros2_ws/build/ais_gng/test_plane_cluster_incremental`。ログは同artifact内の`id_reuse_fix_{build,test}.log`。
再現ソースは`benchmarks/voxel_plane_lookup_20260928/id_reuse_probe.cpp`、実行はコンテナ内で
`timeout 10 /ros2_ws/src/artifacts/voxel_plane_lookup_20260928/id_reuse_probe`。コンパイル・試験とも終了。
初回はartifactへのソース書込不可、次に試験のframeフィールド名でコンパイル失敗。保存先と試験コード訂正後に再現。

即時1本接続への訂正後は製品23件・通常26件成功。追加直後の双方向接続と第2近傍への非接続を確認。
直接追加OFF時の従来動作、候補のない遠方への非接続、後続入力での接続も回帰確認。
通常installへ反映、全ビルド・試験終了。実bagの孤立率・接続待ち時間・処理時間は今回未測定。
以下と同じビルド・CTest・installコマンドを使用。ログは同artifact内のnearest_edge_{product_build,normal_build,install}.log。
先行する接続全撤去のテストは当時の動作確認であり、現行の期待値は即時1本接続へ変更。
未観測寿命・代表点・平面統合条件は今回の接続修正では変更なし。以下の時間測定は接続修正前。
起動コマンドは同コンテナのROS環境読込み後に以下を実行。全ビルド・CTest終了、既存ROS停止なし。

```bash
timeout 240 cmake --build /ros2_ws/src/artifacts/voxel_plane_lookup_20260928/product -j2
LD_LIBRARY_PATH=/ros2_ws/src/artifacts/voxel_plane_lookup_20260928/product:$LD_LIBRARY_PATH timeout 60 ctest --test-dir /ros2_ws/src/artifacts/voxel_plane_lookup_20260928/product --output-on-failure
timeout 240 cmake --build /ros2_ws/build/gng_cpu -j2
timeout 60 ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure
timeout 60 cmake --install /ros2_ws/build/gng_cpu
```

製品構成CTest 23/23成功。Release、外部サンプラーOFF、フレームログOFF。
単点追加、平面説明による抑制、3入力での削除、再観測・OFF・ID再利用時のリセットを検証。
公開APIの学習0回でも観測済み孤立ノードを維持し、消えた入力のノードだけ寿命で削除。
初回のAPI試験はテスト点が既定入力範囲外で失敗。テスト座標を範囲内へ修正して再成功。
入力範囲の本体条件を緩和する変更なし。

Macnica交差点の保存済み先頭60入力を使用。各条件3試行、先頭20入力を集計から除外。
入力上限20万点、ノード上限2万、voxel 0.5 m、学習4,000回、CPU 4固定。
両条件とも実測代表点。旧重心方式との比較ではない。
実GNG・実平面クラスタリングの直接呼出し。ROS側のコピー・履歴失効・配信は測定対象外。
本番の重点サンプラー設定を再現した比較ではない。既存ROS処理とのCPU競合あり。

| 条件 | 平均GNG [ms/入力] | 試行別p95の平均 [ms] | 平均ノード数 |
| --- | ---: | ---: | ---: |
| 通常 | 42.823 | 45.169 | 18,950 |
| 占有追加・寿命 | 47.971 | 50.635 | 19,228 |

差は+5.148 ms、約12%。グラフと追加削除数も変わるため寿命判定単独の費用ではない。
有効時は入力段階の追加1,001.77、未観測加算1,857.43、早期削除233.85個/入力。
実シーンの車体被覆・追従誤差・遮蔽耐性は未検証。
6/6試行成功、予測24秒・実測20.66秒。全試行cleanup成功、ビルド・CTest・runnerとも終了。
結果は`artifacts/voxel_plane_lookup_20260928/occupancy_verified_batch`。
ビルド・試験ログは同ディレクトリ親の`occupancy_verified_*.log`。

起動コマンド（既存artifactの構成・入力データが前提、コンテナ内`/ros2_ws/src`）：

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
timeout 240 cmake --build artifacts/voxel_plane_lookup_20260928/product -j4
LD_LIBRARY_PATH=/ros2_ws/src/artifacts/voxel_plane_lookup_20260928/product:$LD_LIBRARY_PATH timeout 120 ctest --test-dir artifacts/voxel_plane_lookup_20260928/product --output-on-failure
timeout 240 cmake --build artifacts/voxel_plane_lookup_20260928/build -j4
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py benchmarks/voxel_plane_lookup_20260928/occupancy_cases.json --output artifacts/voxel_plane_lookup_20260928/occupancy_verified_batch --repeats 3 --timeout-sec 120 --max-total-sec 720 --estimate-sec 4
```

上記は`docker exec gng_cpu_container bash -lc '…'`で実行済み。再実行時は別の結果保存先を指定。
既存bag・Viewer・TF・GNGは停止していない。新規ROSノード・デーモンの起動なし。
初回確認時のGNG PID 628313は旧版だったが、後続確認では18:51起動のPID 664386が新パラメータ・ライブラリを保持。
前回の「未適用」の判断は初回確認時限定。現行のノード追加削除による不安定化は未切り分け。
並行した本検証のCPU負荷もあり、実稼働の跳ねの原因は未確定。

同日以前の比較39.646→35.966 msは、通常追加を補助枠で置換した旧版の結果。
平均ノード数も19,120→14,242へ減少しており、現行版の性能値として利用不可。
旧版のROS実行argv・結果は`artifacts/node_insertion_20260928/ros_batch`を参照。
通常追加復帰版の検証ログは`artifacts/voxel_plane_lookup_20260928/insertion_restore_*`。
