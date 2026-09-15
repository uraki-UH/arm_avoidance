# 2026-09-15 - SpatialTree比較とTmap静的索引

## Summary

通常版とSpatialTree2を実Tmapで比較したうえで、既存の通常版を目標選択へ採用。環境点群の空間ハッシュとGNG学習索引は変更なし。

## Changed

- `topological_map_goal_selector_node`で、map座標系の3D索引を保持。同じ座標の再配信、ラベル・ID・法線・headerの更新、TF移動での再構築なし。
- 到達セルをmap座標系へ逆変換した外接箱で検索し、従来のセル所属式による厳密な絞り込みを実施。
- [現行仕様](../TECHNICAL_SPEC.md#32-topological_map_goal_selectorlaunchpy-の引数)を更新。

## Added

- 通常版`SpatialTree`に静止点の閉区間`query_aabb`を追加。有限値・上下限検査と空部分木の省略。
- 比較用の有限Tmap購読、同一ソースによる両版・両精度ベンチマーク、旧選択結果との回帰検証。

## Fixed

候補・TFの周期更新のたびに、元マップ全ノードを到達座標へ変換してセル表を再構築していた処理。

## Removed

通常の索引有効時の全ノード変換・セル表再構築。互換呼び出しと比較テスト用の従来経路は維持。

## Behavior Impact

同じ到達セル内での距離・姿勢・可操作性による順位付け、衝突ラベルの除外、同点時の配列順、空入力・TF欠損時の失効を維持。近傍N件だけに候補を制限する変更なし。

## Topics / Params / Messages

変更なし。既存launchの次回起動から適用。既存の動作中プロセスの再起動操作なし。新しい常駐ノード・配信トピック・関節指令なし。

## Verification

### 通常版とSpatialTree2

比較対象はワークスペースの`SpatialTree/include`と、`/home/fuzzrobo/SpatialTree2/SpatialTree/include`。外部原本の変更なし。

- `SpatialTree.hpp`の実装差はinclude guardだけ。既定スカラーは通常版double、SpatialTree2 float。SpatialTree2の`NoHysteresis::enabled`はtrue、通常版はfalse。GNG学習側にも小差分があるが、今回の静的検索の対象外。
- 型をdouble/floatで明示した4構成を同一ソース・`-O3 -DNDEBUG`でビルド。静的検索用バイナリは同じ型で両版のSHA-256が完全一致。doubleは`c03327d1e861c1dbfbd9d985abbcd833929b63aa2a264760268c87c8446cf089`、floatは`78b4e36580b290b90aefca04c3feb5b1ce2403104db21839574894f3f71cd51e`。
- `/ToPoDualArm/Tmap_static`の10,801ノードを有限購読。各1,024件の箱検索・8近傍検索を全件走査と照合し、不一致0。型間を含め検索結果のchecksumは同一。
- 各実行で2回準備後10回測定、実行順を変えて各構成3回実行。下表は実行ごとの中央値3個の中央値。箱・近傍時間は1,024検索の合計であり、1検索の値ではない。

| 版・型 | 構築 [ms] | 箱検索 [ms] | 8近傍 [ms] |
| --- | ---: | ---: | ---: |
| 通常版 double | 0.326 | 2.397 | 1.913 |
| SpatialTree2 double | 0.329 | 2.434 | 1.990 |
| 通常版 float | 0.347 | 2.588 | 1.763 |
| SpatialTree2 float | 0.319 | 2.516 | 1.959 |

同じ型では実行ファイルまで一致し、測定差から版の優劣は判断しない。新しい外部依存を増やす理由がなく、TF・セル境界をdoubleで扱う通常版を採用。動的GNGや異なる空間分布に対する比較結果ではない。

### 目標選択全体

実Tmap座標に合成候補・法線・衝突ラベル・可操作性を設定。回転・並進TF、50 mm到達セル、1候補につき上位8ノード。20回準備後200回で、旧処理と索引処理の順序を毎回交代。全選択ID・出力mapの不一致0。

| 候補数 | 従来中央値 [ms] | 索引使用中央値 [ms] |
| ---: | ---: | ---: |
| 1 | 1.257 | 0.333 |
| 20 | 1.351 | 0.485 |
| 100 | 1.496 | 0.890 |

初回索引構築0.583 ms。同一座標の再受信確認は中央値0.031～0.034 msで、上表には含まない。ROS受信・配信、GNG学習、経路計画、描画の時間も対象外。保存入力と結果は`tmp/static_spatial_20260915/points.txt`、`selection_benchmark.txt`。

### 回帰・結合確認

- Docker Releaseビルド成功。CTestの12件成功（0.17秒）。ランダム4,096ノード・80変換・各20候補、セル境界、負座標、重複位置、非有限座標、同一座標の再受信、ラベル/ID変更、座標変更・空mapを確認。
- AddressSanitizer/UndefinedBehaviorSanitizer付きでも12件成功。
- 隔離ROS_DOMAIN_ID=218で実GNG・候補計画launchを起動。候補ID・経路・関節角・候補ロボットデータ、空入力・領域外でのクリア、復帰、指令トピックの発行者なしを確認。画面描画・実機把持は今回未検証。

実行コマンドは以下。比較用バイナリ・コピーしたSpatialTree2・一時ROSログは削除済み。座標と結果ログは再現用として保持。

検証用ノード・購読・ベンチマーク・ビルドはすべて終了し、残留なしを確認。既存の目標選択ノードPID 497597と関連launchは維持。作業中にAISとfrontendの外部再起動を観測したが、本作業からの停止・再起動操作なし。

```bash
docker exec -e ROS_LOG_DIR=/tmp/static_spatial_20260915_logs gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 30s python3 /ros2_ws/src/gng_vlut_system/test/export_static_spatial_fixture.py /ros2_ws/src/tmp/static_spatial_20260915/points.txt'
docker cp /home/fuzzrobo/SpatialTree2/SpatialTree/include gng_cpu_container:/tmp/static_spatial2_20260915_include
docker exec gng_cpu_container bash -lc 'for version in 1 2; do if [ "$version" = 1 ]; then include=/ros2_ws/src/SpatialTree/include; else include=/tmp/static_spatial2_20260915_include; fi; for scalar in double float; do timeout -s INT -k 5s 60s c++ -std=c++17 -O3 -DNDEBUG -DSPATIAL_BENCH_SCALAR=$scalar -I$include /ros2_ws/src/gng_vlut_system/test/benchmark_static_spatial_index.cpp -o /tmp/static_spatial_20260915_${version}_${scalar} || exit; done; done'
docker exec gng_cpu_container bash -lc 'for name in 1_double 2_double 2_float 1_float 1_float 2_float 2_double 1_double 2_double 1_float 1_double 2_float; do printf "%s " "$name"; timeout -s INT -k 5s 60s /tmp/static_spatial_20260915_$name /ros2_ws/src/tmp/static_spatial_20260915/points.txt || exit; done'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 240s cmake --build /ros2_ws/build/gng_vlut_system --target test_grasp_candidate_reachability topological_map_goal_selector_node -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 60s ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure -R "^test_grasp_candidate_reachability$"'
docker exec gng_cpu_container bash -lc 'timeout -s INT -k 5s 120s c++ -std=c++17 -O3 -DNDEBUG -DSPATIAL_BENCH_SELECTION $(sed -n "s/^CXX_INCLUDES = //p" /ros2_ws/build/gng_vlut_system/CMakeFiles/test_grasp_candidate_reachability.dir/flags.make) /ros2_ws/src/gng_vlut_system/test/benchmark_static_spatial_index.cpp -o /tmp/static_spatial_20260915_selection'
docker exec gng_cpu_container bash -o pipefail -lc 'timeout -s INT -k 5s 60s /tmp/static_spatial_20260915_selection /ros2_ws/src/tmp/static_spatial_20260915/points.txt | tee /ros2_ws/src/tmp/static_spatial_20260915/selection_benchmark.txt'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -o pipefail -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py | tee /ros2_ws/src/tmp/static_spatial_20260915/integration.txt'
docker exec gng_cpu_container bash -lc 'timeout -s INT -k 5s 180s c++ -std=c++17 -O1 -g0 -fsanitize=address,undefined -fno-omit-frame-pointer $(sed -n "s/^CXX_INCLUDES = //p" /ros2_ws/build/gng_vlut_system/CMakeFiles/test_grasp_candidate_reachability.dir/flags.make) /ros2_ws/src/gng_vlut_system/test/test_grasp_candidate_reachability.cpp /ros2_ws/build/gng_vlut_system/gtest/libgtest_main.a /ros2_ws/build/gng_vlut_system/gtest/libgtest.a -pthread -o /tmp/static_spatial_20260915_sanitized && timeout -s INT -k 5s 60s /tmp/static_spatial_20260915_sanitized'
```

結合確認がドメイン218で起動した子プロセスのコマンド:

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin -p base_frame:=ToPoDualArm/base_link -r topological_map:=/ToPoDualArm/Tmap_static
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

## Risk / Notes

- 静的Tmapの目標選択だけへの採用。全点群の毎フレーム構築、把持接触探索、GNG学習をSpatialTreeへ置換した結果ではない。
- 空間索引は検索候補の削減用。把持成功・衝突安全・到達性の新しい保証なし。
- マップが大きく変わる場合は再構築費が必要。受信メッセージを後から直接変更する利用は対象外。
