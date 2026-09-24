# 非平面重点サンプリングの検証（2026-09-25）

## 要約

5ノードの成分採用・4ノードと単独成分の除外、平面除外、不正ノード、枠の混合・失効を単体検証。
CPUの旧unknown枠を無効にした際の全体学習、入力置換・実行後の既定復帰を検証。
非平面ONでも総学習回数を固定し、重点分を独立観測の統計から除外。

ROS側22テスト成功。初期実装時にはCPU側2対象と索引試験の従来56入力組合せも成功。
同日、最大広がり軸での分割・包囲箱の枝刈り・末端16点の連続走査へ変更。抽出点・重みは維持。
実bagから12件を読み取り、対象となる前回成分の座標からSciPyで独立に候補を計算し、現在入力とのXYZ・点数・順序一致を確認。
変更された通常ビルド先でも再検証し、試験前後のSHA-256一致を確認。
現行設定と利用上の制約：[リリースノート](../../gng_vlut_system/docs/releases/2026-09-25_nonplane_attention.md)。

## 条件・検証

- bag：`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の`/lidar_points`。
- 入力100,000点へuniform選択、voxel 0.5 m、学習4,000回、上限20,000ノード。人車分類は無効化。
- `at128.yaml`と共有平面設定を使用。試験では待ち時間の影響を分離するため有効期間5秒、local座標を使用。
- 採用5点・半径0.3 m・重点比率0.5。8件の通常入力後に時刻逆行・10秒程度の時刻差・frame_id変更を挿入。
- 成分ノード数の採用設定65,534による候補なし、成分出力OFF、候補出力OFF、機能OFF、平面OFFも実行。
- 機能OFF・平面OFFでデバッグPublisherなし。成分出力OFFでも内部抽出による重点候補あり。
- `smoke.py`は試験ノードを専用プロセスグループで起動し、全終了経路でSIGINT→TERM→KILLの上限付き後片付け。

コンテナ内`/ros2_ws/src`、`source /ros2_ws/install/setup.bash`後の起動コマンド：

```bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 20 180 \
  python3 benchmarks/nonplane_attention_20260925/smoke.py \
  --executable /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output artifacts/nonplane_attention_20260925/optimized_installed
```

追加選択処理の比較（初回2回除外、各7回中央値。既存ROS稼働中、CPU 0固定）：

```bash
g++ -std=c++17 -O3 -Iais_gng_cpu/src/ais_gng/include \
  benchmarks/nonplane_attention_20260925/selection.cpp \
  -o artifacts/nonplane_attention_20260925/selection_compare
trial_dir=artifacts/nonplane_attention_20260925
for iter in 1 2 3; do
  timeout 30 taskset -c 0 "$trial_dir/selection_compare" "$trial_dir/selection.bin" "$trial_dir/reference.bin" reference
  timeout 30 taskset -c 0 "$trial_dir/selection_compare" "$trial_dir/selection.bin" "$trial_dir/optimized.bin"
  cmp "$trial_dir/reference.bin" "$trial_dir/optimized.bin" || exit 1
done
```

100,000入力点・6,654対象点・17,764候補点で3試行中央値20.219→11.977 ms（約41%減）。
追加3フレーム：対象11,800／6,605／6,932点、候補63,511／21,761／19,019点。
各7回中央値37.839→19.610／20.178→12.809／20.398→12.423 ms。候補重み・添字・混合重みはバイト一致。
`selection_reference.hpp`は最適化前の比較専用。初期実装時のCPU非固定19.479 msとは条件が異なる。
新規再生の保存先は`optimized_installed/selection_01.bin`・`selection_04.bin`・`selection.bin`。上記入力を差替えて再現可能。
対象はkd-tree構築・検索・重み混合。成分抽出・GNG学習・ROS配信は含まず、追加コストゼロとの主張なし。
同じ点群でもGNGの乱数で対象ノードが変わるため、再試験の候補数・時間は一致を保証しない。

生成物・設定・ログはGit対象外の`artifacts/nonplane_attention_20260925/`。
単体検証は分離Releaseビルドでも実施。ROSラッパーは共有ライブラリを動的読込みするため統合検証は通常配布先が対象。
通常ビルド確認は`cmake --build /ros2_ws/build/ais_gng --target ais_gng_component_cpu ais_gng_cpu test_nonplane_attention test_boundary_attention test_grasp_attention test_nonplane_component_extractor -j2`（timeout 240秒）。
`test_optimized_installed.log`・`smoke_optimized_installed.log`・`optimized_installed/smoke_results.json`に最新結果。
性能は`selection_compare.jsonl`・`selection_compare_frames.jsonl`。以前の`test_gng.log`等も保持。
初回API公開漏れ・旧ライブラリ参照・テスト警戒距離不足・ROS未到着のログも`*_initial.log`へ保存。
SciPyはNumPy互換警告を出力。ROS候補の独立照合は成功、実画像の品質・人車認識率は未評価。
高速化時の初回試験は再送による二重学習で不一致（`smoke_optimized.log`）。接続待ちへ変更後6条件成功。
全ビルド・試験・計測プロセスは終了。今回開始時の既存GNG 243106、平面243110、bag 243090、gateway 202494の停止操作なし。
作業中に別セッションの通常ビルドとGNG launchのPID変更（246464）を確認。こちらからの既存ノード停止・再起動なし。
