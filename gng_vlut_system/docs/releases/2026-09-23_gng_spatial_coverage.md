# 2026-09-23 - CPU GNGノード生成順の空間偏り修正

## 1. 要約

AT128点群に対してGNGが低いZ側・一部の距離範囲に偏る現象を単体再現し、ノード生成候補の処理順を修正。

VoxelGridの番号順は主にZ方向の昇順。getDownSamplingがその順にadd_nodeを実行し、先頭側だけでnode.num_maxを消費。後続領域は追加不能。学習回数を増やすだけでは解消しない経路。

全候補の添字をフレーム番号を種にしてshuffleし、一度ずつ処理。元の点配列・ラベル添字・voxel対応は維持。

## 2. 条件・検証

点の除外・入力範囲変更・ノード上限増加なし。YAMLの範囲内での生成順の偏りを抑制。GPU版は変更なし。学習結果・ノードID・エッジ構成には変化あり。全域被覆や密度の保証なし。

追加・変更なし。平面・曲面OFFを維持。

- 二層・8192点、上限512ノード、学習回数0の回帰テスト。旧版は下層512・上層0で失敗、修正版は下層244・上層268で成功。
- CTest 5件、学習イベントAPI、差分APIテスト成功。Release・FRAME_LOG ON/OFFのビルドと、インストール後の単体比較成功。
- Macnica bag先頭の同一フレームを30回入力。at128.yamlの数値設定をAPIへ適用、TFなし。旧版はX上端44.69 m・Z上端1.46 m・X>50 mのノード0個。修正版はX上端79.24 m・Z上端9.99 m・X>50 mのノード2183個。インストール後の再検証は2174個。通常学習の乱数により結果に差あり。
- 比較入力のYAML範囲内点はX上端80.25 m。元点群の範囲すべてを学習対象とする検証ではない。
- node.gridだけを2.0にした旧版比較でも広域化したが、セル当たり固定容量による密度変化を伴うため、今回のYAML変更なし。
- 実ブラウザ・連続bag全体での描画は未検証。入力voxel_grid_unit=0がCPU既定0.1へ残る既存問題は今回未変更。

[再現スクリプト](../../../benchmarks/gng_coverage_20260923/probe.py)。生ログはGit管理外の`artifacts/gng_coverage_20260923/`にローカル保管。起動コマンド（コンテナ内、すべて終了済み）:

```bash
source /opt/ros/humble/setup.bash
cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu -B /tmp/gng_coverage_build -DGNG_BUILD_BENCHMARKS=ON -DGNG_ENABLE_FRAME_LOG=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/gng_coverage_build -j2
ctest --test-dir /tmp/gng_coverage_build --output-on-failure --timeout 30
/tmp/gng_coverage_build/gng_training_event_api_test
/tmp/gng_coverage_build/gng_map_delta_api_test
/tmp/gng_coverage_build/gng_spatial_coverage_api_test
timeout 45 python3 /ros2_ws/src/benchmarks/gng_coverage_20260923/probe.py 0.5
timeout 45 python3 /ros2_ws/src/benchmarks/gng_coverage_20260923/probe.py 2.0
```

初回比較は同じPythonコードをpython3 -cで起動。第2引数で比較ライブラリのパスを指定可能。旧版比較は旧ライブラリが必要。隔離ビルド・一時ログ・旧版テスト実行ファイルは検証後に削除、再現資料のみ保持。

**制約**

CPU共有ライブラリを同じビルド設定で生成し、インストール先で原子的に置換。既存プロセスの停止・再起動なし。次回launchから適用。約4×候補点数byteの一時添字配列とshuffle処理を追加。性能の厳密な比較は未実施。
