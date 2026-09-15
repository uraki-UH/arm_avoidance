# 2026-09-15 - 上方把持候補の凸包・回転包含判定

## Summary

平面のPCA軸固定の寸法判定を、2D凸包から矩形開口に収まる向きを探す方式へ置換。

## Changed

- 単独候補と複合候補の両方で、投影外形の向き・中心・XY寸法を評価。
- 平面追加時は両凸包を合算し、全所属点の外形を維持した再評価。
- [現行仕様](../../../grasping_system/docs/top_grasp_surface_estimation.md#33-投影外接矩形とハンドへの包含判定)を更新。

## Added

- 凸包支持点の切替角区間と寸法比の交点による包含判定。固定角度刻み、追加ライブラリ、新規ROSノードなし。
- 回転正方形・非正方形開口・辺間角度でのみ適合する細長い対象・複合候補・入力順と重複点の不変性・退化形状・独立角度走査との回帰テスト。

## Fixed

- 45度の120 mm正方形が、座標軸に沿う約170 mmの外接箱で過大判定される問題。
- 合算後に回転すれば収まる領域が、起点平面の固定軸によって脱落する問題。

## Removed

PCAによる方向決定と、複合候補の起点軸固定。

## Behavior Impact

- 適合する評価角度の中では外接面積の小さい方向を優先。最小面積矩形1つだけで採否を決定せず、開口の縦横寸法を同時に評価。
- 候補の採否・TCPのXY中心とyaw・外接寸法・面積比・並び順が変化する可能性。高さと下向き進入方向は維持。
- 非平面付属ノードは引き続き平面由来TCPでの後段包含判定。付属込みのTCP再フィット、非平面起点の試験抽出、Viewerの表示箱は変更対象外。

## Topics / Params / Messages

変更なし。既存の`grasp_size_x/y`と`enable_plane_combinations`を使用。

## Verification

Docker `gng_cpu_container` の既存Release設定で、次の有限コマンドを実行・終了済み。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 240s cmake --build /ros2_ws/build/grasping_system --target test_top_grasp_surface_estimator top_grasp_surface_estimator_node -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 60s ctest --test-dir /ros2_ws/build/grasping_system --output-on-failure -R "^top_grasp_surface_estimator$"'
docker exec gng_cpu_container bash -lc 'timeout -s INT -k 5s 180s c++ -std=c++17 -O1 -g0 -fsanitize=address,undefined -fno-omit-frame-pointer $(sed -n "s/^CXX_INCLUDES = //p" /ros2_ws/build/grasping_system/CMakeFiles/test_top_grasp_surface_estimator.dir/flags.make) /ros2_ws/src/grasping_system/test/test_top_grasp_surface_estimator.cpp -o /tmp/convex_grasp_20260915_sanitized && timeout -s INT -k 5s 60s /tmp/convex_grasp_20260915_sanitized'
```

- 両ビルド成功、CTest 1/1成功（0.02秒）。AddressSanitizer・UndefinedBehaviorSanitizerも終了コード0。
- 固定seedのランダム入力120件で0.05度刻みの独立走査と照合。採用候補は出力TCP矩形による全平面所属点の包含、開口XY寸法、下向き姿勢を確認。
- 合成負荷測定は32平面・各96点・計3,072ノード、開口150 mm角、組合せ・上方障害物判定OFF。32候補の生成を各回確認。20回準備後の200回で、内点を含む楕円状分布は中央値0.125 ms・95パーセンタイル0.229 ms、全点が凸包頂点の分布は中央値0.620 ms・95パーセンタイル0.871 ms。
- 一時測定プログラムは下記コマンドでコンパイル・起動・終了済み。測定ソース・実行ファイル・sanitizer実行ファイルは削除済み。
- 全検証プロセスの終了・残留なしを確認。ROSノード・サーバーの新規起動、既存プロセスの停止・再起動操作なし。作業中に外部操作によるROSとfrontendの再起動を観測。終了時の推定器PID 419755の実行inodeは今回ビルドしたバイナリと一致。

```bash
docker exec gng_cpu_container bash -lc 'timeout -s INT -k 5s 120s c++ -std=c++17 -O3 -DNDEBUG $(sed -n "s/^CXX_INCLUDES = //p" /ros2_ws/build/grasping_system/CMakeFiles/test_top_grasp_surface_estimator.dir/flags.make) /tmp/convex_grasp_20260915_bench.cpp -o /tmp/convex_grasp_20260915_bench && timeout -s INT -k 5s 30s /tmp/convex_grasp_20260915_bench'
```

## Risk / Notes

- 実入力での候補の妥当性・Viewer描画・実機把持は今回未検証。合成負荷測定は通信・描画・複合候補探索を含むシステム全体の性能値ではなく、旧実装との速度比較でもない。
- 凸包は矩形包含のための外形表現。空隙の占有、接触、把持安定性や衝突回避の保証なし。
- 同じ凸包には同じ方向を選ぶが、境界ノイズで同等方向間の切替が発生する可能性。時間安定化処理の変更なし。
