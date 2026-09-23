# 2026-09-24 - 学習量を維持した最小GNGの高速化

同日の追加判断で、独立実験版の基数ソートを標準ONへ変更し、本番へ結果保持型の効率化を適用。以下は最初の比較時点の記録。[追加判断・検証](2026-09-24_gng_production_efficiency.md)。

## Summary

ノード追加時の先頭走査、座標演算の関数呼出し、入力voxelの整理を比較。独立実験版の標準構成は元のグラフを維持し、連続300フレームで生点群42.6〜44.3%、voxel31.6〜35.8%の短縮。学習は全ケース4,000回。

## Changed

最小空きノードIDの探索開始位置を保持し、削除時に戻す方式を採用。LTOと重心走査統合を標準で有効化。学習係数・入力上限・ボクセル幅・探索精度・ノード／エッジの更新条件は変更なし。

## Added

基準・開始位置管理・最小ヒープ・LTO・重心走査統合・組合せ・基数ソート・CPU向け命令選択の8構成。各3試行、連続入力、CPU 0固定の確認、再現スクリプト、ハッシュによる出力照合。

## Fixed

コピー元の`Vec3f`既定コンストラクタによる未初期化をテストで再現し、今回の全構成でゼロ初期化へ統一。前回保存済みグラフとの6条件×50フレーム一致を確認。

## Removed

既存の本番・実験ソースからの削除なし。学習・入力点・必要な更新処理の削減なし。

## Behavior Impact

標準構成は測定した全フレームで元の座標・法線・ラベル・接続と一致。基数ソートは同じ全点から同じ個数の重心を生成するが、加算順序による丸め差があり、300フレームでは一部の接続構造も変化。比較用オプションとして保持し、標準ではOFF。

## Topics / Params / Messages

ROS topic・launch・message・既存YAML設定の変更なし。新しい切替は独立CMake設定のみ。[実装仕様](../../../ais_gng_cpu/experimental/gng_runtime_trials/README.md)。

## Verification

Release 8構成のCTest計116件成功。最終ソースの組合せ版21件・基数ソート版13件を再確認し、再ビルドした6ライブラリが計測時とバイト一致。

計測136実行・10,800フレームで最近傍検索4,000回、入力点数・候補数・原点選択数が基準版と一致。基数ソートを除く8,850フレームでグラフ全体とノード追加・削除・移動回数が完全一致。

CPU 0固定の3試行でも、生点群＋treeは11.94→6.86 ms、voxel 0.1 m＋treeは27.14→18.60 ms、基数ソートは12.09 ms。異なるCPUへの割当てを固定した場合も改善を確認。

起動コマンド（既存コンテナ内）:

```bash
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/build_trials.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_trials.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/build_extra.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_extra.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_extended.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_pinned.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/verify_final_build.sh
```

個別のコンパイル・測定は有限タイムアウト付き。段階計時の`voxel_profile.cpp`と`voxel_sort_validation.cpp`も既存コンテナでビルド・実行し、全終了。

本番・コピー元・bsp3dの272ファイル、本番ライブラリのハッシュ不変。今回の計測・テストは全終了、一時ビルド領域削除済み。本番への差替えなし。

作業中に既存GNG・デーモンの停止とbag切替が外部で発生。前後の既存ROS関連プロセス数は13→9、退出5件・追加1件。エージェントによる既存プロセスの停止・再起動操作なし。変更された構成を保持し、検証プロセスの残留0件を確認。

## Risk / Notes

最小構成の処理量を維持した実験であり、全点観測寿命・重点サンプリングを含む本番版との機能同一性を示すものではない。単一bag、固定LPF・乱数の比較。ROS配送・Viewer描画は未計測。

既存プロセスの変化はマシン負荷へ影響し得る。異なる測定期の絶対時間を混合集計せず、各3試行とCPU固定でも改善を確認。計測の入力はsqliteから直接取得しており、外部bag再生の切替による入力変更なし。

基数ソートの0.2 m被覆率の差は、この連続入力では約−0.05〜＋0.005ポイント。グラフ完全一致を必要とする用途の置換とは扱わない。[全結果と測定条件](../../../benchmarks/gng_runtime_trials_20260924/README.md)。
