# 2026-08-27 - AiS-GNG winner residual covariance

## Summary

AiS-GNGの共分散楕円を、ノード移動量の推定値ではなく、学習時の第一勝者ノードに対する入力残差から生成する方式へ変更した。

## Changed

- `gng_cpu`は更新前の勝者ID、勝者rank、ノード生成frame、入力残差XYZを固定長配列へ記録する。
- `ais_gng`は`gng_exec()`直後に配列を読み、ノード生成frame単位でWelford逐次共分散を更新する。
- ノード削除後にIDが再利用された場合、以前の生成frameの統計を破棄する。
- `node.covariance_winner_rank_max=2`で第一・第二勝者を等重みで共分散へ含める。

## Added

- C ABI `gng_setTrainingEventCapture(uint8_t)`
- C ABI `gng_setTrainingEventMaxWinnerRank(uint16_t)`
- C ABI `gng_getTrainingEvents(uint32_t*)`
- `GNG_BUILD_BENCHMARKS` CMake optionとイベント転送ベンチマーク

## Removed

- ノード移動量を学習係数で逆算した入力誤差の推定
- 近傍エッジからの初期分散推定
- 前フレームノードのスナップショット

## Behavior Impact

- `node.covariance_enabled=true`では、共分散楕円が設定した勝者rankまでの実入力残差を表す。
- `node.covariance_enabled=false`では、イベント配列の確保と学習中の記録を行わない。
- topic名、ROS message定義、既定値は変更しない。

## Verification

- Docker内のReleaseビルドで`gng_cpu`と`ais_gng_component_cpu`を一時ディレクトリへビルドした。
- 5,000イベント/フレーム、5,000フレームの比較で、固定長バッファへのコア側記録は約`0.002 ms/frame`だった。
- 固定長バッファ方式の共分散更新まで含む時間は、直接集計との差が約5〜10%だった。
- `nm -D`で新しいC ABI 2シンボルの公開を確認した。

## Risk / Notes

- 学習イベント配列は次回の`gng_exec()`まで有効であり、利用側は同じ処理周期内で読み終える必要がある。
- 現在のCPU GNGでは、1フレームの配列容量は初期化時の学習回数と選択した勝者rank数の積に一致する。
- 現在の勝者探索は第二勝者まで対応する。第N勝者では近傍探索の拡張が別途必要になる。
