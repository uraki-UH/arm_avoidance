# 2026-09-09 - 平面クラスタの統合済み全体による併合判定

## Summary

ノードの勝者入力誤差共分散を必須にしない、位置統計による平面クラスタ併合。
既存のGNG接続・厚み・面内広がり比を維持し、統合済み全体への逐次判定に変更。

## Changed

- A+B成立後の次候補について、元のB+CではなくAB+C全体を評価。
- 統合のたびに代表成分の累積統計・平面フィット・所属数を更新。
- ID選択は統合済み成分の所属数が多い側を優先。同数時は古いIDを優先。
- 少数側RMSを位置共分散C・重心muから算出。
  `RMS^2 = n^T C n + (n・(mu-c))^2`。nとcは統合後平面の法線と重心。

## Added

- 直列6パッチの統合後細長さ判定と、静止入力の繰り返し試験。
- 2x2パッチの面状統合、統合済み成分の二重計上防止、ID維持の試験。
- 勝者入力数0・誤差共分散NaNでも成立する併合の試験。
- 隣接対は合格でも全体の厚みが不適合となる3パッチの試験。
- 小面の微小ずれ許容・浮いた小面の誤吸収防止の試験。

## Fixed

元の隣接対だけの合格判定によって、全体では細長さ・厚み条件を満たさない
クラスタまで連鎖統合される問題。

## Removed

併合判定用のメンバー一覧作成と、少数側RMS計算における候補対ごとのノード再走査。
出力用のメンバー一覧は維持。

## Behavior Impact

- 過度に細長い全体への統合を拒否し、複数の平面クラスタとして保持。
- 接続幅などの追加指標なし。既存の法線処理、生成・保持・分割条件は変更なし。
- 誤差共分散による重み付けは未追加。位置統計と勝者入力誤差共分散は別の情報。
- GNGの学習本体に変更なし。

## Topics / Params / Messages

トピック、メッセージ、launch引数、パラメータ名・既定値は変更なし。
`merge_min_planarity`などの既存条件を統合済み全体に適用。

## Verification

Dockerの`gng_cpu_container`内で、対象実装とgtestを一時ディレクトリへ直接ビルド。
既存の共有ビルド・インストール先への書き込みなし。
修正前は新規の細長さ・厚み試験の2件で不合格、修正後は既存9件と新規4件の計13件で合格。

実行コマンド:

```bash
docker exec gng_cpu_container bash -lc '
set -e
test_dir=$(mktemp -d /tmp/plane-merge-test.XXXXXX)
trap '\''rm -rf "$test_dir"'\'' EXIT
includes=(-I/usr/include/eigen3
  -I/ros2_ws/src/ais_gng_cpu/src/ais_gng/include
  -I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs)
for dir in /opt/ros/humble/include/*; do
  if [ -d "$dir" ]; then includes+=("-I$dir"); fi
done
timeout --kill-after=5s 180s g++ -std=c++17 -O2 -Wall -Wextra "${includes[@]}" \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_plane_cluster_incremental.cpp \
  -lgtest_main -lgtest -pthread -o "$test_dir/test_plane_cluster_incremental"
timeout --kill-after=5s 60s "$test_dir/test_plane_cluster_incremental"
'
```

テストプロセスは終了済み、一時ビルドは削除済み。ROSノード・デーモンの新規起動なし。

追加の合成グラフ性能比較は[処理時間の検証記録](2026-09-09_plane_cluster_timing.md)を参照。

## Risk / Notes

- 合成グラフの単体テストによる検証。実点群での精度・性能比較、パッケージ全体のビルドは未実施。
- 稼働中ノードの再起動やインストール更新は未実施。反映にはais_gngの再ビルドと対象ノードの再起動が必要。
- 貪欲な併合のため候補処理順への依存は残存。全クラスタの大域的な最適分割の保証なし。
- 細長さを制限する既存仕様により、本来の細長い平面も分割状態となる可能性。
- 今回の保証対象は併合判定。学習による位置変化や、その後のノード取り込みに対する全体形状の常時制限は対象外。
