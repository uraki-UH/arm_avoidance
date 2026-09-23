# 2026-09-24 - 本番CPU相当の基数ソート検証

## Summary

本番の全機能を保つ独立コピーでBoostと基数ソートを比較。300フレームの平均はvoxel 0.1 mで108.996→99.970 ms（8.3%短縮）、0.5 mで39.746→35.396 ms（10.9%短縮）。全点・セル割当・学習回数は保持するが、重点サンプリングの実際の選択点とグラフも変化。

## Changed

今回の変更先は`ais_gng_cpu/experimental/gng_radix_trials`と再現資料。本番の空きID管理・重心走査統合をコピーし、入力整列だけをBoost／32bit安定基数ソートで切替。`COLCON_IGNORE`で通常ビルドから除外。

## Added

ソート単体時間、全点のセル対応、実際の学習点、グラフ構造、入力点群の被覆評価。全32bit・同値キー順・全点保存の回帰テスト。

## Fixed

本番の処理変更なし。ソート比較による量子化の変更・入力点削減なし。

## Removed

既存機能・本番設定・過去の最小比較版の削除なし。

## Behavior Impact

重心の最大成分差は初回で0.0038／0.0153 mmだが、学習へ渡る点は4,000点中1,430／2,504点で1 mmを超える差。重点候補の個数・ラベルが同じでも、セル内元点順が変わり、同じ乱数添字から別の点を選択。グラフの座標・接続は両幅とも初回から異なるため、学習結果保持が条件の今回の本番採用は見送り。

## Topics / Params / Messages

本番ROS topic・launch・message・YAML・共有ライブラリの変更なし。独立コピーのCMake設定と比較用APIのみ追加。

## Verification

同じ入力・乱数列・時間刻み、Release・LTO・CPU 0で、各幅3試行。全1,090組・2,180フレームで全点のセル対応・個数・学習4,000回が一致。voxel無効の30フレームは全出力一致。比較用Boost版は前回本番相当の600フレームと一致。ソート単体は58〜75%短縮。

両構成のCTest各15件とAPI各2件、計34件成功。0.2 m被覆率の平均差は0.1 mで+0.361ポイント、0.5 mで+0.066ポイント。品質の一般的な同等性や改善を示すものではない。

既存コンテナ内の起動コマンド:

```bash
bash /ros2_ws/src/benchmarks/gng_radix_production_20260924/build.sh
bash /ros2_ws/src/benchmarks/gng_radix_production_20260924/run.sh
python3 /ros2_ws/src/benchmarks/gng_radix_production_20260924/summarize.py
```

全プロセス終了、一時ビルド削除済み。本番53ファイルとインストール済みライブラリのSHA-256不変。既存ROS関連9プロセスは前後不変。通常colcon探索は30パッケージ、`gng_cpu`は1件で検証コピーの混入なし。

## Risk / Notes

単一bag・同一乱数列の比較。グラフが変わるため、全体時間には後段の仕事量差も含む。全点・同じセル割当でのソート単体時間を併記。ROS配送、平面／曲面フィッティング、分類器、Viewer描画は未計測。

[実装](../../../ais_gng_cpu/experimental/gng_radix_trials/README.md)、[全結果・条件・再現手順](../../../benchmarks/gng_radix_production_20260924/README.md)。
