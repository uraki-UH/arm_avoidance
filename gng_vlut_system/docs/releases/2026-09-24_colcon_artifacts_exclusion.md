# 2026-09-24 - 検証用コピーによるcbのパッケージ重複を修正

## 1. 要約

前回のGNG検証で保存したソース3コピーがcolconに検出され、通常版と合わせて`gng_cpu`が4件となり、引数なし`cb`がビルド開始前に失敗。検証保存先の除外設定を追加して解消。

`artifacts/COLCON_IGNORE`で保存領域全体を通常colcon探索から除外。`.gitignore`は生成物の除外を維持し、このマーカーのみGit管理対象へ変更。

比較準備スクリプトにも実験ディレクトリの`COLCON_IGNORE`生成を追加。Git管理除外だけではcolcon探索を除外しないことを[再現資料の案内](../../../benchmarks/README.md)へ明記。

`gng_cpu`のパッケージ名重複。直前の検証は個別CMakeビルドであり、通常のワークスペース全体探索に対する除外の確認漏れ。

**削除**

保存済みソース・ライブラリ・検証結果の削除なし。

## 2. 条件・検証

引数なし`cb`で通常パッケージだけをビルド。検証用コピーは従来どおりCMakeのソースを明示して個別ビルド可能。

ROSインターフェース・学習処理・YAMLの変更なし。

修正前ログで通常版と比較用3コピーの重複エラーを確認。修正後の`colcon list`は30パッケージ、`gng_cpu`は通常版1件、`artifacts`由来0件。

実行コマンド:

```bash
docker exec gng_cpu_container bash -lc 'cd /ros2_ws/src; timeout --signal=INT --kill-after=20s 1200 bash -ic cb'
```

実際の`cb`で`Summary: 30 packages finished [55.5s]`を確認。失敗・中断0件。`gng_wasm_core`の符号付き／符号なし比較等のコンパイラ警告は存在し、エラーなし。ビルドプロセスは全終了、既存Viewer・bagの9プロセスはPID・親PID・コマンド一致。

ログ・重複エラー・探索結果・前後プロセス・検証JSONは`artifacts/gng_cb_fix_20260924/`へ保存。

**制約**

今回の確認範囲は通常Releaseビルドとパッケージ探索。ROSノードの新規起動・停止・再起動なし。
