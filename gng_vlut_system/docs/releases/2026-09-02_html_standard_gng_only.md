# HTML標準GNG専用化

## 変更内容

- HTMLの未到達な近傍グリッド探索、距離によるedge制限、密度制御分岐を削除
- GNG-Tの局所第三ノード探索と三角形face生成を削除
- `gng_wasm_core`から短距離edge削除の`check_edge_distance()`呼出しを削除
- 内蔵`gng_wasm_core.js`を再生成

## 挙動

HTMLのGNG学習は、基幹CUGNGの第1・第2勝者接続、edge年齢処理、孤立node削除だけを使用する。HTML側およびWASMアダプタ側で、距離・近傍・三角形補助によるedgeの追加、置換、切断は行わない。

`GNG再生`を開始するまで、初期化、把持推定、旧実行経路からGNG学習を開始しない。

## 検証

隔離ブラウザで内蔵WASMを実行し、465点を12,000反復学習して76 nodes、98 edges、0 facesを確認した。
