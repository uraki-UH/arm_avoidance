# 2026-07-29 - point cloud RGB color space

## 1. 要約

ToPoFuzzy Viewer の RGB 点群を、入力された sRGB に忠実な色で描画するよう修正した。

- RGB 頂点色を vertex shader で sRGB から linear RGB へ変換
- RGB 点群の材質では ACES tone mapping を無効化
- Three.js の出力色空間変換は維持

- 8-bit RGB を linear RGB として扱い、tone mapping を重ねることで中間色が白っぽく表示される問題

**性能**

- CPU 側の色配列を複製しない
- 点群数に比例する追加メモリを確保しない
- 色変換は GPU の vertex shader で実行

## 2. 条件・検証

- RGB mode の点群色のみ変更
- Simple、Height、Distance、Intensity mode の処理は変更なし
- RGB データを持たない点群は従来どおり白色表示
