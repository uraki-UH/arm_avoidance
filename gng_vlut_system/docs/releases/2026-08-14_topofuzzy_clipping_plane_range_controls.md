# 2026-08-14 - TopoFuzzy clipping plane range controls

## 1. 要約

TopoFuzzy ViewerのClipping Planesで、範囲スライダーの変更が描画へ反映されず、実点群の境界外に操作つまみが出る問題を修正した。

- Canvas生成時の固定設定からclipping planeを分離し、plane状態が変わるたびに既存WebGL rendererへ同期するようにした。
- 軸planeを追加するときは、現在の点群境界を初期範囲に使用する。
- 保存済みの範囲が現在の点群境界外にある場合も、スライダー操作可能な範囲へ正規化する。

- Clipping Planesのdual range sliderで範囲指定しても点群やロボット描画が更新されない問題を修正した。
- 小さい点群境界に対して、初期つまみがスライダー表示域外になり操作できない問題を修正した。

## 2. 条件・検証

- clipping planeはCanvasやWebGL contextを再生成せずに反映される。
- 新規planeの初期範囲は点群全体を含むため、追加直後は描画を切り落とさない。

- topic名、ROS parameter、message fieldの変更はない。

- `npm run lint`: Dockerのfrontendコンテナ内で成功。
- `npm run build`: Dockerのfrontendコンテナ内で成功。

**制約**

- 点群を未読込の状態でplaneを追加した場合だけ、従来どおり `-100..100` を初期範囲として使用する。
