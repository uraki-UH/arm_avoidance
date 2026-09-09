# 2026-09-09 - Surfelと局所Quadricの後処理検証プロトタイプ

## Summary

GNGの基幹アルゴリズムやバイナリを変更せず、保存済みGNGテンプレートと元点群から、ノード近傍のSurfel統計と局所Implicit Quadricを計算するC++オフラインツールを追加した。

## Added

- `shape_patch_analyzer`
- ノード最近傍による元点群の再割当て
- 近傍ノードをまとめた局所パッチの平均、共分散固有値、Quadric残差
- 元点群、1/2間引き、1/4間引きの計算時間と記述子安定性の比較
- 円筒・球の解析的点群によるQuadric自己検証

## Behavior Impact

既存のGNG学習、保存形式、ROS topic、matcherの実行時挙動は変更しない。保存済みテンプレートにノード所属点や4次モーメントがないため、現段階では元点群を最近傍ノードへ再割当てする近似評価である。

## Verification

```text
shape_patch_analyzer --self-test
shape_patch_analyzer /datasets/basket_gng_template.json.gz --source-dir /datasets
```

自己検証では円筒・球のQuadric残差が閾値以内であること、実データではfull/stride2/stride4のvalid数、残差、処理時間、固有値変化を出力する。

実測結果（`basket`, 279 nodes / 665 edges / 4200 points）:

- full: Quadric有効 `242/279`、処理時間 `6.583 ms`、残差中央値 `0.000883 m`
- stride2: Quadric有効 `117/279`、処理時間 `3.037 ms`
- stride4: Quadric有効 `16/279`、処理時間 `1.283 ms`
- full対stride2の共分散固有値変化中央値: `0.221`（対数差）
- 2 cm voxel filter後: `238 points`、Quadric有効 `0/279`、処理時間 `0.400 ms`
- 非平面ノード32個は3連結成分にまとまり、`surface_extension` 3件として保存できた
- 3件の内訳は16ノード/357点、10ノード/207点、2ノード/43点。Quadric残差中央値はそれぞれ約4.51 mm、3.23 mm、1.06 mm

円筒・球の自己テスト残差はそれぞれ `7.64e-14 m`、`4.58e-14 m` だった。

`--output /tmp/basket_surface_components.json` を指定すると、`surface_components_prototype_v1`形式でノードID、隣接平面クラスタID、共分散固有値、局所Quadric係数、残差を保存する。

## Risk / Notes

非平面成分はGNG edgeだけで連結し、平面クラスタへの隣接があれば`surface_extension`、隣接がなくてもQuadricが有効なら`curved_component`、それ以外は`unassigned_nonplane`とする簡易定義である。近傍パッチは物体境界の最終分割やテンプレートmatcherへの統合ではない。次段階で実データの結果を見て、必要なら`support_moment`またはQuadric用累積統計をエクスポート形式へ追加する。
2 cm入力では点数不足になるため、現状のGNG入力解像度でQuadricを常用するには、ノード統合またはより広い局所パッチが必要である。
