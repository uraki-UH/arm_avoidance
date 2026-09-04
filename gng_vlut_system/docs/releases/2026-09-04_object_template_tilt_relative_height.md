# 2026-09-04 - 物体テンプレートの傾きと相対高さ照合

## 概要

物体テンプレート照合に、template IDごとのroll/pitch許容角と、対応平面間の相対高さ根拠を追加。

## 変更

- `template_orientation_tolerances`によるtemplate IDごとのroll/pitch許容角
- 対応済み平面ペアの重心差z成分による相対高さscore
- 相対高さ未観測時の不一致回避

## 設定

```yaml
template_sources:
  template_orientation_tolerances:
    boat:
      roll_tolerance_deg: 4.0
      pitch_tolerance_deg: 4.0
```

`-1`は`object_template_matching.yaml`の共通許容角、`0`は当該軸の固定。

## 追加parameter

- `enable_plane_relative_height_evaluation`
- `max_plane_relative_height_dev_full`
- `max_plane_relative_height_dev_partial`
- `plane_relative_height_weight`

候補JSONには`is_plane_relative_height_observed`、`plane_relative_height_pair_num`、
`plane_relative_height_score`を追加。

## 検証

- `colcon build --packages-select gng_vlut_system --symlink-install`
- 既存の`object_template_matching_sources.yaml`によるtemplate解決

## 注意

相対高さは2つ以上の有効な平面対応がある場合だけ評価する。世界座標の絶対zは候補判定に使わない。
