# 2026-08-12 - grip_V fully-open geometry

## Summary

`L_grip_V_topological_map`と`R_grip_V_topological_map`を固定box全体から、
グリッパーを最大まで開いた姿勢の実メッシュに挟まれる自由空間へ変更した。

## Changed

- URDFのprismatic上限`0.037 m`を適用し、指メッシュをTCP基準の`y=+0.05536 m`と
  `y=-0.05536 m`へ配置する。
- 外接探索box内で、閉方向の正負rayが対応する左右指STLへ交差する格子だけを採用する。
- 全開時の左右指およびグリッパ基部STLが占有する格子を除外する。
- 実形状ではない外接boxを表示しないよう、最大領域topicの`clusters`を空にする。

## Behavior Impact

- 最大領域graphは504 nodes / 1321 edgesから248 nodes / 585 edgesへ変わる。
- topic名、message型、TCP frame、格子解像度は変更しない。
- `undersize` graphの生成条件と130 nodes / 279 edgesは変更しない。

## Topics / Params / Messages

- `/ToPoDualArm/L_grip_V_topological_map`
- `/ToPoDualArm/R_grip_V_topological_map`
- message: `ais_gng_msgs/msg/TopologicalMap`（変更なし）
- frame: `ToPoDualArm/L_tcp`、`ToPoDualArm/R_tcp`（変更なし）

## Verification

- ToPoDualArm設定YAMLのparseに成功した。
- `ROS_DOMAIN_ID=134`で`gng_viewer_bridge.launch.py`を統合起動した。
- 左右最大領域が各248 nodes / 585 edges、`internal_only=true`、`clusters=0`であることを確認した。
- 左右`undersize`領域が各130 nodes / 279 edgesのままであることを確認した。
- 検証用launchを停止し、検証由来のROS processが残っていないことと、既存Docker containerが
  維持されていることを確認した。

## Risk / Notes

- 指メッシュ位置は現在のURDF joint origin、mimic、upper limitから求めた設定値である。
  URDFのグリッパー寸法や関節上限を変更した場合は、このTCP基準位置も同期して更新する。
