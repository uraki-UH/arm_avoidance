# 2026-08-12 - ToPoDualArm10000 visualization GNG 150 nodes

## Summary

`ToPoDualArm10000`の可視化GNGを500ノードから150ノードへ再学習した。

## Changed

- `vis_gng_L0.bin`を、10,801元ノードから学習した150ノード版へ置き換えた。
- 学習条件は`iterations=200000`、`seed=42`、`max_joint_step=0.05 rad`とした。

## Added

- 従来の500ノード版を`vis_gng_500nodes_L0.bin`として保持した。

## Fixed

- なし。

## Removed

- なし。元の`gng.bin`、`vlut.bin`、500ノード可視化GNGはいずれも保持している。

## Behavior Impact

- `/ToPoDualArm/topological_map_vis_L0`は150ノード、735エッジを配信する。
- 150ノード版は1連結成分、孤立ノード0、空割当0である。
- `vis_gng_L0.bin`は4,104,980 bytesから3,120,702 bytesへ縮小した。

## Topics / Params / Messages

- topic名、parameter、message定義の変更はない。

## Verification

- `visualization_gng_trainer`の保存後再読込検証に成功した。
- 元ノード10,801件が150可視化ノードへ重複・欠落なく割り当てられた。
- 隔離ROS domainで`gng_viewer_bridge.launch.py`を起動し、150ノード、735エッジのロードを確認した。
- `/ToPoDualArm/topological_map_vis_L0`のpublisher数が1であることを確認した。

## Risk / Notes

- 500ノード版へ戻す場合は、現行ファイルと`vis_gng_500nodes_L0.bin`の名前を入れ替える。
