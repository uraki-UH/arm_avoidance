# 2026-08-03 - visualization goal position and spatial aggregation

## Summary

可視化GNGの所属を手先位置の純粋な最近傍割当にし、候補目標を元座標のまま明示表示する。

## Changed

- 未所属の可視化ノードへ遠い元ノードを強制割当する処理を廃止した。
- 各可視化ノードの中心を、所属する`weight_coords[layer]`の重心へ更新する。
- 欠番を含む元ノードIDの逆引きを最大IDまで保持する。
- source signature schemaを4へ更新した。

## Added

- `ais_gng_msgs/msg/TopologicalNode.is_goal`を追加した。
- bridgeが最新軌道を保持し、静的可視化graph完成後に再配信する処理を追加した。

## Fixed

- 複数の候補目標が1個の集約ノードへ潰れ、紫ノードが元目標からずれる問題を修正した。
- 起動順によって可視化候補軌道が空のままになる問題を修正した。
- 静的graphのbest-wins labelによって候補経路上のdanger/collision表示が消える問題を修正した。

## Removed

- Viewerのedge終端から候補目標を推測する処理を削除した。

## Behavior Impact

候補目標は集約せず、元候補と同じ座標へ表示する。通常の可視化GNGノードは手先位置近傍で
所属を決めた後、その所属点の重心へ表示される。空集約ノードを残さないため、要求値より
可視化ノード数が少なくなる場合がある。

## Topics / Params / Messages

- topic名とparameter名は変更しない。
- `ais_gng_msgs/msg/TopologicalNode`へ`bool is_goal`を追加した。
- 既存の可視化binはsignature schema 4と一致しないため再生成が必要。

## Verification

- `colcon build --packages-select ais_gng_msgs gng_vlut_system topo_fuzzy_viewer --symlink-install`: 成功。
- `visualization_gng_trainer`:
  - ToPoDualArm10000: 10,801元ノード、500可視化ノード、3,251 edge、孤立0。
  - ToPoDualArm3: 941元ノード、475可視化ノード、2,904 edge、孤立0。
- 実行中topic比較: source goal 8、visual goal 8、最大座標差0.0 m。
- Frontend `npm run build`: 成功。
- Frontend変更ファイルの`eslint`: error 0。

## Risk / Notes

`ais_gng_msgs`のmessage定義変更後は、その型を使うpublisher/subscriberを同じbuildへ揃えて
再起動する必要がある。全体lintは既存の`SharedControls.tsx`と`ellipsoid.tsx`の3 errorで失敗する。
