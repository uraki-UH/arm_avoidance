# 2026-08-03 - ToPoDualArm 10,000-node training controls

## Summary

現行の941元ノード版を維持したまま、同じロボット設定から約10,000元ノード版を
別実験として学習できるlaunch引数を追加した。

## Added

- `max_node_num`: GNGのノード上限を任意に上書きする。
- `max_iterations`: 初期探索の反復数を任意に上書きする。
- `refine_iterations`: 衝突考慮refinementの反復数を任意に上書きする。
- `gng_profile_names`の空指定時は、params YAMLの`gng.profile_names`を維持する。
- GNG容量変更時に`active_indices_`をclearし、初期2ノードの進捗二重計数を防ぐ。
- `GrowingNeuralGas::load()`は保存ノードIDに応じて内部容量を拡張し、10,000超のbinを
  可視化trainerとruntime bridgeの両方で読み込めるようにした。
- load完了後は最大保存ノードIDへ容量を縮め、倍増時の余分なruntime走査を残さない。

## Behavior Impact

追加した3引数を省略した場合はparams YAMLの値をそのまま使用する。
従来のdual launchは`gng_profile_names=left_arm,right_arm`を常に上書きしていたが、
空指定時はYAMLの選択を使うように修正した。
`experiment_id:=ToPoDualArm10000`を指定した場合は既存の`ToPoDualArm3`を上書きしない。

## Recommended Invocation

```bash
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  experiment_id:=ToPoDualArm10000 \
  max_node_num:=11000 max_iterations:=2200000 refine_iterations:=600000
```

## Notes

学習中の一時上限11,000は、TCP範囲と自己衝突による除去後に約10,000有効ノードを
残すための余裕である。最終的な有効ノード数は厳密な10,000を保証しない。
10,000元ノード版の可視化GNGも生成済みである。

## Measured Result

- 有効元ノード: 10,801
- 学習profile: `left_arm`、7 DOF
- 中間自己衝突除去: 2,350ノード
- refinement後のTCP範囲外除去: 199ノード
- 最終自己衝突除去: 0ノード
- `gng.bin`: 5,268,947 bytes
- `vlut.bin`: 45,167,604 bytes
- 可視化GNG: 500ノード、2,332エッジ、空割当0
- `visualization_gng_layer_0.bin`: 69,892 bytes

## Verification

- Humbleコンテナ内で`gng_vlut_system`全体build成功。
- `visualization_gng_trainer`が10,801元ノードを重複・欠落なく再読込検証。
- 隔離ROS domainの`topofuzzy_bridge_node`で元GNG容量11,000、有効ノード10,801、
  可視化GNG 500ノード/2,332エッジを読み込み、可視化topicを実受信。
- テスト後に既存コンテナ、ROS launch、ROS daemonが開始前と同じ状態であることを確認。
