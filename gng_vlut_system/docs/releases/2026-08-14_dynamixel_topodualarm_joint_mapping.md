# 2026-08-14 - Dynamixel ToPoDualArm joint mapping

## Summary

Dynamixel ID 1-8、11-18、21、22の現在角をToPoDualArmの実機状態とViewer表示へ反映できるようにした。

## Changed

- Dynamixel ID 1-8を右腕、11-18を左腕、21を首pan、22を首tiltのURDF関節名へ対応させた。
- 左右グリッパーのmimic関節をURDFの`multiplier=-1`に合う逆符号へ変更した。
- ブリッジの現在角を通常の`joint_states`に加えてmux入力へもpublishするようにした。
- Dynamixel handlerの自動探索上限をID 20から22へ広げた。
- 右腕2軸目へ-90度、左腕2軸目へ+90度のゼロ点補正を追加した。
- 首tiltの回転方向を実機に合わせて反転した。
- 新URDFのリンク基準方向に合わせ、右腕1軸目と3-7軸の回転方向を反転した。右腕2軸目は正方向のまま-90度補正を適用する。
- Viewer起動時はmuxの目標関節角を速度補間せず、表示へ直接反映するようにした。

## Added

- `command_output_topic`、`control_claim_topic`、`control_claim_priority`、`joint_offsets_deg`パラメータを追加した。
- Viewerの`joint_state_mux_node`へ入力元を登録する`JointControlClaim`をpublishするようにした。
- 仮想関節ドライバへ`direct_tracking`、Viewer起動へ`direct_joint_tracking`パラメータを追加した。

## Behavior Impact

- `namespace:=/ToPoDualArm`で起動すると、実機角が`/ToPoDualArm/joint_states`へ出力され、Viewerにも反映される。
- `gng_viewer_bridge.launch.py`では`direct_joint_tracking:=true`が既定で、速度制限による追従遅れは発生しない。
- IDが割り当てられていない`waist_joint`は既存の初期値を維持する。
- handlerを起動していない場合、`/dynamixel/state/present`は配信されずViewerは動かない。

## Topics / Params / Messages

- 出力: `joint_states`、`dynamixel_joint_states`
- claim: `control_claims`、優先度100、exclusive
- 角度変換: `(position_deg + joint_offsets_deg) * deg_to_rad * joint_scales`
- `direct_joint_tracking:=false`を指定すると、従来の`max_joint_velocity`による補間へ戻せる。
- 入力メッセージに設定済みIDが1つでもない場合は、従来どおりpublishをスキップする。

## Verification

- ROS 2 Humbleコンテナで`colcon build --packages-select dynamixel_joint_state_bridge --symlink-install`が成功した。
- YAMLをパースし、`joint_ids`、`joint_names`、`joint_scales`、`joint_offsets_deg`が各20要素で対応することを確認した。
- 合成`DynamixelPresent`を入力し、通常出力とmux出力が一致することを確認した。
- 実機入力でID 12の約-90度が`L_joint2`のほぼ0度へ補正され、ID 22の約+8.35度が`neck_tilt_joint`の約-8.35度へ反転された。
- 実機診断時に`/dynamixel/state/present`のpublisherが0であることを確認し、handler起動が必要と判定した。

## Risk / Notes

- 実機ごとのゼロ点と回転方向は`joint_scales`を含む校正が別途必要になる場合がある。
