# grasping_system

把持物体の表現・保持状態・候補生成。ロボットの計画／制御はgng_vlut_systemと分離。

## Goal

把持時TCP・物体形状・手との相対姿勢を管理し、保持物体を含む占有判定へ接続。

## Current scope

剛体物体、把持ライフサイクル、物体VLUT、候補生成。実機での把持・持上げ実行は未接続。

## Suggested dependency direction

[設計](docs/design.md)を参照。表現は本パッケージ、計画・制御はgng_vlut_systemの責務。

## Current indexing policy

格子条件はYAMLを正本とし、保存・読込側で一致させること。共通型はvoxel_idx。

## Virtual grasp effectivity validation

```bash
ros2 run gng_vlut_system gng_grasp_vlut_preview --help
```

実GNG/VLUTへの仮想箱の追加・解除で、物体分の占有関係が出現・消失する契約の検証。
実機把持成功の判定ではない。

## Runtime topic control

topofuzzy_bridge_nodeのgrasp_stateへGraspStateを入力。反映結果はgrasp_state_applied。
定義は[GraspState.msg](../MSG/gng_control_msgs/msg/GraspState.msg)を参照。
現行は箱のみ。同一要求の再送は冪等、解除時の空object_idは現在物体を対象。

## Gripper grasp-volume graph

```bash
ros2 launch grasping_system gripper_volume_graph.launch.py \
  grippers_file:=/ros2_ws/src/grasping_system/config/ToPoDualArm_gripper_volumes.yaml \
  tf_prefix:=ToPoDualArm
```

設定は[ToPoDualArm_gripper_volumes.yaml](config/ToPoDualArm_gripper_volumes.yaml)、生成キャッシュは[専用手順](docs/gripper_volume_cache.md)を参照。
出力はTCPに付く静的TopologicalMap、transient-local。
最大開口・最小開口・基部・指の掃引体積は異なる制約。基部はrequired_empty、開口内部は把持対象を許容。
掃引全体を空領域とすると有効な指接触まで棄却するため、対象物の開口内部を除外。

## Top-only surface grasp estimation

```bash
ros2 launch grasping_system top_grasp_pose_candidates.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

設定は[機体YAML](../gng_vlut_system/config/ToPoDualArm.yaml)、処理・制限は[上方候補推定](docs/top_grasp_surface_estimation.md)を参照。
出力は/grasp_pose_cands。Viewerで直接表示可能。
候補生成は上方方式かボクセル方式の一方だけ起動。比較する場合は出力先を分離。
到達範囲・観測形状への適合は、姿勢到達性・指接触・安定把持の保証とは別。
