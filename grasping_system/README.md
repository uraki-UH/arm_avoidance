# grasping_system

`grasping_system` is a separate ROS 2 package for grasp-object state management.

It is intentionally independent from `gng_vlut_system`.

## Goal

The long-term goal is to model:

- the end-effector pose at grasp time,
- the grasped object's shape,
- the pose relation between object and hand,
- and a grasp-object VLUT similar in spirit to the robot GNG/VLUT pipeline.

That later allows planning to consider:

- the robot's joint-space GNG nodes,
- the grasped object's geometry,
- and collision/clearance under the attached object.

## Current scope

This package currently focuses on the rigid-object case and provides:

- shared grasp data types,
- rigid object grasp model types,
- grasp-object VLUT entry types,
- and generic graph/voxel/primitive object representation types.
- and a future builder interface.

## Suggested dependency direction

- `gng_vlut_system` should remain the motion/safety/planning system.
- `grasping_system` should remain the grasp-object representation layer.
- Integration should happen through data exchange, not by merging responsibilities.

## Current indexing policy

For now, voxel indexing is treated as YAML-owned configuration.

- no extra voxel-indexing header is written into saved LUT files,
- both systems are expected to use the same YAML values at build/runtime,
- and the shared `voxel_idx` package exists only to keep the code-side contract aligned.

## Virtual grasp effectivity validation

`gng_vlut_system` provides an integration validator that loads a real GNG and
robot VLUT, adds a virtual box payload through `RigidGraspLifecycleManager`, and
checks one payload-only node/voxel relation across the grasp lifecycle.

```bash
ros2 run gng_vlut_system gng_grasp_vlut_preview \
  --gng /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin \
  --vlut /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin \
  --seed-limit 500 \
  --box-size 0.08 0.08 0.08 \
  --object-offset 0.0 0.0 0.10
```

Success requires the same probe relation to be absent before grasping, present
while grasped, and absent again after release. The command exits nonzero if the
payload does not add geometry or any lifecycle check fails.

This validates the GNG/VLUT overlay contract using production map files.

## Runtime topic control

`topofuzzy_bridge_node` subscribes to the relative `grasp_state` topic. With
the `ToPoDualArm` namespace, activate an 8 cm virtual box payload as follows:

```bash
ros2 topic pub --once /ToPoDualArm/grasp_state \
  gng_control_msgs/msg/GraspState \
  "{state: 1, object_id: virtual_box, eef_link: L_tcp, shape_type: 0, object_pose_in_eef: {position: {x: 0.0, y: 0.0, z: 0.10}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, dimensions: {x: 0.08, y: 0.08, z: 0.08}}"
```

Release the active payload with its object id. An empty `object_id` releases
whichever payload is active.

```bash
ros2 topic pub --once /ToPoDualArm/grasp_state \
  gng_control_msgs/msg/GraspState \
  "{state: 0, object_id: virtual_box}"
```

Successfully applied state changes are published with transient-local QoS:

```bash
ros2 topic echo /ToPoDualArm/grasp_state_applied \
  gng_control_msgs/msg/GraspState \
  --qos-durability transient_local
```

The runtime currently accepts box payloads (`shape_type: 0`). Repeating an
identical grasp message is idempotent and does not rebuild the payload VLUT.

## Gripper grasp-volume graph

`gripper_volume_graph_node` samples a configured graspable volume into the
existing `ais_gng_msgs/msg/TopologicalMap` format. No additional message type
is required.

- `nodes`: volume sample centers
- `edges`: six-neighbor connectivity between samples
- `clusters[0]`: volume center, orientation, dimensions, and member node IDs
- `header.frame_id`: the TCP or tool frame to which the volume is attached

Supported shapes are `box`, `ellipsoid` (or `sphere`), and `cylinder`. The
cluster remains the oriented bounding box used by the current viewer; nodes and
edges preserve the selected non-box shape.

Launch one configurable graph by passing a YAML list. Each element keeps the
gripper name, corresponding TF frame, topic, and geometry together:

```bash
ros2 launch grasping_system gripper_volume_graph.launch.py \
  grippers:='[
    {
      name: main_gripper,
      tf_frame: tool0,
      output_topic: /grip_V_topological_map,
      shape: box,
      dimensions: [0.08, 0.04, 0.10],
      center: [0.0, 0.0, 0.05],
      resolution: 0.01
    }
  ]'
```

For multiple grippers, add entries to the same list. A YAML file can be passed
instead of a long inline argument. The generic launch reads any number of
grippers from it:

```bash
ros2 launch grasping_system gripper_volume_graph.launch.py \
  grippers_file:=/ros2_ws/src/grasping_system/config/ToPoDualArm_gripper_volumes.yaml \
  tf_prefix:=ToPoDualArm
```

This publishes:

- `/ToPoDualArm/L_grip_V_topological_map` in `ToPoDualArm/L_tcp`
- `/ToPoDualArm/R_grip_V_topological_map` in `ToPoDualArm/R_tcp`
- `/ToPoDualArm/L_grip_minV_topological_map` in `ToPoDualArm/L_tcp`
- `/ToPoDualArm/R_grip_minV_topological_map` in `ToPoDualArm/R_tcp`
- `/ToPoDualArm/L_grip_baseV_topological_map` in `ToPoDualArm/L_tcp`
- `/ToPoDualArm/R_grip_baseV_topological_map` in `ToPoDualArm/R_tcp`
- `/ToPoDualArm/L_grip_sweptV_topological_map` in `ToPoDualArm/L_tcp`
- `/ToPoDualArm/R_grip_sweptV_topological_map` in `ToPoDualArm/R_tcp`

`tf_prefix` is optional. It is useful when `robot_state_publisher` prefixes all
frames to isolate multiple robots. Frames that already contain the same prefix
are left unchanged. `gng_viewer_bridge.launch.py` supplies its `robot_name`
automatically.

The ToPoDualArm dimensions provide the search bounds between the fingers at the
URDF prismatic upper limit. The maximum-volume graph retains a grid center only
when rays along both directions of the closing axis hit the corresponding finger
mesh in that fully open pose. Finger and gripper-base mesh occupancy is removed,
so the published graph follows the actual fully open geometry instead of filling
the entire bounding box. Its source-volume cluster is omitted for the same reason.
The two `undersize` graphs start from the same maximum-open volume. A grid center
is retained only when a ray in each direction of the configured closing axis
hits the corresponding closed-pose finger mesh. Mesh occupancy uses a signed
`exclusion_clearance`: positive values shrink the retained void, while negative
values erode mesh occupancy and expand the void. ToPoDualArm uses `-0.005 m`.
This extracts only the free volume bracketed by the closed fingers; exterior
free space is not part of the graph. Its `TopologicalMap` omits the source-volume
cluster so the viewer does not draw the original maximum box around the filtered
nodes. Mesh paths and closed-pose TCP transforms remain robot configuration
rather than a new message contract.
The two `grip_baseV` graphs retain only the gripper-base STL occupancy plus a
`0.005 m` collision margin, sampled at `0.005 m`. They are forbidden-volume
inputs intended for a `required_empty` occupancy constraint, not graspable
volume. The search-bound cluster is omitted.
The two `grip_sweptV` graphs combine the base and both finger meshes across the
open-to-closed stroke. ToPoDualArm fingers are prismatic, so five poses cover
the 37 mm stroke at 10 mm sampling. `swept_meshes` also accepts
`start_orientation_xyzw` and `end_orientation_xyzw`; a rotating finger is
therefore sampled with quaternion interpolation using the same configuration
format. A matcher using this graph exempts the fully open `grip_V` interior:
occupancy there is the prospective object and may meet a closing finger. The
remaining swept occupancy is a forbidden collision region. Treating the whole
finger sweep as empty would reject every valid contact.
The publisher sends the graph exactly once at node startup. Transient-local QoS
keeps that sample available, so a viewer or rosbag recorder started later still
receives the current static graph without application-level retransmission.

## Top-only surface grasp estimation

For a top-only grasp, the surface estimator builds planar-cluster adjacency from
the GNG edges. Each planar cluster is evaluated independently: its XY OBB must
fit the gripper, and its centroid must be at least
`minimum_protrusion_distance` away from every adjacent cluster plane. Adjacent
clusters are not merged into the candidate footprint, so a nearby wall does not
make the OBB oversized. A cluster with no adjacent plane remains eligible. The
output pose always points the TCP local Z axis downward.

```bash
ros2 launch grasping_system top_grasp_surface_estimator.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

The CPU `ais_gng` component generates planar clusters directly in the GNG point
cloud callback. This avoids serializing and subscribing to `/topological_map`
for clustering. The launch starts `plane_cluster_incremental_node` in
markers-only mode by default, so it consumes the direct cluster output without
running the clustering algorithm a second time.

For the GPU backend or the legacy ROS-connected path, disable direct clustering
and clear the marker-only input:

```bash
ros2 launch grasping_system top_grasp_surface_estimator.launch.py \
  plane_clusters_input_topic:=''
```

Pass `start_plane_cluster:=false` when no plane-cluster marker process is needed.

- Candidates: `/top_grasp_pose_cands`
- Footprint fill ratios: `/top_grasp_pose_cand_scores`
- Selection summary: `/top_grasp_pose_cands/summary`
- Markers: `/top_grasp_pose_markers`
