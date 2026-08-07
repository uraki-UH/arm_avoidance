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
