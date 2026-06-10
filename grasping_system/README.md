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
