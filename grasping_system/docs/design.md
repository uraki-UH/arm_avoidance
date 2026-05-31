# Grasping System Design

## Purpose

This package manages grasp-related state for objects held by the robot.

It is not a motion planner.
It is not a GNG/VLUT safety package.

Instead, it stores and derives the information needed so that a planner can later reason about a grasped payload.

## Core concepts

### 1. Grasp object

A grasp object stores:

- object id,
- shape category,
- object pose in a world or scene frame,
- and optional resources such as mesh paths.

It also records how that object is represented:

- graph,
- voxel,
- or primitive.

### 2. Grasp attachment

When grasped, the robot hand and object are related by a fixed transform:

- `T_world_eef`
- `T_eef_object`
- `T_world_object = T_world_eef * T_eef_object`

This relation is the key state that must be preserved while planning.

### 3. Grasp candidate

A candidate is one possible hand pose for a given object.

It may store:

- grasp pose,
- pre-grasp pose,
- approach direction,
- score,
- clearance,
- and future evaluation metadata.

### 4. Grasp-object VLUT

The planned target is a lookup table like the robot VLUT:

- GNG joint-space node id
- corresponding hand pose
- corresponding object pose
- and object-specific collision/clearance information

This lets a planner bias or reject joint-space nodes based on the attached object.

## Object representation layer

The object model layer should stay independent from the underlying shape encoding.

For now, the two most important encodings are:

- graph structure
- voxel grid

Primitive shape support can be added later by implementing the same geometry interface.

## Indexing policy

The current policy is to keep voxel indexing settings in YAML, not in the saved LUT files.

That means:

- YAML is the source of truth,
- LUT data is regenerated or loaded under the same YAML contract,
- and the shared indexing header is only for code-level consistency.

## Rigid-object first

The initial implementation should start with rigid objects only.

Later extensions can add:

- soft objects,
- articulated objects,
- deformable grasp-state models.
