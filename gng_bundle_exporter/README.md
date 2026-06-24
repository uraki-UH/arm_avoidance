# gng_bundle_exporter

Standalone ROS bag to JSON exporter for `ToPo-FUZZY_Manipulation_v1.html`.

## Usage

List topics:

```bash
ros2 run gng_bundle_exporter gng-bundle-export list --bag /path/to/bag
```

Export selected topics:

```bash
ros2 run gng_bundle_exporter gng-bundle-export export \
  --bag /path/to/bag \
  --config config/export_topics.yaml \
  --pretty
```

If `--output` is omitted, the exporter writes into
`gng_bundle_exporter/results/` with a timestamped filename. Existing files are
never overwritten; name collisions get a new suffix.

Use `--gzip` or a `.gz` output path for compressed export. The HTML viewer can
open both plain JSON and gzip-compressed JSON bundles.
The current bundle format is row-based: each topic has a `schema` and `rows`.

Example config:

```yaml
topics:
  - alias: graph
    topic: /topological_map
    kind: topological_map
    role: graph
    required: true
  - alias: candidate_graph
    topic: /ToPoDualArm/candidate_topological_map
    kind: topological_map
    role: candidate_graph
    compact: true
  - alias: pointcloud
    topic: /topo_points
    kind: pointcloud2
    role: pointcloud
    sample_every: 1
  - alias: metrics
    topic: /grasp_candidate_metrics
    kind: generic
    role: metrics
```

The exported `graph` bundle keeps GNG core nodes in `nodes`, while goal/manipulability fields are emitted separately in `node_features`.
Set `compact: true` on a topological map topic to drop per-node `joint_positions` and keep the export smaller.
