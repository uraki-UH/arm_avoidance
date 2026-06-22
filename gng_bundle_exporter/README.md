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
  --output out/topo_bundle.json \
  --pretty
```

Example config:

```yaml
topics:
  - alias: graph
    topic: /topological_map
    kind: topological_map
    role: graph
    selector: latest
    required: true
  - alias: pointcloud
    topic: /topo_points
    kind: pointcloud2
    role: pointcloud
    selector: all
    sample_every: 1
  - alias: metrics
    topic: /grasp_candidate_metrics
    kind: generic
    role: metrics
    selector: all
```

The exported `graph` bundle keeps GNG core nodes in `nodes`, while goal/manipulability fields are emitted separately in `node_features`.
