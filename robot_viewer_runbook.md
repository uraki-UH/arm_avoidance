# ToPoFuzzy Viewer Robot Visualization Runbook

This note summarizes the steps for visualizing the topoarm robot and its topological maps (static and dynamic) in ToPoFuzzy Viewer.

## Overview

The visualization path is:

1. `gng_vlut_system` publishes the robot description, joint states, and topological maps.
2. `topofuzzy_bridge_node` publishes graphs to `/topological_map` (dynamic) or `/topological_map_static` (static).
3. `robot_viewer_bridge_node` publishes `/viewer/internal/stream/robot`.
4. `topo_fuzzy_viewer` backend serves WebSocket and mesh files.
5. The frontend renders the robot and switches between `GraphRenderer` and `StaticGraphRenderer` based on the topic.

## Prerequisites

Build both workspaces.

```bash
cd /home/uraki/arm_avoidance
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gng_vlut_system
```

```bash
cd /home/uraki/arm_avoidance/ToPoFuzzy-Viewer/backend
source /opt/ros/humble/setup.bash
source /home/uraki/arm_avoidance/install/setup.bash
colcon build --symlink-install --packages-select topo_fuzzy_viewer
```

```bash
cd /home/uraki/arm_avoidance/ToPoFuzzy-Viewer/frontend
npm install
# npm run build # Optional, usually npm run dev is used
```

## Start Viewer Backend

The backend serves as the gateway between ROS 2 and the browser.

```bash
cd /home/uraki/arm_avoidance/ToPoFuzzy-Viewer/backend
source /opt/ros/humble/setup.bash
source /home/uraki/arm_avoidance/install/setup.bash
source /home/uraki/arm_avoidance/ToPoFuzzy-Viewer/backend/install/setup.bash
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py ws_port:=9001
```

## Start Frontend

```bash
cd /home/uraki/arm_avoidance/ToPoFuzzy-Viewer/frontend
VITE_VIEWER_WS_PORT=9001 npm run dev -- --host 0.0.0.0
```

## Start Robot & Graph Bridge

To visualize the **static** graph (`gng.bin`) with high performance:

```bash
cd /home/uraki/arm_avoidance
source /opt/ros/humble/setup.bash
source /home/uraki/arm_avoidance/install/setup.bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py id:=topoarm topic_name:=/topological_map_static
```

To visualize a **dynamic** graph (e.g. training), use the standard topic:
`topic_name:=/topological_map` (Note: default for gng_viewer_bridge is now static).

## Start TF Publisher (Testing)

For testing robot movement without a real hardware/sim:

```bash
cd /home/uraki/arm_avoidance
python3 test_tf_publisher.py
```

## Common Issues

- **Static graph not following arm**: Ensure the topic name contains "static" or is registered in `static_topics` in the gateway.
- **Mesh not found**: Ensure `/home/uraki/arm_avoidance/install/setup.bash` is sourced in the backend terminal.
- **Performance lag**: Ensure `gng.bin` is being routed to `StaticGraphRenderer` (check browser console for renderer type).

