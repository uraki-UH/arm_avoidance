# ToPoFuzzy Viewer Robot Visualization Runbook

This note summarizes the steps that currently work for visualizing the topoarm robot in ToPoFuzzy Viewer.

## Overview

The visualization path is:

1. `gng_vlut_system` publishes the robot description and joint states.
2. `robot_viewer_bridge_node` publishes `/viewer/internal/stream/robot`.
3. `topo_fuzzy_viewer` backend serves WebSocket and mesh files.
4. The frontend connects to the backend and renders the robot.

## Prerequisites

Build both workspaces first.

```bash
cd /home/fuzzrobo/uraki_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gng_vlut_system
```

```bash
cd /home/fuzzrobo/uraki_ws/ToPoFuzzy-Viewer/backend
source /opt/ros/humble/setup.bash
source /home/fuzzrobo/uraki_ws/install/setup.bash
colcon build --symlink-install --packages-select topo_fuzzy_viewer
```

```bash
cd /home/fuzzrobo/uraki_ws/ToPoFuzzy-Viewer/frontend
npm install
npm run build
```

## Start Viewer Backend

Use a free port. `10081` was used during debugging because `9001` and `9002` were already occupied.

```bash
cd /home/fuzzrobo/uraki_ws/ToPoFuzzy-Viewer/backend
source /opt/ros/humble/setup.bash
source /home/fuzzrobo/uraki_ws/install/setup.bash
source /home/fuzzrobo/uraki_ws/ToPoFuzzy-Viewer/backend/install/setup.bash
ROS_LOG_DIR=/tmp/ros_logs ROS_HOME=/tmp/ros_home \
  ros2 launch topo_fuzzy_viewer viewer_stack.launch.py ws_port:=10081
```

Important:

- The backend must source `/home/fuzzrobo/uraki_ws/install/setup.bash`.
- The backend and frontend must use the same port.

## Start Frontend

Set the same port as the backend.

```bash
cd /home/fuzzrobo/uraki_ws/ToPoFuzzy-Viewer/frontend
VITE_VIEWER_WS_PORT=10081 npm run dev -- --host 0.0.0.0
```

Open the shown local URL in a browser and connect the viewer.

## Start Robot Runtime

Use `enable_safety_monitor:=false` while checking visualization only. That avoids stopping the launch when `gng.bin` / `vlut.bin` are missing.

```bash
cd /home/fuzzrobo/uraki_ws
source /opt/ros/humble/setup.bash
source /home/fuzzrobo/uraki_ws/install/setup.bash
ROS_LOG_DIR=/tmp/ros_logs ROS_HOME=/tmp/ros_home \
  ros2 launch gng_vlut_system gng_vlut_runtime.launch.py robot_name:=topoarm enable_safety_monitor:=false
```

## RViz Sanity Check

If Viewer looks wrong, confirm the URDF itself first with RViz.

```bash
cd /home/fuzzrobo/uraki_ws
source /opt/ros/humble/setup.bash
source /home/fuzzrobo/uraki_ws/install/setup.bash
ros2 launch gng_vlut_system visualize_robot_rviz.launch.py robot_name:=topoarm
```

If the robot is visible in RViz, the URDF and joint state pipeline are fine.

## Mesh Path

The topoarm meshes are served from:

```text
package://gng_vlut_system/urdf/topoarm_description/meshes/topoarm
```

The backend resolves that to files under the `gng_vlut_system` package share directory.

## Common Issues

- `meshes ... File not found`
  - Usually means the backend shell did not source `/home/fuzzrobo/uraki_ws/install/setup.bash`.
- Viewer backend says `Failed to bind port`
  - Pick another `ws_port` and set `VITE_VIEWER_WS_PORT` to the same value.
- Robot appears sideways
  - Recheck the viewer scene and the current `RobotRenderer` transform handling.
- Robot flashes or looks gray
  - Refresh the browser after backend/frontend restarts, and confirm the latest frontend build is being served.

## What Worked During Debugging

- Backend on port `10081`
- Frontend with `VITE_VIEWER_WS_PORT=10081`
- Robot runtime with `robot_name:=topoarm`
- Safety monitor disabled while only checking visualization

