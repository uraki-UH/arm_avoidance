# AGENTS.md

## Scope
This guide is specific to the current ToPoFuzzyViewer re-architecture (WS Protocol v2 + backend node split).

## Non-negotiable Rules
- Do not modify any file under `backend/src/ais_gng/**`.
- Frontend must not perform point-level delete/filter/transform computation.
- Frontend is responsible for display and user input only.
- Delete regions must be sent as AABB coordinates in `map` frame.
- If TF transform to `map` is unavailable, backend must fail the edit job (no fallback path).
- WebSocket protocol is v2 only. Legacy RPC methods are intentionally unsupported.

## Backend Architecture Rules
- Use split ROS2 nodes:
  - `viewer_ws_gateway_node`
  - `viewer_source_node`
  - `viewer_edit_node`
  - `viewer_file_node`
  - `viewer_rosbag_node`
  - `viewer_gng_node`
  - `viewer_param_node`
- WebSocket gateway delegates processing to backend nodes and only handles client transport/protocol.
- Heavy edit processing runs asynchronously and publishes progress/completion events.

## Protocol Rules
- Request shape:
  - `{ "id": "uuid", "method": "xxx.yyy", "params": { ... } }`
- Response shape:
  - Success: `{ "id": "uuid", "ok": true, "result": { ... } }`
  - Error: `{ "id": "uuid", "ok": false, "error": { "code": "ERR_CODE", "message": "...", "details": {} } }`
- Required pushed events:
  - `stream.pointcloud.meta`
  - `stream.graph`
  - `job.progress`
  - `job.completed`
  - `job.failed`

## Verification Gates (Required)
Run and pass all of the following before merge:
1. Backend build:
   - `cd backend && source /opt/ros/humble/setup.bash && colcon build --packages-select topo_fuzzy_viewer --symlink-install`
2. Frontend lint:
   - `cd frontend && npm run lint`
3. Frontend build:
   - `cd frontend && npm run build`

## Documentation Requirements
- Keep `doc/BACKEND_API.md` aligned with WS v2 methods.
- Mark `common/rpc_protocol.md` as deprecated.
- Maintain the current protocol in `common/ws_protocol_v2.md`.