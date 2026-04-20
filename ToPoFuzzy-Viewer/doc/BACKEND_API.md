# ToPoFuzzyViewer Backend API (WebSocket Protocol v2)

This document describes the **v2** WebSocket API exposed by `viewer_ws_gateway_node`.

## Endpoint
- URL: `ws://<host>:9001`
- Transport:
  - Binary: point cloud stream (`common/protocol.md`)
  - Text: JSON request/response + async events

## Request/Response Format

### Request
```json
{ "id": "uuid", "method": "xxx.yyy", "params": { } }
```

### Success Response
```json
{ "id": "uuid", "ok": true, "result": { } }
```

### Error Response
```json
{
  "id": "uuid",
  "ok": false,
  "error": {
    "code": "ERR_CODE",
    "message": "description",
    "details": {}
  }
}
```

## Stream/Event Messages

### `stream.pointcloud.meta`
Published before each binary cloud frame.
```json
{ "type": "stream.pointcloud.meta", "topic": "/points" }
```

### `stream.graph`
```json
{ "type": "stream.graph", "graph": { "timestamp": 0, "nodes": [], "edges": [], "clusters": [] } }
```

### `job.progress`
```json
{ "type": "job.progress", "jobId": "job-...", "sessionId": "edit-...", "progress": 42, "stage": "processing" }
```

### `job.completed`
```json
{
  "type": "job.completed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "publishedTopic": "/points/edited",
  "pointCount": 12345,
  "durationMs": 1200,
  "message": "Edit commit finished"
}
```

### `job.failed`
```json
{
  "type": "job.failed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "error": { "code": "TF_UNAVAILABLE", "message": "..." }
}
```

## Methods

## Sources / Stream Control
- `sources.list`
- `sources.setActive` (`{ sourceId, active }`)

## File
- `files.list`
- `files.load` (`{ path }`)

## Continuous Publish
- `publish.startContinuous` (`{ topic, rateHz }`)
- `publish.stopContinuous`
- `publish.status`

## Rosbag
- `rosbag.list`
- `rosbag.play` (`{ path, remaps, loop }`)
- `rosbag.stop`
- `rosbag.status`

## GNG
- `gng.listConfigs`
- `gng.start` (`{ inputTopic, configFile?, maxNodes?, learningNum?, voxelGridUnit? }`)
- `gng.stop`
- `gng.status`

## Parameters
- `params.get`
- `params.set` (`{ paramName, value }`)

## Edit Session (AABB / map frame)
- `edit.openSession` (`{ sourceTopic, targetFrame="map" }`)
- `edit.addRegion` (`{ sessionId, min:[x,y,z], max:[x,y,z], frameId="map" }`)
- `edit.removeRegion` (`{ sessionId, regionId }`)
- `edit.clearRegions` (`{ sessionId }`)
- `edit.getSession` (`{ sessionId }`)
- `edit.commit` (`{ sessionId, transform:{position,rotation,scale}, outputTopic? }`)
  - returns immediately with `{ jobId }`
  - execution result arrives through `job.*` events
- `edit.cancelSession` (`{ sessionId }`)

## Notes
- Legacy RPC method names are intentionally unsupported in v2.
- Edit regions must be sent in `map` frame.
- If TF to `map` is unavailable, commit fails with `job.failed`.
- Default output topic is `<sourceTopic>/edited`.
