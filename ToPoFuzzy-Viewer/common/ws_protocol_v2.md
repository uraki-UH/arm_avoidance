# WebSocket Protocol v2

## Transport
- Endpoint: `ws://<host>:9001`
- Binary frames: point cloud payload (`common/protocol.md`)
- Text frames: JSON request/response and asynchronous events

## Request
```json
{ "id": "uuid", "method": "xxx.yyy", "params": { } }
```

## Response
### Success
```json
{ "id": "uuid", "ok": true, "result": { } }
```

### Error
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

## Events

### Point Cloud Metadata
```json
{ "type": "stream.pointcloud.meta", "topic": "/points" }
```

### Graph Stream
```json
{ "type": "stream.graph", "graph": { "timestamp": 0, "nodes": [], "edges": [], "clusters": [] } }
```

### Edit Job Progress
```json
{ "type": "job.progress", "jobId": "job-...", "sessionId": "edit-...", "progress": 35, "stage": "processing" }
```

### Edit Job Completed
```json
{
  "type": "job.completed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "publishedTopic": "/source/edited",
  "pointCount": 1000,
  "durationMs": 250
}
```

### Edit Job Failed
```json
{
  "type": "job.failed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "error": { "code": "TF_UNAVAILABLE", "message": "..." }
}
```

## Method Groups
- `sources.*`
- `files.*`
- `publish.*`
- `rosbag.*`
- `gng.*`
- `params.*`
- `edit.*`

Refer to `doc/BACKEND_API.md` for concrete method parameters and response payloads.
