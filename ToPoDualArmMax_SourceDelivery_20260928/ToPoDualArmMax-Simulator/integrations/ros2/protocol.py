"""TFV1 read-only viewer wire format and validated ROS CDR decoders.

No ROS import: parsers and encoder can be tested independently. CDR graph
schema is the installed original ais_gng_msgs schema, not the experimental one.
"""
from dataclasses import dataclass
import json
import struct
import time
import numpy as np


class FormatError(ValueError):
    pass


class CDR:
    def __init__(self, raw):
        self.buf = memoryview(raw).cast('B')
        if len(self.buf) < 4 or bytes(self.buf[:4]) not in (b'\0\1\0\0', b'\0\0\0\0'):
            raise FormatError('Expected little/big endian CDR1 with zero options')
        self.endian = '<' if self.buf[1] else '>'
        self.pos = 4

    def align(self, n):
        self.pos += (-(self.pos - 4)) % n

    def read(self, fmt, align):
        self.align(align)
        size = struct.calcsize(fmt)
        if self.pos + size > len(self.buf):
            raise FormatError('Truncated CDR field')
        result = struct.unpack_from(self.endian + fmt, self.buf, self.pos)
        self.pos += size
        return result[0] if len(result) == 1 else result

    def array(self, dtype, count, align=4):
        if count == 0:
            return np.empty(0, self.endian + dtype)
        self.align(align)
        dt = np.dtype(self.endian + dtype)
        end = self.pos + dt.itemsize * count
        if count < 0 or end > len(self.buf):
            raise FormatError('Truncated CDR array')
        result = np.frombuffer(self.buf, dt, count, self.pos)
        self.pos = end
        return result

    def string(self):
        n = self.read('I', 4)
        if not n or self.pos + n > len(self.buf) or self.buf[self.pos+n-1] != 0:
            raise FormatError('Invalid CDR string')
        value = bytes(self.buf[self.pos:self.pos+n-1]).decode('utf-8')
        if '\0' in value:
            raise FormatError('Embedded null in CDR string')
        self.pos += n
        return value

    def header(self):
        sec, ns = self.read('iI', 4)
        return sec * 1_000_000_000 + ns, self.string()

    def done(self):
        # rmw_fastrtps delivers DDS payloads padded to a 4-byte boundary;
        # rclpy.serialize_message returns the unpadded CDR. Accept only the
        # exact 0..3 zero padding bytes, never arbitrary trailing content.
        remaining = len(self.buf)-self.pos
        padding = (-self.pos) % 4
        if remaining and not (remaining == padding and remaining <= 3 and not any(self.buf[self.pos:])):
            raise FormatError('Unexpected trailing CDR bytes/schema mismatch')


@dataclass
class Graph:
    stamp_ns: int
    frame_id: str
    nodes: np.ndarray
    labels: np.ndarray
    edges: np.ndarray
    clusters: list
    node_cluster_ids: np.ndarray
    assigned_points: np.ndarray
    assigned_point_cluster_ids: np.ndarray
    cluster_node_indices: np.ndarray
    cluster_node_offsets: np.ndarray


def parse_graph(raw):
    c = CDR(raw)
    stamp, frame = c.header()
    nn = c.read('I', 4)
    if nn > 1_000_000:
        raise FormatError('Unreasonable graph node count')
    # Each original TopologicalNode has a 40-byte fixed prefix followed by
    # n Point32 records. Walk only the variable lengths, then gather positions,
    # labels and associated points in bulk; no per-node Python tuple/array.
    starts = np.empty(nn, np.intp)
    counts = np.empty(nn, np.intp)
    count_struct = struct.Struct(c.endian+'I')
    for i in range(nn):
        if c.pos+40 > len(c.buf):
            raise FormatError('Truncated topological node prefix')
        starts[i] = c.pos
        n, = count_struct.unpack_from(c.buf, c.pos+36)
        counts[i] = n
        c.pos += 40+n*12
        if c.pos > len(c.buf):
            raise FormatError('Truncated topological node associated points')
    words = np.frombuffer(c.buf, c.endian+'f4', (len(c.buf)-4)//4, 4)
    word_starts = (starts-4)//4
    nodes = words[word_starts[:,None]+np.arange(3)].astype('<f4',copy=False)
    labels = np.frombuffer(c.buf,'u1')[starts+28]
    total_associated = int(counts.sum())
    prefix = np.r_[0,np.cumsum(counts[:-1])] if nn else np.empty(0,np.intp)
    associated_starts = np.repeat(word_starts+10-3*prefix,counts)+3*np.arange(total_associated)
    ap = words[associated_starts[:,None]+np.arange(3)].astype('<f4',copy=False)
    ne = c.read('I', 4)
    edges = c.array('u2', ne, 2).astype('<u4')
    if ne % 2 or (ne and int(edges.max()) >= nn):
        raise FormatError('Odd edge count or invalid graph endpoint')
    nc = c.read('I', 4)
    if nc > 1_000_000:
        raise FormatError('Unreasonable cluster count')
    clusters, members = [], []
    node_ids = np.full(nn, -1, np.int32)
    seen_ids = set()
    for _ in range(nc):
        cid, label = c.read('I', 4), c.read('B', 1)
        if cid > 2**31-1 or cid in seen_ids:
            raise FormatError('Duplicate or non-int32 cluster identifier')
        seen_ids.add(cid)
        pos, scale, quat = c.read('fff', 4), c.read('fff', 4), c.read('dddd', 8)
        age = c.read('i', 4)
        match, velocity, node_age_ave, reliability = c.read('ffff', 4)
        constant, human = c.read('BB', 1)
        count = c.read('I', 4)
        ids = c.array('u2', count, 2).astype('<u4')
        if count and int(ids.max()) >= nn:
            raise FormatError('Invalid cluster node index')
        if not np.isfinite(pos + scale + quat).all():
            raise FormatError('Nonfinite cluster geometry')
        overlap = node_ids[ids] != -1
        node_ids[ids] = np.where(overlap, -2, cid)  # -2 means explicit overlapping membership
        clusters.append(dict(id=cid, label=label, centroid=list(pos), scale=list(scale),
                             quat=list(quat), nodeCount=count, age=age,
                             match=match, velocity=velocity, node_age_ave=node_age_ave,
                             reliability=reliability,
                             constant=bool(constant), human=bool(human)))
        members.append(ids)
    c.done()
    if not np.isfinite(nodes).all():
        raise FormatError('Nonfinite published graph nodes')
    ai = np.repeat(node_ids, counts).astype('<i4')
    if not np.isfinite(ap).all():
        raise FormatError('Nonfinite graph associated points')
    offsets = np.r_[0, np.cumsum([len(a) for a in members])].astype('<u4')
    indices = np.concatenate(members) if members else np.empty(0, '<u4')
    return Graph(stamp, frame, nodes, labels, edges.reshape(-1, 2), clusters,
                 node_ids, ap, ai, indices, offsets)


def marker_records(message):
    """Preserve every rendered identity-pose CUBE_LIST candidate and its RGBA.

    Only actual observer's documented marker schema is accepted. Unsupported
    geometry is rejected explicitly rather than rendered at an invented pose.
    """
    stamps, frames, records = set(), set(), []
    for m in message.markers:
        stamps.add(int(m.header.stamp.sec)*1_000_000_000 + int(m.header.stamp.nanosec))
        frames.add(m.header.frame_id)
        if m.action in (2, 3):
            continue
        if m.action != 0 or m.type != 6:
            raise FormatError('FVG bridge expects ADD/DELETE CUBE_LIST markers')
        q, p = m.pose.orientation, m.pose.position
        if (p.x, p.y, p.z, q.x, q.y, q.z, q.w) != (0., 0., 0., 0., 0., 0., 1.):
            raise FormatError('FVG marker pose is not identity')
        if m.colors and len(m.colors) != len(m.points):
            raise FormatError('FVG color count mismatch')
        sx, sy, sz = m.scale.x, m.scale.y, m.scale.z
        for i, point in enumerate(m.points):
            color = m.colors[i] if m.colors else m.color
            records.append((point.x, point.y, point.z, sx, sy, sz,
                            color.r, color.g, color.b, color.a))
    if len(stamps) != 1 or len(frames) != 1:
        raise FormatError('FVG marker array must have one exact header')
    arr = np.asarray(records, dtype='<f4').reshape(-1, 10)
    if not np.isfinite(arr).all():
        raise FormatError('Nonfinite FVG marker geometry/color')
    return stamps.pop(), frames.pop(), arr


def parse_markers(raw):
    """Zero-object CDR decoder for the exact MarkerArray used by FVG observer."""
    c = CDR(raw)
    count = c.read('I', 4)
    stamps, frames, result = set(), set(), []
    if count > 100_000:
        raise FormatError('Unreasonable marker count')
    for _ in range(count):
        stamp, frame = c.header()
        stamps.add(stamp); frames.add(frame)
        c.string()  # ns
        mid, kind, action = c.read('iii', 4)
        pose = c.read('ddddddd', 8)
        scale = c.read('ddd', 8)
        color = c.read('ffff', 4)
        c.read('iI', 4)  # lifetime
        c.read('B', 1)  # frame_locked
        npnt = c.read('I', 4)
        points = c.array('f8', npnt * 3, 8).reshape(npnt, 3)
        ncolor = c.read('I', 4)
        colors = c.array('f4', ncolor * 4).reshape(ncolor, 4)
        # Installed Humble includes the newer texture/MeshFile fields.
        texture_resource = c.string()
        c.header(); image_format = c.string()
        image_bytes = c.read('I', 4); c.array('u1', image_bytes, 1)
        nuv = c.read('I', 4); c.array('f4', nuv * 2)
        c.string(); mesh_resource = c.string()  # text, mesh resource
        mesh_file = c.string(); mesh_bytes = c.read('I', 4); c.array('u1', mesh_bytes, 1)
        c.read('B', 1)  # embedded materials
        if texture_resource or image_format or image_bytes or nuv or mesh_resource or mesh_file or mesh_bytes:
            raise FormatError('FVG cubes must not include texture/mesh payloads')
        if action in (2, 3):
            continue
        if action != 0 or kind != 6 or pose != (0., 0., 0., 0., 0., 0., 1.):
            raise FormatError('Unexpected FVG marker geometry/action/pose')
        if ncolor not in (0, npnt):
            raise FormatError('FVG point/color count mismatch')
        rows = np.empty((npnt, 10), dtype='<f4')
        rows[:, :3] = points
        rows[:, 3:6] = scale
        rows[:, 6:] = colors if ncolor else color
        result.append(rows)
    c.done()
    if len(stamps) != 1 or len(frames) != 1:
        raise FormatError('FVG marker array must have one exact header')
    records = np.concatenate(result) if result else np.empty((0, 10), '<f4')
    if not np.isfinite(records).all():
        raise FormatError('Nonfinite FVG records')
    return stamps.pop(), frames.pop(), records


def encode_frame(snapshot, graph, fvg, metrics, sequence, fvg_age_ms, join_ms=0.):
    begin = time.perf_counter()
    if snapshot.roi_flags != 2:
        raise FormatError('Viewer refuses spatially cropped input snapshots')
    if snapshot.stamp_ns != graph.stamp_ns or snapshot.frame_id != graph.frame_id:
        raise FormatError('Snapshot/map header mismatch')
    if snapshot.edges_count != len(graph.edges) or not np.array_equal(snapshot.nodes, graph.nodes):
        raise FormatError('Snapshot/map graph content mismatch')
    if not np.isfinite(snapshot.points).all() or (len(snapshot.points) and np.any(np.all(snapshot.points == 0, axis=1))):
        raise FormatError('Compact points must be finite nonzero full-range input')
    if int(metrics['stamp_ns']) != snapshot.stamp_ns or int(metrics['frame_sequence']) != snapshot.frame_sequence:
        raise FormatError('Processing metrics do not match snapshot sequence/stamp')
    kinds = {
        'points': (snapshot.points, 'f32', 3), 'nodes': (graph.nodes, 'f32', 3),
        'nodeLabels': (graph.labels, 'u8', 1), 'nodeClusterIds': (graph.node_cluster_ids, 'i32', 1),
        'edges': (graph.edges, 'u32', 2), 'assignedPoints': (graph.assigned_points, 'f32', 3),
        'assignedPointClusterIds': (graph.assigned_point_cluster_ids, 'i32', 1),
        'clusterNodeIndices': (graph.cluster_node_indices, 'u32', 1),
        'clusterNodeOffsets': (graph.cluster_node_offsets, 'u32', 1),
    }
    kinds.update({'fvg'+k.title(): (fvg[k], 'f32', 10) for k in ('add','delete','memory')})
    dtypes = {'f32':'<f4','u32':'<u4','i32':'<i4','u8':'u1'}
    descriptor, chunks, offset = {}, [], 0
    for name, (arr, kind, components) in kinds.items():
        padding = (-offset) % 4
        if padding:
            chunks.append(bytes(padding)); offset += padding
        data = np.asarray(arr, dtype=dtypes[kind], order='C').tobytes()
        descriptor[name] = dict(offset=offset, type=kind, count=int(np.size(arr)), components=components)
        chunks.append(data); offset += len(data)
    clean_metrics = {k: metrics.get(k) for k in (
        'processing_sum_ms','ais_ms','fvg_ms','pipeline_latency_ms','input_hz',
        'input_interval_ms','input_interval_max100_ms','queue_ms','worker_drops')}
    meta = dict(version=1, sequence=int(sequence), source_sequence=int(snapshot.frame_sequence),
                stamp_ns=str(snapshot.stamp_ns), frame_id=snapshot.frame_id,
                generated_wall_ms=time.time()*1000., bridge_ms=(time.perf_counter()-begin)*1000,
                join_ms=join_ms, input_records=snapshot.original_point_count,
                pointCount=len(snapshot.points), nodeCount=len(graph.nodes), edgeCount=len(graph.edges),
                assignedPointCount=len(graph.assigned_points), clusters=graph.clusters,
                metrics=clean_metrics, metrics_available=True, stale=False, full_range=True,
                fvg_age_ms=fvg_age_ms, fvg_same_frame=True, arrays=descriptor,
                node_ids_persistent=False, point_cluster_membership='raw input unassigned; assignedPoints are node-associated representatives')
    header = json.dumps(meta, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return b'TFV1' + struct.pack('<I', len(header)) + header + bytes((-len(header)) % 4) + b''.join(chunks)


def decode_packet(packet):
    """Validation helper; browser uses the same offsets with TypedArrays."""
    if packet[:4] != b'TFV1':
        raise FormatError('Bad TFV1 magic')
    n, = struct.unpack_from('<I', packet, 4)
    meta = json.loads(packet[8:8+n])
    start = 8 + ((n + 3) // 4)*4
    dtypes = {'f32':'<f4','u32':'<u4','i32':'<i4','u8':'u1'}
    arrays = {}
    for name, desc in meta['arrays'].items():
        arrays[name] = np.frombuffer(packet, dtypes[desc['type']], desc['count'], start+desc['offset']).reshape(-1, desc['components'])
    return meta, arrays
