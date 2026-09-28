"""Validated zero-copy parser for /ais_gng/fvg_frame AISFVG3 snapshots.

CDR1 envelope may be BE or LE. The version3 snapshot payload is always LE.
flags=3 is the legacy exact ROI/zero exclusion. flags=2 is full-range finite,
nonzero input: all six unused bounds must be zero. Consumer configuration must
match, so clipped snapshots can never silently feed a full-range observer.
"""
from dataclasses import dataclass
import struct
import numpy as np

MAGIC=b'AISFVG3\0'
HEADER=struct.Struct('<8sqIIIIQII6d')


class SnapshotFormatError(ValueError):
    pass


def _payload(raw):
    try:view=memoryview(raw).cast('B')
    except (TypeError,ValueError) as error:raise SnapshotFormatError('Contiguous CDR buffer required') from error
    if len(view)<16 or bytes(view[:4]) not in (b'\0\1\0\0',b'\0\0\0\0'):
        raise SnapshotFormatError('Expected CDR1 BE/LE with zero options')
    endian='<' if view[1] else '>'
    dimensions,data_offset,length=struct.unpack_from(endian+'III',view,4)
    if dimensions or data_offset:raise SnapshotFormatError('Snapshot requires empty layout and data_offset0')
    end=16+length
    padding=(-end)%4
    if end>len(view) or (len(view)!=end and not (len(view)==end+padding and padding and not any(view[end:]))):
        raise SnapshotFormatError('CDR data length mismatch')
    return view[16:end].toreadonly()


def _header(payload):
    if len(payload)<HEADER.size:raise SnapshotFormatError('Truncated snapshot header')
    magic,stamp,npoints,nnodes,nedges,nframe,sequence,original_count,flags,*bounds=HEADER.unpack_from(payload)
    if magic!=MAGIC:raise SnapshotFormatError('Unsupported snapshot magic/version or payload byte order')
    if flags not in (2,3) or npoints>original_count:raise SnapshotFormatError('Invalid ROI flags or original point count')
    if flags==3 and (not np.isfinite(bounds).all() or any(a>=b for a,b in zip(bounds[:3],bounds[3:]))):
        raise SnapshotFormatError('Invalid ROI bounds')
    if flags==2 and any(value!=0 for value in bounds):
        raise SnapshotFormatError('Full-range snapshot requires zero unused ROI bounds')
    end=HEADER.size+nframe
    if end>len(payload):raise SnapshotFormatError('Truncated frame_id')
    if end+npoints*12+nnodes*16!=len(payload):raise SnapshotFormatError('Snapshot array size mismatch')
    try:frame=bytes(payload[HEADER.size:end]).decode('utf-8')
    except UnicodeDecodeError as error:raise SnapshotFormatError('Invalid frame_id UTF-8') from error
    if '\0' in frame:raise SnapshotFormatError('Embedded frame_id null')
    return stamp,frame,sequence,npoints,nnodes,nedges,end,original_count,flags,tuple(bounds)


def read_snapshot_header(raw):
    payload=_payload(raw)
    stamp,frame,sequence,npoints,nnodes,nedges,offset,original_count,flags,bounds=_header(payload)
    return stamp,frame,sequence,npoints,nnodes,nedges,len(raw)


@dataclass(frozen=True)
class CompactFrame:
    stamp_ns: int
    frame_id: str
    frame_sequence: int
    points: np.ndarray
    nodes: np.ndarray
    ages: np.ndarray
    edges_count: int
    wire_bytes: int
    payload_bytes: int
    original_point_count: int
    roi_flags: int
    roi_bounds: tuple


def parse_snapshot(raw):
    payload=_payload(raw);stamp,frame,sequence,npoints,nnodes,nedges,offset,original_count,flags,bounds=_header(payload)
    points=np.frombuffer(payload,dtype='<f4',count=npoints*3,offset=offset).reshape(npoints,3)
    node_start=offset+npoints*12
    nodes=np.ndarray((nnodes,3),dtype='<f4',buffer=payload,offset=node_start,strides=(16,4))
    ages=np.ndarray((nnodes,),dtype='<u4',buffer=payload,offset=node_start+12 if nnodes else node_start,strides=(16,))
    points.flags.writeable=nodes.flags.writeable=ages.flags.writeable=False
    return CompactFrame(stamp,frame,sequence,points,nodes,ages,nedges,len(raw),len(payload),original_count,flags,bounds)


def validate_engine_roi(snapshot,config):
    """Reject incompatible transport filtering; never silently change semantics."""
    roi_enabled=getattr(config,'roi_enabled',True)
    expected=tuple(config.roi_min)+tuple(config.roi_max) if roi_enabled else (0.,)*6
    expected_flags=3 if roi_enabled else 2
    if snapshot.roi_flags!=expected_flags or snapshot.roi_bounds!=expected or not config.reject_zero_returns:
        raise SnapshotFormatError('Snapshot ROI/zero filtering differs from FVG engine configuration')
